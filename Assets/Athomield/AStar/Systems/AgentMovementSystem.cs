using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Rendering;
using KNN;
using KNN.Jobs;
using Unity.Jobs;
using Unity.Physics;
using UnityEngine;

namespace Athomield.AStar
{
    /// <summary>
    /// 代理移动系统，系统的核心执行层，每帧负责：
    ///
    /// 1. 路径结果同步：
    ///    检查各代理对应的寻路操作 Entity 是否完成，若完成则将路径点复制到代理自身缓冲区
    ///
    /// 2. 目标追踪（ASConstantTargetFollower）：
    ///    检测目标 Entity 位置变化，动态触发重新寻路
    ///
    /// 3. 无避障代理的移动：
    ///    简单沿路径点前进，到达路径点后切换到下一个，并通过物理射线防止穿越障碍物
    ///
    /// 4. RVO 避障代理的移动（核心算法）：
    ///    a. 使用 KD-Tree（KNN）查找每个代理的 K 近邻
    ///    b. 为每个邻居计算 ORCA（最优互惠碰撞避免）速度约束线
    ///    c. 通过线性规划在所有约束下找最优速度
    ///    d. 物理射线检测阻止穿越墙壁
    /// </summary>
    partial struct AgentMovementSystem : ISystem
    {
        /// <summary>
        /// 寻路判定"目标不可达"后的重试冷却（秒）。
        /// 目的：跨岛/被墙封死的目标不会让代理每帧重发请求（每次请求都要跑一遍
        /// 带预算的 A* + 岛屿判定），把"每帧卡"变成"每 0.5 秒试一次"。
        /// 调大更省性能、代理反应更迟钝；调小则相反。
        /// </summary>
        private const float UnreachableRetryCooldownSeconds = 0.5f;

        // 各组件的查找缓存（在 OnCreate 中初始化，每帧在 OnUpdate 中 Update）
        public ComponentLookup<ASPathfindingResult> AllResults;
        public BufferLookup<ASPathBufferComponent> AllPathPoints;
        public ComponentLookup<ASConstantTargetFollower> AllConstantFollowers;

        public ComponentLookup<ASAgent> AllAgents;
        public ComponentLookup<LocalTransform> AllLocalTransforms;
        public ComponentLookup<ASPathFollower> AllPathFollowers;
        public BufferLookup<ASAgentNeighboursBufferComponent> AllAgentsNeighbours;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            AllResults = state.GetComponentLookup<ASPathfindingResult>();
            AllPathPoints = state.GetBufferLookup<ASPathBufferComponent>();
            AllConstantFollowers = state.GetComponentLookup<ASConstantTargetFollower>();

            AllAgents = state.GetComponentLookup<ASAgent>();
            AllLocalTransforms = state.GetComponentLookup<LocalTransform>();
            AllPathFollowers = state.GetComponentLookup<ASPathFollower>();
            AllAgentsNeighbours = state.GetBufferLookup<ASAgentNeighboursBufferComponent>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧更新所有 Lookup（确保访问最新数据）
            AllResults.Update(ref state);
            AllPathPoints.Update(ref state);
            AllConstantFollowers.Update(ref state);
            AllAgents.Update(ref state);
            AllLocalTransforms.Update(ref state);
            AllPathFollowers.Update(ref state);
            AllAgentsNeighbours.Update(ref state);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
            NativeList<Entity> allAgents = new NativeList<Entity>(Allocator.Temp);

            float deltaTime = SystemAPI.Time.DeltaTime;

            // 获取 Unity Physics World 单例（用于射线检测）与 Grid 参数。
            // 用 state.GetEntityQuery 复用系统持有的查询缓存：旧实现每帧 CreateEntityQuery 新建查询
            // （虽然随即 Dispose，但每帧分配纯属浪费），下面避障代理的查询更是每帧 Build 且从不释放。
            EntityQuery singletonQuery = state.GetEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton>());
            var physicsWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            EntityQuery gridQuery = state.GetEntityQuery(ComponentType.ReadOnly<ASGrid>());
            var grid = gridQuery.GetSingleton<ASGrid>();

            // =============================================
            // 阶段 1：路径结果同步 + 目标追踪
            // =============================================
            foreach (var (pathfindingRequester, pathFollower, agentEntity) in SystemAPI
                .Query<RefRW<ASPathfindingRequester>, RefRW<ASPathFollower>>()
                .WithAll<ASPathBufferComponent>().WithEntityAccess())
            {
                allAgents.Add(agentEntity);

                // ----- 1a. 检查寻路操作是否完成，完成则复制路径 -----
                if (pathfindingRequester.ValueRO.RequestEntity != Entity.Null &&
                    SystemAPI.Exists(pathfindingRequester.ValueRO.RequestEntity))
                {
                    ASPathfindingResult reqResult = AllResults[pathfindingRequester.ValueRO.RequestEntity];

                    if (reqResult.FinishedSearch)
                    {
                        // 结果处理：目标不可达（跨岛 / 被墙封死 / 搜索预算耗尽）时进入重试冷却。
                        // 没有这道冷却，跟随目标的代理会每帧重发请求 → 每帧一次全区域 A* 搜索，
                        // 就是"走进封闭区域瞬间掉帧、且持续卡"的直接原因。
                        pathfindingRequester.ValueRW.RetryCooldown = reqResult.Unreachable
                            ? UnreachableRetryCooldownSeconds
                            : 0f;

                        if (reqResult.PathFound)
                        {
                            // 从操作 Entity 的路径缓冲区复制路径点到代理的路径缓冲区
                            DynamicBuffer<ASPathBufferComponent> reqPathPoints = AllPathPoints[pathfindingRequester.ValueRO.RequestEntity];
                            DynamicBuffer<ASPathBufferComponent> agentPathPoints = state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                            pathFollower.ValueRW.PathAvailable = true;
                            pathFollower.ValueRW.TargetIndex = 0;       // 从路径起点开始追踪
                            pathFollower.ValueRW.DestinationReached = false;

                            agentPathPoints.Clear();
                            for (int i = 0; i < reqPathPoints.Length; i++)
                            {
                                agentPathPoints.Add(new ASPathBufferComponent() { Point = reqPathPoints[i].Point });
                            }
                        }
                        else
                        {
                            // 没找到路径（DontPath 下与目标不同岛 / 被墙封死且预算耗尽）：
                            // 必须把代理"停住"并清掉旧路径，否则它会沿着上一次的过期路径继续跑 ——
                            // 这就是"不在同一岛屿的 agent 一直乱跑"的原因。
                            // 新路径到来时 PathAvailable / DestinationReached 会被重置，代理自动恢复移动。
                            DynamicBuffer<ASPathBufferComponent> agentPathPoints = state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);
                            agentPathPoints.Clear();
                            pathFollower.ValueRW.PathAvailable = false;
                            pathFollower.ValueRW.TargetIndex = 0;
                            pathFollower.ValueRW.DestinationReached = true;
                        }

                        // 路径已读取，销毁临时操作 Entity（清理内存）
                        ecb.DestroyEntity(pathfindingRequester.ValueRO.RequestEntity);
                    }
                }

                // ----- 1b. 持续目标追踪：检测目标位置变化，触发重新寻路 -----
                // 冷却期内不重发请求（冷却时间在上一次"不可达"结果中设置）
                float retryCooldown = pathfindingRequester.ValueRO.RetryCooldown;
                if (retryCooldown > 0f)
                {
                    retryCooldown = math.max(0f, retryCooldown - deltaTime);
                    pathfindingRequester.ValueRW.RetryCooldown = retryCooldown;
                }

                if (AllConstantFollowers.HasComponent(agentEntity) && retryCooldown <= 0f)
                {
                    ASConstantTargetFollower constantTargetFollower = AllConstantFollowers[agentEntity];

                    if (constantTargetFollower.Target != Entity.Null &&
                        SystemAPI.Exists(constantTargetFollower.Target))
                    {
                        if (AllLocalTransforms.HasComponent(constantTargetFollower.Target))
                        {
                            if (constantTargetFollower.RecalculateOnStationary)
                            {
                                // 无论目标是否移动都重新寻路
                                pathfindingRequester.ValueRW.RequestGiven = false;
                                pathfindingRequester.ValueRW.Destination = AllLocalTransforms[constantTargetFollower.Target].Position;

                                constantTargetFollower.TargetLastPosition = AllLocalTransforms[constantTargetFollower.Target].Position;
                                AllConstantFollowers[agentEntity] = constantTargetFollower;
                            }
                            else
                            {
                                // 仅在目标移动超过阈值（0.01 单位距离²）时重新寻路
                                if (math.distancesq(AllLocalTransforms[constantTargetFollower.Target].Position,
                                    constantTargetFollower.TargetLastPosition) > 0.01f)
                                {
                                    pathfindingRequester.ValueRW.RequestGiven = false;
                                    pathfindingRequester.ValueRW.Destination = AllLocalTransforms[constantTargetFollower.Target].Position;

                                    constantTargetFollower.TargetLastPosition = AllLocalTransforms[constantTargetFollower.Target].Position;
                                    AllConstantFollowers[agentEntity] = constantTargetFollower;
                                }
                            }
                        }
                    }
                }
            }

            // =============================================
            // 阶段 2：无避障代理的移动（WithAbsent<ASAvoidanceAgent>）
            // =============================================
            foreach (var (agent, pathFollower, agentTransform, pathRequester, agentEntity) in SystemAPI
                .Query<RefRW<ASAgent>, RefRW<ASPathFollower>, RefRW<LocalTransform>, RefRO<ASPathfindingRequester>>()
                .WithAll<ASPathBufferComponent>().WithAbsent<ASAvoidanceAgent>().WithEntityAccess())
            {
                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached) continue;

                // 强制 Y 轴固定在代理中心高度（防止 Y 轴漂移）
                if (math.abs(agentTransform.ValueRO.Position.y - agent.ValueRO.Height / 2) > 0.001f)
                {
                    agentTransform.ValueRW.Position.y = agent.ValueRO.Height / 2;
                }

                DynamicBuffer<ASPathBufferComponent> agentPath = state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                // 当前追踪路径点（Y 轴对齐代理高度）
                float3 currentPoint = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                currentPoint.y = agent.ValueRO.Height / 2;

                // ----- 路径点切换逻辑 -----
                if (pathFollower.ValueRO.TargetIndex != agentPath.Length - 1)
                {
                    // 非最后一个点：使用 NextTargetTouchDistance 阈值
                    float nextTouchDistance = agent.ValueRO.NextTargetTouchDistance < 0
                        ? grid.NodeSize
                        : agent.ValueRO.NextTargetTouchDistance;

                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius + nextTouchDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++; // 切换到下一个路径点
                    }
                }
                else
                {
                    // 最后一个点：使用 StoppingDistance 阈值
                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++;
                    }
                }

                // 索引超出路径长度：到达终点
                if (pathFollower.ValueRO.TargetIndex > agentPath.Length - 1)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                // 检查是否足够接近最终目的地（精确停止）
                float3 finalDestinationPoint = pathRequester.ValueRO.Destination;
                finalDestinationPoint.y = agent.ValueRO.Height / 2;

                if (math.distance(finalDestinationPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                // 计算朝向当前路径点的速度
                float3 destination = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                destination.y = agent.ValueRO.Height / 2;
                float3 destinationDir = math.normalize(destination - agentTransform.ValueRO.Position);
                agent.ValueRW.CurrentVelocity = destinationDir * agent.ValueRO.MaxSpeed;

                // ----- 障碍物射线检测（防止穿墙）-----
                var collFilter = new CollisionFilter()
                {
                    BelongsTo = ~0u,
                    CollidesWith = (uint)grid.InaccessibleLMIndex, // 只检测不可通行层
                    GroupIndex = 0
                };

                bool obsAhead = false;

                // for (int i = 0; i < 1; i++)
                // {
                //     float3 offset = agentTransform.ValueRW.Right() * (agent.ValueRO.Radius * 0.00f);
                //     float3 currentOffset = -offset + (i * offset);
                //
                //     // 从代理底部向前方发射射线，检测是否有障碍物
                //     obsAhead = physicsWorld.CastRay(new RaycastInput()
                //     {
                //         Start = agentTransform.ValueRO.Position - (agentTransform.ValueRO.Up() * agent.ValueRO.Height) + currentOffset,
                //         End = agentTransform.ValueRO.Position - (agentTransform.ValueRO.Up() * agent.ValueRO.Height) + (destinationDir * agent.ValueRO.Radius) + currentOffset,
                //         Filter = collFilter
                //     });
                //
                //     if (obsAhead) break;
                // }

                if (!obsAhead)
                {
                    // 无障碍：更新位置
                    agentTransform.ValueRW.Position += agent.ValueRW.CurrentVelocity * SystemAPI.Time.DeltaTime;

                    // 平滑旋转朝向移动方向
                    quaternion destRot = quaternion.LookRotationSafe(destinationDir, agentTransform.ValueRO.Up());
                    float ang = math.angle(destRot, agentTransform.ValueRW.Rotation);

                    if (math.abs(ang) > 0.01f)
                    {
                        float angStep = math.min(ang, math.radians(SystemAPI.Time.DeltaTime * agent.ValueRO.TurnSpeed));
                        agentTransform.ValueRW.Rotation = math.slerp(agentTransform.ValueRW.Rotation, destRot, angStep / ang);
                    }
                }
            }

            // =============================================
            // 阶段 3：KD-Tree 构建与邻居查询（仅限有避障代理时）
            // =============================================
            int kNeighbours = grid.AvoidanceNeighbours + 1; // +1 是因为 KNN 结果包含自身
            NativeList<float3> avoidanceAgentsPositions = new NativeList<float3>(0, Allocator.TempJob);

            // 避障代理查询：复用系统持有的查询缓存。
            // 旧实现每帧 new EntityQueryBuilder(...).Build(ref state) 且从不释放 —— 每帧都会
            // 新建一个系统持有的 EntityQuery，长时间运行会持续累积（内存 + 查询匹配开销）。
            EntityQuery avoidanceAgents = state.GetEntityQuery(
                ComponentType.ReadOnly<LocalTransform>(), ComponentType.ReadOnly<ASAvoidanceAgent>());

            // 如果没有任何避障代理，跳过 KNN 阶段
            if (avoidanceAgents.IsEmpty)
            {
                avoidanceAgentsPositions.Dispose();
                ecb.Playback(state.EntityManager);
                ecb.Dispose();
                allAgents.Dispose();
                return;
            }

            // 收集所有避障代理的世界坐标
            NativeArray<Entity> avAgEnts = avoidanceAgents.ToEntityArray(Allocator.Temp);
            for (int i = 0; i < avAgEnts.Length; i++)
            {
                avoidanceAgentsPositions.Add(AllLocalTransforms[avAgEnts[i]].Position);
            }

            // 构建 KD-Tree 并执行批量 K 近邻查询
            var results = new NativeArray<int>(avoidanceAgentsPositions.Length * kNeighbours, Allocator.TempJob);
            var knnContainer = new KnnContainer(avoidanceAgentsPositions.AsArray(), false, Allocator.TempJob);

            var rebuildJob = new KnnRebuildJob(knnContainer);
            rebuildJob.Schedule().Complete(); // 构建 KD-Tree

            // 并行查询每个代理的 K 近邻（结果存储在 results 数组中）
            var batchQueryJob = new QueryKNearestBatchJob(knnContainer, avoidanceAgentsPositions.AsArray(), results);
            batchQueryJob.ScheduleBatch(avoidanceAgentsPositions.Length, avoidanceAgentsPositions.Length / 32).Complete();

            // 将 KNN 查询结果写入每个代理的邻居缓冲区（过滤掉自身）
            for (int i = 0; i < avAgEnts.Length; i++)
            {
                Entity e = avAgEnts[i];
                DynamicBuffer<ASAgentNeighboursBufferComponent> neighbours = AllAgentsNeighbours[e];
                neighbours.Clear();

                int n = math.min(kNeighbours, avAgEnts.Length);

                for (int j = 0; j < n; j++)
                {
                    Entity otherEntity = avAgEnts[results[i * kNeighbours + j]];

                    if (e != otherEntity) // 排除自身
                    {
                        // 排除"已到达目的地且配置了IgnoreAvoidanceWhenDestReached"的静止代理
                        if (!(AllPathFollowers[otherEntity].DestinationReached &&
                              AllAgents[otherEntity].IgnoreAvoidanceWhenDestReached))
                        {
                            neighbours.Add(new ASAgentNeighboursBufferComponent() { Agent = avAgEnts[results[i * kNeighbours + j]] });
                        }
                    }
                }
            }

            // =============================================
            // 阶段 4：RVO 避障代理的移动
            // =============================================
            foreach (var (agent, pathFollower, agentAvoidance, agentTransform, pathRequester, agentEntity) in SystemAPI
                .Query<RefRW<ASAgent>, RefRW<ASPathFollower>, RefRW<ASAvoidanceAgent>, RefRW<LocalTransform>, RefRO<ASPathfindingRequester>>()
                .WithAll<ASPathBufferComponent>().WithEntityAccess())
            {
                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached) continue;

                // Y 轴固定
                if (math.abs(agentTransform.ValueRO.Position.y - agent.ValueRO.Height / 2) > 0.001f)
                {
                    agentTransform.ValueRW.Position.y = agent.ValueRO.Height / 2;
                }

                DynamicBuffer<ASAgentNeighboursBufferComponent> agentNeighbours = state.EntityManager.GetBuffer<ASAgentNeighboursBufferComponent>(agentEntity);
                DynamicBuffer<ASPathBufferComponent> agentPath = state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                float3 currentPoint = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                currentPoint.y = agent.ValueRO.Height / 2;

                // ----- 路径点切换逻辑（同无避障版本）-----
                if (pathFollower.ValueRO.TargetIndex != agentPath.Length - 1)
                {
                    float nextTouchDistance = agent.ValueRO.NextTargetTouchDistance < 0
                        ? grid.NodeSize
                        : agent.ValueRO.NextTargetTouchDistance;

                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius + nextTouchDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++;
                    }
                }
                else
                {
                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius)
                    {
                        pathFollower.ValueRW.TargetIndex++;
                    }
                }

                if (pathFollower.ValueRO.TargetIndex > agentPath.Length - 1)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                float3 finalDestinationPoint = pathRequester.ValueRO.Destination;
                finalDestinationPoint.y = agent.ValueRO.Height / 2;

                if (math.distance(finalDestinationPoint, agentTransform.ValueRO.Position) < agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                float3 destination = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                destination.y = agent.ValueRO.Height / 2;

                float2 newVelocity = float2.zero;
                NativeList<ASLine> orcaLines = new NativeList<ASLine>(Allocator.Temp);

                // 理想速度方向（单位向量）：朝向当前路径点
                float2 optVel = PathfindingUtility.V2FromV3(math.normalize((destination - agentTransform.ValueRO.Position)));

                // 执行 ORCA 速度计算
                CalculateFinalVelocity(orcaLines, agent.ValueRW, agentAvoidance.ValueRW, agentTransform.ValueRW,
                    destination, agentNeighbours, SystemAPI.Time.DeltaTime, optVel, ref newVelocity);

                float3 newVel3 = new float3(newVelocity.x, 0, newVelocity.y);
                float3 destinationDir = math.normalize(newVel3);

                // ----- 障碍物射线检测 -----
                var collFilter = new CollisionFilter()
                {
                    BelongsTo = ~0u,
                    CollidesWith = (uint)grid.InaccessibleLMIndex,
                    GroupIndex = 0
                };

                bool obsAhead = false;

                if (math.length(agent.ValueRO.CurrentVelocity) > 0)
                {
                    for (int i = 0; i < 1; i++)
                    {
                        float3 offset = agentTransform.ValueRW.Right() * (agent.ValueRO.Radius * 0.0f);
                        float3 currentOffset = -offset + (i * offset);

                        obsAhead = physicsWorld.CastRay(new RaycastInput()
                        {
                            Start = agentTransform.ValueRO.Position - (agentTransform.ValueRO.Up() * agent.ValueRO.Height) + currentOffset,
                            End = agentTransform.ValueRO.Position - (agentTransform.ValueRO.Up() * agent.ValueRO.Height) +
                                  (math.normalize(agent.ValueRW.CurrentVelocity) * agent.ValueRO.Radius) + currentOffset,
                            Filter = collFilter
                        });

                        if (obsAhead) break;
                    }
                }

                if (obsAhead)
                {
                    newVel3 = float3.zero; // 前方有障碍物，停止移动
                }
                else
                {
                    // 平滑旋转朝向移动方向
                    quaternion destRot = quaternion.LookRotationSafe(destinationDir, agentTransform.ValueRO.Up());
                    float ang = math.angle(destRot, agentTransform.ValueRW.Rotation);

                    if (math.abs(ang) > 0.01f)
                    {
                        float angStep = math.min(ang, math.radians(SystemAPI.Time.DeltaTime * agent.ValueRO.TurnSpeed));
                        agentTransform.ValueRW.Rotation = math.slerp(agentTransform.ValueRW.Rotation, destRot, angStep / ang);
                    }
                }

                // 更新位置和速度
                agentTransform.ValueRW.Position += newVel3 * SystemAPI.Time.DeltaTime;
                agent.ValueRW.CurrentVelocity = newVel3;

                orcaLines.Dispose();
            }

            // 释放所有临时内存
            results.Dispose();
            avoidanceAgentsPositions.Dispose();
            avAgEnts.Dispose();
            knnContainer.Dispose();
            ecb.Playback(state.EntityManager);
            ecb.Dispose();
            allAgents.Dispose();
        }

        /// <summary>
        /// 检测线段是否与圆相交（用于 ORCA 调试，当前未在主流程使用）
        /// </summary>
        bool LineInsideCircle(float _cirlceRadius, float2 _circleCenter, ASLine _line, float _lineLength)
        {
            if (math.distancesq(_circleCenter, _line.Point + _line.Direction * _lineLength) < math.pow(_cirlceRadius, 2))
                return true;

            float radiusSq = math.pow(_cirlceRadius, 2);

            if (math.distancesq(_circleCenter, _line.Point) < radiusSq)
                return true;

            float2 centerToLineStart = _line.Point - _circleCenter;
            float2 directionWLength = _line.Direction * _lineLength;

            float a = math.dot(directionWLength, directionWLength);
            float b = 2 * math.dot(centerToLineStart, directionWLength);
            float c = math.dot(centerToLineStart, centerToLineStart) - radiusSq;

            if (a < float.Epsilon) return false;

            float d = math.pow(b, 2) - 4 * a * c;
            if (d < 0) return false;

            float sqrtD = math.sqrt(d);
            float x1 = (-b - sqrtD) / (2 * a);
            float x2 = (-b + sqrtD) / (2 * a);

            return (x1 <= 1 && x2 >= 0);
        }

        /// <summary>
        /// 将二维向量按指定弧度角旋转（用于 ORCA 切线方向计算）
        /// </summary>
        public float2 RotateFloat2(float2 _vector, float _radAngle)
        {
            return new float2(_vector.x * math.cos(_radAngle) - _vector.y * math.sin(_radAngle),
                _vector.x * math.sin(_radAngle) + _vector.y * math.cos(_radAngle));
        }

        /// <summary>
        /// ORCA（最优互惠碰撞避免）核心算法：为当前代理计算考虑所有邻居后的最优速度。
        ///
        /// 算法原理（基于 RVO2 论文）：
        /// 对每个邻居代理：
        ///   - 计算相对位置向量 relativePosition（其他代理位置 - 当前代理位置）
        ///   - 计算相对速度向量 relativeVelocity（当前代理速度 - 其他代理速度）
        ///   - 情况1：代理未重叠（distSq > combinedRadiusSq）
        ///     - 在时间窗口 timeHorizon 内计算速度障碍（Velocity Obstacle）的截锥
        ///     - 若当前相对速度在锥外：正常计算 ORCA 切线方向和偏移量 u
        ///     - 若当前相对速度在锥内（圆心方向）：沿圆弧法线方向逃逸
        ///   - 情况2：代理已重叠（distSq <= combinedRadiusSq）
        ///     - 使用当前帧时间步长代替 timeHorizon（紧急避让）
        ///     - 立即计算最小分离速度
        ///   - ORCA 线：point = 当前速度 + u/2（互惠各承担一半责任）
        ///              direction = 垂直于 u 的方向（约束线方向）
        ///
        /// 速度优化：
        ///   调用 FindSubOptimalPath 在所有 ORCA 约束线形成的半平面集合中
        ///   用线性规划找最接近理想速度（方向×最大速度）的合法速度
        /// </summary>
        void CalculateFinalVelocity(NativeList<ASLine> _orcaLines, ASAgent _agent, ASAvoidanceAgent _avoidanceAgent,
            LocalTransform _agentLocalTrasnfrom, float3 _currentPathPosition,
            DynamicBuffer<ASAgentNeighboursBufferComponent> _agentNeighbours,
            float deltaTime, float2 _optimalVel, ref float2 _newVelocity)
        {
            int numObstLines = _orcaLines.Length;
            float invTimeHorizon = 1.0f / _avoidanceAgent.TimeHorizon; // 1/timeHorizon

            // 遍历所有邻居，生成对应的 ORCA 约束线
            for (int i = 0; i < _agentNeighbours.Length; ++i)
            {
                Entity otherAgentEntity = _agentNeighbours[i].Agent;

                // 2D 相对位置（XZ 平面）：其他代理 → 当前代理
                float2 relativePosition = PathfindingUtility.V2FromV3(AllLocalTransforms[otherAgentEntity].Position)
                    - PathfindingUtility.V2FromV3(_agentLocalTrasnfrom.Position);

                // 相对速度：当前代理速度 - 其他代理速度
                float2 relativeVelocity = PathfindingUtility.V2FromV3(_agent.CurrentVelocity)
                    - PathfindingUtility.V2FromV3(AllAgents[otherAgentEntity].CurrentVelocity);

                float distSq = math.pow(math.length(relativePosition), 2);
                float combinedRadius = _agent.Radius + AllAgents[otherAgentEntity].Radius;
                float combinedRadiusSq = math.pow(combinedRadius, 2);

                ASLine line;
                float2 u; // 速度修正量（从当前相对速度到 ORCA 边界的最短向量）

                if (distSq > combinedRadiusSq)
                {
                    // ===== 代理未重叠：标准 ORCA 计算 =====
                    // w = 相对速度 - (1/timeHorizon) * 相对位置（在截锥中的位移向量）
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    float wLengthSq = math.pow(math.length(w), 2);
                    float dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && math.pow(dotProduct1, 2) > combinedRadiusSq * wLengthSq)
                    {
                        // w 指向截锥圆底：沿圆弧法线方向（最短逃逸到圆外）
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.Direction = new float2(unitW.y, -unitW.x); // 切线方向（垂直 unitW）
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW; // 推出圆的修正量
                    }
                    else
                    {
                        // w 在截锥腿外：沿截锥腿方向（最短逃逸到截锥外）
                        float leg = math.sqrt(distSq - combinedRadiusSq); // 切线长度

                        if (math.determinant(new float2x2(relativePosition, w)) > 0.0f)
                        {
                            // w 在左腿外侧：计算左截锥腿的方向
                            line.Direction = new float2(
                                relativePosition.x * leg - relativePosition.y * combinedRadius,
                                relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            // w 在右腿外侧：计算右截锥腿的方向
                            line.Direction = -new float2(
                                relativePosition.x * leg + relativePosition.y * combinedRadius,
                                -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                        u = dotProduct2 * line.Direction - relativeVelocity; // 沿约束线方向的投影修正
                    }
                }
                else
                {
                    // ===== 代理已重叠：紧急避让 =====
                    // 使用当前时间步长（而非 timeHorizon）立即分离
                    float invTimeStep = 1.0f / deltaTime;

                    float2 w = relativeVelocity - invTimeStep * relativePosition;
                    float wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                // ORCA 线的参考点：当前速度 + 修正量的一半（互惠：双方各承担一半避让责任）
                line.Point = PathfindingUtility.V2FromV3(_agent.CurrentVelocity) + 0.5f * u;
                _orcaLines.Add(line);
            }

            // ===== 线性规划求最优速度 =====
            // FindSubOptimalPath：在速度圆（半径=MaxSpeed）和所有 ORCA 半平面约束下
            // 找最接近理想速度（optimalVel * MaxSpeed）的合法速度
            int lineFail = FindSubOptimalPath(_orcaLines, _agent.MaxSpeed, _optimalVel * _agent.MaxSpeed, false, ref _newVelocity);

            // 若第一轮线性规划有约束线无法满足，使用 3D 线性规划（FindSubOptimalPathWithLines）处理
            if (lineFail < _orcaLines.Length)
            {
                FindSubOptimalPathWithLines(_orcaLines, numObstLines, lineFail, _agent.MaxSpeed, ref _newVelocity);
            }
        }

        /// <summary>
        /// 在一条 ORCA 约束线上寻找满足所有之前约束的最优点（2D 线性规划子问题）。
        ///
        /// 算法步骤：
        /// 1. 将约束线与速度圆（radius）求交，得到参数范围 [tLeft, tRight]
        /// 2. 遍历之前所有约束线，进一步缩小参数范围（半平面交集）
        /// 3. 在合法范围内找最接近 optVelocity 的点（投影到线上）
        /// </summary>
        bool FindOptimalPath(NativeList<ASLine> lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            float dotProduct = math.dot(lines[lineNo].Point, lines[lineNo].Direction);
            // 判别式：约束线与速度圆的交点参数
            float discriminant = math.pow(dotProduct, 2) + math.pow(radius, 2) - math.pow(math.length(lines[lineNo].Point), 2);

            if (discriminant < 0.0f)
            {
                return false; // 约束线完全在速度圆外，无解
            }

            float sqrtDiscriminant = math.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;   // 与速度圆的左交点参数
            float tRight = -dotProduct + sqrtDiscriminant;  // 与速度圆的右交点参数

            // 用之前的每条约束线进一步缩小合法参数范围
            for (int i = 0; i < lineNo; ++i)
            {
                // 计算两条线的叉积（行列式）：判断是否平行
                float denominator = math.determinant(new float2x2(lines[lineNo].Direction, lines[i].Direction));
                // 计算两条线的交点参数
                float numerator = math.determinant(new float2x2(lines[i].Direction, lines[lineNo].Point - lines[i].Point));

                if (math.abs(denominator) <= 0.00001f)
                {
                    // 两线平行：检查是否同向（平行且互相满足则跳过，否则无解）
                    if (numerator < 0.0f)
                    {
                        return false;
                    }
                    continue;
                }

                float t = numerator / denominator; // 交点在当前约束线上的参数

                if (denominator >= 0.0f)
                {
                    tRight = math.min(tRight, t); // 缩小右边界
                }
                else
                {
                    tLeft = math.max(tLeft, t);  // 缩小左边界
                }

                if (tLeft > tRight)
                {
                    return false; // 合法范围为空，无解
                }
            }

            // 在合法范围 [tLeft, tRight] 内选择最优点
            if (directionOpt)
            {
                // 方向优化模式：选择最接近目标方向的端点
                if (math.dot(optVelocity, lines[lineNo].Direction) > 0.0f)
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                else
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
            }
            else
            {
                // 速度优化模式：将目标速度投影到线上，然后 clamp 到合法范围
                float t = math.dot(lines[lineNo].Direction, optVelocity - lines[lineNo].Point);

                if (t < tLeft)
                    result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
                else if (t > tRight)
                    result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                else
                    result = lines[lineNo].Point + t * lines[lineNo].Direction;
            }

            return true;
        }

        /// <summary>
        /// 第一阶段线性规划：在速度圆和 ORCA 约束半平面集合中找最优速度（O(n²) 增量算法）。
        ///
        /// 思路：
        /// - 初始结果 = 理想速度（若超出速度圆则截断）
        /// - 逐条添加约束线：若当前结果违反该约束，在该约束线上重新求最优点
        /// - 若无法满足某条约束（FindOptimalPath 返回 false），返回失败的约束线索引
        ///
        /// 返回值：若所有约束均满足则返回 lines.Length；否则返回失败的约束线索引
        /// </summary>
        private int FindSubOptimalPath(NativeList<ASLine> lines, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            if (directionOpt)
            {
                result = optVelocity * radius; // 方向优化：速度大小 = radius
            }
            else if (math.pow(math.length(optVelocity), 2) > math.pow(radius, 2))
            {
                result = math.normalize(optVelocity) * radius; // 超出速度圆：截断到圆上
            }
            else
            {
                result = optVelocity; // 理想速度在速度圆内：直接使用
            }

            for (int i = 0; i < lines.Length; ++i)
            {
                // 检查当前结果是否违反约束线 i（叉积 > 0 表示在错误侧）
                if (math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result)) > 0.0f)
                {
                    float2 tempResult = result;

                    if (!FindOptimalPath(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult; // 恢复之前的结果
                        return i;           // 返回失败的约束线索引
                    }
                }
            }

            return lines.Length; // 所有约束均满足
        }

        /// <summary>
        /// 第二阶段线性规划（3D 线性规划）：处理第一阶段无法满足的约束（过度约束情况）。
        ///
        /// 当第一阶段失败时，意味着存在约束冲突（所有 ORCA 约束同时满足是不可能的）。
        /// 此阶段通过将约束投影到更高维度，找到违反约束最少的"最优次优"速度。
        ///
        /// 算法：
        /// 对每条未能满足的约束线，将之前所有约束线投影到该约束线上，
        /// 在投影约束下用方向优化模式重新求解，最小化违反约束的程度
        /// </summary>
        private void FindSubOptimalPathWithLines(NativeList<ASLine> lines, int numObstLines, int beginLine, float radius, ref float2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Length; ++i)
            {
                // 检查当前结果违反约束的程度（行列式值 > distance 表示更严重的违反）
                if (math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result)) > distance)
                {
                    // 将之前所有约束线投影到当前约束线 i 上
                    NativeList<ASLine> projLines = new NativeList<ASLine>(Allocator.Temp);

                    // 先复制障碍物约束线（不投影，保持不变）
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    // 将代理约束线逐条投影到当前约束线 i
                    for (int j = numObstLines; j < i; ++j)
                    {
                        ASLine line;

                        float determinant = math.determinant(new float2x2(lines[i].Direction, lines[j].Direction));

                        if (math.abs(determinant) <= 0.00001f)
                        {
                            // 两线平行：同向则忽略，反向则取中点
                            if (math.dot(lines[i].Direction, lines[j].Direction) > 0.0f)
                            {
                                continue; // 同向平行：约束线 j 被 i 覆盖
                            }
                            else
                            {
                                line.Point = 0.5f * (lines[i].Point + lines[j].Point); // 反向平行：取中间点
                            }
                        }
                        else
                        {
                            // 非平行：计算两线交点（约束线 j 在约束线 i 上的投影点）
                            line.Point = lines[i].Point + (math.determinant(new float2x2(lines[j].Direction, lines[i].Point - lines[j].Point)) / determinant) * lines[i].Direction;
                        }

                        // 投影线方向：两原始约束线方向的差的单位化
                        line.Direction = math.normalize(lines[j].Direction - lines[i].Direction);
                        projLines.Add(line);
                    }

                    float2 tempResult = result;

                    // 在投影约束下用方向优化重新求解（沿约束线 i 的法线方向）
                    if (FindSubOptimalPath(projLines, radius, new float2(-lines[i].Direction.y, lines[i].Direction.x), true, ref result) < projLines.Length)
                    {
                        result = tempResult; // 若投影求解也失败，保持原结果
                    }

                    // 更新当前违反约束的程度
                    distance = math.determinant(new float2x2(lines[i].Direction, lines[i].Point - result));

                    projLines.Dispose();
                }
            }
        }
    }
}
