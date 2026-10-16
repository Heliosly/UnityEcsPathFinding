using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Rendering;
using KNN;
using KNN.Jobs;
using Unity.Jobs;
using Unity.Physics;

namespace Athomield.AStar.Demos
{
    /// <summary>
    /// COA（Circle Of Agents）场景的代理生成系统。
    /// 游戏启动后仅执行一次：将代理按同心圆排列生成在场景中，
    /// 每个代理的目标点设置为圆心对面方向，形成对穿效果以测试大规模避障。
    /// </summary>
    //RequireMatchingQueriesForUpdate 是一个应用于 SystemBase 或 ISystem 的特性（Attribute）。
    //它的核心作用是进行性能优化：如果该系统管理的所有查询（EntityQuery）都找不到任何匹配的实体，
    //就自动跳过该系统的 OnUpdate 方法，不执行这一帧的更新逻辑
    [RequireMatchingQueriesForUpdate]
    partial struct COASpawnerSystem : ISystem
    {
        /// <summary>防止重复生成的标志，OnUpdate 只执行一次生成逻辑</summary>
        bool hasSpawned;

        //[BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            if (hasSpawned)
            {
                return;
            }

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            Entity agentPrefab = Entity.Null;

            // 从场景中读取由 COASpawner 烘焙好的 Prefab Entity 引用
            foreach(var agentPrefHolder in SystemAPI.Query<AgentPrefabComponent>())
            {
                agentPrefab = agentPrefHolder.Value;
            }

            // Prefab 未就绪时跳过，等下一帧再试
            if (agentPrefab == Entity.Null)
            {
                ecb.Dispose();
                return;
            }

            // 到达目的地后停止参与避障，减少静止代理的计算开销
            ASAgent ag = state.EntityManager.GetComponentData<ASAgent>(agentPrefab);
            ag.IgnoreAvoidanceWhenDestReached = true;
            ecb.SetComponent(agentPrefab, ag);

            hasSpawned = true;

            // 4 圈同心圆配置：distances 为圆半径，numbers 为该圈的代理数量
            // 共生成 75 + 80 + 85 + 90 = 330 个代理
            NativeArray<int> distances = new NativeArray<int>(4, Allocator.Temp);
            NativeArray<int> numbers = new NativeArray<int>(4, Allocator.Temp);

            distances[0] = 25;  numbers[0] = 75;
            distances[1] = 26;  numbers[1] = 80;
            distances[2] = 27;  numbers[2] = 85;
            distances[3] = 28;  numbers[3] = 90;

            for (int j = 0; j < distances.Length; j++)
            {
                for (int i = 0; i < numbers[j]; i++)
                {
                    // 将索引 i 均匀映射到 [0, 360) 度，用于圆周均布和 HSV 着色
                    float ang = (math.PI * 2) / numbers[j] * i;
                    ang = math.degrees(ang);
                    ang = ang < 0 ? ang + 360 : ang;
                    Debug.LogError(ang.ToString());

                    // 用角度作为色相（H），生成彩虹色以区分不同代理
                    float3 newColor = new(ang / 360, 1, 1);
                    Color rgbColor = Color.HSVToRGB(newColor.x, newColor.y, newColor.z);
                    rgbColor.a = 1;

                    // 代理在圆周上的生成位置，Y=1 避免陷入地面
                    float3 spawnPos = new float3(
                        math.cos((math.PI * 2) / numbers[j] * i) * distances[j],
                        1,
                        math.sin((math.PI * 2) / numbers[j] * i) * distances[j]);

                    // 面朝圆心（-spawnPos 方向），用于初始朝向
                    LocalTransform locTransform = new LocalTransform()
                    {
                        Position = spawnPos,
                        Rotation = quaternion.LookRotationSafe(-spawnPos, math.up()),
                        Scale = 1
                    };

                    // 目标点 = 当前位置沿正前方延伸 2 倍半径，即穿过圆心到对面
                    ASPathfindingRequester requester = new ASPathfindingRequester()
                    {
                        Destination = spawnPos + (locTransform.Forward() * distances[j] * 2),
                        RequestEntity = Entity.Null,
                        RequestGiven = false,   // false = 立即触发寻路请求
                    };

                    // 设置代理的渲染颜色（通过 Entities Graphics 的 MaterialProperty）
                    URPMaterialPropertyBaseColor color = new URPMaterialPropertyBaseColor()
                    {
                        Value = new float4(rgbColor.r, rgbColor.g, rgbColor.b, rgbColor.a)
                    };

                    Entity e = ecb.Instantiate(agentPrefab);
                    ecb.SetComponent(e, locTransform);
                    ecb.SetComponent(e, requester);
                    ecb.AddComponent(e, color);
                }
            }

            distances.Dispose();
            numbers.Dispose();

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

            // 打印本次生成的代理总数
            int totalAgents = 75 + 80 + 85 + 90;
            Debug.Log($"[COA] 已生成代理数量：{totalAgents}");

            // 若需要查询场景中实际存活的 ASAgent Entity 数量，使用以下方式：
            // var query = state.GetEntityQuery(ComponentType.ReadOnly<ASAgent>());
            // Debug.Log($"[COA] 当前场景 ASAgent 数量：{query.CalculateEntityCount()}");
        }
    }
}
