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
    /// GroupTarget 场景的代理生成系统。
    /// 游戏启动后仅执行一次：将代理按同心圆排列生成，并为每个代理挂载
    /// ASConstantTargetFollower，使其持续追踪玩家控制的目标球（ASTestSphere）。
    /// </summary>
    [RequireMatchingQueriesForUpdate]
    partial struct GTSpawnerSystem : ISystem
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

            // 从场景中读取由 GTSpawner 烘焙好的 Prefab Entity 引用
            foreach(var agentPrefHolder in SystemAPI.Query<GTAgentPrefabComponent>())
            {
                agentPrefab = agentPrefHolder.Value;
            }

            // Prefab 未就绪时跳过，等下一帧再试
            if (agentPrefab == Entity.Null)
            {
                ecb.Dispose();
                return;
            }

            // 获取目标球 Entity，用于设置 ASConstantTargetFollower.Target
            Entity sphere = SystemAPI.GetSingletonEntity<ASTestSphere>();

            // 调大停止距离，避免代理过于靠近目标球时互相挤压
            ASAgent ag = state.EntityManager.GetComponentData<ASAgent>(agentPrefab);
            //ag.IgnoreAvoidanceWhenDestReached = true;
            ag.StoppingDistance = 3;
            ecb.SetComponent(agentPrefab, ag);

            hasSpawned = true;

            // 2 圈同心圆配置：distances 为圆半径，numbers 为该圈的代理数量
            // 共生成 20 + 20 = 40 个代理
            NativeArray<int> distances = new NativeArray<int>(2, Allocator.Temp);
            NativeArray<int> numbers = new NativeArray<int>(2, Allocator.Temp);

            distances[0] = 10;  numbers[0] = 100;
            distances[1] = 11;  numbers[1] = 100;

            for (int j = 0; j < distances.Length; j++)
            {
                for (int i = 0; i < numbers[j]; i++)
                {
                    // 将索引 i 均匀映射到 [0, 360) 度，用于圆周均布和 HSV 着色
                    float ang = (math.PI * 2) / numbers[j] * i;
                    ang = math.degrees(ang);
                    ang = ang < 0 ? ang + 360 : ang;

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

                    // 初始目标点先设置为圆心对面（后续由 ASConstantTargetFollower 动态更新）
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

                    // 持续追踪目标球：目标移动时自动重新寻路
                    ASConstantTargetFollower targetFollower = new ASConstantTargetFollower()
                    {
                        Target = sphere,
                    };

                    Entity e = ecb.Instantiate(agentPrefab);
                    ecb.SetComponent(e, locTransform);
                    ecb.SetComponent(e, requester);
                    ecb.AddComponent(e, color);
                    ecb.AddComponent(e, targetFollower);
                }
            }

            distances.Dispose();
            numbers.Dispose();

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
