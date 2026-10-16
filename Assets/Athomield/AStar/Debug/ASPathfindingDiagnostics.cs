using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Athomield.AStar
{
    /// <summary>
    /// 寻路诊断日志（只读，默认每秒一行）——用于确认"跨岛之后到底发生了什么"。
    ///
    /// 输出内容：
    ///   · 当前 DifferentIslandsPolicy（0 = DontPath / 1 = GoToClosest）
    ///   · 目标（ASTestSphere，找不到就用第一个跟随代理的目标）所在节点与岛屿 ID
    ///   · 前 N 个跟随代理：所在节点/岛屿、Destination 的节点/岛屿、是否跨岛、路径状态
    ///   · 跟随代理统计：总数 / 在跑 / 已停 / 跨岛数
    ///
    /// 用法：把本组件挂到场景里任意 GameObject（例如挂了 GridDrawGizmos 的那个），
    /// 走进封闭区域看 Console：
    ///   · 若"跨岛数 = 总数"，说明岛屿判定生效、代理已被停下 —— 此时若仍掉帧，
    ///     问题就不在这条逻辑上，去看 Profiler → Entities → Systems 哪个系统耗时高；
    ///   · 若"跨岛数 = 0"，说明岛屿数据认为它们是同岛（墙体没在网格上封死 / 岛屿数据未刷新），
    ///     这时搜索确实会跑，属于"网格数据与视觉不一致"的问题。
    ///
    /// 注意：只读诊断，不修改任何 ECS 数据；不需要时可以移除组件或关掉 EnableDiagnostics。
    /// </summary>
    public class ASPathfindingDiagnostics : MonoBehaviour
    {
        [Tooltip("日志间隔（秒）")]
        [SerializeField] float Interval = 1f;

        [Tooltip("每次最多打印几个代理的详情")]
        [SerializeField] int MaxAgentDetails = 3;

        [Tooltip("仅在跨岛代理数量变化时打印（避免刷屏）")]
        [SerializeField] bool OnlyLogOnChange = false;

        [Tooltip("总开关")]
        [SerializeField] bool EnableDiagnostics = true;

        private float m_Timer;
        private int m_LastCrossCount = -1;

        private void Update()
        {
            if (!EnableDiagnostics) return;

            m_Timer += Time.unscaledDeltaTime;
            if (m_Timer < Interval) return;
            m_Timer = 0f;

            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated) return;
            var em = world.EntityManager;

            // ---- Grid 单例 ----
            var gridQuery = em.CreateEntityQuery(ComponentType.ReadOnly<ASGrid>());
            if (gridQuery.IsEmpty)
            {
                gridQuery.Dispose();
                return;
            }
            Entity gridEntity = gridQuery.GetSingletonEntity();
            gridQuery.Dispose();

            if (!em.HasBuffer<ASGridNodesBufferComponent>(gridEntity) ||
                !em.HasBuffer<ASNodeIslandIDBufferComponent>(gridEntity))
            {
                return;
            }

            ASGrid grid = em.GetComponentData<ASGrid>(gridEntity);
            LocalTransform gridTr = em.GetComponentData<LocalTransform>(gridEntity);
            DynamicBuffer<ASGridNodesBufferComponent> nodes = em.GetBuffer<ASGridNodesBufferComponent>(gridEntity);
            DynamicBuffer<ASNodeIslandIDBufferComponent> islands = em.GetBuffer<ASNodeIslandIDBufferComponent>(gridEntity);

            if (islands.Length < nodes.Length)
            {
                Debug.Log($"[AStarDiag] 岛屿数据未就绪: islands={islands.Length} nodes={nodes.Length} —— " +
                          "岛屿分析还没跑完（需要一次 RefreshGrid），此时 DontPath 判定不生效。");
                return;
            }

            // ---- 跟随代理统计 ----
            var agentQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<ASPathfindingRequester>(),
                ComponentType.ReadOnly<ASConstantTargetFollower>(),
                ComponentType.ReadOnly<ASPathFollower>(),
                ComponentType.ReadOnly<LocalTransform>());
            NativeArray<Entity> agents = agentQuery.ToEntityArray(Allocator.Temp);
            agentQuery.Dispose();

            int total = agents.Length;
            int moving = 0, stopped = 0, crossIsland = 0;
            var details = new System.Text.StringBuilder();

            for (int i = 0; i < agents.Length; i++)
            {
                Entity e = agents[i];
                ASPathfindingRequester requester = em.GetComponentData<ASPathfindingRequester>(e);
                ASPathFollower follower = em.GetComponentData<ASPathFollower>(e);
                LocalTransform tr = em.GetComponentData<LocalTransform>(e);

                int agentIndex = WorldPosToIndex(tr.Position, grid, gridTr);
                int destIndex = WorldPosToIndex(requester.Destination, grid, gridTr);
                int agentIsland = ResolveIslandID(grid, nodes, islands, agentIndex);
                int destIsland = ResolveIslandID(grid, nodes, islands, destIndex);
                bool cross = agentIsland >= 0 && destIsland >= 0 && agentIsland != destIsland;

                if (cross) crossIsland++;
                if (follower.PathAvailable && !follower.DestinationReached) moving++; else stopped++;

                if (i < MaxAgentDetails)
                {
                    details.Append($"\n    agent#{i} 节点({XOf(grid, agentIndex)},{YOf(grid, agentIndex)}) 岛={agentIsland}"
                                   + $" → 目标节点({XOf(grid, destIndex)},{YOf(grid, destIndex)}) 岛={destIsland}"
                                   + $" 跨岛={(cross ? "是" : "否")} 路径可用={follower.PathAvailable} 已到达={follower.DestinationReached}"
                                   + $" 冷却={requester.RetryCooldown:F2}s");
                }
            }
            agents.Dispose();

            // ---- 目标（玩家球） ----
            string targetInfo = "未找到 ASTestSphere";
            var sphereQuery = em.CreateEntityQuery(ComponentType.ReadOnly<ASTestSphere>());
            if (!sphereQuery.IsEmpty)
            {
                Entity sphere = sphereQuery.GetSingletonEntity();
                if (em.HasComponent<LocalTransform>(sphere))
                {
                    LocalTransform sphereTr = em.GetComponentData<LocalTransform>(sphere);
                    int sIndex = WorldPosToIndex(sphereTr.Position, grid, gridTr);
                    int sIsland = ResolveIslandID(grid, nodes, islands, sIndex);
                    bool sAccessible = nodes[sIndex].Node.IsAccessible;
                    targetInfo = $"节点({XOf(grid, sIndex)},{YOf(grid, sIndex)}) 岛={sIsland} 该节点可通行={(sAccessible ? "是" : "否")}";
                }
            }
            sphereQuery.Dispose();

            // ---- 寻路参数（每帧上限）----
            int opsPerFrame = -1;
            if (em.HasComponent<ASPathfindingParameters>(gridEntity))
            {
                opsPerFrame = em.GetComponentData<ASPathfindingParameters>(gridEntity).MaxPathfindingOpsPerFrame;
            }

            if (OnlyLogOnChange && crossIsland == m_LastCrossCount) return;
            m_LastCrossCount = crossIsland;

            Debug.Log($"[AStarDiag] 策略={grid.DifferentIslandsPolicy} (0=DontPath/1=GoToClosest) | 网格 {grid.NodeXCount}×{grid.NodeYCount}"
                      + $" | 每帧寻路上限={opsPerFrame} | 目标球: {targetInfo}"
                      + $"\n  跟随代理: 总 {total} / 在跑 {moving} / 已停 {stopped} / 跨岛 {crossIsland}"
                      + details);
        }

        // ---- 与 GridUtility.WorldPosToNode / 各系统保持一致的口径 ----

        private static int WorldPosToIndex(float3 pos, ASGrid grid, LocalTransform gridTr)
        {
            float percentX = math.clamp((pos.x - gridTr.Position.x + (grid.NodeXCount * grid.NodeSize) / 2)
                                        / (grid.NodeXCount * grid.NodeSize), 0, 1);
            float percentY = math.clamp((pos.z - gridTr.Position.z + (grid.NodeYCount * grid.NodeSize) / 2)
                                        / (grid.NodeYCount * grid.NodeSize), 0, 1);

            int x = (int)math.round((grid.NodeXCount - 1) * percentX);
            int y = (int)math.round((grid.NodeYCount - 1) * percentY);

            int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;
            return grid.NodeXCount > grid.NodeYCount ? x + y * d : y + x * d;
        }

        private static int XOf(ASGrid grid, int index)
        {
            int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;
            return grid.NodeXCount > grid.NodeYCount ? index % d : index / d;
        }

        private static int YOf(ASGrid grid, int index)
        {
            int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;
            return grid.NodeXCount > grid.NodeYCount ? index / d : index % d;
        }

        private static int ResolveIslandID(ASGrid grid, DynamicBuffer<ASGridNodesBufferComponent> nodes,
            DynamicBuffer<ASNodeIslandIDBufferComponent> islands, int index)
        {
            if (index < 0 || index >= nodes.Length || index >= islands.Length) return -1;
            if (nodes[index].Node.IsAccessible) return islands[index].IslandID;

            int x = XOf(grid, index);
            int y = YOf(grid, index);

            int island = TryNeighbour(grid, nodes, islands, x + 1, y);
            if (island >= 0) return island;
            island = TryNeighbour(grid, nodes, islands, x, y + 1);
            if (island >= 0) return island;
            island = TryNeighbour(grid, nodes, islands, x - 1, y);
            if (island >= 0) return island;
            return TryNeighbour(grid, nodes, islands, x, y - 1);
        }

        private static int TryNeighbour(ASGrid grid, DynamicBuffer<ASGridNodesBufferComponent> nodes,
            DynamicBuffer<ASNodeIslandIDBufferComponent> islands, int x, int y)
        {
            if (x < 0 || x >= grid.NodeXCount || y < 0 || y >= grid.NodeYCount) return -1;

            int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;
            int index = grid.NodeXCount > grid.NodeYCount ? x + y * d : y + x * d;

            if (index < 0 || index >= nodes.Length || index >= islands.Length) return -1;
            if (!nodes[index].Node.IsAccessible) return -1;

            return islands[index].IslandID;
        }
    }
}
