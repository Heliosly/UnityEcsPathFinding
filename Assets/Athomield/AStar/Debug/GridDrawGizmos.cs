using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Athomield.AStar
{
    /// <summary>
    /// 导航网格和代理的 Scene 视图可视化组件（Editor + Play Mode 均可使用）。
    ///
    /// 挂载此 MonoBehaviour 到任意 GameObject 后，在 Inspector 中选择需要显示的调试层：
    /// - ShowGrid：绘制每个网格节点（可通行=GridColor，不可通行=红色）
    /// - ShowGridIslands：用不同颜色区分连通区域（Island），需要 Play Mode 下才有岛屿数据
    /// - ShowPenalties：按惩罚值给地面上色（惩罚=0 为普通地面色 GridColor，
    ///   越高越向中段色/高端色过渡，而不是"无惩罚=黑"）
    /// - ShowPaths：绘制每个代理当前的路径点连线（黄色折线）
    ///
    /// 此外在 Update 中监听 A 键，按下后触发一次网格刷新（调试用途）。
    ///
    /// 注意：OnDrawGizmos 每帧都调用，包含 EntityQuery，在节点数量大时可能有性能开销，
    /// 建议在 Profile 时关闭不需要的可视化选项。
    /// </summary>
    public class GridDrawGizmos : MonoBehaviour
    {
        /// <summary>ECS EntityManager 缓存，首次访问时从默认 World 获取</summary>
        private EntityManager mEntityManager;

        /// <summary>可通行节点的填充颜色（ShowGrid 开启时生效）</summary>
        [SerializeField]
        Color GridColor;

        /// <summary>节点边框的线框颜色（ShowGrid 开启时叠加绘制）</summary>
        [SerializeField]
        Color GridWireColor;

        /// <summary>是否绘制网格节点（true = 绘制所有节点的填充方块和线框）</summary>
        [SerializeField]
        bool ShowGrid;

        /// <summary>
        /// 是否用颜色区分岛屿（连通区域）。
        /// 仅在 Play Mode 且 CheckNodeIslandSystem 已完成计算后有效。
        /// 不同岛屿使用 IslandColors 数组中的对应颜色，数量不够时自动随机生成新颜色。
        /// </summary>
        [SerializeField]
        bool ShowGridIslands;

        /// <summary>
        /// 各岛屿对应的显示颜色数组。
        /// 索引 i 对应 IslandID=i 的岛屿颜色。
        /// 若岛屿数量超过数组长度，会自动调用 GenerateRandomColors 扩容。
        /// </summary>
        [SerializeField]
        Color[] IslandColors;

        /// <summary>
        /// 是否按惩罚值给节点上色（惩罚热力图）。
        /// 颜色映射为三段色阶：t = Penalty / 归一化上限，
        ///   t = 0      → GridColor（普通地面色，无惩罚）
        ///   0 < t ≤ 0.5 → GridColor → PenaltyMediumColor
        ///   0.5 < t ≤ 1 → PenaltyMediumColor → PenaltyHighColor
        /// 仅在 Play Mode 下有效（CheckObstaclesSystem 才会填充惩罚值）。
        /// 不可通行节点仍显示为红色（障碍信息优先于惩罚热力）。
        /// 与 ShowGridIslands 同时开启时，惩罚色阶覆盖岛屿颜色（后绘制的优先）。
        /// 注意：节点绘制整体受 ShowGrid 控制，只开 ShowPenalties 不会单独出图。
        /// </summary>
        [SerializeField]
        bool ShowPenalties;

        /// <summary>
        /// 惩罚热力图的归一化上限：t = Penalty / MaxPenalty。
        /// 填 &gt; 0 表示手动指定（建议填场景中的最大惩罚值，超出部分会被截断到色阶顶端）；
        /// 填 &lt;= 0（默认）表示每帧自动取当前网格【可通行节点】的最大惩罚值 —— 无需手动配置，
        /// 也避免旧实现中 MaxPenalty=0 导致 Penalty/0 = NaN 而被渲染成纯黑的问题。
        /// </summary>
        [SerializeField] private int MaxPenalty;

        /// <summary>色阶中段颜色（对应 t=0.5 附近的惩罚强度）</summary>
        [SerializeField]
        Color PenaltyMediumColor = new Color(1f, 0.85f, 0.15f);

        /// <summary>色阶高端颜色（对应 t=1，即惩罚最大处）</summary>
        [SerializeField]
        Color PenaltyHighColor = new Color(1f, 0.15f, 0.1f);

        /// <summary>是否绘制每个代理的当前路径（黄色折线连接路径点）</summary>
        [SerializeField]
        bool ShowPaths;

        /// <summary>
        /// Unity 的 Gizmos 绘制回调，每帧（包括 Editor 中）均会调用。
        /// 根据 Inspector 中选择的选项，绘制网格、岛屿、惩罚热力图、路径和代理胶囊体。
        /// </summary>
        private void OnDrawGizmos()
        {
            // 懒初始化 EntityManager（在 Editor 中 World 可能尚未创建，需保护）
            if (mEntityManager == default)
                mEntityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

            // 注意：这里不再改写序列化字段 MaxPenalty。
            // 旧实现写的是 "if (MaxPenalty <= 0) MaxPenalty = 0;" —— 既没有防止除零，
            // 还会把用户在 Inspector 里设的负值/0 状态永久改写。归一化上限改为每网格局部计算。

            // 查询所有有路径缓冲区的代理 Entity（用于绘制路径和胶囊体）
            var agentsQuery = mEntityManager.CreateEntityQuery(new []{ComponentType.ReadOnly<ASPathBufferComponent>(), ComponentType.ReadOnly<LocalTransform>()});
            var agentEntities = agentsQuery.ToEntityArray(Unity.Collections.Allocator.Temp);

            if(ShowGrid)
            {
                // 查询唯一的 Grid Entity
                var query = mEntityManager.CreateEntityQuery(ComponentType.ReadOnly<ASGrid>());
                var entities = query.ToEntityArray(Unity.Collections.Allocator.Temp);

                foreach (var entity in entities)
                {
                    ASGrid grid = mEntityManager.GetComponentData<ASGrid>(entity);
                    LocalTransform transform = mEntityManager.GetComponentData<LocalTransform>(entity);
                    DynamicBuffer<ASGridNodesBufferComponent> nodes = mEntityManager.GetBuffer<ASGridNodesBufferComponent>(entity);
                    DynamicBuffer<ASNodeIslandIDBufferComponent> nodeIslands = mEntityManager.GetBuffer<ASNodeIslandIDBufferComponent>(entity);

                    // 计算网格左下角（X-，Z- 方向）的世界坐标（节点绘制的起点参考）
                    Vector3 startCorner = new Vector3(transform.Position.x, transform.Position.y, transform.Position.z)
                        - Vector3.right * ((grid.NodeXCount * grid.NodeSize) / 2)
                        - Vector3.forward * ((grid.NodeYCount * grid.NodeSize) / 2);

                    // d = max(NodeXCount, NodeYCount)，用于一维索引计算（与 GridAuthoring 一致）
                    int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;

                    // 惩罚热力图的归一化上限（本网格局部）：
                    //   MaxPenalty > 0  → 手动指定，直接用（超出部分在取色时截断）
                    //   MaxPenalty <= 0 → 自动取【可通行节点】的最大惩罚值
                    // 两种情况下 penaltyMax 都保证 >= 0；取色处再对 0 做保护，
                    // 因此不会再出现 Penalty / 0 = NaN → 纯黑的问题。
                    float penaltyMax = MaxPenalty;
                    if (ShowPenalties && penaltyMax <= 0f)
                    {
                        penaltyMax = 0f;
                        for (int i = 0; i < nodes.Length; i++)
                        {
                            // 只统计实际会上色的节点（不可通行节点保持红色，不参与色阶归一化）
                            if (!nodes[i].Node.IsAccessible) continue;
                            int p = nodes[i].Node.Penalty;
                            if (p > penaltyMax) penaltyMax = p;
                        }
                    }

                    for (int x = 0; x < grid.NodeXCount; x++)
                    {
                        for (int y = 0; y < grid.NodeYCount; y++)
                        {
                            // 计算当前节点在一维缓冲区中的索引
                            int index = 0;
                            if (grid.NodeXCount > grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            // 默认颜色：可通行=GridColor，不可通行=红色
                            Gizmos.color = nodes[index].Node.IsAccessible ? GridColor : Color.red;

                            // 岛屿颜色覆盖（Play Mode + 岛屿数据就绪 + 节点可通行）
                            if(ShowGridIslands && Application.isPlaying && nodeIslands.Length > 0 && nodes[index].Node.IsAccessible)
                            {
                                // IslandColors 未在 Inspector 赋值时（null）也要能自动生成，避免空引用
                                if(IslandColors == null || IslandColors.Length - 1 < nodeIslands[index].IslandID)
                                {
                                    GenerateRandomColors(nodeIslands[index].IslandID + 1);
                                }

                                Gizmos.color = IslandColors[nodeIslands[index].IslandID];
                            }

                            // 惩罚热力图颜色覆盖（Play Mode 下，CheckObstaclesSystem 运行后才有数据）
                            // 只对可通行节点上色：不可通行节点保留上面的红色，障碍信息优先。
                            if(ShowPenalties && Application.isPlaying && nodes[index].Node.IsAccessible)
                            {
                                // 归一化：t ∈ [0,1]；penaltyMax=0（全图无惩罚）时 t=0 → 普通地面色
                                int penalty = nodes[index].Node.Penalty;
                                float t = penaltyMax > 0f ? Mathf.Clamp01(penalty / penaltyMax) : 0f;
                                Gizmos.color = PenaltyColor(t);
                            }

                            // 绘制节点填充方块（高度 0.1f 使其贴近地面但仍可见）
                            Gizmos.DrawCube(
                                startCorner + Vector3.right * ((x * grid.NodeSize * 2 + grid.NodeSize) / 2)
                                            + Vector3.forward * ((y * grid.NodeSize * 2 + grid.NodeSize) / 2),
                                new Vector3(grid.NodeSize, 0.1f, grid.NodeSize));

                            // 绘制节点线框（使用 GridWireColor，叠加在填充方块上方，便于区分边界）
                            Gizmos.color = GridWireColor;
                            Gizmos.DrawWireCube(
                                startCorner + Vector3.right * ((x * grid.NodeSize * 2 + grid.NodeSize) / 2)
                                            + Vector3.forward * ((y * grid.NodeSize * 2 + grid.NodeSize) / 2),
                                new Vector3(grid.NodeSize, 0.1f, grid.NodeSize));
                        }
                    }
                }

                entities.Dispose();
            }

            // 遍历所有代理：绘制路径折线和代理胶囊体
            foreach (var agentEntity in agentEntities)
            {
                if(ShowPaths)
                {
                    // 用黄色折线依次连接路径点（不包含最后一个点到自身的线）
                    DynamicBuffer<ASPathBufferComponent> pathPoints = mEntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);
                    for (int i = 0; i < pathPoints.Length - 1; i++)
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawLine(pathPoints[i].Point, pathPoints[i + 1].Point);
                    }
                }

                // 读取代理参数和位置，绘制橙色胶囊体（便于直观感知代理的物理大小）
                ASAgent agent = mEntityManager.GetComponentData<ASAgent>(agentEntity);
                LocalTransform agentLocTr = mEntityManager.GetComponentData<LocalTransform>(agentEntity);

                DrawAgentCyl(agent.Radius, agent.Height, agentLocTr.Position, agentLocTr.Rotation);
            }

            agentEntities.Dispose();
        }

        /// <summary>
        /// 惩罚值归一化系数 → 地面颜色（三段色阶）。
        /// t = 0        → GridColor（无惩罚 = 普通地面色；旧灰度实现在这里是黑色，正是"地面变黑"的第二个原因）
        /// 0 &lt; t ≤ 0.5 → GridColor → PenaltyMediumColor
        /// 0.5 &lt; t ≤ 1 → PenaltyMediumColor → PenaltyHighColor
        /// </summary>
        /// <param name="t">归一化后的惩罚强度，调用方保证已 clamp 到 [0,1]</param>
        Color PenaltyColor(float t)
        {
            t = Mathf.Clamp01(t);
            if (t <= 0f) return GridColor;
            if (t <= 0.5f) return Color.Lerp(GridColor, PenaltyMediumColor, t * 2f);
            return Color.Lerp(PenaltyMediumColor, PenaltyHighColor, (t - 0.5f) * 2f);
        }

        /// <summary>
        /// 在指定位置和朝向绘制一个胶囊形状的 Gizmos（用两个圆 + 四条竖线近似）。
        /// 颜色固定为橙色（红色 * 0.7 + 黄色 * 0.7），用于直观显示代理的碰撞体积。
        /// </summary>
        /// <param name="_radius">代理的物理半径</param>
        /// <param name="_height">代理的物理高度</param>
        /// <param name="_pos">代理的世界中心位置</param>
        /// <param name="_rot">代理的朝向（用于确定上方向）</param>
        void DrawAgentCyl(float _radius, float _height, float3 _pos, quaternion _rot)
        {
            Gizmos.color = Color.red * .7f + Color.yellow * .7f;
            int segments = 20; // 圆的分段数，越多越平滑但 Gizmos 开销越大

            Vector3 center = _pos;
            Quaternion rotation = _rot;
            Vector3 up = rotation * Vector3.up;

            // 顶部和底部圆心位置（上下各偏移 Height/2）
            Vector3 topCenter = center + up * _height / 2;
            Vector3 bottomCenter = center - up * _height / 2;

            // 绘制顶部和底部圆形
            DrawCircle(topCenter, _radius, rotation, segments);
            DrawCircle(bottomCenter, _radius, rotation, segments);

            // 绘制 4 条竖线连接上下圆（均匀分布在圆周上）
            int lineSegments = 4;
            for (int i = 0; i < lineSegments; i++)
            {
                float angle = i * Mathf.PI * 2 / lineSegments;
                // 在 XZ 平面上均匀采样角度，转换为旋转后的世界方向
                Vector3 dir = rotation * new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
                Vector3 topPoint = topCenter + dir * _radius;
                Vector3 bottomPoint = bottomCenter + dir * _radius;
                Gizmos.DrawLine(topPoint, bottomPoint);
            }
        }

        /// <summary>
        /// 在指定位置绘制一个圆形（由若干线段近似），用于构成胶囊体的顶部和底部。
        /// </summary>
        /// <param name="center">圆心的世界位置</param>
        /// <param name="radius">圆的半径</param>
        /// <param name="rotation">圆所在平面的朝向（决定圆的法向量方向）</param>
        /// <param name="segments">圆的分段数，越大越平滑</param>
        void DrawCircle(Vector3 center, float radius, Quaternion rotation, int segments)
        {
            // 在旋转后的局部空间中，圆位于 right-forward 平面上
            Vector3 right = rotation * Vector3.right;
            Vector3 forward = rotation * Vector3.forward;

            // 从 0° 处的第一个点开始，依次连接到下一个点，最后连回起点（i=segments 时与 i=0 相同）
            Vector3 prevPoint = center + right * radius;
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * Mathf.PI * 2 / segments;
                // 参数化圆：P(angle) = center + right * cos(angle) + forward * sin(angle)
                Vector3 dir = right * Mathf.Cos(angle) + forward * Mathf.Sin(angle);
                Vector3 nextPoint = center + dir * radius;
                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// 将 IslandColors 扩容到 _newLength 并为每个岛屿随机分配一种颜色。
        /// 在发现新岛屿（IslandID 超出当前数组范围）时自动调用。
        /// </summary>
        /// <param name="_newLength">新数组的目标长度（通常 = 最大 IslandID + 1）</param>
        void GenerateRandomColors(int _newLength)
        {
            IslandColors = new Color[_newLength];
            for (int i = 0; i < _newLength; i++)
            {
                IslandColors[i] = new Color(
                    UnityEngine.Random.Range(0, 1f),
                    UnityEngine.Random.Range(0, 1f),
                    UnityEngine.Random.Range(0, 1f));
            }
        }

        /// <summary>
        /// 监听 A 键按下事件，按下后立即触发一次网格刷新（调试用途）。
        /// 将 Grid Entity 上的 RefreshGrid 标志设为 true，
        /// CheckObstaclesSystem 下一帧会检测到该标志并重新扫描障碍物。
        /// </summary>
        private void Update()
        {
            // 懒初始化 EntityManager
            if (mEntityManager == default)
                mEntityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

            if (Input.GetKeyDown(KeyCode.A))
            {
                // 查找 Grid Entity 并设置刷新标志
                var query = mEntityManager.CreateEntityQuery(ComponentType.ReadOnly<ASGrid>());
                var entities = query.ToEntityArray(Unity.Collections.Allocator.Temp);

                foreach (var entity in entities)
                {
                    ASGrid grid = mEntityManager.GetComponentData<ASGrid>(entity);
                    grid.RefreshGrid = true;
                    mEntityManager.SetComponentData(entity, grid);
                    Debug.LogError("Regiter");
                }
            }
        }
    }
}