using System.Collections.Generic;
using KNN;
using KNN.Internal;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Athomield.AStar
{
    /// <summary>
    /// KD 树实时可视化组件（Scene 视图 Gizmos）。
    ///
    /// 使用方式：
    ///   1. 挂载此脚本到场景中任意 GameObject
    ///   2. 运行游戏，在 Scene 视图中即可看到 KD 树结构
    ///   3. 通过 Inspector 开关各层可视化选项
    ///
    /// 显示内容：
    ///   - KD 树各层包围盒（颜色渐变：根=红，叶=蓝）
    ///   - 分割平面线框（X轴=橙，Y轴=绿，Z轴=紫）
    ///   - 代理间的 K 近邻连线（青色）
    ///   - 代理位置点（黄色球）
    ///
    /// 注意：每帧会重新构建一个临时 KD 树用于可视化，仅供 Scene 调试使用，
    /// 不影响 AgentMovementSystem 中的正式 KD 树计算。
    /// </summary>
    [AddComponentMenu("Athomield/AStar/KD Tree Visualizer")]
    public class KdTreeVisualizer : MonoBehaviour
    {
        [Header("KD 树包围盒")]
        [Tooltip("是否显示 KD 树各层节点的包围盒")]
        [SerializeField] bool showTree = true;

        [Tooltip("最大显示深度（建议 4~6，过大会有大量小包围盒）")]
        [SerializeField, Range(1, 12)] int maxDepth = 5;

        [Tooltip("各深度的颜色渐变（左=根节点，右=叶节点）")]
        [SerializeField] Gradient depthGradient;

        [Header("分割平面")]
        [Tooltip("是否显示每个内部节点的分割平面线框")]
        [SerializeField] bool showSplitPlanes = true;

        [Header("邻居连线")]
        [Tooltip("是否显示代理间的 KNN 邻居连线（复用 ASAgentNeighboursBufferComponent 的已计算结果）")]
        [SerializeField] bool showNeighbours = true;

        [Tooltip("邻居连线颜色")]
        [SerializeField] Color neighbourColor = new Color(0f, 1f, 1f, 0.7f);

        [Header("代理位置")]
        [Tooltip("是否显示每个避障代理的位置标记球")]
        [SerializeField] bool showAgentDots = true;

        [Tooltip("标记球半径")]
        [SerializeField, Range(0.02f, 1f)] float dotRadius = 0.15f;

        // ─── 缓存数据（Update 写入，OnDrawGizmos 读取）───────────────────

        struct CachedNode
        {
            public Vector3 center;
            public Vector3 size;
            public int depth;
            public int partitionAxis;   // -1=叶节点  0=X  1=Y  2=Z
            public float partitionCoord;
            public Vector3 boundsMin;
            public Vector3 boundsMax;
        }

        readonly List<CachedNode> m_cachedNodes        = new List<CachedNode>();
        readonly List<(Vector3 from, Vector3 to)> m_neighbourLines = new List<(Vector3, Vector3)>();
        readonly List<Vector3> m_agentPositions        = new List<Vector3>();
        int m_maxDepthFound = 1;

        EntityManager m_em;

        // 分割平面颜色：X=橙 Y=绿 Z=紫
        static readonly Color[] k_splitColors =
        {
            new Color(1.0f, 0.55f, 0.0f, 0.5f),
            new Color(0.2f, 1.0f, 0.3f, 0.5f),
            new Color(0.7f, 0.2f, 1.0f, 0.5f),
        };

        // ─── Unity 回调 ─────────────────────────────────────────────────

        void Reset()
        {
            // 组件首次添加时设置默认渐变（红→黄→绿→蓝）
            depthGradient = new Gradient();
            depthGradient.SetKeys(
                new[]
                {
                    new GradientColorKey(new Color(1.0f, 0.2f, 0.2f), 0.00f),
                    new GradientColorKey(new Color(1.0f, 0.85f, 0.1f), 0.33f),
                    new GradientColorKey(new Color(0.1f, 0.9f, 0.4f), 0.66f),
                    new GradientColorKey(new Color(0.2f, 0.4f, 1.0f), 1.00f),
                },
                new[]
                {
                    new GradientAlphaKey(0.85f, 0.0f),
                    new GradientAlphaKey(0.25f, 1.0f),
                }
            );
        }

        void Update()
        {
            if (!Application.isPlaying) return;

            m_cachedNodes.Clear();
            m_neighbourLines.Clear();
            m_agentPositions.Clear();
            m_maxDepthFound = 1;

            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null) return;
            m_em = world.EntityManager;

            // 查询所有避障代理
            var query = m_em.CreateEntityQuery(
                ComponentType.ReadOnly<LocalTransform>(),
                ComponentType.ReadOnly<ASAvoidanceAgent>());

            if (query.IsEmpty)
            {
                query.Dispose();
                return;
            }

            var entities  = query.ToEntityArray(Allocator.Temp);
            var positions = new NativeArray<float3>(entities.Length, Allocator.Temp);

            for (int i = 0; i < entities.Length; i++)
            {
                positions[i] = m_em.GetComponentData<LocalTransform>(entities[i]).Position;
                m_agentPositions.Add(positions[i]);
            }

            // 构建临时 KD 树：Allocator.Temp 不可传给 Job，直接调用 Rebuild() 在主线程同步完成
            var container = new KnnContainer(positions, false, Allocator.Temp);
            container.Rebuild();

            if (showTree || showSplitPlanes)
                ExtractTreeNodes(container);

            if (showNeighbours)
                ExtractNeighbourLines(entities, positions);

            container.Dispose();
            positions.Dispose();
            entities.Dispose();
            query.Dispose();
        }

        void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            // 1. KD 树包围盒（按深度着色）
            if (showTree)
            {
                foreach (var node in m_cachedNodes)
                {
                    float t = m_maxDepthFound > 0 ? (float)node.depth / m_maxDepthFound : 0f;
                    Gizmos.color = depthGradient != null
                        ? depthGradient.Evaluate(t)
                        : Color.white;
                    Gizmos.DrawWireCube(node.center, node.size);
                }
            }

            // 2. 分割平面线框（X=橙 Y=绿 Z=紫）
            if (showSplitPlanes)
            {
                foreach (var node in m_cachedNodes)
                {
                    int axis = node.partitionAxis;
                    if (axis < 0 || axis > 2) continue;
                    Gizmos.color = k_splitColors[axis];
                    DrawSplitPlane(node.boundsMin, node.boundsMax, axis, node.partitionCoord);
                }
            }

            // 3. 邻居连线
            if (showNeighbours)
            {
                Gizmos.color = neighbourColor;
                foreach (var (from, to) in m_neighbourLines)
                    Gizmos.DrawLine(from, to);
            }

            // 4. 代理位置球
            if (showAgentDots)
            {
                Gizmos.color = new Color(1f, 0.9f, 0.05f, 0.9f);
                foreach (var pos in m_agentPositions)
                    Gizmos.DrawSphere(pos, dotRadius);
            }
        }

        // ─── 私有辅助 ────────────────────────────────────────────────────

        void ExtractTreeNodes(KnnContainer container)
        {
            int root = container.RootIndex;
            if (root < 0) return;

            var queue = new Queue<(int idx, int depth)>();
            queue.Enqueue((root, 0));

            while (queue.Count > 0)
            {
                var (idx, depth) = queue.Dequeue();
                if (depth > maxDepth) continue;

                KdNode node = container.InternalNodes[idx];
                float3 bMin = node.Bounds.Min;
                float3 bMax = node.Bounds.Max;

                m_cachedNodes.Add(new CachedNode
                {
                    center         = (Vector3)((bMin + bMax) * 0.5f),
                    size           = (Vector3)(bMax - bMin),
                    depth          = depth,
                    partitionAxis  = node.PartitionAxis,
                    partitionCoord = node.PartitionCoordinate,
                    boundsMin      = bMin,
                    boundsMax      = bMax,
                });

                if (depth > m_maxDepthFound) m_maxDepthFound = depth;

                if (!node.Leaf)
                {
                    if (node.NegativeChildIndex >= 0)
                        queue.Enqueue((node.NegativeChildIndex, depth + 1));
                    if (node.PositiveChildIndex >= 0)
                        queue.Enqueue((node.PositiveChildIndex, depth + 1));
                }
            }
        }

        void ExtractNeighbourLines(NativeArray<Entity> entities, NativeArray<float3> positions)
        {
            for (int i = 0; i < entities.Length; i++)
            {
                Entity e = entities[i];
                if (!m_em.HasBuffer<ASAgentNeighboursBufferComponent>(e)) continue;

                var buf  = m_em.GetBuffer<ASAgentNeighboursBufferComponent>(e);
                var from = (Vector3)positions[i];

                for (int j = 0; j < buf.Length; j++)
                {
                    Entity nb = buf[j].Agent;
                    if (!m_em.Exists(nb) || !m_em.HasComponent<LocalTransform>(nb)) continue;
                    m_neighbourLines.Add((from, (Vector3)m_em.GetComponentData<LocalTransform>(nb).Position));
                }
            }
        }

        /// <summary>
        /// 在节点包围盒的截面位置绘制分割平面线框矩形。
        /// </summary>
        static void DrawSplitPlane(Vector3 bMin, Vector3 bMax, int axis, float coord)
        {
            Vector3 p0, p1, p2, p3;
            if (axis == 0) // X 轴：YZ 截面
            {
                p0 = new Vector3(coord, bMin.y, bMin.z);
                p1 = new Vector3(coord, bMax.y, bMin.z);
                p2 = new Vector3(coord, bMax.y, bMax.z);
                p3 = new Vector3(coord, bMin.y, bMax.z);
            }
            else if (axis == 1) // Y 轴：XZ 截面
            {
                p0 = new Vector3(bMin.x, coord, bMin.z);
                p1 = new Vector3(bMax.x, coord, bMin.z);
                p2 = new Vector3(bMax.x, coord, bMax.z);
                p3 = new Vector3(bMin.x, coord, bMax.z);
            }
            else // Z 轴：XY 截面
            {
                p0 = new Vector3(bMin.x, bMin.y, coord);
                p1 = new Vector3(bMax.x, bMin.y, coord);
                p2 = new Vector3(bMax.x, bMax.y, coord);
                p3 = new Vector3(bMin.x, bMax.y, coord);
            }

            Gizmos.DrawLine(p0, p1);
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawLine(p2, p3);
            Gizmos.DrawLine(p3, p0);
        }
    }
}
