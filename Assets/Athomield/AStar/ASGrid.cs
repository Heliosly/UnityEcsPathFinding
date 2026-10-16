using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System;
using UnityEngine;

namespace Athomield.AStar
{
    /// <summary>
    /// 不同岛屿（Island）间的寻路策略枚举。
    /// 岛屿是指由不可通行节点隔开的相互独立的可通行区域
    /// </summary>
    public enum DifferentIslandsPolicy
    {
        /// <summary>起点与终点不在同一岛屿时，不进行寻路计算（直接放弃）</summary>
        DontPath,

        /// <summary>
        /// 起点与终点不在同一岛屿时，寻路到起点所在岛屿上距目标最近的节点。
        /// 警告：在代理数量多、网格密集时性能消耗较大，不推荐使用
        /// </summary>
        GoToClosest
    }

    /// <summary>
    /// 寻路网格单例组件，描述整个导航网格的全局参数。
    /// 整个场景只有一个 Grid Entity 拥有此组件
    /// </summary>
    public struct ASGrid : IComponentData
    {
        /// <summary>网格总宽度（世界单位）</summary>
        public float Width;

        /// <summary>网格总高度（世界单位）</summary>
        public float Height;

        /// <summary>X 方向节点数量 = Width / NodeSize（四舍五入）</summary>
        public int NodeXCount;

        /// <summary>Y 方向节点数量 = Height / NodeSize（四舍五入）</summary>
        public int NodeYCount;

        /// <summary>
        /// 不可通行层的 LayerMask 值（int 类型）。
        /// 此层上的碰撞体会将对应节点标记为 IsAccessible = false
        /// </summary>
        public int InaccessibleLMIndex;

        /// <summary>
        /// 惩罚层的 LayerMask 值（int 类型）。
        /// 此层上的碰撞体会给节点增加 Penalty，使寻路算法尽量绕开该区域
        /// </summary>
        public int PenaltyLMIndex;

        /// <summary>每个节点的边长（世界单位），决定网格精度</summary>
        public float NodeSize;

        /// <summary>
        /// 当起点和终点分属不同岛屿时的处理策略。
        /// 详见 DifferentIslandsPolicy 枚举
        /// </summary>
        public DifferentIslandsPolicy DifferentIslandsPolicy;

        /// <summary>
        /// 目标节点不可通行时，BFS 搜索最近可通行节点的范围上限（节点数）。
        /// 推荐值：10-20，随网格密度增大而适当增大
        /// </summary>
        public int TargetInaccessibleSearchTolerance;

        /// <summary>
        /// 每个代理在 RVO 避障时最多考虑的邻居数量。
        /// 推荐值：4-10，值越大避障越精确但性能消耗越高
        /// </summary>
        public int AvoidanceNeighbours;

        /// <summary>网格刷新后是否让所有代理重新规划路径</summary>
        public bool RecalcPathsAfterGridRefresh;

        /// <summary>
        /// 网格刷新触发标志。添加或移动障碍物/惩罚区域后，
        /// 将此值设为 true 以重新扫描网格节点可通行性
        /// </summary>
        public bool RefreshGrid;

        /// <summary>
        /// 岛屿刷新触发标志。网格刷新后由系统自动置为 true，
        /// 触发 CheckNodeIslandSystem 重新计算连通区域
        /// </summary>
        public bool RefreshIslands;
    }

    /// <summary>
    /// 寻路性能参数组件，与 ASGrid 同挂载在 Grid Entity 上
    /// </summary>
    public struct ASPathfindingParameters : IComponentData
    {
        /// <summary>
        /// 每帧最多执行的寻路操作数量。
        /// 限制此值可以防止单帧内大量寻路请求导致卡顿（时间分片）
        /// </summary>
        public int MaxPathfindingOpsPerFrame;
    }

    /// <summary>
    /// 网格节点缓冲组件，以一维数组存储二维网格的所有节点。
    /// 索引计算：若 NodeXCount > NodeYCount，则 index = x + y * d；否则 index = y + x * d（d = max(NodeXCount, NodeYCount)）
    /// InternalBufferCapacity(0) 表示数据存储在堆内存中，避免影响 Chunk 布局
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct ASGridNodesBufferComponent : IBufferElementData
    {
        /// <summary>该位置的网格节点数据</summary>
        public ASNode Node;
    }

    /// <summary>
    /// 单个网格节点的数据结构，实现了 IHeapItem 接口以支持最小堆排序（A* open set）
    /// </summary>
    public struct ASNode : IEquatable<ASNode>, IHeapItem<ASNode>
    {
        /// <summary>节点是否可通行（false 表示被障碍物占据）</summary>
        public bool IsAccessible;

        /// <summary>节点在网格中的 X 坐标（列）</summary>
        public int X;

        /// <summary>节点在网格中的 Y 坐标（行）</summary>
        public int Y;

        /// <summary>
        /// G 代价：从起点到此节点的实际移动代价（含地形惩罚）。
        /// 直线移动代价=10，斜线移动代价=14
        /// </summary>
        public int GCost;

        /// <summary>
        /// H 代价：此节点到目标节点的启发式估计代价（曼哈顿/切比雪夫混合启发）。
        /// H = 14 * min(dx,dy) + 10 * abs(dx-dy)
        /// </summary>
        public int HCost;

        /// <summary>
        /// 地形惩罚值，由 CheckObstaclesSystem 高斯模糊后写入。
        /// 值越大，A* 算法越倾向于绕开该节点（但不是不可通行）
        /// </summary>
        public int Penalty;

        /// <summary>此节点在最小堆数组中的索引（由堆自动维护）</summary>
        public int HeapIndex;

        /// <summary>
        /// A* 回溯路径时使用：记录父节点在 Nodes 数组中的一维索引，
        /// 寻路完成后通过此链依次回溯直到起点
        /// </summary>
        public int PreviousNodeIndex;

        // 实现 IHeapItem 接口，使 NativeMinHeap 可以读写此节点的堆索引
        int IHeapItem<ASNode>.HeapIndex { get => HeapIndex; set => HeapIndex = value; }

        /// <summary>
        /// 堆排序比较函数：按 F 代价（G+H）升序排列，F 相同时按 H 升序。
        /// 返回负值表示 this 应排在 _other 前面（最小堆语义下优先级更高）
        /// </summary>
        public int CompareTo(ASNode _other)
        {
            int compare = GridUtility.NodeFCost(this).CompareTo(GridUtility.NodeFCost(_other));
            if (compare == 0)
            {
                compare = HCost.CompareTo(_other.HCost);
            }

            return -compare; // 取反使 F 最小的节点具有最高优先级
        }

        /// <summary>节点相等判断：坐标相同即视为同一节点</summary>
        public bool Equals(ASNode other)
        {
            return X == other.X &&
                   Y == other.Y;
        }

        /// <summary>调试输出：显示该节点的 F 代价（G+H）</summary>
        public override string ToString()
        {
            return (GCost + HCost).ToString();
        }

        /// <summary>
        /// 哈希值：将 X、Y 坐标编码为单个整数，用于 NativeHashSet 的快速查找。
        /// 通过位运算将 Y 的高低16位分别异或到不同位置，降低碰撞概率
        /// </summary>
        public override int GetHashCode()
        {
            return X ^ (Y << 16) ^ (Y >> 16);
        }
    }

    /// <summary>
    /// 岛屿 ID 缓冲组件，与 ASGridNodesBufferComponent 索引对应。
    /// 相同 IslandID 的节点属于同一连通区域（岛屿），
    /// 由 CheckNodeIslandSystem 通过 BFS/DFS 填充
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct ASNodeIslandIDBufferComponent : IBufferElementData
    {
        /// <summary>该节点所属岛屿的编号（从 0 开始递增）</summary>
        public int IslandID;
    }
}
