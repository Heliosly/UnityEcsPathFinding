
using Unity.Mathematics;

namespace KNN.Internal {
    /// <summary>
    /// KNN 查询过程中，最小堆（MinHeap）中存放的待访问节点信息。
    ///
    /// 在 KNN 查询时，我们使用一个最小优先队列（MinHeap）来按照"最可能包含近邻"
    /// 的顺序逐步访问 KD 树的节点。QueryNode 就是这个优先队列中的元素。
    ///
    /// 查询算法流程：
    /// 1. 从根节点开始，将根节点作为 QueryNode 推入 MinHeap。
    /// 2. 每次从 MinHeap 弹出距离最小的节点。
    /// 3. 若该节点距离已超过当前已找到的最远近邻，跳过（剪枝）。
    /// 4. 若是叶节点，遍历其中所有点，更新近邻候选列表。
    /// 5. 若是内部节点，将两个子节点（带各自的最近点估算）推入 MinHeap。
    /// 6. 重复直到 MinHeap 为空。
    /// </summary>
    public struct QueryNode {
        /// <summary>
        /// 该条目对应的 KD 树节点在 m_nodes 列表中的索引。
        /// 查询时通过此索引取出对应的 KdNode 进行处理。
        /// </summary>
        public int NodeIndex;

        /// <summary>
        /// 查询点到该 KD 树节点包围盒的最近点（在包围盒表面或内部）。
        ///
        /// 这个点是从父节点的最近点"投影"到子节点边界所得到的，
        /// 避免了重新计算 clamp，是一种增量式的优化。
        ///
        /// 在遍历子节点时，将此点在分割轴上的坐标修改为分割坐标，
        /// 即可得到另一侧子节点的最近点估算。
        /// </summary>
        public float3 TempClosestPoint;

        /// <summary>
        /// 查询点到 TempClosestPoint 的平方距离（Squared Distance）。
        ///
        /// 使用平方距离而非真实距离，避免开方运算，提升性能。
        /// 作为 MinHeap 的排序键，距离越小越优先被访问。
        ///
        /// 当此距离已大于当前已知的最远近邻距离时，该节点可被安全剪枝。
        /// </summary>
        public float Distance;
    }
}
