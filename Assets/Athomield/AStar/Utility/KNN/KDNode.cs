
namespace KNN.Internal {
	/// <summary>
	/// KD 树中单个节点的数据结构。
	///
	/// KD 树（K-Dimensional Tree）是一种用于空间划分的二叉树，
	/// 广泛应用于 K 近邻（KNN）查询、碰撞检测、空间索引等场景。
	///
	/// 树的每个节点负责管理一段点的索引区间 [Start, End)，
	/// 并记录该区间内所有点的包围盒以及将空间一分为二的分割平面信息。
	///
	/// 节点分为两类：
	/// - 内部节点（Internal Node）：PartitionAxis != -1，有左右子节点，不直接存储点。
	/// - 叶节点（Leaf Node）：PartitionAxis == -1，直接包含少量点，不再继续分割。
	/// </summary>
	public struct KdNode {
		/// <summary>
		/// 该节点所覆盖空间区域的轴对齐包围盒（AABB）。
		/// 包含此节点所有关联点的最小外接长方体。
		/// 查询时先检测查询点与包围盒的距离，实现快速剪枝。
		/// </summary>
		public KdNodeBounds Bounds;

		/// <summary>
		/// 该节点管理的点索引区间的起始位置（在排列数组 m_permutation 中）。
		/// 区间为左闭右开：[Start, End)
		/// </summary>
		public int Start;

		/// <summary>
		/// 该节点管理的点索引区间的结束位置（不含）。
		/// 区间为左闭右开：[Start, End)
		/// </summary>
		public int End;

		/// <summary>
		/// 分割平面所在的坐标轴：0 = X 轴，1 = Y 轴，2 = Z 轴，-1 = 叶节点（不分割）。
		/// 构建时选择包围盒最长轴作为分割轴，以使树尽量平衡。
		/// </summary>
		public int PartitionAxis;

		/// <summary>
		/// 分割平面在 PartitionAxis 轴上的坐标值。
		/// 坐标 < PartitionCoordinate 的点归入负子节点（NegativeChild），
		/// 坐标 >= PartitionCoordinate 的点归入正子节点（PositiveChild）。
		/// </summary>
		public float PartitionCoordinate;

		/// <summary>
		/// 负子节点（左子树）在 m_nodes 列表中的索引。
		/// 负子节点包含坐标值 < PartitionCoordinate 的点。
		/// 叶节点中此值为 -1（无子节点）。
		/// </summary>
		public int NegativeChildIndex;

		/// <summary>
		/// 正子节点（右子树）在 m_nodes 列表中的索引。
		/// 正子节点包含坐标值 >= PartitionCoordinate 的点。
		/// 叶节点中此值为 -1（无子节点）。
		/// </summary>
		public int PositiveChildIndex;

		/// <summary>
		/// 该节点所管理的点的数量，即 End - Start。
		/// </summary>
		public int Count => End - Start;

		/// <summary>
		/// 判断该节点是否为叶节点。
		/// 叶节点的 PartitionAxis == -1，表示不再向下分割。
		/// 叶节点直接遍历其包含的点来寻找最近邻。
		/// </summary>
		public bool Leaf => PartitionAxis == -1;
	}
}
