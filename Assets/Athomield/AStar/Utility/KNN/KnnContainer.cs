
using System;
using KNN.Internal;
using KNN.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

// ─────────────────────────────────────────────────────────────────────────────
// 辅助工具：非托管内存分配
// ─────────────────────────────────────────────────────────────────────────────
namespace KNN.Internal {
	/// <summary>
	/// 非托管内存分配辅助类，对 Unity UnsafeUtility.Malloc 进行类型安全封装。
	/// 用于在 Burst/Job 兼容的非托管内存中分配泛型数组。
	/// </summary>
	public static unsafe class UnsafeUtilityEx {
		/// <summary>
		/// 分配指定长度的非托管类型数组，返回指向首元素的指针。
		///
		/// 等价于 C 中的 malloc，但具有类型安全和对齐信息。
		/// 调用方负责在合适时机通过 UnsafeUtility.Free 释放内存。
		/// </summary>
		/// <typeparam name="T">元素类型，必须是 unmanaged</typeparam>
		/// <param name="length">数组元素数量</param>
		/// <param name="allocator">内存分配器（Temp / TempJob / Persistent）</param>
		/// <returns>指向分配内存首元素的指针</returns>
		public static T* AllocArray<T>(int length, Allocator allocator) where T : unmanaged {
			return (T*)UnsafeUtility.Malloc(length * UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>(), allocator);
		}
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 核心容器：KD 树 + KNN 查询
// ─────────────────────────────────────────────────────────────────────────────
namespace KNN {
	/// <summary>
	/// KNN（K-Nearest Neighbors，K 近邻）查询容器，基于 KD 树实现。
	///
	/// 功能概述：
	/// - 接受一组 3D 点（float3[]），在内部构建 KD 树索引。
	/// - 提供两种查询接口：
	///   1. QueryKNearest：查询距给定位置最近的 K 个点的索引。
	///   2. QueryRange：查询给定位置半径范围内所有点的索引。
	///
	/// KD 树原理（简述）：
	/// - KD 树是将 K 维空间递归地用超平面（在3D中是轴对齐平面）分割成两个子空间的二叉树。
	/// - 构建时每次选最长轴分割，使用"滑动中点"规则确定分割坐标。
	/// - 查询时用包围盒剪枝（距离 > 当前最远近邻距离的节点直接跳过），大幅减少比较次数。
	/// - 时间复杂度：构建 O(n log n)，查询平均 O(log n)，最坏 O(n)。
	///
	/// Unity Job System 兼容：
	/// - 实现了 NativeContainer 接口，支持在 Job 中只读访问。
	/// - 通过 AtomicSafetyHandle 在 Editor 模式下进行线程安全检查。
	/// - 底层使用 NativeArray/NativeList 等 Burst 可优化容器。
	/// </summary>
	[NativeContainerSupportsDeallocateOnJobCompletion, NativeContainer, System.Diagnostics.DebuggerDisplay("Length = {Points.Length}")]
	public struct KnnContainer : IDisposable {
		// We manage safety by our own sentinel. Disable unity's safety system for internal caches / arrays
		/// <summary>
		/// 输入的原始 3D 点集合（公开只读）。
		/// 所有查询结果返回的是点在此数组中的原始索引。
		/// 禁用了 Unity 的容器安全限制，因为我们自己管理安全句柄。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		public NativeArray<float3> Points;

		/// <summary>
		/// 排列数组（Permutation Array），长度与 Points 相同。
		/// 存储 Points 中各点的间接索引，KD 树构建时通过重排此数组来组织点的顺序，
		/// 避免实际移动 Points 数组中的元素。
		/// m_permutation[i] 表示当前逻辑第 i 个点在 Points 中的原始索引。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		NativeArray<int> m_permutation;

		/// <summary>
		/// KD 树节点列表。
		/// 构建树时动态添加节点，根节点在 m_rootNodeIndex[0] 处。
		/// 使用 NativeList 以支持动态添加（构建时节点数量未知）。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		NativeList<KdNode> m_nodes;

		/// <summary>
		/// 根节点在 m_nodes 中的索引（长度为1的数组，用于在 Job 中可访问的间接引用）。
		/// 初始值为 -1（未构建），构建后为 0。
		/// 使用数组而非直接 int 是因为 struct 值语义在 Job 中的限制。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		NativeArray<int> m_rootNodeIndex;

		/// <summary>
		/// 构建队列（广度优先），用于迭代构建 KD 树（代替递归，避免栈溢出且 Burst 更友好）。
		/// 存储待分割的节点索引，每次取出一个节点进行分割，将新的子节点加入队列。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		NativeQueue<int> m_buildQueue;

		/// <summary>
		/// 获取根节点数据。
		/// </summary>
		KdNode RootNode => m_nodes[m_rootNodeIndex[0]];

#if ENABLE_UNITY_COLLECTIONS_CHECKS
		// Note: MUST be named m_Safey, m_DisposeSentinel exactly
		// ReSharper disable once InconsistentNaming
		/// <summary>
		/// Unity 的原子安全句柄，用于在 Editor 下检测多线程读写冲突。
		/// 字段名必须精确为 m_Safety（Unity 反射要求）。
		/// </summary>
		internal AtomicSafetyHandle m_Safety;
		[NativeSetClassTypeToNullOnSchedule]
		// ReSharper disable once InconsistentNaming
		/// <summary>
		/// Unity 的 Dispose 哨兵，确保容器在 Job 完成后被正确释放。
		/// 字段名必须精确为 m_DisposeSentinel（Unity 反射要求）。
		/// </summary>
		internal DisposeSentinel m_DisposeSentinel;
#endif

		/// <summary>
		/// 每个叶节点最多包含的点数量。
		/// 当一个节点的点数 <= 此值时，不再继续分割（成为叶节点）。
		/// 值越小：树更深，查询精确剪枝更多，但节点开销更大。
		/// 值越大：树更浅，叶节点遍历点数更多，但节点开销更小。
		/// 64 是经验上的较优值。
		/// </summary>
		const int c_maxPointsPerLeafNode = 64;

		/// <summary>
		/// KNN 单次查询的临时工作内存，包含两个堆。
		///
		/// 每次查询（QueryKNearest / QueryRange）都会创建一个此结构，查询完毕后立即释放。
		/// 使用 Allocator.Temp 分配，生命周期在一帧内。
		/// </summary>
		public struct KnnQueryTemp : IDisposable {
			/// <summary>
			/// 最大堆：维护当前已找到的 K 个最近邻候选。
			/// 堆顶是候选中距离最大的点，用于快速判断新点是否应替换当前最远候选。
			/// 容量固定为 K（查询的近邻数量）。
			/// </summary>
			public MinMaxHeap<int> MaxHeap;

			/// <summary>
			/// 最小堆：KD 树遍历的优先队列，按"最近可能距离"排序待访问节点。
			/// 堆顶是最可能包含近邻的节点（距离最小），优先访问它。
			///
			/// 容量固定为 64，对应树的最大深度约为 log_64(n)，
			/// 在任意时刻堆中最多有左右子节点各一个路径的节点，即树深 * 2。
			/// 假设树最深 32 层（可处理约 2^39 个节点），则最多 64 个节点在堆中。
			/// </summary>
			public MinMaxHeap<QueryNode> MinHeap;

			/// <summary>
			/// 创建查询临时内存。
			/// </summary>
			/// <param name="kCapacity">要查找的近邻数量 K（MaxHeap 的容量）</param>
			/// <returns>初始化好的 KnnQueryTemp</returns>
			public static KnnQueryTemp Create(int kCapacity) {
				KnnQueryTemp temp;
				temp.MaxHeap = new MinMaxHeap<int>(kCapacity, Allocator.Temp);

				// Min heap keeps track of current stack.
				// The max stack depth is the tree depth
				// The tree depth is log_c(nodes)
				// Let's assume people have a tree at most 32 deep (which equals 2^32 * c_maxPointsPerLeafNode ~ 2^39 nodes)
				// There are left/right nodes -> 64 max on stack at any given time
				temp.MinHeap = new MinMaxHeap<QueryNode>(64, Allocator.Temp);
				return temp;
			}

			/// <summary>
			/// 将一个 KD 树节点推入最小堆（待访问节点优先队列）。
			///
			/// 计算查询点到该节点包围盒最近点的平方距离，作为优先级。
			/// 距离越小，优先级越高（越早被访问）。
			/// </summary>
			/// <param name="index">节点在 m_nodes 中的索引</param>
			/// <param name="closestPoint">查询点到该节点包围盒的最近点（已由调用方计算好）</param>
			/// <param name="queryPosition">查询点的位置</param>
			public void PushQueryNode(int index, float3 closestPoint, float3 queryPosition) {
				float lengthsq = math.lengthsq(closestPoint - queryPosition);

				MinHeap.PushObjMin(new QueryNode {
					NodeIndex = index,
					TempClosestPoint = closestPoint,
					Distance = lengthsq
				}, lengthsq);
			}

			/// <summary>
			/// 释放两个堆的非托管内存。
			/// </summary>
			public void Dispose() {
				MaxHeap.Dispose();
				MinHeap.Dispose();
			}
		}

		/// <summary>
		/// 构造函数，初始化 KNN 容器并可选地立即构建 KD 树。
		/// </summary>
		/// <param name="points">输入的 3D 点集合（容器持有此数组的引用，不复制数据）</param>
		/// <param name="buildNow">
		/// 若为 true，立即同步构建 KD 树（通过调度 KnnRebuildJob 并等待完成）。
		/// 若为 false，需要手动调用 Rebuild() 或调度 KnnRebuildJob。
		/// </param>
		/// <param name="allocator">内存分配器类型</param>
		public KnnContainer(NativeArray<float3> points, bool buildNow, Allocator allocator) {
			// 预估节点数量：每个叶节点最多 c_maxPointsPerLeafNode 个点，
			// 叶节点数约为 ceil(n / c_maxPointsPerLeafNode)，内部节点数与叶节点数相近，
			// 故总节点数约为叶节点数的 4 倍（保守估计以减少扩容次数）
			int nodeCountEstimate = 4 * (int) math.ceil(points.Length / (float) c_maxPointsPerLeafNode + 1) + 1;
			Points = points;

			// Both arrays are filled in as we go, so start with uninitialized mem
			// 节点列表，使用预估容量初始化，避免频繁扩容
			m_nodes = new NativeList<KdNode>(nodeCountEstimate, allocator);

			// Dumb way to create an int* essentially..
			// 排列数组初始化为未初始化内存（将在 Rebuild 中填充）
			m_permutation = new NativeArray<int>(points.Length, allocator, NativeArrayOptions.UninitializedMemory);
			m_rootNodeIndex = new NativeArray<int>(1, allocator);
			m_rootNodeIndex[0] = -1; // -1 表示尚未构建
			m_buildQueue = new NativeQueue<int>(allocator);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
			if (allocator <= Allocator.None) {
				throw new ArgumentException("Allocator must be Temp, TempJob or Persistent", nameof(allocator));
			}

			if (points.Length <= 0) {
				throw new ArgumentOutOfRangeException(nameof(points), "Input points length must be >= 0");
			}

			DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, 0, allocator);
#endif

			if (buildNow) {
				// 立即同步构建：调度 Job 并阻塞等待完成
				var rebuild = new KnnRebuildJob(this);
				rebuild.Schedule().Complete();
			}
		}

		/// <summary>
		/// 重建 KD 树（当 Points 内容更新后需要调用此方法刷新索引）。
		///
		/// 构建流程（广度优先迭代）：
		/// 1. 清空旧节点列表。
		/// 2. 初始化排列数组为 [0, 1, 2, ..., n-1]。
		/// 3. 计算所有点的整体包围盒，创建根节点。
		/// 4. 将根节点加入队列，开始广度优先分割：
		///    a. 取出队列中的节点，对其进行分割（SplitNode）。
		///    b. 若子节点的点数 > c_maxPointsPerLeafNode，继续加入队列分割。
		///    c. 否则子节点成为叶节点，停止分割。
		/// 5. 队列为空时构建完成。
		/// </summary>
		public void Rebuild() {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			// 确保当前没有 Job 在读取此容器
			AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

			// 清空旧节点
			m_nodes.Clear();

			// 初始化排列数组：每个点初始时的"逻辑顺序"等于其原始索引
			for (int i = 0; i < m_permutation.Length; ++i) {
				m_permutation[i] = i;
			}

			// 创建根节点（包含所有点的包围盒，覆盖全部点 [0, Points.Length)）
			int rootNode = GetKdNode(MakeBounds(), 0, Points.Length);

			m_rootNodeIndex[0] = rootNode;
			m_buildQueue.Enqueue(rootNode);

			// 广度优先迭代分割
			while (m_buildQueue.Count > 0) {
				int index = m_buildQueue.Dequeue();
				// 分割当前节点，得到正负两个子节点的索引
				SplitNode(index, out int posNodeIndex, out int negNodeIndex);

				// 若负子节点的点数仍多于叶节点阈值，则继续分割正子节点
				// 注意：此处变量名 posNodeIndex/negNodeIndex 与下面的入队顺序需注意
				// negNodeIndex 对应分割后较小侧（负侧），posNodeIndex 对应较大侧（正侧）
				if (m_nodes[negNodeIndex].Count > c_maxPointsPerLeafNode) {
					m_buildQueue.Enqueue(posNodeIndex);
				}

				if (m_nodes[posNodeIndex].Count > c_maxPointsPerLeafNode) {
					m_buildQueue.Enqueue(negNodeIndex);
				}
			}
		}

		/// <summary>
		/// 内部节点列表的只读访问接口，供编辑器可视化工具遍历 KD 树结构。
		/// 必须在 Rebuild 完成后访问，否则列表为空。
		/// </summary>
		public NativeList<KdNode> InternalNodes => m_nodes;

		/// <summary>
		/// 根节点在节点列表中的索引。未调用 Rebuild 时为 -1。
		/// </summary>
		public int RootIndex => m_rootNodeIndex[0];

		/// <summary>
		/// 释放所有非托管资源（NativeArray、NativeList、NativeQueue）。
		/// 必须在使用完毕后调用，否则会造成内存泄漏。
		/// </summary>
		public void Dispose() {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif

			m_permutation.Dispose();
			m_nodes.Dispose();
			m_rootNodeIndex.Dispose();
			m_buildQueue.Dispose();
		}

		/// <summary>
		/// 创建一个新的 KdNode 并添加到 m_nodes 列表，返回其索引。
		///
		/// 初始创建的节点为叶节点（PartitionAxis = -1），
		/// 若后续对其调用 SplitNode，则会变为内部节点。
		/// </summary>
		/// <param name="bounds">节点的包围盒</param>
		/// <param name="start">节点管理的点区间起始（在 m_permutation 中）</param>
		/// <param name="end">节点管理的点区间结束（不含）</param>
		/// <returns>新节点在 m_nodes 中的索引</returns>
		int GetKdNode(KdNodeBounds bounds, int start, int end) {
			m_nodes.Add(new KdNode {
				Bounds = bounds,
				Start = start,
				End = end,
				PartitionAxis = -1,      // -1 表示叶节点（尚未分割）
				PartitionCoordinate = 0.0f,
				PositiveChildIndex = -1,
				NegativeChildIndex = -1
			});

			return m_nodes.Length - 1;
		}

		/// <summary>
		/// 计算所有点的整体轴对齐包围盒（用于根节点）。
		///
		/// 优化策略：使用"成对比较"技巧，每次处理两个点，
		/// 先比较这两个点之间的大小，再分别与 min/max 比较，
		/// 将比较次数从 2n 降低到约 3n/2（节省约 25% 的比较次数）。
		/// </summary>
		/// <returns>包含所有点的最小包围盒</returns>
		/// <summary>
		/// For calculating root node bounds
		/// </summary>
		/// <returns>Boundary of all Vector3 points</returns>
		KdNodeBounds MakeBounds() {
			var max = new float3(float.MinValue, float.MinValue, float.MinValue);
			var min = new float3(float.MaxValue, float.MaxValue, float.MaxValue);
			int even = Points.Length & ~1; // 取偶数部分，若为奇数则忽略最后一个（后面单独处理）

			// min, max calculations
			// 3n/2 calculations instead of 2n
			// 成对处理点，每次取 i0 和 i1 = i0+1 两个点进行比较
			for (int i0 = 0; i0 < even; i0 += 2) {
				int i1 = i0 + 1;

				// X Coords
				// 先比较 i0 和 i1 的 x 坐标大小，决定谁可能是更小值/更大值
				if (Points[i0].x > Points[i1].x) {
					// i0 是较大的，i1 是较小的
					if (Points[i1].x < min.x) {
						min.x = Points[i1].x;
					}

					if (Points[i0].x > max.x) {
						max.x = Points[i0].x;
					}
				} else {
					// i1 是较大的，i0 是较小的（或相等）
					if (Points[i0].x < min.x) {
						min.x = Points[i0].x;
					}

					if (Points[i1].x > max.x) {
						max.x = Points[i1].x;
					}
				}

				// Y Coords
				if (Points[i0].y > Points[i1].y) {
					// i0 is bigger, i1 is smaller
					if (Points[i1].y < min.y) {
						min.y = Points[i1].y;
					}

					if (Points[i0].y > max.y) {
						max.y = Points[i0].y;
					}
				} else {
					// i1 is smaller, i0 is bigger
					if (Points[i0].y < min.y) {
						min.y = Points[i0].y;
					}

					if (Points[i1].y > max.y) {
						max.y = Points[i1].y;
					}
				}

				// Z Coords
				if (Points[i0].z > Points[i1].z) {
					// i0 is bigger, i1 is smaller
					if (Points[i1].z < min.z) {
						min.z = Points[i1].z;
					}

					if (Points[i0].z > max.z) {
						max.z = Points[i0].z;
					}
				} else {
					// i1 is smaller, i0 is bigger
					if (Points[i0].z < min.z) {
						min.z = Points[i0].z;
					}

					if (Points[i1].z > max.z) {
						max.z = Points[i1].z;
					}
				}
			}

			// if array was odd, calculate also min/max for the last element
			// 若点总数为奇数，单独处理最后一个点
			if (even != Points.Length) {
				// X
				if (min.x > Points[even].x) {
					min.x = Points[even].x;
				}

				if (max.x < Points[even].x) {
					max.x = Points[even].x;
				}

				// Y
				if (min.y > Points[even].y) {
					min.y = Points[even].y;
				}

				if (max.y < Points[even].y) {
					max.y = Points[even].y;
				}

				// Z
				if (min.z > Points[even].z) {
					min.z = Points[even].z;
				}

				if (max.z < Points[even].z) {
					max.z = Points[even].z;
				}
			}

			var b = new KdNodeBounds();
			b.Min = min;
			b.Max = max;
			return b;
		}

		// TODO: When multiple points overlap exactly this function breaks.
		/// <summary>
		/// 将一个节点分割为正负两个子节点（KD 树构建的核心步骤）。
		///
		/// 分割策略：
		/// 1. 选择包围盒最长的轴作为分割轴（使树尽量平衡）。
		/// 2. 使用"滑动中点"算法计算分割坐标（CalculatePivot）。
		/// 3. 使用类似 Hoare 划分的算法重排排列数组（Partition）。
		/// 4. 创建负子节点（坐标 < 分割值的点）和正子节点（坐标 >= 分割值的点）。
		/// 5. 更新父节点的分割信息和子节点引用。
		///
		/// 注意：当多个点完全重叠时，此函数可能产生无限循环（TODO 标记）。
		/// </summary>
		/// <param name="parentIndex">待分割节点在 m_nodes 中的索引</param>
		/// <param name="posNodeIndex">输出：正子节点（坐标 >= 分割值）的索引</param>
		/// <param name="negNodeIndex">输出：负子节点（坐标 < 分割值）的索引</param>
		/// Recursive splitting procedure
		void SplitNode(int parentIndex, out int posNodeIndex, out int negNodeIndex) {
			KdNode parent = m_nodes[parentIndex];

			// center of bounding box
			KdNodeBounds parentBounds = parent.Bounds;
			float3 parentBoundsSize = parentBounds.Size;

			// Find axis where bounds are largest
			// 选择包围盒中最长的轴作为分割轴（0=X, 1=Y, 2=Z）
			int splitAxis = 0;
			float axisSize = parentBoundsSize.x;

			if (axisSize < parentBoundsSize.y) {
				splitAxis = 1;
				axisSize = parentBoundsSize.y;
			}

			if (axisSize < parentBoundsSize.z) {
				splitAxis = 2;
			}

			// Our axis min-max bounds
			float boundsStart = parentBounds.Min[splitAxis];
			float boundsEnd = parentBounds.Max[splitAxis];

			// Calculate the spiting coords
			// 使用滑动中点算法计算最优分割坐标
			float splitPivot = CalculatePivot(parent.Start, parent.End, boundsStart, boundsEnd, splitAxis);

			// 'Spiting' array to two sub arrays
			// 用 Hoare 划分重排 m_permutation，返回分割点索引
			// [parent.Start, splittingIndex) 中的点坐标 < splitPivot（负侧）
			// [splittingIndex, parent.End)   中的点坐标 >= splitPivot（正侧）
			int splittingIndex = Partition(parent.Start, parent.End, splitPivot, splitAxis);

			// Negative / Left node
			// 负子节点：包围盒的最大坐标在分割轴上被截断为 splitPivot
			float3 negMax = parentBounds.Max;
			negMax[splitAxis] = splitPivot;

			var bounds = parentBounds;
			bounds.Max = negMax;
			negNodeIndex = GetKdNode(bounds, parent.Start, splittingIndex);

			// 更新父节点的分割信息（此后父节点变为内部节点）
			parent.PartitionAxis = splitAxis;
			parent.PartitionCoordinate = splitPivot;

			// Positive / Right node
			// 正子节点：包围盒的最小坐标在分割轴上被截断为 splitPivot
			float3 posMin = parentBounds.Min;
			posMin[splitAxis] = splitPivot;

			bounds = parentBounds;
			bounds.Min = posMin;
			posNodeIndex = GetKdNode(bounds, splittingIndex, parent.End);

			parent.NegativeChildIndex = negNodeIndex;
			parent.PositiveChildIndex = posNodeIndex;

			// Write back node to array to update those values
			// 注意：KdNode 是 struct，必须写回到列表中才能保存修改
			m_nodes[parentIndex] = parent;
		}

		/// <summary>
		/// 滑动中点（Sliding Midpoint）分割坐标计算算法。
		///
		/// 算法步骤：
		/// 1. 首先尝试使用边界中点（midPoint = (boundsStart + boundsEnd) / 2）作为分割点。
		/// 2. 遍历区间内所有点，检查是否在中点两侧都有点：
		///    - 若两侧都有点，直接返回中点（最理想情况，树较平衡）。
		/// 3. 若所有点都在中点的某一侧（点集偏向一边）：
		///    - 若都在左侧（negative），返回左侧点的最大值（negMax），
		///      使分割线紧贴最右边的点，右侧变成空节点。
		///    - 若都在右侧（positive），返回右侧点的最小值（posMin），
		///      使分割线紧贴最左边的点，左侧变成空节点。
		///
		/// 这样可以避免在点集高度不均匀时产生空节点占据过多空间，
		/// 并保证每次分割都至少有一侧是非空的。
		/// </summary>
		/// <param name="start">点区间起始（在 m_permutation 中）</param>
		/// <param name="end">点区间结束（不含）</param>
		/// <param name="boundsStart">分割轴的最小值（包围盒 Min 在该轴上的值）</param>
		/// <param name="boundsEnd">分割轴的最大值（包围盒 Max 在该轴上的值）</param>
		/// <param name="axis">分割轴（0=X, 1=Y, 2=Z）</param>
		/// <returns>最优分割坐标</returns>
		float CalculatePivot(int start, int end, float boundsStart, float boundsEnd, int axis) {
			//! sliding midpoint rule
			float midPoint = (boundsStart + boundsEnd) / 2.0f;

			bool negative = false; // 是否有点在中点左侧（< midPoint）
			bool positive = false; // 是否有点在中点右侧（>= midPoint）

			float negMax = float.MinValue; // 左侧点中的最大坐标值
			float posMin = float.MaxValue; // 右侧点中的最小坐标值

			// this for loop section is used both for sorted and unsorted data
			// 第一次遍历：检测两侧是否都有点
			for (int i = start; i < end; i++) {
				float val = Points[m_permutation[i]][axis];

				if (val < midPoint) {
					negative = true;
				} else {
					positive = true;
				}

				// 若两侧都有点，中点即为最优分割，提前返回
				if (negative && positive) {
					return midPoint;
				}
			}

			// 所有点都在左侧（negative）：返回左侧最大值，分割线紧贴最右边的点
			if (negative) {
				for (int i = start; i < end; i++) {
					float val = Points[m_permutation[i]][axis];

					if (negMax < val) {
						negMax = val;
					}
				}

				return negMax;
			}

			// 所有点都在右侧（positive）：返回右侧最小值，分割线紧贴最左边的点
			for (int i = start; i < end; i++) {
				float val = Points[m_permutation[i]][axis];

				if (posMin > val) {
					posMin = val;
				}
			}

			return posMin;
		}

		/// <summary>
		/// 类 Hoare 划分算法，将 m_permutation[start..end) 重排，
		/// 使得坐标 < partitionPivot 的点排在左边，>= partitionPivot 的点排在右边。
		///
		/// 算法（双指针）：
		/// - 左指针（lp）从 start-1 向右移动，跳过所有 < partitionPivot 的点（它们已在正确位置）。
		/// - 右指针（rp）从 end 向左移动，跳过所有 >= partitionPivot 的点（它们已在正确位置）。
		/// - 当两指针相遇时（lp >= rp），返回 lp 作为分割索引。
		/// - 在此之前，若找到一个"越界"的元素，交换 lp 和 rp 位置的元素，继续。
		///
		/// 时间复杂度：O(n)，原地操作（只修改 m_permutation，不移动 Points 数据）。
		/// </summary>
		/// <param name="start">区间起始（含）</param>
		/// <param name="end">区间结束（不含）</param>
		/// <param name="partitionPivot">分割值：左侧元素的轴坐标 < pivot，右侧 >= pivot</param>
		/// <param name="axis">比较使用的坐标轴（0=X, 1=Y, 2=Z）</param>
		/// <returns>
		/// 分割索引 lp，满足：
		/// m_permutation[start..lp) 对应的点坐标均 < partitionPivot（左/负侧）
		/// m_permutation[lp..end)   对应的点坐标均 >= partitionPivot（右/正侧）
		/// </returns>
		int Partition(int start, int end, float partitionPivot, int axis) {
			// note: increasing right pointer is actually decreasing!
			int lp = start - 1; // left pointer (negative side)，初始在区间左侧外
			int rp = end;        // right pointer (positive side)，初始在区间右侧外

			while (true) {
				do {
					// move from left to the right until "out of bounds" value is found
					// 向右移动，直到找到一个坐标 >= partitionPivot 的点（它不该在左侧）
					lp++;
				} while (lp < rp && Points[m_permutation[lp]][axis] < partitionPivot);

				do {
					// move from right to the left until "out of bounds" value found
					// 向左移动，直到找到一个坐标 < partitionPivot 的点（它不该在右侧）
					rp--;
				} while (lp < rp && Points[m_permutation[rp]][axis] >= partitionPivot);

				if (lp < rp) {
					// 两指针还未相遇，交换这两个"错位"的元素
					int temp = m_permutation[lp];
					m_permutation[lp] = m_permutation[rp];
					m_permutation[rp] = temp;
				} else {
					// 两指针相遇，划分完成，lp 即为分割索引
					return lp;
				}
			}
		}

		/// <summary>
		/// 范围查询：找出所有与查询点距离不超过 radius 的点，将其原始索引写入 result。
		///
		/// 算法（最优先搜索，Best-First Search）：
		/// 1. 初始化：将根节点（及到其包围盒的最近点）推入最小堆。
		/// 2. 循环从最小堆中弹出最近节点（距离最小的优先）：
		///    a. 若该节点到查询点的最短可能距离 > radius²，跳过（不可能包含近邻）。
		///    b. 若为叶节点：遍历其中所有点，将在半径内的点加入 MaxHeap（并在需要时扩容）。
		///    c. 若为内部节点：确定查询点在分割平面的哪一侧：
		///       - 先侧（查询点所在侧）直接推入堆（最近点不变）。
		///       - 远侧将最近点在分割轴上的坐标投影到分割平面，推入堆。
		/// 3. 最小堆清空后，从 MaxHeap 取出所有结果写入 result。
		///
		/// 注意：MaxHeap 会根据结果数量动态扩容（初始 32，满后翻倍）。
		/// </summary>
		/// <param name="queryPosition">查询点的位置</param>
		/// <param name="radius">查询半径（注意：内部使用平方距离比较，避免 sqrt）</param>
		/// <param name="result">输出：在半径范围内的点的原始索引列表</param>
		public void QueryRange(float3 queryPosition, float radius, NativeList<int> result) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

			// Start with a temp of some size. This will be resized dynamically
			// 初始容量 32，不够时动态翻倍（因为范围查询结果数量不定）
			var temp = KnnQueryTemp.Create(32);

			// Biggest Smallest Squared Radius（最大的"可接受"平方距离）
			float bssr = radius * radius;
			float3 rootClosestPoint = RootNode.Bounds.ClosestPoint(queryPosition);

			// 将根节点推入最小堆，开始搜索
			temp.PushQueryNode(m_rootNodeIndex[0], rootClosestPoint, queryPosition);

			while (temp.MinHeap.Count > 0) {
				QueryNode queryNode = temp.MinHeap.PopObjMin();

				// 若节点到查询点的最短距离已超出范围，跳过（剪枝）
				if (queryNode.Distance > bssr) {
					continue;
				}

				KdNode node = m_nodes[queryNode.NodeIndex];

				if (!node.Leaf) {
					// 内部节点：确定查询点在哪一侧，优先访问同侧子节点
					int partitionAxis = node.PartitionAxis;
					float partitionCoord = node.PartitionCoordinate;
					float3 tempClosestPoint = queryNode.TempClosestPoint;

					if (tempClosestPoint[partitionAxis] - partitionCoord < 0) {
						// we already know we are on the side of negative bound/node,
						// so we don't need to test for distance
						// push to stack for later querying
						// 查询点在负侧（tempClosestPoint 在分割平面左侧），先访问负子节点
						temp.PushQueryNode(node.NegativeChildIndex, tempClosestPoint, queryPosition);

						// project the tempClosestPoint to other bound
						// 将最近点投影到分割平面（改变分割轴坐标），用于估算远侧（正子节点）的最短距离
						tempClosestPoint[partitionAxis] = partitionCoord;

						if (node.Count != 0) {
							temp.PushQueryNode(node.PositiveChildIndex, tempClosestPoint, queryPosition);
						}
					}
					else {
						// we already know we are on the side of positive bound/node,
						// so we don't need to test for distance
						// push to stack for later querying
						// 查询点在正侧，先访问正子节点
						temp.PushQueryNode(node.PositiveChildIndex, tempClosestPoint, queryPosition);

						// project the tempClosestPoint to other bound
						tempClosestPoint[partitionAxis] = partitionCoord;

						if (node.Count != 0) {
							temp.PushQueryNode(node.NegativeChildIndex, tempClosestPoint, queryPosition);
						}
					}
				} else {
					// 叶节点：遍历其中所有点，筛选在半径内的点
					for (int i = node.Start; i < node.End; i++) {
						int index = m_permutation[i];
						float sqrDist = math.lengthsq(Points[index] - queryPosition);

						if (sqrDist <= bssr) {
							// Unlike the k-query we want to keep _all_ objects in range
							// So resize the heap when pushing this node
							// 范围查询需要保留所有满足条件的点，不限数量
							if (temp.MaxHeap.IsFull) {
								// 容量不足时翻倍扩容
								temp.MaxHeap.Resize(temp.MaxHeap.Count * 2);
							}

							temp.MaxHeap.PushObjMax(index, sqrDist);
						}
					}
				}
			}

			// 从 MaxHeap 中取出所有结果（顺序不保证）
			while (temp.MaxHeap.Count > 0) {
				result.Add(temp.MaxHeap.PopObjMax());
			}

			temp.Dispose();
		}

		/// <summary>
		/// K 近邻查询：找出距查询点最近的 K 个点，将其原始索引写入 result（result.Length = K）。
		///
		/// 与 QueryRange 相似，但：
		/// - MaxHeap 容量固定为 K，不扩容。
		/// - 通过动态更新 bssr（当前已找到的最远近邻距离），逐步收紧剪枝条件：
		///   初始为正无穷，每当 MaxHeap 满后，将 bssr 更新为堆顶值（当前最远近邻距离）。
		///   这样后续距离更大的节点会被更快剪枝，效率更高。
		///
		/// 结果写入顺序：从堆中弹出，即从距离最大到最小（可通过 result 内容确认）。
		/// 调用方可对结果排序，或使用 result 索引访问 Points。
		/// </summary>
		/// <param name="queryPosition">查询点的位置</param>
		/// <param name="result">
		/// 输出缓冲区，长度决定 K 值（即要查找多少个近邻）。
		/// 调用方需预分配此 NativeSlice 且长度 >= 1。
		/// </param>
		public void QueryKNearest(float3 queryPosition, NativeSlice<int> result) {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
			AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

			var temp = KnnQueryTemp.Create(result.Length);
			int k = result.Length;

			// Biggest Smallest Squared Radius（当前已知最远近邻的平方距离）
			// 初始为正无穷（尚未找到任何近邻时，不做距离剪枝）
			float bssr = float.PositiveInfinity;
			float3 rootClosestPoint = RootNode.Bounds.ClosestPoint(queryPosition);

			temp.PushQueryNode(m_rootNodeIndex[0], rootClosestPoint, queryPosition);

			while (temp.MinHeap.Count > 0) {
				QueryNode queryNode = temp.MinHeap.PopObjMin();

				// 若节点到查询点的最短距离 > 当前已知最远近邻，剪枝
				if (queryNode.Distance > bssr) {
					continue;
				}

				KdNode node = m_nodes[queryNode.NodeIndex];

				if (!node.Leaf) {
					// 内部节点处理（逻辑与 QueryRange 相同）
					int partitionAxis = node.PartitionAxis;
					float partitionCoord = node.PartitionCoordinate;
					float3 tempClosestPoint = queryNode.TempClosestPoint;

					if (tempClosestPoint[partitionAxis] - partitionCoord < 0) {
						// we already know we are on the side of negative bound/node,
						// so we don't need to test for distance
						// push to stack for later querying
						temp.PushQueryNode(node.NegativeChildIndex, tempClosestPoint, queryPosition);

						// project the tempClosestPoint to other bound
						tempClosestPoint[partitionAxis] = partitionCoord;

						if (node.Count != 0) {
							temp.PushQueryNode(node.PositiveChildIndex, tempClosestPoint, queryPosition);
						}
					} else {
						// we already know we are on the side of positive bound/node,
						// so we don't need to test for distance
						// push to stack for later querying
						temp.PushQueryNode(node.PositiveChildIndex, tempClosestPoint, queryPosition);

						// project the tempClosestPoint to other bound
						tempClosestPoint[partitionAxis] = partitionCoord;

						if (node.Count != 0) {
							temp.PushQueryNode(node.NegativeChildIndex, tempClosestPoint, queryPosition);
						}
					}
				} else {
					// 叶节点：遍历所有点，更新近邻候选
					for (int i = node.Start; i < node.End; i++) {
						int index = m_permutation[i];
						float sqrDist = math.lengthsq(Points[index] - queryPosition);

						if (sqrDist <= bssr) {
							// 新点距离在当前范围内，尝试加入 MaxHeap
							temp.MaxHeap.PushObjMax(index, sqrDist);

							// 当 MaxHeap 已满（找到了 K 个候选），更新剪枝半径
							// bssr = MaxHeap 堆顶值 = 当前 K 个候选中距离最大的
							if (temp.MaxHeap.Count == k) {
								bssr = temp.MaxHeap.HeadValue;
							}
						}
					}
				}
			}

			// 将 K 个近邻索引从 MaxHeap 写入结果（顺序从远到近）
			for (int i = 0; i < k; i++) {
				result[i] = temp.MaxHeap.PopObjMax();
			}

			temp.Dispose();
		}
	}
}
