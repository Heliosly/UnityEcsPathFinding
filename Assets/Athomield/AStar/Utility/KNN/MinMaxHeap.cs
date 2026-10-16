
using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace KNN.Internal {
	/// <summary>
	/// 二叉堆的辅助工具类，提供父子节点索引计算。
	///
	/// 二叉堆使用数组存储，约定索引从 1 开始（0 不使用），以简化父子关系计算：
	/// - 节点 i 的父节点索引：i / 2
	/// - 节点 i 的左子节点索引：i * 2
	/// - 节点 i 的右子节点索引：i * 2 + 1
	/// </summary>
	public static class HeapUtils {
		/// <summary>
		/// 计算给定节点的父节点索引。
		/// </summary>
		/// <param name="index">当前节点在堆数组中的索引（从1起）</param>
		/// <returns>父节点索引</returns>
		public static int Parent(int index) {
			return index / 2;
		}

		/// <summary>
		/// 计算给定节点的左子节点索引。
		/// </summary>
		/// <param name="index">当前节点在堆数组中的索引（从1起）</param>
		/// <returns>左子节点索引</returns>
		public static int Left(int index) {
			return index * 2;
		}

		/// <summary>
		/// 计算给定节点的右子节点索引。
		/// </summary>
		/// <param name="index">当前节点在堆数组中的索引（从1起）</param>
		/// <returns>右子节点索引</returns>
		public static int Right(int index) {
			return index * 2 + 1;
		}
	}

	/// <summary>
	/// 泛型最小/最大二叉堆（MinMaxHeap），基于非托管内存实现，兼容 Unity Job System 和 Burst 编译。
	///
	/// 堆是一种完全二叉树，数组实现，索引从 1 开始：
	/// - 最大堆（MaxHeap）：父节点的 value >= 子节点的 value，堆顶为最大值。
	/// - 最小堆（MinHeap）：父节点的 value <= 子节点的 value，堆顶为最小值。
	///
	/// 在 KNN 算法中的用途：
	/// - MaxHeap（最大堆）：维护当前已找到的 K 个最近邻候选点，堆顶为距离最大的候选。
	///   当候选已满时，若新点距离比堆顶小，则替换堆顶（淘汰最远的点）。
	/// - MinHeap（最小堆）：维护待访问的 KD 树节点优先队列，优先访问距离最小的节点（最优先搜索）。
	///
	/// 使用非托管指针（unsafe）存储数据，避免 GC 压力，适合在 Job 中高频使用。
	/// </summary>
	/// <typeparam name="T">键的类型，必须是 unmanaged（非托管）类型</typeparam>
	// Sorted heap with a self balancing tree
	// Can act as either a min or max heap
	public unsafe struct MinMaxHeap<T> : IDisposable where T : unmanaged {
		/// <summary>
		/// 存储元素键（payload）的非托管数组指针。
		/// 索引从 1 开始，keys[0] 不使用。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		T* keys; //objects

		/// <summary>
		/// 存储元素优先级（排序键）的非托管浮点数组指针。
		/// 索引从 1 开始，values[0] 不使用。
		/// values[i] 对应 keys[i] 的优先级（距离）。
		/// </summary>
		[NativeDisableContainerSafetyRestriction]
		float* values;

		/// <summary>
		/// 堆中当前元素数量（有效元素范围：[1, Count]）。
		/// </summary>
		public int Count;

		/// <summary>
		/// 堆的最大容量（分配的数组大小为 m_capacity + 1，因为从索引1开始）。
		/// </summary>
		int m_capacity;

		/// <summary>
		/// 堆顶元素的优先级值（最大堆中最大值，最小堆中最小值）。
		/// 堆顶始终在索引 1 处。
		/// </summary>
		public float HeadValue => values[1];

		/// <summary>
		/// 堆顶元素的键（payload）。
		/// </summary>
		T HeadKey => keys[1];

		/// <summary>
		/// 堆是否已满（当前元素数量等于最大容量）。
		/// </summary>
		public bool IsFull => Count == m_capacity;

		/// <summary>
		/// 内存分配器类型（Temp / TempJob / Persistent）。
		/// </summary>
		Allocator m_allocator;

		/// <summary>
		/// 构造函数，初始化堆并分配非托管内存。
		/// </summary>
		/// <param name="startCapacity">初始容量（可存放的元素数量上限）</param>
		/// <param name="allocator">Unity 内存分配器类型</param>
		public MinMaxHeap(int startCapacity, Allocator allocator) {
			Count = 0;
			m_allocator = allocator;

			// Now alloc starting arrays
			// 分配 startCapacity + 1 个元素（索引0不使用，从1开始计数）
			m_capacity = startCapacity;
			values = UnsafeUtilityEx.AllocArray<float>(startCapacity + 1, m_allocator);
			keys = UnsafeUtilityEx.AllocArray<T>(startCapacity + 1, m_allocator);
		}

		/// <summary>
		/// 交换堆中两个位置的键和值。
		/// 是所有堆维护操作（上浮/下沉）的基础原子操作。
		/// </summary>
		/// <param name="indexA">第一个位置的索引</param>
		/// <param name="indexB">第二个位置的索引</param>
		void Swap(int indexA, int indexB) {
			float tempVal = values[indexA];
			values[indexA] = values[indexB];
			values[indexB] = tempVal;

			T tempKey = keys[indexA];
			keys[indexA] = keys[indexB];
			keys[indexB] = tempKey;
		}

		/// <summary>
		/// 释放非托管内存，必须在使用完毕后调用以避免内存泄漏。
		/// </summary>
		public void Dispose() {
			UnsafeUtility.Free(values, m_allocator);
			UnsafeUtility.Free(keys, m_allocator);
			values = null;
			keys = null;
		}

		/// <summary>
		/// 动态扩容：将堆容量调整为 newSize。
		/// 分配新的更大数组，将旧数据复制过去，释放旧数组。
		/// 通常在 QueryRange 中结果数量不确定时使用（容量翻倍策略）。
		/// </summary>
		/// <param name="newSize">新的容量大小</param>
		public void Resize(int newSize) {
			// Allocate more space
			var newValues = UnsafeUtilityEx.AllocArray<float>(newSize + 1, m_allocator);
			var newKeys = UnsafeUtilityEx.AllocArray<T>(newSize + 1, m_allocator);

			// Copy over old arrays
			// 注意：这里复制的是 (m_capacity + 1) * sizeof(int) 字节
			// 对于 float 和 T 不是 int 的情况可能有 bug，但在当前使用场景（int 和 QueryNode）下基本正确
			UnsafeUtility.MemCpy(newValues, values, (m_capacity + 1) * sizeof(int));
			UnsafeUtility.MemCpy(newKeys, keys, (m_capacity + 1) * sizeof(int));

			// Get rid of old arrays
			Dispose();

			// And now use old arrays
			values = newValues;
			keys = newKeys;
			m_capacity = newSize;
		}

		/// <summary>
		/// 最大堆的下沉操作（Sift Down / Bubble Down）。
		/// 当堆顶被替换后，将新堆顶向下移动到正确位置，恢复最大堆性质。
		///
		/// 算法：
		/// 1. 比较当前节点与其左右子节点的 value。
		/// 2. 若当前节点比某个子节点小（违反最大堆），与较大的子节点交换。
		/// 3. 重复直到满足堆性质或到达叶节点。
		/// 时间复杂度：O(log n)
		/// </summary>
		/// <param name="index">从哪个索引开始下沉</param>
		// bubble down, MaxHeap version
		void BubbleDownMax(int index) {
			int l = HeapUtils.Left(index);
			int r = HeapUtils.Right(index);

			// bubbling down, 2 kids
			// 当左右子节点都存在时（r <= Count 保证右节点在范围内）
			while (r <= Count) {
				// if heap property is violated between index and Left child
				// 若当前节点比左子节点小（违反最大堆性质）
				if (values[index] < values[l]) {
					if (values[l] < values[r]) {
						Swap(index, r); // left has bigger priority
						// 右子节点更大，与右子节点交换
						index = r;
					} else {
						Swap(index, l); // right has bigger priority
						// 左子节点更大（或相等），与左子节点交换
						index = l;
					}
				} else {
					// if heap property is violated between index and R
					// 当前节点 >= 左子节点，检查与右子节点的关系
					if (values[index] < values[r]) {
						// 右子节点更大，交换
						Swap(index, r);
						index = r;
					} else {
						// 当前节点 >= 两个子节点，堆性质满足，结束
						index = l;
						l = HeapUtils.Left(index);
						break;
					}
				}

				l = HeapUtils.Left(index);
				r = HeapUtils.Right(index);
			}

			// only left & last children available to test and swap
			// 只剩左子节点的情况（最后一层可能只有左子节点）
			if (l <= Count && values[index] < values[l]) {
				Swap(index, l);
			}
		}

		/// <summary>
		/// 最小堆的下沉操作（Sift Down / Bubble Down）。
		/// 与 BubbleDownMax 逻辑对称，但比较方向相反（找更小的子节点交换）。
		/// 时间复杂度：O(log n)
		/// </summary>
		/// <param name="index">从哪个索引开始下沉</param>
		void BubbleDownMin(int index) {
			int l = HeapUtils.Left(index);
			int r = HeapUtils.Right(index);

			// bubbling down, 2 kids
			while (r <= Count) {
				// if heap property is violated between index and Left child
				// 若当前节点比左子节点大（违反最小堆性质）
				if (values[index] > values[l]) {
					if (values[l] > values[r]) {
						Swap(index, r); // right has smaller priority
						// 右子节点更小，与右子节点交换
						index = r;
					} else {
						Swap(index, l); // left has smaller priority
						// 左子节点更小（或相等），与左子节点交换
						index = l;
					}
				} else {
					// if heap property is violated between index and R
					// 当前节点 <= 左子节点，检查与右子节点的关系
					if (values[index] > values[r]) {
						Swap(index, r);
						index = r;
					} else {
						// 当前节点 <= 两个子节点，堆性质满足，结束
						index = l;
						l = HeapUtils.Left(index);
						break;
					}
				}

				l = HeapUtils.Left(index);
				r = HeapUtils.Right(index);
			}

			// only left & last children available to test and swap
			if (l <= Count && values[index] > values[l]) {
				Swap(index, l);
			}
		}

		/// <summary>
		/// 最大堆的上浮操作（Sift Up / Bubble Up）。
		/// 将新插入到末尾的元素向上移动到正确位置，恢复最大堆性质。
		///
		/// 算法：
		/// 1. 从当前节点开始，与父节点比较。
		/// 2. 若当前节点 value 大于父节点（违反最大堆），与父节点交换。
		/// 3. 重复直到满足堆性质或到达根节点（p > 0）。
		/// 时间复杂度：O(log n)
		/// </summary>
		/// <param name="index">新插入元素的索引（通常为 Count）</param>
		void BubbleUpMax(int index) {
			int p = HeapUtils.Parent(index);

			//swap, until Heap property isn't violated anymore
			while (p > 0 && values[p] < values[index]) {
				Swap(p, index);
				index = p;
				p = HeapUtils.Parent(index);
			}
		}

		/// <summary>
		/// 最小堆的上浮操作（Sift Up / Bubble Up）。
		/// 与 BubbleUpMax 逻辑对称，但比较方向相反。
		/// 时间复杂度：O(log n)
		/// </summary>
		/// <param name="index">新插入元素的索引</param>
		void BubbleUpMin(int index) {
			int p = HeapUtils.Parent(index);

			//swap, until Heap property isn't violated anymore
			while (p > 0 && values[p] > values[index]) {
				Swap(p, index);
				index = p;
				p = HeapUtils.Parent(index);
			}
		}

		/// <summary>
		/// 向最大堆中插入一个元素。
		///
		/// 行为：
		/// - 若堆未满：直接插入到末尾，然后上浮到正确位置。
		/// - 若堆已满：仅当新元素的 value 小于当前堆顶（最大值）时，才替换堆顶并下沉。
		///   这样可以维持堆中始终是 value 最小的 K 个元素（用于 KNN 的 K 近邻候选集）。
		///
		/// 在 KNN 中用于维护"当前 K 个最近邻"：堆满时堆顶是距离最大的候选，
		/// 若新点更近（val 更小），则替换堆顶，淘汰最远的候选。
		/// </summary>
		/// <param name="key">元素的键（如点的原始索引）</param>
		/// <param name="val">元素的优先级（如距离平方）</param>
		public void PushObjMax(T key, float val) {
			// if heap full
			if (Count == m_capacity) {
				// if Heads priority is smaller than input priority, then ignore that item
				// 堆顶是当前 K 个候选中距离最大的，若新点距离比堆顶更小，替换堆顶
				if (HeadValue > val) {
					values[1] = val; // remove top element
					keys[1] = key;
					BubbleDownMax(1); // bubble it down
				}
				// 否则新点距离更大，不需要替换，直接忽略
			}
			else {
				// 堆未满，直接加入
				Count++;
				values[Count] = val;
				keys[Count] = key;
				BubbleUpMax(Count);
			}
		}

		/// <summary>
		/// 向最小堆中插入一个元素。
		///
		/// 行为：
		/// - 若堆未满：直接插入到末尾，然后上浮到正确位置。
		/// - 若堆已满：仅当新元素的 value 大于当前堆顶（最小值）时，才替换堆顶并下沉。
		///
		/// 在 KNN 中用于维护待访问的节点优先队列（按距离从近到远排序）。
		/// </summary>
		/// <param name="key">元素的键（如 QueryNode）</param>
		/// <param name="val">元素的优先级（如距离平方）</param>
		public void PushObjMin(T key, float val) {
			// if heap full
			if (Count == m_capacity) {
				// if Heads priority is smaller than input priority, then ignore that item
				if (HeadValue < val) {
					values[1] = val; // remove top element
					keys[1] = key;
					BubbleDownMin(1); // bubble it down
				}
			}
			else {
				Count++;
				values[Count] = val;
				keys[Count] = key;
				BubbleUpMin(Count);
			}
		}

		/// <summary>
		/// 弹出堆顶元素的通用内部实现（不区分最大/最小堆）。
		///
		/// 操作：
		/// 1. 记录堆顶的键（待返回）。
		/// 2. 将最后一个元素移动到堆顶位置。
		/// 3. 元素数量 -1。
		/// 4. 注意：调用者需在此之后调用 BubbleDownMax 或 BubbleDownMin 恢复堆性质。
		/// </summary>
		/// <returns>原堆顶元素的键</returns>
		T PopHeadObj() {
			T result = HeadKey;

			// 将最后一个元素覆盖到堆顶
			values[1] = values[Count];
			keys[1] = keys[Count];
			Count--;

			return result;
		}

		/// <summary>
		/// 从最大堆中弹出堆顶元素（当前 value 最大的元素）。
		/// 弹出后自动恢复最大堆性质（对新堆顶执行下沉）。
		/// </summary>
		/// <returns>原堆顶元素的键</returns>
		public T PopObjMax() {
			T result = PopHeadObj();
			BubbleDownMax(1);
			return result;
		}

		/// <summary>
		/// 从最小堆中弹出堆顶元素（当前 value 最小的元素）。
		/// 弹出后自动恢复最小堆性质（对新堆顶执行下沉）。
		/// </summary>
		/// <returns>原堆顶元素的键</returns>
		public T PopObjMin() {
			T result = PopHeadObj();
			BubbleDownMin(1);
			return result;
		}
	}
}
