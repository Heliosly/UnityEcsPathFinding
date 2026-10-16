using System;
using Unity.Collections;
using UnityEngine;


namespace Athomield.AStar
{
    /// <summary>
    /// 泛型最小堆（Min-Heap）数据结构，基于 NativeArray 实现，可在 Burst 编译的 Job 中使用。
    /// 用于 A* 算法的 Open Set（优先队列），始终快速取出 F 代价最小的节点。
    /// 堆操作时间复杂度：Add/Pop = O(log n)，Contains = O(1)
    ///
    /// 注意：这里语义上是"最小堆"（F 最小的优先），但因为 ASNode.CompareTo 返回了负值（取反），
    /// 内部实现实际上是最大堆逻辑，等效于最小堆的行为
    /// </summary>
    /// <typeparam name="T">元素类型，需实现 IHeapItem 和 IEquatable 接口</typeparam>
    public struct NativeMinHeap<T> : IDisposable where T : unmanaged, IHeapItem<T>, IEquatable<T>
    {
        // 内部存储数组，预分配最大容量（网格节点总数）
        NativeArray<T> mItems;

        // 当前元素数量（有效区域：[0, mItemCount)）
        int mItemCount;

        /// <summary>当前堆中的元素数量</summary>
        public int Length => mItemCount;

        /// <summary>访问内部数组（用于遍历查找元素更新代价）</summary>
        public NativeArray<T> Items { get => mItems; }

        /// <summary>
        /// 构造函数，预分配指定容量的内存
        /// </summary>
        /// <param name="_maxHeapSize">最大容量（通常为网格节点总数）</param>
        /// <param name="_allocator">内存分配器（Job 中使用 Allocator.Temp）</param>
        public NativeMinHeap(int _maxHeapSize, Allocator _allocator)
        {
            mItems = new NativeArray<T>(_maxHeapSize, _allocator);
            mItemCount = 0;
        }

        /// <summary>直接在指定索引写入元素（用于更新 Open Set 中节点的代价）</summary>
        public void SetItemAt(T _item, int _index)
        {
            mItems[_index] = _item;
        }

        /// <summary>
        /// 向堆中添加新元素（O(log n)）：
        /// 1. 将元素追加到末尾
        /// 2. 向上冒泡（BubbleUp）维持堆性质
        /// </summary>
        public void Add(T _item)
        {
            _item.HeapIndex = mItemCount;
            mItems[mItemCount] = _item;

            BubbleUp(_item.HeapIndex); // 新元素可能优先级更高，向上调整

            mItemCount++;
        }

        /// <summary>
        /// 更新堆中已有元素（当节点 G 代价降低时调用，O(log n)）：
        /// 先尝试向上冒泡（代价降低），再向下沉淀（理论上不会发生）
        /// </summary>
        public void UpdateItem(T _item)
        {
            BubbleUp(_item.HeapIndex);
            BubbleDown(_item.HeapIndex);
        }

        /// <summary>
        /// 向上冒泡：将索引处的元素与父节点比较，若优先级更高则交换，直到满足堆性质
        /// 父节点索引 = (i - 1) / 2
        /// </summary>
        private void BubbleUp(int _itemIndex)
        {
            while (_itemIndex > 0)
            {
                T item = mItems[_itemIndex];
                int parentIndex = (_itemIndex - 1) / 2;

                T parentItem = mItems[parentIndex];

                if (item.CompareTo(parentItem) > 0) // item 优先级高于 parent
                {
                    SwapItems(item, parentItem);
                    _itemIndex = parentIndex; // 继续向上检查
                }
                else break; // 已满足堆性质
            }
        }

        /// <summary>
        /// 检查堆中是否存在指定元素（O(1)）：
        /// 通过 HeapIndex 直接访问元素并比较（HeapIndex 由堆自动维护）
        /// </summary>
        public bool Contains(T _item)
        {
            return mItems[_item.HeapIndex].Equals(_item);
        }

        /// <summary>
        /// 向下沉淀：将索引处的元素与子节点比较，若优先级低于子节点则与最高优先级子节点交换
        /// 左子节点索引 = i*2+1，右子节点索引 = i*2+2
        /// </summary>
        private void BubbleDown(int _itemIndex)
        {
            while (true)
            {
                int childIndexLeft = _itemIndex * 2 + 1;
                int childIndexRight = _itemIndex * 2 + 2;
                int swapIndex = _itemIndex; // 先假设当前节点最大

                // 找到优先级最高的子节点
                if (childIndexLeft < mItemCount &&
                    mItems[childIndexLeft].CompareTo(mItems[swapIndex]) > 0)
                {
                    swapIndex = childIndexLeft;
                }

                if (childIndexRight < mItemCount &&
                    mItems[childIndexRight].CompareTo(mItems[swapIndex]) > 0)
                {
                    swapIndex = childIndexRight;
                }

                if (swapIndex == _itemIndex)
                {
                    break; // 没有子节点优先级更高，堆性质已满足
                }

                SwapItems(mItems[_itemIndex], mItems[swapIndex]);
                _itemIndex = swapIndex;
            }
        }

        /// <summary>
        /// 交换两个元素在堆数组中的位置，同时更新各自的 HeapIndex
        /// </summary>
        private void SwapItems(T _item1, T _item2)
        {
            T newItem1 = mItems[_item1.HeapIndex];
            T newItem2 = mItems[_item2.HeapIndex];

            // 交换 HeapIndex
            int itemTHeapIndex = newItem1.HeapIndex;
            newItem1.HeapIndex = newItem2.HeapIndex;
            newItem2.HeapIndex = itemTHeapIndex;

            // 写回交换后的元素
            mItems[newItem1.HeapIndex] = newItem1;
            mItems[newItem2.HeapIndex] = newItem2;
        }

        /// <summary>
        /// 弹出并返回堆顶元素（优先级最高 = F 代价最小，O(log n)）：
        /// 1. 取堆顶元素（index 0）
        /// 2. 将末尾元素移到堆顶
        /// 3. 向下沉淀（BubbleDown）维持堆性质
        /// </summary>
        public T PopFirstItem()
        {
            T firstItem = mItems[0]; // 堆顶 = 最高优先级（F 最小）
            mItemCount--;

            // 将末尾元素移到堆顶替代被弹出的元素
            T item = mItems[mItemCount];
            item.HeapIndex = 0;
            mItems[0] = item;

            BubbleDown(0); // 新堆顶可能优先级低，向下调整
            return firstItem;
        }

        /// <summary>调试用：返回堆内容字符串</summary>
        public string ToString(string _par)
        {
            string s = _par;
            for (int i = 0; i < mItemCount; i++)
            {
                s += mItems[i].ToString() + " | ";
            }

            return s;
        }

        /// <summary>释放内部 NativeArray 内存（必须在使用完毕后调用）</summary>
        public void Dispose()
        {
            mItems.Dispose();
        }
    }

    /// <summary>
    /// 最小堆元素接口，要求元素：
    /// 1. 实现 IComparable 用于堆排序
    /// 2. 维护 HeapIndex 属性用于 O(1) 定位元素在堆中的位置
    /// </summary>
    public interface IHeapItem<T> : IComparable<T>
    {
        /// <summary>该元素在堆数组中的当前索引（由堆自动维护，勿手动修改）</summary>
        int HeapIndex { get; set; }
    }
}
