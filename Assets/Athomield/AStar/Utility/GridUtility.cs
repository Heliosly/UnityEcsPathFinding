using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;
using Unity.Entities;

namespace Athomield.AStar
{
    /// <summary>
    /// 网格坐标工具类，提供网格坐标与世界坐标互转、节点距离计算、邻居查询等功能。
    /// 所有方法均为静态方法，可在 Job 内调用（不涉及托管类型）
    /// </summary>
    public static class GridUtility
    {
        // Grid Entity 的全局引用（Authoring 阶段设置，方便外部代码访问）
        private static Entity mGridEntity;

        /// <summary>获取 Grid Entity 的引用</summary>
        public static Entity GetGridEntity()
        {
            return mGridEntity;
        }

        /// <summary>设置 Grid Entity 的引用（由 GridBaker 在 Bake 时调用）</summary>
        public static void SetGridEntity(Entity _gridEntity)
        {
            mGridEntity = _gridEntity;
        }

        /// <summary>
        /// 将网格坐标（列 x, 行 y）转换为世界坐标（XZ 平面中心点）。
        /// 公式：世界坐标 = 网格左下角坐标 + (x * NodeSize + NodeSize/2) * right + (y * NodeSize + NodeSize/2) * forward
        /// </summary>
        /// <param name="_x">网格列坐标（0 到 NodeXCount-1）</param>
        /// <param name="_y">网格行坐标（0 到 NodeYCount-1）</param>
        /// <param name="_grid">网格参数</param>
        /// <param name="_transform">网格 Entity 的世界变换（Grid 中心位置）</param>
        /// <returns>该节点在世界空间中的中心点坐标（Y=网格 Y）</returns>
        public static float3 GridCoordsToWorld(int _x, int _y, ASGrid _grid, LocalTransform _transform)
        {
            // 计算网格左下角（X-，Z- 方向）世界坐标
            float3 startCorner = new float3(_transform.Position.x, _transform.Position.y, _transform.Position.z)
                - math.right() * ((_grid.NodeXCount * _grid.NodeSize) / 2)
                - math.forward() * ((_grid.NodeYCount * _grid.NodeSize) / 2);

            // 加上节点偏移（节点中心，所以乘2再加1后除2 = x*NodeSize + NodeSize/2）
            return startCorner
                + math.right() * ((_x * _grid.NodeSize * 2 + _grid.NodeSize) / 2)
                + math.forward() * ((_y * _grid.NodeSize * 2 + _grid.NodeSize) / 2);
        }

        /// <summary>
        /// 将网格坐标转换为代理视角的世界坐标（Y 轴偏移到代理中心高度）
        /// </summary>
        /// <param name="_agent">代理参数（提供 Height）</param>
        public static float3 GridCoordsToWorldForAgent(int _x, int _y, ASGrid _grid, LocalTransform _transform, ASAgent _agent)
        {
            float3 startCorner = new float3(_transform.Position.x, _transform.Position.y, _transform.Position.z)
                - math.right() * ((_grid.NodeXCount * _grid.NodeSize) / 2)
                - math.forward() * ((_grid.NodeYCount * _grid.NodeSize) / 2);

            return startCorner
                + math.right() * ((_x * _grid.NodeSize * 2 + _grid.NodeSize) / 2)
                + math.forward() * ((_y * _grid.NodeSize * 2 + _grid.NodeSize) / 2)
                + math.up() * _agent.Height / 2; // Y 轴抬升到代理中心高度
        }

        /// <summary>
        /// 将世界坐标转换为最近网格节点（使用 NativeArray 版本，供 Job 内调用）。
        /// 超出网格范围的坐标会被 clamp 到边界节点
        /// </summary>
        /// <param name="_pos">世界坐标</param>
        /// <param name="_grid">网格参数</param>
        /// <param name="_gridLocalTransform">网格 Entity 的世界变换</param>
        /// <param name="_nodes">节点一维数组（扁平化二维数组）</param>
        /// <returns>距离世界坐标最近的网格节点</returns>
        public static ASNode WorldPosToNode(float3 _pos, ASGrid _grid, LocalTransform _gridLocalTransform, NativeArray<ASNode> _nodes)
        {
            // 计算归一化百分比（0~1），超界时 clamp
            float percentX = math.clamp((_pos.x - _gridLocalTransform.Position.x + (_grid.NodeXCount * _grid.NodeSize) / 2) / (_grid.NodeXCount * _grid.NodeSize), 0, 1);
            float percentY = math.clamp((_pos.z - _gridLocalTransform.Position.z + (_grid.NodeYCount * _grid.NodeSize) / 2) / (_grid.NodeYCount * _grid.NodeSize), 0, 1);

            // 映射到节点坐标（四舍五入到最近节点）
            int x = (int)math.round((_grid.NodeXCount - 1) * percentX);
            int y = (int)math.round((_grid.NodeYCount - 1) * percentY);

            // 计算一维数组索引（根据长宽大小选择主轴）
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            int index = 0;
            if (_grid.NodeXCount > _grid.NodeYCount)
                index = x + y * d;
            else index = y + x * d;

            return _nodes[index];
        }

        /// <summary>
        /// 将世界坐标转换为最近网格节点（使用 DynamicBuffer 版本，供主线程调用）
        /// </summary>
        public static ASNode WorldPosToNode(float3 _pos, ASGrid _grid, LocalTransform _gridLocalTransform, DynamicBuffer<ASGridNodesBufferComponent> _nodes)
        {
            float percentX = math.clamp((_pos.x - _gridLocalTransform.Position.x + (_grid.NodeXCount * _grid.NodeSize) / 2) / (_grid.NodeXCount * _grid.NodeSize), 0, 1);
            float percentY = math.clamp((_pos.z - _gridLocalTransform.Position.z + (_grid.NodeYCount * _grid.NodeSize) / 2) / (_grid.NodeYCount * _grid.NodeSize), 0, 1);

            int x = (int)math.round((_grid.NodeXCount - 1) * percentX);
            int y = (int)math.round((_grid.NodeYCount - 1) * percentY);

            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            int index = 0;
            if (_grid.NodeXCount > _grid.NodeYCount)
                index = x + y * d;
            else index = y + x * d;

            return _nodes[index].Node;
        }

        /// <summary>
        /// 计算节点的 F 代价（F = G + H）
        /// </summary>
        public static int NodeFCost(ASNode _node)
        {
            return _node.HCost + _node.GCost;
        }

        /// <summary>
        /// 返回 8 方向邻居的坐标偏移数组（4 正交 + 4 斜向）。
        /// A* 允许对角线移动，斜线代价 ≈ 14（√2 × 10）
        /// </summary>
        /// <param name="_allocator">内存分配器</param>
        /// <returns>包含 8 个 int2 偏移量的 NativeArray</returns>
        public static NativeArray<int2> NodePossibleNeighbours(Allocator _allocator)
        {
            NativeArray<int2> neighboursOffsetIndecies = new NativeArray<int2>(8, _allocator);

            // 4 个正交方向（代价 10）
            neighboursOffsetIndecies[0] = new int2(1, 0);   // 右
            neighboursOffsetIndecies[4] = new int2(-1, 0);  // 左
            neighboursOffsetIndecies[1] = new int2(0, 1);   // 上
            neighboursOffsetIndecies[5] = new int2(0, -1);  // 下

            // 4 个斜向方向（代价 14）
            neighboursOffsetIndecies[2] = new int2(1, 1);   // 右上
            neighboursOffsetIndecies[3] = new int2(-1, -1); // 左下
            neighboursOffsetIndecies[6] = new int2(-1, 1);  // 左上
            neighboursOffsetIndecies[7] = new int2(1, -1);  // 右下

            return neighboursOffsetIndecies;
        }

        /// <summary>
        /// 计算两个节点之间的启发式距离（切比雪夫+曼哈顿混合），
        /// 等价于允许对角移动时的最短移动代价：
        ///   - 斜线段：min(dx, dy) 步，代价 14/步
        ///   - 直线段：|dx - dy| 步，代价 10/步
        ///   - 公式：14 * min(dx,dy) + 10 * (max(dx,dy) - min(dx,dy))
        /// </summary>
        public static int NodeDistance(ASNode _node1, ASNode _node2)
        {
            return NodeDistance(_node1.X, _node1.Y, _node2.X, _node2.Y);
        }

        /// <summary>
        /// 网格坐标版启发式距离（同上，避免为了算距离而构造 ASNode）。
        /// </summary>
        public static int NodeDistance(int _x1, int _y1, int _x2, int _y2)
        {
            int xDis = math.abs(_x1 - _x2);
            int yDis = math.abs(_y1 - _y2);

            if (xDis > yDis)
                return 14 * yDis + 10 * (xDis - yDis);

            return 14 * xDis + 10 * (yDis - xDis);
        }
    }
}
