using Unity.Burst;
using Unity.Entities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

namespace Athomield.AStar
{
    /// <summary>
    /// 网格连通区域（岛屿）分析系统。
    ///
    /// 职责：
    /// 将网格中所有相互连通的可通行节点划分为同一个"岛屿"（Island），
    /// 并为每个节点记录其所属岛屿的 ID。
    ///
    /// 算法：BFS（广度优先搜索）洪水填充
    /// - 遍历所有节点，对每个未访问的可通行节点启动一次 BFS
    /// - BFS 扩展到所有4方向相邻的可通行节点，赋予相同的 IslandID
    /// - 每完成一次 BFS 将 IslandID 递增
    ///
    /// 用途：PathfindingRequestSystem 在提交寻路请求前检查起点和终点的 IslandID，
    /// 若不同则根据 DifferentIslandsPolicy 决定是否寻路
    ///
    /// 触发条件：ASGrid.RefreshIslands = true（障碍物扫描完成后自动触发）
    /// </summary>
    partial struct CheckNodeIslandSystem : ISystem
    {
        // 是否已完成岛屿分析（避免重复计算）
        private bool mHasCheckedForIslands;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // 初始化时标记为已分析（等待 RefreshIslands 触发才执行）
            mHasCheckedForIslands = true;
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            int currentIslandID = 0; // 岛屿 ID 从 0 开始递增

            foreach (var (grid, entity) in SystemAPI.Query<RefRW<ASGrid>>().WithEntityAccess())
            {
                // 检测到 RefreshIslands 请求时，重置分析状态
                if (grid.ValueRO.RefreshIslands)
                {
                    grid.ValueRW.RefreshIslands = false;
                    mHasCheckedForIslands = false;
                }

                if (!mHasCheckedForIslands)
                {
                    DynamicBuffer<ASGridNodesBufferComponent> nodes = SystemAPI.GetBuffer<ASGridNodesBufferComponent>(entity);
                    DynamicBuffer<ASNodeIslandIDBufferComponent> nodesIDs = SystemAPI.GetBuffer<ASNodeIslandIDBufferComponent>(entity);
                    int nodeXCount = grid.ValueRO.NodeXCount;
                    int nodeYCount = grid.ValueRO.NodeYCount;

                    // 确保 IslandID 缓冲区大小与节点数一致
                    nodesIDs.Length = nodeXCount * nodeYCount;

                    int d = nodeXCount > nodeYCount ? nodeXCount : nodeYCount;
                    // 访问标记数组，防止 BFS 重复访问同一节点
                    NativeArray<bool> visited = new NativeArray<bool>(nodeXCount * nodeYCount, Allocator.Temp);

                    // 遍历所有节点，对每个未访问的可通行节点启动一次 BFS
                    for (int x = 0; x < nodeXCount; x++)
                    {
                        for (int y = 0; y < nodeYCount; y++)
                        {
                            int index;
                            if (nodeXCount > nodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            // 只对"未访问 + 可通行"的节点启动新的 BFS（发现新岛屿）
                            if (!visited[index] && nodes[index].Node.IsAccessible)
                            {
                                CheckIslandBFS(x, y, nodeXCount, nodeYCount, d, nodes, nodesIDs, visited, currentIslandID);
                                currentIslandID++; // 每个连通区域分配唯一 ID
                            }
                        }
                    }

                    mHasCheckedForIslands = true;
                }
            }
        }

        /// <summary>
        /// BFS 洪水填充：从 (_x, _y) 出发，将所有与其连通的可通行节点
        /// 标记为相同的 IslandID（_currentIslandID）
        ///
        /// 实现细节：
        /// - 不可通行节点也赋予当前 IslandID（用于边界标记），但不继续扩展
        /// - 4 方向扩展（上下左右），不考虑对角连通性
        /// </summary>
        private void CheckIslandBFS(int _x, int _y, int _nodeXCount, int _nodeYCount, int _d,
            DynamicBuffer<ASGridNodesBufferComponent> _nodes,
            DynamicBuffer<ASNodeIslandIDBufferComponent> _nodesIDs,
            NativeArray<bool> _visited, int _currentIslandID)
        {
            // 使用显式队列实现 BFS（避免递归深度溢出）
            NativeQueue<(int, int)> nodeQueue = new NativeQueue<(int, int)>(Allocator.Temp);
            nodeQueue.Enqueue((_x, _y));

            while (nodeQueue.Count > 0)
            {
                (int, int) currCorrds = nodeQueue.Dequeue();

                // 边界检查（超出网格范围的坐标直接跳过）
                if (currCorrds.Item1 < 0 || currCorrds.Item1 >= _nodeXCount ||
                    currCorrds.Item2 < 0 || currCorrds.Item2 >= _nodeYCount) continue;

                // 计算一维索引
                int currIndex;
                if (_nodeXCount > _nodeYCount)
                    currIndex = currCorrds.Item1 + currCorrds.Item2 * _d;
                else currIndex = currCorrds.Item2 + currCorrds.Item1 * _d;

                if (_visited[currIndex]) continue; // 已访问，跳过

                // 不可通行节点：赋予 IslandID 但不继续扩展（边界墙壁）
                if (!_nodes[currIndex].Node.IsAccessible)
                {
                    _nodesIDs[currIndex] = new ASNodeIslandIDBufferComponent() { IslandID = _currentIslandID };
                    continue;
                }

                // 标记当前节点已访问并赋予 IslandID
                _visited[currIndex] = true;
                _nodesIDs[currIndex] = new ASNodeIslandIDBufferComponent() { IslandID = _currentIslandID };

                // 向4方向扩展（BFS）
                nodeQueue.Enqueue((currCorrds.Item1 + 1, currCorrds.Item2)); // 右
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 + 1)); // 上
                nodeQueue.Enqueue((currCorrds.Item1 - 1, currCorrds.Item2)); // 左
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 - 1)); // 下
            }
        }
    }
}
