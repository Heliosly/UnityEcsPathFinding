using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Athomield.AStar
{
    /// <summary>
    /// A* 寻路系统，负责并行执行所有待处理的寻路操作。
    ///
    /// 工作流程：
    /// 1. 获取当前网格节点数据快照
    /// 2. 遍历所有待处理的 ASPathfindingOperation Entity（由 PathfindingRequestSystem 创建）
    /// 3. 以 IJobParallelFor 并行方式对每个操作执行 A* 算法（ASRunParallelSearch Job）
    /// 4. 将规划出的路径点写入对应 Entity 的 ASPathBufferComponent 缓冲区
    /// 5. 写入 ASPathfindingResult（PathFound + FinishedSearch），通知 AgentMovementSystem
    ///
    /// 并行安全：每个操作有独立的 Entity，通过 NativeDisableContainerSafetyRestriction 标记
    /// 允许在并行 Job 中安全地读写不同 Entity 的组件
    /// </summary>
    partial struct PathfindingSystem : ISystem
    {
        private EntityTypeHandle EntityTypeHandle;

        // 操作数据（只读）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<ASPathfindingOperation> SystemAllOperations;

        // 操作结果（读写，写入 PathFound/FinishedSearch）
        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<ASPathfindingResult> SystemAllResults;

        // 路径点缓冲区（读写，写入规划出的路径）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASPathBufferComponent> SystemAllPathPoints;

        // 网格节点缓冲区（读写，Job 内会临时修改 G/H 代价）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASGridNodesBufferComponent> SystemAllGridNodes;

        // 节点岛屿 ID 缓冲区（只读；用于 DontPath 策略在搜索内部做"不可达"兜底判断）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASNodeIslandIDBufferComponent> SystemAllIslandIDs;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            EntityTypeHandle = SystemAPI.GetEntityTypeHandle();
            SystemAllOperations = state.GetComponentLookup<ASPathfindingOperation>(true);
            SystemAllResults = state.GetComponentLookup<ASPathfindingResult>(true);
            SystemAllPathPoints = state.GetBufferLookup<ASPathBufferComponent>(true);
            SystemAllGridNodes = state.GetBufferLookup<ASGridNodesBufferComponent>();
            SystemAllIslandIDs = state.GetBufferLookup<ASNodeIslandIDBufferComponent>(true);

            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<ASPathfindingResult>();
            types[1] = ComponentType.ReadOnly<ASPathfindingOperation>();
            types.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            SystemAllPathPoints.Update(ref state);
            SystemAllResults.Update(ref state);
            SystemAllPathPoints.Update(ref state);
            SystemAllGridNodes.Update(ref state);
            SystemAllIslandIDs.Update(ref state);
            EntityTypeHandle.Update(ref state);

            // 查询所有同时拥有 ASPathfindingResult 和 ASPathfindingOperation 的 Entity（寻路操作 Entity）
            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<ASPathfindingResult>();
            types[1] = ComponentType.ReadOnly<ASPathfindingOperation>();

            var query = state.GetEntityQuery(types);
            var chunks = query.ToArchetypeChunkArray(Allocator.Temp);

            // 遍历所有 Grid Entity（通常只有一个）
            foreach (var (grid, gridLocalTransform, gridEntity) in SystemAPI.Query<ASGrid, LocalTransform>().WithEntityAccess())
            {
                // 将网格节点复制到 NativeArray（供 Burst Job 使用）。
                // 只做这一次拷贝：旧实现还有"每 chunk 一份 + 每个操作一份"两次多余的全数组 memcpy，
                // 6720 节点的网格下每次搜索要多搬 ~500KB 内存。
                DynamicBuffer<ASGridNodesBufferComponent> nodeBuffer = SystemAllGridNodes[gridEntity];
                NativeArray<ASNode> nodes = new NativeArray<ASNode>(nodeBuffer.Length, Allocator.TempJob);
                for (int i = 0; i < nodeBuffer.Length; i++)
                {
                    nodes[i] = nodeBuffer[i].Node;
                }

                // 岛屿 ID 快照（与节点数组同索引）。岛屿数据未就绪时长度不足，
                // 此时传长度 0 的空数组，搜索内部会跳过 DontPath 兜底判断。
                NativeArray<int> islandIDs = new NativeArray<int>(0, Allocator.TempJob);
                if (SystemAllIslandIDs.HasBuffer(gridEntity) &&
                    SystemAllIslandIDs[gridEntity].Length >= nodes.Length)
                {
                    islandIDs = new NativeArray<int>(nodes.Length, Allocator.TempJob);
                    DynamicBuffer<ASNodeIslandIDBufferComponent> islandBuffer = SystemAllIslandIDs[gridEntity];
                    for (int i = 0; i < nodes.Length; i++)
                    {
                        islandIDs[i] = islandBuffer[i].IslandID;
                    }
                }

                // 对每个 Archetype Chunk 并行处理其中的所有寻路操作
                foreach (var archetypeChunk in chunks)
                {
                    // 调度并行 Job：每个操作 Entity 独立执行 A* 算法
                    JobHandle jh = Process(archetypeChunk, grid, gridLocalTransform, nodes, islandIDs,
                        new JobHandle(), SystemAllGridNodes[gridEntity]);
                    jh.Complete(); // 等待完成（同帧完成寻路）
                }

                nodes.Dispose();
                islandIDs.Dispose();
            }

            types.Dispose();
            chunks.Dispose();
        }

        /// <summary>
        /// 为一个 Chunk 内的所有寻路操作调度并行 A* Job
        /// </summary>
        private JobHandle Process(in ArchetypeChunk _chunk, in ASGrid _grid, in LocalTransform _gridLocalTransform,
            NativeArray<ASNode> _nodes, NativeArray<int> _islandIDs, JobHandle _inputDeps,
            DynamicBuffer<ASGridNodesBufferComponent> _originalNodes)
        {
            // 搜索只【读】节点数组（可通行性 / 惩罚值），不再为每个 chunk 复制一份：
            // 全部搜索状态写在各操作自己的临时数组里（见 ASParallelSearch.Execute）。
            ASRunParallelSearch searchJob = new ASRunParallelSearch()
            {
                OperationEntities = _chunk.GetNativeArray(EntityTypeHandle), // 本 Chunk 中的操作 Entity 列表
                JobAllOperations = SystemAllOperations,
                JobAllResults = SystemAllResults,
                JobAllPathPoints = SystemAllPathPoints,
                JobNodes = _nodes,
                JobIslandIDs = _islandIDs,
                JobGrid = _grid,
                JobGridLocalTransform = _gridLocalTransform,
            };

            // 调度并行 Job（每个操作分配到独立线程，批次大小 2）
            return searchJob.Schedule(_chunk.Count, 2, _inputDeps);
        }
    }

    /// <summary>
    /// 并行 A* 搜索 Job。
    /// 为 Chunk 内每个寻路操作 Entity 各启动一个 ASParallelSearch 执行 A* 算法。
    /// BurstCompile 标注使此 Job 被 Burst 编译为原生代码，性能大幅提升
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct ASRunParallelSearch : IJobParallelFor
    {
        // 本 Chunk 中的操作 Entity 列表（只读）
        [ReadOnly]
        public NativeArray<Entity> OperationEntities;

        // 各种组件/缓冲区查找（NativeDisableContainerSafetyRestriction 允许并行访问）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<ASPathfindingOperation> JobAllOperations;

        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<ASPathfindingResult> JobAllResults;

        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASPathBufferComponent> JobAllPathPoints;

        // 所有并行搜索共享同一份【只读】节点数组（只读可通行性/惩罚值）。
        // 注意：这里不能再用 DeallocateOnJobCompletion —— 数组由 OnUpdate 统一分配与释放。
        [NativeDisableParallelForRestriction, ReadOnly]
        public NativeArray<ASNode> JobNodes;

        // 节点岛屿 ID（只读；与 JobNodes 同索引，长度 0 表示岛屿数据未就绪）
        [ReadOnly]
        public NativeArray<int> JobIslandIDs;

        public ASGrid JobGrid;
        public LocalTransform JobGridLocalTransform;

        public void Execute(int index)
        {
            Entity e = OperationEntities[index];
            ASPathfindingResult res = JobAllResults[e];

            // 只处理尚未完成搜索的操作（避免重复计算）
            if (!res.FinishedSearch)
            {
                ASParallelSearch parallelSearch = new ASParallelSearch()
                {
                    OperationEntity = e,
                    AllOperations = JobAllOperations,
                    AllResults = JobAllResults,
                    AllPathPoints = JobAllPathPoints,
                    Nodes = JobNodes,
                    IslandIDs = JobIslandIDs,
                    Grid = JobGrid,
                    GridLocalTransform = JobGridLocalTransform,
                };

                parallelSearch.Execute(); // 执行 A* 算法
            }
        }
    }

    /// <summary>
    /// 核心 A* 寻路算法实现（按节点一维索引搜索）。
    ///
    /// 算法流程：
    /// 1. 将起点和终点世界坐标转换为网格节点索引
    /// 2. 若终点不可通行，使用 BFS 寻找最近的可通行节点作为替代终点
    /// 3. DontPath 策略下：替代终点与起点不在同一岛屿 → 直接判失败（不搜索）
    /// 4. 计算搜索预算（估计步数 × 系数），防止"目标不可达"时穷举整块可达区域
    /// 5. A* 主循环（全部状态按节点索引 O(1) 访问）：
    ///    a. 索引式最小堆取出 F 最优节点（decrease-key O(log n)）
    ///    b. 到达终点 → 回溯路径返回
    ///    c. 遍历 8 方向邻居，松弛 G 代价（闭合位图 O(1) 判重）
    /// 6. 预算耗尽或开集为空 = 目标不可达：
    ///    返回一条"尽力而为"的路径（走到离目标最近的节点）并置 Unreachable，
    ///    由 AgentMovementSystem 触发重试冷却，避免每帧重复全区域搜索
    /// 7. 路径回溯：沿父节点索引链从终点走回起点，反转后输出世界坐标
    /// </summary>
    public struct ASParallelSearch
    {
        public Entity OperationEntity;

        public ComponentLookup<ASPathfindingOperation> AllOperations;
        public ComponentLookup<ASPathfindingResult> AllResults;
        public BufferLookup<ASPathBufferComponent> AllPathPoints;

        // 网格节点数组（只读：可通行性 / 惩罚值）。
        // 搜索状态（G 代价 / 父节点 / 闭合位图 / 开集）全部放在 Execute 内按节点索引的
        // 临时数组里，不再把整份节点数组复制出来改（旧实现每次搜索拷 2 份全网格数组 + 每节点一次临时分配）。
        // 语义与旧实现一致：闭集用 (X,Y)→一维索引的位图（ASNode.Equals 只比 X/Y）。
        [NativeDisableContainerSafetyRestriction] public NativeArray<ASNode> Nodes;

        // 节点岛屿 ID（只读，与 Nodes 同索引；长度 0 = 岛屿数据未就绪，跳过不可达兜底判断）
        [ReadOnly] public NativeArray<int> IslandIDs;

        public ASGrid Grid;
        public LocalTransform GridLocalTransform;

        // ===== 搜索预算 =====
        // 目标可达时 A* 只展开起终点之间的窄带（几百节点）；目标不可达（跨岛/被墙封死）时
        // 必须展开整块可达区域才能确认失败 —— 跟随目标的代理每帧重发请求，就会每帧触发一次
        // 全区域搜索，这是"走进封闭区域后极度卡顿"的根因之一。
        // 现在整图搜索已是 O(N log N)（索引堆 + 位图），全量展开也很便宜，
        // 因此不再按估算步数裁剪（旧上限会把正常绕远路误判为不可达，见 Execute 内说明）。
        // 下面两个常数当前未使用，保留供"超大网格要限制单帧成本"时重新启用。
        private const int MinExpansionBudget = 1024;  // 下限：短距离也不至于过早放弃
        private const int ExpansionBudgetFactor = 16; // 预算 = 估计步数 × 该系数
        private const int StraightCost = 10;          // 正交移动代价
        private const int DiagonalCost = 14;          // 斜向移动代价

        public void Execute()
        {
            ASPathfindingOperation operation = AllOperations[OperationEntity];
            ASPathfindingResult operationResult = AllResults[OperationEntity];
            DynamicBuffer<ASPathBufferComponent> pathBuffer = AllPathPoints[OperationEntity];

            operationResult.PathFound = false;
            operationResult.FinishedSearch = false;
            operationResult.Unreachable = false;

            int nodeXCount = Grid.NodeXCount;
            int nodeYCount = Grid.NodeYCount;
            int nodeCount = nodeXCount * nodeYCount;

            // 将起点/终点世界坐标转换为网格节点（Nodes 只读，不再整份复制）
            ASNode startNode = GridUtility.WorldPosToNode(operation.StartPoint, Grid, GridLocalTransform, Nodes);
            ASNode targetNode = GridUtility.WorldPosToNode(operation.TargetPoint, Grid, GridLocalTransform, Nodes);
            int startIndex = IndexOf(Grid, startNode.X, startNode.Y);

            // ===== DontPath 策略：岛屿判定必须放在最前面 =====
            // 两个要点：
            //  1) 顺序：旧实现把它放在"终点不可通行 → BFS 找替代终点"之后，于是每次请求
            //     都要先白跑一遍全图 BFS（含整张 bool 网格分配 + 队列），才判定"不同岛"退出。
            //     玩家贴墙/站门口时目标节点常常不可通行，这条路径每次请求都会走一遍。
            //  2) 取值：不可通行节点的 IslandID 是"哪个岛屿先碰到算谁的"（岛屿 BFS 对这类节点
            //     只赋值不扩展），语义不可靠；这里改用它的【可通行邻居】的岛屿 ID，避免误判成同岛。
            if (Grid.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath && IslandIDs.Length > 0)
            {
                int startIsland = ResolveIsland(startIndex);
                int targetIsland = ResolveIsland(IndexOf(Grid, targetNode.X, targetNode.Y));

                // -1 = 无法判定（节点在障碍内部且四周也是障碍）→ 放行，交给后面的搜索/BFS 处理
                if (startIsland >= 0 && targetIsland >= 0 && startIsland != targetIsland)
                {
                    // 与策略语义一致（不同岛屿不寻路）：直接判失败，不做任何搜索。
                    // Unreachable = true → 代理进入重试冷却，并（在 AgentMovementSystem 里）停下来，
                    // 不会继续沿着过期路径"乱跑"。
                    operationResult.PathFound = false;
                    operationResult.FinishedSearch = true;
                    operationResult.Unreachable = true;
                    AllResults[OperationEntity] = operationResult;
                    return;
                }
            }

            // ===== 终点不可通行时：BFS 寻找最近可通行节点 =====
            if (!targetNode.IsAccessible)
            {
                ASNode newTargetNode = new ASNode();

                if (ExpandInaccessibleBFS(ref newTargetNode, targetNode, startNode, Grid, Nodes))
                {
                    targetNode = newTargetNode; // 使用 BFS 找到的替代终点
                }
                else
                {
                    // 完全无法找到可通行的替代终点：失败 + 标记不可达（代理会走重试冷却）
                    operationResult.PathFound = false;
                    operationResult.FinishedSearch = true;
                    operationResult.Unreachable = true;
                    AllResults[OperationEntity] = operationResult;
                    return;
                }
            }

            int targetIndex = IndexOf(Grid, targetNode.X, targetNode.Y);

            // ===== DontPath 策略二次判定：BFS 替换后的终点可能落在另一个岛屿 =====
            if (Grid.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath && IslandIDs.Length > 0)
            {
                int startIsland = ResolveIsland(startIndex);
                int targetIsland = ResolveIsland(targetIndex);
                if (startIsland >= 0 && targetIsland >= 0 && startIsland != targetIsland)
                {
                    operationResult.PathFound = false;
                    operationResult.FinishedSearch = true;
                    operationResult.Unreachable = true;
                    AllResults[OperationEntity] = operationResult;
                    return;
                }
            }

            // ===== 搜索预算 =====
            // 改为"不裁剪"：整图搜索现在只是 O(N log N)（索引堆 + 位图，已无线性扫描），
            // 6720 节点约十万次操作、亚毫秒级；而"预算裁剪"会把正常的绕远路误判成不可达——
            // 实测（48×48 网格，一道只在 y>=45 留缺口的竖墙，起点(5,5)→终点(40,5)）：
            // 1024 次预算全被墙侧同 F 值的大平台吃掉，没走到缺口就返回半截路径并标记 Unreachable，
            // 代理表现为"走两步就停"。真正的病态场景（跨岛 / 被墙封死）已由前面的
            // 岛屿早退 + 代理 0.5s 重试冷却兜住，不需要靠裁剪来省时间。
            int heuristicCost = GridUtility.NodeDistance(startNode.X, startNode.Y, targetNode.X, targetNode.Y);
            int expansionBudget = nodeCount;

            // ===== 初始化 A* 数据结构（全部按节点一维索引，O(1) 访问）=====
            NativeArray<int> gCost = new NativeArray<int>(nodeCount, Allocator.Temp);
            NativeArray<int> parent = new NativeArray<int>(nodeCount, Allocator.Temp);
            NativeArray<bool> closed = new NativeArray<bool>(nodeCount, Allocator.Temp);
            for (int i = 0; i < nodeCount; i++)
            {
                gCost[i] = int.MaxValue;
                parent[i] = -1;
            }

            var openSet = new ASIndexedMinHeap(nodeCount, Allocator.Temp);
            gCost[startIndex] = 0;
            openSet.Push(startIndex, 0, heuristicCost); // 起点入开集

            // 邻居偏移：循环外只分配一次（旧实现每展开一个节点就分配/释放一次）
            NativeArray<int2> neighbourOffsets = GridUtility.NodePossibleNeighbours(Allocator.Temp);

            int closestIndex = startIndex;         // 搜索到的离目标最近的节点（不可达时做 best-effort）
            int closestDistFromGoal = heuristicCost;
            bool targetFound = false;
            int expansions = 0;

            // ===== A* 主循环 =====
            while (!openSet.IsEmpty)
            {
                // 预算用尽：停止穷举，改用"已探索到的最近节点"做尽力而为的路径
                if (expansions >= expansionBudget) break;
                expansions++;

                // 取出 F 代价最小的节点（索引堆 + decrease-key，无重复项、无需线性扫描）
                int currentIndex = openSet.Pop(out int currentG);
                if (closed[currentIndex]) continue; // 防御：正常流程不会重复出堆
                closed[currentIndex] = true;        // 加入 Closed Set（位图 O(1)）

                // 到达终点：回溯路径并返回
                if (currentIndex == targetIndex)
                {
                    targetFound = true;
                    break;
                }

                int currentX = XOf(Grid, currentIndex);
                int currentY = YOf(Grid, currentIndex);

                // 记录距目标最近的节点（不可达时做 best-effort 路径用）
                int currNodeDistanceFromGoal = GridUtility.NodeDistance(currentX, currentY, targetNode.X, targetNode.Y);
                if (currNodeDistanceFromGoal < closestDistFromGoal)
                {
                    closestDistFromGoal = currNodeDistanceFromGoal;
                    closestIndex = currentIndex;
                }

                // ===== 遍历 8 方向邻居（偏移数组在循环外已分配）=====
                for (int n = 0; n < neighbourOffsets.Length; n++)
                {
                    int neighbourX = currentX + neighbourOffsets[n].x;
                    int neighbourY = currentY + neighbourOffsets[n].y;

                    // 边界检查
                    if (neighbourX < 0 || neighbourX >= nodeXCount ||
                        neighbourY < 0 || neighbourY >= nodeYCount) continue;

                    int neighbourIndex = IndexOf(Grid, neighbourX, neighbourY);

                    // 跳过已关闭节点（位图直查 O(1)）
                    if (closed[neighbourIndex]) continue;

                    // 跳过不可通行节点
                    ASNode neighbourNode = Nodes[neighbourIndex];
                    if (!neighbourNode.IsAccessible) continue;

                    // 计算经过当前节点到达邻居的代价（G = 当前G + 移动代价 + 地形惩罚）
                    int stepCost = (neighbourOffsets[n].x != 0 && neighbourOffsets[n].y != 0) ? DiagonalCost : StraightCost;
                    int tentativeG = currentG + stepCost + neighbourNode.Penalty;

                    // 不是更优解则跳过（G 表代替旧实现往节点数组里回写 G 值）
                    if (tentativeG >= gCost[neighbourIndex]) continue;

                    gCost[neighbourIndex] = tentativeG;
                    parent[neighbourIndex] = currentIndex;

                    int h = GridUtility.NodeDistance(neighbourX, neighbourY, targetNode.X, targetNode.Y);
                    if (openSet.Contains(neighbourIndex))
                        openSet.DecreaseKey(neighbourIndex, tentativeG, h); // 真正的 decrease-key, O(log n)
                    else
                        openSet.Push(neighbourIndex, tentativeG, h);        // 新节点入开集, O(log n)
                }
            }

            neighbourOffsets.Dispose();
            openSet.Dispose();

            if (targetFound)
            {
                operationResult.PathFound = true;
                operationResult.FinishedSearch = true;
                operationResult.Unreachable = false;
                RetracePath(startIndex, targetIndex, parent, pathBuffer, true);
            }
            else
            {
                // 未找到路径（预算用尽，或可达区域已全部探测完）= 目标在当前可达区域内不可达。
                // 仍然给一条"尽力而为"的路径（走到离目标最近的节点），但标记 Unreachable：
                // AgentMovementSystem 据此让代理进入重试冷却，避免每帧重复全区域搜索。
                // 注意 best-effort 路径不再追加精确目标点 —— 否则代理会一直朝墙里挤。
                operationResult.PathFound = closestIndex != startIndex;
                operationResult.FinishedSearch = true;
                operationResult.Unreachable = true;
                if (operationResult.PathFound)
                {
                    RetracePath(startIndex, closestIndex, parent, pathBuffer, false);
                }
                else
                {
                    pathBuffer.Clear();
                }
            }

            gCost.Dispose();
            parent.Dispose();
            closed.Dispose();

            AllResults[OperationEntity] = operationResult;
        }

        /// <summary>
        /// 网格坐标 → 节点一维索引（与 GridAuthoring/WorldPosToNode 使用同一套主轴规则）。
        /// 闭合位图、G 表、父节点表、岛屿 ID 数组都按这个索引访问。
        /// </summary>
        static int IndexOf(ASGrid _grid, int _x, int _y)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;
            return _grid.NodeXCount > _grid.NodeYCount ? _x + _y * d : _y + _x * d;
        }

        /// <summary>节点一维索引 → 网格 X 坐标。</summary>
        static int XOf(ASGrid _grid, int _index)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;
            return _grid.NodeXCount > _grid.NodeYCount ? _index % d : _index / d;
        }

        /// <summary>节点一维索引 → 网格 Y 坐标。</summary>
        static int YOf(ASGrid _grid, int _index)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;
            return _grid.NodeXCount > _grid.NodeYCount ? _index / d : _index % d;
        }

        /// <summary>
        /// 取节点所属岛屿 ID。
        /// 节点本身可通行时直接取其 IslandID；不可通行时（岛屿 BFS 对这类节点只赋"边界 ID"、
        /// 语义不可靠）改用其 4 邻域中第一个可通行节点的岛屿 ID（与岛屿 BFS 的 4 连通口径一致）。
        /// 返回 -1 表示无法判定（索引越界 / 身处障碍内部且四周也是障碍）。
        /// </summary>
        int ResolveIsland(int _nodeIndex)
        {
            if (_nodeIndex < 0 || _nodeIndex >= IslandIDs.Length || _nodeIndex >= Nodes.Length) return -1;

            if (Nodes[_nodeIndex].IsAccessible) return IslandIDs[_nodeIndex];

            int x = XOf(Grid, _nodeIndex);
            int y = YOf(Grid, _nodeIndex);

            // 手写 4 邻域：Burst 下不引入静态数组（避免静态托管数据导致的编译回退）
            if (TryNeighbourIsland(x + 1, y, out int island)) return island;
            if (TryNeighbourIsland(x, y + 1, out island)) return island;
            if (TryNeighbourIsland(x - 1, y, out island)) return island;
            if (TryNeighbourIsland(x, y - 1, out island)) return island;
            return -1;
        }

        /// <summary>取(_x,_y)处可通行节点的岛屿 ID；不可通行/越界返回 false。</summary>
        bool TryNeighbourIsland(int _x, int _y, out int _island)
        {
            _island = -1;
            if (_x < 0 || _x >= Grid.NodeXCount || _y < 0 || _y >= Grid.NodeYCount) return false;

            int index = IndexOf(Grid, _x, _y);
            if (!Nodes[index].IsAccessible) return false;

            _island = IslandIDs[index];
            return true;
        }

        /// <summary>
        /// 路径回溯：沿父节点索引链从终点走回起点，反转后写入路径点（世界坐标）。
        /// _appendExactTarget = true：可达路径末尾追加精确目标世界坐标（减少最终误差，保持原行为）；
        /// _appendExactTarget = false：best-effort（不可达）路径不追加 —— 否则代理会一直朝墙里挤。
        /// </summary>
        void RetracePath(int _startIndex, int _endIndex, NativeArray<int> _parent,
            DynamicBuffer<ASPathBufferComponent> _pathPoints, bool _appendExactTarget)
        {
            _pathPoints.Clear();

            if (_endIndex != _startIndex)
            {
                NativeList<int> chain = new NativeList<int>(Allocator.Temp);
                int cursor = _endIndex;
                int guard = 0; // 防御：父链异常时不至于死循环（正常最多 nodeCount 步）
                while (cursor != _startIndex && cursor >= 0 && guard++ < _parent.Length)
                {
                    chain.Add(cursor);
                    cursor = _parent[cursor]; // 跳到父节点
                }

                // 反转路径（从起点到终点的正向顺序），转换为世界坐标
                for (int i = chain.Length - 1; i >= 0; i--)
                {
                    int index = chain[i];
                    _pathPoints.Add(new ASPathBufferComponent()
                    {
                        Point = GridUtility.GridCoordsToWorld(XOf(Grid, index), YOf(Grid, index), Grid, GridLocalTransform)
                    });
                }

                chain.Dispose();
            }

            if (_appendExactTarget)
            {
                // 最后追加精确的目标世界坐标（替代最近节点中心，减少最终误差）
                _pathPoints.Add(new ASPathBufferComponent() { Point = AllOperations[OperationEntity].TargetPoint });
            }
        }

        /// <summary>
        /// BFS 扩展搜索：当终点节点不可通行时，从终点向外 BFS 寻找最近的可通行节点。
        /// 搜索范围受 Grid.TargetInaccessibleSearchTolerance 限制，
        /// 在所有找到的可通行节点中选择距起点最近的一个（避免绕远路）
        /// </summary>
        bool ExpandInaccessibleBFS(ref ASNode _closestNode, ASNode _targetNode, ASNode _startingNode,
            ASGrid _grid, NativeArray<ASNode> _nodes)
        {
            int d = Grid.NodeXCount > Grid.NodeYCount ? Grid.NodeXCount : Grid.NodeYCount;
            NativeArray<bool> visited = new NativeArray<bool>(_grid.NodeXCount * _grid.NodeYCount, Allocator.Temp);
            NativeQueue<(int, int)> nodeQueue = new NativeQueue<(int, int)>(Allocator.Temp);
            nodeQueue.Enqueue((_targetNode.X, _targetNode.Y));

            bool foundClosestTarget = false;
            int closestDistance = int.MaxValue;

            while (nodeQueue.Count > 0)
            {
                (int, int) currCorrds = nodeQueue.Dequeue();

                int currIndex;
                if (_grid.NodeXCount > _grid.NodeYCount)
                    currIndex = currCorrds.Item1 + currCorrds.Item2 * d;
                else currIndex = currCorrds.Item2 + currCorrds.Item1 * d;

                // 边界检查 + 搜索范围限制（TargetInaccessibleSearchTolerance * 10 个节点距离）
                if (currCorrds.Item1 < 0 || currCorrds.Item1 >= _grid.NodeXCount ||
                    currCorrds.Item2 < 0 || currCorrds.Item2 >= _grid.NodeYCount ||
                    GridUtility.NodeDistance(_targetNode, _nodes[currIndex]) > _grid.TargetInaccessibleSearchTolerance * 10) continue;

                if (visited[currIndex]) continue;

                // 找到可通行节点：记录距起点最近的一个
                if (_nodes[currIndex].IsAccessible)
                {
                    foundClosestTarget = true;
                    int dist = GridUtility.NodeDistance(_startingNode, _nodes[currIndex]);
                    if (dist < closestDistance)
                    {
                        _closestNode = _nodes[currIndex];
                        closestDistance = dist;
                    }
                    continue; // 不继续扩展（可通行节点是终点）
                }

                visited[currIndex] = true;

                // 向4方向扩展（只扩展不可通行节点内部，寻找边界上的可通行节点）
                nodeQueue.Enqueue((currCorrds.Item1 + 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 + 1));
                nodeQueue.Enqueue((currCorrds.Item1 - 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 - 1));
            }

            return foundClosestTarget;
        }
    }

    /// <summary>
    /// A* 开集元素：只存"节点一维索引 + G/H"（16 字节；旧实现往堆里塞整个 ASNode 是 40 字节）。
    /// </summary>
    internal struct ASOpenEntry
    {
        public int Index;
        public int GCost;
        public int HCost;
    }

    /// <summary>
    /// 索引式二叉最小堆（按 F = G + H 排序），支持 O(log n) 插入与 decrease-key。
    ///
    /// 为什么不用通用 NativeMinHeap&lt;ASNode&gt;：
    ///   旧代码在 G 代价降低时是"遍历整个堆数组找同索引元素（O(|open|)），
    ///   再用元素里可能已经过期的 HeapIndex 调 SetItemAt/UpdateItem"——
    ///   既慢（每次松弛 O(|open|)，整场搜索 O(N²)），又因为 Contains 依赖
    ///   item.HeapIndex 而经常判错（栈上拷贝的 HeapIndex 是过期值），会重复入堆。
    /// 现在用 slots 数组记录"节点索引 → 堆内位置"，decrease-key 原地上浮，语义干净、复杂度确定。
    /// F 相同时优先 H 更小的节点（路径更直，堆行为也更稳定）。
    /// </summary>
    internal struct ASIndexedMinHeap
    {
        private NativeArray<ASOpenEntry> mEntries;
        private NativeArray<int> mSlots;   // 节点索引 → 堆内位置（-1 = 不在堆中）
        private int mCount;

        public bool IsEmpty => mCount == 0;

        public ASIndexedMinHeap(int _capacity, Allocator _allocator)
        {
            mEntries = new NativeArray<ASOpenEntry>(_capacity, _allocator);
            mSlots = new NativeArray<int>(_capacity, _allocator);
            for (int i = 0; i < _capacity; i++) mSlots[i] = -1;
            mCount = 0;
        }

        /// <summary>该节点当前是否在开集中（O(1)）。</summary>
        public bool Contains(int _nodeIndex)
        {
            return mSlots[_nodeIndex] >= 0;
        }

        public void Push(int _nodeIndex, int _gCost, int _hCost)
        {
            int slot = mCount++;
            mEntries[slot] = new ASOpenEntry { Index = _nodeIndex, GCost = _gCost, HCost = _hCost };
            mSlots[_nodeIndex] = slot;
            BubbleUp(slot);
        }

        /// <summary>节点 G 代价降低：原地上浮（O(log n)）。调用前需确保 Contains 为 true。</summary>
        public void DecreaseKey(int _nodeIndex, int _gCost, int _hCost)
        {
            int slot = mSlots[_nodeIndex];
            ASOpenEntry entry = mEntries[slot];
            entry.GCost = _gCost;
            entry.HCost = _hCost;
            mEntries[slot] = entry;
            BubbleUp(slot);
        }

        /// <summary>弹出 F 最小的节点索引，并通过 out 返回其 G 代价。</summary>
        public int Pop(out int _gCost)
        {
            ASOpenEntry top = mEntries[0];
            mSlots[top.Index] = -1;
            mCount--;

            if (mCount > 0)
            {
                mEntries[0] = mEntries[mCount];
                mSlots[mEntries[0].Index] = 0;
                BubbleDown(0);
            }

            _gCost = top.GCost;
            return top.Index;
        }

        public void Dispose()
        {
            mEntries.Dispose();
            mSlots.Dispose();
        }

        private void BubbleUp(int _slot)
        {
            while (_slot > 0)
            {
                int parentSlot = (_slot - 1) / 2;
                if (!HigherPriority(_slot, parentSlot)) break;
                Swap(_slot, parentSlot);
                _slot = parentSlot;
            }
        }

        private void BubbleDown(int _slot)
        {
            while (true)
            {
                int left = _slot * 2 + 1;
                int right = left + 1;
                int best = _slot;

                if (left < mCount && HigherPriority(left, best)) best = left;
                if (right < mCount && HigherPriority(right, best)) best = right;
                if (best == _slot) break;

                Swap(_slot, best);
                _slot = best;
            }
        }

        private bool HigherPriority(int _a, int _b)
        {
            int fa = mEntries[_a].GCost + mEntries[_a].HCost;
            int fb = mEntries[_b].GCost + mEntries[_b].HCost;
            if (fa != fb) return fa < fb;
            return mEntries[_a].HCost < mEntries[_b].HCost;
        }

        private void Swap(int _a, int _b)
        {
            ASOpenEntry tmp = mEntries[_a];
            mEntries[_a] = mEntries[_b];
            mEntries[_b] = tmp;
            mSlots[mEntries[_a].Index] = _a;
            mSlots[mEntries[_b].Index] = _b;
        }
    }
}
