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
    /// 障碍物和地形惩罚检测系统（网格初始化的核心系统）。
    ///
    /// ──── 完整执行流程 ────
    ///
    ///  RefreshGrid = true（由 GridAuthoring 初始烘焙或调试 A 键触发）
    ///         ↓
    ///  OnUpdate 检测到标志 → 重置扫描状态
    ///         ↓
    ///  CheckForObstacles：对每个节点执行 OverlapBox 物理查询
    ///    ├─ 命中 ASObstacle → IsAccessible = false，写入障碍物边缘惩罚
    ///    └─ 命中 ASPenalty → Penalty += 地形惩罚值
    ///         ↓
    ///  BlurPenalties：对惩罚值做可分离高斯模糊（水平 + 垂直两次箱式滤波）
    ///         ↓
    ///  RefreshIslands = true → 触发 CheckNodeIslandSystem 重新计算连通区域
    ///
    /// ──── 关键设计说明 ────
    /// • 此系统不用 Job，直接在主线程运行。原因：每帧节点数量可能非常大，
    ///   但此系统只在网格刷新时执行一次，性能可接受；且 OverlapBox 需要访问
    ///   PhysicsWorld，在 Job 内需要额外的安全标记。
    /// • ComponentLookup 在 OnCreate 时创建，每帧 Update 刷新引用，
    ///   避免重复创建带来的堆分配开销。
    /// </summary>
    partial struct CheckObstaclesSystem : ISystem
    {
        // 障碍物扫描完成标志（true = 本轮已扫描，无需重复执行）
        private bool mHasCheckedForObstacles;
        // 惩罚扫描完成标志（CheckForPenalties 已合并入 CheckForObstacles，此标志暂未使用）
        private bool mHasCheckedForPenalties;

        // ComponentLookup 缓存：按 Entity 快速查找 ASObstacle / ASPenalty 组件
        // 比 EntityManager.GetComponentData 更高效，内部使用 TypeIndex 直接寻址
        ComponentLookup<ASObstacle> mAllObstacles;
        ComponentLookup<ASPenalty> mAllPenalties;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            mHasCheckedForObstacles = false;
            mHasCheckedForPenalties = false;

            // 初始注册组件类型，GetComponentLookup 记录 TypeIndex 但不立即读数据
            mAllObstacles = state.GetComponentLookup<ASObstacle>();
            mAllPenalties = state.GetComponentLookup<ASPenalty>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧刷新 Lookup 缓存，使其指向当前帧的 Chunk 数据
            // （Structural Change 会导致 Chunk 重组，不刷新会读到过期指针）
            mAllObstacles.Update(ref state);
            mAllPenalties.Update(ref state);

            // 遍历网格 Entity（通常只有一个，但框架支持多个网格共存）
            foreach (var (grid, transform, entity) in SystemAPI.Query<RefRW<ASGrid>, LocalTransform>().WithEntityAccess())
            {
                bool gridRefreshed = false;

                // ── 检测刷新请求 ──
                // RefreshGrid 由外部设置（如按 A 键、障碍物移动后）
                // 重置后重新执行一次完整的障碍物扫描
                if (grid.ValueRO.RefreshGrid)
                {
                    grid.ValueRW.RefreshGrid = false;
                    mHasCheckedForObstacles = false;
                    mHasCheckedForPenalties = false;
                    gridRefreshed = true;
                }

                // ── 执行障碍物 + 惩罚扫描 ──
                // 只在 mHasCheckedForObstacles = false 时执行，防止每帧重复扫描
                if (!mHasCheckedForObstacles)
                {
                    DynamicBuffer<ASGridNodesBufferComponent> nodes = SystemAPI.GetBuffer<ASGridNodesBufferComponent>(entity);
                    CheckForObstacles(grid.ValueRO, nodes, transform, ref state);

                    mHasCheckedForObstacles = true;
                    // 扫描完成后通知岛屿系统重新计算（岛屿依赖 IsAccessible 数据）
                    grid.ValueRW.RefreshIslands = true;
                }

                // ── 网格刷新后强制所有代理重新寻路 ──
                // 场景：障碍物移动导致原有路径不再可行，需要全局重新规划
                // 仅当 RecalcPathsAfterGridRefresh = true 且本次确实发生了刷新时才触发
                if (grid.ValueRO.RecalcPathsAfterGridRefresh && gridRefreshed)
                {
                    foreach (var (requester, follower) in SystemAPI.Query<RefRW<ASPathfindingRequester>, RefRO<ASPathFollower>>())
                    {
                        // 仅对尚未到达目的地的代理发起重新寻路请求
                        // 已到达的代理路径为空，没有必要重新规划
                        if (!follower.ValueRO.DestinationReached)
                        {
                            requester.ValueRW.RequestGiven = false; // false = 待处理的新请求
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 遍历每个网格节点，通过 OverlapBox 物理查询检测障碍物和惩罚区域。
        ///
        /// ─── OverlapBox 工作原理 ───
        /// Unity Physics 的 OverlapBox 以节点中心为原点，以 NodeSize/2 为半径，
        /// 投出一个轴对齐盒子，返回所有与该盒子相交的碰撞体列表（DistanceHit）。
        /// CollisionFilter 通过位掩码（LayerMask）过滤只关心的层（障碍物层 | 惩罚层），
        /// 避免匹配到不相关的物体（如地面、代理自身）。
        ///
        /// ─── 结果处理逻辑 ───
        /// 对每个命中的碰撞体：
        ///   1. 如果其 Entity 有 ASObstacle 组件 → 节点标记为不可通行
        ///      （OverlapBox 的 bool 返回值本身代表"是否有碰撞" = IsAccessible 的反值）
        ///   2. 如果其 Entity 有 ASPenalty 组件 → 节点惩罚值 += PenaltyValue
        ///      （可以和障碍物惩罚叠加，允许一个区域同时是难走地形 + 障碍物边缘）
        ///
        /// 最后调用 BlurPenalties 对所有惩罚值做平滑扩散。
        /// </summary>
        private void CheckForObstacles(ASGrid _grid, DynamicBuffer<ASGridNodesBufferComponent> _nodes, LocalTransform _transform, ref SystemState _state)
        {
            // d = max(NodeXCount, NodeYCount)，用于一维索引计算
            // 一维数组索引公式（主轴对齐）：
            //   NodeXCount > NodeYCount（宽>高）→ index = x + y * NodeXCount
            //   NodeYCount >= NodeXCount（高>=宽）→ index = y + x * NodeYCount
            // 统一用 d 代替较大的那个值，公式化简为：
            //   宽>高：x + y * d     高>=宽：y + x * d
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            for (int x = 0; x < _grid.NodeXCount; x++)
            {
                for (int y = 0; y < _grid.NodeYCount; y++)
                {
                    // 计算当前节点在一维缓冲区中的索引
                    int iindex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        iindex = x + y * d;
                    else iindex = y + x * d;

                    // 每次刷新前先将节点还原为默认状态（可通行、无惩罚）
                    // 否则旧障碍物移走后，节点仍保留上次扫描留下的 IsAccessible = false
                    var nn = _nodes[iindex];
                    nn.Node.IsAccessible = true;
                    nn.Node.Penalty = 0;
                    _nodes[iindex] = nn;

                    // ── 获取当前帧的 Physics World ──
                    // PhysicsWorldSingleton 每帧由 Unity Physics 系统更新，
                    // 必须在此处重新查询而不能缓存（缓存的引用在下一帧可能失效）
                    EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();
                    EntityQuery singletonQuery = _state.EntityManager.CreateEntityQuery(builder);
                    var physicsWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
                    singletonQuery.Dispose();

                    // ── 构造 OverlapBox 参数 ──
                    // extents：盒子的半边长（等于节点半径），使检测盒与节点格子完全重合
                    // worldCoords：节点中心的世界坐标（GridUtility 将网格坐标转换为世界坐标）
                    float3 extents = new float3(1, 1, 1) * (_grid.NodeSize / 2);
                    float3 worldCoords = GridUtility.GridCoordsToWorld(x, y, _grid, _transform);

                    NativeList<DistanceHit> results = new NativeList<DistanceHit>(Allocator.Temp);

                    // ── OverlapBox 物理查询 ──
                    // 返回值：true = 有碰撞体与盒子相交（即有障碍物），false = 节点区域内无障碍
                    // CollisionFilter 说明：
                    //   BelongsTo = ~0u        → 本次查询不属于任何特定层（全 1 = 与任意层都能交互）
                    //   CollidesWith = 障碍物层 | 惩罚层  → 只检测这两个层上的碰撞体
                    //   GroupIndex = 0         → 不使用分组过滤
                    // 注意：OverlapBox 的 bool 返回值只要 CollidesWith 内任意层有碰撞就返回 true，
                    //       即使命中的碰撞体只是惩罚层而非障碍物层，也会返回 true。
                    //       所以不能直接用返回值决定 IsAccessible，需要再遍历 results 区分
                    bool isAccessible = !physicsWorld.OverlapBox(worldCoords, quaternion.identity, extents, ref results, new CollisionFilter()
                    {
                        BelongsTo = ~0u,
                        CollidesWith = (uint)_grid.InaccessibleLMIndex | (uint)_grid.PenaltyLMIndex,
                        GroupIndex = 0
                    });

                    // ── 处理所有命中的碰撞体 ──
                    for (int i = 0; i < results.Length; i++)
                    {
                        // ── 障碍物处理 ──
                        // 只有碰撞体对应的 Entity 上有 ASObstacle 才标记为不可通行
                        // 此时 isAccessible = false（因为障碍物层命中导致 OverlapBox 返回 true）
                        if (mAllObstacles.HasComponent(results[i].Entity))
                        {
                            int index;
                            if (_grid.NodeXCount > _grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            ASObstacle obstacle = mAllObstacles[results[i].Entity];
                            var newNodes = _nodes[index];
                            // IsAccessible 由 OverlapBox 的 bool 反值决定（有障碍物命中 → false）
                            newNodes.Node.IsAccessible = isAccessible;
                            // PenaltyOnObstacleValue：障碍物边缘惩罚（经过高斯模糊后会向外扩散，
                            // 让路径自然远离障碍物边缘，而不是贴着边走）
                            newNodes.Node.Penalty = obstacle.PenaltyOnObstacleValue;
                            _nodes[index] = newNodes;
                        }

                        // ── 地形惩罚处理 ──
                        // ASPenalty 不影响可通行性（IsAccessible 保持 true），只增加寻路代价
                        // 用 += 叠加，支持一个节点同时被多个惩罚区域覆盖（代价累加）
                        if (mAllPenalties.HasComponent(results[i].Entity))
                        {
                            int index;
                            if (_grid.NodeXCount > _grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            ASPenalty penalty = mAllPenalties[results[i].Entity];
                            var newNodes = _nodes[index];
                            // 叠加：惩罚值 = 原有惩罚（如障碍物边缘）+ 地形惩罚
                            newNodes.Node.Penalty = penalty.PenaltyValue + newNodes.Node.Penalty;
                            _nodes[index] = newNodes;
                        }
                    }

                    results.Dispose();
                }
            }

            // 扫描完成后对惩罚值进行高斯模糊（blurSize=3 → 7×7 核）
            // 目的：让集中在障碍物/惩罚区域的高惩罚值向外平滑扩散，
            // 使 A* 在规划路径时会自然远离障碍物边缘
            BlurPenalties(3, _grid, _nodes);
        }
        
        /// <summary>
        /// 对所有节点的惩罚值进行可分离高斯模糊（两次 1D 箱式滤波近似）。
        ///
        /// ──── 算法原理 ────
        ///
        /// 二维高斯模糊可以分解为两次独立的一维高斯模糊（可分离性）：
        ///   二维高斯(x,y) = 一维高斯(x) × 一维高斯(y)
        ///
        /// 一维高斯模糊可以用"箱式滤波"（Box Filter）近似：
        ///   对每个元素，取其左右 blurSize 个邻居的平均值作为输出
        ///   即：output[x] = mean( input[x-blurSize .. x+blurSize] )
        ///
        /// 所以完整步骤：
        ///   第一遍（水平）：horizontalPass[x,y] = mean( Penalty[x-blur..x+blur, y] )
        ///   第二遍（垂直）：verticalPass[x,y]   = mean( horizontalPass[x, y-blur..y+blur] )
        ///   最终结果：blurred[x,y] = verticalPass[x,y] / kernelSize²
        ///
        /// ──── 滑动窗口优化（为什么不直接循环求和）────
        ///
        /// 朴素方式：对每个节点循环遍历窗口内所有元素求和 → O(N * kernelSize²)
        /// 滑动窗口：维护一个"当前窗口总和"，向右移动时：
        ///   新窗口和 = 旧窗口和 - 最左边离开的值 + 最右边新进来的值
        ///   每次只做 1 次减法 + 1 次加法 → O(N)
        ///
        /// ──── 边界处理（Clamp）────
        ///
        /// 节点 x=0 时，窗口左侧 [x-blurSize..-1] 超出网格范围。
        /// 通过 math.clamp 将越界索引夹紧到边界节点（0 或 NodeXCount-1），
        /// 等效于"边界像素重复填充"（Clamp To Edge），这会导致边界区域的模糊值偏低，
        /// 但对寻路结果影响可忽略（网格边界通常是不可通行区域）。
        ///
        /// ──── 为什么最终除以 kernelSize² ────
        ///
        /// 水平方向箱式滤波对一行求和（未除以 kernelSize），
        /// 垂直方向对列求和（未除以 kernelSize），
        /// 最终等效于二维核内所有元素之和 = 一维和 × 一维和，
        /// 所以必须除以 kernelSize × kernelSize 才能得到平均值（归一化）。
        ///
        /// ──── 注意：当前代码只在垂直第二遍的 y>=1 循环中写回节点 ────
        /// y=0 那行在初始化窗口时未写回，导致第一行的惩罚值不被模糊。
        /// 对寻路影响极小，但若需要精确可在第二遍后补写 y=0 的结果。
        /// </summary>
        /// <param name="_blurSize">模糊半径，kernel 大小 = 2*blurSize+1（默认 3 → 7×7 核）</param>
        void BlurPenalties(int _blurSize, ASGrid _grid, DynamicBuffer<ASGridNodesBufferComponent> _nodes)
        {
            int kernelSize = _blurSize * 2 + 1;  // 核大小（blurSize=3 时为 7）
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            // 两个中间结果数组（按节点总数分配，用同一套一维索引访问）
            NativeArray<int> horizontalPass = new NativeArray<int>(_grid.NodeXCount * _grid.NodeYCount, Allocator.TempJob);
            NativeArray<int> verticalPass = new NativeArray<int>(_grid.NodeXCount * _grid.NodeYCount, Allocator.TempJob);

            // ════════════════════════════════════════
            // 第一遍：水平方向箱式滤波
            // 对每行 y，从左到右滑动窗口，计算每个 x 位置的水平方向区间和
            // ════════════════════════════════════════
            for (int y = 0; y < _grid.NodeYCount; y++)
            {
                // ── 初始化 x=0 的滑动窗口 ──
                // 累加窗口范围 [0-blurSize, 0+blurSize] 内所有节点的惩罚值
                // 左侧超出边界（x<0）时，clamp 到 0（重复边界节点）
                for (int x = -_blurSize; x <= _blurSize; x++)
                {
                    int clampX = math.clamp(x, 0, _blurSize);  // 左侧边界 clamp（右侧用 _blurSize 截断，因为 x<=_blurSize）

                    // 被采样的源节点索引
                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = clampX + y * d;
                    else index = y + clampX * d;

                    // 写入目标：x=0 位置的水平过滤结果
                    int hpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        hpIndex = 0 + y * d;
                    else hpIndex = y + 0 * d;

                    horizontalPass[hpIndex] += _nodes[index].Node.Penalty;
                }

                // ── 滑动窗口：从 x=1 向右推进 ──
                // 每次：新窗口和 = 上一个窗口和 - 最左边离开的元素 + 最右边新加入的元素
                for (int x = 1; x < _grid.NodeXCount; x++)
                {
                    // 离开窗口的节点：x - blurSize - 1（超出左边界时 clamp 到 0）
                    // clamp 到 NodeXCount（注意：不是 NodeXCount-1）是安全的，
                    // 因为该索引仅用于"取减数"，超界时被 clamp 到边界意味着重复减同一个节点
                    int removeIndex = math.clamp(x - _blurSize - 1, 0, _grid.NodeXCount);
                    // 进入窗口的节点：x + blurSize（超出右边界时 clamp 到最后一个节点）
                    int addIndex = math.clamp(x + _blurSize, 0, _grid.NodeXCount - 1);

                    // 离开节点的一维索引
                    int removehpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        removehpIndex = removeIndex + y * d;
                    else removehpIndex = y + removeIndex * d;

                    // 进入节点的一维索引
                    int addhpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        addhpIndex = addIndex + y * d;
                    else addhpIndex = y + addIndex * d;

                    // 当前节点 x 的水平过滤结果目标索引
                    int hpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        hpIndex = x + y * d;
                    else hpIndex = y + x * d;

                    // 上一个节点（x-1）的水平过滤结果索引（用于继承上一窗口和）
                    int prevhpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        prevhpIndex = (x - 1) + y * d;
                    else prevhpIndex = y + (x - 1) * d;

                    // 核心滑动公式：新窗口和 = 上一窗口和 - 离开的节点惩罚 + 进入的节点惩罚
                    horizontalPass[hpIndex] = horizontalPass[prevhpIndex]
                        - _nodes[removehpIndex].Node.Penalty
                        + _nodes[addhpIndex].Node.Penalty;
                }
            }

            // ════════════════════════════════════════
            // 第二遍：垂直方向箱式滤波（以 horizontalPass 为输入）
            // 对每列 x，从上到下滑动窗口，计算每个 y 位置的垂直方向区间和
            // 最终 verticalPass[x,y] = 水平和 的垂直和 = 二维 7×7 核内惩罚之和（未归一化）
            // ════════════════════════════════════════
            for (int x = 0; x < _grid.NodeXCount; x++)
            {
                // ── 初始化 y=0 的滑动窗口 ──
                // 累加 horizontalPass 中 [0-blurSize, 0+blurSize] 行的值
                for (int y = -_blurSize; y <= _blurSize; y++)
                {
                    int clampY = math.clamp(y, 0, _blurSize);  // 上方超出边界时 clamp 到第 0 行

                    // 源索引（horizontalPass 中第 clampY 行、第 x 列）
                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = x + clampY * d;
                    else index = clampY + x * d;

                    // 目标：y=0 的垂直过滤结果
                    int vpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        vpIndex = x + 0 * d;
                    else vpIndex = 0 + x * d;

                    verticalPass[vpIndex] += horizontalPass[index];
                }

                // ── 滑动窗口：从 y=1 向下推进 ──
                for (int y = 1; y < _grid.NodeYCount; y++)
                {
                    // 当前节点的垂直过滤结果目标索引
                    int vpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        vpIndex = x + y * d;
                    else vpIndex = y + x * d;

                    // 上一行（y-1）的垂直过滤结果索引
                    int prevvpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        prevvpIndex = x + (y - 1) * d;
                    else prevvpIndex = (y - 1) + x * d;

                    // 离开窗口的行索引（y - blurSize - 1，超出上边界时 clamp）
                    int removeIndex = math.clamp(y - _blurSize - 1, 0, _grid.NodeYCount);
                    // 进入窗口的行索引（y + blurSize，超出下边界时 clamp）
                    int addIndex = math.clamp(y + _blurSize, 0, _grid.NodeYCount - 1);

                    // 离开行在 horizontalPass 中的索引
                    int removevpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        removevpIndex = x + removeIndex * d;
                    else removevpIndex = removeIndex + x * d;

                    // 进入行在 horizontalPass 中的索引
                    int addvpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        addvpIndex = x + addIndex * d;
                    else addvpIndex = addIndex + x * d;

                    // 垂直方向滑动：新窗口和 = 上一窗口和 - 离开行的水平和 + 进入行的水平和
                    verticalPass[vpIndex] = verticalPass[prevvpIndex]
                        - horizontalPass[removevpIndex]
                        + horizontalPass[addvpIndex];

                    // ── 归一化并写回节点 ──
                    // verticalPass[vpIndex] = 7×7 核内所有惩罚之和（水平和 × 垂直和）
                    // 除以 kernelSize² = 7×7 = 49，得到平均惩罚值（四舍五入）
                    // 这就是该节点最终的模糊惩罚值
                    int blurredWeight = (int)math.round((float)verticalPass[vpIndex] / (kernelSize * kernelSize));

                    // 目标节点索引
                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = x + y * d;
                    else index = y + x * d;

                    // 写回：只更新 Penalty，IsAccessible 保持障碍物扫描时的结果不变
                    ASNode newNode = _nodes[index].Node;
                    newNode.Penalty = blurredWeight;
                    _nodes[index] = new ASGridNodesBufferComponent() { Node = newNode };
                }
            }

            horizontalPass.Dispose();
            verticalPass.Dispose();
        }
    }
}
