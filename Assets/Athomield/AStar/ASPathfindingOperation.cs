using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

namespace Athomield.AStar
{
    /// <summary>
    /// 一次寻路操作的输入数据组件，由 PathfindingRequestSystem 创建并挂载到临时 Entity 上，
    /// 寻路完成后该 Entity 由 AgentMovementSystem 销毁
    /// </summary>
    public struct ASPathfindingOperation : IComponentData
    {
        /// <summary>起始世界坐标（代理当前位置）</summary>
        public float3 StartPoint;

        /// <summary>目标世界坐标（代理期望到达的位置）</summary>
        public float3 TargetPoint;
    }

    /// <summary>
    /// 路径点缓冲组件，存储 A* 算法规划出的完整路径（有序路径点列表）。
    /// 同时挂载在：
    /// 1. 寻路操作 Entity（临时）：PathfindingSystem 写入结果
    /// 2. 代理 Entity（持久）：AgentMovementSystem 从操作 Entity 复制到此处后跟随
    /// InternalBufferCapacity(0) 表示所有数据存储在堆内存中，路径长度不受限制
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct ASPathBufferComponent : IBufferElementData
    {
        /// <summary>路径点的世界坐标（按 起点→终点 顺序排列）</summary>
        public float3 Point;
    }

    /// <summary>
    /// 寻路操作结果组件，与 ASPathfindingOperation 挂载在同一临时 Entity 上。
    /// PathfindingSystem 写入结果，AgentMovementSystem 读取后处理路径并销毁该 Entity
    /// </summary>
    public struct ASPathfindingResult : IComponentData
    {
        /// <summary>是否找到路径（即使目标不可达也可能为 true，此时返回尽量接近目标的路径）</summary>
        public bool PathFound;

        /// <summary>寻路是否已完成（完成后 AgentMovementSystem 才会读取结果）</summary>
        public bool FinishedSearch;

        /// <summary>
        /// 目标是否"在当前可达区域内不可达"（跨岛 / 被障碍封死 / 搜索预算耗尽）。
        /// 为 true 时 PathFound 可能仍为 true（best-effort 路径：走到离目标最近的节点），
        /// 但 AgentMovementSystem 会据此给代理设置重试冷却，
        /// 避免"目标不可达 → 每帧重发请求 → 每帧全岛搜索"造成的卡顿。
        /// </summary>
        public bool Unreachable;
    }

    /// <summary>
    /// 寻路请求队列缓冲组件，挂载在 Grid Entity 上作为全局请求队列。
    /// PathfindingRequestSystem 将代理的寻路请求追加到此队列，
    /// 然后按 MaxPathfindingOpsPerFrame 的限制逐帧取出并创建操作 Entity
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct ASPathfindingOperationsBuffer : IBufferElementData
    {
        /// <summary>发起寻路请求的代理 Entity（用于寻路完成后将路径写回该代理）</summary>
        public Entity RequestingAgentEntity;

        /// <summary>该代理的寻路操作数据（起点和终点）</summary>
        public ASPathfindingOperation Operation;
    }
}
