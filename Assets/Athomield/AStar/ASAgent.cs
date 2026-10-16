using Unity.Entities;
using Unity.Mathematics;

namespace Athomield.AStar
{
    /// <summary>
    /// 代理（Agent）核心组件，存储寻路代理的基本属性
    /// 挂载此组件的 Entity 会被寻路系统识别为可移动的角色
    /// </summary>
    public struct ASAgent : IComponentData
    {
        /// <summary>移动速度（单位/秒）</summary>
        public float MaxSpeed;

        /// <summary>
        /// 代理的物理半径，用于：
        /// 1. 判断是否到达路径点
        /// 2. RVO 避障碰撞半径计算
        /// </summary>
        public float Radius;

        /// <summary>代理的高度，用于确定 Y 轴悬浮高度（Position.y = Height / 2）</summary>
        public float Height;

        /// <summary>转向速度（度/秒），控制旋转朝向目标的平滑程度</summary>
        public float TurnSpeed;

        /// <summary>
        /// 路径点切换距离阈值：代理距离当前路径点小于此值时，切换到下一个点。
        /// 推荐设置为 Grid.NodeSize，设为负值则自动使用 Grid.NodeSize
        /// </summary>
        public float NextTargetTouchDistance;

        /// <summary>
        /// 是否在到达目的地后忽略避障。
        /// 设为 true 时，其他代理不会因本代理停止而绕开它（减少运算开销）
        /// </summary>
        public bool IgnoreAvoidanceWhenDestReached;

        /// <summary>停止距离：代理距最终目的地小于此值时，视为到达目标</summary>
        public float StoppingDistance;

        /// <summary>当前速度向量（由系统每帧更新，记录实际移动方向和大小）</summary>
        public float3 CurrentVelocity;
    }

    /// <summary>
    /// 路径跟随组件，记录代理沿路径移动的状态。
    /// 与 ASPathBufferComponent 配合使用，追踪当前目标路径点索引
    /// </summary>
    public struct ASPathFollower : IComponentData
    {
        /// <summary>是否有可用路径（路径规划成功后由系统置为 true）</summary>
        public bool PathAvailable;

        /// <summary>当前追踪的路径点索引（在 ASPathBufferComponent 中的位置）</summary>
        public int TargetIndex;

        /// <summary>是否已到达最终目的地</summary>
        public bool DestinationReached;
    }

    /// <summary>
    /// 寻路请求组件，代理通过此组件向寻路系统提交请求。
    /// 将 RequestGiven 置为 false 即可触发新的寻路请求
    /// </summary>
    public struct ASPathfindingRequester : IComponentData
    {
        /// <summary>目标世界坐标</summary>
        public float3 Destination;

        /// <summary>
        /// 请求是否已提交。
        /// false = 待处理的新请求，系统处理后置为 true
        /// </summary>
        public bool RequestGiven;

        /// <summary>
        /// 对应的寻路操作 Entity（由 PathfindingRequestSystem 自动创建和管理）。
        /// 不要手动设置此值
        /// </summary>
        public Entity RequestEntity;

        /// <summary>
        /// 重试冷却（秒）。上一次寻路判定"目标不可达"时由 AgentMovementSystem 置为冷却时长，
        /// 冷却期内即使目标移动也不再自动重发请求 —— 否则不可达目标会每帧触发一次
        /// 全区域 A* 搜索（跨岛场景下这是主要卡顿源）。归零后恢复正常跟随。
        /// </summary>
        public float RetryCooldown;
    }

    /// <summary>
    /// 持续跟随目标组件。挂载此组件后，代理会持续追踪目标 Entity 的位置并重新寻路。
    /// 目标 Entity 必须拥有 LocalTransform 组件
    /// </summary>
    public struct ASConstantTargetFollower : IComponentData
    {
        /// <summary>追踪的目标 Entity（必须有 LocalTransform 组件）</summary>
        public Entity Target;

        /// <summary>
        /// 是否在目标静止时也重新寻路。
        /// 不推荐设为 true，除非场景地形频繁变化（性能开销大）
        /// </summary>
        public bool RecalculateOnStationary;

        /// <summary>目标上一帧的位置，用于判断目标是否移动（避免不必要的重新寻路）</summary>
        public float3 TargetLastPosition;
    }

    /// <summary>
    /// RVO（互惠速度障碍）避障组件。
    /// 挂载此组件后，代理会参与多代理碰撞避免计算（ORCA 算法）。
    /// 不挂载则代理不参与避障，也不被其他代理避让
    /// </summary>
    public struct ASAvoidanceAgent : IComponentData
    {
        /// <summary>
        /// 时间窗口（秒）：预测未来多长时间内的碰撞。
        /// 值越大，代理越早开始规避；推荐值为 2
        /// </summary>
        public float TimeHorizon;
    }

    /// <summary>
    /// 邻居代理缓冲组件，存储 RVO 避障计算时需要考虑的邻居代理列表。
    /// 由 AgentMovementSystem 通过 KNN 算法自动填充，无需手动设置。
    /// InternalBufferCapacity(0) 表示数据存储在堆内存中（不限制大小）
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct ASAgentNeighboursBufferComponent : IBufferElementData
    {
        /// <summary>邻居代理的 Entity 引用</summary>
        public Entity Agent;
    }

    /// <summary>
    /// 测试用球体标记组件（用于 Demo 场景中的目标球体识别）
    /// </summary>
    public struct ASTestSphere : IComponentData
    {

    }
}
