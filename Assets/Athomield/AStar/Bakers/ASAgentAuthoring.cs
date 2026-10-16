using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace Athomield.AStar
{
    /// <summary>
    /// 寻路代理的 Authoring 组件（挂载在 Unity GameObject 上）。
    /// 在 SubScene 烘焙（Bake）阶段，由 ASAgentBaker 将 Inspector 参数转换为 ECS 组件。
    /// 支持可选的初始目的地和 RVO 避障功能
    /// </summary>
    public class ASAgentAuthoring : MonoBehaviour
    {
        /// <summary>代理移动速度（世界单位/秒）</summary>
        public float MaxSpeed;

        /// <summary>代理物理半径（用于路径点判断和 RVO 碰撞）</summary>
        public float Radius;

        /// <summary>代理高度（代理 Y 轴中心 = Height/2）</summary>
        public float Height;

        [Tooltip("代理的转向速度（度/秒）")]
        /// <summary>转向速度（度/秒）</summary>
        public float TurnSpeed;

        [Tooltip("代理到达最终目的地所需的最小距离，小于此值视为已到达")]
        /// <summary>停止距离：与最终目的地的距离小于此值时视为到达</summary>
        public float StoppingDistance;

        [Tooltip("切换到下一个路径点所需的最小距离。设为负值时自动使用 Grid.NodeSize")]
        /// <summary>
        /// 路径中间点切换距离阈值。
        /// 负值 = 自动使用 Grid.NodeSize
        /// </summary>
        public float NextTargetTouchDistance;

        [Tooltip("是否启用 RVO 避障。关闭后该代理既不会主动避让其他代理，也不会被其他代理避让")]
        /// <summary>
        /// 是否启用 RVO 避障。
        /// true = 添加 ASAvoidanceAgent 和 ASAgentNeighboursBufferComponent 组件
        /// </summary>
        public bool ActivateAvoidance;

        [Tooltip("到达目的地后，其他代理不再绕避本代理（减少静止状态的避障计算开销）")]
        /// <summary>到达目的地后其他代理不再绕避本代理（优化停止状态的性能）</summary>
        public bool IgnoreAvoidanceWhenDestReached;

        /// <summary>是否有初始目的地（在烘焙时自动提交第一次寻路请求）</summary>
        public bool HasIntitalDestination;

        /// <summary>初始目的地世界坐标（仅当 HasIntitalDestination = true 时有效）</summary>
        public Vector3 InitialDestination;

        /// <summary>
        /// Baker 内部类：将 ASAgentAuthoring 的参数烘焙为 ECS 组件。
        /// 在 SubScene 烘焙阶段自动执行，生成静态 ECS 数据
        /// </summary>
        public class ASAgentBaker : Baker<ASAgentAuthoring>
        {
            public override void Bake(ASAgentAuthoring authoring)
            {
                // 获取或创建对应的 Entity（Dynamic = 运行时可移动）
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                // 核心代理参数
                ASAgent agent = new ASAgent()
                {
                    MaxSpeed = authoring.MaxSpeed,
                    Radius = authoring.Radius,
                    Height = authoring.Height,
                    TurnSpeed = authoring.TurnSpeed,
                    NextTargetTouchDistance = authoring.NextTargetTouchDistance,
                    IgnoreAvoidanceWhenDestReached = authoring.IgnoreAvoidanceWhenDestReached,
                    StoppingDistance = authoring.StoppingDistance
                };

                // 路径跟随状态初始化（尚无路径）
                ASPathFollower follower = new ASPathFollower()
                {
                    PathAvailable = false,
                    TargetIndex = 0
                };

                // 寻路请求初始化
                // RequestGiven = !HasIntitalDestination 意思：
                //   有初始目的地 → RequestGiven=false → 立即发起请求
                //   无初始目的地 → RequestGiven=true → 不自动请求
                ASPathfindingRequester requester = new ASPathfindingRequester()
                {
                    Destination = new float3(authoring.InitialDestination.x, authoring.InitialDestination.y, authoring.InitialDestination.z),
                    RequestEntity = Entity.Null,
                    RequestGiven = !authoring.HasIntitalDestination,
                };

                // RVO 避障参数（TimeHorizon=2 秒为推荐默认值）
                ASAvoidanceAgent avoidance = new ASAvoidanceAgent()
                {
                    TimeHorizon = 2
                };

                // 添加必要的核心组件
                AddComponent(entity, agent);
                AddComponent(entity, follower);
                AddComponent(entity, requester);
                AddBuffer<ASPathBufferComponent>(entity); // 路径点缓冲区（初始为空）

                // 如果启用避障，额外添加避障相关组件
                if (authoring.ActivateAvoidance)
                {
                    AddComponent(entity, avoidance);
                    AddBuffer<ASAgentNeighboursBufferComponent>(entity); // 邻居缓冲区（由系统自动填充）
                }
            }
        }
    }
}
