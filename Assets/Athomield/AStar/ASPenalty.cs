using Unity.Entities;

namespace Athomield.AStar
{
    /// <summary>
    /// 地形惩罚组件，需挂载在已设置正确 LayerMask 的碰撞体 Entity 上。
    /// 与 ASObstacle 的区别：ASObstacle 使节点完全不可通行，
    /// 而 ASPenalty 仅增加节点代价，寻路算法会尽量绕开高惩罚区域但仍可通行。
    /// 典型用途：道路（低惩罚）vs 草地（高惩罚），引导代理优先走道路
    /// </summary>
    public struct ASPenalty : IComponentData
    {
        /// <summary>
        /// 惩罚值，叠加到被此碰撞体覆盖的网格节点上。
        /// 经过 CheckObstaclesSystem 的高斯模糊后，惩罚会平滑扩散到周边节点。
        /// 值越大，A* 算法越倾向于选择其他路径
        /// </summary>
        public int PenaltyValue;
    }
}
