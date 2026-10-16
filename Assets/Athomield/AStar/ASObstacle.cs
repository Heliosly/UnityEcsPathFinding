using Unity.Entities;
using Unity.Mathematics;

namespace Athomield.AStar
{
    /// <summary>
    /// 障碍物组件，需挂载在已设置正确 LayerMask 的碰撞体 Entity 上。
    /// CheckObstaclesSystem 会通过物理重叠检测找到此组件，
    /// 并将对应网格节点标记为不可通行（IsAccessible = false）。
    /// 同时可设置边缘惩罚值，使代理不紧贴障碍物边缘行走
    /// </summary>
    public struct ASObstacle : IComponentData
    {
        /// <summary>
        /// 障碍物边缘惩罚值：在障碍物覆盖的节点上额外叠加此惩罚，
        /// 经高斯模糊后扩散到周边节点，让寻路路径自然远离障碍物边缘。
        /// 设为 0 表示不需要边缘惩罚
        /// </summary>
        public int PenaltyOnObstacleValue;
    }
}
