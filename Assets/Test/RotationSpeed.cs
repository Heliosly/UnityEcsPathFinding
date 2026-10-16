using Unity.Entities;

/// <summary>
/// 旋转速度组件 - 纯数据，只存储数值，不包含任何逻辑
/// IComponentData 是 ECS 中最基础的数据组件，结构体类型，性能极高
/// </summary>
public struct RotationSpeed : IComponentData
{
    /// <summary>每秒旋转的弧度数（弧度制，360度 = 2π 弧度）</summary>
    public float RadiansPerSecond;
}