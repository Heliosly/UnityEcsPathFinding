using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

/// <summary>
/// 作者化组件 - 挂载在 GameObject 上，让策划/美术可以在 Inspector 面板中配置参数
/// 这个类本身不是 ECS 组件，只是一个"包装器"，通过 Baker 转换成真正的 ECS 组件
/// </summary>
public class RotationSpeedAuthoring : MonoBehaviour
{
    /// <summary>每秒旋转角度（度制），Inspector 中直接填 90 比填 1.57 更直观</summary>
    public float DegreesPerSecond = 90f;

    /// <summary>
    /// Baker 类 - 负责将 MonoBehaviour 转换成 ECS 组件
    /// 必须继承自 Baker<T>，其中 T 是作者化组件的类型
    /// Bake 方法在子场景（SubScene）烘焙时自动调用，不是在运行时调用
    /// </summary>
    public class Baker : Baker<RotationSpeedAuthoring>
    {
        /// <summary>
        /// 烘焙方法 - 在编辑器中自动执行
        /// </summary>
        /// <param name="authoring">挂载在 GameObject 上的作者化组件实例</param>
        public override void Bake(RotationSpeedAuthoring authoring)
        {
            // 为当前 GameObject 创建一个 ECS 实体
            // TransformUsageFlags.Dynamic 表示这个实体会移动/旋转，需要动态 Transform 组件
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            
            // 向该实体添加 RotationSpeed 组件，并将角度值转换为弧度制后存入
            // math.radians() 是 Unity.Mathematics 库提供的角度转弧度函数
            AddComponent(entity, new RotationSpeed
            {
                RadiansPerSecond = math.radians(authoring.DegreesPerSecond)
            });
        }
    }
}