using Unity.Entities;
using Unity.Transforms;

/// <summary>
/// 旋转系统 - 负责更新所有带有 LocalTransform + RotationSpeed 组件的实体
/// ISystem 接口：ECS 中的纯逻辑系统，无状态，性能高
/// </summary>
public partial struct RotationSystem : ISystem
{
    /// <summary>
    /// 每帧更新时调用
    /// </summary>
    /// <param name="state">系统状态，可通过它获取 EntityManager、World 等信息</param>
    public void OnUpdate(ref SystemState state)
    {
        // 获取帧间隔时间（秒），确保旋转速度与帧率解耦
        float deltaTime = SystemAPI.Time.DeltaTime;

        // ========== 核心 ECS 查询语法 ==========
        // SystemAPI.Query 自动查询所有同时包含 RefRW<LocalTransform> 和 RotationSpeed 的实体
        // RefRW<LocalTransform>：读写引用，可以修改 Transform 的位置/旋转/缩放
        // RotationSpeed：直接读取即可，因为旋转速度不需要改变
        // 注意：这里没有显式的 where 条件，ECS 自动处理
        foreach (var (transform, speed) in 
                 SystemAPI.Query<RefRW<LocalTransform>, RotationSpeed>())
        {
            // 绕 Y 轴旋转
            // transform.ValueRW 获取可写的 Transform 引用
            // transform.ValueRO 获取只读的 Transform 引用（这里用于获取当前旋转值）
            // RotateY() 返回一个新的 Transform，在原旋转基础上增加 Y 轴旋转
            transform.ValueRW = transform.ValueRO.RotateY(speed.RadiansPerSecond * deltaTime);
        }
    }
}