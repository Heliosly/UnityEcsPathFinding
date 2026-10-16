using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;

/// ==================== 第1部分：组件定义 ====================

/// <summary>
/// 立方体预制体引用组件 - 存储已经烘焙好的 Entity Prefab 的引用
/// 注意：这个组件存的是 Entity 类型的引用，不是 GameObject 引用
/// </summary>
public struct CubePrefab : IComponentData
{
    /// <summary>Entity 预制体的引用，可用于 Instantiate 批量生成</summary>
    public Entity Value;
}

/// <summary>
/// 生成器参数组件 - 存储生成逻辑需要的参数
/// </summary>
public struct SpawnerData : IComponentData
{
    /// <summary>需要生成的总数量</summary>
    public int Count;
    
    /// <summary>分布半径（圆形区域）</summary>
    public float Radius;
    
    /// <summary>是否已经生成过（避免每帧重复生成）</summary>
    public bool HasSpawned;
}

/// ==================== 第2部分：作者化组件 ====================

/// <summary>
/// 生成器作者化组件 - 在 Inspector 中配置生成器的参数
/// </summary>
public class SpawnerAuthoring : MonoBehaviour
{
    [Header("预制体配置")]
    [Tooltip("需要生成的立方体预制体（必须包含 RotationSpeedAuthoring 组件）")]
    public GameObject cubePrefab;
    
    [Header("生成参数")]
    [Tooltip("生成数量")]
    public int count = 1000;
    
    [Tooltip("分布半径（米）")]
    public float radius = 10f;

    /// <summary>
    /// Baker 类 - 将 MonoBehaviour 中的配置数据转换成 ECS 组件
    /// </summary>
    public class Baker : Baker<SpawnerAuthoring>
    {
        public override void Bake(SpawnerAuthoring authoring)
        {
            // ===== 步骤1：转换预制体 =====
            // 将用户拖入的 GameObject 预制体（Cube Prefab）转换为 ECS 实体预制体
            // TransformUsageFlags.Dynamic：生成的实体会移动/旋转，需要动态 Transform
            // 这一步是关键！在编辑时完成转换，运行时不需要再做
            var prefabEntity = GetEntity(authoring.cubePrefab, TransformUsageFlags.Dynamic);
            
            // ===== 步骤2：创建生成器实体 =====
            // 为当前挂载 SpawnerAuthoring 的 GameObject 创建一个实体
            // TransformUsageFlags.None：生成器本身不需要移动，没有 Transform
            var entity = GetEntity(TransformUsageFlags.None);
            
            // ===== 步骤3：添加组件 =====
            // 添加预制体引用组件
            AddComponent(entity, new CubePrefab { Value = prefabEntity });
            
            // 添加生成参数组件
            AddComponent(entity, new SpawnerData
            {
                Count = authoring.count,
                Radius = authoring.radius,
                HasSpawned = false   // 初始状态未生成
            });
        }
    }
}

/// ==================== 第3部分：系统（运行时逻辑） ====================

/// <summary>
/// 生成器系统 - 在运行时读取烘焙好的数据，实际执行批量生成
/// </summary>
public partial struct SpawnerSystem : ISystem
{
    /// <summary>
    /// 第一帧更新时调用
    /// </summary>
    public void OnUpdate(ref SystemState state)
    {
        // 查询所有包含 SpawnerData + CubePrefab 的实体
        // RefRW<SpawnerData>：读写，因为需要修改 HasSpawned 标志位
        // CubePrefab：只读，预制体引用不需要修改
        foreach (var (spawnerData, prefabRef) in 
                 SystemAPI.Query<RefRW<SpawnerData>, CubePrefab>())
        {
            // 如果已经生成过，跳过（避免每帧重复生成）
            if (spawnerData.ValueRO.HasSpawned) continue;

            // 获取 EntityManager 引用，用于操作实体
            var entityManager = state.EntityManager;
            
            // 获取预制体引用（Entity 类型的）
            var prefab = prefabRef.Value;
            
            // 获取生成参数
            int count = spawnerData.ValueRO.Count;
            float radius = spawnerData.ValueRO.Radius;

            // ===== 批量生成实体 =====
            // Instantiate 是高效率的实体复制操作，本质是内存拷贝，远快于 new GameObject
            for (int i = 0; i < count; i++)
            {
                // 实例化实体预制体（创建新的实体实例）
                var newEntity = entityManager.Instantiate(prefab);
                
                // 计算随机位置（圆形分布算法）
                // 随机角度：0 到 2π
                float angle = UnityEngine.Random.Range(0, math.PI * 2);
                
                // 随机半径：使用 sqrt 让分布更均匀（否则会聚集在圆心附近）
                // 原理：让点在圆面积内均匀分布，半径的概率密度需要与面积成正比
                float r = math.sqrt(UnityEngine.Random.Range(0f, radius * radius));
                
                // 计算最终坐标
                float x = math.cos(angle) * r;
                float z = math.sin(angle) * r;
                
                // 设置实体的 Transform 位置
                // ECS 使用 LocalTransform 而不是传统 Transform 组件
                entityManager.SetComponentData(newEntity, new LocalTransform
                {
                    Position = new float3(x, 0, z),      // 位置
                    Rotation = quaternion.identity,      // 初始无旋转
                    Scale = 1f                           // 缩放为1
                });
            }
            
            // 标记已生成，避免下一帧再次执行
            spawnerData.ValueRW.HasSpawned = true;
            
            // 输出日志确认生成成功
            UnityEngine.Debug.Log($"生成完成！共生成 {count} 个立方体，分布半径 {radius} 米");
        }
    }
}