using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;


namespace Athomield.AStar
{
    /// <summary>
    /// 障碍物的 Authoring 组件，挂载在需要标记为"不可通行"的 GameObject 上。
    ///
    /// 使用前提：
    /// 1. 该 GameObject 必须同时拥有碰撞体（Collider / PhysicsShape）。
    /// 2. 碰撞体所在的 Layer 必须与 GridAuthoring.InaccessibleNodes 的 LayerMask 一致，
    ///    否则 CheckObstaclesSystem 的 OverlapBox 检测将无法命中此障碍物。
    ///
    /// 烘焙后，CheckObstaclesSystem 会在运行时通过物理重叠检测找到此 Entity，
    /// 并将被碰撞体覆盖的网格节点标记为 IsAccessible = false（不可通行）。
    ///
    /// PenaltyOnObstacle 可选设置边缘惩罚值：
    ///   - 值 = 0：节点仅被标记为不可通行，无额外惩罚扩散
    ///   - 值 > 0：在障碍物节点上叠加此惩罚值，经高斯模糊扩散后，
    ///             周边节点也会有一定惩罚值，引导路径自然远离障碍物边缘
    /// </summary>
    public class ASObstacleAuthoring : MonoBehaviour
    {
        /// <summary>
        /// 障碍物边缘惩罚值（可选，0 表示不启用边缘惩罚扩散）。
        /// 值越大，代理路径越倾向于远离此障碍物边缘（但不影响节点的可通行性判断）。
        /// </summary>
        public int PenaltyOnObstacle;

        /// <summary>
        /// Baker 内部类：将 ASObstacleAuthoring 参数烘焙为 ECS 组件 ASObstacle。
        /// 在 SubScene 烘焙阶段自动执行，生成静态 ECS 数据。
        /// </summary>
        public class ASObstacleBaker : Baker<ASObstacleAuthoring>
        {
            public override void Bake(ASObstacleAuthoring authoring)
            {
                // Dynamic：障碍物可能在运行时移动（如移动平台），需要动态 Transform
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                AddComponent(entity, new ASObstacle()
                {
                    PenaltyOnObstacleValue = authoring.PenaltyOnObstacle
                });
            }
        }
    }
}