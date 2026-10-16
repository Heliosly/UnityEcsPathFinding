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
    /// 测试目标球的 Authoring 组件，挂载在 GroupTarget Demo 场景中的目标球 GameObject 上。
    ///
    /// 烘焙后，对应 Entity 会被添加 ASTestSphere 标记组件，
    /// 用途：
    ///   1. GTSpawnerSystem 通过 SystemAPI.GetSingletonEntity&lt;ASTestSphere&gt;() 找到目标球 Entity，
    ///      并将其赋值给代理的 ASConstantTargetFollower.Target，使代理持续追踪它。
    ///   2. MoveSphere MonoBehaviour 通过 EntityManager.CreateEntityQuery 找到此 Entity，
    ///      在每帧根据方向键输入更新其 LocalTransform.Position，实现玩家控制目标球移动。
    ///
    /// 注意：场景中只应有一个挂载 SphereAuthoring 的 GameObject，
    /// 否则 GetSingletonEntity 会抛出异常（要求有且仅有一个匹配 Entity）。
    /// </summary>
    public class SphereAuthoring : MonoBehaviour
    {
        /// <summary>
        /// Baker 内部类：将 SphereAuthoring 烘焙为带 ASTestSphere 标记组件的 ECS Entity。
        /// 在 SubScene 烘焙阶段自动执行。
        /// </summary>
        public class SphereBaker : Baker<SphereAuthoring>
        {
            public override void Bake(SphereAuthoring authoring)
            {
                // Dynamic：目标球在运行时由 MoveSphere 脚本驱动移动，需要动态 Transform
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                // ASTestSphere 是纯标记组件（空 struct），无需设置任何字段
                AddComponent(entity, new ASTestSphere() { });
            }
        }
    }
}