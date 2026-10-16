using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

namespace Athomield.AStar.Demos
{
    /// <summary>
    /// 存储代理 Prefab 对应的 ECS Entity 引用，供 GTSpawnerSystem 读取并批量实例化代理。
    /// </summary>
    public struct GTAgentPrefabComponent : IComponentData
    {
        public Entity Value;
    }

    /// <summary>
    /// GroupTarget 场景的代理生成器 Authoring 组件。
    /// 在 SubScene 烘焙阶段将 mAgentPrefab 转换为 ECS Entity 引用，
    /// 由 GTSpawnerSystem 在运行时读取并批量生成持续追踪目标球的代理。
    /// </summary>
    public class GTSpawner : MonoBehaviour
    {
        /// <summary>代理 Prefab（需包含 ASAgentAuthoring 和 ASConstantTargetFollower 所需组件）</summary>
        public GameObject mAgentPrefab;

        public class GTSpawnerBaker : Baker<GTSpawner>
        {
            public override void Bake(GTSpawner authoring)
            {
                // WorldSpace：Spawner 自身不需要跟随父节点变换
                var entity = GetEntity(TransformUsageFlags.WorldSpace);
                // Dynamic：代理在运行时会移动，需要动态 Transform
                var agentEntity = GetEntity(authoring.mAgentPrefab, TransformUsageFlags.Dynamic);

                var prefCom = new GTAgentPrefabComponent()
                {
                    Value = agentEntity,
                };

                AddComponent(entity, prefCom);
            }
        }
    }
}
