using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;

namespace Athomield.AStar.Demos
{
    /// <summary>
    /// GroupTarget 场景中目标球的控制器（普通 MonoBehaviour，挂载在场景外的 GameObject 上）。
    /// 通过方向键移动 ECS 中的 ASTestSphere Entity，所有挂载 ASConstantTargetFollower 的代理
    /// 会自动检测到目标位置变化并重新寻路。
    /// </summary>
    public class MoveSphere : MonoBehaviour
    {
        private EntityManager EM;
        private Entity SphereEntity;

        private float Speed = 10;

        void Start()
        {
            // 延迟 1 秒初始化，等待 ECS World 和 SubScene 完成烘焙
            StartCoroutine(Init());
        }

        IEnumerator Init()
        {
            yield return new WaitForSeconds(1);
            EM = World.DefaultGameObjectInjectionWorld.EntityManager;
            // 查询场景中唯一的 ASTestSphere 单例 Entity
            var q = EM.CreateEntityQuery(ComponentType.ReadOnly<ASTestSphere>());
            SphereEntity = q.GetSingletonEntity();
            q.Dispose();
        }

        void Update()
        {
            EM = World.DefaultGameObjectInjectionWorld.EntityManager;

            if (EM.Exists(SphereEntity))
            {
                LocalTransform lt = EM.GetComponentData<LocalTransform>(SphereEntity);

                // 方向键控制目标球在 XZ 平面移动
                if (Input.GetKey(KeyCode.RightArrow))
                {
                    lt.Position.x += Time.deltaTime * Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.LeftArrow))
                {
                    lt.Position.x += Time.deltaTime * -Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.UpArrow))
                {
                    lt.Position.z += Time.deltaTime * Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.DownArrow))
                {
                    lt.Position.z += Time.deltaTime * -Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }
            }
        }
    }
}
