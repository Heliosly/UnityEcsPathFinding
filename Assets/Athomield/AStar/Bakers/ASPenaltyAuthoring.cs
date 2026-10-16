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
    /// 地形惩罚区域的 Authoring 组件，挂载在需要增加寻路代价（但仍可通行）的 GameObject 上。
    ///
    /// 与 ASObstacleAuthoring 的区别：
    ///   - ASObstacle：节点完全不可通行（IsAccessible = false），代理绝对不会经过
    ///   - ASPenaltyAuthoring：节点仍然可通行，但移动代价增加，A* 倾向于选择代价更低的绕路
    ///
    /// 典型用途：
    ///   - 沼泽、草地等难以通行的地形（高惩罚） vs 道路（低惩罚），引导代理优先走道路
    ///   - 狭窄通道（高惩罚），让代理尽量走宽阔路径
    ///
    /// 使用前提：
    ///   该 GameObject 必须同时拥有碰撞体，且所在 Layer 与 GridAuthoring.PenaltyNodes 的 LayerMask 一致。
    ///
    /// 惩罚扩散：
    ///   CheckObstaclesSystem 会在完成节点扫描后对所有惩罚值进行高斯模糊，
    ///   使惩罚平滑扩散到周边节点，避免路径在惩罚区域边界产生"锯齿感"。
    /// </summary>
    public class ASPenaltyAuthoring : MonoBehaviour
    {
        /// <summary>
        /// 地形惩罚值（正整数，值越大代理越倾向于绕开该区域）。
        /// 建议范围：10~200，具体数值需结合场景中节点数量和距离进行调试。
        /// </summary>
        public int Penalty;

        /// <summary>
        /// Baker 内部类：将 ASPenaltyAuthoring 参数烘焙为 ECS 组件 ASPenalty。
        /// 在 SubScene 烘焙阶段自动执行，生成静态 ECS 数据。
        /// </summary>
        public class ASPenaltyBaker : Baker<ASPenaltyAuthoring>
        {
            public override void Bake(ASPenaltyAuthoring authoring)
            {
                // Dynamic：惩罚区域可能在运行时移动，需要动态 Transform
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                AddComponent(entity, new ASPenalty() { PenaltyValue = authoring.Penalty });
            }
        }
    }
}