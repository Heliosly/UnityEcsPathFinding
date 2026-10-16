using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Entities;
using Unity.Collections;

namespace Athomield.AStar
{
    /// <summary>
    /// 寻路辅助工具类，提供坐标系转换等通用方法
    /// </summary>
    public static class PathfindingUtility
    {
        /// <summary>
        /// 将 3D 世界坐标（float3）转换为 2D 平面坐标（float2），
        /// 取 X 和 Z 分量，忽略 Y 轴高度。
        /// RVO/ORCA 避障计算在水平平面（XZ 平面）进行，因此需要此转换
        /// </summary>
        /// <param name="_vector3">3D 世界坐标（x, y, z）</param>
        /// <returns>2D 平面坐标（x, z），对应世界空间的水平方向</returns>
        public static float2 V2FromV3(float3 _vector3)
        {
            return new float2(_vector3.x, _vector3.z);
        }
    }
}
