package com.openvins.android.models

import com.openvins.android.engine.Utils
import glm_.vec3.Vec3d
import kotlin.math.sqrt


class RadiationField(
    val path: Trajectory,
    val maxDistance: Double,
    val initialIntensity: Double = 1.0
) {
    // 计算某点的辐射强度
    fun getIntensityAt(point: Vec3d): Double {
        val distanceToPath = sqrt(Utils.distance2PointToTrajectory(point, path))
        if (distanceToPath > maxDistance) return 0.0
        return initialIntensity * (1 - distanceToPath / maxDistance)
    }

    fun calculateRadiationImpact(secondPath: Trajectory): Double {
        var totalImpact = 0.0
        // 遍历第二条路径的每一段
        for (i in 0 until secondPath.points.size) {
            val p = secondPath.points[i]
            val intensity = getIntensityAt(p)
            totalImpact += intensity
        }
        totalImpact /= secondPath.points.size.toDouble()
        return totalImpact
    }
}