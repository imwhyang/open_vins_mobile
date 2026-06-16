package com.openvins.android.engine

import com.openvins.android.models.Trajectory
import glm_.vec3.Vec3d
import kotlin.math.pow
import kotlin.math.sqrt

object Utils {
    fun distance2PointToTrajectory(p: Vec3d, path: Trajectory): Double {
        var minDist2 = Double.POSITIVE_INFINITY
        for (q in path.points) {
            val d2 = distance2PointToPoint(p, q)
            if (d2 < minDist2) minDist2 = d2
        }
        return minDist2
    }

    fun distance2PointToPoint(p1: Vec3d, p2: Vec3d): Double {
        val dx = p1.x - p2.x
        val dy = p1.y - p2.y
        return dx * dx + dy * dy
    }

    fun distancePointToPoint(a: Vec3d, b: Vec3d): Double {
        return sqrt(distance2PointToPoint(a, b))
    }

    fun distancePointToPoint(a: DoubleArray, b: DoubleArray): Double {
        var summary = 0.0
        for (i in 0 until 2) {
            summary += (a[i] - b[i]).pow(2.0)
        }
        return sqrt(summary)
    }

    fun distancePointToPoint3d(a: DoubleArray, b: DoubleArray): Double {
        var summary = 0.0
        for (i in 0 until 3) {
            summary += (a[i] - b[i]).pow(2.0)
        }
        return sqrt(summary)
    }
}