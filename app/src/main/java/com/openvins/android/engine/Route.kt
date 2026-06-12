package com.openvins.android.engine

import com.openvins.android.models.Trajectory

object Route {
    /*
    这个工具用于路径截取
    输入
       一个查询路径 queryPath，
       一个原始路径 originPath，
       一个最大距离 maxDistance，
    输出
       一个新的路径 newPath，
    其中 newPath 是 originPath 的最大连续子路径，
    newPath 的起点和终点在 queryPath 以 maxDistance 为界的邻域内。
     */
    fun getMaximumSubPath(queryPath: Trajectory, originPath: Trajectory, maxDistance: Double): Trajectory {
        val origin = originPath.points
        if (origin.isEmpty()) return Trajectory.empty()
        if (queryPath.points.isEmpty()) return Trajectory.empty()
        val n = origin.size
        // Precompute boolean array indicating which origin points are within maxDistance to queryPath
        val within = BooleanArray(n)
        val distanceSquare = maxDistance * maxDistance
        for (i in 0 until n) {
            within[i] = Utils.distance2PointToTrajectory(origin[i], queryPath) <= distanceSquare
        }
        // Collect indices that are within neighborhood
        var start = Int.MAX_VALUE
        var end = Int.MIN_VALUE
        for (i in 0 until n) {
            if (within[i]) {
                if (i < start) start = i
                if (i > end) end = i
            }
        }
        if (end <= start) return Trajectory.empty()
        // The longest contiguous subpath whose endpoints are within the neighborhood
        // will have start = first within index and end = last within index
        return originPath.slice(start, end + 1)
    }

    /*
    路径里程定义为路径上点的距离的累加，本函数的目标是从路径中截取满足最小里程的最小子路径，要求子路径起点是 startIndex 点。
     输入
         一个路径 path，
         一个最小里程 minMileage，
         一个下标 startIndex，默认为 0
     输出
         一个新的路径 subPath， 长度不小于 minMileage，且包含 startIndex 点的最小子路径
    * 从路径中截取满足最小里程的最小子路径，要求子路径起点是 startIndex 点，并向前（索引增大方向）延伸。
    * 若 minMileage <= 0，则返回只包含 startIndex 的单点路径。
    * 若剩余路径累计里程不能达到 minMileage，则返回空路径。
    */
    fun getMinimumSubPath(path: Trajectory, minMileage: Double = 1.5, startIndex: Int = 0, reversed: Boolean = false): Trajectory {
        val pts = if (reversed) path.points.reversed() else path.points
        val n = pts.size
        if (n == 0) return Trajectory.empty()
        val s = startIndex.coerceIn(0, n - 1)
        var acc = 0.0

        var e = s
        while (e + 1 < n && acc < minMileage) {
            acc += Utils.distancePointToPoint(pts[e], pts[e + 1])
            e++
        }
        val subPath = if (reversed) path.slice(n - 1 - e, n - s) else path.slice(s, e + 1)
        return if (acc >= minMileage) subPath else path.copy()
    }
}