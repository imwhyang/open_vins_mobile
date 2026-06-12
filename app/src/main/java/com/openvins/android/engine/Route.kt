package com.openvins.android.engine

import glm_.vec3.Vec3d

object Route {
    /*
    计算路径里程，里程定义为路径上点的距离的累加
     输入
        一个路径 path，
     输出
        路径的里程 mileage
     */
    fun calculateMileage(path: List<Vec3d>): Double {
        if (path.size < 2) return 0.0
        var sum = 0.0
        for (i in 0 until path.size - 1) {
            sum += Utils.distancePointToPoint(path[i], path[i + 1])
        }
        return sum
    }

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
    fun getMaximumSubPath(
        queryPath: List<Vec3d>,
        originPath: List<Vec3d>,
        maxDistance: Double = 0.5
    ): Pair<Int, Int> {
        // Ensure the input trajectories are not empty
        if (queryPath.isEmpty() || originPath.isEmpty()) return Pair(-1, -1)
        val multiplier = 4.0
        val threshold = maxDistance * maxDistance * multiplier * multiplier
        val indices = mutableListOf<Int>()
        for (j in originPath.indices) {
            for (i in queryPath.indices) {
                if (Utils.distance2PointToPoint(queryPath[i], originPath[j]) < threshold) {
                    indices.add(j)
                    break
                }
            }
        }
        // If no valid indices are found, return an empty array
        if (indices.isEmpty()) return Pair(-1, -1)
        // Get the minimum and maximum indices
        val minIndex = indices.minOrNull() ?: 0
        val maxIndex = indices.maxOrNull() ?: (originPath.size - 1)
        return Pair(minIndex, maxIndex + 1)
    }

    /*
    路径里程定义为路径上点的距离的累加，本函数的目标是从路径尾部向头部截取满足最小里程的最小子路径，要求子路径起点是 start 点。
     输入
         一个路径 path，
         一个最小里程 minMileage，
         一个下标 startIndex，默认为 0
     输出
         一个新的路径 subPath，里程不小于 minMileage，且包含 start 点的最小子路径
         一个索引点 endDistanceIndex，表示 path 路径从该点到的尾部点在的里程至少为 minMileage + maxDistance
    * 从路径中截取满足最小里程的最小子路径，要求子路径起点是 start 点，并尾部向头部延伸。
    * 若 minMileage <= 0，则返回只包含 start 的单点路径。
    * 若剩余路径累计里程不能达到 minMileage，则返回空路径。
    */
    fun getMinimumSubPathReversed(
        path: List<Vec3d>,
        mileage: Double = 1.5,
        maxDistance: Double = 0.5,
        start: Int = 0
    ): Triple<Int, Int, Int> {
        // Ensure the trajectory is not empty
        if (path.isEmpty()) return Triple(-1, -1, -1)

        val n = path.size
        if (n < 2) return Triple(-1, -1, -1)

        // Calculate distances between consecutive points in the trajectory (only x, y dimensions)
        val distances = DoubleArray(n) { i ->
            if (i == 0) 0.0 else Utils.distance2PointToPoint(path[i - 1], path[i])
        }

        // Compute cumulative sum of distances
        val cumsum = DoubleArray(n)
        cumsum[0] = distances[0]
        for (i in 1 until n) {
            cumsum[i] = cumsum[i - 1] + distances[i]
        }

        // Reverse the cumulative sum and adjust it relative to the start index
        val reversedCumsum = DoubleArray(n) { i ->
            cumsum.last() - cumsum[n - i - 1]
        }
        for (i in reversedCumsum.indices) {
            reversedCumsum[i] -= reversedCumsum[start]
        }

        // Find the end index where the cumulative mileage is less than or equal to the given mileage
        val end = reversedCumsum.indexOfFirst { it > mileage }.takeIf { it != -1 } ?: (n - 1)

        // Find the end distance index where the cumulative mileage is less than or equal to mileage + maxDistance
        val endDistance =
            reversedCumsum.indexOfFirst { it > mileage + maxDistance }.takeIf { it != -1 } ?: n

        // Return the sub-path and the adjusted end distance index
        return Triple(n - end - 1, n - start, n - endDistance - 1)
    }
}