package com.openvins.android.engine

import glm_.vec3.Vec3d
import kotlin.math.ceil
import kotlin.math.exp


/*
    这个工具用于平滑3d路径
    输入为有序的3d点集，代表一个空间中的路径，要求对这个路径滤波，使路径平滑

    1. 实现使用 gaussian 平滑算法
 */
object Smoother {

    /**
     * Apply Gaussian smoothing to a list of 3D points.
     * Each point is a FloatArray of size 3: [x, y, z].
     * Returns a new list with the smoothed points (same size).
     */
    fun gaussianSmooth(points: List<Vec3d>, sigma: Double = 1.0): List<Vec3d> {
        if (points.isEmpty()) return listOf()
        val n = points.size
        // separate channels
        val xs = DoubleArray(n)
        val ys = DoubleArray(n)
        val zs = DoubleArray(n)
        for (i in 0 until n) {
            val p = points[i]
            xs[i] = p.x
            ys[i] = p.y
            zs[i] = p.z
        }
        val sx = convolveGaussian(xs, sigma)
        val sy = convolveGaussian(ys, sigma)
        val sz = convolveGaussian(zs, sigma)
        val out = mutableListOf<Vec3d>()
        for (i in 0 until n) {
            out.add(Vec3d(sx[i], sy[i], sz[i]))
        }
        return out
    }

    /**
     * Apply Gaussian smoothing to an interleaved float array (e.g. [x0,y0,z0,x1,y1,z1,...]).
     * stride defines number of components per point (default 3).
     * Returns a new FloatArray with the same size containing smoothed values.
     */
    fun gaussianSmoothInterleaved(
        data: DoubleArray,
        stride: Int = 3,
        sigma: Double = 1.0
    ): DoubleArray {
        if (data.isEmpty()) return DoubleArray(0)
        if (stride <= 0) throw IllegalArgumentException("stride must be positive")
        val nPoints = data.size / stride
        if (nPoints == 0) return DoubleArray(0)
        val out = DoubleArray(nPoints * stride)
        // process each channel separately
        for (c in 0 until stride) {
            val channel = DoubleArray(nPoints)
            for (i in 0 until nPoints) channel[i] = data[i * stride + c]
            val sch = convolveGaussian(channel, sigma)
            for (i in 0 until nPoints) out[i * stride + c] = sch[i]
        }
        return out
    }

    // Helper: create Gaussian kernel and convolve with reflect border handling
    private fun convolveGaussian(data: DoubleArray, sigma: Double): DoubleArray {
        val n = data.size
        if (n == 0) return DoubleArray(0)
        // when sigma very small, return copy
        if (sigma <= 0.000001) return data.copyOf()
        val radius = ceil(3.0 * sigma).toInt()
        val size = radius * 2 + 1
        val kernel = DoubleArray(size)
        val sigma2 = sigma * sigma
        var sum = 0.0
        for (i in -radius..radius) {
            val v = exp(-(i * i) / (2.0 * sigma2))
            kernel[i + radius] = v
            sum += v
        }
        // normalize
        for (i in kernel.indices) kernel[i] /= sum
        val out = DoubleArray(n)
        for (i in 0 until n) {
            var acc = 0.0
            for (k in -radius..radius) {
                var idx = i + k
                // reflect border
                if (idx < 0) idx = -idx - 1
                if (idx >= n) idx = 2 * n - idx - 1
                // clamp border
                if (idx < 0) idx = 0
                if (idx >= n) idx = n - 1
                acc += data[idx] * kernel[k + radius]
            }
            out[i] = acc
        }
        return out
    }
}