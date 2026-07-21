package com.openvins.android

import kotlin.math.abs

/**
 * 将 Android 分别回调的加速度计和陀螺仪数据配成同一条 IMU 测量。
 *
 * 不同手机的两类传感器频率和回调顺序可能不同。若直接拼接最近值，会把两个
 * 不同时刻的读数交给 VIO，快速转身时尤其容易造成错误的速度和位置积分。
 */
class ImuSampleSynchronizer(
    private val maxPairDeltaNs: Long = DEFAULT_MAX_PAIR_DELTA_NS,
) {
    data class Sample(
        val accel: FloatArray,
        val gyro: FloatArray,
        val timestampNs: Long,
        val pairDeltaNs: Long,
    )

    private data class SensorSample(
        val values: FloatArray,
        val timestampNs: Long,
    )

    private var pendingAccel: SensorSample? = null
    private var pendingGyro: SensorSample? = null
    private var lastAccelTimestampNs = Long.MIN_VALUE
    private var lastGyroTimestampNs = Long.MIN_VALUE
    private var lastOutputTimestampNs = Long.MIN_VALUE
    private var estimatedAccelPeriodNs = 0.0
    private var estimatedGyroPeriodNs = 0.0

    @Synchronized
    fun addAccelerometer(values: FloatArray, timestampNs: Long): Sample? {
        if (timestampNs <= lastAccelTimestampNs || values.size < 3) return null
        estimatedAccelPeriodNs = updateEstimatedPeriod(estimatedAccelPeriodNs, lastAccelTimestampNs, timestampNs)
        lastAccelTimestampNs = timestampNs
        pendingAccel = SensorSample(values.copyOf(3), timestampNs)
        return tryCreateSample()
    }

    @Synchronized
    fun addGyroscope(values: FloatArray, timestampNs: Long): Sample? {
        if (timestampNs <= lastGyroTimestampNs || values.size < 3) return null
        estimatedGyroPeriodNs = updateEstimatedPeriod(estimatedGyroPeriodNs, lastGyroTimestampNs, timestampNs)
        lastGyroTimestampNs = timestampNs
        pendingGyro = SensorSample(values.copyOf(3), timestampNs)
        return tryCreateSample()
    }

    @Synchronized
    fun reset() {
        pendingAccel = null
        pendingGyro = null
        lastAccelTimestampNs = Long.MIN_VALUE
        lastGyroTimestampNs = Long.MIN_VALUE
        lastOutputTimestampNs = Long.MIN_VALUE
        estimatedAccelPeriodNs = 0.0
        estimatedGyroPeriodNs = 0.0
    }

    private fun tryCreateSample(): Sample? {
        val accel = pendingAccel ?: return null
        val gyro = pendingGyro ?: return null
        val pairDeltaNs = abs(accel.timestampNs - gyro.timestampNs)

        if (pairDeltaNs > currentPairDeltaLimitNs()) {
            // 时间差过大时保留较新的样本，等待另一类传感器追上，避免使用陈旧读数。
            if (accel.timestampNs < gyro.timestampNs) {
                pendingAccel = null
            } else {
                pendingGyro = null
            }
            return null
        }

        val outputTimestampNs = maxOf(accel.timestampNs, gyro.timestampNs)
        pendingAccel = null
        pendingGyro = null
        if (outputTimestampNs <= lastOutputTimestampNs) return null

        lastOutputTimestampNs = outputTimestampNs
        return Sample(
            accel = accel.values,
            gyro = gyro.values,
            timestampNs = outputTimestampNs,
            pairDeltaNs = pairDeltaNs,
        )
    }

    private fun updateEstimatedPeriod(currentEstimateNs: Double, previousTimestampNs: Long, timestampNs: Long): Double {
        if (previousTimestampNs == Long.MIN_VALUE) return currentEstimateNs
        val periodNs = timestampNs - previousTimestampNs
        if (periodNs <= 0L || periodNs > MAX_VALID_SENSOR_PERIOD_NS) return currentEstimateNs
        if (currentEstimateNs <= 0.0) return periodNs.toDouble()
        return currentEstimateNs * (1.0 - PERIOD_EWMA_ALPHA) + periodNs * PERIOD_EWMA_ALPHA
    }

    private fun currentPairDeltaLimitNs(): Long {
        // 两类传感器都获得稳定周期前沿用绝对上限，避免启动阶段误丢首批数据。
        if (estimatedAccelPeriodNs <= 0.0 || estimatedGyroPeriodNs <= 0.0) return maxPairDeltaNs
        val slowerPeriodNs = maxOf(estimatedAccelPeriodNs, estimatedGyroPeriodNs)
        val minimumLimitNs = minOf(MIN_ADAPTIVE_PAIR_DELTA_NS, maxPairDeltaNs)
        return (slowerPeriodNs * PAIR_DELTA_PERIOD_RATIO)
            .toLong()
            .coerceIn(minimumLimitNs, maxPairDeltaNs)
    }

    companion object {
        // 兼容约 50 Hz 的传感器，同时拒绝明显跨周期的错误配对。
        const val DEFAULT_MAX_PAIR_DELTA_NS = 20_000_000L
        const val MIN_ADAPTIVE_PAIR_DELTA_NS = 5_000_000L
        const val MAX_VALID_SENSOR_PERIOD_NS = 100_000_000L
        const val PAIR_DELTA_PERIOD_RATIO = 0.6
        const val PERIOD_EWMA_ALPHA = 0.1
    }
}
