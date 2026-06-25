package com.openvins.android.sdk

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.Looper
import com.openvins.android.Camera2ResView
import com.openvins.android.CameraFrameListener
import com.openvins.android.Trajectory3DView
import com.openvins.android.VioEngine
import com.openvins.android.component.TrajectoryRevisitor
import com.openvins.android.models.Result
import java.io.File

data class OpenVinsPose(
    val position: DoubleArray,
    val quaternion: DoubleArray,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as OpenVinsPose
        return position.contentEquals(other.position) && quaternion.contentEquals(other.quaternion)
    }

    override fun hashCode(): Int {
        var result = position.contentHashCode()
        result = 31 * result + quaternion.contentHashCode()
        return result
    }
}

data class OpenVinsTrajectory(
    val positions: FloatArray,
    val quaternions: FloatArray,
    val currentPose: OpenVinsPose,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as OpenVinsTrajectory
        return positions.contentEquals(other.positions) &&
            quaternions.contentEquals(other.quaternions) &&
            currentPose == other.currentPose
    }

    override fun hashCode(): Int {
        var result = positions.contentHashCode()
        result = 31 * result + quaternions.contentHashCode()
        result = 31 * result + currentPose.hashCode()
        return result
    }
}

data class BarnRevisitResult(
    val isRevisit: Boolean,
    val isRetrieve: Boolean,
    val isDuplicated: Boolean,
    val isMovingFast: Boolean,
    val impact: Double,
    val outerImpact: Double,
    val rotationError: Double,
    val pointDistance: Double,
) {
    companion object {
        fun from(result: Result, isMovingFast: Boolean): BarnRevisitResult {
            return BarnRevisitResult(
                isRevisit = result.isRevisit || result.isRetrieve || result.isDuplicated,
                isRetrieve = result.isRetrieve,
                isDuplicated = result.isDuplicated,
                isMovingFast = isMovingFast,
                impact = result.impact,
                outerImpact = result.outerImpact,
                rotationError = result.absPoseErrRotFro,
                pointDistance = result.pointDistance,
            )
        }
    }
}

interface OpenVinsSdkListener {
    fun onPoseChanged(pose: OpenVinsPose) {}
    fun onTrajectoryChanged(trajectory: OpenVinsTrajectory) {}
    fun onBarnRevisitChecked(result: BarnRevisitResult) {}
    fun onError(message: String, throwable: Throwable? = null) {}
}

class OpenVinsSdk(
    context: Context,
    private val listener: OpenVinsSdkListener? = null,
) : CameraFrameListener, SensorEventListener {

    private val appContext = context.applicationContext
    private val engine = VioEngine()
    private val sensorManager = appContext.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val trajectoryRevisitor = TrajectoryRevisitor()
    private val mainHandler = Handler(Looper.getMainLooper())

    private var cameraView: Camera2ResView? = null
    private var trajectoryView: Trajectory3DView? = null
    private var latestAccel: FloatArray? = null
    private var latestGyro: FloatArray? = null
    private var latestAccelTimestamp = 0L
    private var latestGyroTimestamp = 0L
    private var running = false
    private var poseRecording = false

    private val candidateTranslations = arrayListOf<DoubleArray>()
    private val candidateQuaternions = arrayListOf<DoubleArray>()

    private val trajectoryRunnable = object : Runnable {
        override fun run() {
            updateTrajectory()
            if (running) {
                mainHandler.postDelayed(this, TRAJECTORY_UPDATE_MS)
            }
        }
    }

    fun initialize(recordFolder: File? = null) {
        val privateRoot = appContext.filesDir.absolutePath
        val configRoot = engine.copyConfigIfNeeded(appContext)
        engine.setPrivateFolder(configRoot.ifBlank { privateRoot })
        recordFolder?.let {
            if (!it.exists()) it.mkdirs()
            engine.setRecordFolder(it.absolutePath)
        }
    }

    fun bindCameraView(view: Camera2ResView) {
        cameraView = view
        view.setCameraFrameListener(this)
    }

    fun bindTrajectoryView(view: Trajectory3DView?) {
        trajectoryView = view
        trajectoryView?.forceRender()
    }

    fun configureBarnRevisit(config: Map<String, Any>) {
        trajectoryRevisitor.setConfig(config)
    }

    fun resetBarnRevisitCandidates() {
        candidateTranslations.clear()
        candidateQuaternions.clear()
        trajectoryRevisitor.reset()
    }

    fun notifyCameraPermissionGranted() {
        cameraView?.setCameraPermissionGranted()
    }

    fun start() {
        if (running) return
        running = true
        resetBarnRevisitCandidates()
        trajectoryView?.clearTrajectory()
        cameraView?.enableView()
        registerSensors()
        engine.toggleSystem(true)
        mainHandler.post(trajectoryRunnable)
    }

    fun stop() {
        if (!running) return
        running = false
        mainHandler.removeCallbacks(trajectoryRunnable)
        unregisterSensors()
        engine.toggleSystem(false)
        cameraView?.disableView()
        if (poseRecording) {
            setPoseRecording(false)
        }
    }

    fun setPoseRecording(enabled: Boolean) {
        poseRecording = enabled
        engine.setRecording(enabled)
    }

    fun checkBarnRevisit(): BarnRevisitResult? {
        val snapshot = getTrajectorySnapshot() ?: return null
        val pose = snapshot.currentPose
        val result = trajectoryRevisitor.captureRevisited(
            snapshot.positions,
            snapshot.quaternions,
            pose.position,
            pose.quaternion,
            candidateTranslations,
            candidateQuaternions,
            pose.position,
            pose.quaternion,
        )
        val isMovingFast = trajectoryRevisitor.shiftingTrajectory(pose.position, pose.quaternion)
        val sdkResult = BarnRevisitResult.from(result, isMovingFast)
        candidateTranslations.add(pose.position.copyOf())
        candidateQuaternions.add(pose.quaternion.copyOf())
        listener?.onBarnRevisitChecked(sdkResult)
        return sdkResult
    }

    fun getCurrentPose(): OpenVinsPose? {
        val position = DoubleArray(3)
        val quaternion = DoubleArray(4)
        if (!engine.getCurrentPose(position, quaternion)) {
            return null
        }
        return OpenVinsPose(position, quaternion)
    }

    fun getTrajectorySnapshot(maxSize: Int = MAX_TRAJECTORY_POINTS): OpenVinsTrajectory? {
        val pose = getCurrentPose() ?: return null
        val positions = DoubleArray(maxSize * 3)
        val quaternions = DoubleArray(maxSize * 4)
        val size = engine.getTrajectoryData(positions, quaternions)
        if (size <= 0) {
            return OpenVinsTrajectory(floatArrayOf(), floatArrayOf(), pose)
        }

        val posFloats = FloatArray(size * 3)
        val quatFloats = FloatArray(size * 4)
        for (i in posFloats.indices) {
            posFloats[i] = positions[i].toFloat()
        }
        for (i in quatFloats.indices) {
            quatFloats[i] = quaternions[i].toFloat()
        }
        return OpenVinsTrajectory(posFloats, quatFloats, pose)
    }

    fun captureCameraImage(file: File, callback: (File?) -> Unit) {
        val view = cameraView
        if (view == null) {
            listener?.onError("Camera view is not bound.")
            callback(null)
            return
        }
        view.captureRawImage(file.absolutePath, callback = callback)
    }

    fun captureTrajectoryImage(file: File, callback: (File?) -> Unit) {
        val view = trajectoryView
        if (view == null) {
            listener?.onError("Trajectory view is not bound.")
            callback(null)
            return
        }
        view.captureImage(file.absolutePath, callback = callback)
    }

    override fun onFrame(matAddr: Long, timestampSec: Double) {
        engine.processImage(matAddr, timestampSec)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        when (event?.sensor?.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                latestAccel = event.values.copyOf()
                latestAccelTimestamp = event.timestamp
            }
            Sensor.TYPE_GYROSCOPE -> {
                latestGyro = event.values.copyOf()
                latestGyroTimestamp = event.timestamp
            }
        }

        val accel = latestAccel ?: return
        val gyro = latestGyro ?: return
        val timestampSec = maxOf(latestAccelTimestamp, latestGyroTimestamp) * 1e-9
        engine.processImu(
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            timestampSec,
        )
        latestAccel = null
        latestGyro = null
        latestAccelTimestamp = 0L
        latestGyroTimestamp = 0L
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun registerSensors() {
        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (accel == null || gyro == null) {
            listener?.onError("Accelerometer or gyroscope is not available on this device.")
            return
        }
        sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST)
    }

    private fun unregisterSensors() {
        sensorManager.unregisterListener(this)
        latestAccel = null
        latestGyro = null
        latestAccelTimestamp = 0L
        latestGyroTimestamp = 0L
    }

    private fun updateTrajectory() {
        val snapshot = getTrajectorySnapshot() ?: return
        val currentPosition = FloatArray(3) { snapshot.currentPose.position[it].toFloat() }
        val currentQuaternion = FloatArray(4) { snapshot.currentPose.quaternion[it].toFloat() }
        trajectoryView?.updateTrajectory(
            snapshot.positions,
            snapshot.quaternions,
            currentPosition,
            currentQuaternion,
        )
        listener?.onPoseChanged(snapshot.currentPose)
        listener?.onTrajectoryChanged(snapshot)
    }

    companion object {
        private const val TRAJECTORY_UPDATE_MS = 50L
        private const val MAX_TRAJECTORY_POINTS = 10000
    }
}
