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
import com.openvins.android.ImuSampleSynchronizer
import com.openvins.android.Trajectory3DView
import com.openvins.android.VioEngine
import com.openvins.android.component.TrajectoryRevisitor
import com.openvins.android.models.Result
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.UUID
import kotlin.math.cos

/** OpenVINS 当前的定位工作状态。 */
enum class OpenVinsTrackingState {
    /** 定位系统尚未启动。 */
    STOPPED,
    /** 正在收集图像和 IMU 数据完成首次初始化。 */
    INITIALIZING,
    /** 定位正常，设备正在运动。 */
    TRACKING,
    /** 定位正常，零速更新判断设备当前静止。 */
    STATIONARY,
    /** 摄像头遮挡、模糊或纹理不足，当前画面未参与定位。 */
    CAMERA_UNAVAILABLE,
    /** 画面恢复后正在将新坐标系接续到原轨迹。 */
    ALIGNING,
    /** 检测到持续异常位移，轨迹已暂停等待用户处理。 */
    PAUSED;

    internal companion object {
        fun fromNative(value: Int): OpenVinsTrackingState {
            return values().getOrElse(value) { STOPPED }
        }
    }
}

/** SDK 状态快照。 */
data class OpenVinsStatus(
    /** 当前定位状态。 */
    val state: OpenVinsTrackingState,
    /** 轨迹暂停原因码，仅 [OpenVinsTrackingState.PAUSED] 状态下有效。 */
    val pauseReason: Int? = null,
) {
    /** 当前状态是否允许拍照并记录可靠 Pose。 */
    val canUsePose: Boolean
        get() = state == OpenVinsTrackingState.TRACKING ||
            state == OpenVinsTrackingState.STATIONARY
}

/** 单个 VIO 位姿，位置单位为米，四元数顺序为 `[w, x, y, z]`。 */
data class OpenVinsPose(
    /** 三维位置 `[x, y, z]`，单位为米。 */
    val position: DoubleArray,
    /** 姿态四元数 `[w, x, y, z]`。 */
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

/** 当前完整轨迹及最新位姿快照。 */
data class OpenVinsTrajectory(
    /** 连续位置数组，按 `[x0, y0, z0, x1, y1, z1...]` 排列。 */
    val positions: FloatArray,
    /** 连续四元数数组，按 `[w0, x0, y0, z0...]` 排列。 */
    val quaternions: FloatArray,
    /** 采集快照时的最新位姿。 */
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

/** 猪圈重复拍摄判断结果。 */
data class BarnRevisitResult(
    /** 是否需要提示重复；点位重复或同侧路径重复任一成立时为 true。 */
    val isRevisit: Boolean,
    /** 是否命中同侧历史拍摄点。 */
    val isRetrieve: Boolean,
    /** 最近局部路径是否与历史路径重复。 */
    val isDuplicated: Boolean,
    /** 当前短时间移动是否异常过快。 */
    val isMovingFast: Boolean,
    /** 当前轨迹与历史轨迹的重合度。 */
    val impact: Double,
    /** 考虑拍摄方向后的外侧轨迹重合度。 */
    val outerImpact: Double,
    /** 与最佳同侧候选的姿态误差，仅用于诊断。 */
    val rotationError: Double,
    /** 与最佳同侧候选的水平距离，单位为米。 */
    val pointDistance: Double,
    /** 历史记录中是否存在与当前镜头朝向同侧的候选点。 */
    val isSameSide: Boolean,
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
                isSameSide = result.isSameSide,
            )
        }
    }
}

/** 一次养殖场拍摄任务的业务会话。 */
data class BarnInsuranceSession(
    /** 会话唯一标识。 */
    val sessionId: String = UUID.randomUUID().toString(),
    /** 业务保单或任务标识。 */
    val policyId: String,
    /** 养殖场或猪舍标识。 */
    val barnId: String,
    /** 会话开始时间，Unix 毫秒。 */
    val startedAtMs: Long = System.currentTimeMillis(),
)

/** 一次猪圈拍摄请求。 */
data class PenCheckRequest(
    /** 当前拍摄点的业务标识。 */
    val penId: String,
    /** 可选的预计猪只数量。 */
    val expectedPigCount: Int? = null,
    /** 可选的操作员备注。 */
    val operatorRemark: String? = null,
)

/** 一次成功拍摄后形成的业务记录。 */
data class PenCaptureRecord(
    /** 所属拍摄会话标识。 */
    val sessionId: String,
    /** 所属业务保单或任务标识。 */
    val policyId: String,
    /** 所属养殖场或猪舍标识。 */
    val barnId: String,
    /** 本次拍摄点业务标识。 */
    val penId: String,
    /** 发起拍摄的时间，Unix 毫秒。 */
    val capturedAtMs: Long,
    /** 发起拍摄时冻结的 Pose。 */
    val pose: OpenVinsPose,
    /** 发起拍摄时冻结的完整轨迹。 */
    val trajectory: OpenVinsTrajectory,
    /** 本次拍摄的查重结果。 */
    val revisitResult: BarnRevisitResult,
    /** 可选的预计猪只数量。 */
    val expectedPigCount: Int? = null,
    /** 可选的操作员备注。 */
    val operatorRemark: String? = null,
    /** 成功保存的照片绝对路径；旧兼容接口可能为 null。 */
    val photoPath: String? = null,
)

/**
 * 文件存储配置。
 *
 * SDK 会在 [rootDirectory] 下自动创建照片、视频和 Pose 子目录。
 */
data class OpenVinsStorageConfig(
    /** 本次业务采集文件的根目录。 */
    val rootDirectory: File,
    /** 照片子目录名称。 */
    val photoDirectoryName: String = "photos",
    /** 视频子目录名称。 */
    val videoDirectoryName: String = "videos",
    /** Pose 文件子目录名称。 */
    val poseDirectoryName: String = "poses",
    /** 是否生成轨迹和相机质量诊断日志；正式环境建议关闭。 */
    val debugLoggingEnabled: Boolean = false,
) {
    /** 实际照片目录。 */
    val photoDirectory: File
        get() = File(rootDirectory, photoDirectoryName)
    /** 实际视频目录。 */
    val videoDirectory: File
        get() = File(rootDirectory, videoDirectoryName)
    /** 实际 Pose 目录。 */
    val poseDirectory: File
        get() = File(rootDirectory, poseDirectoryName)
}

/** 一次原子拍摄的返回结果。 */
data class OpenVinsCaptureResult(
    /** 包含 Pose、轨迹及查重结论的业务记录。 */
    val record: PenCaptureRecord,
    /** 本次保存的原始相机画面文件。 */
    val photoFile: File,
)

/** 正在进行的视频与 Pose 同步录制会话。 */
data class OpenVinsRecordingSession(
    /** 录制唯一标识。 */
    val recordingId: String,
    /** 录制开始时间，Unix 毫秒。 */
    val startedAtMs: Long,
    /** 预期生成的视频文件。 */
    val videoFile: File,
    /** 本次 Pose 文件所在目录。 */
    val poseDirectory: File,
)

/** 视频与 Pose 同步录制停止后的文件结果。 */
data class OpenVinsRecordingResult(
    /** 对应的录制会话。 */
    val session: OpenVinsRecordingSession,
    /** 录制结束时间，Unix 毫秒。 */
    val endedAtMs: Long,
    /** 成功生成的视频文件，失败时为 null。 */
    val videoFile: File?,
    /** 成功生成的 `pose0.csv` 文件，失败时为 null。 */
    val poseFile: File?,
)

/**
 * 猪圈查重配置。
 *
 * 点位查重首先通过 [sameSideMaxAngleDegrees] 判断是否拍摄过道同一侧，
 * 只有同侧且水平距离小于 [pointDistanceMeters] 才直接判为重复。
 */
data class BarnRevisitConfig(
    /** 用于局部路径查重的最近轨迹长度，单位为米。 */
    val revisitMileageMeters: Double = 1.5,
    /** 路径点允许偏离历史路径的最大距离，单位为米。 */
    val routeMaxDistanceMeters: Double = 0.5,
    /** 局部路径重合度阈值，取值范围 0 至 1。 */
    val routeImpactThreshold: Double = 0.35,
    /** 同侧拍摄点判重的最大水平距离，单位为米。 */
    val pointDistanceMeters: Double = 0.5,
    /** 两次镜头水平朝向仍视为同侧的最大夹角，单位为度。 */
    val sameSideMaxAngleDegrees: Double = 90.0,
    /** 0.5 秒窗口内用于标记移动过快的累计里程，单位为米。 */
    val shiftingMileageMeters: Double = 1.0,
)

/** SDK 事件监听器；所有界面交互应由宿主应用在这些回调中完成。 */
interface OpenVinsSdkListener {
    /** 定位状态发生变化时回调。 */
    fun onTrackingStateChanged(status: OpenVinsStatus) {}
    /** 最新 Pose 更新时回调，频率约为 20Hz。 */
    fun onPoseChanged(pose: OpenVinsPose) {}
    /** 轨迹更新时回调，频率约为 20Hz。 */
    fun onTrajectoryChanged(trajectory: OpenVinsTrajectory) {}
    /** 执行一次查重后回调。 */
    fun onBarnRevisitChecked(result: BarnRevisitResult) {}
    /** 一次照片、Pose 和查重记录全部保存成功后回调。 */
    fun onPenCaptured(record: PenCaptureRecord) {}
    /** SDK 操作失败或设备能力不足时回调。 */
    fun onError(message: String, throwable: Throwable? = null) {}
}

/**
 * OpenVINS Android 高层入口。
 *
 * 负责相机帧、IMU、轨迹绘制、业务拍摄、同侧查重及视频/Pose 录制。
 */
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
    private val imuSampleSynchronizer = ImuSampleSynchronizer()
    private var running = false
    private var poseRecording = false
    private var lastStatus: OpenVinsStatus? = null
    private var activeSession: BarnInsuranceSession? = null
    private var storageConfig: OpenVinsStorageConfig? = null
    private var activeRecording: OpenVinsRecordingSession? = null
    private var captureInProgress = false
    private var lastTrajectorySnapshot: OpenVinsTrajectory? = null

    private val candidateTranslations = arrayListOf<DoubleArray>()
    private val candidateQuaternions = arrayListOf<DoubleArray>()
    private val penCaptureRecords = arrayListOf<PenCaptureRecord>()

    private val trajectoryRunnable = object : Runnable {
        override fun run() {
            dispatchTrackingStatus()
            updateTrajectory()
            if (running) {
                mainHandler.postDelayed(this, TRAJECTORY_UPDATE_MS)
            }
        }
    }

    /**
     * 使用兼容方式初始化 SDK。
     *
     * 新接入项目建议使用 [initialize] 的 [OpenVinsStorageConfig] 重载。
     */
    fun initialize(
        recordFolder: File? = null,
        debugLoggingEnabled: Boolean = false,
    ) {
        val defaultRoot = recordFolder ?: File(appContext.filesDir, "openvins")
        initialize(
            OpenVinsStorageConfig(
                rootDirectory = defaultRoot,
                poseDirectoryName = if (recordFolder != null) "." else "poses",
                debugLoggingEnabled = debugLoggingEnabled,
            )
        )
    }

    /**
     * 初始化配置文件和存储目录。创建 SDK 后、调用 [start] 前执行一次。
     *
     * @throws IllegalArgumentException 任一存储目录无法创建时抛出。
     */
    fun initialize(config: OpenVinsStorageConfig) {
        val privateRoot = appContext.filesDir.absolutePath
        val configRoot = engine.copyConfigIfNeeded(appContext)
        engine.setPrivateFolder(configRoot.ifBlank { privateRoot })
        configureStorage(config)
    }

    /**
     * 配置所有业务文件的根目录。目录会统一创建，调用方不再需要分别拼接照片、视频和 Pose 路径。
     */
    fun configureStorage(config: OpenVinsStorageConfig) {
        require(config.rootDirectory.ensureDirectory()) {
            "无法创建 OpenVINS 存储目录：${config.rootDirectory.absolutePath}"
        }
        require(config.photoDirectory.ensureDirectory()) {
            "无法创建照片目录：${config.photoDirectory.absolutePath}"
        }
        require(config.videoDirectory.ensureDirectory()) {
            "无法创建视频目录：${config.videoDirectory.absolutePath}"
        }
        require(config.poseDirectory.ensureDirectory()) {
            "无法创建 Pose 目录：${config.poseDirectory.absolutePath}"
        }
        storageConfig = config
        engine.setRecordFolder(config.poseDirectory.absolutePath)
        engine.setDebugLoggingEnabled(config.debugLoggingEnabled)
    }

    /** 绑定必需的相机预览控件，SDK 将自动接收并处理相机帧。 */
    fun bindCameraView(view: Camera2ResView) {
        cameraView = view
        view.setCameraFrameListener(this)
    }

    /** 绑定可选的轨迹视图；传入 null 时仍可正常定位和记录 Pose。 */
    fun bindTrajectoryView(view: Trajectory3DView?) {
        trajectoryView = view
        trajectoryView?.forceRender()
    }

    /**
     * 控制相机画面上的原生 init/zvupt 调试文字。
     * 正式业务建议关闭，并通过 onTrackingStateChanged 自定义交互。
     */
    fun setStatusOverlayEnabled(enabled: Boolean) {
        engine.setStatusOverlayEnabled(enabled)
    }

    /** 配置下次录制是否额外生成诊断日志；正式环境默认只生成 pose0.csv。 */
    fun setDebugLoggingEnabled(enabled: Boolean) {
        engine.setDebugLoggingEnabled(enabled)
    }

    /** 主动获取当前定位状态；通常优先使用监听器的状态变化回调。 */
    fun getStatus(): OpenVinsStatus {
        return buildTrackingStatus()
    }

    /**
     * 使用类型安全的参数配置猪圈查重。
     */
    fun configureBarnRevisit(config: BarnRevisitConfig) {
        require(config.sameSideMaxAngleDegrees in 0.0..180.0) {
            "同侧最大夹角必须在 0 至 180 度之间。"
        }
        trajectoryRevisitor.setConfig(
            mapOf(
                "revisitMileage" to config.revisitMileageMeters,
                "maxDistance" to config.routeMaxDistanceMeters,
                "impactThreshold" to config.routeImpactThreshold,
                "pointDistance" to config.pointDistanceMeters,
                "sameSideCosThreshold" to cos(Math.toRadians(config.sameSideMaxAngleDegrees)),
                "shiftingMileage" to config.shiftingMileageMeters,
            )
        )
    }

    /**
     * 使用键值参数配置猪圈查重。保留用于兼容旧项目，新项目建议使用 [BarnRevisitConfig]。
     */
    fun configureBarnRevisit(config: Map<String, Any>) {
        trajectoryRevisitor.setConfig(config)
    }

    /**
     * 开始一次业务拍摄会话，并清空上一次会话的拍摄记录与查重候选。
     */
    fun beginInsuranceSession(
        policyId: String,
        barnId: String,
        sessionId: String = UUID.randomUUID().toString(),
    ): BarnInsuranceSession {
        val session = BarnInsuranceSession(
            sessionId = sessionId,
            policyId = policyId,
            barnId = barnId,
        )
        activeSession = session
        resetBarnRevisitCandidates()
        penCaptureRecords.clear()
        return session
    }

    /** 结束当前业务会话，并返回该会话内全部成功拍摄记录的只读副本。 */
    fun finishInsuranceSession(): List<PenCaptureRecord> {
        val records = penCaptureRecords.toList()
        activeSession = null
        return records
    }

    /** 获取当前业务会话内全部成功拍摄记录的只读副本。 */
    fun getPenCaptureRecords(): List<PenCaptureRecord> {
        return penCaptureRecords.toList()
    }

    /** 清空查重候选和短时移动检测状态，不会删除已经生成的文件。 */
    fun resetBarnRevisitCandidates() {
        candidateTranslations.clear()
        candidateQuaternions.clear()
        trajectoryRevisitor.reset()
    }

    /** 宿主应用取得 CAMERA 权限后调用，使已绑定的相机控件可以打开摄像头。 */
    fun notifyCameraPermissionGranted() {
        cameraView?.setCameraPermissionGranted()
    }

    /** 启动相机、IMU、VIO 与轨迹刷新；重复调用不会重复启动。 */
    fun start() {
        if (running) return
        running = true
        lastTrajectorySnapshot = null
        resetBarnRevisitCandidates()
        trajectoryView?.clearTrajectory()
        cameraView?.enableView()
        registerSensors()
        engine.toggleSystem(true)
        dispatchTrackingStatus(force = true)
        mainHandler.post(trajectoryRunnable)
    }

    /** 停止定位并释放传感器和相机资源；正在进行的证据录制也会自动停止。 */
    fun stop() {
        if (!running) return
        stopEvidenceRecording()
        running = false
        mainHandler.removeCallbacks(trajectoryRunnable)
        unregisterSensors()
        engine.toggleSystem(false)
        dispatchTrackingStatus(force = true)
        cameraView?.disableView()
        if (poseRecording) {
            setPoseRecording(false)
        }
    }

    /**
     * 单独启停 Pose 记录的低层入口。
     *
     * 业务项目通常应使用 [startEvidenceRecording] 同时管理视频和 Pose。
     */
    fun setPoseRecording(enabled: Boolean) {
        poseRecording = enabled
        engine.setRecording(enabled)
    }

    /**
     * 同时开始录制视频和 Pose。二者共用 recordingId，便于后台关联同一次采集。
     *
     * @param recordingId 业务录制标识，同时用作视频文件名。
     * @param videoBitrate H.264 视频码率，单位 bit/s。
     * @return 成功时返回录制会话，失败时返回 null 并触发 [OpenVinsSdkListener.onError]。
     */
    @JvmOverloads
    fun startEvidenceRecording(
        recordingId: String = timestampName(),
        videoBitrate: Int = DEFAULT_VIDEO_BITRATE,
    ): OpenVinsRecordingSession? {
        if (activeRecording != null) {
            listener?.onError("当前已经在录制视频和 Pose。")
            return null
        }
        val config = storageConfig
        val view = cameraView
        if (config == null || view == null) {
            listener?.onError("开始录制前，请先配置存储目录并绑定相机视图。")
            return null
        }
        val safeId = recordingId.toSafeFileName()
        val videoFile = File(config.videoDirectory, "$safeId.mp4")
        if (!view.startRecording(videoFile.absolutePath, videoBitrate)) {
            listener?.onError("视频录制启动失败。")
            return null
        }

        setPoseRecording(true)
        return OpenVinsRecordingSession(
            recordingId = safeId,
            startedAtMs = System.currentTimeMillis(),
            videoFile = videoFile,
            poseDirectory = config.poseDirectory,
        ).also { activeRecording = it }
    }

    /**
     * 同时停止视频和 Pose，并返回本次生成的文件。
     *
     * @return 当前没有录制时返回 null，否则返回视频与 Pose 文件结果。
     */
    fun stopEvidenceRecording(): OpenVinsRecordingResult? {
        val session = activeRecording ?: return null
        activeRecording = null
        if (poseRecording) {
            setPoseRecording(false)
        }
        val videoPath = cameraView?.stopRecording()
        val poseFile = session.poseDirectory.listFiles()
            ?.filter { it.isFile && it.name.endsWith("pose0.csv") && it.lastModified() >= session.startedAtMs - 1_000L }
            ?.maxByOrNull { it.lastModified() }
        return OpenVinsRecordingResult(
            session = session,
            endedAtMs = System.currentTimeMillis(),
            videoFile = videoPath?.let(::File)?.takeIf { it.exists() },
            poseFile = poseFile,
        )
    }

    /** 当前是否正在同步录制视频和 Pose。 */
    fun isEvidenceRecording(): Boolean = activeRecording != null

    /**
     * 仅使用当前 Pose 执行一次查重，并将非重复点加入候选。
     *
     * 该方法不拍照；正式拍摄流程应优先使用 [capturePen]。
     */
    fun checkBarnRevisit(): BarnRevisitResult? {
        val snapshot = getTrajectorySnapshot() ?: return null
        return evaluateBarnRevisit(snapshot, commitCandidate = true)
    }

    /** 预览当前查重结果，不会把当前 Pose 加入后续查重候选。 */
    fun previewBarnRevisit(): BarnRevisitResult? {
        val snapshot = getTrajectorySnapshot() ?: return null
        return evaluateBarnRevisit(snapshot, commitCandidate = false)
    }

    @Deprecated(
        message = "请使用 capturePen() 同时保存照片、Pose 和查重结果。",
        replaceWith = ReplaceWith("capturePen(request, callback)"),
    )
    fun capturePenCheck(request: PenCheckRequest): PenCaptureRecord? {
        val session = activeSession
        if (session == null) {
            listener?.onError("业务拍摄会话尚未开始。")
            return null
        }
        val snapshot = getTrajectorySnapshot() ?: return null
        val result = checkBarnRevisit() ?: return null
        val record = PenCaptureRecord(
            sessionId = session.sessionId,
            policyId = session.policyId,
            barnId = session.barnId,
            penId = request.penId,
            capturedAtMs = System.currentTimeMillis(),
            pose = snapshot.currentPose,
            trajectory = snapshot,
            revisitResult = result,
            expectedPigCount = request.expectedPigCount,
            operatorRemark = request.operatorRemark,
        )
        penCaptureRecords.add(record)
        listener?.onPenCaptured(record)
        return record
    }

    /**
     * 完成一次业务拍摄：保存下一帧原始画面，并将同一时刻的 Pose、轨迹和查重结果一起返回。
     * 照片保存失败时不会写入候选点，避免产生没有影像证据的查重记录。
     *
     * @param request 当前猪圈的业务参数。
     * @param callback 成功时返回照片与业务记录，失败时返回 null。
     */
    fun capturePen(
        request: PenCheckRequest,
        callback: (OpenVinsCaptureResult?) -> Unit,
    ) {
        val session = activeSession
        val config = storageConfig
        val view = cameraView
        if (session == null || config == null || view == null) {
            listener?.onError("拍照前，请先开始业务会话、配置存储目录并绑定相机视图。")
            callback(null)
            return
        }
        if (captureInProgress) {
            listener?.onError("正在保存上一张照片，请稍后再试。")
            callback(null)
            return
        }
        if (!getStatus().canUsePose) {
            listener?.onError("当前定位状态不稳定，暂时无法记录可靠 Pose。")
            callback(null)
            return
        }

        val snapshot = getTrajectorySnapshot()
        if (snapshot == null) {
            listener?.onError("当前 Pose 暂不可用。")
            callback(null)
            return
        }
        val capturedAtMs = System.currentTimeMillis()
        val photoFile = File(
            config.photoDirectory,
            "${session.sessionId.toSafeFileName()}_${request.penId.toSafeFileName()}_$capturedAtMs.jpg"
        )
        captureInProgress = true
        view.captureRawImage(photoFile.absolutePath) { savedFile ->
            captureInProgress = false
            if (savedFile == null) {
                listener?.onError("当前相机画面保存失败。")
                callback(null)
                return@captureRawImage
            }

            val revisitResult = evaluateBarnRevisit(snapshot, commitCandidate = true)
            val record = PenCaptureRecord(
                sessionId = session.sessionId,
                policyId = session.policyId,
                barnId = session.barnId,
                penId = request.penId,
                capturedAtMs = capturedAtMs,
                pose = snapshot.currentPose,
                trajectory = snapshot,
                revisitResult = revisitResult,
                expectedPigCount = request.expectedPigCount,
                operatorRemark = request.operatorRemark,
                photoPath = savedFile.absolutePath,
            )
            penCaptureRecords.add(record)
            listener?.onPenCaptured(record)
            callback(OpenVinsCaptureResult(record, savedFile))
        }
    }

    /** 将当前会话和全部拍摄记录序列化为可上传的 JSON 字符串。 */
    fun exportSessionJson(): String {
        val session = activeSession
        val root = JSONObject()
        if (session != null) {
            root.put("sessionId", session.sessionId)
            root.put("policyId", session.policyId)
            root.put("barnId", session.barnId)
            root.put("startedAtMs", session.startedAtMs)
        }
        val records = JSONArray()
        for (record in penCaptureRecords) {
            records.put(record.toJson())
        }
        root.put("penRecords", records)
        return root.toString()
    }

    /** 获取当前可靠 Pose；VIO 尚未初始化时返回 null。 */
    fun getCurrentPose(): OpenVinsPose? {
        val position = DoubleArray(3)
        val quaternion = DoubleArray(4)
        if (!engine.getCurrentPose(position, quaternion)) {
            return null
        }
        return OpenVinsPose(position, quaternion)
    }

    /** 获取当前轨迹快照，最多返回 [maxSize] 个轨迹点。 */
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
        return OpenVinsTrajectory(posFloats, quatFloats, pose).also {
            // 保留最后可靠快照，使定位停止后仍能导出最终轨迹图。
            lastTrajectorySnapshot = it
        }
    }

    /** 将下一帧原始相机画面保存到 [file]，结果在主线程通过 [callback] 返回。 */
    fun captureCameraImage(file: File, callback: (File?) -> Unit) {
        val view = cameraView
        if (view == null) {
            listener?.onError("尚未绑定相机视图。")
            callback(null)
            return
        }
        view.captureRawImage(file.absolutePath, callback = callback)
    }

    /** 将当前轨迹视图保存到 [file]；未绑定轨迹视图时返回 null。 */
    fun captureTrajectoryImage(file: File, callback: (File?) -> Unit) {
        val view = trajectoryView
        if (view == null) {
            listener?.onError("尚未绑定轨迹视图。")
            callback(null)
            return
        }
        view.captureImage(file.absolutePath, callback = callback)
    }

    /**
     * 不依赖 [Trajectory3DView]，直接将当前轨迹快照导出为俯视 JPEG 图片。
     *
     * 页面不展示轨迹时应使用该方法。绘制在后台线程执行，结果回调位于主线程。
     */
    fun exportTrajectoryImage(
        file: File,
        config: TrajectoryImageConfig = TrajectoryImageConfig(),
        callback: (File?) -> Unit,
    ) {
        val snapshot = getTrajectorySnapshot() ?: lastTrajectorySnapshot
        if (snapshot == null || snapshot.positions.isEmpty()) {
            listener?.onError("当前没有可导出的轨迹数据。")
            callback(null)
            return
        }
        Thread {
            val result = TrajectoryImageExporter.export(snapshot, file, config)
            mainHandler.post {
                if (result == null) {
                    listener?.onError("轨迹图片导出失败。")
                }
                callback(result)
            }
        }.start()
    }

    /** 用户确认继续后，从最后可靠轨迹点启动恢复与坐标对齐。 */
    fun resumeTrajectory() {
        engine.resumeTrajectory()
    }

    override fun onFrame(matAddr: Long, timestampSec: Double) {
        engine.processImage(matAddr, timestampSec)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        event ?: return
        val sample = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> imuSampleSynchronizer.addAccelerometer(event.values, event.timestamp)
            Sensor.TYPE_GYROSCOPE -> imuSampleSynchronizer.addGyroscope(event.values, event.timestamp)
            else -> null
        } ?: return

        engine.processImu(
            sample.accel[0], sample.accel[1], sample.accel[2],
            sample.gyro[0], sample.gyro[1], sample.gyro[2],
            sample.timestampNs * 1e-9,
            sample.pairDeltaNs * 1e-9,
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun registerSensors() {
        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (accel == null || gyro == null) {
            listener?.onError("当前设备缺少可用的加速度计或陀螺仪。")
            return
        }
        sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST)
    }

    private fun unregisterSensors() {
        sensorManager.unregisterListener(this)
        imuSampleSynchronizer.reset()
    }

    private fun buildTrackingStatus(): OpenVinsStatus {
        val state = OpenVinsTrackingState.fromNative(engine.getTrackingState())
        val pauseReason = if (state == OpenVinsTrackingState.PAUSED) {
            engine.getTrajectoryPauseReason()
        } else {
            null
        }
        return OpenVinsStatus(state, pauseReason)
    }

    private fun dispatchTrackingStatus(force: Boolean = false) {
        val status = buildTrackingStatus()
        if (force || status != lastStatus) {
            lastStatus = status
            listener?.onTrackingStateChanged(status)
        }
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
        private const val DEFAULT_VIDEO_BITRATE = 1_000_000
    }

    private fun evaluateBarnRevisit(
        snapshot: OpenVinsTrajectory,
        commitCandidate: Boolean,
    ): BarnRevisitResult {
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
        val sdkResult = BarnRevisitResult.from(
            result,
            trajectoryRevisitor.shiftingTrajectory(pose.position, pose.quaternion),
        )
        // 重复点保留在业务拍摄记录中，但不作为新的查重基准，避免连续放大一次误报。
        if (commitCandidate && !sdkResult.isRevisit) {
            candidateTranslations.add(pose.position.copyOf())
            candidateQuaternions.add(pose.quaternion.copyOf())
        }
        listener?.onBarnRevisitChecked(sdkResult)
        return sdkResult
    }
}

private fun PenCaptureRecord.toJson(): JSONObject {
    return JSONObject()
        .put("sessionId", sessionId)
        .put("policyId", policyId)
        .put("barnId", barnId)
        .put("penId", penId)
        .put("capturedAtMs", capturedAtMs)
        .put("expectedPigCount", expectedPigCount)
        .put("operatorRemark", operatorRemark)
        .put("photoPath", photoPath)
        .put("pose", pose.toJson())
        .put("trajectory", trajectory.toJson())
        .put("revisitResult", revisitResult.toJson())
}

private fun OpenVinsPose.toJson(): JSONObject {
    return JSONObject()
        .put("position", position.toJsonArray())
        .put("quaternion", quaternion.toJsonArray())
}

private fun OpenVinsTrajectory.toJson(): JSONObject {
    return JSONObject()
        .put("positions", positions.toJsonArray())
        .put("quaternions", quaternions.toJsonArray())
        .put("currentPose", currentPose.toJson())
}

private fun BarnRevisitResult.toJson(): JSONObject {
    return JSONObject()
        .put("isRevisit", isRevisit)
        .put("isRetrieve", isRetrieve)
        .put("isDuplicated", isDuplicated)
        .put("isMovingFast", isMovingFast)
        .put("impact", impact)
        .put("outerImpact", outerImpact)
        .put("rotationError", rotationError)
        .put("pointDistance", pointDistance)
        .put("isSameSide", isSameSide)
}

private fun DoubleArray.toJsonArray(): JSONArray {
    val array = JSONArray()
    for (value in this) array.put(value)
    return array
}

private fun File.ensureDirectory(): Boolean {
    return (isDirectory || mkdirs()) && !isFile
}

private fun timestampName(): String {
    return SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(Date())
}

private fun String.toSafeFileName(): String {
    return replace(Regex("[^A-Za-z0-9._-]"), "_").take(80).ifBlank { "capture" }
}

private fun FloatArray.toJsonArray(): JSONArray {
    val array = JSONArray()
    for (value in this) array.put(value.toDouble())
    return array
}
