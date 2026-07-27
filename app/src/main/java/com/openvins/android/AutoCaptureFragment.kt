package com.openvins.app

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Environment
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import com.openvins.android.Camera2ResView
import com.openvins.android.sdk.BarnRevisitConfig
import com.openvins.android.sdk.OpenVinsSdk
import com.openvins.android.sdk.OpenVinsSdkListener
import com.openvins.android.sdk.OpenVinsStatus
import com.openvins.android.sdk.OpenVinsStorageConfig
import com.openvins.android.sdk.OpenVinsTrackingState
import com.openvins.android.sdk.PenCheckRequest
import java.io.File

/** Activity 发起一次拍摄后收到的结果。 */
data class FragmentCaptureResult(
    /** SDK 自动生成的拍摄点编号。 */
    val penId: String,
    /** 照片绝对路径。 */
    val photoPath: String,
    /** 当前拍摄是否与同侧历史拍摄点重复。 */
    val isDuplicate: Boolean,
    /** 与最佳同侧历史拍摄点的水平距离，单位为米。 */
    val duplicateDistanceMeters: Double,
)

/** Activity 完成任务后收到的全部文件结果。 */
data class FragmentTaskResult(
    /** 本次任务的时间戳标识。 */
    val taskId: String,
    /** 本次采集视频绝对路径，未成功录制时为 null。 */
    val videoPath: String?,
    /** 本次 Pose CSV 绝对路径，未成功录制时为 null。 */
    val posePath: String?,
    /** 最终轨迹图片绝对路径，轨迹不足时为 null。 */
    val trajectoryImagePath: String?,
    /** 本次任务全部拍摄记录的 JSON。 */
    val sessionJson: String,
)

/** Fragment 向宿主 Activity 返回业务结果的接口。 */
interface AutoCaptureFragmentListener {
    /** 一次拍摄完成。 */
    fun onCaptureCompleted(result: FragmentCaptureResult)

    /** 整个采集任务完成，所有文件已经关闭并生成。 */
    fun onTaskCompleted(result: FragmentTaskResult)

    /** Fragment 或 SDK 操作失败。 */
    fun onCaptureError(message: String)
}

/**
 * 自动管理 OpenVINS 生命周期的相机 Fragment。
 *
 * Fragment 不包含业务按钮，由 Activity 调用 [capture] 和 [finishTask]。
 */
class AutoCaptureFragment : Fragment(), OpenVinsSdkListener {

    private lateinit var sdk: OpenVinsSdk
    private lateinit var cameraView: Camera2ResView
    private lateinit var statusText: TextView
    private lateinit var storageRoot: File

    private var listener: AutoCaptureFragmentListener? = null
    private var isFragmentResumed = false
    private var isTrackingStarted = false
    private var isCaptureInProgress = false
    private var isTaskFinished = false
    private var pauseDialog: AlertDialog? = null
    private var nextCaptureIndex = 1

    private val taskId: String
        get() = requireArguments().getString(ARG_TASK_ID).orEmpty()

    override fun onAttach(context: Context) {
        super.onAttach(context)
        listener = context as? AutoCaptureFragmentListener
            ?: error("宿主 Activity 必须实现 AutoCaptureFragmentListener。")
    }

    override fun onDetach() {
        pauseDialog?.dismiss()
        pauseDialog = null
        listener = null
        super.onDetach()
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?,
    ): View {
        return inflater.inflate(R.layout.fragment_auto_capture, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        cameraView = view.findViewById(R.id.openvins_camera)
        statusText = view.findViewById(R.id.tracking_status)
        initializeSdk()
        requestCameraPermissionIfNeeded()
    }

    private fun initializeSdk() {
        val documents = requireContext().getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
            ?: requireContext().filesDir
        storageRoot = File(documents, "openvins/$taskId")

        sdk = OpenVinsSdk(requireContext(), this).apply {
            bindCameraView(cameraView)
            initialize(
                OpenVinsStorageConfig(
                    rootDirectory = storageRoot,
                    debugLoggingEnabled = false,
                )
            )
            configureBarnRevisit(
                BarnRevisitConfig(
                    pointDistanceMeters = 0.5,
                    sameSideMaxAngleDegrees = 90.0,
                )
            )
            setStatusOverlayEnabled(false)
            // 没有外部业务编号时统一使用任务时间戳，便于关联本次所有文件。
            beginInsuranceSession(
                policyId = taskId,
                barnId = taskId,
                sessionId = taskId,
            )
        }
    }

    override fun onResume() {
        super.onResume()
        isFragmentResumed = true
        if (!isTaskFinished && hasCameraPermission()) {
            startWhenCameraViewReady()
        }
    }

    override fun onPause() {
        isFragmentResumed = false
        // 跳转到同一应用的其他页面时保持相机、IMU、轨迹、视频和 Pose 连续运行。
        // 采集资源只在完成任务或当前 Fragment 被真正移除时释放。
        super.onPause()
    }

    override fun onDestroyView() {
        val shouldReleaseCollection = !isTaskFinished && (isRemoving || activity?.isFinishing == true)
        if (shouldReleaseCollection && ::sdk.isInitialized && isTrackingStarted) {
            sdk.stopEvidenceRecording()
            sdk.stop()
            isTrackingStarted = false
        }
        pauseDialog?.dismiss()
        pauseDialog = null
        super.onDestroyView()
    }

    private fun startAutomaticCollection() {
        if (isTrackingStarted || isTaskFinished) return
        sdk.notifyCameraPermissionGranted()
        sdk.start()
        isTrackingStarted = true
    }

    /**
     * 等待 Fragment 完成布局并创建 Surface 后再启动相机，避免首次进入页面黑屏。
     */
    private fun startWhenCameraViewReady() {
        cameraView.post {
            if (isAdded && isFragmentResumed && !isTaskFinished) {
                startAutomaticCollection()
            }
        }
    }

    /**
     * 由 Activity 调用拍摄。
     *
     * 结果通过 [AutoCaptureFragmentListener.onCaptureCompleted] 返回。
     */
    fun capture() {
        if (!::sdk.isInitialized || isTaskFinished) {
            listener?.onCaptureError("当前任务已经结束，无法继续拍摄。")
            return
        }
        if (isCaptureInProgress) {
            listener?.onCaptureError("正在保存上一张照片，请稍后再试。")
            return
        }
        if (!sdk.getStatus().canUsePose) {
            listener?.onCaptureError("定位尚未稳定，请等待页面提示定位正常后再拍摄。")
            return
        }

        isCaptureInProgress = true
        val penId = "PEN-${nextCaptureIndex.toString().padStart(3, '0')}"
        sdk.capturePen(PenCheckRequest(penId = penId)) { result ->
            isCaptureInProgress = false
            if (result == null) return@capturePen

            nextCaptureIndex += 1
            val revisit = result.record.revisitResult
            listener?.onCaptureCompleted(
                FragmentCaptureResult(
                    penId = penId,
                    photoPath = result.photoFile.absolutePath,
                    isDuplicate = revisit.isRevisit,
                    duplicateDistanceMeters = revisit.pointDistance,
                )
            )
        }
    }

    /**
     * 由 Activity 调用完成任务。
     *
     * 方法会停止视频、Pose 和 VIO，生成最终轨迹图后通过
     * [AutoCaptureFragmentListener.onTaskCompleted] 返回所有路径。
     */
    fun finishTask() {
        if (!::sdk.isInitialized || isTaskFinished) return
        isTaskFinished = true

        val recording = sdk.stopEvidenceRecording()
        if (isTrackingStarted) {
            sdk.stop()
            isTrackingStarted = false
        }
        val sessionJson = sdk.exportSessionJson()
        val trajectoryFile = File(storageRoot, "trajectory/final.jpg")
        sdk.exportTrajectoryImage(trajectoryFile) { imageFile ->
            sdk.finishInsuranceSession()
            listener?.onTaskCompleted(
                FragmentTaskResult(
                    taskId = taskId,
                    videoPath = recording?.videoFile?.absolutePath,
                    posePath = recording?.poseFile?.absolutePath,
                    trajectoryImagePath = imageFile?.absolutePath,
                    sessionJson = sessionJson,
                )
            )
        }
    }

    override fun onTrackingStateChanged(status: OpenVinsStatus) {
        if (!isAdded) return
        statusText.text = trackingStateText(status.state)

        if (status.state == OpenVinsTrackingState.PAUSED) {
            showTrajectoryPausedDialog(status.pauseReason)
        }

        // 首次定位可靠后自动开始同步记录视频和 Pose。
        if (status.canUsePose && !isTaskFinished && !sdk.isEvidenceRecording()) {
            sdk.startEvidenceRecording(recordingId = taskId)
        }
    }

    /**
     * 漂移发生后由底层冻结异常点，用户确认后从最后可靠轨迹点重新接续。
     * 同一次暂停只显示一个弹窗，避免状态轮询造成重复提示。
     */
    private fun showTrajectoryPausedDialog(reason: Int?) {
        if (!isFragmentResumed || isTaskFinished || pauseDialog?.isShowing == true) return

        pauseDialog = AlertDialog.Builder(requireContext())
            .setTitle("轨迹疑似漂移")
            .setMessage(
                "异常轨迹数据已停止记录。\n\n" +
                    "原因：${pauseReasonText(reason)}\n\n" +
                    "点击继续后，将从最后一个可靠轨迹点接着记录。"
            )
            .setCancelable(false)
            .setPositiveButton("继续") { _, _ ->
                pauseDialog = null
                statusText.text = "正在接续上次轨迹"
                sdk.resumeTrajectory()
            }
            .setNegativeButton("结束任务") { _, _ ->
                pauseDialog = null
                finishTask()
            }
            .create()
            .also { it.show() }
    }

    /** 将底层暂停原因转换成用户容易理解的中文提示。 */
    private fun pauseReasonText(reason: Int?): String {
        return when (reason) {
            3 -> "单帧位置跳变过大"
            4 -> "移动速度异常"
            7 -> "转身后出现反向位移"
            8 -> "转身后速度异常"
            9 -> "转身后视觉特征不足"
            11 -> "画面不可用期间设备发生移动"
            14 -> "定位连续输出异常位移"
            else -> "轨迹点连续异常"
        }
    }

    override fun onError(message: String, throwable: Throwable?) {
        Log.e(TAG, message, throwable)
        listener?.onCaptureError(message)
    }

    private fun trackingStateText(state: OpenVinsTrackingState): String {
        return when (state) {
            OpenVinsTrackingState.STOPPED -> "定位未开始"
            OpenVinsTrackingState.INITIALIZING -> "正在初始化，请保持画面清晰并缓慢移动手机"
            OpenVinsTrackingState.TRACKING -> "定位正常，可以拍摄"
            OpenVinsTrackingState.STATIONARY -> "定位正常，可以拍摄"
            OpenVinsTrackingState.CAMERA_UNAVAILABLE -> "画面不可用，请恢复摄像头"
            OpenVinsTrackingState.ALIGNING -> "正在接续上次轨迹"
            OpenVinsTrackingState.PAUSED -> "检测到轨迹异常，已暂停记录"
        }
    }

    private fun requestCameraPermissionIfNeeded() {
        if (!hasCameraPermission()) {
            requestPermissions(arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_REQUEST)
        }
    }

    private fun hasCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            requireContext(),
            Manifest.permission.CAMERA,
        ) == PackageManager.PERMISSION_GRANTED
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != CAMERA_PERMISSION_REQUEST) return
        if (grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED) {
            if (isFragmentResumed) startWhenCameraViewReady()
        } else {
            listener?.onCaptureError("没有摄像头权限，无法开始拍摄。")
        }
    }

    companion object {
        private const val TAG = "AutoCaptureFragment"
        private const val CAMERA_PERMISSION_REQUEST = 1001
        private const val ARG_TASK_ID = "task_id"

        /** 创建新任务，任务 ID 自动使用当前毫秒时间戳。 */
        fun newInstance(): AutoCaptureFragment {
            return AutoCaptureFragment().apply {
                arguments = Bundle().apply {
                    putString(ARG_TASK_ID, System.currentTimeMillis().toString())
                }
            }
        }
    }
}
