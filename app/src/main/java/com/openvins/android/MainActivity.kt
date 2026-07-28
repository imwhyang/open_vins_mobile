package com.openvins.app

import android.Manifest
import android.content.pm.PackageManager
import android.content.pm.ApplicationInfo
import android.os.Bundle
import android.os.Environment
import android.text.InputType
import android.util.Log
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.android.material.floatingactionbutton.FloatingActionButton
import com.openvins.android.Camera2ResView
import com.openvins.android.sdk.BarnRevisitConfig
import com.openvins.android.sdk.BarnRevisitResult
import com.openvins.android.sdk.OpenVinsSdk
import com.openvins.android.sdk.OpenVinsSdkListener
import com.openvins.android.sdk.OpenVinsStatus
import com.openvins.android.sdk.OpenVinsStorageConfig
import com.openvins.android.sdk.OpenVinsTrackingState
import com.openvins.android.sdk.PenCaptureRecord
import com.openvins.android.sdk.PenCheckRequest
import java.io.File

/**
 * SDK 功能示例页。业务项目只需要负责权限、界面和业务参数，不需要直接处理相机帧或 IMU。
 */
class MainActivity : AppCompatActivity(), OpenVinsSdkListener {

    private lateinit var sdk: OpenVinsSdk
    private lateinit var cameraView: Camera2ResView
    private lateinit var statusText: TextView
    private lateinit var recordButton: FloatingActionButton
    private lateinit var systemButton: FloatingActionButton

    private var storageRoot: File? = null
    private var isTrackingStarted = false
    private var nextCaptureIndex = 1
    private var isPauseDialogShowing = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)

        cameraView = findViewById(R.id.test_view)
        statusText = findViewById(R.id.tv_pose)
        recordButton = findViewById(R.id.toggle_record)
        systemButton = findViewById(R.id.toggle_reset)

        initializeSdk()
        bindActions()
        requestCameraPermission()
    }

    /**
     * 推荐初始化顺序：创建 SDK、绑定视图、配置存储和查重规则、开始业务会话。
     */
    private fun initializeSdk() {
        sdk = OpenVinsSdk(this, this).apply {
            bindCameraView(cameraView)
            // 示例页保留 init/zvupt；正式项目可关闭并完全使用状态回调。
            setStatusOverlayEnabled(true)
            // 原生特征点和定位参数仅用于 Debug 调试，Release 包不展示。
            setDebugVisualizationEnabled(
                applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE != 0
            )
        }
        configureDefaultStorage()
        configureRevisitRules()
        sdk.beginInsuranceSession(
            policyId = DEMO_POLICY_ID,
            barnId = DEMO_BARN_ID,
        )
    }

    private fun configureDefaultStorage() {
        val documents = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS) ?: filesDir
        configureSdkStorage(File(documents, "openvins"))
    }

    private fun configureSdkStorage(root: File): Boolean {
        return try {
            sdk.initialize(
                OpenVinsStorageConfig(
                    rootDirectory = root,
                    // 示例项目开启诊断日志；正式接入保持 false，只记录 pose0.csv。
                    debugLoggingEnabled = true,
                )
            )
            storageRoot = root
            true
        } catch (error: IllegalArgumentException) {
            onError("存储目录不可用：${root.absolutePath}", error)
            false
        }
    }

    private fun configureRevisitRules() {
        sdk.configureBarnRevisit(
            BarnRevisitConfig(
                pointDistanceMeters = 0.5,
                // 镜头水平夹角小于 90 度视为过道同侧。
                sameSideMaxAngleDegrees = 90.0,
            )
        )
    }

    private fun bindActions() {
        findViewById<FloatingActionButton>(R.id.open_folder).setOnClickListener {
            showStorageDialog()
        }
        recordButton.setOnClickListener {
            toggleEvidenceRecording()
        }
        systemButton.setOnClickListener {
            toggleSystem()
        }
        findViewById<FloatingActionButton>(R.id.take_photo).setOnClickListener {
            capturePenEvidence()
        }
        findViewById<Button>(R.id.toggle_trajectory).setOnClickListener {
            captureTrajectoryImage()
        }
        findViewById<Button>(R.id.toggle_check).setOnClickListener {
            // 调试预览不会污染正式拍摄的查重候选。
            showRevisitResult(sdk.previewBarnRevisit())
        }
    }

    private fun toggleSystem() {
        if (isTrackingStarted) {
            sdk.stop()
            isTrackingStarted = false
            systemButton.setImageResource(R.drawable.ic_baseline_play_arrow_24)
            recordButton.setImageResource(R.drawable.ic_save)
        } else {
            sdk.start()
            isTrackingStarted = true
            systemButton.setImageResource(R.drawable.ic_stop_system)
        }
    }

    private fun toggleEvidenceRecording() {
        if (!isTrackingStarted) {
            toast("请先启动轨迹")
            return
        }
        if (sdk.isEvidenceRecording()) {
            val result = sdk.stopEvidenceRecording()
            recordButton.setImageResource(R.drawable.ic_save)
            toast("录制完成：${result?.poseFile?.name ?: "Pose 文件"}")
        } else {
            val recording = sdk.startEvidenceRecording()
            if (recording != null) {
                recordButton.setImageResource(R.drawable.ic_baseline_close_24)
                toast("已开始同步记录视频和 Pose")
            }
        }
    }

    private fun capturePenEvidence() {
        if (!isTrackingStarted) {
            toast("请先启动并等待定位完成")
            return
        }
        val request = PenCheckRequest(
            penId = "PEN-${nextCaptureIndex.toString().padStart(3, '0')}",
        )
        sdk.capturePen(request) { result ->
            if (result == null) return@capturePen
            nextCaptureIndex += 1
            val revisit = result.record.revisitResult
            if (revisit.isRevisit) {
                AlertDialog.Builder(this)
                    .setTitle("拍摄位置可能重复")
                    .setMessage(
                        "照片已保存。当前镜头与历史记录朝向同侧，" +
                                "拍摄位置相距 ${"%.2f".format(revisit.pointDistance)} 米，请确认是否重复拍摄。"
                    )
                    .setPositiveButton("知道了", null)
                    .show()
            } else {
                toast("照片和 Pose 已记录：${result.photoFile.name}")
            }
        }
    }

    private fun captureTrajectoryImage() {
        val root = storageRoot ?: return
        val file = File(root, "trajectory/trajectory_${System.currentTimeMillis()}.jpg")
        // 无需在页面中放置 Trajectory3DView，SDK 直接根据最终轨迹生成俯视图。
        sdk.exportTrajectoryImage(file) { saved ->
            Log.i(TAG, "轨迹图片：${saved?.absoluteFile}")
            toast(if (saved != null) "轨迹图已保存" else "轨迹图保存失败")
        }
    }

    private fun showStorageDialog() {
        val input = EditText(this).apply {
            inputType = InputType.TYPE_CLASS_TEXT
            setText(storageRoot?.absolutePath.orEmpty())
        }
        AlertDialog.Builder(this)
            .setTitle("设置采集文件根目录")
            .setView(input)
            .setPositiveButton("确定") { _, _ ->
                if (sdk.isEvidenceRecording()) {
                    toast("请先停止当前录制")
                    return@setPositiveButton
                }
                if (configureSdkStorage(File(input.text.toString().trim()))) {
                    toast("存储目录已更新")
                }
            }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun requestCameraPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
        ) {
            sdk.notifyCameraPermissionGranted()
        } else {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.CAMERA),
                PERMISSION_REQUEST,
            )
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != PERMISSION_REQUEST) return
        if (grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED) {
            sdk.notifyCameraPermissionGranted()
        } else {
            onError("没有摄像头权限，无法开始轨迹定位。")
        }
    }

    override fun onTrackingStateChanged(status: OpenVinsStatus) {
        statusText.text = when (status.state) {
            OpenVinsTrackingState.STOPPED -> "定位未开始"
            OpenVinsTrackingState.INITIALIZING -> "正在初始化，请保持画面清晰并缓慢移动手机"
            OpenVinsTrackingState.TRACKING -> "定位正常"
            OpenVinsTrackingState.STATIONARY -> "定位正常，设备静止"
            OpenVinsTrackingState.CAMERA_UNAVAILABLE -> "摄像头画面不可用，当前轨迹数据已过滤"
            OpenVinsTrackingState.ALIGNING -> "画面已恢复，正在接续上次轨迹"
            OpenVinsTrackingState.PAUSED -> "检测到轨迹异常，已暂停记录"
        }
        if (status.state == OpenVinsTrackingState.PAUSED) {
            showTrajectoryPausedDialog(status.pauseReason)
        }
    }

    private fun showTrajectoryPausedDialog(reason: Int?) {
        if (isPauseDialogShowing) return
        isPauseDialogShowing = true
        AlertDialog.Builder(this)
            .setTitle("轨迹疑似漂移")
            .setMessage("异常数据已停止记录。\n\n原因：${pauseReasonText(reason)}\n\n恢复后将接着最后可靠点继续。")
            .setPositiveButton("继续") { _, _ ->
                sdk.resumeTrajectory()
                isPauseDialogShowing = false
            }
            .setNegativeButton("停止任务") { _, _ ->
                sdk.stop()
                isTrackingStarted = false
                systemButton.setImageResource(R.drawable.ic_baseline_play_arrow_24)
                isPauseDialogShowing = false
            }
            .setOnCancelListener { isPauseDialogShowing = false }
            .show()
    }

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

    private fun showRevisitResult(result: BarnRevisitResult?) {
        if (result == null) {
            toast("当前还没有可靠 Pose")
            return
        }
        val message = if (result.isRevisit) {
            "存在重复风险，距离 ${"%.2f".format(result.pointDistance)} 米"
        } else {
            "当前位置未发现重复风险"
        }
        toast(message)
    }

    override fun onPenCaptured(record: PenCaptureRecord) {
        Log.i(TAG, "采集完成：${record.penId}, ${record.photoPath}")
    }

    override fun onError(message: String, throwable: Throwable?) {
        Log.e(TAG, message, throwable)
        runOnUiThread { toast(message) }
    }

    override fun onStop() {
        if (isTrackingStarted) {
            sdk.stop()
            isTrackingStarted = false
        }
        super.onStop()
    }

    private fun toast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    companion object {
        private const val TAG = "MainActivity"
        private const val PERMISSION_REQUEST = 1
        private const val DEMO_POLICY_ID = "DEMO-POLICY"
        private const val DEMO_BARN_ID = "DEMO-BARN"
    }
}
