package com.openvins.app

import com.openvins.android.Camera2ResView
import com.openvins.android.CameraFrameListener
import com.openvins.android.ImuSampleSynchronizer
import com.openvins.android.Trajectory3DView
import com.openvins.android.VioEngine

import android.Manifest
import android.content.Context
import android.content.DialogInterface
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Environment
import android.text.InputType
import android.util.Log
import android.view.View
import android.view.WindowManager
import android.widget.EditText
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.google.android.material.floatingactionbutton.FloatingActionButton
import java.io.File
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.RelativeLayout
import android.widget.TextView
import androidx.core.content.ContextCompat
import com.openvins.android.component.TrajectoryRevisitor


class MainActivity : AppCompatActivity(), CameraFrameListener, SensorEventListener {
    var tvPose: TextView? = null
    private var mOpenCvCameraView: Camera2ResView? = null
    private var isRecording: Boolean = false
    private var isRunningOV: Boolean = false
    private var hasRecordFolder: Boolean = false
    private var recordFolder: String = ""

    private lateinit var sensorManager: SensorManager
    private var sensorAccel: Sensor? = null
    private var sensorGyro: Sensor? = null
    private val imuSampleSynchronizer = ImuSampleSynchronizer()

    private val trajectoryRevisitor: TrajectoryRevisitor = TrajectoryRevisitor()

    private var trajectoryView: Trajectory3DView? = null
    private var trajectoryPauseDialogShowing = false
    private var trajectoryPauseAcknowledged = false
    private val trajectoryUpdateHandler = Handler(Looper.getMainLooper())
    private val trajectoryUpdateRunnable = object : Runnable {
        override fun run() {
            updateTrajectoryView()
            trajectoryUpdateHandler.postDelayed(this, 50) // 20 Hz
        }
    }

    private var vioEngine = VioEngine()
   private val candidateTranslations: ArrayList<DoubleArray> = arrayListOf()
   private val candidateQuaternions: ArrayList<DoubleArray> = arrayListOf()

    init {
        Log.i(TAG, "Instantiated new " + this.javaClass)
    }


    public override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Permissions for Android 6+
        ActivityCompat.requestPermissions(
            this@MainActivity,
            arrayOf(
                Manifest.permission.CAMERA,
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
            ),
            PERMISSION_REQUEST
        )

        // Setup our camera
        setContentView(R.layout.activity_main)
        mOpenCvCameraView = findViewById<View>(R.id.test_view) as Camera2ResView
        mOpenCvCameraView!!.setCameraFrameListener(this)

        // Setup trajectory view
        trajectoryView = findViewById<Trajectory3DView>(R.id.trajectory_view)
        // Force initial render to show axes and grid
        trajectoryView?.forceRender()
        //mOpenCvCameraView!!.setMaxFrameSize(640, 480)
        //mOpenCvCameraView!!.setFocusMode(this, Camera.Parameters.FOCUS_MODE_INFINITY)

        // Check that we have our accelerometer and gyroscope sensors
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        sensorAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        sensorGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (sensorAccel == null) {
            Toast.makeText(
                applicationContext,
                "ERROR: unable to open accelerometer", Toast.LENGTH_LONG
            ).show()
        }
        if (sensorGyro == null) {
            Toast.makeText(
                applicationContext,
                "ERROR: unable to open accelerometer", Toast.LENGTH_LONG
            ).show()
        }

        // 权限申请是异步的，存储相关初始化在 onRequestPermissionsResult 权限授予后执行

        // Button for the user to change if they want to do that
        val fab_folder = findViewById(R.id.open_folder) as FloatingActionButton
        fab_folder.setOnClickListener {
            val builder: AlertDialog.Builder = AlertDialog.Builder(this)
            builder.setTitle("Desired Save Folder")
            val input = EditText(this)
            input.setInputType(InputType.TYPE_CLASS_TEXT)
            input.setText(recordFolder)
            builder.setView(input)
            // Set up the buttons
            builder.setPositiveButton("OK", object : DialogInterface.OnClickListener {
                override fun onClick(dialog: DialogInterface?, which: Int) {
                    val file = File(input.getText().toString())
                    recordFolder = file.toString()
                    if ((!file.isDirectory && !file.mkdirs()) || file.isFile) {
                        Toast.makeText(
                            applicationContext,
                            "ERROR: unable to create directory. ${file.toString()}",
                            Toast.LENGTH_LONG
                        ).show()
                    } else {
                        hasRecordFolder = true
                        vioEngine.setRecordFolder(recordFolder)
                    }
                }
            })
            builder.setNegativeButton("Cancel", object : DialogInterface.OnClickListener {
                override fun onClick(dialog: DialogInterface, which: Int) {
                    dialog.cancel()
                }
            })
            builder.show()

        }

        // Our record button
        val fab = findViewById(R.id.toggle_record) as FloatingActionButton
        fab.setOnClickListener {
            if (!hasRecordFolder) {
                Toast.makeText(this, "ERROR: select record folder first!", Toast.LENGTH_LONG).show()
                return@setOnClickListener
            }
            isRecording = if (isRecording) {
                fab.setImageResource(R.drawable.ic_save)
                false
            } else {
                fab.setImageResource(R.drawable.ic_baseline_close_24)
                true
            }
            vioEngine.setRecording(isRecording)
        }

        // Our start / stop openvins button
        val reset = findViewById(R.id.toggle_reset) as FloatingActionButton
        reset.setOnClickListener {
            trajectoryPauseDialogShowing = false
            trajectoryPauseAcknowledged = false
            isRunningOV = if (isRunningOV) {
                reset.setImageResource(R.drawable.ic_baseline_play_arrow_24)
//                时间戳
                val timestamp = System.currentTimeMillis() / 1000
                val externalStoragePublicDirectory =
                    Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                        .toString() + "/openvins/video/"
                mOpenCvCameraView!!.startRecording(externalStoragePublicDirectory + "/${timestamp}.mp4")
                false
            } else {
                reset.setImageResource(R.drawable.ic_stop_system)
                // Clear trajectory view when starting (in case there was leftover data)
                trajectoryView?.clearTrajectory()
                candidateTranslations.clear()
                candidateQuaternions.clear()
                trajectoryRevisitor.reset()
                val stopRecording = mOpenCvCameraView!!.stopRecording()
                Log.d(TAG, "Stopped recording: $stopRecording")
                true
            }
            vioEngine.toggleSystem(isRunningOV)

            tvPose = findViewById(R.id.tv_pose)
            val takePicButton = findViewById(R.id.take_photo) as FloatingActionButton
            takePicButton.setOnClickListener {
                val timestamp = System.currentTimeMillis() / 1000
                val externalStoragePublicDirectory =
                    Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                        .toString() + "/openvins/pointPic/"
                    mOpenCvCameraView!!.captureRawImage(externalStoragePublicDirectory + "/${timestamp}.jpg") { file ->
                        if (file != null) {
                            Toast.makeText(this, "Capture Success", Toast.LENGTH_LONG).show()
                            Log.d(TAG, "Capture Success" + file.absolutePath)
                        } else {
                            Toast.makeText(this, "Capture Failed", Toast.LENGTH_LONG).show()
                        }
                    }

            }

            val toggleTrajectory = findViewById(R.id.toggle_trajectory) as FloatingActionButton
            toggleTrajectory.setOnClickListener {
                val timestamp = System.currentTimeMillis() / 1000
                val externalStoragePublicDirectory =
                    Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                        .toString() + "/openvins/trajectory/"

                trajectoryView?.captureImage(externalStoragePublicDirectory + "/${timestamp}.jpg") { file ->
                    if (file != null) {
                        Log.i("TAG", "轨迹图已保存: ${file.absolutePath}")
                    }else
                        Log.i("TAG", "轨迹图保存失败")
                }
            }

            val toggleCheck = findViewById(R.id.toggle_check) as Button
            toggleCheck.setOnClickListener {
                checkCurrentPost()
            }
        }

    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        when (requestCode) {
            PERMISSION_REQUEST -> {
                val cameraGranted =
                    grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED
                if (cameraGranted) {
                    mOpenCvCameraView!!.setCameraPermissionGranted()
                } else {
                    Log.e(TAG, "Camera permission was not granted")
                    Toast.makeText(this, "Camera permission was not granted", Toast.LENGTH_LONG)
                        .show()
                }
                // 权限授予后（无论存储权限是否获得）执行初始化
                // Android 10+ 访问 getExternalFilesDir 不需要存储权限
                initFoldersAndConfig()
            }

            else -> {
                Log.e(TAG, "Unexpected permission request")
            }
        }
    }

    /**
     * 初始化录制目录与配置文件，需在存储权限确认后调用。
     */
    private fun initFoldersAndConfig() {
        val appPrivateFolderRoot = filesDir.toString()
        val appRecordFolder =
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                .toString() + "/openvins/pose"

        recordFolder = appRecordFolder
        val file = File(recordFolder)
        if ((!file.isDirectory && !file.mkdirs()) || file.isFile) {
            Toast.makeText(
                applicationContext,
                "ERROR: unable to create directory. $recordFolder",
                Toast.LENGTH_LONG
            ).show()
        } else {
            hasRecordFolder = true
            // 从 assets 解压配置文件到私有目录（仅首次）
            vioEngine.copyConfigIfNeeded(this)
            // 设置录制目录（公共，用户可访问）
            vioEngine.setRecordFolder(recordFolder)
            // 设置私有目录根路径（原生层会在其下查找 /config/ 子目录）
            vioEngine.setPrivateFolder(appPrivateFolderRoot)
        }
    }

    public override fun onPause() {
        super.onPause()
        if (mOpenCvCameraView != null) mOpenCvCameraView!!.disableView()
        sensorManager.unregisterListener(this)
        imuSampleSynchronizer.reset()
        trajectoryUpdateHandler.removeCallbacks(trajectoryUpdateRunnable)
    }

    public override fun onResume() {
        super.onResume()
        imuSampleSynchronizer.reset()
        // Activate camera feed
        mOpenCvCameraView!!.enableView()

        sensorAccel?.also { sensor ->
            sensorManager.registerListener(
                this,
                sensor,
                SensorManager.SENSOR_DELAY_FASTEST
            )
        }
        sensorGyro?.also { sensor ->
            sensorManager.registerListener(
                this,
                sensor,
                SensorManager.SENSOR_DELAY_FASTEST
            )
        }

        // Start trajectory updates
        trajectoryUpdateHandler.post(trajectoryUpdateRunnable)
    }

    public override fun onStop() {
        super.onStop()
        val stopRecording = mOpenCvCameraView!!.stopRecording()
        if (stopRecording != null) {
            Log.d(TAG, "Stopped recording: $stopRecording")
        }
        // Stop trajectory updates
        trajectoryUpdateHandler.removeCallbacks(trajectoryUpdateRunnable)

        // Unregister sensor listeners
        sensorManager.unregisterListener(this)

        // Stop OpenVINS system and clean up native resources
        if (isRunningOV) {
            vioEngine.toggleSystem(false)
        }

        // Stop recording if active
        if (isRecording) {
            vioEngine.setRecording(false)
        }

        // Disable camera view
        if (mOpenCvCameraView != null) {
            mOpenCvCameraView!!.disableView()
        }
    }

    override fun onFrame(matAddr: Long, timestampSec: Double) {
        // Native function processes the frame and updates it in-place
        // Mat address is already from native code, so we can use it directly
        vioEngine.processImage(matAddr, timestampSec)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        event ?: return
        val sample = when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> imuSampleSynchronizer.addAccelerometer(event.values, event.timestamp)
            Sensor.TYPE_GYROSCOPE -> imuSampleSynchronizer.addGyroscope(event.values, event.timestamp)
            else -> null
        } ?: return

        vioEngine.processImu(
            sample.accel[0], sample.accel[1], sample.accel[2],
            sample.gyro[0], sample.gyro[1], sample.gyro[2],
            sample.timestampNs * 1e-9,
            sample.pairDeltaNs * 1e-9,
        )
    }

    override fun onAccuracyChanged(p0: Sensor?, p1: Int) {
        Log.d(TAG, "[sensor]: accuracy level of ${p0.toString()} changed to $p1")
    }

    private fun updateTrajectoryView() {
        if (trajectoryView == null) return

        val visualRecoveryState = vioEngine.getVisualRecoveryState()
        if (visualRecoveryState != 0) {
            tvPose?.text = when (visualRecoveryState) {
                1 -> "摄像头画面不可用，轨迹暂不记录，请停止移动并恢复画面"
                2 -> "初始化已完成，正在对齐上次轨迹，请继续正常移动"
                3 -> "INIT 初始化中：保持画面清晰，缓慢移动手机以完成定位"
                else -> ""
            }
            tvPose?.setTextColor(ContextCompat.getColor(this, android.R.color.holo_orange_light))
        }

        if (vioEngine.isTrajectoryPaused()) {
            updateTrajectoryViewWithLastReliablePose()
            showTrajectoryPausedDialogIfNeeded(vioEngine.getTrajectoryPauseReason())
            return
        } else {
            trajectoryPauseAcknowledged = false
        }

        // Allocate arrays with maximum expected size (MAX_TRAJECTORY_POINTS = 10000)
        val maxSize = 10000
        val positions = DoubleArray(maxSize * 3)
        val quaternions = DoubleArray(maxSize * 4)
        val trajectorySize = vioEngine.getTrajectoryData(positions, quaternions)

        // Get current pose
        val currentPos = DoubleArray(3)
        val currentQuat = DoubleArray(4)
        if (!vioEngine.getCurrentPose(currentPos, currentQuat)) {
            if (trajectorySize > 0) {
                val lastIndex = trajectorySize - 1
                trajectoryView?.updateTrajectory(
                    FloatArray(trajectorySize * 3) { positions[it].toFloat() },
                    FloatArray(trajectorySize * 4) { quaternions[it].toFloat() },
                    floatArrayOf(
                        positions[lastIndex * 3].toFloat(),
                        positions[lastIndex * 3 + 1].toFloat(),
                        positions[lastIndex * 3 + 2].toFloat()
                    ),
                    floatArrayOf(
                        quaternions[lastIndex * 4].toFloat(),
                        quaternions[lastIndex * 4 + 1].toFloat(),
                        quaternions[lastIndex * 4 + 2].toFloat(),
                        quaternions[lastIndex * 4 + 3].toFloat()
                    )
                )
            }
            return // System not initialized
        }

        if (trajectorySize == 0) {
            // Empty trajectory or error
            trajectoryView?.updateTrajectory(
                floatArrayOf(), floatArrayOf(),
                FloatArray(3) { currentPos[it].toFloat() },
                FloatArray(4) { currentQuat[it].toFloat() })
            return
        }

        // Convert only the actual number of points to float arrays
        val posFloats = FloatArray(trajectorySize * 3)
        val quatFloats = FloatArray(trajectorySize * 4)
        for (i in 0 until trajectorySize * 3) {
            posFloats[i] = positions[i].toFloat()
        }
        for (i in 0 until trajectorySize * 4) {
            quatFloats[i] = quaternions[i].toFloat()
        }

        val currPosFloats = FloatArray(3) { currentPos[it].toFloat() }
        val currQuatFloats = FloatArray(4) { currentQuat[it].toFloat() }

//        当前数据是否飘移
        if (visualRecoveryState == 0) {
            val shifting = trajectoryRevisitor.shiftingTrajectory(
                currentPos, currentQuat
            )
            if (shifting) {
                tvPose?.text = String.format("shifting")
                val params = RelativeLayout.LayoutParams(
                    RelativeLayout.LayoutParams.WRAP_CONTENT,
                    RelativeLayout.LayoutParams.WRAP_CONTENT
                )
                params.setMargins(200, 400, 0, 0)
                tvPose?.layoutParams = params
            } else {
                tvPose?.text = ""
            }
        }

        // 判断路径重访状态
//        val result = trajectoryRevisitor.searchRevisitTrajectory(
//            posFloats, quatFloats, currentPos, currentQuat
//        )
//        tvPose?.text = String.format(
//            "Pose: currPosFloats: %f %f %f  isRevisit %b",
//            currPosFloats[0],
//            currPosFloats[1],
//            currPosFloats[2],
//            result.isRevisit
//        )
//        if (result.isRevisit) {
//            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.red))
//        } else {
//            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.white))
//        }


        // Update the 3D view
        trajectoryView?.updateTrajectory(posFloats, quatFloats, currPosFloats, currQuatFloats)
    }

    private fun updateTrajectoryViewWithLastReliablePose() {
        // 轨迹暂停时不要继续显示 VIO 的当前漂移位姿，方向框固定在最后一个可靠轨迹点上。
        val maxSize = 10000
        val positions = DoubleArray(maxSize * 3)
        val quaternions = DoubleArray(maxSize * 4)
        val trajectorySize = vioEngine.getTrajectoryData(positions, quaternions)
        if (trajectorySize <= 0) return

        val posFloats = FloatArray(trajectorySize * 3)
        val quatFloats = FloatArray(trajectorySize * 4)
        for (i in 0 until trajectorySize * 3) {
            posFloats[i] = positions[i].toFloat()
        }
        for (i in 0 until trajectorySize * 4) {
            quatFloats[i] = quaternions[i].toFloat()
        }

        val lastIndex = trajectorySize - 1
        val currPosFloats = floatArrayOf(
            positions[lastIndex * 3].toFloat(),
            positions[lastIndex * 3 + 1].toFloat(),
            positions[lastIndex * 3 + 2].toFloat()
        )
        val currentPos = DoubleArray(3)
        val currentQuat = DoubleArray(4)
        val currQuatFloats = if (vioEngine.getCurrentPose(currentPos, currentQuat)) {
            // native 暂停状态下会返回最后可靠方向，避免原地转身后又显示成转身前方向。
            FloatArray(4) { currentQuat[it].toFloat() }
        } else {
            floatArrayOf(
                quaternions[lastIndex * 4].toFloat(),
                quaternions[lastIndex * 4 + 1].toFloat(),
                quaternions[lastIndex * 4 + 2].toFloat(),
                quaternions[lastIndex * 4 + 3].toFloat()
            )
        }
        trajectoryView?.updateTrajectory(posFloats, quatFloats, currPosFloats, currQuatFloats)
    }

    private fun showTrajectoryPausedDialogIfNeeded(reason: Int) {
        if (trajectoryPauseDialogShowing || trajectoryPauseAcknowledged) return

        trajectoryPauseDialogShowing = true
        AlertDialog.Builder(this)
            .setTitle("轨迹疑似漂移")
            .setMessage("检测到轨迹数据异常，已暂停轨迹记录与绘制。\n\n原因：${trajectoryPauseReasonText(reason)}\n\n是否继续接着上次轨迹记录与绘制？")
            .setPositiveButton("继续") { dialog, _ ->
                vioEngine.resumeTrajectory()
                trajectoryPauseAcknowledged = false
                trajectoryPauseDialogShowing = false
                dialog.dismiss()
            }
            .setNegativeButton("暂不继续") { dialog, _ ->
                trajectoryPauseAcknowledged = true
                trajectoryPauseDialogShowing = false
                dialog.dismiss()
            }
            .setOnCancelListener {
                trajectoryPauseAcknowledged = true
                trajectoryPauseDialogShowing = false
            }
            .show()
    }

    private fun trajectoryPauseReasonText(reason: Int): String {
        return when (reason) {
            3 -> "单帧跳变过大"
            4 -> "移动速度异常过快"
            7 -> "转身后出现反向位移"
            8 -> "转身后速度异常"
            9 -> "转身后特征点偏少且跳动较大"
            11 -> "摄像头不可用期间检测到手机移动，无法可靠恢复这段距离"
            14 -> "定位连续输出异常位移，系统无法自动恢复"
            else -> "轨迹点连续异常"
        }
    }

    private fun checkCurrentPost() {
        // Get current pose
        val currentPos = DoubleArray(3)
        val currentQuat = DoubleArray(4)
        if (!vioEngine.getCurrentPose(currentPos, currentQuat)) {
            return // System not initialized
        }

        // Allocate arrays with maximum expected size (MAX_TRAJECTORY_POINTS = 10000)
        val maxSize = 10000
        val positions = DoubleArray(maxSize * 3)
        val quaternions = DoubleArray(maxSize * 4)

        val trajectorySize = vioEngine.getTrajectoryData(positions, quaternions)

        if (trajectorySize == 0) {
            // Empty trajectory or error
            trajectoryView?.updateTrajectory(
                floatArrayOf(), floatArrayOf(),
                FloatArray(3) { currentPos[it].toFloat() },
                FloatArray(4) { currentQuat[it].toFloat() })
            return
        }

        // Convert only the actual number of points to float arrays
        val posFloats = FloatArray(trajectorySize * 3)
        val quatFloats = FloatArray(trajectorySize * 4)
        for (i in 0 until trajectorySize * 3) {
            posFloats[i] = positions[i].toFloat()
        }
        for (i in 0 until trajectorySize * 4) {
            quatFloats[i] = quaternions[i].toFloat()
        }

        val currPosFloats = FloatArray(3) { currentPos[it].toFloat() }
        val currQuatFloats = FloatArray(4) { currentQuat[it].toFloat() }

        // 判断路径重访状态
        val result = trajectoryRevisitor.captureRevisited(
            posFloats,
            quatFloats,
            currentPos,
            currentQuat,
            candidateTranslations,
            candidateQuaternions,
            currentPos,
            currentQuat
        )
        tvPose?.text = String.format(
            "%d %b %b (%.3f %.3f %.3f %.3f)",
            candidateQuaternions.size,
            result.isRetrieve,
            result.isDuplicated,
            result.impact,
            result.outerImpact,
            result.absPoseErrRotFro,
            result.pointDistance,
        )
        val params = RelativeLayout.LayoutParams(
            RelativeLayout.LayoutParams.WRAP_CONTENT,
            RelativeLayout.LayoutParams.WRAP_CONTENT
        )
        params.setMargins(200, 100, 0, 0)
        tvPose?.layoutParams = params
        if (result.isRetrieve) {
            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.red))
        } else if (result.isDuplicated) {
            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.teal_700))
        } else {
            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.white))
        }
        candidateTranslations.add(currentPos)
        candidateQuaternions.add(currentQuat)

    }

    companion object {
        private const val TAG = "MainActivity"
        private const val PERMISSION_REQUEST = 1
    }
}
