package com.openvins.app

import com.openvins.android.Camera2ResView
import com.openvins.android.CameraFrameListener
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
    private var eventAccel: SensorEvent? = null
    private var eventGyro: SensorEvent? = null

    private val trajectoryRevisitor: TrajectoryRevisitor = TrajectoryRevisitor()

    private var trajectoryView: Trajectory3DView? = null
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
        trajectoryUpdateHandler.removeCallbacks(trajectoryUpdateRunnable)
    }

    public override fun onResume() {
        super.onResume()
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

        // First check if we have any new events
        when (event?.sensor?.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                //Log.e(TAG, "[acc]: ${event.values[0]}, ${event.values[1]}, ${event.values[2]}")
                eventAccel = event
            }

            Sensor.TYPE_GYROSCOPE -> {
                //Log.e(TAG, "[gyro]: ${event.values[0]}, ${event.values[1]}, ${event.values[2]}")
                eventGyro = event
            }
        }

        // Next wait till we have both gyroscope and accelerometer
        // TODO: we should try to be smarter about this selection as they could be
        // TODO: out of sync and we should never know this...
        if (eventAccel != null && eventGyro != null) {
            // Use the sensor event timestamp (nanoseconds since boot, converted to seconds)
            // This ensures consistent timing with camera timestamps which also use boot time reference
            val timestampSec = (event!!.timestamp * 1e-9).toDouble()
            vioEngine.processImu(
                eventAccel!!.values[0], eventAccel!!.values[1], eventAccel!!.values[2],
                eventGyro!!.values[0], eventGyro!!.values[1], eventGyro!!.values[2],
                timestampSec
            );
            eventAccel = null;
            eventGyro = null;
        }

    }

    override fun onAccuracyChanged(p0: Sensor?, p1: Int) {
        Log.d(TAG, "[sensor]: accuracy level of ${p0.toString()} changed to $p1")
    }

    private fun updateTrajectoryView() {
        if (trajectoryView == null) return

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
            "Pose: currPosFloats: %f %f %f  isRevisit %b",
            currPosFloats[0],
            currPosFloats[1],
            currPosFloats[2],
            result.isRevisit
        )
        if (result.isRevisit) {
            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.red))
        } else {
            tvPose?.setTextColor(ContextCompat.getColor(this, R.color.white))
            candidateTranslations.add(currentPos)
            candidateQuaternions.add(currentQuat)
        }

    }

    companion object {
        private const val TAG = "MainActivity"
        private const val PERMISSION_REQUEST = 1
    }
}