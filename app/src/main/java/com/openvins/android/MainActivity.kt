package com.openvins.android

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
import java.io.FileOutputStream
import java.io.IOException
import android.os.Handler
import android.os.Looper
import androidx.core.content.ContextCompat


class MainActivity : AppCompatActivity(), CameraFrameListener, SensorEventListener {

    private var mOpenCvCameraView: Camera2ResView? = null
    private var isRecording: Boolean = false
    private var isRunningOV: Boolean = false
    private var hasRecordFolder: Boolean = false
    private var recordFolder: String = ""
    private var storageInitialized: Boolean = false  // 记录文件夹是否已初始化（避免重复初始化）

    private lateinit var sensorManager: SensorManager
    private var sensorAccel: Sensor? = null
    private var sensorGyro: Sensor? = null
    private var eventAccel: SensorEvent? = null
    private var eventGyro: SensorEvent? = null
    
    private var trajectoryView: Trajectory3DView? = null
    private val trajectoryUpdateHandler = Handler(Looper.getMainLooper())
    private val trajectoryUpdateRunnable = object : Runnable {
        override fun run() {
            updateTrajectoryView()
            // 每 100ms 更新一次（10Hz），从原来的 50ms（20Hz）降低频率以避免 ANR
            trajectoryUpdateHandler.postDelayed(this, 100)
        }
    }

    // 预分配轨迹数据数组，避免每次 updateTrajectoryView() 调用时分配新数组
    // 之前每 50ms 分配约 560KB 临时数组（DoubleArray(30000)+DoubleArray(40000)+Float 转换），
    // 产生约 11MB/s 垃圾导致频繁 GC，引发主线程 ANR
    private val maxTrajectoryPoints = 10000  // 最大轨迹点数
    private val trajectoryPositions = DoubleArray(maxTrajectoryPoints * 3)  // 预分配位置数组（x,y,z 交替存储）
    private val trajectoryQuaternions = DoubleArray(maxTrajectoryPoints * 4)  // 预分配四元数数组（x,y,z,w 交替存储）
    private val trajectoryPosFloats = FloatArray(maxTrajectoryPoints * 3)  // 位置 Float 数组（用于渲染）
    private val trajectoryQuatFloats = FloatArray(maxTrajectoryPoints * 4)  // 四元数 Float 数组（用于渲染）
    private val currentPosDouble = DoubleArray(3)  // 当前位置预分配数组
    private val currentQuatDouble = DoubleArray(4)  // 当前四元数预分配数组

    init {
        // Load native library
        System.loadLibrary("native-lib")
    }

    public override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Setup our camera
        setContentView(R.layout.activity_main)
        mOpenCvCameraView = findViewById<View>(R.id.test_view) as Camera2ResView
        mOpenCvCameraView!!.setCameraFrameListener(this)
        
        // Setup trajectory view
        trajectoryView = findViewById<Trajectory3DView>(R.id.trajectory_view)
        // Force initial render to show axes and grid
        trajectoryView?.forceRender()

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
                "ERROR: unable to open sensorGyro", Toast.LENGTH_LONG
            ).show()
        }

        // 从 APK assets 复制配置文件到内部存储（无需任何权限）
        // 使用内部存储（getFilesDir）而非外部存储（getExternalFilesDir），
        // 因为 Android 11+ 的 Scoped Storage 限制可能导致外部存储 EACCES 错误
        val configFolderRoot = filesDir.absolutePath  // 内部存储根目录
        val configDir = configFolderRoot + "/config/"  // 配置文件子目录
        File(configDir).mkdirs()  // 确保目录存在
        copyConfigAssetsIfNeeded(configDir)  // 首次运行时从 assets 复制配置文件
        // 将私有目录路径传递给 C++ 层（C++ 会拼接 /config/ 子目录）
        setAppPrivateFolderJNI(configFolderRoot)

        // 检查是否已拥有存储权限（例如应用重启后权限仍保留）
        if (hasStoragePermissions()) {
            initializeRecordFolder()  // 已有权限，直接初始化记录文件夹
        }

        // 请求 Android 6+ 运行时权限（相机 + 存储）
        ActivityCompat.requestPermissions(
            this@MainActivity,
            arrayOf(
                Manifest.permission.CAMERA,
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
            ),
            PERMISSION_REQUEST
        )

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
                        setAppRecordFolderJNI(recordFolder)
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
            setRecordStateJNI(isRecording)
        }

        // Our start / stop openvins button
        val reset = findViewById(R.id.toggle_reset) as FloatingActionButton
        reset.setOnClickListener {
            isRunningOV = if (isRunningOV) {
                reset.setImageResource(R.drawable.ic_baseline_play_arrow_24)
                false
            } else {
                reset.setImageResource(R.drawable.ic_stop_system)
                // Clear trajectory view when starting (in case there was leftover data)
                trajectoryView?.clearTrajectory()
                true
            }
            toggleSystemJNI(isRunningOV)
        }

    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        when (requestCode) {
            PERMISSION_REQUEST -> {
                val cameraGranted = grantResults.isNotEmpty() &&
                    grantResults.getOrNull(0) == PackageManager.PERMISSION_GRANTED
                val storageGranted = grantResults.size > 1 &&
                    grantResults.getOrNull(1) == PackageManager.PERMISSION_GRANTED &&
                    grantResults.getOrNull(2) == PackageManager.PERMISSION_GRANTED

                // 相机权限：启动相机预览
                if (cameraGranted) {
                    mOpenCvCameraView!!.setCameraPermissionGranted()
                } else {
                    Log.e(TAG, "Camera permission was not granted")
                    Toast.makeText(this, "Camera permission was not granted", Toast.LENGTH_LONG)
                        .show()
                }

                // 存储权限：初始化记录文件夹（仅首次授权时执行）
                if (storageGranted && !storageInitialized) {
                    initializeRecordFolder()
                } else if (!storageGranted) {
                    Log.w(TAG, "Storage permission was not granted, recording may not work")
                }
            }
            else -> {
                Log.e(TAG, "Unexpected permission request")
            }
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

    public override fun onDestroy() {
        super.onDestroy()
        
        // Stop trajectory updates
        trajectoryUpdateHandler.removeCallbacks(trajectoryUpdateRunnable)
        
        // Unregister sensor listeners
        sensorManager.unregisterListener(this)
        
        // Stop OpenVINS system and clean up native resources
        if (isRunningOV) {
            toggleSystemJNI(false)
        }
        
        // Stop recording if active
        if (isRecording) {
            setRecordStateJNI(false)
        }
        
        // Disable camera view
        if (mOpenCvCameraView != null) {
            mOpenCvCameraView!!.disableView()
        }
    }

    override fun onFrame(matAddr: Long, timestampSec: Double) {
        // Native function processes the frame and updates it in-place
        // Mat address is already from native code, so we can use it directly
        processImageJNI(matAddr, timestampSec)
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
            processInertialJNI(
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

    private external fun setAppRecordFolderJNI(dir: String)
    private external fun setAppPrivateFolderJNI(dir: String)
    private external fun setRecordStateJNI(state: Boolean)
    private external fun toggleSystemJNI(state: Boolean)
    private external fun processImageJNI(matAddr: Long, timestampSec: Double)
    private external fun processInertialJNI(
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float,
        timestampSec: Double
    )
    private external fun getCurrentPoseJNI(position: DoubleArray, quaternion: DoubleArray): Boolean
    // Combined function: returns number of points copied (0 on error/empty)
    // Arrays must be pre-allocated with sufficient size (max_size * 3 for positions, max_size * 4 for quaternions)
    private external fun getTrajectoryDataJNI(positions: DoubleArray, quaternions: DoubleArray): Int
    
    private fun updateTrajectoryView() {
        if (trajectoryView == null) return
        if (!isRunningOV) return  // VIO 系统未运行时跳过更新，减少无效计算

        // 获取当前位姿（复用预分配数组，避免每次分配新数组）
        if (!getCurrentPoseJNI(currentPosDouble, currentQuatDouble)) {
            return // 系统尚未初始化
        }

        // 获取轨迹数据（复用预分配数组，native 函数返回实际点数）
        val trajectorySize = getTrajectoryDataJNI(trajectoryPositions, trajectoryQuaternions)

        // 将当前位置转换为 Float 数组用于渲染
        val currPosFloats = FloatArray(3) { currentPosDouble[it].toFloat() }
        val currQuatFloats = FloatArray(4) { currentQuatDouble[it].toFloat() }

        if (trajectorySize == 0) {
            trajectoryView?.updateTrajectory(floatArrayOf(), floatArrayOf(),
                currPosFloats, currQuatFloats)
            return
        }

        // 仅转换实际点数的 Double→Float（复用预分配数组）
        val posCount = trajectorySize * 3
        val quatCount = trajectorySize * 4
        for (i in 0 until posCount) {
            trajectoryPosFloats[i] = trajectoryPositions[i].toFloat()
        }
        for (i in 0 until quatCount) {
            trajectoryQuatFloats[i] = trajectoryQuaternions[i].toFloat()
        }

        // 更新 3D 视图 - 只复制预分配数组中的有效部分
        trajectoryView?.updateTrajectory(
            trajectoryPosFloats.copyOfRange(0, posCount),
            trajectoryQuatFloats.copyOfRange(0, quatCount),
            currPosFloats, currQuatFloats
        )
    }

    companion object {
        private const val TAG = "MainActivity"
        private const val PERMISSION_REQUEST = 1
    }

    /**
     * 检查是否已获取存储读写权限。
     */
    private fun hasStoragePermissions(): Boolean {
        val writePermission = ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
        val readPermission = ContextCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
        return writePermission == PackageManager.PERMISSION_GRANTED &&
               readPermission == PackageManager.PERMISSION_GRANTED
    }

    /**
     * 初始化记录文件夹（公共外部存储的 Documents 目录）。
     * 在 Android 11 以下设备需要 WRITE_EXTERNAL_STORAGE 权限。
     * 配置文件已单独在 onCreate() 中使用内部存储处理（无需权限）。
     */
    private fun initializeRecordFolder() {
        if (storageInitialized) return  // 防止重复初始化

        // 使用公共 Documents 目录存放记录数据（用户可访问）
        val appRecordFolder =
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                .toString() + "/openvins/"

        // 设置记录文件夹路径
        recordFolder = appRecordFolder

        // 创建记录文件夹
        val file = File(recordFolder)
        if ((!file.isDirectory && !file.mkdirs()) || file.isFile) {
            Toast.makeText(
                applicationContext,
                "ERROR: unable to create directory. ${file.toString()}",
                Toast.LENGTH_LONG
            ).show()
        } else {
            hasRecordFolder = true
            // 将记录目录路径传递给 C++ 层
            setAppRecordFolderJNI(recordFolder)
        }

        storageInitialized = true  // 标记已初始化
        Log.i(TAG, "Record folder initialized: $recordFolder")
    }

    /**
     * 将 APK assets 中的配置 YAML 文件复制到私有存储目录。
     * 仅复制磁盘上不存在的文件，因此用户通过 sync_device.sh 推送的
     * 自定义配置不会被覆盖。
     */
    private fun copyConfigAssetsIfNeeded(configDir: String) {
        // 需要从 assets 复制的配置文件列表
        val configAssets = listOf("estimator_config.yaml", "kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml")
        var copied = 0
        for (assetName in configAssets) {
            val targetFile = File(configDir, assetName)
            if (targetFile.exists()) {
                // 文件已存在，跳过（保留用户可能的自定义修改）
                Log.d(TAG, "Config file already exists: $assetName")
                continue
            }
            try {
                // 从 APK assets 中读取并写入到内部存储
                assets.open("config/$assetName").use { input ->
                    FileOutputStream(targetFile).use { output ->
                        input.copyTo(output)
                    }
                }
                Log.i(TAG, "Copied config asset to: $targetFile")
                copied++
            } catch (e: IOException) {
                // 复制失败不影响应用启动，后续 YamlParser 会报告具体问题
                Log.e(TAG, "Failed to copy config asset: $assetName", e)
            }
        }
        if (copied > 0) {
            Log.i(TAG, "Copied $copied config file(s) from assets to $configDir")
        }
    }

    init {
        Log.i(TAG, "Instantiated new " + this.javaClass)
    }
}