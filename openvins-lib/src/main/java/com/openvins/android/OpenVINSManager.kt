package com.openvins.android

import android.content.Context
import android.util.Log
import java.io.File
import java.io.FileOutputStream
import java.io.IOException

/**
 * OpenVINS 管理器 —— 库的公开 API 入口。
 *
 * 封装了 VIO 系统的启动/停止、数据录制、配置管理、轨迹获取等核心功能，
 * 使调用方无需直接与 JNI 交互。
 *
 * 使用流程：
 * 1. 调用 [initConfig] 初始化配置文件
 * 2. 设置 [onInitializationListener] 监听初始化状态
 * 3. 调用 [setRecordFolder] 设置录制目录
 * 4. 调用 [toggleSystem] 启动/停止 VIO
 * 5. 调用 [getCurrentPose] / [getTrajectoryData] 获取位姿和轨迹
 * 6. 初始化成功后会自动回调 [OnInitializationListener.onVioInitialized]
 */
class OpenVINSManager {

    /**
     * VIO 系统初始化完成回调接口。
     * 当系统从 "init" 状态变为 "CAM" 状态时触发（即 sys->initialized() 为 true）。
     */
    interface OnInitializationListener {
        /**
         * VIO 系统初始化完成回调，在主线程调用。
         * 回调后 [getCurrentPose] 和 [getTrajectoryData] 返回的数据才有效。
         */
        fun onVioInitialized()
    }

    /** 初始化状态监听器 */
    var onInitializationListener: OnInitializationListener? = null

    /**
     * C++ 层回调：VIO 系统初始化完成时由 native 代码调用。
     * 通过 Handler 切换到主线程后通知监听器。
     */
    private fun onVioInitializedFromNative() {
        Log.i(TAG, "VIO system initialized (callback from native)")
        onInitializationListener?.onVioInitialized()
    }

    companion object {
        private const val TAG = "OpenVINSManager"

        init {
            // 加载 native 库
            System.loadLibrary("native-lib")
        }
    }

    // ==================== 配置管理 ====================

    /**
     * 初始化配置文件：将 APK assets 中的 YAML 配置复制到内部存储。
     * 应在 Application.onCreate() 或 Activity.onCreate() 最早期调用。
     *
     * @param context 应用上下文
     * @return 配置文件根目录路径
     */
    fun initConfig(context: Context): String {
        // 使用内部存储（无需权限，Android 11+ Scoped Storage 兼容）
        val configFolderRoot = context.filesDir.absolutePath
        val configDir = configFolderRoot + "/config/"
        File(configDir).mkdirs()
        copyConfigAssetsIfNeeded(context, configDir)
        // 将私有目录路径传递给 C++ 层（C++ 会拼接 /config/ 子目录）
        setAppPrivateFolderJNI(configFolderRoot)
        return configFolderRoot
    }

    /**
     * 将 APK assets 中的配置 YAML 文件复制到私有存储目录。
     * 仅复制磁盘上不存在的文件，不会覆盖用户自定义配置。
     */
    private fun copyConfigAssetsIfNeeded(context: Context, configDir: String) {
        val configAssets = listOf(
            "estimator_config.yaml",
            "kalibr_imu_chain.yaml",
            "kalibr_imucam_chain.yaml"
        )
        var copied = 0
        for (assetName in configAssets) {
            val targetFile = File(configDir, assetName)
            if (targetFile.exists()) {
                Log.d(TAG, "Config file already exists: $assetName")
                continue
            }
            try {
                context.assets.open("config/$assetName").use { input ->
                    FileOutputStream(targetFile).use { output ->
                        input.copyTo(output)
                    }
                }
                Log.i(TAG, "Copied config asset to: $targetFile")
                copied++
            } catch (e: IOException) {
                Log.e(TAG, "Failed to copy config asset: $assetName", e)
            }
        }
        if (copied > 0) {
            Log.i(TAG, "Copied $copied config file(s) from assets to $configDir")
        }
    }

    // ==================== 系统控制 ====================

    /**
     * 启动或停止 VIO 系统。
     *
     * @param running true 启动，false 停止
     */
    fun toggleSystem(running: Boolean) {
        toggleSystemJNI(running)
        // 启动时注册 C++ 回调，以便初始化完成后通知 Java 层
        if (running) {
            registerInitCallbackJNI()
        }
    }

    /**
     * 检查 VIO 系统是否已完成初始化（即预览画面从 "init" 变为 "CAM"）。
     * 初始化完成后，位姿和轨迹数据才有效。
     *
     * @return true 已初始化，false 未初始化
     */
    fun isInitialized(): Boolean {
        return isSystemInitializedJNI()
    }

    // ==================== 录制管理 ====================

    /**
     * 设置录制数据存放目录。
     *
     * @param dir 目录路径
     */
    fun setRecordFolder(dir: String) {
        setAppRecordFolderJNI(dir)
    }

    /**
     * 开始或停止数据录制。
     *
     * @param recording true 开始录制，false 停止录制
     */
    fun setRecording(recording: Boolean) {
        setRecordStateJNI(recording)
    }

    // ==================== 帧和 IMU 数据输入 ====================

    /**
     * 处理相机帧数据（由 Camera2ResView 的 CameraFrameListener 回调调用）。
     *
     * @param matAddr OpenCV Mat 的 native 地址
     * @param timestampSec 帧时间戳（秒，基于 CLOCK_BOOTTIME）
     */
    fun processImage(matAddr: Long, timestampSec: Double) {
        processImageJNI(matAddr, timestampSec)
    }

    /**
     * 处理 IMU 惯性数据。
     *
     * @param ax,ay,az 加速度计三轴数据
     * @param gx,gy,gz 陀螺仪三轴数据
     * @param timestampSec IMU 时间戳（秒，基于 CLOCK_BOOTTIME）
     */
    fun processInertial(ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float, timestampSec: Double) {
        processInertialJNI(ax, ay, az, gx, gy, gz, timestampSec)
    }

    // ==================== 位姿和轨迹 ====================

    /**
     * 快照结果数据类。
     *
     * @property imagePath 保存的图片文件绝对路径，失败时为 null
     * @property position 相机坐标系下的位置 [x, y, z]
     * @property quaternion 相机坐标系下的姿态 [qw, qx, qy, qz]（Hamilton 约定）
     */
    data class SnapshotResult(
        val imagePath: String?,
        val position: DoubleArray,
        val quaternion: DoubleArray
    ) {
        override fun equals(other: Any?) = this === other
        override fun hashCode() = System.identityHashCode(this)
    }

    /**
     * 拍摄快照：保存当前预览画面为图片文件，同时获取当前位姿。
     *
     * 图片上会叠加当前位姿信息（位置和四元数），保存为 JPEG 格式，
     * 文件名为 snapshot_&lt;timestamp_ns&gt;.jpg。
     *
     * @param saveDir 图片保存目录
     * @return [SnapshotResult] 包含图片路径和位姿数据；imagePath 为 null 表示保存失败
     */
    fun takeSnapshot(saveDir: String): SnapshotResult {
        val position = DoubleArray(3)
        val quaternion = DoubleArray(4)
        val path = takeSnapshotJNI(saveDir, position, quaternion)
        return SnapshotResult(path, position, quaternion)
    }

    /**
     * 获取当前位姿（相机坐标系）。
     *
     * @param position 长度为 3 的 DoubleArray，输出 [x, y, z]
     * @param quaternion 长度为 4 的 DoubleArray，输出 [qw, qx, qy, qz]（Hamilton 约定）
     * @return true 获取成功，false 系统未运行或未初始化
     */
    fun getCurrentPose(position: DoubleArray, quaternion: DoubleArray): Boolean {
        return getCurrentPoseJNI(position, quaternion)
    }

    /**
     * 获取历史轨迹数据。
     *
     * @param positions 预分配的 DoubleArray（大小 >= maxPoints * 3）
     * @param quaternions 预分配的 DoubleArray（大小 >= maxPoints * 4）
     * @return 实际轨迹点数，0 表示无数据或出错
     */
    fun getTrajectoryData(positions: DoubleArray, quaternions: DoubleArray): Int {
        return getTrajectoryDataJNI(positions, quaternions)
    }

    // ==================== JNI Native 方法 ====================

    private external fun processImageJNI(matAddr: Long, timestampSec: Double)
    private external fun processInertialJNI(
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float,
        timestampSec: Double
    )
    private external fun setAppRecordFolderJNI(dir: String)
    private external fun setAppPrivateFolderJNI(dir: String)
    private external fun setRecordStateJNI(state: Boolean)
    private external fun toggleSystemJNI(state: Boolean)
    private external fun isSystemInitializedJNI(): Boolean
    private external fun registerInitCallbackJNI()
    private external fun getCurrentPoseJNI(position: DoubleArray, quaternion: DoubleArray): Boolean
    private external fun getTrajectoryDataJNI(positions: DoubleArray, quaternions: DoubleArray): Int
    private external fun takeSnapshotJNI(saveDir: String, position: DoubleArray, quaternion: DoubleArray): String?
}
