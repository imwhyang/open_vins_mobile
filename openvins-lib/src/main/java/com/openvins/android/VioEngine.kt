package com.openvins.android

import android.content.Context
import android.util.Log
import java.io.File
import java.io.FileOutputStream

/**
 * OpenVINS 视觉惯性导航引擎的统一公开 API。
 * 封装所有 JNI 原生函数调用，供外部使用者集成。
 */
class VioEngine {

    companion object {
        private const val TAG = "VioEngine"

        init {
            System.loadLibrary("native-lib")
        }
    }

    /**
     * 将 assets/config/ 中的配置文件解压到应用私有目录，仅当目标文件不存在时执行。
     * 应在 setPrivateFolder() 之前调用。
     * @param context 应用 Context，用于访问 assets 和私有存储
     * @return 配置文件所在目录路径（即 externalFilesDir/config/）
     */
    fun copyConfigIfNeeded(context: Context): String {
        val configDir = File(context.filesDir, "config")
        if (!configDir.exists()) configDir.mkdirs()

        val assetFiles = listOf(
            "estimator_config.yaml",
            "kalibr_imu_chain.yaml",
            "kalibr_imucam_chain.yaml"
        )
        for (fileName in assetFiles) {
            val dest = File(configDir, fileName)
            try {
                context.assets.open("config/$fileName").use { input ->
                    val assetBytes = input.readBytes()
                    val shouldWrite = !dest.exists() || !dest.readBytes().contentEquals(assetBytes)
                    if (shouldWrite) {
                        FileOutputStream(dest).use { output ->
                            output.write(assetBytes)
                        }
                        Log.i(TAG, "已同步 assets 配置文件：$fileName")
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "同步配置文件失败：$fileName", e)
            }
        }
        return configDir.parent ?: ""
    }

    /**
     * 设置录制文件的公共保存目录（用户可访问的路径）。
     * @param dir 录制文件保存的目录路径
     */
    fun setRecordFolder(dir: String) {
        setAppRecordFolderJNI(dir)
    }

    /**
     * 设置应用私有目录（用于存放配置文件）。
     * 原生代码会在该目录下查找 /config/ 子目录。
     * @param dir 应用私有文件根目录路径
     */
    fun setPrivateFolder(dir: String) {
        setAppPrivateFolderJNI(dir)
    }

    /**
     * 设置录制状态。
     * @param recording true 开始录制，false 停止录制
     */
    fun setRecording(recording: Boolean) {
        setRecordStateJNI(recording)
    }

    /**
     * 配置是否生成轨迹和相机质量调试日志，默认关闭。
     * 请在开始录制前设置；关闭时仅保留 pose0.csv。
     */
    fun setDebugLoggingEnabled(enabled: Boolean) {
        setDebugLoggingEnabledJNI(enabled)
    }

    /**
     * 启动或停止 OpenVINS 系统。
     * @param running true 启动系统，false 停止系统
     */
    fun toggleSystem(running: Boolean) {
        toggleSystemJNI(running)
    }

    /**
     * 处理一帧相机图像。
     * @param matAddr OpenCV Mat 对象的内存地址
     * @param timestampSec 帧的时间戳（秒，boot time 参考系）
     */
    fun processImage(matAddr: Long, timestampSec: Double) {
        processImageJNI(matAddr, timestampSec)
    }

    /**
     * 处理一组 IMU 惯性测量数据。
     * @param ax 加速度计 X 轴读数
     * @param ay 加速度计 Y 轴读数
     * @param az 加速度计 Z 轴读数
     * @param gx 陀螺仪 X 轴读数
     * @param gy 陀螺仪 Y 轴读数
     * @param gz 陀螺仪 Z 轴读数
     * @param timestampSec 时间戳（秒，boot time 参考系）
     * @param pairDeltaSec 加速度计与陀螺仪原始时间戳差（秒）
     */
    fun processImu(
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float,
        timestampSec: Double,
        pairDeltaSec: Double = 0.0,
    ) {
        processInertialJNI(ax, ay, az, gx, gy, gz, timestampSec, pairDeltaSec)
    }

    /**
     * 获取当前位姿估计。
     * @param position 长度为 3 的数组，返回 [x, y, z] 位置
     * @param quaternion 长度为 4 的数组，返回 [qw, qx, qy, qz] 四元数
     * @return true 表示系统已初始化且位姿有效，false 表示未初始化
     */
    fun getCurrentPose(position: DoubleArray, quaternion: DoubleArray): Boolean {
        return getCurrentPoseJNI(position, quaternion)
    }

    /**
     * 获取轨迹历史数据。
     * 调用前需预先分配足够大的数组（建议 max_size = 10000）。
     * @param positions 预分配的长度为 max_size*3 的数组，存储 [x,y,z,...]
     * @param quaternions 预分配的长度为 max_size*4 的数组，存储 [qw,qx,qy,qz,...]
     * @return 实际复制的轨迹点数量，0 表示无数据或出错
     */
    fun getTrajectoryData(positions: DoubleArray, quaternions: DoubleArray): Int {
        return getTrajectoryDataJNI(positions, quaternions)
    }

    fun isTrajectoryPaused(): Boolean {
        return isTrajectoryPausedJNI()
    }

    /** 获取定位恢复状态：0正常、1遮挡帧过滤、2轨迹对齐、3后台初始化。 */
    fun getVisualRecoveryState(): Int {
        return getVisualRecoveryStateJNI()
    }

    /** 获取结构化定位状态，具体状态码由 SDK 层转换为 OpenVinsTrackingState。 */
    fun getTrackingState(): Int {
        return getTrackingStateJNI()
    }

    /** 是否在相机画面显示原生 init/zvupt 调试文字。 */
    fun setStatusOverlayEnabled(enabled: Boolean) {
        setStatusOverlayEnabledJNI(enabled)
    }

    fun getTrajectoryPauseReason(): Int {
        return getTrajectoryPauseReasonJNI()
    }

    fun resumeTrajectory() {
        resumeTrajectoryJNI()
    }

    /**
     * 将 YUV_420_888 格式的图像数据转换为 RGBA Mat。
     * @return 新分配的 Mat 对象内存地址，0 表示失败
     */
    fun processYUVToRGBA(
        yData: ByteArray?, uData: ByteArray?, vData: ByteArray?,
        width: Int, height: Int,
        yStride: Int, uStride: Int, vStride: Int,
        chromaPixelStride: Int
    ): Long {
        return processYUVToRGBAJNI(yData, uData, vData, width, height, yStride, uStride, vStride, chromaPixelStride)
    }

    /**
     * 获取用于显示的图像（带叠加层或原始相机帧）。
     * @param rawMatAddr 原始相机 Mat 地址
     * @return 新分配的显示用 Mat 地址，0 表示失败
     */
    fun getDisplayImage(rawMatAddr: Long): Long {
        return getDisplayImageJNI(rawMatAddr)
    }

    /**
     * 释放原生 Mat 对象内存。
     * @param matAddr 需要释放的 Mat 地址
     */
    fun deleteMat(matAddr: Long) {
        deleteMatJNI(matAddr)
    }

    // ========================
    // JNI 原生函数声明
    // ========================
    private external fun setAppRecordFolderJNI(dir: String)
    private external fun setAppPrivateFolderJNI(dir: String)
    private external fun setRecordStateJNI(state: Boolean)
    private external fun setDebugLoggingEnabledJNI(enabled: Boolean)
    private external fun toggleSystemJNI(state: Boolean)
    private external fun processImageJNI(matAddr: Long, timestampSec: Double)
    private external fun processInertialJNI(
        ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float,
        timestampSec: Double,
        pairDeltaSec: Double,
    )
    private external fun getCurrentPoseJNI(position: DoubleArray, quaternion: DoubleArray): Boolean
    private external fun getTrajectoryDataJNI(positions: DoubleArray, quaternions: DoubleArray): Int
    private external fun isTrajectoryPausedJNI(): Boolean
    private external fun getVisualRecoveryStateJNI(): Int
    private external fun getTrackingStateJNI(): Int
    private external fun setStatusOverlayEnabledJNI(enabled: Boolean)
    private external fun getTrajectoryPauseReasonJNI(): Int
    private external fun resumeTrajectoryJNI()
    private external fun processYUVToRGBAJNI(
        yData: ByteArray?, uData: ByteArray?, vData: ByteArray?,
        width: Int, height: Int,
        yStride: Int, uStride: Int, vStride: Int,
        chromaPixelStride: Int
    ): Long
    private external fun getDisplayImageJNI(rawCameraMatAddr: Long): Long
    private external fun deleteMatJNI(matAddr: Long)
}
