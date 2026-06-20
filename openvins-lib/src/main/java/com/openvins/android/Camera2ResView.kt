package com.openvins.android

import android.annotation.TargetApi
import android.content.Context
import android.graphics.ImageFormat
import android.graphics.YuvImage
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.params.StreamConfigurationMap
import android.media.Image
import android.media.ImageReader
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.media.MediaMuxer
import android.os.Handler
import android.os.HandlerThread
import android.util.AttributeSet
import android.util.Log
import android.util.Size
import android.graphics.Bitmap
import android.view.SurfaceHolder
import android.view.SurfaceView
import org.opencv.android.Utils
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer

interface CameraFrameListener {
    fun onFrame(matAddr: Long, timestampSec: Double)
}

@TargetApi(21)
class Camera2ResView(context: Context?, attrs: AttributeSet?) : SurfaceView(context, attrs), SurfaceHolder.Callback {

    private val TAG = "Camera2ResView"

    // Store the last frame timestamp (synchronized access)
    @Volatile
    private var lastFrameTimestampSec: Double = 0.0

    // Getter for last frame timestamp
    fun getLastFrameTimestamp(): Double = lastFrameTimestampSec

    private var mImageReader: ImageReader? = null
    private var mCameraDevice: CameraDevice? = null
    private var mCaptureSession: CameraCaptureSession? = null
    private var mPreviewRequestBuilder: CaptureRequest.Builder? = null
    private var mCameraID: String? = null
    private var mPreviewSize: Size = Size(-1, -1)
    private var mBackgroundThread: HandlerThread? = null
    private var mBackgroundHandler: Handler? = null
    private val mPreviewFormat = ImageFormat.YUV_420_888

    private var mFrameListener: CameraFrameListener? = null
    private var mEnabled = false
    // 相机是否活跃，退出时先置 false 让 onImageAvailable 丢弃后续帧
    @Volatile
    private var mIsCameraActive = false
    private var mCameraPermissionGranted = false

    // ---- 视频录制相关 ----
    private var mMediaCodec: MediaCodec? = null
    private var mMediaMuxer: MediaMuxer? = null
    @Volatile
    private var mIsRecording = false
    private var mMuxerStarted = false
    private var mVideoTrackIndex = -1
    private var mRecordingStartTimeNs: Long = -1L  // -1 表示尚未收到首帧，首帧 timestamp 作为基准
    private var mRecordingFilePath: String? = null
    private var mEncodedFrameCount: Int = 0  // 录制期间编码的帧数
    // 录制锁，保护编码器并发访问
    private val mRecordingLock = Object()
    
    // Temporary display bitmap
    private var mDisplayBitmap: Bitmap? = null
    private val mDisplayLock = Object()

    // ---- 原始帧截图相关 ----
    @Volatile
    private var mPendingCapturePath: String? = null
    @Volatile
    private var mPendingCaptureQuality: Int = 90
    private var mCaptureCallbackHandler: Handler? = null
    private var mCaptureCallback: ((File?) -> Unit)? = null

    init {
        holder.addCallback(this)
        holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS)
    }

    fun setCameraFrameListener(listener: CameraFrameListener?) {
        mFrameListener = listener
    }

    fun setCameraPermissionGranted() {
        mCameraPermissionGranted = true
        checkState()
    }

    fun enableView() {
        mEnabled = true
        checkState()
    }

    fun disableView() {
        mEnabled = false
        checkState()
    }

    private fun checkState() {
        if (mEnabled && mCameraPermissionGranted && holder.surface != null && visibility == VISIBLE) {
            if (mCameraDevice == null) {
                connectCamera()
            }
        } else {
            if (mCameraDevice != null) {
                disconnectCamera()
            }
        }
    }

    private fun startBackgroundThread() {
        stopBackgroundThread()
        mBackgroundThread = HandlerThread("CameraBackground")
        mBackgroundThread!!.start()
        mBackgroundHandler = Handler(mBackgroundThread!!.looper)
    }

    private fun stopBackgroundThread() {
        if (mBackgroundThread != null) {
            mBackgroundThread!!.quitSafely()
            try {
                mBackgroundThread!!.join()
                mBackgroundThread = null
                mBackgroundHandler = null
            } catch (e: InterruptedException) {
                Log.e(TAG, "stopBackgroundThread", e)
            }
        }
    }

    private fun connectCamera() {
        Log.i(TAG, "connectCamera")
        startBackgroundThread()
        initializeCamera()
    }

    private fun disconnectCamera() {
        Log.i(TAG, "disconnectCamera")
        // 标记相机正在关闭，让 onImageAvailable 回调丢弃后续帧
        mIsCameraActive = false
        // 断开相机前，若正在录制则自动停止
        if (mIsRecording) {
            Log.i(TAG, "Auto-stopping recording due to camera disconnect")
            stopRecording()
        }
        val handler = mBackgroundHandler
        
        // Close capture session first
        if (mCaptureSession != null) {
            try {
                mCaptureSession!!.close()
            } catch (e: Exception) {
                Log.e(TAG, "Error closing capture session", e)
            }
            mCaptureSession = null
        }
        
        // Close ImageReader
        if (mImageReader != null) {
            try {
                mImageReader!!.close()
            } catch (e: Exception) {
                Log.e(TAG, "Error closing ImageReader", e)
            }
            mImageReader = null
        }
        
        // Close camera device - close directly to avoid handler issues
        // The Camera2 framework will call callbacks, but we handle them synchronously
        val camera = mCameraDevice
        synchronized(this) {
            mCameraDevice = null
        }
        
        if (camera != null) {
            try {
                // Close directly - don't use handler to avoid dead thread issues
                // Any callbacks will be handled synchronously in the state callback
                camera.close()
                // Give a moment for any callbacks to process
                Thread.sleep(50)
            } catch (e: Exception) {
                Log.e(TAG, "Error closing camera", e)
            }
        }
        
        // Now safe to stop background thread
        stopBackgroundThread()
    }

    private fun initializeCamera() {
        val manager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        try {
            val camList = manager.cameraIdList
            if (camList.isEmpty()) {
                Log.e(TAG, "Error: camera isn't detected.")
                return
            }
            // Use back camera by default
            mCameraID = null
            for (cameraID in camList) {
                val characteristics = manager.getCameraCharacteristics(cameraID)
                if (characteristics.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK) {
                    mCameraID = cameraID
                    break
                }
            }
            if (mCameraID == null) {
                mCameraID = camList[0]
            }
            
            Log.i(TAG, "Opening camera: $mCameraID")
            manager.openCamera(mCameraID!!, mStateCallback, mBackgroundHandler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "CameraAccessException", e)
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException", e)
        }
    }

    private val mStateCallback = object : CameraDevice.StateCallback() {
        override fun onOpened(camera: CameraDevice) {
            val handler = mBackgroundHandler
            if (handler != null) {
                try {
                    handler.post {
                        if (mCameraDevice == null) { // Only if not already closed
                            mCameraDevice = camera
                            createCameraPreviewSession()
                        }
                    }
                } catch (e: IllegalStateException) {
                    // Handler thread is dead, handle synchronously
                    Log.w(TAG, "Handler thread dead, handling onOpened synchronously")
                    if (mCameraDevice == null) {
                        mCameraDevice = camera
                        createCameraPreviewSession()
                    }
                }
            }
        }

        override fun onDisconnected(camera: CameraDevice) {
            Log.w(TAG, "Camera disconnected")
            // Handle synchronously to avoid dead thread issues
            synchronized(this@Camera2ResView) {
                if (mCameraDevice == camera) {
                    mCameraDevice = null
                }
            }
        }

        override fun onError(camera: CameraDevice, error: Int) {
            Log.e(TAG, "Camera error: $error")
            // Handle synchronously to avoid dead thread issues
            // Don't try to close here as it may cause more callbacks
            synchronized(this@Camera2ResView) {
                if (mCameraDevice == camera) {
                    mCameraDevice = null
                }
            }
        }
    }

    private fun createCameraPreviewSession() {
        val width = width
        val height = height
        
        if (width <= 0 || height <= 0) {
            return
        }
        
        calcPreviewSize(width, height)
        
        val w = mPreviewSize.width
        val h = mPreviewSize.height
        
        Log.d(TAG, "createCameraPreviewSession($w x $h)")
        if (w < 0 || h < 0)
            return
        try {
            if (mCameraDevice == null) {
                Log.e(TAG, "createCameraPreviewSession: camera isn't opened")
                return
            }
            if (mCaptureSession != null) {
                Log.e(TAG, "createCameraPreviewSession: mCaptureSession is already started")
                return
            }

            mIsCameraActive = true
            mImageReader = ImageReader.newInstance(w, h, mPreviewFormat, 2)
            mImageReader!!.setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage()
                if (image == null)
                    return@setOnImageAvailableListener

                // 相机正在关闭时，丢弃帧避免访问已关闭的 Image
                if (!mIsCameraActive) {
                    image.close()
                    return@setOnImageAvailableListener
                }

                try {
                // Get hardware timestamp from the Image (nanoseconds since boot)
                val frameTimestampNs = image.timestamp
                lastFrameTimestampSec = frameTimestampNs * 1e-9

                // Extract YUV plane data directly from Image
                val planes = image.planes
                val imgWidth = image.width
                val imgHeight = image.height
                
                val yPlane = planes[0].buffer
                val yStride = planes[0].rowStride
                val uPlane = planes[1].buffer
                val uStride = planes[1].rowStride
                val uPixelStride = planes[1].pixelStride
                val vPlane = if (planes.size > 2) planes[2].buffer else null
                val vStride = if (vPlane != null) planes[2].rowStride else 0
                
                // Extract raw byte data - use buffer's actual capacity
                yPlane.rewind()
                val ySize = yPlane.remaining()
                val yBytes = ByteArray(ySize)
                yPlane.get(yBytes)
                
                val uBytes = if (uPlane != null) {
                    uPlane.rewind()
                    val uSize = uPlane.remaining()
                    val bytes = ByteArray(uSize)
                    uPlane.get(bytes)
                    bytes
                } else null
                
                val vBytes = if (vPlane != null) {
                    vPlane.rewind()
                    val vSize = vPlane.remaining()
                    val bytes = ByteArray(vSize)
                    vPlane.get(bytes)
                    bytes
                } else null
                
                // ---- 原始帧截图：在 YUV 提取之后、VIO 处理之前检查挂起的捕获请求 ----
                val pendingPath = mPendingCapturePath
                if (pendingPath != null) {
                    mPendingCapturePath = null
                    val quality = mPendingCaptureQuality
                    try {
                        saveRawFrameAsJpeg(yBytes, uBytes, vBytes, imgWidth, imgHeight,
                            yStride, uStride, vStride, uPixelStride,
                            pendingPath, quality)
                    } catch (e: Exception) {
                        Log.e(TAG, "Failed to capture raw frame", e)
                        notifyCaptureResult(null)
                    }
                }

                // Convert YUV to RGBA in native code - returns Mat address
                val rgbaMatAddr = engine.processYUVToRGBA(
                    yBytes, uBytes, vBytes,
                    imgWidth, imgHeight,
                    yStride, uStride, vStride,
                    uPixelStride
                )
                
                if (rgbaMatAddr == 0L) {
                    // VIO 未初始化时仍录制原始相机帧，不跳过
                    if (mIsRecording) {
                        synchronized(mRecordingLock) {
                            if (mIsRecording) {
                                feedFrameToEncoder(
                                    yBytes, uBytes, vBytes,
                                    imgWidth, imgHeight,
                                    yStride, uStride, vStride,
                                    uPixelStride,
                                    frameTimestampNs
                                )
                            }
                        }
                    }
                    return@setOnImageAvailableListener
                }
                
                try {
                    // Notify listener with Mat address and timestamp (for processing)
                    // processImageJNI will clone the Mat before queuing, so it's safe to delete after
                    mFrameListener?.onFrame(rgbaMatAddr, lastFrameTimestampSec)
                    
                    // Get display image (raw camera if not running, or viz with overlays if running)
                    // Note: getDisplayImageJNI creates a new Mat, so we can safely delete rgbaMatAddr after this
                    val displayMatAddr = engine.getDisplayImage(rgbaMatAddr)
                    
                    // Now safe to delete the raw camera Mat since:
                    // 1. processImageJNI has cloned the data it needs before queuing
                    // 2. getDisplayImageJNI has created a new Mat and is done with the original
                    engine.deleteMat(rgbaMatAddr)
                    
                    if (displayMatAddr != 0L) {
                        try {
                            // Use Java Mat wrapper temporarily to access Mat data for bitmap conversion
                            val displayMat = Mat(displayMatAddr)
                            synchronized(mDisplayLock) {
                                val displayWidth = displayMat.width()
                                val displayHeight = displayMat.height()
                                if (mDisplayBitmap == null || mDisplayBitmap!!.width != displayWidth || mDisplayBitmap!!.height != displayHeight) {
                                    mDisplayBitmap?.recycle()
                                    mDisplayBitmap = Bitmap.createBitmap(displayWidth, displayHeight, Bitmap.Config.ARGB_8888)
                                }
                                // Convert Mat to Bitmap
                                Utils.matToBitmap(displayMat, mDisplayBitmap!!)
                                
                                // Draw to surface on UI thread
                                post {
                                    drawFrame()
                                }
                            }
                            // Delete the Mat manually (created with 'new' in C++)
                            // Set nativeObj to 0 first to prevent Java finalizer from double-deleting
                            val field = Mat::class.java.getDeclaredField("nativeObj")
                            field.isAccessible = true
                            field.setLong(displayMat, 0L)
                            engine.deleteMat(displayMatAddr)
                        } catch (e: Exception) {
                            Log.e(TAG, "Error displaying frame", e)
                            // Make sure we still delete the Mat even if there's an error
                            if (displayMatAddr != 0L) {
                                engine.deleteMat(displayMatAddr)
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e(TAG, "Error processing frame", e)
                    // Make sure we still delete the Mat even if there's an error
                    if (rgbaMatAddr != 0L) {
                        try {
                            engine.deleteMat(rgbaMatAddr)
                        } catch (e2: Exception) {
                            Log.e(TAG, "Error deleting Mat", e2)
                        }
                    }
                }
                
                // ---- 录制：在 VIO 处理之后、image.close() 之前，将帧送入编码器 ----
                if (mIsRecording) {
                    synchronized(mRecordingLock) {
                        if (mIsRecording && yBytes.isNotEmpty()) {
                            feedFrameToEncoder(
                                yBytes, uBytes, vBytes,
                                imgWidth, imgHeight,
                                yStride, uStride, vStride,
                                uPixelStride,
                                frameTimestampNs
                            )
                        }
                    }
                }

                } catch (e: IllegalStateException) {
                    // Image 可能在处理过程中被关闭（退出时竞态）
                    Log.w(TAG, "Image closed during processing", e)
                } catch (e: Exception) {
                    Log.e(TAG, "Error processing frame", e)
                } finally {
                    try {
                        image.close()
                    } catch (_: Exception) {
                        // Image 可能已经被关闭
                    }
                }
            }, mBackgroundHandler)
            
            mPreviewRequestBuilder = mCameraDevice!!.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
            mPreviewRequestBuilder!!.addTarget(mImageReader!!.surface)

            // Set focus mode to infinity or fixed for consistent exposure
            try {
                mPreviewRequestBuilder!!.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
                mPreviewRequestBuilder!!.set(CaptureRequest.LENS_FOCUS_DISTANCE, 0.0f) // Infinity
                Log.d(TAG, "Focus set to infinity")
            } catch (e: Exception) {
                Log.w(TAG, "Could not set focus to infinity: ${e.message}")
                mPreviewRequestBuilder!!.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            }

            // Set exposure compensation for better frame rate consistency
            try {
                mPreviewRequestBuilder!!.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
                mPreviewRequestBuilder!!.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, -2)
            } catch (e: Exception) {
                Log.w(TAG, "Could not set exposure compensation: ${e.message}")
            }

            mCameraDevice!!.createCaptureSession(
                listOf(mImageReader!!.surface),
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        Log.i(TAG, "createCaptureSession::onConfigured")
                        if (mCameraDevice == null) {
                            return
                        }
                        mCaptureSession = session
                        try {
                            mCaptureSession!!.setRepeatingRequest(
                                mPreviewRequestBuilder!!.build(),
                                null,
                                mBackgroundHandler
                            )
                            Log.i(TAG, "CameraPreviewSession has been started")
                        } catch (e: Exception) {
                            Log.e(TAG, "createCaptureSession failed", e)
                        }
                    }

                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        Log.e(TAG, "createCameraPreviewSession failed")
                    }
                },
                null
            )
        } catch (e: CameraAccessException) {
            Log.e(TAG, "createCameraPreviewSession", e)
        }
    }

    private fun calcPreviewSize(width: Int, height: Int): Boolean {
        if (mCameraID == null) {
            Log.e(TAG, "Camera isn't initialized!")
            return false
        }
        val manager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        try {
            val characteristics = manager.getCameraCharacteristics(mCameraID!!)
            val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            val sizes = map!!.getOutputSizes(ImageReader::class.java)
            var bestSize = sizes[0]
            var bestDiff = Integer.MAX_VALUE
            for (size in sizes) {
                val diff = Math.abs(size.width - width) + Math.abs(size.height - height)
                if (diff < bestDiff && size.width <= 800 && size.height <= 640) {
                    bestDiff = diff
                    bestSize = size
                }
            }
            if (mPreviewSize == bestSize) {
                return false
            }
            mPreviewSize = bestSize
            Log.i(TAG, "Selected preview size: ${mPreviewSize.width}x${mPreviewSize.height}")
            return true
        } catch (e: CameraAccessException) {
            Log.e(TAG, "calcPreviewSize", e)
        }
        return false
    }

    override fun onLayout(changed: Boolean, left: Int, top: Int, right: Int, bottom: Int) {
        super.onLayout(changed, left, top, right, bottom)
        // 布局完成后（View 有了实际宽高），重新检查状态以连接相机。
        // 解决 Fragment 首次展示时 enableView() 早于布局完成导致预览无法创建的问题。
        if (right - left > 0 && bottom - top > 0) {
            checkState()
        }
    }

    override fun surfaceCreated(holder: SurfaceHolder) {
        // Do nothing, wait for surfaceChanged
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        checkState()
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        checkState()
        synchronized(mDisplayLock) {
            mDisplayBitmap?.recycle()
            mDisplayBitmap = null
        }
    }
    
    private fun drawFrame() {
        val bitmap = synchronized(mDisplayLock) {
            mDisplayBitmap
        }
        
        if (bitmap == null) {
            return
        }
        
        val canvas = holder.lockCanvas()
        if (canvas != null) {
            try {
                // Clear canvas
                canvas.drawColor(android.graphics.Color.BLACK)
                
                // Scale bitmap to fit view while maintaining aspect ratio
                val viewWidth = width.toFloat()
                val viewHeight = height.toFloat()
                val bitmapWidth = bitmap.width.toFloat()
                val bitmapHeight = bitmap.height.toFloat()
                
                val scale = Math.min(viewWidth / bitmapWidth, viewHeight / bitmapHeight)
                val scaledWidth = bitmapWidth * scale
                val scaledHeight = bitmapHeight * scale
                val left = (viewWidth - scaledWidth) / 2f
                val top = (viewHeight - scaledHeight) / 2f
                
                canvas.drawBitmap(bitmap, null, android.graphics.RectF(left, top, left + scaledWidth, top + scaledHeight), null)
            } finally {
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    // ---- 视频录制公开接口 ----

    /**
     * 开始录制视频快照。
     * @param filePath 输出 MP4 文件路径（建议使用 getExternalFilesDir 或 getFilesDir 下的子目录）
     * @param bitrate  编码码率，默认 1Mbps（低码率节省空间）
     * @return true 表示成功启动录制
     */
    @JvmOverloads
    fun startRecording(filePath: String, bitrate: Int = 1_000_000): Boolean {
        synchronized(mRecordingLock) {
            if (mIsRecording) {
                Log.w(TAG, "Already recording")
                return false
            }
            if (mPreviewSize.width <= 0 || mPreviewSize.height <= 0) {
                Log.e(TAG, "Cannot start recording: preview size not set")
                return false
            }
            try {
                val width = mPreviewSize.width
                val height = mPreviewSize.height
                // 确保宽高为偶数（H.264 要求）
                val encWidth = width and 0x7FFFFFFE
                val encHeight = height and 0x7FFFFFFE

                val format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, encWidth, encHeight)
                format.setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
                format.setInteger(MediaFormat.KEY_FRAME_RATE, 30)
                format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 2)
                format.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar)

                val codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
                codec.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
                codec.start()

                // 确保输出目录存在
                val outFile = File(filePath)
                outFile.parentFile?.mkdirs()

                val muxer = MediaMuxer(filePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)

                mMediaCodec = codec
                mMediaMuxer = muxer
                mMuxerStarted = false
                mVideoTrackIndex = -1
                mRecordingStartTimeNs = -1L  // 等待首帧 timestamp 作为基准
                mEncodedFrameCount = 0
                mRecordingFilePath = filePath
                mIsRecording = true

                Log.i(TAG, "Recording started: $filePath (${encWidth}x${encHeight}, ${bitrate / 1000}kbps)")
                return true
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start recording", e)
                releaseRecorder()
                return false
            }
        }
    }

    /**
     * 停止录制并释放编码器资源。
     * @return 输出文件路径，若失败返回 null
     */
    fun stopRecording(): String? {
        synchronized(mRecordingLock) {
            if (!mIsRecording) {
                Log.w(TAG, "Not recording")
                return null
            }
            mIsRecording = false

            try {
                // 发送结束信号（EOS）
                mMediaCodec?.let { codec ->
                    // 循环等待直到拿到输入 buffer，确保 EOS 信号一定发出
                    var sent = false
                    for (attempt in 0 until 10) {
                        val bufIndex = codec.dequeueInputBuffer(10_000)  // 每次等 10ms
                        if (bufIndex >= 0) {
                            codec.queueInputBuffer(bufIndex, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                            sent = true
                            break
                        }
                    }
                    if (!sent) {
                        Log.w(TAG, "Failed to send EOS after retries, forcing drain")
                    }
                    drainEncoder(true)
                }
                Log.i(TAG, "Recording stopped")
            } catch (e: Exception) {
                Log.e(TAG, "Error stopping recording", e)
            }

            val outputPath = mRecordingFilePath
            mRecordingFilePath = null

            releaseRecorder()
            return outputPath
        }
    }

    /** 当前是否正在录制 */
    fun isRecording(): Boolean = mIsRecording

    /**
     * 截取下一帧原始相机图像并保存为 JPEG 文件。
     * 不依赖 VIO 引擎，不叠加任何处理，纯原始帧。
     * 保存完成后通过 callback 回调返回 File 对象（回调在 UI 线程执行）。
     * @param filePath 输出 JPEG 文件路径
     * @param quality  JPEG 压缩质量（0-100），默认 90
     * @param callback 保存完成后的回调，参数为保存的 File 或 null（失败时）
     */
    @JvmOverloads
    fun captureRawImage(filePath: String, quality: Int = 90, callback: (File?) -> Unit) {
        if (mCameraDevice == null) {
            Log.w(TAG, "captureRawImage: camera not connected")
            callback(null)
            return
        }
        synchronized(mRecordingLock) {
            mPendingCapturePath = filePath
            mPendingCaptureQuality = quality
            mCaptureCallbackHandler = Handler(android.os.Looper.getMainLooper())
            mCaptureCallback = callback
        }
        Log.i(TAG, "Raw frame capture requested: $filePath")
    }

    /** 在后台线程中执行原始帧保存（从 onImageAvailable 调用） */
    private fun saveRawFrameAsJpeg(
        yBytes: ByteArray, uBytes: ByteArray?, vBytes: ByteArray?,
        width: Int, height: Int,
        yStride: Int, uStride: Int, vStride: Int, uPixelStride: Int,
        filePath: String, quality: Int
    ) {
        val frameSize = width * height
        val nv21 = ByteArray(frameSize * 3 / 2)

        // 拷贝 Y 平面
        if (yStride == width) {
            System.arraycopy(yBytes, 0, nv21, 0, frameSize)
        } else {
            for (row in 0 until height) {
                System.arraycopy(yBytes, row * yStride, nv21, row * width, width)
            }
        }

        // 拷贝 VU 交织平面（NV21: V0 U0 V1 U1 ...）
        val vuOffset = frameSize
        val vuRows = height / 2
        if (uPixelStride == 2 && uBytes != null && vBytes != null && uStride == width) {
            // YUV_420_888 的 UV 已经是 NV21 布局（V 在前），直接拷贝 V 平面
            val vuSize = frameSize / 2
            System.arraycopy(vBytes, 0, nv21, vuOffset, minOf(vuSize, vBytes.size))
        } else if (uBytes != null && vBytes != null) {
            // 逐像素交织 V/U（NV21 顺序）
            var vuPos = 0
            for (row in 0 until vuRows) {
                for (col in 0 until width / 2) {
                    val vIdx = row * vStride + col * uPixelStride
                    val uIdx = row * uStride + col * uPixelStride
                    if (vIdx < vBytes.size && uIdx < uBytes.size && vuPos + 1 < nv21.size - vuOffset) {
                        nv21[vuOffset + vuPos] = vBytes[vIdx]
                        nv21[vuOffset + vuPos + 1] = uBytes[uIdx]
                        vuPos += 2
                    }
                }
            }
        }

        // 用 YuvImage 压缩为 JPEG
        val outFile = File(filePath)
        outFile.parentFile?.mkdirs()
        val yuvImage = YuvImage(nv21, ImageFormat.NV21, width, height, null)
        FileOutputStream(outFile).use { fos ->
            yuvImage.compressToJpeg(android.graphics.Rect(0, 0, width, height), quality, fos)
        }
        Log.i(TAG, "Raw frame saved: $filePath (${width}x${height})")
        notifyCaptureResult(outFile)
    }

    /** 将截图结果回调到 UI 线程 */
    private fun notifyCaptureResult(file: File?) {
        val handler = mCaptureCallbackHandler
        val callback = mCaptureCallback
        mCaptureCallbackHandler = null
        mCaptureCallback = null
        if (handler != null && callback != null) {
            handler.post { callback(file) }
        }
    }

    /** 释放编码器和复用器资源（内部使用） */
    private fun releaseRecorder() {
        try {
            mMediaCodec?.stop()
        } catch (_: Exception) {}
        try {
            mMediaCodec?.release()
        } catch (_: Exception) {}
        try {
            if (mMuxerStarted) {
                mMediaMuxer?.stop()
            }
            mMediaMuxer?.release()
        } catch (_: Exception) {}
        mMediaCodec = null
        mMediaMuxer = null
        mMuxerStarted = false
        mVideoTrackIndex = -1
    }

    /**
     * 将 YUV_420_888 帧送入编码器（在 mRecordingLock 保护下调用）。
     * 仅在 mIsRecording == true 时被调用。
     */
    private fun feedFrameToEncoder(
        yBytes: ByteArray, uBytes: ByteArray?, vBytes: ByteArray?,
        width: Int, height: Int,
        yStride: Int, uStride: Int, vStride: Int,
        uPixelStride: Int,
        timestampNs: Long
    ) {
        val codec = mMediaCodec ?: return
        // 确保宽高为偶数
        val encWidth = width and 0x7FFFFFFE
        val encHeight = height and 0x7FFFFFFE
        val frameSize = encWidth * encHeight

        // 构造 NV12（YUV420SP）：Y 平面 + 交织 UV 平面
        val nv12 = ByteArray(frameSize * 3 / 2)

        // 拷贝 Y 平面（处理 stride）
        if (yStride == encWidth) {
            System.arraycopy(yBytes, 0, nv12, 0, frameSize)
        } else {
            for (row in 0 until encHeight) {
                System.arraycopy(yBytes, row * yStride, nv12, row * encWidth, encWidth)
            }
        }

        // 拷贝 UV 交织平面（NV12: U0 V0 U1 V1 ...）
        val uvOffset = frameSize
        val uvRows = encHeight / 2
        if (uPixelStride == 2 && uBytes != null && uStride == encWidth) {
            // 已经是 NV12 布局，直接拷贝
            val uvSize = frameSize / 2
            System.arraycopy(uBytes, 0, nv12, uvOffset, minOf(uvSize, uBytes.size))
        } else if (uBytes != null && vBytes != null) {
            // 逐像素交织 U/V
            var uvPos = 0
            for (row in 0 until uvRows) {
                for (col in 0 until encWidth / 2) {
                    val uIdx = row * uStride + col * uPixelStride
                    val vIdx = row * vStride + col * uPixelStride  // V 与 U 有相同 pixelStride
                    if (uIdx < uBytes.size && vIdx < vBytes.size && uvPos + 1 < nv12.size - uvOffset) {
                        nv12[uvOffset + uvPos] = uBytes[uIdx]
                        nv12[uvOffset + uvPos + 1] = vBytes[vIdx]
                        uvPos += 2
                    }
                }
            }
        }

        try {
            val bufIndex = codec.dequeueInputBuffer(10_000)  // 10ms 超时，等待编码器就绪
            if (bufIndex < 0) {
                Log.w(TAG, "Encoder input buffer not available, skipping frame")
                return  // 编码器忙，跳过此帧
            }

            val inputBuffer = codec.getInputBuffer(bufIndex) ?: return
            val capacity = inputBuffer.capacity()
            inputBuffer.clear()
            inputBuffer.put(nv12)
            // 填充剩余空间为零，确保缓冲区完全填满（某些硬件编码器要求）
            val remaining = capacity - nv12.size
            if (remaining > 0) {
                inputBuffer.put(ByteArray(remaining))
            }

            // 使用首帧的 image.timestamp（CLOCK_BOOTTIME）作为基准，避免时钟不匹配
            if (mRecordingStartTimeNs < 0) {
                mRecordingStartTimeNs = timestampNs
                Log.i(TAG, "First recording frame timestamp: ${timestampNs}ns")
            }
            // 计算相对时间戳（微秒），相对于录制首帧时间
            val presentationTimeUs = (timestampNs - mRecordingStartTimeNs) / 1000  // ns -> us

            // 报告完整缓冲区大小（含零填充），确保编码器能正确处理
            codec.queueInputBuffer(bufIndex, 0, capacity, presentationTimeUs, 0)
            mEncodedFrameCount++
            if (mEncodedFrameCount % 30 == 1) {
                Log.d(TAG, "Recording frame #$mEncodedFrameCount, pts=${presentationTimeUs}us, muxerStarted=$mMuxerStarted")
            }
            drainEncoder(false)
        } catch (e: MediaCodec.CodecException) {
            if (!e.isRecoverable) {
                Log.e(TAG, "Encoder fatal error, stopping recording", e)
                // 注意：此处不重置 mIsRecording，由 stopRecording() 统一处理
                // 仅释放编码器资源，保留 mRecordingFilePath 以便 stopRecording 返回路径
                releaseRecorder()
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error feeding frame to encoder", e)
        }
    }

    /** 从编码器输出端取出编码数据并写入 Muxer */
    private fun drainEncoder(endOfStream: Boolean) {
        val codec = mMediaCodec ?: return
        val muxer = mMediaMuxer ?: return
        val bufferInfo = MediaCodec.BufferInfo()
        // 保护计数器，防止无限循环（某些编码器可能不产生输出）
        var maxIterations = if (endOfStream) 100 else 10

        while (maxIterations-- > 0) {
            val outputIndex = codec.dequeueOutputBuffer(bufferInfo, if (endOfStream) 10000 else 0)
            when {
                outputIndex == MediaCodec.INFO_TRY_AGAIN_LATER -> {
                    if (!endOfStream) return
                    // EOS 模式下继续等待
                }
                outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                    if (!mMuxerStarted) {
                        val newFormat = codec.outputFormat
                        Log.i(TAG, "Encoder output format changed: $newFormat")
                        mVideoTrackIndex = muxer.addTrack(newFormat)
                        muxer.start()
                        mMuxerStarted = true
                    }
                }
                outputIndex == MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED -> {
                    // 编码器内部缓冲区变更，继续循环即可
                    Log.d(TAG, "Encoder output buffers changed")
                }
                outputIndex >= 0 -> {
                    val outputBuffer = codec.getOutputBuffer(outputIndex)
                    if (outputBuffer != null && bufferInfo.size > 0 && mMuxerStarted) {
                        outputBuffer.position(bufferInfo.offset)
                        outputBuffer.limit(bufferInfo.offset + bufferInfo.size)
                        muxer.writeSampleData(mVideoTrackIndex, outputBuffer, bufferInfo)
                    }
                    codec.releaseOutputBuffer(outputIndex, false)
                    if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) {
                        Log.i(TAG, "Encoder EOS received, total encoded frames: $mEncodedFrameCount")
                        return
                    }
                }
            }
        }
        if (endOfStream) {
            Log.w(TAG, "drainEncoder: max iterations reached without EOS, muxerStarted=$mMuxerStarted")
        }
    }

    companion object {
        // 原生库由 VioEngine 统一加载
        private val engine = VioEngine()
    }
}
