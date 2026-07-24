package com.openvins.android.sdk

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import java.io.File
import java.io.FileOutputStream
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/** 无需轨迹 View 即可导出图片的配置。 */
data class TrajectoryImageConfig(
    /** 输出图片宽度，单位像素。 */
    val width: Int = 1200,
    /** 输出图片高度，单位像素。 */
    val height: Int = 1200,
    /** 网格间距，单位米。 */
    val gridSpacingMeters: Float = 1.0f,
    /** JPEG 压缩质量，取值 0 至 100。 */
    val jpegQuality: Int = 92,
)

/**
 * 将轨迹快照绘制为独立的俯视图，不依赖 Activity 布局或 OpenGL Surface。
 */
internal object TrajectoryImageExporter {

    fun export(
        trajectory: OpenVinsTrajectory,
        file: File,
        config: TrajectoryImageConfig,
    ): File? {
        if (trajectory.positions.size < 3 || config.width <= 0 || config.height <= 0) {
            return null
        }

        val pointCount = trajectory.positions.size / 3
        var minX = Float.POSITIVE_INFINITY
        var maxX = Float.NEGATIVE_INFINITY
        var minY = Float.POSITIVE_INFINITY
        var maxY = Float.NEGATIVE_INFINITY
        for (index in 0 until pointCount) {
            val x = trajectory.positions[index * 3]
            val y = trajectory.positions[index * 3 + 1]
            minX = min(minX, x)
            maxX = max(maxX, x)
            minY = min(minY, y)
            maxY = max(maxY, y)
        }

        val rangeX = max(maxX - minX, MIN_RANGE_METERS)
        val rangeY = max(maxY - minY, MIN_RANGE_METERS)
        val worldPadding = max(MIN_WORLD_PADDING_METERS, max(rangeX, rangeY) * 0.1f)
        minX -= worldPadding
        maxX += worldPadding
        minY -= worldPadding
        maxY += worldPadding

        val contentWidth = config.width - IMAGE_PADDING_PX * 2f
        val contentHeight = config.height - IMAGE_PADDING_PX * 2f
        val scale = min(contentWidth / (maxX - minX), contentHeight / (maxY - minY))
        val drawnWidth = (maxX - minX) * scale
        val drawnHeight = (maxY - minY) * scale
        val offsetX = (config.width - drawnWidth) / 2f
        val offsetY = (config.height - drawnHeight) / 2f

        fun screenX(worldX: Float): Float = offsetX + (worldX - minX) * scale
        fun screenY(worldY: Float): Float = config.height - (offsetY + (worldY - minY) * scale)

        val bitmap = Bitmap.createBitmap(config.width, config.height, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        canvas.drawColor(Color.rgb(12, 15, 18))
        drawGrid(canvas, minX, maxX, minY, maxY, config.gridSpacingMeters, ::screenX, ::screenY)
        drawTrajectory(canvas, trajectory.positions, pointCount, ::screenX, ::screenY)
        drawCurrentDirection(canvas, trajectory.currentPose, scale, ::screenX, ::screenY)

        return try {
            file.parentFile?.mkdirs()
            FileOutputStream(file).use { output ->
                bitmap.compress(Bitmap.CompressFormat.JPEG, config.jpegQuality.coerceIn(0, 100), output)
            }
            file
        } catch (_: Exception) {
            null
        } finally {
            bitmap.recycle()
        }
    }

    private fun drawGrid(
        canvas: Canvas,
        minX: Float,
        maxX: Float,
        minY: Float,
        maxY: Float,
        spacing: Float,
        screenX: (Float) -> Float,
        screenY: (Float) -> Float,
    ) {
        if (spacing <= 0f) return
        val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.rgb(55, 61, 66)
            strokeWidth = 1f
        }
        var x = floor(minX / spacing) * spacing
        while (x <= ceil(maxX / spacing) * spacing) {
            canvas.drawLine(screenX(x), screenY(minY), screenX(x), screenY(maxY), gridPaint)
            x += spacing
        }
        var y = floor(minY / spacing) * spacing
        while (y <= ceil(maxY / spacing) * spacing) {
            canvas.drawLine(screenX(minX), screenY(y), screenX(maxX), screenY(y), gridPaint)
            y += spacing
        }
    }

    private fun drawTrajectory(
        canvas: Canvas,
        positions: FloatArray,
        pointCount: Int,
        screenX: (Float) -> Float,
        screenY: (Float) -> Float,
    ) {
        if (pointCount < 2) return
        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeCap = Paint.Cap.ROUND
            strokeJoin = Paint.Join.ROUND
            strokeWidth = 7f
        }
        for (index in 1 until pointCount) {
            val progress = index.toFloat() / (pointCount - 1).toFloat()
            paint.color = trajectoryColor(progress)
            val previous = (index - 1) * 3
            val current = index * 3
            canvas.drawLine(
                screenX(positions[previous]),
                screenY(positions[previous + 1]),
                screenX(positions[current]),
                screenY(positions[current + 1]),
                paint,
            )
        }
    }

    private fun drawCurrentDirection(
        canvas: Canvas,
        pose: OpenVinsPose,
        scale: Float,
        screenX: (Float) -> Float,
        screenY: (Float) -> Float,
    ) {
        if (pose.position.size < 2 || pose.quaternion.size < 4) return
        val w = pose.quaternion[0]
        val x = pose.quaternion[1]
        val y = pose.quaternion[2]
        val z = pose.quaternion[3]
        val directionX = (1.0 - 2.0 * (y * y + z * z)).toFloat()
        val directionY = (2.0 * (x * y + z * w)).toFloat()
        val startX = screenX(pose.position[0].toFloat())
        val startY = screenY(pose.position[1].toFloat())
        val arrowLength = min(MAX_ARROW_LENGTH_PX, max(MIN_ARROW_LENGTH_PX, scale * 0.45f))
        val endX = startX + directionX * arrowLength
        val endY = startY - directionY * arrowLength

        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            style = Paint.Style.STROKE
            strokeWidth = 5f
            strokeCap = Paint.Cap.ROUND
        }
        canvas.drawCircle(startX, startY, 9f, paint)
        canvas.drawLine(startX, startY, endX, endY, paint)

        val length = max(1f, kotlin.math.sqrt((endX - startX) * (endX - startX) + (endY - startY) * (endY - startY)))
        val unitX = (endX - startX) / length
        val unitY = (endY - startY) / length
        val arrow = Path().apply {
            moveTo(endX, endY)
            lineTo(endX - unitX * 18f - unitY * 9f, endY - unitY * 18f + unitX * 9f)
            moveTo(endX, endY)
            lineTo(endX - unitX * 18f + unitY * 9f, endY - unitY * 18f - unitX * 9f)
        }
        canvas.drawPath(arrow, paint)
    }

    private fun trajectoryColor(progress: Float): Int {
        return if (progress < 0.5f) {
            val ratio = progress * 2f
            Color.rgb(0, (220 * ratio).toInt(), (255 * (1f - ratio)).toInt())
        } else {
            val ratio = (progress - 0.5f) * 2f
            Color.rgb((255 * ratio).toInt(), (220 * (1f - ratio)).toInt(), 0)
        }
    }

    private const val IMAGE_PADDING_PX = 64
    private const val MIN_RANGE_METERS = 1.0f
    private const val MIN_WORLD_PADDING_METERS = 0.5f
    private const val MIN_ARROW_LENGTH_PX = 42f
    private const val MAX_ARROW_LENGTH_PX = 100f
}
