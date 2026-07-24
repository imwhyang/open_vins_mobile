package com.openvins.app

import android.app.Activity
import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.android.material.floatingactionbutton.FloatingActionButton

/**
 * Activity 调用 Fragment 拍摄并接收结果的完整示例。
 */
class FragmentCaptureActivity : AppCompatActivity(), AutoCaptureFragmentListener {

    private lateinit var captureFragment: AutoCaptureFragment
    private lateinit var captureButton: FloatingActionButton
    private lateinit var finishButton: Button
    private lateinit var resultText: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_fragment_capture)

        captureButton = findViewById(R.id.capture_button)
        finishButton = findViewById(R.id.finish_task_button)
        resultText = findViewById(R.id.capture_result)

        captureFragment = supportFragmentManager.findFragmentByTag(FRAGMENT_TAG)
            as? AutoCaptureFragment
            ?: AutoCaptureFragment.newInstance().also { fragment ->
                supportFragmentManager.beginTransaction()
                    .replace(R.id.fragment_container, fragment, FRAGMENT_TAG)
                    .commit()
            }

        captureButton.setOnClickListener {
            captureFragment.capture()
        }
        finishButton.setOnClickListener {
            captureButton.isEnabled = false
            finishButton.isEnabled = false
            resultText.text = "正在生成任务文件..."
            captureFragment.finishTask()
        }
    }

    override fun onCaptureCompleted(result: FragmentCaptureResult) {
        resultText.text = if (result.isDuplicate) {
            "可能重复：${result.photoPath}\n距离：${"%.2f".format(result.duplicateDistanceMeters)} 米"
        } else {
            "拍摄成功：${result.photoPath}"
        }
        Log.i(TAG, "拍摄结果：$result")
    }

    override fun onTaskCompleted(result: FragmentTaskResult) {
        Log.i(TAG, "任务完成：$result")

        // 将最终文件路径返回给启动本 Activity 的页面。
        val data = Intent().apply {
            putExtra(EXTRA_TASK_ID, result.taskId)
            putExtra(EXTRA_VIDEO_PATH, result.videoPath)
            putExtra(EXTRA_POSE_PATH, result.posePath)
            putExtra(EXTRA_TRAJECTORY_IMAGE_PATH, result.trajectoryImagePath)
            putExtra(EXTRA_SESSION_JSON, result.sessionJson)
        }
        setResult(Activity.RESULT_OK, data)
        finish()
    }

    override fun onCaptureError(message: String) {
        captureButton.isEnabled = true
        finishButton.isEnabled = true
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    companion object {
        private const val TAG = "FragmentCaptureActivity"
        private const val FRAGMENT_TAG = "auto_capture_fragment"

        const val EXTRA_TASK_ID = "openvins_task_id"
        const val EXTRA_VIDEO_PATH = "openvins_video_path"
        const val EXTRA_POSE_PATH = "openvins_pose_path"
        const val EXTRA_TRAJECTORY_IMAGE_PATH = "openvins_trajectory_image_path"
        const val EXTRA_SESSION_JSON = "openvins_session_json"
    }
}
