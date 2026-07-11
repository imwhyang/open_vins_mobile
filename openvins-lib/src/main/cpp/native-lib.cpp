#include <atomic>
#include <algorithm>
#include <cassert>
#include <condition_variable>
#include <cmath>
#include <errno.h>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <jni.h>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <time.h>
#include <unistd.h>

// Android headers
#include <android/asset_manager.h>
#include <android/log.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// when building boost we persisted the NDK version used (BOOST_BUILT_WITH_NDK_VERSION) in this custom header file
#include <boost/chrono.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/version_ndk.hpp>

// OpenVINS project
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#define TAG "OpenVINSNative"

bool is_recording = false;
bool is_running_ov = false;
bool app_record_folder_set = false;
std::string app_record_folder = "/sdcard/";  // Public directory for recordings (user accessible)
std::string app_private_folder = "/sdcard/"; // Private external files directory (app has full access)
std::string save_folder = "/sdcard/";
std::ofstream imu_csv;
std::ofstream pose_ext_csv;
std::ofstream trajectory_debug_csv;
std::mutex pose_ext_csv_mtx;

//=========================================================
// OPENVINS SPECIFIC VARS - START
//=========================================================

// Master VIO system :)
std::shared_ptr<ov_msckf::VioManager> sys = nullptr;
std::mutex sys_mtx;

// Persistent worker thread for processing camera measurements
std::thread processing_thread;
std::atomic<bool> thread_should_run(false);
std::atomic<bool> thread_running(false);
std::mutex processing_mtx;
std::condition_variable processing_cv;

// Latest IMU timestamp (for determining which camera measurements can be processed)
double latest_imu_timestamp = 0.0;
double last_accepted_imu_timestamp = 0.0;
size_t accepted_imu_count = 0;
std::mutex imu_timestamp_mtx;

// Android IMU can occasionally emit large one-frame spikes. OpenVINS expects
// raw accelerometer readings including gravity, so keep normal ~9.81m/s^2 data
// and only suppress physically implausible glitches before propagation.
const double MAX_ACCEL_NORM = 60.0;      // m/s^2, about 6g including gravity.
const double MAX_GYRO_NORM = 20.0;       // rad/s, far above normal handheld motion.
const double MAX_ACCEL_JUMP = 35.0;      // m/s^2 jump between adjacent FASTEST samples.
const double MAX_GYRO_JUMP = 8.0;        // rad/s jump between adjacent FASTEST samples.
const double IMU_FILTER_RESET_DT = 0.25; // seconds; reset filter after long gaps.
bool imu_filter_initialized = false;
double last_valid_imu_timestamp = 0.0;
double last_valid_ax = 0.0;
double last_valid_ay = 0.0;
double last_valid_az = 9.81;
double last_valid_gx = 0.0;
double last_valid_gy = 0.0;
double last_valid_gz = 0.0;
std::mutex imu_filter_mtx;

// Queue up camera measurements sorted by time and trigger once we have
// exactly one IMU measurement with timestamp newer than the camera measurement
// This also handles out-of-order camera measurements, which is rare, but
// a nice feature to have for general robustness to bad camera drivers.
std::deque<ov_core::CameraData> camera_queue;
std::mutex camera_queue_mtx;

// Last camera message timestamps we have received (mapped by cam id)
std::map<int, double> camera_last_timestamp;

// Maximum age (in seconds) for camera measurements before skipping
const double MAX_CAMERA_AGE_SECONDS = 0.5; // Skip measurements older than 500ms

//=========================================================
// OPENVINS SPECIFIC VARS - END
//=========================================================

// Visualization data files
double viz_rate = 30.0;
double viz_time = -1.0;
double viz_track_rate = 0.0;
double viz_track_last_time = -1.0; // Track last processing timestamp for rate calculation
cv::Mat viz_image;
std::string viz_state1 = "";
std::string viz_state2 = "";
std::string viz_state3 = "";

// Trajectory storage for 3D visualization
struct TrajectoryPoint {
  double x, y, z;
  double qw, qx, qy, qz;
  TrajectoryPoint(double x_, double y_, double z_, double qw_, double qx_, double qy_, double qz_)
      : x(x_), y(y_), z(z_), qw(qw_), qx(qx_), qy(qy_), qz(qz_) {}
};
std::vector<TrajectoryPoint> trajectory_history;
std::mutex trajectory_mtx;
const size_t MAX_TRAJECTORY_POINTS = 10000; // Limit trajectory size
const double TRAJECTORY_DEBUG_JUMP_METERS = 0.25;
const double TRAJECTORY_DEBUG_SPEED_MPS = 1.5;
const double TRAJECTORY_DEBUG_ROTATION_RAD = 0.52; // 30 deg
const size_t TRAJECTORY_GATE_MIN_FEATURES = 20;
const size_t TRAJECTORY_GATE_LOW_FEATURES = 50;
const size_t TRAJECTORY_GATE_ROTATION_FEATURES = 60;
const double TRAJECTORY_GATE_MAX_STEP_METERS = 0.80;
const double TRAJECTORY_GATE_MAX_SPEED_MPS = 1.5;
const double TRAJECTORY_GATE_LOW_FEATURE_STEP_METERS = 0.35;
const double TRAJECTORY_GATE_ROTATION_STEP_METERS = 0.15;
const double TRAJECTORY_GATE_ROTATION_RAD = 0.52; // 30 deg
const double TRAJECTORY_TURN_GUARD_ROT_RAD = 0.08;
const double TRAJECTORY_TURN_GUARD_MAX_STEP_METERS = 0.08;
const double TRAJECTORY_TURN_GUARD_SECONDS = 1.00;
const double TRAJECTORY_TURN_GUARD_BACKWARD_PROJECTION = -0.25;
const double TRAJECTORY_TURN_GUARD_MIN_STEP_METERS = 0.03;
const double TRAJECTORY_TURN_GUARD_FAST_SPEED_MPS = 1.5;
const double TRAJECTORY_TURN_GUARD_FAST_STEP_METERS = 0.05;
const double TRAJECTORY_TURN_GUARD_LOW_FEATURE_STEP_METERS = 0.12;
const size_t TRAJECTORY_TURN_GUARD_LOW_FEATURES = 80;

enum TrajectoryRejectReason {
  TRAJECTORY_REJECT_NONE = 0,
  TRAJECTORY_REJECT_BAD_DT = 1,
  TRAJECTORY_REJECT_LOW_FEATURES = 2,
  TRAJECTORY_REJECT_LARGE_JUMP = 3,
  TRAJECTORY_REJECT_HIGH_SPEED = 4,
  TRAJECTORY_REJECT_LOW_FEATURE_JUMP = 5,
  TRAJECTORY_REJECT_ROTATION_LOW_FEATURE_JUMP = 6,
  TRAJECTORY_REJECT_BACKWARD_AFTER_TURN = 7,
  TRAJECTORY_REJECT_FAST_AFTER_TURN = 8,
  TRAJECTORY_REJECT_LOW_FEATURE_TURN_JUMP = 9,
};
const size_t TRAJECTORY_DRIFT_REJECT_STREAK = 3;

double trajectory_debug_last_timestamp = -1.0;
double trajectory_turn_guard_until_timestamp = -1.0;
bool trajectory_debug_has_last_pose = false;
TrajectoryPoint trajectory_debug_last_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
bool trajectory_data_paused = false;
bool trajectory_pause_prompt_pending = false;
int trajectory_pause_reason = TRAJECTORY_REJECT_NONE;
size_t trajectory_drift_reject_streak = 0;
bool trajectory_alignment_pending = false;
bool trajectory_alignment_active = false;
Eigen::Vector3d trajectory_alignment_anchor(0.0, 0.0, 0.0);
Eigen::Vector3d trajectory_alignment_offset(0.0, 0.0, 0.0);

double trajectory_quaternion_angle(const TrajectoryPoint &last, double qw, double qx, double qy, double qz) {
  double dot = std::abs(last.qw * qw + last.qx * qx + last.qy * qy + last.qz * qz);
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot);
}

double trajectory_forward_projection(const Eigen::Vector3d &position) {
  if (!trajectory_debug_has_last_pose) {
    return 0.0;
  }

  Eigen::Vector3d delta(position(0) - trajectory_debug_last_pose.x, position(1) - trajectory_debug_last_pose.y,
                        position(2) - trajectory_debug_last_pose.z);
  double step = delta.norm();
  if (step <= 1e-6) {
    return 0.0;
  }

  Eigen::Vector4d q_GtoC;
  q_GtoC << trajectory_debug_last_pose.qx, trajectory_debug_last_pose.qy, trajectory_debug_last_pose.qz, trajectory_debug_last_pose.qw;
  Eigen::Vector3d camera_forward_in_global = ov_core::quat_2_Rot(q_GtoC).transpose() * Eigen::Vector3d::UnitZ();
  return delta.normalized().dot(camera_forward_in_global.normalized());
}

bool compute_trajectory_delta(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion, double &dt, double &step,
                              double &speed, double &rot, double &forward_projection) {
  dt = 0.0;
  step = 0.0;
  speed = 0.0;
  rot = 0.0;
  forward_projection = 0.0;

  if (!trajectory_debug_has_last_pose) {
    return false;
  }

  double qw = quaternion(3);
  double qx = quaternion(0);
  double qy = quaternion(1);
  double qz = quaternion(2);
  dt = timestamp - trajectory_debug_last_timestamp;
  double dx = position(0) - trajectory_debug_last_pose.x;
  double dy = position(1) - trajectory_debug_last_pose.y;
  double dz = position(2) - trajectory_debug_last_pose.z;
  step = std::sqrt(dx * dx + dy * dy + dz * dz);
  speed = (dt > 1e-6) ? step / dt : 0.0;
  rot = trajectory_quaternion_angle(trajectory_debug_last_pose, qw, qx, qy, qz);
  forward_projection = trajectory_forward_projection(position);
  return true;
}

bool should_accept_trajectory_point(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion, size_t feature_count,
                                    int &reject_reason) {
  reject_reason = TRAJECTORY_REJECT_NONE;

  double dt = 0.0;
  double step = 0.0;
  double speed = 0.0;
  double rot = 0.0;
  double forward_projection = 0.0;
  if (!compute_trajectory_delta(timestamp, position, quaternion, dt, step, speed, rot, forward_projection)) {
    return true;
  }

  if (rot > TRAJECTORY_TURN_GUARD_ROT_RAD && step < TRAJECTORY_TURN_GUARD_MAX_STEP_METERS) {
    trajectory_turn_guard_until_timestamp = std::max(trajectory_turn_guard_until_timestamp, timestamp + TRAJECTORY_TURN_GUARD_SECONDS);
  }
  bool turn_guard_active = timestamp <= trajectory_turn_guard_until_timestamp;

  if (dt <= 0.0) {
    reject_reason = TRAJECTORY_REJECT_BAD_DT;
    return false;
  }
  if (feature_count < TRAJECTORY_GATE_MIN_FEATURES) {
    reject_reason = TRAJECTORY_REJECT_LOW_FEATURES;
    return false;
  }
  if (step > TRAJECTORY_GATE_MAX_STEP_METERS) {
    reject_reason = TRAJECTORY_REJECT_LARGE_JUMP;
    return false;
  }
  if (speed > TRAJECTORY_GATE_MAX_SPEED_MPS) {
    reject_reason = TRAJECTORY_REJECT_HIGH_SPEED;
    return false;
  }
  if (step > TRAJECTORY_GATE_LOW_FEATURE_STEP_METERS && feature_count < TRAJECTORY_GATE_LOW_FEATURES) {
    reject_reason = TRAJECTORY_REJECT_LOW_FEATURE_JUMP;
    return false;
  }
  if (step > TRAJECTORY_GATE_ROTATION_STEP_METERS && rot > TRAJECTORY_GATE_ROTATION_RAD &&
      feature_count < TRAJECTORY_GATE_ROTATION_FEATURES) {
    reject_reason = TRAJECTORY_REJECT_ROTATION_LOW_FEATURE_JUMP;
    return false;
  }
  if (turn_guard_active && step > TRAJECTORY_TURN_GUARD_MIN_STEP_METERS &&
      forward_projection < TRAJECTORY_TURN_GUARD_BACKWARD_PROJECTION) {
    reject_reason = TRAJECTORY_REJECT_BACKWARD_AFTER_TURN;
    return false;
  }
  if (turn_guard_active && step > TRAJECTORY_TURN_GUARD_FAST_STEP_METERS && speed > TRAJECTORY_TURN_GUARD_FAST_SPEED_MPS) {
    reject_reason = TRAJECTORY_REJECT_FAST_AFTER_TURN;
    return false;
  }
  if (turn_guard_active && step > TRAJECTORY_TURN_GUARD_LOW_FEATURE_STEP_METERS &&
      feature_count < TRAJECTORY_TURN_GUARD_LOW_FEATURES) {
    reject_reason = TRAJECTORY_REJECT_LOW_FEATURE_TURN_JUMP;
    return false;
  }

  return true;
}

bool is_trajectory_drift_reason(int reject_reason) {
  return reject_reason == TRAJECTORY_REJECT_LARGE_JUMP || reject_reason == TRAJECTORY_REJECT_HIGH_SPEED ||
         reject_reason == TRAJECTORY_REJECT_BACKWARD_AFTER_TURN || reject_reason == TRAJECTORY_REJECT_FAST_AFTER_TURN ||
         reject_reason == TRAJECTORY_REJECT_LOW_FEATURE_TURN_JUMP;
}

void pause_trajectory_data(int reject_reason) {
  trajectory_data_paused = true;
  trajectory_pause_prompt_pending = true;
  trajectory_pause_reason = reject_reason;
  trajectory_drift_reject_streak = 0;
}

void reset_trajectory_debug_state() {
  trajectory_debug_last_timestamp = -1.0;
  trajectory_turn_guard_until_timestamp = -1.0;
  trajectory_debug_has_last_pose = false;
  trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  trajectory_data_paused = false;
  trajectory_pause_prompt_pending = false;
  trajectory_pause_reason = TRAJECTORY_REJECT_NONE;
  trajectory_drift_reject_streak = 0;
  trajectory_alignment_pending = false;
  trajectory_alignment_active = false;
  trajectory_alignment_anchor = Eigen::Vector3d::Zero();
  trajectory_alignment_offset = Eigen::Vector3d::Zero();
}

void record_trajectory_debug(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion, size_t trajectory_size,
                             size_t feature_count, bool zupt_active, bool accepted, int reject_reason) {
  double qw = quaternion(3);
  double qx = quaternion(0);
  double qy = quaternion(1);
  double qz = quaternion(2);

  double dt = 0.0;
  double step = 0.0;
  double speed = 0.0;
  double rot = 0.0;
  double forward_projection = 0.0;
  bool anomaly = false;
  bool turn_guard_active = timestamp <= trajectory_turn_guard_until_timestamp;

  if (compute_trajectory_delta(timestamp, position, quaternion, dt, step, speed, rot, forward_projection)) {
    anomaly = dt <= 0.0 || step > TRAJECTORY_DEBUG_JUMP_METERS || speed > TRAJECTORY_DEBUG_SPEED_MPS ||
              (step > 0.03 && rot > TRAJECTORY_DEBUG_ROTATION_RAD);
  }

  if (trajectory_debug_csv.is_open()) {
    trajectory_debug_csv << std::fixed << std::setprecision(9) << timestamp << "," << trajectory_size << "," << position(0) << ","
                         << position(1) << "," << position(2) << "," << qx << "," << qy << "," << qz << "," << qw << "," << dt
                         << "," << step << "," << speed << "," << rot << "," << feature_count << "," << (zupt_active ? 1 : 0) << ","
                         << forward_projection << "," << (turn_guard_active ? 1 : 0) << "," << (accepted ? 1 : 0) << ","
                         << reject_reason << "," << (anomaly ? 1 : 0) << std::endl;
  }

  if (anomaly || !accepted) {
    __android_log_print(ANDROID_LOG_WARN, TAG,
                        "Trajectory point: t=%.6f dt=%.4f step=%.3f speed=%.3f rot_deg=%.1f forward=%.2f guard=%d features=%zu zupt=%d accepted=%d reason=%d size=%zu\n",
                        timestamp, dt, step, speed, rot * 180.0 / M_PI, forward_projection, turn_guard_active ? 1 : 0, feature_count,
                        zupt_active ? 1 : 0, accepted ? 1 : 0, reject_reason, trajectory_size);
  }

  trajectory_debug_last_timestamp = timestamp;
  trajectory_debug_has_last_pose = true;
  trajectory_debug_last_pose = TrajectoryPoint(position(0), position(1), position(2), qw, qx, qy, qz);
}

// JNI OnLoad/OnUnload handlers for proper cleanup
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) { return JNI_VERSION_1_6; }

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM *vm, void *reserved) {
  // Ensure all threads are stopped and resources are cleaned up
  __android_log_print(ANDROID_LOG_INFO, TAG, "JNI_OnUnload: cleaning up resources\n");

  // Stop the worker thread if running
  if (thread_running) {
    thread_should_run = false;
    processing_cv.notify_all();

    // Wait for thread to finish (with timeout)
    if (processing_thread.joinable()) {
      processing_thread.join();
    }
  }

  // Clean up VIO system
  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    sys = nullptr;
  }
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);
    camera_queue.clear();
    camera_last_timestamp.clear();
  }

  // Close IMU CSV if open
  if (imu_csv.is_open()) {
    imu_csv.close();
  }
  if (pose_ext_csv.is_open()) {
    pose_ext_csv.close();
  }

  __android_log_print(ANDROID_LOG_INFO, TAG, "JNI_OnUnload: cleanup complete\n");
}

// Worker thread function that continuously processes camera measurements
void processing_worker_thread() {
  __android_log_print(ANDROID_LOG_INFO, TAG, "Processing worker thread started\n");

  while (thread_should_run) {
    {
      std::unique_lock<std::mutex> proc_lck(processing_mtx);

      // Wait for either new data or shutdown signal
      // Timeout after 100ms to periodically check if we should exit
      processing_cv.wait_for(proc_lck, std::chrono::milliseconds(100), [&] { return !thread_should_run; });
    } // Release lock before processing

    if (!thread_should_run) {
      break;
    }

    // Get latest IMU timestamp
    double current_imu_timestamp;
    {
      std::lock_guard<std::mutex> imu_lck(imu_timestamp_mtx);
      current_imu_timestamp = latest_imu_timestamp;
    }

    size_t current_imu_count;
    {
      std::lock_guard<std::mutex> imu_lck(imu_timestamp_mtx);
      current_imu_count = accepted_imu_count;
    }

    std::shared_ptr<ov_msckf::VioManager> local_sys;
    {
      std::lock_guard<std::mutex> sys_lck(sys_mtx);
      local_sys = sys;
    }

    // Check if we have a valid system and enough IMU data to propagate safely.
    if (local_sys == nullptr || current_imu_timestamp <= 0.0 || current_imu_count < 5) {
      continue;
    }

    // Calculate IMU timestamp in camera frame (outside lock since sys is thread-safe)
    double timestamp_imu_inC = current_imu_timestamp - local_sys->get_state()->_calib_dt_CAMtoIMU->value()(0);

    // Get current time in boot time reference (to match camera/IMU timestamps)
    // Use CLOCK_BOOTTIME to get nanoseconds since boot (same reference as camera/IMU)
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    unsigned long long time_now_ns = (unsigned long long)ts.tv_sec * 1000000000ULL + (unsigned long long)ts.tv_nsec;
    double time_now_sec = 1e-9 * (double)time_now_ns;

    // Process camera measurements that are ready and not too old
    while (true) {
      // Time the queue operations
      auto t_queue_start = boost::posix_time::microsec_clock::local_time();

      // Lock to check, copy, and pop the front element
      ov_core::CameraData cam_msg;
      bool has_message = false;
      size_t queue_size_after_pop = 0;
      {
        std::lock_guard<std::mutex> cam_lck(camera_queue_mtx);
        if (camera_queue.empty()) {
          break;
        }

        // Check if measurement is too old (falling behind realtime)
        double age_seconds = time_now_sec - camera_queue.at(0).timestamp;
        if (age_seconds > MAX_CAMERA_AGE_SECONDS) {
          __android_log_print(ANDROID_LOG_WARN, TAG, "Skipping old camera measurement: %.3f seconds old\n", age_seconds);
          camera_queue.pop_front();
          continue;
        }

        // Only process if IMU timestamp is newer than camera timestamp
        if (camera_queue.at(0).timestamp >= timestamp_imu_inC) {
          // Log when waiting for IMU data
          double imu_wait = camera_queue.at(0).timestamp - timestamp_imu_inC;
          __android_log_print(ANDROID_LOG_DEBUG, TAG, "Waiting for IMU: cam_ts=%.4f, imu_ts=%.4f, diff=%.4f, queue_size=%zu\n",
                              camera_queue.at(0).timestamp, timestamp_imu_inC, imu_wait, camera_queue.size());
          break; // Wait for more IMU data
        }

        // Copy and pop while holding the lock
        cam_msg = camera_queue.at(0);
        camera_queue.pop_front();
        queue_size_after_pop = camera_queue.size();
        has_message = true;
      }

      auto t_queue_end = boost::posix_time::microsec_clock::local_time();
      double time_queue = (t_queue_end - t_queue_start).total_microseconds() * 1e-6;

      // Log when we start processing a frame (queue draining)
      //__android_log_print(ANDROID_LOG_INFO, TAG, "Processing frame, queue size after pop = %zu\n", queue_size_after_pop);
      if (!has_message) {
        break;
      }

      // Process this camera measurement (lock is released during this call)
      auto t_feed_start = boost::posix_time::microsec_clock::local_time();
      double update_dt = 100.0 * (timestamp_imu_inC - cam_msg.timestamp);
      if (!thread_should_run) {
        break;
      }
      try {
        local_sys->feed_measurement_camera(cam_msg);
      } catch (const std::exception &e) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "feed_measurement_camera exception: %s\n", e.what());
        continue;
      } catch (...) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "feed_measurement_camera unknown exception\n");
        continue;
      }
      auto t_feed_end = boost::posix_time::microsec_clock::local_time();
      double time_feed = (t_feed_end - t_feed_start).total_microseconds() * 1e-6;

      // Time state retrieval
      auto t_state_start = boost::posix_time::microsec_clock::local_time();
      auto state = local_sys->get_state();
      auto q_GtoI = state->_imu->quat();
      auto p_IinG = state->_imu->pos();

      // Transform IMU pose to camera pose using calibration
      // Get calibration for camera 0 (assuming single camera setup)
      assert(state->_calib_IMUtoCAM.find(0) != state->_calib_IMUtoCAM.end() && "Camera calibration for camera 0 must exist");
      auto calib = state->_calib_IMUtoCAM.at(0);
      Eigen::Vector4d q_ItoC = calib->quat();
      Eigen::Vector3d p_IinC = calib->pos();
      Eigen::Vector4d q_cam = ov_core::quat_multiply(q_ItoC, q_GtoI);
      Eigen::Vector3d p_cam = p_IinG - ov_core::quat_2_Rot(q_cam).transpose() * p_IinC;

      auto t_state_end = boost::posix_time::microsec_clock::local_time();
      double time_state = (t_state_end - t_state_start).total_microseconds() * 1e-6;

      // Update visualization stuff
      // Calculate actual processing rate based on time between consecutive frames
      auto t_viz_start = boost::posix_time::microsec_clock::local_time();
      double current_time = cam_msg.timestamp;
      if (viz_track_last_time > 0.0) {
        double time_delta = current_time - viz_track_last_time;
        if (time_delta > 0.0) {
          viz_track_rate = 1.0 / time_delta;
        }
      }
      viz_track_last_time = current_time;

      // Display things if we have initialized
      if (local_sys->initialized()) {
        // Store trajectory point (camera pose)
        // q_cam is JPL format [qx, qy, qz, qw], but TrajectoryPoint expects [qw, qx, qy, qz]
        std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
        Eigen::Vector3d p_traj = p_cam;
        if (trajectory_alignment_pending) {
          Eigen::Vector3d alignment_anchor = trajectory_alignment_anchor;
          Eigen::Vector3d alignment_offset = alignment_anchor - p_cam;
          reset_trajectory_debug_state();
          trajectory_alignment_anchor = alignment_anchor;
          trajectory_alignment_offset = alignment_offset;
          trajectory_alignment_active = true;
          trajectory_alignment_pending = false;
        }
        if (trajectory_alignment_active) {
          p_traj = p_cam + trajectory_alignment_offset;
        }
        size_t feature_count = local_sys->get_last_track_count();
        bool zupt_active = local_sys->last_update_used_zupt();
        int reject_reason = TRAJECTORY_REJECT_NONE;
        bool accept_trajectory_point = false;
        if (!trajectory_data_paused) {
          accept_trajectory_point = should_accept_trajectory_point(state->_timestamp, p_traj, q_cam, feature_count, reject_reason);
        }
        if (accept_trajectory_point) {
          trajectory_history.emplace_back(p_traj(0), p_traj(1), p_traj(2), q_cam(3), q_cam(0), q_cam(1), q_cam(2));
          if (trajectory_history.size() > MAX_TRAJECTORY_POINTS) {
            trajectory_history.erase(trajectory_history.begin());
          }
          trajectory_drift_reject_streak = 0;
        } else if (!trajectory_data_paused && is_trajectory_drift_reason(reject_reason)) {
          trajectory_drift_reject_streak++;
          if (trajectory_drift_reject_streak >= TRAJECTORY_DRIFT_REJECT_STREAK) {
            pause_trajectory_data(reject_reason);
          }
        } else if (!trajectory_data_paused) {
          trajectory_drift_reject_streak = 0;
        }
        if (trajectory_data_paused) {
          accept_trajectory_point = false;
          reject_reason = trajectory_pause_reason;
        }
        record_trajectory_debug(state->_timestamp, p_traj, q_cam, trajectory_history.size(), feature_count, zupt_active, accept_trajectory_point,
                                reject_reason);

        // Display the current state
        std::stringstream ss1, ss2, ss3;
        // Combine q and p on same line
        ss1 << std::fixed << std::setprecision(3);
        ss1 << "q = " << q_GtoI(0) << "," << q_GtoI(1) << "," << q_GtoI(2) << "," << q_GtoI(3);
        ss1 << "  p = " << std::setprecision(2) << p_IinG(0) << "," << p_IinG(1) << "," << p_IinG(2);

        // Combine q_c and p_c on same line
        ss2 << std::fixed << std::setprecision(3);
        ss2 << "q_c = " << q_ItoC(0) << "," << q_ItoC(1) << "," << q_ItoC(2) << "," << q_ItoC(3);
        ss2 << "  p_c = " << p_IinC(0) << "," << p_IinC(1) << "," << p_IinC(2);

        // Camera-IMU time offset (should always be available)
        assert(state->_calib_dt_CAMtoIMU != nullptr && "Camera-IMU time offset calibration must exist");
        ss3 << std::fixed << std::setprecision(5);
        ss3 << "dt = " << state->_calib_dt_CAMtoIMU->value()(0);

        viz_state1 = ss1.str();
        viz_state2 = ss2.str();
        viz_state3 = ss3.str();
      }
      auto t_viz_end = boost::posix_time::microsec_clock::local_time();
      double time_viz = (t_viz_end - t_viz_start).total_microseconds() * 1e-6;

      // Calculate total time and log breakdown
      auto t_total_end = boost::posix_time::microsec_clock::local_time();
      double time_total = (t_total_end - t_queue_start).total_microseconds() * 1e-6;
      PRINT_ERROR("[TIME]: %.4f total (%.1f hz, %.2f ms behind) | queue=%.4f | feed=%.4f | state=%.4f | viz=%.4f\n", time_total,
                  1.0 / time_total, update_dt, time_queue, time_feed, time_state, time_viz);
    }
  }

  thread_running = false;
  __android_log_print(ANDROID_LOG_INFO, TAG, "Processing worker thread stopped\n");
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_setAppRecordFolderJNI(JNIEnv *env, jobject instance, jstring dir) {
  const char *temp = env->GetStringUTFChars(dir, NULL);
  app_record_folder = std::string(temp);
  app_record_folder_set = true;
  __android_log_print(ANDROID_LOG_INFO, TAG, "export app record folder: %s\n", app_record_folder.c_str());
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_setAppPrivateFolderJNI(JNIEnv *env, jobject instance, jstring dir) {
  const char *temp = env->GetStringUTFChars(dir, NULL);
  app_private_folder = std::string(temp);
  __android_log_print(ANDROID_LOG_INFO, TAG, "export app private folder: %s\n", app_private_folder.c_str());
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_setRecordStateJNI(JNIEnv *env, jobject instance, jboolean stateAddr) {
  is_recording = (bool)stateAddr;
  if (is_recording) {

    // Create folder with the current time as the folder name
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time),
                        "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');

    // Normalize path: remove trailing slash from app_record_folder if present
    std::string normalized_app_record_folder = app_record_folder;
    if (!normalized_app_record_folder.empty() && normalized_app_record_folder.back() == '/') {
      normalized_app_record_folder.pop_back();
    }
    //  修改保存路径与文件名
    save_folder = normalized_app_record_folder + "/";
    //    save_folder = normalized_app_record_folder + "/" + s + "/";

    // Make the folder if not there
    struct stat st = {0};
    if (stat(save_folder.c_str(), &st) == -1) {
      mkdir(save_folder.c_str(), 0700);
    }
    //    取消保存图片
    //    mkdir((save_folder + "cam0/").c_str(), 0700);

    //    文件名加时间
    // Open our IMU csv file
    //    imu_csv.open(save_folder + "imu0.csv");
    //    std::string imu_csv_name = s + "imu0.csv";
    //    imu_csv.open(save_folder + imu_csv_name);
    //    imu_csv << "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z" << std::endl;
    //   只保留Pose数据
    std::string pose_csv_name = s + "pose0.csv";
    pose_ext_csv.open(save_folder + pose_csv_name);
    pose_ext_csv << "timestamp,p_x,p_y,p_z,q_x,q_y,q_z,q_w" << std::endl;

    std::string trajectory_debug_csv_name = s + "trajectory_debug.csv";
    trajectory_debug_csv.open(save_folder + trajectory_debug_csv_name);
    trajectory_debug_csv
        << "timestamp,trajectory_size,p_x,p_y,p_z,q_x,q_y,q_z,q_w,dt,step_m,speed_mps,rot_rad,feature_count,zupt_active,forward_projection,turn_guard_active,accepted,reject_reason,anomaly"
        << std::endl;
    reset_trajectory_debug_state();
  } else {

    // If the file was open, then close it
    if (imu_csv.is_open()) {
      imu_csv.close();
    }
    if (pose_ext_csv.is_open()) {
      pose_ext_csv.close();
    }
    if (trajectory_debug_csv.is_open()) {
      trajectory_debug_csv.close();
    }
    reset_trajectory_debug_state();
  }
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_toggleSystemJNI(JNIEnv *env, jobject instance, jboolean stateAddr) {
  is_running_ov = (bool)stateAddr;
  if (!is_running_ov) {
    // Stop the system: shutdown immediately
    __android_log_print(ANDROID_LOG_INFO, TAG, "Stopping OpenVINS system...\n");

    // Stop the worker thread
    if (thread_running) {
      thread_should_run = false;
      processing_cv.notify_all();

      // Wait for thread to finish (with timeout)
      if (processing_thread.joinable()) {
        processing_thread.join();
      }
    }

    // Set sys to nullptr - this signals shutdown to all threads
    // Shared_ptr will automatically destroy VioManager when last reference is released
    {
      std::lock_guard<std::mutex> sys_lck(sys_mtx);
      sys = nullptr;
    }
    {
      std::lock_guard<std::mutex> lck(camera_queue_mtx);
      // Clear queues immediately
      camera_queue.clear();
      camera_last_timestamp.clear();
    }

    // Reset IMU timestamp
    {
      std::lock_guard<std::mutex> imu_lck(imu_timestamp_mtx);
      latest_imu_timestamp = 0.0;
      last_accepted_imu_timestamp = 0.0;
      accepted_imu_count = 0;
    }
    {
      std::lock_guard<std::mutex> filter_lck(imu_filter_mtx);
      imu_filter_initialized = false;
      last_valid_imu_timestamp = 0.0;
      last_valid_ax = 0.0;
      last_valid_ay = 0.0;
      last_valid_az = 9.81;
      last_valid_gx = 0.0;
      last_valid_gy = 0.0;
      last_valid_gz = 0.0;
    }

    // Reset visualization state
    viz_time = -1;
    viz_track_rate = 0.0;
    viz_track_last_time = -1.0;
    viz_state1 = "";
    viz_state2 = "";
    viz_state3 = "";
    reset_trajectory_debug_state();

    __android_log_print(ANDROID_LOG_INFO, TAG, "OpenVINS system stopped\n");
  } else {
    // Starting the system: ensure clean state
    __android_log_print(ANDROID_LOG_INFO, TAG, "Starting OpenVINS system...\n");

    // Clear trajectory and queues
    {
      std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
      trajectory_history.clear();
    }
    reset_trajectory_debug_state();
    {
      std::lock_guard<std::mutex> lck(camera_queue_mtx);
      camera_queue.clear();
      camera_last_timestamp.clear();
    }

    // Reset IMU timestamp
    {
      std::lock_guard<std::mutex> imu_lck(imu_timestamp_mtx);
      latest_imu_timestamp = 0.0;
      last_accepted_imu_timestamp = 0.0;
      accepted_imu_count = 0;
    }
    {
      std::lock_guard<std::mutex> filter_lck(imu_filter_mtx);
      imu_filter_initialized = false;
      last_valid_imu_timestamp = 0.0;
      last_valid_ax = 0.0;
      last_valid_ay = 0.0;
      last_valid_az = 9.81;
      last_valid_gx = 0.0;
      last_valid_gy = 0.0;
      last_valid_gz = 0.0;
    }

    // Reset visualization state
    viz_time = -1;
    viz_track_rate = 0.0;
    viz_track_last_time = -1.0;
    viz_state1 = "";
    viz_state2 = "";
    viz_state3 = "";

    // Start the worker thread
    if (!thread_running) {
      thread_should_run = true;
      thread_running = true;
      processing_thread = std::thread(processing_worker_thread);
      __android_log_print(ANDROID_LOG_INFO, TAG, "Processing worker thread started\n");
    }
  }
}

extern "C" JNIEXPORT jlong JNICALL Java_com_openvins_android_VioEngine_processYUVToRGBAJNI(JNIEnv *env, jobject clazz, jbyteArray yData,
                                                                                           jbyteArray uData, jbyteArray vData, jint width,
                                                                                           jint height, jint yStride, jint uStride,
                                                                                           jint vStride, jint chromaPixelStride) {
  // This is a helper function to convert YUV to RGBA in native code
  // Creates Mat in native code and returns its address

  // NOTE TO FUTURE SELF
  // This is a helper function, but I am not really sure if if it is needed. Why don't we just
  // get the YUV directly in the other function from java and then do the conversion instead
  // of passing this converted information back up into android level and then passing it back
  // down into the other camera feed processing function?

  // Create output Mat in native code
  cv::Mat *outputMat = new cv::Mat(height, width, CV_8UC4);
  if (yData == nullptr) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Y plane data is null");
    return 0;
  }

  jsize yLength = env->GetArrayLength(yData);
  jbyte *yBytes = env->GetByteArrayElements(yData, nullptr);
  if (chromaPixelStride == 2) {
    // Interleaved chroma (NV12/NV21)
    if (uData == nullptr) {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "UV plane data is null for interleaved format");
      env->ReleaseByteArrayElements(yData, yBytes, JNI_ABORT);
      delete outputMat;
      return 0;
    }

    jsize uLength = env->GetArrayLength(uData);
    jbyte *uBytes = env->GetByteArrayElements(uData, nullptr);

    // Create Y Mat with stride
    cv::Mat yMat;
    if (yStride == width) {
      // No padding, can use directly
      yMat = cv::Mat(height, width, CV_8UC1, (void *)yBytes);
    } else {
      // Has padding, need to copy row by row
      yMat = cv::Mat(height, width, CV_8UC1);
      for (int row = 0; row < height; row++) {
        memcpy(yMat.ptr(row), yBytes + row * yStride, width);
      }
    }

    // Create UV Mat (UV interleaved, so size is height/2 x width/2 x 2)
    cv::Mat uvMat;
    if (uStride == width) {
      // No padding, can use directly
      uvMat = cv::Mat(height / 2, width / 2, CV_8UC2, (void *)uBytes);
    } else {
      // Has padding, need to copy row by row
      uvMat = cv::Mat(height / 2, width / 2, CV_8UC2);
      int uvRowSize = (width / 2) * 2;
      for (int row = 0; row < height / 2; row++) {
        memcpy(uvMat.ptr(row), uBytes + row * uStride, uvRowSize);
      }
    }

    // Convert using two-plane conversion (more efficient)
    // Use NV21 (most common on Android devices)
    // Note: If colors are inverted, try NV12 instead
    cv::cvtColorTwoPlane(yMat, uvMat, *outputMat, cv::COLOR_YUV2RGBA_NV21);

    env->ReleaseByteArrayElements(uData, uBytes, JNI_ABORT);
  } else {
    // Non-interleaved chroma (I420)
    if (uData == nullptr || vData == nullptr) {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "U or V plane data is null for I420 format");
      env->ReleaseByteArrayElements(yData, yBytes, JNI_ABORT);
      delete outputMat;
      return 0;
    }

    jsize uLength = env->GetArrayLength(uData);
    jsize vLength = env->GetArrayLength(vData);
    jbyte *uBytes = env->GetByteArrayElements(uData, nullptr);
    jbyte *vBytes = env->GetByteArrayElements(vData, nullptr);

    // Create Y Mat
    cv::Mat yMat;
    if (yStride == width) {
      yMat = cv::Mat(height, width, CV_8UC1, (void *)yBytes);
    } else {
      yMat = cv::Mat(height, width, CV_8UC1);
      for (int row = 0; row < height; row++) {
        memcpy(yMat.ptr(row), yBytes + row * yStride, width);
      }
    }

    // Create U and V Mats
    cv::Mat uMat, vMat;
    int chromaWidth = width / 2;
    int chromaHeight = height / 2;

    if (uStride == chromaWidth) {
      uMat = cv::Mat(chromaHeight, chromaWidth, CV_8UC1, (void *)uBytes);
    } else {
      uMat = cv::Mat(chromaHeight, chromaWidth, CV_8UC1);
      for (int row = 0; row < chromaHeight; row++) {
        memcpy(uMat.ptr(row), uBytes + row * uStride, chromaWidth);
      }
    }

    if (vStride == chromaWidth) {
      vMat = cv::Mat(chromaHeight, chromaWidth, CV_8UC1, (void *)vBytes);
    } else {
      vMat = cv::Mat(chromaHeight, chromaWidth, CV_8UC1);
      for (int row = 0; row < chromaHeight; row++) {
        memcpy(vMat.ptr(row), vBytes + row * vStride, chromaWidth);
      }
    }

    // Combine into I420 format: Y + U + V
    cv::Mat yuvMat(height + chromaHeight, width, CV_8UC1);
    yMat.copyTo(yuvMat(cv::Rect(0, 0, width, height)));
    uMat.copyTo(yuvMat(cv::Rect(0, height, chromaWidth, chromaHeight)));
    vMat.copyTo(yuvMat(cv::Rect(0, height + chromaHeight, chromaWidth, chromaHeight)));

    // Convert to RGBA
    cv::cvtColor(yuvMat, *outputMat, cv::COLOR_YUV2RGBA_I420, 4);

    env->ReleaseByteArrayElements(uData, uBytes, JNI_ABORT);
    env->ReleaseByteArrayElements(vData, vBytes, JNI_ABORT);
  }

  env->ReleaseByteArrayElements(yData, yBytes, JNI_ABORT);

  // Return Mat address to Java (Java will then call getDisplayImageJNI with this address)
  return reinterpret_cast<jlong>(outputMat);
}

// Get display image - returns raw camera if not running, or viz image with overlays if running
extern "C" JNIEXPORT jlong JNICALL Java_com_openvins_android_VioEngine_getDisplayImageJNI(JNIEnv *env, jobject clazz,
                                                                                          jlong rawCameraMatAddr) {
  std::shared_ptr<ov_msckf::VioManager> local_sys;
  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    local_sys = sys;
  }

  // If not running, just return the raw camera image (converted to RGB)
  if (!is_running_ov || local_sys == nullptr) {
    if (rawCameraMatAddr == 0) {
      return 0;
    }

    cv::Mat &rawMat = *(cv::Mat *)rawCameraMatAddr;
    // Convert RGBA to RGB for display
    cv::Mat *rgbMat = new cv::Mat();
    cv::cvtColor(rawMat, *rgbMat, cv::COLOR_RGBA2GRAY);
    return reinterpret_cast<jlong>(rgbMat);
  }

  // System is running - get visualization image with overlays
  cv::Mat viz_img = local_sys->get_historical_viz_image();
  if (viz_img.empty()) {
    // Fallback to raw camera if no viz image
    if (rawCameraMatAddr != 0) {
      cv::Mat &rawMat = *(cv::Mat *)rawCameraMatAddr;
      cv::Mat *rgbMat = new cv::Mat();
      cv::cvtColor(rawMat, *rgbMat, cv::COLOR_RGBA2GRAY);
      return reinterpret_cast<jlong>(rgbMat);
    }
    return 0;
  }

  // Clone the viz image and apply overlays
  cv::Mat *displayMat = new cv::Mat(viz_img.clone());

  // Apply overlays (framerate, recording status, state info)
  std::string framerate_str = std::to_string((int)(viz_track_rate)) + "hz";
  cv::Point point0(displayMat->cols - 150, 30);
  cv::putText(*displayMat, framerate_str, point0, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 255, 0), 2);
  std::string recording_str = (is_recording) ? "recording" : "waiting";
  cv::Point point1(displayMat->cols - 150, 60);
  cv::putText(*displayMat, recording_str, point1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 255, 0), 2);

  // Show the current state estimate if we are estimating!
  // Use smaller font and thinner outline to fit more lines
  const double font_scale = 0.6;
  const int font_thickness = 1;
  const int line_spacing = 18; // Spacing between lines in pixels

  if (local_sys != nullptr && !viz_state1.empty()) {
    int y_start = displayMat->rows - (line_spacing * 3); // Start 3 lines from bottom
    cv::Point point1(10, y_start);
    cv::putText(*displayMat, viz_state1, point1, cv::FONT_HERSHEY_COMPLEX_SMALL, font_scale, cv::Scalar(255, 0, 0), font_thickness);

    if (!viz_state2.empty()) {
      cv::Point point2(10, y_start + line_spacing);
      cv::putText(*displayMat, viz_state2, point2, cv::FONT_HERSHEY_COMPLEX_SMALL, font_scale, cv::Scalar(255, 0, 0), font_thickness);
    }

    if (!viz_state3.empty()) {
      cv::Point point3(10, y_start + line_spacing * 2);
      cv::putText(*displayMat, viz_state3, point3, cv::FONT_HERSHEY_COMPLEX_SMALL, font_scale, cv::Scalar(255, 0, 0), font_thickness);
    }
  }

  return reinterpret_cast<jlong>(displayMat);
}

// Delete a Mat object that was allocated with new (to prevent memory leaks)
extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_deleteMatJNI(JNIEnv *env, jobject clazz, jlong matAddr) {
  if (matAddr != 0) {
    cv::Mat *mat = reinterpret_cast<cv::Mat *>(matAddr);
    delete mat;
  }
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_processImageJNI(JNIEnv *env, jobject instance, jlong matAddr,
                                                                                      jdouble timestampSec) {

  // Use the hardware timestamp from Camera2 (nanoseconds since boot, converted to seconds)
  // This ensures consistent frame-to-frame timing and matches IMU timestamp reference
  double time_in_sec = timestampSec;
  unsigned long long time_in_ns = (unsigned long long)(time_in_sec * 1e9);

  // get Mat from raw address
  clock_t begin = clock();
  cv::Mat &mat = *(cv::Mat *)matAddr;

  // Convert to gray scale (Mat is RGBA from YUV conversion)
  cv::Mat mat_gray;
  cv::cvtColor(mat, mat_gray, cv::COLOR_RGBA2GRAY);

  // If recording save to disk 取消保存图片
  //  if (is_recording) {
  //    std::string filename = save_folder + "cam0/" + std::to_string(time_in_ns) + ".png";
  //    cv::imwrite(filename, mat_gray);
  //    __android_log_print(ANDROID_LOG_INFO, TAG, "saved file: %s\n", filename.c_str());
  //  }

  // Return if the app record folder has not been set yet
  if (!app_record_folder_set) {
    return;
  }

  // Construct our tracker object if needed
  std::shared_ptr<ov_msckf::VioManager> local_sys;
  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    local_sys = sys;
  }

  if (is_running_ov && local_sys == nullptr) {

    // Log level
    ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::ALL);

    // Load the config
    // Use app_private_folder (private external files directory root) for config files
    // Add /config/ subdirectory in native code
    // Normalize path: remove trailing slash from app_private_folder if present
    std::string normalized_app_private_folder = app_private_folder;
    if (!normalized_app_private_folder.empty() && normalized_app_private_folder.back() == '/') {
      normalized_app_private_folder.pop_back();
    }
    std::string config_path = normalized_app_private_folder + "/config/estimator_config.yaml";

    // Debug: Check if file exists and is accessible
    struct stat st;
    if (stat(config_path.c_str(), &st) == 0) {
      __android_log_print(ANDROID_LOG_INFO, TAG, "Config file exists: %s (size: %ld bytes, mode: %o)\n", config_path.c_str(), st.st_size,
                          st.st_mode);
    } else {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "Config file does not exist or is not accessible: %s (errno: %d)\n", config_path.c_str(),
                          errno);
    }

    auto parser = std::make_shared<ov_core::YamlParser>(config_path, true);
    ov_msckf::VioManagerOptions params;
    params.print_and_load(parser);

    // Hardcoded parameters
    params.use_multi_threading_subs = true;
    params.num_opencv_threads = 1;            // phones suck
    params.use_aruco = false;                 // no extra opencv lib
    params.init_options.init_dyn_use = false; // no ceres solver lib

    // Timing stats
    // TODO: In the future it would be great to always record this info the dataset folder..
    params.record_timing_information = false;
    params.record_timing_filepath = "ov_msckf_timing.txt";

    //=====================================================
    // Camera settings
    //=====================================================

    // Ensure we read in all parameters required, create the VIO manager
    if (!parser->successful()) {
      // PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
      __android_log_print(ANDROID_LOG_ERROR, TAG, "unable to parse all parameters, please fix!!!");
      return;
    } else {
      local_sys = std::make_shared<ov_msckf::VioManager>(params);
      std::lock_guard<std::mutex> sys_lck(sys_mtx);
      sys = local_sys;
    }
  }

  // Try to process the image, check if we should drop this image
  // We will append this image to the queue if we need to
  if (local_sys != nullptr && is_running_ov) {

    // See if the message should be dropped / skipped
    int cam_id0 = 0;
    double time_delta = 1.0 / local_sys->get_params().track_frequency;
    bool should_queue = false;
    double frame_delta = 0.0;
    double expected_frame_rate = 0.0;
    {
      std::lock_guard<std::mutex> lck(camera_queue_mtx);
      auto last_it = camera_last_timestamp.find(cam_id0);
      should_queue = last_it == camera_last_timestamp.end() || time_in_sec > last_it->second + time_delta;
      if (last_it != camera_last_timestamp.end()) {
        frame_delta = time_in_sec - last_it->second;
        if (frame_delta > 0.0) {
          expected_frame_rate = 1.0 / frame_delta;
        }
      }
      if (should_queue) {
        camera_last_timestamp[cam_id0] = time_in_sec;
      }
    }

    if (!should_queue && frame_delta > 0.0) {
      // Frame dropped due to throttling (too soon after last frame)
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "Frame DROPPED: delta=%.4f sec (%.1f Hz) < threshold=%.4f sec\n", frame_delta,
                          expected_frame_rate, time_delta);
    }

    if (should_queue) {

      // Create the measurement
      ov_core::CameraData message;
      message.timestamp = time_in_sec;
      message.sensor_ids.push_back(cam_id0);
      message.images.push_back(mat_gray.clone());
      message.masks.push_back(cv::Mat::zeros(mat_gray.rows, mat_gray.cols, CV_8UC1));

      // Append it to our queue of images
      {
        std::lock_guard<std::mutex> lck(camera_queue_mtx);
        camera_queue.push_back(message);
        std::sort(camera_queue.begin(), camera_queue.end());
        __android_log_print(ANDROID_LOG_INFO, TAG, "SIZE CAMERA QUEUE = %zu", camera_queue.size());
        if (frame_delta > 0.0) {
          __android_log_print(ANDROID_LOG_INFO, TAG, " | frame_delta=%.4f sec (%.1f Hz)", frame_delta, expected_frame_rate);
        }
        __android_log_print(ANDROID_LOG_INFO, TAG, "\n");
      }

      // Notify the worker thread that new camera data is available
      processing_cv.notify_one();
    }
  }

  // Apply our transformations
  // cv::adaptiveThreshold(mat, mat, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 21, 5);
  // std::vector<cv::KeyPoint> pts_new;
  // cv::FAST(mat_gray, pts_new, 15.0, true);
  // for(const auto &pt : pts_new) {
  //    cv::circle(mat, pt.pt, 1, cv::Scalar(255,0,0), cv::FILLED);
  //}

  // log computation time to Android Logcat
  // double totalTime = double(clock() - begin) / CLOCKS_PER_SEC;
  //__android_log_print(ANDROID_LOG_INFO, TAG, "adaptiveThreshold computation time = %f seconds (%f hz)\n", totalTime, 1.0/totalTime);
  //__android_log_print(ANDROID_LOG_INFO, TAG, "adaptiveThreshold matrix size %d x %d\n", mat.rows, mat.cols);

  //================================================================
  //================================================================
  //================================================================

  // Update visualization image cache (used by getDisplayImageJNI for overlays)
  // This is separate from the actual processing - just updating the cache
    if (viz_time == -1 || (time_in_sec - viz_time) > 1.0 / viz_rate) {
    cv::Mat temp_img;
    if (local_sys != nullptr) {
      temp_img = local_sys->get_historical_viz_image();
    }
    if (!temp_img.empty()) {
      viz_image = temp_img.clone();
      viz_time = time_in_sec;
    }
  }
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_processInertialJNI(JNIEnv *env, jobject instance, jfloat ax,
                                                                                         jfloat ay, jfloat az, jfloat gx, jfloat gy,
                                                                                         jfloat gz, jdouble timestampSec) {

  // Use the hardware timestamp from SensorEvent (nanoseconds since boot, converted to seconds)
  // This ensures consistent timing and matches camera timestamp reference
  double time_in_sec = timestampSec;
  unsigned long long time_in_ns = (unsigned long long)(time_in_sec * 1e9);

  // Cast to our native type
  double n_ax = static_cast<double>(ax);
  double n_ay = static_cast<double>(ay);
  double n_az = static_cast<double>(az);
  double n_gx = static_cast<double>(gx);
  double n_gy = static_cast<double>(gy);
  double n_gz = static_cast<double>(gz);

  bool imu_sample_valid = true;
  {
    std::lock_guard<std::mutex> filter_lck(imu_filter_mtx);
    const double accel_norm = std::sqrt(n_ax * n_ax + n_ay * n_ay + n_az * n_az);
    const double gyro_norm = std::sqrt(n_gx * n_gx + n_gy * n_gy + n_gz * n_gz);
    const bool finite_sample = std::isfinite(time_in_sec) && std::isfinite(accel_norm) && std::isfinite(gyro_norm);
    const double dt = imu_filter_initialized ? time_in_sec - last_valid_imu_timestamp : 0.0;

    bool hard_spike = !finite_sample || accel_norm > MAX_ACCEL_NORM || gyro_norm > MAX_GYRO_NORM;
    bool jump_spike = false;
    if (imu_filter_initialized && dt > 0.0 && dt < IMU_FILTER_RESET_DT) {
      const double dax = n_ax - last_valid_ax;
      const double day = n_ay - last_valid_ay;
      const double daz = n_az - last_valid_az;
      const double dgx = n_gx - last_valid_gx;
      const double dgy = n_gy - last_valid_gy;
      const double dgz = n_gz - last_valid_gz;
      const double accel_jump = std::sqrt(dax * dax + day * day + daz * daz);
      const double gyro_jump = std::sqrt(dgx * dgx + dgy * dgy + dgz * dgz);
      jump_spike = accel_jump > MAX_ACCEL_JUMP || gyro_jump > MAX_GYRO_JUMP;
    }

    if (hard_spike || jump_spike) {
      if (imu_filter_initialized && dt > 0.0 && dt < IMU_FILTER_RESET_DT) {
        __android_log_print(ANDROID_LOG_WARN, TAG,
                            "Replacing IMU spike at %.9f: acc_norm=%.3f gyro_norm=%.3f hard=%d jump=%d\n", time_in_sec, accel_norm,
                            gyro_norm, hard_spike ? 1 : 0, jump_spike ? 1 : 0);
        n_ax = last_valid_ax;
        n_ay = last_valid_ay;
        n_az = last_valid_az;
        n_gx = last_valid_gx;
        n_gy = last_valid_gy;
        n_gz = last_valid_gz;
        last_valid_imu_timestamp = time_in_sec;
      } else {
        __android_log_print(ANDROID_LOG_WARN, TAG,
                            "Dropping IMU spike at %.9f: acc_norm=%.3f gyro_norm=%.3f hard=%d jump=%d\n", time_in_sec, accel_norm,
                            gyro_norm, hard_spike ? 1 : 0, jump_spike ? 1 : 0);
        imu_sample_valid = false;
      }
    } else {
      if (!imu_filter_initialized || dt <= 0.0 || dt >= IMU_FILTER_RESET_DT) {
        imu_filter_initialized = true;
      }
      last_valid_imu_timestamp = time_in_sec;
      last_valid_ax = n_ax;
      last_valid_ay = n_ay;
      last_valid_az = n_az;
      last_valid_gx = n_gx;
      last_valid_gy = n_gy;
      last_valid_gz = n_gz;
    }
  }

  if (!imu_sample_valid) {
    return;
  }

  // If recording save to disk
  if (is_recording && imu_csv.is_open()) {
    imu_csv << time_in_ns << "," << n_gx << "," << n_gy << "," << n_gz << "," << n_ax << "," << n_ay << "," << n_az << std::endl;
    //    __android_log_print(ANDROID_LOG_INFO, TAG, "%.4f, %.4f, %.4f | %.4f, %.4f, %.4f \n", n_ax, n_ay, n_az, n_gx, n_gy, n_gz);
  }

  std::shared_ptr<ov_msckf::VioManager> local_sys;
  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    local_sys = sys;
  }

  if (is_running_ov) {
    std::lock_guard<std::mutex> imu_lck(imu_timestamp_mtx);
    if (time_in_sec <= last_accepted_imu_timestamp) {
      __android_log_print(ANDROID_LOG_WARN, TAG, "Skipping non-monotonic IMU timestamp: %.9f <= %.9f\n", time_in_sec,
                          last_accepted_imu_timestamp);
      return;
    }
    last_accepted_imu_timestamp = time_in_sec;
    latest_imu_timestamp = time_in_sec;
    accepted_imu_count++;
  }

  // Feed if the system is running!
  if (local_sys != nullptr) {
    // Send it into the system
    ov_core::ImuData message_imu;
    message_imu.timestamp = time_in_sec;
    message_imu.wm << n_gx, n_gy, n_gz;
    message_imu.am << n_ax, n_ay, n_az;
    try {
      local_sys->feed_measurement_imu(message_imu);
    } catch (const std::exception &e) {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "feed_measurement_imu exception: %s\n", e.what());
    } catch (...) {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "feed_measurement_imu unknown exception\n");
    }
  }
}

// JNI functions for trajectory visualization
extern "C" JNIEXPORT jboolean JNICALL Java_com_openvins_android_VioEngine_getCurrentPoseJNI(JNIEnv *env, jobject instance,
                                                                                            jdoubleArray position,
                                                                                            jdoubleArray quaternion) {
  std::shared_ptr<ov_msckf::VioManager> local_sys;
  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    local_sys = sys;
  }

  if (local_sys == nullptr || !is_running_ov) {
    return JNI_FALSE;
  }

  auto state = local_sys->get_state();
  if (state == nullptr) {
    return JNI_FALSE;
  }

  auto q_GtoI = state->_imu->quat();
  auto p_IinG = state->_imu->pos();

  // Transform IMU pose to camera pose using calibration
  assert(state->_calib_IMUtoCAM.find(0) != state->_calib_IMUtoCAM.end() && "Camera calibration for camera 0 must exist");
  auto calib = state->_calib_IMUtoCAM.at(0);
  Eigen::Vector4d q_ItoC = calib->quat();
  Eigen::Vector3d p_IinC = calib->pos();

  // Compose rotations: q_GtoC = q_ItoC * q_GtoI
  Eigen::Vector4d q_cam = ov_core::quat_multiply(q_ItoC, q_GtoI);

  // Transform position: p_CinG = p_IinG - R_GtoC * p_IinC
  // where R_GtoC = quat_2_Rot(q_GtoC).transpose()
  Eigen::Vector3d p_cam = p_IinG - ov_core::quat_2_Rot(q_cam).transpose() * p_IinC;
  {
    std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
    if (trajectory_alignment_active) {
      p_cam = p_cam + trajectory_alignment_offset;
    } else if (trajectory_alignment_pending && !trajectory_history.empty()) {
      const auto &last = trajectory_history.back();
      p_cam = Eigen::Vector3d(last.x, last.y, last.z);
    }
  }

  jdouble pos[3] = {p_cam(0), p_cam(1), p_cam(2)};
  // Convert JPL [qx, qy, qz, qw] to Hamilton [qw, qx, qy, qz] for Java
  jdouble quat[4] = {q_cam(3), q_cam(0), q_cam(1), q_cam(2)};

  if (is_recording && pose_ext_csv.is_open()) {
    unsigned long long time_in_ns = (unsigned long long)(state->_timestamp * 1e9);
    pose_ext_csv << time_in_ns << "," << p_cam(0) << "," << p_cam(1) << "," << p_cam(2) << "," << q_cam(0) << "," << q_cam(1) << ","
                 << q_cam(2) << "," << q_cam(3) << std::endl;
  }

  env->SetDoubleArrayRegion(position, 0, 3, pos);
  env->SetDoubleArrayRegion(quaternion, 0, 4, quat);

  return JNI_TRUE;
}

extern "C" JNIEXPORT jint JNICALL Java_com_openvins_android_VioEngine_getTrajectoryDataJNI(JNIEnv *env, jobject instance,
                                                                                           jdoubleArray positions,
                                                                                           jdoubleArray quaternions) {
  std::lock_guard<std::mutex> lck(trajectory_mtx);

  size_t size = trajectory_history.size();
  if (size == 0) {
    return 0;
  }

  // Check array sizes
  jsize pos_size = env->GetArrayLength(positions);
  jsize quat_size = env->GetArrayLength(quaternions);

  jsize required_pos_size = static_cast<jsize>(size * 3);
  jsize required_quat_size = static_cast<jsize>(size * 4);

  if (pos_size < required_pos_size || quat_size < required_quat_size) {
    // Arrays too small - return 0 to indicate error
    // Java side should allocate larger arrays
    return 0;
  }

  jdouble *pos_array = env->GetDoubleArrayElements(positions, nullptr);
  jdouble *quat_array = env->GetDoubleArrayElements(quaternions, nullptr);

  if (pos_array == nullptr || quat_array == nullptr) {
    return 0;
  }

  // Copy trajectory data
  for (size_t i = 0; i < size; i++) {
    pos_array[i * 3 + 0] = trajectory_history[i].x;
    pos_array[i * 3 + 1] = trajectory_history[i].y;
    pos_array[i * 3 + 2] = trajectory_history[i].z;

    quat_array[i * 4 + 0] = trajectory_history[i].qw;
    quat_array[i * 4 + 1] = trajectory_history[i].qx;
    quat_array[i * 4 + 2] = trajectory_history[i].qy;
    quat_array[i * 4 + 3] = trajectory_history[i].qz;
  }

  env->ReleaseDoubleArrayElements(positions, pos_array, 0);
  env->ReleaseDoubleArrayElements(quaternions, quat_array, 0);

  return static_cast<jint>(size);
}

extern "C" JNIEXPORT jboolean JNICALL Java_com_openvins_android_VioEngine_isTrajectoryPausedJNI(JNIEnv *env, jobject instance) {
  std::lock_guard<std::mutex> lck(trajectory_mtx);
  return trajectory_data_paused ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jint JNICALL Java_com_openvins_android_VioEngine_getTrajectoryPauseReasonJNI(JNIEnv *env, jobject instance) {
  std::lock_guard<std::mutex> lck(trajectory_mtx);
  return trajectory_pause_reason;
}

extern "C" JNIEXPORT void JNICALL Java_com_openvins_android_VioEngine_resumeTrajectoryJNI(JNIEnv *env, jobject instance) {
  {
    std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
    trajectory_data_paused = false;
    trajectory_pause_prompt_pending = false;
    trajectory_pause_reason = TRAJECTORY_REJECT_NONE;
    trajectory_drift_reject_streak = 0;
    trajectory_debug_last_timestamp = -1.0;
    trajectory_turn_guard_until_timestamp = -1.0;
    trajectory_debug_has_last_pose = false;
    trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

    if (!trajectory_history.empty()) {
      const auto &last = trajectory_history.back();
      trajectory_alignment_anchor = Eigen::Vector3d(last.x, last.y, last.z);
      trajectory_alignment_pending = true;
      trajectory_alignment_active = false;
      trajectory_alignment_offset = Eigen::Vector3d::Zero();
    } else {
      trajectory_alignment_pending = false;
      trajectory_alignment_active = false;
      trajectory_alignment_anchor = Eigen::Vector3d::Zero();
      trajectory_alignment_offset = Eigen::Vector3d::Zero();
    }
  }

  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    sys = nullptr;
  }
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);
    camera_queue.clear();
    camera_last_timestamp.clear();
  }
  viz_time = -1;
  viz_track_rate = 0.0;
  viz_track_last_time = -1.0;
  viz_state1 = "";
  viz_state2 = "";
  viz_state3 = "";

  __android_log_print(ANDROID_LOG_INFO, TAG, "Trajectory resumed with VIO reinitialization\n");
}
