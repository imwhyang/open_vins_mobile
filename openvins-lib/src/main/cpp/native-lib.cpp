#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <condition_variable>
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
std::ofstream camera_quality_debug_csv;
std::mutex pose_ext_csv_mtx;
std::mutex camera_quality_debug_csv_mtx;

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
std::atomic<double> latest_imu_pair_delta_seconds(0.0);
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

struct ImuMotionSample {
  double timestamp;
  double accel_norm;
  double gyro_norm;
};

// 使用短时间窗口描述手机是否静止。第一阶段只记录结果并辅助分析，
// 不直接改写 OpenVINS 的原始 IMU 数据，避免影响正常转弯和缓慢移动。
std::deque<ImuMotionSample> imu_motion_window;
std::atomic<double> latest_imu_accel_norm(0.0);
std::atomic<double> latest_imu_gyro_norm(0.0);
std::atomic<double> latest_imu_accel_stddev(0.0);
std::atomic<double> latest_imu_gyro_mean(0.0);
std::atomic<bool> latest_imu_static(false);
const double IMU_MOTION_WINDOW_SECONDS = 0.5;
const double IMU_STATIC_MIN_WINDOW_SECONDS = 0.3;
const double IMU_STATIC_ACCEL_GRAVITY_TOLERANCE = 0.35;
const double IMU_STATIC_ACCEL_STDDEV_MAX = 0.12;
const double IMU_STATIC_GYRO_MEAN_MAX = 0.08;

// Queue up camera measurements sorted by time and trigger once we have
// exactly one IMU measurement with timestamp newer than the camera measurement
// This also handles out-of-order camera measurements, which is rare, but
// a nice feature to have for general robustness to bad camera drivers.
std::deque<ov_core::CameraData> camera_queue;
std::mutex camera_queue_mtx;
std::atomic<double> latest_camera_processing_age_seconds(0.0);
std::atomic<double> latest_camera_imu_lead_seconds(0.0);
std::atomic<size_t> latest_camera_queue_after_pop(0);

// Last camera message timestamps we have received (mapped by cam id)
std::map<int, double> camera_last_timestamp;

// Maximum age (in seconds) for camera measurements before skipping
const double MAX_CAMERA_AGE_SECONDS = 0.5; // Skip measurements older than 500ms

// 只拦截高置信度的全黑/近全黑画面，普通暗场景先保留并通过日志继续评估。
// 阈值故意设置得较保守，避免猪圈光线偏暗时误判为摄像头遮挡。
const double CAMERA_BLOCKED_MEAN_MAX = 10.0;
const double CAMERA_BLOCKED_STDDEV_MAX = 10.0;
const double CAMERA_BLOCKED_DARK_RATIO_MIN = 0.95;
const int CAMERA_DARK_PIXEL_THRESHOLD = 18;
// 实测手掌遮挡不一定是全黑，但拉普拉斯分数会长期接近 0。
// 快速晃动则只有在角速度较大且清晰度明显下降时才过滤，避免影响正常转弯。
const double CAMERA_UNUSABLE_BLUR_MAX = 20.0;
const double CAMERA_MOTION_BLUR_MAX = 150.0;
const double CAMERA_MOTION_BLUR_GYRO_MIN = 2.0;
const size_t VISUAL_INTERRUPTION_CONFIRM_FRAMES = 3;
const size_t VISUAL_INTERRUPTION_MOTION_CONFIRM_FRAMES = 5;
const double VISUAL_INTERRUPTION_MOTION_GRACE_SECONDS = 0.7;

// 视觉中断确认后，旧 VIO 会在画面恢复时被丢弃，避免十几秒纯 IMU 积分
// 在恢复第一帧一次性产生几十米跳变。
std::mutex visual_interruption_mtx;
size_t visual_blocked_consecutive_frames = 0;
size_t visual_clear_consecutive_frames = 0;
size_t visual_motion_consecutive_frames = 0;
bool visual_interruption_active = false;
bool visual_interruption_had_motion = false;
double visual_interruption_start_timestamp = -1.0;
// 0=正常，1=当前画面质量过低，图像帧未参与定位。
std::atomic<int> visual_recovery_user_state(0);
// 漂移后继续流程单独维护状态，避免摄像头遮挡状态将初始化提示覆盖。
// 0=正常，2=VIO 已初始化并正在等待轨迹稳定，3=正在重新初始化 VIO。
std::atomic<int> trajectory_recovery_user_state(0);
// 画面质量过滤期间轨迹不会更新。恢复后的首个有效帧需要重新接到最后可靠点，
// 否则过滤造成的轨迹缺口会被误认为持续漂移并触发重新 INIT。
std::atomic<bool> camera_filter_recovery_pending(false);
// 仅用于界面选择实时预览，不参与定位判断。过滤帧期间不能继续显示历史处理帧，
// 否则用户看到的画面会像应用卡死一样停在原地。
std::atomic<bool> camera_frame_filtered_for_display(false);

void reset_visual_interruption_state() {
  std::lock_guard<std::mutex> lck(visual_interruption_mtx);
  visual_blocked_consecutive_frames = 0;
  visual_clear_consecutive_frames = 0;
  visual_motion_consecutive_frames = 0;
  visual_interruption_active = false;
  visual_interruption_had_motion = false;
  visual_interruption_start_timestamp = -1.0;
  camera_filter_recovery_pending.store(false);
  camera_frame_filtered_for_display.store(false);
  visual_recovery_user_state.store(0);
  trajectory_recovery_user_state.store(0);
}

void update_imu_motion_state(double timestamp, double accel_norm, double gyro_norm) {
  imu_motion_window.push_back({timestamp, accel_norm, gyro_norm});
  const double cutoff = timestamp - IMU_MOTION_WINDOW_SECONDS;
  while (!imu_motion_window.empty() && imu_motion_window.front().timestamp < cutoff) {
    imu_motion_window.pop_front();
  }

  latest_imu_accel_norm.store(accel_norm);
  latest_imu_gyro_norm.store(gyro_norm);
  if (imu_motion_window.empty()) {
    latest_imu_accel_stddev.store(0.0);
    latest_imu_gyro_mean.store(0.0);
    latest_imu_static.store(false);
    return;
  }

  double accel_sum = 0.0;
  double gyro_sum = 0.0;
  for (const auto &sample : imu_motion_window) {
    accel_sum += sample.accel_norm;
    gyro_sum += sample.gyro_norm;
  }
  const double count = static_cast<double>(imu_motion_window.size());
  const double accel_mean = accel_sum / count;
  const double gyro_mean = gyro_sum / count;
  double accel_variance = 0.0;
  for (const auto &sample : imu_motion_window) {
    const double delta = sample.accel_norm - accel_mean;
    accel_variance += delta * delta;
  }
  const double accel_stddev = std::sqrt(accel_variance / count);
  const double window_duration = timestamp - imu_motion_window.front().timestamp;
  const bool is_static = window_duration >= IMU_STATIC_MIN_WINDOW_SECONDS &&
                         std::abs(accel_mean - 9.81) <= IMU_STATIC_ACCEL_GRAVITY_TOLERANCE &&
                         accel_stddev <= IMU_STATIC_ACCEL_STDDEV_MAX && gyro_mean <= IMU_STATIC_GYRO_MEAN_MAX;
  latest_imu_accel_stddev.store(accel_stddev);
  latest_imu_gyro_mean.store(gyro_mean);
  latest_imu_static.store(is_static);
}

void reset_imu_motion_state() {
  imu_motion_window.clear();
  latest_imu_accel_norm.store(0.0);
  latest_imu_gyro_norm.store(0.0);
  latest_imu_accel_stddev.store(0.0);
  latest_imu_gyro_mean.store(0.0);
  latest_imu_static.store(false);
}

struct CameraQuality {
  double mean = 0.0;
  double stddev = 0.0;
  double dark_ratio = 0.0;
  double blur_score = 0.0;
  bool unusable_texture = false;
  bool blocked = false;
};

CameraQuality evaluate_camera_quality(const cv::Mat &gray) {
  CameraQuality quality;
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(gray, mean, stddev);
  quality.mean = mean[0];
  quality.stddev = stddev[0];

  cv::Mat dark_mask;
  cv::compare(gray, CAMERA_DARK_PIXEL_THRESHOLD, dark_mask, cv::CMP_LT);
  quality.dark_ratio = static_cast<double>(cv::countNonZero(dark_mask)) / static_cast<double>(gray.total());

  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);
  cv::Scalar laplacian_mean;
  cv::Scalar laplacian_stddev;
  cv::meanStdDev(laplacian, laplacian_mean, laplacian_stddev);
  quality.blur_score = laplacian_stddev[0] * laplacian_stddev[0];
  quality.unusable_texture = quality.blur_score <= CAMERA_UNUSABLE_BLUR_MAX;
  quality.blocked = quality.dark_ratio >= CAMERA_BLOCKED_DARK_RATIO_MIN ||
                    (quality.mean <= CAMERA_BLOCKED_MEAN_MAX && quality.stddev <= CAMERA_BLOCKED_STDDEV_MAX);
  return quality;
}

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
const size_t TRAJECTORY_GATE_MIN_FEATURES = 15;
const size_t TRAJECTORY_GATE_LOW_FEATURES = 40;
const size_t TRAJECTORY_GATE_ROTATION_FEATURES = 40;
const double TRAJECTORY_GATE_MAX_STEP_METERS = 0.30;
// 当前正常步行与转弯样本最大瞬时速度约 3.9m/s，5m/s 仍保留设备抖动余量，
// 同时可以拦截本次日志中 6.9~7.7m/s 的残留漂移点。
const double TRAJECTORY_GATE_MAX_SPEED_MPS = 5.0;
const double TRAJECTORY_GATE_LOW_FEATURE_STEP_METERS = 0.15;
const double TRAJECTORY_GATE_ROTATION_STEP_METERS = 0.18;
const double TRAJECTORY_GATE_ROTATION_RAD = 0.52; // 30 deg
// 业务操作中手机朝向与行走方向基本一致。负值表示估计位移明显落在摄像头后方；
// 阈值保留转弯和轻微侧移余量，不要求轨迹必须严格沿光轴。
const double TRAJECTORY_GATE_BACKWARD_PROJECTION = -0.35;
const double TRAJECTORY_GATE_BACKWARD_MIN_STEP_METERS = 0.03;
const double TRAJECTORY_GATE_TURN_BACKWARD_MIN_STEP_METERS = 0.08;
const double TRAJECTORY_TURN_GUARD_ROT_RAD = 0.08;
const double TRAJECTORY_TURN_GUARD_MAX_STEP_METERS = 0.08;
const double TRAJECTORY_TURN_GUARD_SECONDS = 1.00;
const double TRAJECTORY_TURN_GUARD_BACKWARD_PROJECTION = -0.25;
const double TRAJECTORY_TURN_GUARD_MIN_STEP_METERS = 0.03;
const double TRAJECTORY_TURN_GUARD_FAST_SPEED_MPS = 1.5;
const double TRAJECTORY_TURN_GUARD_FAST_STEP_METERS = 0.05;
const double TRAJECTORY_TURN_GUARD_LOW_FEATURE_STEP_METERS = 0.12;
const size_t TRAJECTORY_TURN_GUARD_LOW_FEATURES = 80;
const double TRAJECTORY_SHIFT_WINDOW_SECONDS = 0.5;
const double TRAJECTORY_SHIFT_MAX_MILEAGE_METERS = 1.0;
const double TRAJECTORY_MAX_CONNECT_STEP_METERS = 0.45;
const double TRAJECTORY_TURN_IN_PLACE_ROT_RAD = 0.12;
const double TRAJECTORY_TURN_IN_PLACE_MAX_STEP_METERS = 0.08;
const double TRAJECTORY_TURN_PROTECT_ROT_RAD = 0.04;
const double TRAJECTORY_TURN_PROTECT_SECONDS = 0.8;
const double TRAJECTORY_TURN_PROTECT_MAX_END_DISTANCE = 0.18;
const double TRAJECTORY_STRONG_TURN_ROT_RAD = 0.10;
const double TRAJECTORY_STRONG_TURN_PROTECT_SECONDS = 0.45;
const double TRAJECTORY_RESUME_STABLE_STEP_METERS = 0.12;
const size_t TRAJECTORY_RESUME_STABLE_FRAMES = 5;
const size_t TRAJECTORY_DRIFT_PAUSE_STREAK = 3;
const double TRAJECTORY_CONSISTENT_DRIFT_MIN_STEP_METERS = 0.12;
const double TRAJECTORY_CONSISTENT_DRIFT_DIRECTION_DOT = 0.85;
const double TRAJECTORY_CONSISTENT_DRIFT_STEP_RATIO = 0.45;
const double TRAJECTORY_AUTO_RECOVERY_OBSERVE_SECONDS = 1.0;
const double TRAJECTORY_AUTO_RECOVERY_MAX_WAIT_SECONDS = 2.5;
const double TRAJECTORY_SOFT_RECOVERY_COOLDOWN_SECONDS = 20.0;
const double TRAJECTORY_SOFT_RECOVERY_MAX_DISTANCE_METERS = 1.5;
const double TRAJECTORY_BACKGROUND_REINIT_WAIT_SECONDS = 3.0;
const double TRAJECTORY_BACKGROUND_REINIT_MIN_RANSAC_RATIO = 0.60;
const size_t TRAJECTORY_SOFT_RECOVERY_MIN_UPDATE_FRAMES = 5;
const size_t TRAJECTORY_SOFT_RECOVERY_MIN_UPDATE_FEATURES = 20;
const double TRAJECTORY_STATIC_VISUAL_CONFLICT_METERS = 0.015;
const double TRAJECTORY_STATIC_CONFLICT_RELEASE_GYRO_MEAN = 0.12;
const double TRAJECTORY_STATIC_CONFLICT_RELEASE_ACCEL_STDDEV = 0.18;
const size_t TRAJECTORY_STATIC_CONFLICT_RELEASE_FRAMES = 5;
const double TRAJECTORY_FILTERED_ORIENTATION_GYRO_MEAN_MIN = 0.08;
const size_t TRAJECTORY_RANSAC_MIN_CANDIDATES = 20;
const double TRAJECTORY_RANSAC_MIN_INLIER_RATIO = 0.30;
const double TRAJECTORY_RANSAC_RECOVERY_INLIER_RATIO = 0.45;
const size_t TRAJECTORY_RANSAC_BAD_CONFIRM_FRAMES = 3;
const size_t TRAJECTORY_RANSAC_GOOD_RECOVERY_FRAMES = 5;
const double TRAJECTORY_RANSAC_REJECT_MIN_STEP_METERS = 0.03;

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
  TRAJECTORY_REJECT_SHIFTING_WINDOW = 10,
  TRAJECTORY_REJECT_VISUAL_INTERRUPTION_MOVED = 11,
  TRAJECTORY_REJECT_STATIC_VISUAL_CONFLICT = 12,
  TRAJECTORY_REJECT_LOW_RANSAC_INLIERS = 13,
};
const double TRAJECTORY_RESUME_GRACE_SECONDS = 2.0;
double trajectory_debug_last_timestamp = -1.0;
double trajectory_turn_guard_until_timestamp = -1.0;
size_t trajectory_debug_session_start_size = 0;
bool trajectory_debug_has_last_pose = false;
TrajectoryPoint trajectory_debug_last_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
bool trajectory_data_paused = false;
bool trajectory_pause_prompt_pending = false;
int trajectory_pause_reason = TRAJECTORY_REJECT_NONE;
size_t trajectory_drift_reject_streak = 0;
bool trajectory_resume_grace_pending = false;
double trajectory_resume_grace_until_timestamp = -1.0;
bool trajectory_resume_waiting_stable = false;
size_t trajectory_resume_stable_count = 0;
double trajectory_turn_protect_until_timestamp = -1.0;
double trajectory_strong_turn_protect_until_timestamp = -1.0;
bool trajectory_has_last_drift_delta = false;
Eigen::Vector3d trajectory_last_drift_delta(0.0, 0.0, 0.0);
size_t trajectory_consistent_drift_count = 0;
bool trajectory_auto_recovery_active = false;
double trajectory_auto_recovery_start_timestamp = -1.0;
double trajectory_last_soft_recovery_timestamp = -1.0;
size_t trajectory_soft_recovery_count = 0;
size_t trajectory_auto_recovery_update_frames = 0;
size_t trajectory_auto_recovery_update_features = 0;
bool trajectory_static_visual_conflict_active = false;
size_t trajectory_static_conflict_motion_frames = 0;
bool trajectory_dynamic_visual_inconsistent = false;
size_t trajectory_ransac_bad_frames = 0;
size_t trajectory_ransac_good_frames = 0;

struct TrajectoryShiftSample {
  double timestamp;
  Eigen::Vector3d position;
  size_t history_size_after_append;
};
std::deque<TrajectoryShiftSample> trajectory_shift_window;
bool trajectory_alignment_pending = false;
bool trajectory_alignment_active = false;
Eigen::Vector3d trajectory_alignment_anchor(0.0, 0.0, 0.0);
Eigen::Vector3d trajectory_alignment_offset(0.0, 0.0, 0.0);
Eigen::Vector3d trajectory_alignment_source_position(0.0, 0.0, 0.0);
Eigen::Matrix3d trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
size_t trajectory_last_clean_size = 0;
bool trajectory_has_clean_anchor = false;
TrajectoryPoint trajectory_clean_anchor(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
bool trajectory_has_reliable_orientation = false;
Eigen::Vector4d trajectory_reliable_orientation(0.0, 0.0, 0.0, 1.0);

double trajectory_quaternion_angle(const TrajectoryPoint &last, double qw, double qx, double qy, double qz) {
  double dot = std::abs(last.qw * qw + last.qx * qx + last.qy * qy + last.qz * qz);
  dot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(dot);
}

bool get_last_trajectory_position(Eigen::Vector3d &position);

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

bool compute_trajectory_delta(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion, double &dt,
                              double &step, double &speed, double &rot, double &forward_projection) {
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

bool should_accept_trajectory_point(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion,
                                    size_t feature_count, bool dynamic_visual_inconsistent, int &reject_reason) {
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
  if (step > TRAJECTORY_RANSAC_REJECT_MIN_STEP_METERS && dynamic_visual_inconsistent) {
    // 多组独立运动持续存在时才过滤已有明显位移的点。单帧低内点率不会
    // 直接影响正常轨迹，避免普通转弯和短暂模糊产生缺口。
    reject_reason = TRAJECTORY_REJECT_LOW_RANSAC_INLIERS;
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
  const double backward_min_step =
      turn_guard_active ? TRAJECTORY_GATE_TURN_BACKWARD_MIN_STEP_METERS : TRAJECTORY_GATE_BACKWARD_MIN_STEP_METERS;
  if (step > backward_min_step && forward_projection < TRAJECTORY_GATE_BACKWARD_PROJECTION) {
    // 正常完成 180 度转身后，位置增量和新的相机前向仍应同向。
    // 这里只拦截方向明显相反且已有实际位移的点，原地转身抖动不会进入该条件。
    reject_reason = TRAJECTORY_REJECT_BACKWARD_AFTER_TURN;
    return false;
  }

  return true;
}

void update_dynamic_visual_consistency(size_t candidate_count, double inlier_ratio) {
  if (candidate_count < TRAJECTORY_RANSAC_MIN_CANDIDATES) {
    // 候选点不足无法可靠判断多运动模型，保持当前状态但不累计进入或退出帧数。
    trajectory_ransac_bad_frames = 0;
    trajectory_ransac_good_frames = 0;
    return;
  }

  if (inlier_ratio < TRAJECTORY_RANSAC_MIN_INLIER_RATIO) {
    trajectory_ransac_bad_frames++;
    trajectory_ransac_good_frames = 0;
    if (trajectory_ransac_bad_frames >= TRAJECTORY_RANSAC_BAD_CONFIRM_FRAMES) {
      trajectory_dynamic_visual_inconsistent = true;
    }
  } else if (inlier_ratio >= TRAJECTORY_RANSAC_RECOVERY_INLIER_RATIO) {
    trajectory_ransac_good_frames++;
    trajectory_ransac_bad_frames = 0;
    if (trajectory_ransac_good_frames >= TRAJECTORY_RANSAC_GOOD_RECOVERY_FRAMES) {
      trajectory_dynamic_visual_inconsistent = false;
    }
  } else {
    // 中间区间作为滞回带，不允许一次边界波动改变状态。
    trajectory_ransac_bad_frames = 0;
    trajectory_ransac_good_frames = 0;
  }
}

bool is_trajectory_drift_reason(int reject_reason) {
  return reject_reason == TRAJECTORY_REJECT_SHIFTING_WINDOW;
}

bool should_pause_for_trajectory_drift(int reject_reason) {
  // 弹窗只处理连续异常，避免正常转弯时的单帧/短暂假位移打断用户。
  (void)reject_reason;
  return trajectory_drift_reject_streak >= TRAJECTORY_DRIFT_PAUSE_STREAK;
}

double trajectory_shifting_mileage_with_candidate(double timestamp, const Eigen::Vector3d &position) {
  const double cutoff = timestamp - TRAJECTORY_SHIFT_WINDOW_SECONDS;
  bool has_last = false;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  double mileage = 0.0;

  for (const auto &sample : trajectory_shift_window) {
    if (sample.timestamp < cutoff) {
      continue;
    }
    if (has_last) {
      mileage += (sample.position - last_position).norm();
    }
    last_position = sample.position;
    has_last = true;
  }

  if (has_last) {
    mileage += (position - last_position).norm();
  }
  return mileage;
}

bool is_trajectory_shifting(double timestamp, const Eigen::Vector3d &position) {
  // 与 Kotlin 里的 shiftingTrajectory 保持一致：只看最近 0.5 秒的累计里程，
  // 不再用单帧速度、转弯方向等条件触发弹窗，避免转身时过于敏感。
  return trajectory_shifting_mileage_with_candidate(timestamp, position) > TRAJECTORY_SHIFT_MAX_MILEAGE_METERS;
}

void reset_consistent_drift_detector() {
  trajectory_has_last_drift_delta = false;
  trajectory_last_drift_delta = Eigen::Vector3d::Zero();
  trajectory_consistent_drift_count = 0;
}

void reset_trajectory_auto_recovery() {
  trajectory_auto_recovery_active = false;
  trajectory_auto_recovery_start_timestamp = -1.0;
  trajectory_auto_recovery_update_frames = 0;
  trajectory_auto_recovery_update_features = 0;
}

bool update_consistent_drift_detector(const Eigen::Vector3d &position) {
  Eigen::Vector3d last_position;
  if (!get_last_trajectory_position(last_position)) {
    reset_consistent_drift_detector();
    return false;
  }

  Eigen::Vector3d delta = position - last_position;
  double step = delta.norm();
  if (step < TRAJECTORY_CONSISTENT_DRIFT_MIN_STEP_METERS) {
    reset_consistent_drift_detector();
    return false;
  }

  if (!trajectory_has_last_drift_delta) {
    trajectory_last_drift_delta = delta;
    trajectory_has_last_drift_delta = true;
    trajectory_consistent_drift_count = 1;
    return false;
  }

  double last_step = trajectory_last_drift_delta.norm();
  double direction_dot = delta.normalized().dot(trajectory_last_drift_delta.normalized());
  double step_ratio = std::abs(step - last_step) / std::max(step, last_step);
  if (direction_dot >= TRAJECTORY_CONSISTENT_DRIFT_DIRECTION_DOT && step_ratio <= TRAJECTORY_CONSISTENT_DRIFT_STEP_RATIO) {
    trajectory_consistent_drift_count++;
  } else {
    trajectory_consistent_drift_count = 1;
  }

  trajectory_last_drift_delta = delta;
  // 真漂移通常是连续几帧同方向、同量级地偏；转身抖动方向杂乱，不应触发弹窗。
  return trajectory_consistent_drift_count >= TRAJECTORY_DRIFT_PAUSE_STREAK;
}

void remember_trajectory_shift_sample(double timestamp, const Eigen::Vector3d &position, size_t history_size_after_append) {
  // 只把已经接受并绘制的可靠点放入窗口，避免被拒绝的漂移点污染后续判断。
  const double cutoff = timestamp - TRAJECTORY_SHIFT_WINDOW_SECONDS;
  while (!trajectory_shift_window.empty() && trajectory_shift_window.front().timestamp < cutoff) {
    trajectory_shift_window.pop_front();
  }
  trajectory_shift_window.push_back({timestamp, position, history_size_after_append});
}

size_t rollback_size_for_shifting_window(double timestamp) {
  const double cutoff = timestamp - TRAJECTORY_SHIFT_WINDOW_SECONDS;
  for (const auto &sample : trajectory_shift_window) {
    if (sample.timestamp >= cutoff) {
      // 第一个落入漂移窗口的点也可能已经属于异常段，所以回退到它之前。
      return sample.history_size_after_append > 0 ? sample.history_size_after_append - 1 : 0;
    }
  }
  return trajectory_history.size();
}

void rollback_trajectory_after_shifting(double timestamp) {
  // shiftingTrajectory 是窗口判断，触发时窗口内前几帧可能已经画出去了。
  // 这里把轨迹回退到窗口开始前，保证继续时接在最后一个更可靠的点上。
  size_t keep_size = rollback_size_for_shifting_window(timestamp);
  if (keep_size < trajectory_history.size()) {
    trajectory_history.erase(trajectory_history.begin() + keep_size, trajectory_history.end());
  }

  trajectory_shift_window.clear();
  if (!trajectory_history.empty()) {
    trajectory_has_clean_anchor = true;
    trajectory_last_clean_size = trajectory_history.size();
    trajectory_clean_anchor = trajectory_history.back();
    trajectory_reliable_orientation << trajectory_clean_anchor.qx, trajectory_clean_anchor.qy, trajectory_clean_anchor.qz, trajectory_clean_anchor.qw;
    trajectory_has_reliable_orientation = true;
  } else {
    trajectory_has_clean_anchor = false;
    trajectory_last_clean_size = 0;
    trajectory_clean_anchor = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    trajectory_reliable_orientation << 0.0, 0.0, 0.0, 1.0;
    trajectory_has_reliable_orientation = false;
  }
}

void remember_reliable_orientation(const Eigen::Vector4d &quaternion) {
  // 原地转身时位置可能几乎不变，但方向已经是可信的新方向。
  // 继续对齐时要使用这个方向，避免 180 度转身后被拉回转身前朝向。
  trajectory_reliable_orientation = quaternion;
  trajectory_has_reliable_orientation = true;
}

Eigen::Vector4d trajectory_point_to_jpl_quat(const TrajectoryPoint &point) {
  Eigen::Vector4d q;
  q << point.qx, point.qy, point.qz, point.qw;
  return q;
}

void apply_trajectory_alignment(Eigen::Vector3d &position, Eigen::Vector4d &quaternion) {
  if (!trajectory_alignment_active) {
    return;
  }

  // 继续后的 VIO 坐标系可能同时改变原点和朝向。trajectory_alignment_rotation
  // 表示“新 VIO 全局坐标 -> 旧轨迹全局坐标”的旋转，所以位置用它左乘；
  // 四元数是 GtoC，需要乘这个全局旋转的逆，才能和位置方向保持一致。
  position = trajectory_alignment_anchor + trajectory_alignment_rotation * (position - trajectory_alignment_source_position);
  Eigen::Matrix3d aligned_rotation = ov_core::quat_2_Rot(quaternion) * trajectory_alignment_rotation.transpose();
  quaternion = ov_core::rot_2_quat(aligned_rotation);
}

bool get_last_trajectory_pose_for_display(Eigen::Vector3d &position, Eigen::Vector4d &quaternion) {
  if (!trajectory_has_clean_anchor && trajectory_history.empty()) {
    return false;
  }

  // 显示用位姿始终贴着轨迹线最后一个可靠点，避免暂停或重建 VIO 时方向框跳到漂移位置。
  const TrajectoryPoint &anchor = trajectory_has_clean_anchor ? trajectory_clean_anchor : trajectory_history.back();
  position = Eigen::Vector3d(anchor.x, anchor.y, anchor.z);
  quaternion = trajectory_has_reliable_orientation ? trajectory_reliable_orientation : trajectory_point_to_jpl_quat(anchor);
  return true;
}

bool get_last_trajectory_position(Eigen::Vector3d &position) {
  if (trajectory_history.empty()) {
    return false;
  }
  const auto &last = trajectory_history.back();
  position = Eigen::Vector3d(last.x, last.y, last.z);
  return true;
}

bool is_too_far_from_trajectory_end(const Eigen::Vector3d &position) {
  Eigen::Vector3d last_position;
  if (!get_last_trajectory_position(last_position)) {
    return false;
  }
  return (position - last_position).norm() > TRAJECTORY_MAX_CONNECT_STEP_METERS;
}

bool is_turning_in_place(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion) {
  if (!trajectory_debug_has_last_pose) {
    return false;
  }

  double rot = trajectory_quaternion_angle(trajectory_debug_last_pose, quaternion(3), quaternion(0), quaternion(1), quaternion(2));
  Eigen::Vector3d last_position(trajectory_debug_last_pose.x, trajectory_debug_last_pose.y, trajectory_debug_last_pose.z);
  double step = (position - last_position).norm();
  return rot > TRAJECTORY_TURN_IN_PLACE_ROT_RAD && step < TRAJECTORY_TURN_IN_PLACE_MAX_STEP_METERS;
}

bool update_turn_protection(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion) {
  if (!trajectory_debug_has_last_pose) {
    return false;
  }

  double rot = trajectory_quaternion_angle(trajectory_debug_last_pose, quaternion(3), quaternion(0), quaternion(1), quaternion(2));
  Eigen::Vector3d last_position;
  bool has_last_position = get_last_trajectory_position(last_position);
  double distance_from_end = has_last_position ? (position - last_position).norm() : 0.0;
  bool close_to_trajectory_end = !has_last_position || distance_from_end <= TRAJECTORY_TURN_PROTECT_MAX_END_DISTANCE;

  if (rot > TRAJECTORY_STRONG_TURN_ROT_RAD) {
    // 快速 90/180 度转身时，VIO 可能瞬间产生较大的横向假位移。
    // 这种情况下旋转信号优先，不再要求位置贴近终点，避免误弹漂移框。
    trajectory_strong_turn_protect_until_timestamp =
        std::max(trajectory_strong_turn_protect_until_timestamp, timestamp + TRAJECTORY_STRONG_TURN_PROTECT_SECONDS);
    trajectory_turn_protect_until_timestamp = std::max(trajectory_turn_protect_until_timestamp, timestamp + TRAJECTORY_TURN_PROTECT_SECONDS);
  } else if (rot > TRAJECTORY_TURN_PROTECT_ROT_RAD && close_to_trajectory_end) {
    // 只有“正在转向且仍贴近轨迹终点”才进入保护期。
    // 正常向前移动时会离开终点，不能继续静默过滤，否则轨迹会不绘制。
    trajectory_turn_protect_until_timestamp = std::max(trajectory_turn_protect_until_timestamp, timestamp + TRAJECTORY_TURN_PROTECT_SECONDS);
  }

  if (timestamp <= trajectory_strong_turn_protect_until_timestamp) {
    return true;
  }

  if (timestamp > trajectory_turn_protect_until_timestamp) {
    return false;
  }

  return close_to_trajectory_end;
}

bool update_resume_stability(const Eigen::Vector3d &position) {
  Eigen::Vector3d last_position;
  if (!get_last_trajectory_position(last_position)) {
    trajectory_resume_waiting_stable = false;
    trajectory_resume_stable_count = 0;
    return true;
  }

  // 继续后先等待 VIO 的输出贴近旧轨迹终点，稳定前不记录、不弹窗。
  if ((position - last_position).norm() <= TRAJECTORY_RESUME_STABLE_STEP_METERS) {
    trajectory_resume_stable_count++;
  } else {
    trajectory_resume_stable_count = 0;
  }

  if (trajectory_resume_stable_count >= TRAJECTORY_RESUME_STABLE_FRAMES) {
    trajectory_resume_waiting_stable = false;
    trajectory_resume_stable_count = 0;
    return true;
  }
  return false;
}

void rebase_alignment_to_trajectory_end(const Eigen::Vector3d &raw_position, const Eigen::Vector4d &raw_quaternion) {
  Eigen::Vector3d anchor_position;
  Eigen::Vector4d anchor_quaternion;
  if (!get_last_trajectory_pose_for_display(anchor_position, anchor_quaternion)) {
    return;
  }

  // 继续后的 VIO 偶尔第一帧仍然不稳定。如果对齐后的点离轨迹终点很远，
  // 就用当前 raw 位姿重新建立“当前帧 -> 轨迹终点”的对齐关系，避免画长线。
  trajectory_alignment_anchor = anchor_position;
  trajectory_alignment_source_position = raw_position;
  trajectory_alignment_rotation = ov_core::quat_2_Rot(anchor_quaternion).transpose() * ov_core::quat_2_Rot(raw_quaternion);
  trajectory_alignment_offset = trajectory_alignment_anchor - trajectory_alignment_source_position;
  trajectory_alignment_active = true;
  trajectory_alignment_pending = false;
}

void set_pose_arrays(JNIEnv *env, jdoubleArray position, jdoubleArray quaternion, const Eigen::Vector3d &p, const Eigen::Vector4d &q_jpl) {
  jdouble pos[3] = {p(0), p(1), p(2)};
  // Java 层使用 Hamilton 顺序 [qw, qx, qy, qz]，native 内部保持 JPL 顺序 [qx, qy, qz, qw]。
  jdouble quat[4] = {q_jpl(3), q_jpl(0), q_jpl(1), q_jpl(2)};
  env->SetDoubleArrayRegion(position, 0, 3, pos);
  env->SetDoubleArrayRegion(quaternion, 0, 4, quat);
}

void pause_trajectory_data(int reject_reason) {
  // 当前被拒绝的点还没有写入 trajectory_history，因此历史轨迹会停在最后一个可靠点。
  if (!trajectory_history.empty()) {
    trajectory_has_clean_anchor = true;
    trajectory_last_clean_size = trajectory_history.size();
    trajectory_clean_anchor = trajectory_history.back();
  } else {
    trajectory_has_clean_anchor = false;
    trajectory_last_clean_size = 0;
    trajectory_clean_anchor = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  }
  trajectory_data_paused = true;
  trajectory_pause_prompt_pending = true;
  trajectory_pause_reason = reject_reason;
  trajectory_drift_reject_streak = 0;
}

void prepare_trajectory_for_visual_recovery(bool interruption_had_motion) {
  std::lock_guard<std::mutex> traj_lck(trajectory_mtx);

  if (!trajectory_history.empty()) {
    trajectory_has_clean_anchor = true;
    trajectory_last_clean_size = trajectory_history.size();
    trajectory_clean_anchor = trajectory_history.back();
    trajectory_alignment_anchor = Eigen::Vector3d(trajectory_clean_anchor.x, trajectory_clean_anchor.y, trajectory_clean_anchor.z);
    trajectory_alignment_pending = true;
  } else {
    trajectory_alignment_anchor = Eigen::Vector3d::Zero();
    trajectory_alignment_pending = false;
  }
  trajectory_alignment_active = false;
  trajectory_alignment_offset = Eigen::Vector3d::Zero();
  trajectory_alignment_source_position = Eigen::Vector3d::Zero();
  trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
  trajectory_resume_grace_pending = true;
  trajectory_resume_grace_until_timestamp = -1.0;
  trajectory_resume_waiting_stable = !trajectory_history.empty();
  trajectory_resume_stable_count = 0;
  trajectory_turn_protect_until_timestamp = -1.0;
  trajectory_strong_turn_protect_until_timestamp = -1.0;
  trajectory_shift_window.clear();
  reset_consistent_drift_detector();
  trajectory_debug_last_timestamp = -1.0;
  trajectory_turn_guard_until_timestamp = -1.0;
  trajectory_debug_has_last_pose = false;
  trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

  if (interruption_had_motion) {
    // 摄像头不可用期间发生了移动，单靠手机 IMU 无法恢复真实距离，必须让用户确认后继续。
    pause_trajectory_data(TRAJECTORY_REJECT_VISUAL_INTERRUPTION_MOVED);
  }
}

void reset_trajectory_debug_state() {
  trajectory_debug_last_timestamp = -1.0;
  trajectory_turn_guard_until_timestamp = -1.0;
  trajectory_debug_session_start_size = 0;
  trajectory_debug_has_last_pose = false;
  trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  trajectory_data_paused = false;
  trajectory_pause_prompt_pending = false;
  trajectory_pause_reason = TRAJECTORY_REJECT_NONE;
  trajectory_drift_reject_streak = 0;
  trajectory_resume_grace_pending = false;
  trajectory_resume_grace_until_timestamp = -1.0;
  trajectory_resume_waiting_stable = false;
  trajectory_resume_stable_count = 0;
  trajectory_turn_protect_until_timestamp = -1.0;
  trajectory_strong_turn_protect_until_timestamp = -1.0;
  reset_consistent_drift_detector();
  trajectory_shift_window.clear();
  trajectory_alignment_pending = false;
  trajectory_alignment_active = false;
  trajectory_alignment_anchor = Eigen::Vector3d::Zero();
  trajectory_alignment_offset = Eigen::Vector3d::Zero();
  trajectory_alignment_source_position = Eigen::Vector3d::Zero();
  trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
  trajectory_last_clean_size = 0;
  trajectory_has_clean_anchor = false;
  trajectory_clean_anchor = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  trajectory_has_reliable_orientation = false;
  trajectory_reliable_orientation << 0.0, 0.0, 0.0, 1.0;
  reset_trajectory_auto_recovery();
  trajectory_last_soft_recovery_timestamp = -1.0;
  trajectory_soft_recovery_count = 0;
  trajectory_static_visual_conflict_active = false;
  trajectory_static_conflict_motion_frames = 0;
  trajectory_dynamic_visual_inconsistent = false;
  trajectory_ransac_bad_frames = 0;
  trajectory_ransac_good_frames = 0;
}

void record_trajectory_debug(double timestamp, const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion, size_t trajectory_size,
                             size_t feature_count, size_t update_feature_count, double update_grid_coverage,
                             double update_max_grid_ratio, size_t ransac_candidate_count, double ransac_inlier_ratio,
                             bool dynamic_visual_inconsistent, bool zupt_active, bool accepted, int reject_reason) {
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
  Eigen::Vector3d trajectory_end;
  bool has_trajectory_end = get_last_trajectory_position(trajectory_end);
  double distance_to_trajectory_end = has_trajectory_end ? (position - trajectory_end).norm() : 0.0;
  double shifting_mileage = trajectory_shifting_mileage_with_candidate(timestamp, position);
  bool turning_in_place = is_turning_in_place(position, quaternion);
  bool turn_protect_active = timestamp <= trajectory_turn_protect_until_timestamp;
  bool strong_turn_protect_active = timestamp <= trajectory_strong_turn_protect_until_timestamp;
  size_t session_trajectory_size = trajectory_size >= trajectory_debug_session_start_size
                                       ? trajectory_size - trajectory_debug_session_start_size
                                       : 0;
  const double feature_use_ratio = feature_count > 0 ? static_cast<double>(update_feature_count) / static_cast<double>(feature_count) : 0.0;
  const double auto_recovery_age = trajectory_auto_recovery_active && trajectory_auto_recovery_start_timestamp >= 0.0
                                       ? timestamp - trajectory_auto_recovery_start_timestamp
                                       : 0.0;

  if (compute_trajectory_delta(timestamp, position, quaternion, dt, step, speed, rot, forward_projection)) {
    anomaly = dt <= 0.0 || step > TRAJECTORY_DEBUG_JUMP_METERS || speed > TRAJECTORY_DEBUG_SPEED_MPS ||
              (step > 0.03 && rot > TRAJECTORY_DEBUG_ROTATION_RAD);
  }

  if (trajectory_debug_csv.is_open()) {
    trajectory_debug_csv << std::fixed << std::setprecision(9) << timestamp << "," << trajectory_size << "," << position(0) << ","
                         << position(1) << "," << position(2) << "," << qx << "," << qy << "," << qz << "," << qw << "," << dt << ","
                         << step << "," << speed << "," << rot << "," << feature_count << "," << (zupt_active ? 1 : 0) << ","
                         << forward_projection << "," << (turn_guard_active ? 1 : 0) << "," << (accepted ? 1 : 0) << "," << reject_reason
                         << "," << (anomaly ? 1 : 0) << "," << session_trajectory_size << "," << distance_to_trajectory_end
                         << "," << shifting_mileage << "," << (turning_in_place ? 1 : 0) << "," << (turn_protect_active ? 1 : 0)
                         << "," << (strong_turn_protect_active ? 1 : 0) << "," << (trajectory_resume_waiting_stable ? 1 : 0)
                         << "," << trajectory_resume_stable_count << "," << trajectory_consistent_drift_count << ","
                         << latest_imu_pair_delta_seconds.load() << "," << update_feature_count << "," << feature_use_ratio << ","
                         << (trajectory_auto_recovery_active ? 1 : 0) << "," << auto_recovery_age << ","
                         << trajectory_soft_recovery_count << "," << trajectory_auto_recovery_update_frames << ","
                         << trajectory_auto_recovery_update_features << "," << update_grid_coverage << "," << update_max_grid_ratio
                         << "," << ransac_candidate_count << "," << ransac_inlier_ratio
                         << "," << (dynamic_visual_inconsistent ? 1 : 0) << "," << trajectory_ransac_bad_frames << ","
                         << trajectory_ransac_good_frames
                         << std::endl;
  }

  if (anomaly || !accepted) {
    __android_log_print(ANDROID_LOG_WARN, TAG,
                        "Trajectory point: t=%.6f dt=%.4f step=%.3f speed=%.3f rot_deg=%.1f forward=%.2f guard=%d features=%zu zupt=%d "
                        "accepted=%d reason=%d size=%zu update_features=%zu use_ratio=%.3f\n",
                        timestamp, dt, step, speed, rot * 180.0 / M_PI, forward_projection, turn_guard_active ? 1 : 0, feature_count,
                        zupt_active ? 1 : 0, accepted ? 1 : 0, reject_reason, trajectory_size, update_feature_count, feature_use_ratio);
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
      // 不使用只检查退出状态的 predicate。旧写法会忽略相机入队时的 notify，
      // 导致线程经常等满 100ms 后才批量处理，不同帧率手机会产生不同队列延迟。
      processing_cv.wait_for(proc_lck, std::chrono::milliseconds(100));
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

      latest_camera_processing_age_seconds.store(std::max(0.0, time_now_sec - cam_msg.timestamp));
      latest_camera_imu_lead_seconds.store(std::max(0.0, timestamp_imu_inC - cam_msg.timestamp));
      latest_camera_queue_after_pop.store(queue_size_after_pop);

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
        if (trajectory_resume_waiting_stable && trajectory_recovery_user_state.load() == 3) {
          // 初始化完成后还需要等待新坐标系与最后可靠轨迹点完成对齐。
          trajectory_recovery_user_state.store(2);
        }
        if (trajectory_recovery_user_state.load() == 3 && trajectory_history.empty()) {
          trajectory_recovery_user_state.store(0);
        }
        Eigen::Vector3d p_traj = p_cam;
        Eigen::Vector3d p_raw_for_alignment = p_cam;
        Eigen::Vector4d q_raw_for_alignment = q_cam;
        if (trajectory_alignment_pending) {
          Eigen::Vector3d alignment_anchor = trajectory_alignment_anchor;
          Eigen::Vector4d anchor_quat =
              trajectory_has_reliable_orientation ? trajectory_reliable_orientation
                                                  : (trajectory_has_clean_anchor ? trajectory_point_to_jpl_quat(trajectory_clean_anchor) : q_cam);
          Eigen::Matrix3d source_rotation = ov_core::quat_2_Rot(q_cam);
          Eigen::Matrix3d anchor_rotation = ov_core::quat_2_Rot(anchor_quat);
          // 继续后 VIO 会进入新的局部坐标系。这里记录“新起点 -> 最后可靠点”的
          // 位置和朝向差，后续每个点都用同一个刚体变换接到旧轨迹上。
          trajectory_debug_last_timestamp = -1.0;
          trajectory_turn_guard_until_timestamp = -1.0;
          trajectory_debug_has_last_pose = false;
          trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
          trajectory_shift_window.clear();
          trajectory_alignment_anchor = alignment_anchor;
          trajectory_alignment_source_position = p_cam;
          trajectory_alignment_rotation = anchor_rotation.transpose() * source_rotation;
          trajectory_alignment_offset = trajectory_alignment_anchor - trajectory_alignment_source_position;
          trajectory_alignment_active = true;
          trajectory_alignment_pending = false;
        }
        apply_trajectory_alignment(p_traj, q_cam);
        const bool recovering_from_camera_filter = camera_filter_recovery_pending.exchange(false);
        if (recovering_from_camera_filter && is_too_far_from_trajectory_end(p_traj)) {
          // 被过滤帧没有可靠视觉约束，其间产生的位移不能作为漂移证据。恢复首帧只
          // 重建坐标接续关系，不销毁 VIO；后续有效帧再从可靠终点继续记录轨迹。
          rebase_alignment_to_trajectory_end(p_raw_for_alignment, q_raw_for_alignment);
          p_traj = p_raw_for_alignment;
          q_cam = q_raw_for_alignment;
          apply_trajectory_alignment(p_traj, q_cam);
          trajectory_shift_window.clear();
          trajectory_drift_reject_streak = 0;
          reset_consistent_drift_detector();
          reset_trajectory_auto_recovery();
        }
        if (trajectory_resume_grace_pending) {
          // 刚继续或重新初始化时，OpenVINS 可能有少量稳定过程。
          // 保护期内仍然丢弃异常点，但不立刻弹窗打断用户。
          trajectory_resume_grace_until_timestamp = state->_timestamp + TRAJECTORY_RESUME_GRACE_SECONDS;
          trajectory_resume_grace_pending = false;
        }
        bool suppress_pause_prompt = state->_timestamp <= trajectory_resume_grace_until_timestamp;
        if (trajectory_alignment_active && (suppress_pause_prompt || trajectory_resume_waiting_stable) && is_too_far_from_trajectory_end(p_traj)) {
          rebase_alignment_to_trajectory_end(p_raw_for_alignment, q_raw_for_alignment);
          p_traj = p_raw_for_alignment;
          q_cam = q_raw_for_alignment;
          apply_trajectory_alignment(p_traj, q_cam);
        }
        size_t feature_count = local_sys->get_last_track_count();
        size_t update_feature_count = local_sys->get_last_update_feature_count();
        double update_grid_coverage = local_sys->get_last_update_grid_coverage();
        double update_max_grid_ratio = local_sys->get_last_update_max_grid_ratio();
        size_t ransac_candidate_count = local_sys->get_last_ransac_candidate_count();
        double ransac_inlier_ratio = local_sys->get_last_ransac_inlier_ratio();
        update_dynamic_visual_consistency(ransac_candidate_count, ransac_inlier_ratio);
        bool zupt_active = local_sys->last_update_used_zupt();
        int reject_reason = TRAJECTORY_REJECT_NONE;
        bool accept_trajectory_point = false;
        if (!trajectory_data_paused) {
          Eigen::Vector3d reliable_end;
          const bool has_reliable_end = get_last_trajectory_position(reliable_end);
          const bool static_conflict_candidate = zupt_active && latest_imu_static.load() && has_reliable_end &&
                                                 (p_traj - reliable_end).norm() > TRAJECTORY_STATIC_VISUAL_CONFLICT_METERS;
          if (!trajectory_static_visual_conflict_active && static_conflict_candidate) {
            trajectory_static_visual_conflict_active = true;
            trajectory_static_conflict_motion_frames = 0;
          }

          if (trajectory_static_visual_conflict_active) {
            const bool physical_motion_confirmed =
                latest_imu_gyro_mean.load() > TRAJECTORY_STATIC_CONFLICT_RELEASE_GYRO_MEAN ||
                latest_imu_accel_stddev.load() > TRAJECTORY_STATIC_CONFLICT_RELEASE_ACCEL_STDDEV;
            trajectory_static_conflict_motion_frames =
                physical_motion_confirmed ? trajectory_static_conflict_motion_frames + 1 : 0;
            if (trajectory_static_conflict_motion_frames >= TRAJECTORY_STATIC_CONFLICT_RELEASE_FRAMES) {
              // 只有 IMU 连续确认手机真实运动后才解除锁定。ZUPT 或静止标志的
              // 单帧波动不能让方向框在可靠终点和错误 VIO 位姿之间来回跳。
              trajectory_static_visual_conflict_active = false;
              trajectory_static_conflict_motion_frames = 0;
            }
          }
          const bool static_visual_conflict = trajectory_static_visual_conflict_active;
          bool turn_protected = update_turn_protection(state->_timestamp, p_traj, q_cam);
          // 先执行基础健康门控。此前该函数虽然存在但没有接入轨迹写入流程，
          // 导致几十米每秒的明显错误点仍可能被转弯保护分支接受。
          // 这类单点异常只静默过滤，不累计弹窗次数。
          const bool point_health_ok =
              should_accept_trajectory_point(state->_timestamp, p_traj, q_cam, feature_count,
                                             trajectory_dynamic_visual_inconsistent, reject_reason);
          if (static_visual_conflict) {
            // 手机被 IMU 和零速更新共同判定为静止时，视觉画面产生的位置变化
            // 只能来自动态目标或错误特征。立即冻结显示和轨迹，不允许软恢复
            // 把这个视觉假运动重新接到可靠终点。
            reject_reason = TRAJECTORY_REJECT_STATIC_VISUAL_CONFLICT;
            trajectory_shift_window.clear();
            trajectory_drift_reject_streak = 0;
            reset_consistent_drift_detector();
            reset_trajectory_auto_recovery();
          } else if (!point_health_ok) {
            trajectory_shift_window.clear();
            reset_consistent_drift_detector();
          } else if (is_turning_in_place(p_traj, q_cam)) {
            // 严格原地转身只更新方向锚点，不追加轨迹点。
            // 普通转弯不能走这里，否则边走边转时会出现“方向框动、轨迹不画”。
            Eigen::Vector3d last_position;
            if (get_last_trajectory_position(last_position)) {
              p_traj = last_position;
            }
            remember_reliable_orientation(q_cam);
            trajectory_shift_window.clear();
            trajectory_drift_reject_streak = 0;
            reset_consistent_drift_detector();
          } else if (trajectory_resume_waiting_stable) {
            // 点击继续后先等 VIO 连续几帧贴近旧轨迹终点，稳定前不绘制、不弹窗。
            if (update_resume_stability(p_traj)) {
              trajectory_debug_last_timestamp = -1.0;
              trajectory_debug_has_last_pose = false;
              trajectory_shift_window.clear();
              reset_consistent_drift_detector();
              accept_trajectory_point = true;
              trajectory_recovery_user_state.store(0);
            }
          } else if (turn_protected && !is_too_far_from_trajectory_end(p_traj)) {
            // 转弯保护期内允许正常移动轨迹通过，但不累计 shifting 窗口，
            // 避免转弯时的姿态变化被当成漂移。
            trajectory_shift_window.clear();
            reset_consistent_drift_detector();
            accept_trajectory_point = true;
          } else if (turn_protected && is_too_far_from_trajectory_end(p_traj)) {
            // 转弯时如果 VIO 点跳得很远，静默丢弃该点，不弹窗也不画长线。
            remember_reliable_orientation(q_cam);
            trajectory_shift_window.clear();
            trajectory_drift_reject_streak = 0;
            reset_consistent_drift_detector();
          } else if (is_too_far_from_trajectory_end(p_traj)) {
            // 距离异常先不弹窗，只有连续同方向、同量级偏移才认为是真漂移。
            if (update_consistent_drift_detector(p_traj)) {
              reject_reason = TRAJECTORY_REJECT_SHIFTING_WINDOW;
            }
          } else if (!turn_protected && is_trajectory_shifting(state->_timestamp, p_traj)) {
            if (update_consistent_drift_detector(p_traj)) {
              reject_reason = TRAJECTORY_REJECT_SHIFTING_WINDOW;
              rollback_trajectory_after_shifting(state->_timestamp);
            }
          } else {
            reset_consistent_drift_detector();
            accept_trajectory_point = true;
          }
        }
        if (accept_trajectory_point) {
          trajectory_history.emplace_back(p_traj(0), p_traj(1), p_traj(2), q_cam(3), q_cam(0), q_cam(1), q_cam(2));
          if (trajectory_history.size() > MAX_TRAJECTORY_POINTS) {
            trajectory_history.erase(trajectory_history.begin());
            if (trajectory_last_clean_size > 0) {
              trajectory_last_clean_size--;
            }
          }
          trajectory_has_clean_anchor = true;
          trajectory_last_clean_size = trajectory_history.size();
          trajectory_clean_anchor = trajectory_history.back();
          remember_trajectory_shift_sample(state->_timestamp, p_traj, trajectory_history.size());
          remember_reliable_orientation(q_cam);
          trajectory_drift_reject_streak = 0;
          // 候选异常自行回到可接受状态时，直接结束观察，不打断用户。
          reset_trajectory_auto_recovery();
        } else if (!trajectory_data_paused && is_trajectory_drift_reason(reject_reason)) {
          // 漂移候选点不会写入 trajectory_history。即使恢复保护期内不弹窗，
          // 可视轨迹也会停在最后一个可靠点，不会把异常点连成线。
          if (latest_imu_gyro_mean.load() >= TRAJECTORY_FILTERED_ORIENTATION_GYRO_MEAN_MIN) {
            // 过滤期间只冻结不可靠的位置。陀螺仪持续确认真实旋转时，保留已经
            // 对齐到旧轨迹坐标系的当前方向，避免用户转身后方向退回过滤前。
            // 手机静止、仅画面内目标运动时不会满足该条件。
            remember_reliable_orientation(q_cam);
          }
          if (suppress_pause_prompt) {
            trajectory_drift_reject_streak = 0;
            reset_trajectory_auto_recovery();
          } else {
            if (!trajectory_auto_recovery_active) {
              trajectory_auto_recovery_active = true;
              trajectory_auto_recovery_start_timestamp = state->_timestamp;
              trajectory_auto_recovery_update_frames = 0;
              trajectory_auto_recovery_update_features = 0;
              trajectory_drift_reject_streak = 0;
            }

            // 跟踪点总数不代表 VIO 已经重新获得视觉约束。只有真正参与滤波更新的
            // 特征持续出现，才允许使用当前 VIO 位姿建立新的轨迹对齐关系。
            if (update_feature_count > 0) {
              trajectory_auto_recovery_update_frames++;
              trajectory_auto_recovery_update_features += update_feature_count;
            }

            const double observe_age = state->_timestamp - trajectory_auto_recovery_start_timestamp;
            const bool visual_update_ready =
                trajectory_auto_recovery_update_frames >= TRAJECTORY_SOFT_RECOVERY_MIN_UPDATE_FRAMES &&
                trajectory_auto_recovery_update_features >= TRAJECTORY_SOFT_RECOVERY_MIN_UPDATE_FEATURES;
            Eigen::Vector3d soft_recovery_end;
            const bool has_soft_recovery_end = get_last_trajectory_position(soft_recovery_end);
            const double recovery_distance = has_soft_recovery_end ? (p_traj - soft_recovery_end).norm() : 0.0;
            const bool recovery_distance_safe =
                !has_soft_recovery_end || recovery_distance <= TRAJECTORY_SOFT_RECOVERY_MAX_DISTANCE_METERS;
            const bool recovery_cooldown_ready = trajectory_last_soft_recovery_timestamp < 0.0 ||
                                                 state->_timestamp - trajectory_last_soft_recovery_timestamp >=
                                                     TRAJECTORY_SOFT_RECOVERY_COOLDOWN_SECONDS;
            const bool soft_recovery_available = recovery_distance_safe && recovery_cooldown_ready;
            if (observe_age >= TRAJECTORY_AUTO_RECOVERY_OBSERVE_SECONDS && soft_recovery_available && visual_update_ready) {
              // 不销毁 VIO，只把当前局部坐标重新接到最后可靠轨迹点。观察期间的异常点
              // 从未写入轨迹，因此不会出现一条长线连到漂移位置。
              rebase_alignment_to_trajectory_end(p_raw_for_alignment, q_raw_for_alignment);
              p_traj = p_raw_for_alignment;
              q_cam = q_raw_for_alignment;
              apply_trajectory_alignment(p_traj, q_cam);
              trajectory_last_soft_recovery_timestamp = state->_timestamp;
              trajectory_soft_recovery_count++;
              trajectory_drift_reject_streak = 0;
              trajectory_shift_window.clear();
              reset_consistent_drift_detector();
              reset_trajectory_auto_recovery();
              __android_log_print(ANDROID_LOG_INFO, TAG, "Trajectory soft recovery applied at %.6f (count=%zu)\n",
                                  state->_timestamp, trajectory_soft_recovery_count);
            } else if (observe_age >= TRAJECTORY_AUTO_RECOVERY_MAX_WAIT_SECONDS && !visual_update_ready) {
              // 没有有效视觉更新时继续冻结并过滤异常点，不重建 VIO、不打断用户。
              // 后续视觉重新稳定后仍可使用同一个观察窗口完成软对齐。
              trajectory_drift_reject_streak = 0;
            } else if (observe_age >= TRAJECTORY_AUTO_RECOVERY_OBSERVE_SECONDS && !soft_recovery_available) {
              // 大于 1.5 米的偏移已经不是坐标接续误差，软对齐会掩盖失稳的速度和
              // 航向状态；短时间复发也说明上次对齐没有修好估计器。两种情况均只
              // 冻结和过滤，不把错误状态重新接入可靠轨迹。
              trajectory_drift_reject_streak = 0;
              if (!recovery_distance_safe && observe_age >= TRAJECTORY_BACKGROUND_REINIT_WAIT_SECONDS && visual_update_ready &&
                  ransac_candidate_count >= TRAJECTORY_RANSAC_MIN_CANDIDATES &&
                  ransac_inlier_ratio >= TRAJECTORY_BACKGROUND_REINIT_MIN_RANSAC_RATIO &&
                  !trajectory_dynamic_visual_inconsistent) {
                // 已确认旧 VIO 严重失稳时统一进入暂停弹窗，不再后台自动 INIT。
                // 只有用户点击“继续”后才重建 VIO，避免运行中无感知地突然初始化。
                pause_trajectory_data(TRAJECTORY_REJECT_SHIFTING_WINDOW);
                reset_trajectory_auto_recovery();
              }
            }
          }
        } else if (!trajectory_data_paused) {
          trajectory_drift_reject_streak = 0;
        }
        if (trajectory_data_paused) {
          accept_trajectory_point = false;
          reject_reason = trajectory_pause_reason;
        }
        record_trajectory_debug(state->_timestamp, p_traj, q_cam, trajectory_history.size(), feature_count, update_feature_count,
                                update_grid_coverage, update_max_grid_ratio, ransac_candidate_count, ransac_inlier_ratio,
                                trajectory_dynamic_visual_inconsistent, zupt_active, accept_trajectory_point, reject_reason);

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
    trajectory_debug_csv << "timestamp,trajectory_size,p_x,p_y,p_z,q_x,q_y,q_z,q_w,dt,step_m,speed_mps,rot_rad,feature_count,zupt_active,"
                            "forward_projection,turn_guard_active,accepted,reject_reason,anomaly,session_trajectory_size,"
                            "distance_to_trajectory_end_m,shifting_mileage_m,turning_in_place,turn_protect_active,"
                            "strong_turn_protect_active,resume_waiting_stable,resume_stable_count,consistent_drift_count,imu_pair_delta_s"
                            ",update_feature_count,feature_use_ratio,auto_recovery_active,auto_recovery_age_s,soft_recovery_count,"
                            "auto_recovery_update_frames,auto_recovery_update_features,update_grid_coverage,update_max_grid_ratio,"
                            "ransac_candidate_count,ransac_inlier_ratio,dynamic_visual_inconsistent,ransac_bad_frames,ransac_good_frames"
                         << std::endl;
    std::string camera_quality_csv_name = s + "camera_quality_debug.csv";
    camera_quality_debug_csv.open(save_folder + camera_quality_csv_name);
    camera_quality_debug_csv
        << "timestamp,image_mean,image_stddev,dark_pixel_ratio,blur_score,blocked,imu_accel_norm,imu_gyro_norm,"
           "imu_accel_stddev,imu_gyro_mean,imu_static,frame_delta_s,camera_queue_size,imu_pair_delta_s,"
           "visual_interruption_active,recovery_detected,interruption_had_motion,vio_time_offset_s,"
           "unusable_texture,motion_blurred,frame_filtered,camera_processing_age_s,camera_imu_lead_s,"
           "camera_queue_after_pop"
        << std::endl;
    reset_trajectory_debug_state();
    {
      std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
      // 继续后的记录允许沿用既有轨迹；单独记录本次起始点数，分析时不再受旧数据干扰。
      trajectory_debug_session_start_size = trajectory_history.size();
    }
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
    if (camera_quality_debug_csv.is_open()) {
      std::lock_guard<std::mutex> log_lck(camera_quality_debug_csv_mtx);
      camera_quality_debug_csv.close();
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
      reset_imu_motion_state();
    }
    reset_visual_interruption_state();

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
      reset_imu_motion_state();
    }
    reset_visual_interruption_state();
    // 点击开始后立即显示初始化状态，不等待首批相机帧和 IMU 数据进入估计器。
    trajectory_recovery_user_state.store(3);

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
    if (is_running_ov && trajectory_recovery_user_state.load() == 3) {
      // VIO 对象创建前也立即给出 INIT 反馈，避免用户误以为点击开始没有生效。
      cv::putText(*rgbMat, "INIT", cv::Point(24, 48), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 2, cv::LINE_AA);
    }
    return reinterpret_cast<jlong>(rgbMat);
  }

  if ((visual_recovery_user_state.load() != 0 || camera_frame_filtered_for_display.load()) && rawCameraMatAddr != 0) {
    // 真正过滤图像帧时改用实时预览，避免继续展示冻结的历史关键点画面。
    // 小标识只说明当前帧未参与定位，不覆盖主状态文字，也不触发 INIT。
    cv::Mat &rawMat = *(cv::Mat *)rawCameraMatAddr;
    cv::Mat *rgbMat = new cv::Mat();
    cv::cvtColor(rawMat, *rgbMat, cv::COLOR_RGBA2GRAY);
    cv::putText(*rgbMat, "FILTER", cv::Point(24, 42), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255), 1, cv::LINE_AA);
    return reinterpret_cast<jlong>(rgbMat);
  }

  // 系统运行时保留 OpenVINS 原生可视化。漂移后重新初始化期间，用户可以直接看到
  // INIT、特征点和初始化参数变化，比只显示一行文字更容易理解当前进度。
  cv::Mat viz_img = local_sys->get_historical_viz_image();
  if (viz_img.empty()) {
    // Fallback to raw camera if no viz image
    if (rawCameraMatAddr != 0) {
      cv::Mat &rawMat = *(cv::Mat *)rawCameraMatAddr;
      cv::Mat *rgbMat = new cv::Mat();
      cv::cvtColor(rawMat, *rgbMat, cv::COLOR_RGBA2GRAY);
      if (trajectory_recovery_user_state.load() == 3) {
        // VIO 已创建但首张可视化图尚未生成时，也不能让 INIT 提示短暂消失。
        cv::putText(*rgbMat, "INIT", cv::Point(24, 48), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 2, cv::LINE_AA);
      }
      return reinterpret_cast<jlong>(rgbMat);
    }
    return 0;
  }

  // Clone the viz image and apply overlays
  cv::Mat *displayMat = new cv::Mat(viz_img.clone());

  if (trajectory_recovery_user_state.load() == 3) {
    // 无论 VIO 对象和历史可视化图是否已创建，点击开始后都立即展示 INIT。
    cv::putText(*displayMat, "INIT", cv::Point(24, 48), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 2, cv::LINE_AA);
  }

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
  const double raw_time_in_sec = timestampSec;
  const double time_in_sec = raw_time_in_sec;
  unsigned long long time_in_ns = (unsigned long long)(time_in_sec * 1e9);

  // get Mat from raw address
  clock_t begin = clock();
  cv::Mat &mat = *(cv::Mat *)matAddr;

  // Convert to gray scale (Mat is RGBA from YUV conversion)
  cv::Mat mat_gray;
  cv::cvtColor(mat, mat_gray, cv::COLOR_RGBA2GRAY);
  const CameraQuality camera_quality = evaluate_camera_quality(mat_gray);

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

    size_t camera_queue_size = 0;
    {
      std::lock_guard<std::mutex> lck(camera_queue_mtx);
      camera_queue_size = camera_queue.size();
    }

    bool interruption_active_for_log = false;
    bool interruption_had_motion = false;
    const bool motion_blurred = camera_quality.blur_score <= CAMERA_MOTION_BLUR_MAX &&
                                latest_imu_gyro_norm.load() >= CAMERA_MOTION_BLUR_GYRO_MIN;
    // 白墙等低纹理画面只影响定位数据，不等同于摄像头被遮挡。只有 blocked
    // 状态才切换用户提示和相机展示，避免关键点画面与原始画面反复闪动。
    // 低纹理不等于无效图像：继续送入 OpenVINS，保持相机时间轴和可视化连续，
    // 再由特征数量及轨迹健康门控决定是否记录点位。只有明确遮挡或高速模糊才丢帧。
    const bool frame_filtered = camera_quality.blocked || motion_blurred;
    camera_frame_filtered_for_display.store(frame_filtered);
    {
      std::lock_guard<std::mutex> state_lck(visual_interruption_mtx);
      if (camera_quality.blocked) {
        visual_blocked_consecutive_frames++;
        visual_clear_consecutive_frames = 0;
        if (!visual_interruption_active && visual_blocked_consecutive_frames >= VISUAL_INTERRUPTION_CONFIRM_FRAMES) {
          visual_interruption_active = true;
          visual_interruption_start_timestamp = raw_time_in_sec;
          visual_motion_consecutive_frames = 0;
          visual_interruption_had_motion = false;
          visual_recovery_user_state.store(1);
        }
        if (visual_interruption_active &&
            raw_time_in_sec - visual_interruption_start_timestamp >= VISUAL_INTERRUPTION_MOTION_GRACE_SECONDS) {
          if (latest_imu_static.load()) {
            visual_motion_consecutive_frames = 0;
          } else {
            visual_motion_consecutive_frames++;
            if (visual_motion_consecutive_frames >= VISUAL_INTERRUPTION_MOTION_CONFIRM_FRAMES) {
              visual_interruption_had_motion = true;
            }
          }
        }
      } else {
        if (visual_interruption_active) {
          // 只过滤明确异常的图像帧。首个正常帧到达后立即解除遮挡状态，
          // 不冻结 IMU、不压缩时间轴，也不重建 VIO，避免恢复流程卡住。
          interruption_had_motion = visual_interruption_had_motion;
          visual_blocked_consecutive_frames = 0;
          visual_clear_consecutive_frames = 0;
          visual_motion_consecutive_frames = 0;
          visual_interruption_active = false;
          visual_interruption_had_motion = false;
          visual_interruption_start_timestamp = -1.0;
          visual_recovery_user_state.store(0);
        } else {
          visual_blocked_consecutive_frames = 0;
          visual_clear_consecutive_frames = 0;
          visual_motion_consecutive_frames = 0;
        }
      }
      interruption_active_for_log = visual_interruption_active;
    }

    if (is_recording && camera_quality_debug_csv.is_open()) {
      std::lock_guard<std::mutex> log_lck(camera_quality_debug_csv_mtx);
      camera_quality_debug_csv << std::fixed << std::setprecision(9) << raw_time_in_sec << "," << camera_quality.mean << ","
                               << camera_quality.stddev << "," << camera_quality.dark_ratio << "," << camera_quality.blur_score << ","
                               << (camera_quality.blocked ? 1 : 0) << "," << latest_imu_accel_norm.load() << ","
                               << latest_imu_gyro_norm.load() << "," << latest_imu_accel_stddev.load() << ","
                               << latest_imu_gyro_mean.load() << "," << (latest_imu_static.load() ? 1 : 0) << "," << frame_delta << ","
                               << camera_queue_size << "," << latest_imu_pair_delta_seconds.load() << ","
                               << (interruption_active_for_log ? 1 : 0) << "," << 0 << ","
                               << (interruption_had_motion ? 1 : 0) << "," << 0.0 << ","
                               << (camera_quality.unusable_texture ? 1 : 0) << "," << (motion_blurred ? 1 : 0) << ","
                               << (frame_filtered ? 1 : 0) << "," << latest_camera_processing_age_seconds.load() << ","
                               << latest_camera_imu_lead_seconds.load() << "," << latest_camera_queue_after_pop.load() << std::endl;
    }

    if (frame_filtered) {
      // 遮挡、低纹理或高速旋转下的严重模糊帧不能提供可靠视觉约束。
      // 这里只过滤当前图像帧，IMU 与相机预览保持连续，也不会触发重新初始化。
      camera_filter_recovery_pending.store(true);
      __android_log_print(ANDROID_LOG_WARN, TAG,
                          "Dropping low-quality camera frame: mean=%.2f std=%.2f dark=%.3f blur=%.2f gyro=%.3f blocked=%d motion_blur=%d\n",
                          camera_quality.mean, camera_quality.stddev, camera_quality.dark_ratio, camera_quality.blur_score,
                          latest_imu_gyro_norm.load(), camera_quality.blocked ? 1 : 0, motion_blurred ? 1 : 0);
      return;
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
                                                                                         jfloat gz, jdouble timestampSec,
                                                                                         jdouble pairDeltaSec) {

  // Use the hardware timestamp from SensorEvent (nanoseconds since boot, converted to seconds)
  // This ensures consistent timing and matches camera timestamp reference
  double time_in_sec = timestampSec;
  latest_imu_pair_delta_seconds.store(std::max(0.0, static_cast<double>(pairDeltaSec)));
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
        __android_log_print(ANDROID_LOG_WARN, TAG, "Replacing IMU spike at %.9f: acc_norm=%.3f gyro_norm=%.3f hard=%d jump=%d\n",
                            time_in_sec, accel_norm, gyro_norm, hard_spike ? 1 : 0, jump_spike ? 1 : 0);
        n_ax = last_valid_ax;
        n_ay = last_valid_ay;
        n_az = last_valid_az;
        n_gx = last_valid_gx;
        n_gy = last_valid_gy;
        n_gz = last_valid_gz;
        last_valid_imu_timestamp = time_in_sec;
      } else {
        __android_log_print(ANDROID_LOG_WARN, TAG, "Dropping IMU spike at %.9f: acc_norm=%.3f gyro_norm=%.3f hard=%d jump=%d\n",
                            time_in_sec, accel_norm, gyro_norm, hard_spike ? 1 : 0, jump_spike ? 1 : 0);
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

    if (imu_sample_valid) {
      const double filtered_accel_norm = std::sqrt(n_ax * n_ax + n_ay * n_ay + n_az * n_az);
      const double filtered_gyro_norm = std::sqrt(n_gx * n_gx + n_gy * n_gy + n_gz * n_gz);
      update_imu_motion_state(time_in_sec, filtered_accel_norm, filtered_gyro_norm);
    }
  }

  if (!imu_sample_valid) {
    return;
  }

  const double vio_time_in_sec = time_in_sec;

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
    if (vio_time_in_sec <= last_accepted_imu_timestamp) {
      __android_log_print(ANDROID_LOG_WARN, TAG, "Skipping non-monotonic IMU timestamp: %.9f <= %.9f\n", vio_time_in_sec,
                          last_accepted_imu_timestamp);
      return;
    }
    last_accepted_imu_timestamp = vio_time_in_sec;
    latest_imu_timestamp = vio_time_in_sec;
    accepted_imu_count++;
  }

  // Feed if the system is running!
  if (local_sys != nullptr) {
    // Send it into the system
    ov_core::ImuData message_imu;
    message_imu.timestamp = vio_time_in_sec;
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

  // 相机帧可能因时间戳略新于最新 IMU 而暂存在队列中。旧逻辑只能等下一帧相机
  // 或 100ms 超时后重试；每条有效 IMU 到达时唤醒一次，可在 IMU 追上后立即处理。
  processing_cv.notify_one();
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
    std::lock_guard<std::mutex> traj_lck(trajectory_mtx);
    Eigen::Vector3d display_position;
    Eigen::Vector4d display_quaternion;
    if (get_last_trajectory_pose_for_display(display_position, display_quaternion)) {
      set_pose_arrays(env, position, quaternion, display_position, display_quaternion);
      return JNI_TRUE;
    }
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
    if (trajectory_data_paused || trajectory_alignment_pending || trajectory_resume_waiting_stable ||
        trajectory_static_visual_conflict_active) {
      // 暂停、等待继续对齐或自动恢复观察期间，方向框固定在轨迹线最后一点。
      // 只有恢复完成后，才重新显示经过刚体变换后的 VIO 当前位姿。
      get_last_trajectory_pose_for_display(p_cam, q_cam);
    } else if (trajectory_alignment_active) {
      apply_trajectory_alignment(p_cam, q_cam);
    }
    Eigen::Vector3d reliable_position;
    if (get_last_trajectory_position(reliable_position) &&
        (p_cam - reliable_position).norm() > TRAJECTORY_MAX_CONNECT_STEP_METERS) {
      // 显示是否固定只由“当前位姿是否已明显跑远”决定，不再跟随过滤状态开关。
      // 这样既不显示远处错误方向框，也不会因过滤标志抖动而来回切换。
      get_last_trajectory_pose_for_display(p_cam, q_cam);
    }
  }

  if (is_recording && pose_ext_csv.is_open()) {
    unsigned long long time_in_ns = (unsigned long long)(state->_timestamp * 1e9);
    pose_ext_csv << time_in_ns << "," << p_cam(0) << "," << p_cam(1) << "," << p_cam(2) << "," << q_cam(0) << "," << q_cam(1) << ","
                 << q_cam(2) << "," << q_cam(3) << std::endl;
  }

  set_pose_arrays(env, position, quaternion, p_cam, q_cam);

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

extern "C" JNIEXPORT jint JNICALL Java_com_openvins_android_VioEngine_getVisualRecoveryStateJNI(JNIEnv *env, jobject instance) {
  // 漂移恢复提示优先级高于遮挡提示，两个状态不再互相清除。
  const int trajectory_state = trajectory_recovery_user_state.load();
  if (trajectory_state != 0) {
    return static_cast<jint>(trajectory_state);
  }
  return static_cast<jint>(visual_recovery_user_state.load());
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
    trajectory_resume_grace_pending = true;
    trajectory_resume_grace_until_timestamp = -1.0;
    trajectory_resume_waiting_stable = true;
    trajectory_resume_stable_count = 0;
    trajectory_turn_protect_until_timestamp = -1.0;
    trajectory_strong_turn_protect_until_timestamp = -1.0;
    trajectory_static_visual_conflict_active = false;
    trajectory_static_conflict_motion_frames = 0;
    trajectory_dynamic_visual_inconsistent = false;
    trajectory_ransac_bad_frames = 0;
    trajectory_ransac_good_frames = 0;
    reset_consistent_drift_detector();
    reset_trajectory_auto_recovery();
    // 完整重新初始化后允许新 VIO 状态再次使用一次静默软恢复机会。
    trajectory_last_soft_recovery_timestamp = -1.0;
    trajectory_shift_window.clear();
    trajectory_debug_last_timestamp = -1.0;
    trajectory_turn_guard_until_timestamp = -1.0;
    trajectory_debug_has_last_pose = false;
    trajectory_debug_last_pose = TrajectoryPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

    if (trajectory_has_clean_anchor) {
      trajectory_alignment_anchor = Eigen::Vector3d(trajectory_clean_anchor.x, trajectory_clean_anchor.y, trajectory_clean_anchor.z);
      trajectory_alignment_pending = true;
      trajectory_alignment_active = false;
      trajectory_alignment_offset = Eigen::Vector3d::Zero();
      trajectory_alignment_source_position = Eigen::Vector3d::Zero();
      trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
    } else if (!trajectory_history.empty()) {
      const auto &last = trajectory_history.back();
      trajectory_alignment_anchor = Eigen::Vector3d(last.x, last.y, last.z);
      trajectory_alignment_pending = true;
      trajectory_alignment_active = false;
      trajectory_alignment_offset = Eigen::Vector3d::Zero();
      trajectory_alignment_source_position = Eigen::Vector3d::Zero();
      trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
    } else {
      trajectory_alignment_pending = false;
      trajectory_alignment_active = false;
      trajectory_alignment_anchor = Eigen::Vector3d::Zero();
      trajectory_alignment_offset = Eigen::Vector3d::Zero();
      trajectory_alignment_source_position = Eigen::Vector3d::Zero();
      trajectory_alignment_rotation = Eigen::Matrix3d::Identity();
    }
  }

  {
    std::lock_guard<std::mutex> sys_lck(sys_mtx);
    // 重新创建 VIO，避免已经异常的估计状态在继续后沿着原来的方向继续漂。
    sys = nullptr;
  }
  trajectory_recovery_user_state.store(3);
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
