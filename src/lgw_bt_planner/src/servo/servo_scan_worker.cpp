// src/servo/servo_scan_worker.cpp
// SCANNING state worker (2026-11-13).  The node pauses Servo, drives a
// joint-space scan through MoveGroup, and resumes tracking when a drivable
// target appears.

#include "lgw_bt_planner/servo/servo_node.hpp"

#include "lgw_bt_planner/servo/scan_roll_lock.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace lgw_bt_planner
{

void ServoNode::force_hold_for_test(HoldReason reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sm_.handle(ServoEvent::TAKEOVER);
  sm_.handle(ServoEvent::UNPAUSE_OK);
  sm_.handle(ServoEvent::TARGET_LOST);
  hold_reason_ = reason;
  hold_since_ns_ = now().nanoseconds() - 2'000'000'000LL;
  pose_seen_ = true;
  pose_recv_ns_ = now().nanoseconds();
}

bool ServoNode::base_is_static_locked(int64_t now_ns)
{
  const auto T = lookup_odom_base_locked();
  if (!T) {
    return false;
  }
  const tf2::Vector3 p = T->getOrigin();
  if (last_base_sample_ns_ == 0) {
    last_base_sample_ns_ = now_ns;
    last_base_pos_ = p;
    return false;  // need a second sample
  }
  const int64_t elapsed_ns = now_ns - last_base_sample_ns_;
  if (elapsed_ns < 100'000'000LL) {
    return false;  // not enough motion history to certify "static"
  }
  const double moved = (p - last_base_pos_).length();
  last_base_sample_ns_ = now_ns;
  last_base_pos_ = p;
  return moved < 0.02;
}

void ServoNode::maybe_start_scan_locked(int64_t now_ns, bool target_ok)
{
  if (!auto_scan_ || target_ok) {
    return;
  }
  if (sm_.state() != ServoState::HOLD ||
    hold_reason_ != HoldReason::kTargetLost)
  {
    return;  // operator pause / follower lag / stale pose / tracker death
  }
  if (scan_in_progress_.load()) {
    return;
  }
  if (now_ns - hold_since_ns_ < scan_start_delay_ms_ * 1'000'000LL) {
    return;
  }
  if (!scan_session_active_ &&
    watchdog_.timed_out(now_ns, tracker_stream_timeout_ms_))
  {
    return;
  }
  if (!pose_seen_ || (now_ns - pose_recv_ns_) > kPoseWatchTimeoutNs) {
    return;
  }
  if (!measured_complete_locked()) {
    return;
  }
  if (scan_require_base_static_ && !base_is_static_locked(now_ns)) {
    return;
  }
  if (!sm_.handle(ServoEvent::SCAN_START)) {
    return;
  }

  scan_in_progress_.store(true);
  scan_target_found_.store(false);
  scan_cancel_.store(false);
  scan_started_ns_ = now_ns;
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }
  scan_thread_ = std::thread(&ServoNode::scan_worker, this);
  RCLCPP_INFO(get_logger(), "[scan] started");
}

bool ServoNode::scan_worker_aborted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return scan_cancel_.load() || sm_.state() != ServoState::SCANNING;
}

void ServoNode::publish_scan_stop()
{
  // Hold where the arm IS, not where it was last commanded: the JTC checks
  // every incoming trajectory's first point against its own (real-arm) state
  // and rejects a mismatch beyond allowed_start_tolerance (~0.01 rad).
  std::array<double, kArmJointCount> q{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = measured_positions_;
  }
  std::array<std::string, kArmJointCount> names;
  for (size_t i = 0; i < kArmJointCount; ++i) {
    names[i] = kArmJoints[i];
  }
  scan_traj_pub_->publish(
    make_hold_trajectory(q, names, scan_hold_s_, now()));
}

void ServoNode::scan_worker()
{
  call_trigger_sync(pause_client_, "pause_servo(scan)", kPauseTimeoutSec);
  preposition_in_progress_.store(true);

  // Pre-settle: MoveGroup plans from /feedback while the JTC holds
  // /control; without this the first goal can be rejected on
  // allowed_start_tolerance.
  int64_t settled_since_ns = 0;
  while (true) {
    if (scan_worker_aborted()) {
      finish_scan_timeout();
      return;
    }
    bool ready = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double gap = follow_gap_locked();
      // scan_presettle_gap_rad_(默认 0.15)而不是 recovery 的 0.005:后者对
      // 这套 follow:=true 的系统物理上不可达(正常跟随差 0.1-0.5 rad),只会
      // 让每次扫描白等满 recovery_presettle_timeout_ms。
      ready = !follow_check_enabled_ || !commanded_valid_ || !measured_valid_ ||
        !std::isfinite(gap) || gap <= scan_presettle_gap_rad_;
    }
    if (ready) {
      const int64_t now_ns = now().nanoseconds();
      if (settled_since_ns == 0) {
        settled_since_ns = now_ns;
      } else if (now_ns - settled_since_ns >=
        recovery_presettle_stable_ms_ * 1'000'000LL)
      {
        break;
      }
    } else {
      settled_since_ns = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::array<std::string, kArmJointCount> names;
  for (size_t i = 0; i < kArmJointCount; ++i) {
    names[i] = kArmJoints[i];
  }

  // SCANNING is a state, not a bounded attempt: the wrist keeps drawing the
  // Lissajous until a target appears or the session ends.  One trajectory
  // covers several whole figures, so the seam comes once per trajectory.
  bool first_cycle = true;
  while (true) {
    if (scan_worker_aborted() || scan_target_found_.load()) {
      break;
    }

    std::array<double, kArmJointCount> measured{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      measured = measured_positions_;
    }
    if (first_cycle) {
      // ONE collision-checked MoveGroup move gets the arm to the figure's
      // centre (j4 = j5 = 0).  Later trajectories start there by themselves.
      const auto probe = make_scan_circle(
        measured, scan_circle_params_, now());
      if (probe.points.empty()) {
        RCLCPP_ERROR(get_logger(), "[scan] empty scan figure (check params)");
        publish_scan_stop();
        finish_scan_timeout();
        return;
      }
      std::array<double, kArmJointCount> start_q{};
      for (size_t j = 0; j < kArmJointCount; ++j) {
        start_q[j] = probe.points.front().positions[j];
      }
      if (!skip_ingress_for_test_.load()) {
        std::string error;
        if (!prepositioner_->move_to(
            start_q, names, scan_velocity_scaling_, scan_accel_scaling_,
            scan_tolerance_rad_, std::chrono::milliseconds(5000), error))
        {
          RCLCPP_WARN(get_logger(), "[scan] ingress failed: %s", error.c_str());
          publish_scan_stop();
          finish_scan_timeout();
          return;
        }
      }
      first_cycle = false;
      if (scan_worker_aborted() || scan_target_found_.load()) {
        break;
      }
    }

    auto traj = make_scan_circle(
      measured, scan_circle_params_, now());
    if (roll_lock_enable_ && !traj.points.empty()) {
      // Keep the image upright: j4/j5 roll the camera, j6 cancels it.  The
      // reference is the figure centre, where the scanner starts.
      const auto roll_of = [this](const std::array<double, 6> & q) {
          tf2::Vector3 p_cam;
          tf2::Matrix3x3 R_cam;
          std::string error;
          if (!posture_.cameraPose(
              q, R_cam_in_tcp_, t_cam_in_tcp_, p_cam, R_cam, error))
          {
            return 0.0;  // best effort: leave the sample unlocked
          }
          return image_roll(R_cam);
        };
      // "Level" is the requirement: the image's up vector must stay aligned
      // with the world's up, not merely constant.  Locking to the pose's own
      // roll would keep the image steadily tilted.
      const double roll_ref = 0.0;
      lock_camera_roll(traj, roll_of, roll_ref, -1.99, 1.99);
      // Diagnostic: how far the locked samples really land from the target.
      double max_residual = 0.0;
      std::array<double, 6> probe{};
      for (const auto & pt : traj.points) {
        for (size_t j = 0; j < kArmJointCount; ++j) {
          probe[j] = pt.positions[j];
        }
        max_residual = std::max(
          max_residual, std::abs(std::remainder(roll_of(probe) - roll_ref, 2.0 * M_PI)));
      }
      RCLCPP_INFO(
        get_logger(), "[scan] roll lock: ref=%.3f rad (%.0f deg), max residual=%.3f rad (%.1f deg)",
        roll_ref, roll_ref * 180.0 / M_PI, max_residual,
        max_residual * 180.0 / M_PI);
    }
    if (traj.points.size() < 2) {
      RCLCPP_ERROR(
        get_logger(), "[scan] trajectory generation produced nothing");
      publish_scan_stop();
      finish_scan_timeout();
      return;
    }
    const double duration_s =
      static_cast<double>(traj.points.back().time_from_start.sec) +
      1e-9 * static_cast<double>(traj.points.back().time_from_start.nanosec);
    scan_traj_pub_->publish(traj);
    RCLCPP_INFO(
      get_logger(),
      "[scan] circle published: %zu points, %.2f s (j4 r=%.2f, j5 r=%.2f)",
      traj.points.size(), duration_s,
      scan_circle_params_.j4_radius_rad,
      scan_circle_params_.j5_radius_rad);

    const int64_t end_ns = now().nanoseconds() +
      static_cast<int64_t>(duration_s * 1e9);
    while (true) {
      if (scan_worker_aborted() || scan_target_found_.load()) {
        break;
      }
      if (now().nanoseconds() > end_ns) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (scan_worker_aborted() || scan_target_found_.load()) {
      break;
    }

    // The JTC rejects a trajectory whose first point is far from its state,
    // and the arm trails the command by a few milliradians.  The figure ends
    // at its centre, so waiting for the arm to catch up costs a fraction of
    // a second once per trajectory (not per sweep).
    const int64_t settle_deadline = now().nanoseconds() + 500'000'000LL;
    while (now().nanoseconds() < settle_deadline) {
      std::array<double, kArmJointCount> q{};
      {
        std::lock_guard<std::mutex> lock(mutex_);
        q = measured_positions_;
      }
      bool close = true;
      for (size_t j = 0; j < kArmJointCount; ++j) {
        close = close &&
          std::abs(q[j] - traj.points.front().positions[j]) <= 5e-3;
      }
      if (close || scan_worker_aborted()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Stop whatever the JTC is still executing before the (blocking) handover
  // back to Servo.  A completed cycle is simply followed by the next one;
  // only a target / cancel / failure reaches this point.
  publish_scan_stop();
  if (scan_target_found_.load() && !scan_worker_aborted()) {
    finish_scan_found();
  } else {
    finish_scan_timeout();
  }
}

void ServoNode::finish_scan_found()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sm_.state() != ServoState::SCANNING ||
      !sm_.handle(ServoEvent::SCAN_FOUND))
    {
      scan_in_progress_.store(false);
      preposition_in_progress_.store(false);
      return;
    }
    scan_in_progress_.store(false);
    preposition_in_progress_.store(false);
    // A target is now driving the session; future scans must pass the
    // normal stream/health gate again.
    scan_session_active_ = false;
    state_dirty_ = true;
  }

  auto park_and_fault = [this](const char * detail) {
      call_trigger_sync(pause_client_, "pause_servo(scan-fail)", kPauseTimeoutSec);
      std::lock_guard<std::mutex> lock(mutex_);
      sm_.handle(ServoEvent::UNPAUSE_FAIL);  // PREEMPT -> FAULT
      state_dirty_ = true;
      RCLCPP_ERROR(get_logger(), "[scan] %s", detail);
    };

  if (scan_cancel_.load()) {
    park_and_fault("cancelled after SCAN_FOUND");
    return;
  }

  // 扫描刚停下时 JTC 还在减速。这段等待原本是为了绕开 verify_servo_seed 的
  // 跨域比较(2026-11-16 已改成比指令域,臂动不再影响判定),所以只留一个
  // 短兜底:等臂基本静止最多 150 ms,重点是不让交接变慢。
  {
    constexpr int64_t kSettleDeadlineNs = 150'000'000LL;   // 最多等 150 ms
    constexpr double kSettleVelocityRadS = 0.05;
    const int64_t deadline = now().nanoseconds() + kSettleDeadlineNs;
    while (now().nanoseconds() < deadline) {
      double vmax = 0.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < kArmJointCount; ++i) {
          vmax = std::max(vmax, std::fabs(measured_velocities_[i]));
        }
      }
      if (vmax < kSettleVelocityRadS) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // 播种 + 校验,失败重试一次:瞬时抖动不该直接把整个任务打成 FAULT。
  // 真正播种失败(重试后仍不过)依旧 FAULT —— 宁停勿错。
  constexpr int kSeedAttempts = 2;
  std::string error{"re-seed never attempted"};
  bool seeded = false;
  for (int attempt = 1; attempt <= kSeedAttempts && !seeded; ++attempt) {
    if (scan_cancel_.load()) {
      park_and_fault("cancelled during re-seed");
      return;
    }
    const TriggerOutcome reseed = call_trigger_sync(
      reseed_client_, "start_servo(scan)", kReseedTimeoutSec);
    if (!reseed.ok) {
      error = "re-seed failed";
      continue;
    }
    std::array<double, kArmJointCount> expected{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expected = measured_positions_;
    }
    if (verify_servo_seed(expected, 0.10, error)) {
      seeded = true;
      break;
    }
    RCLCPP_WARN(
      get_logger(), "[scan] seed verify attempt %d/%d failed: %s",
      attempt, kSeedAttempts, error.c_str());
  }
  if (!seeded) {
    park_and_fault(("seed verify failed: " + error).c_str());
    return;
  }
  if (scan_cancel_.load()) {
    park_and_fault("cancelled during seed verify");
    return;
  }

  const TriggerOutcome outcome = call_trigger_sync(
    unpause_client_, "unpause_servo(scan)", kUnpauseTimeoutSec);
  std::lock_guard<std::mutex> lock(mutex_);
  sm_.handle(
    outcome.ok ? ServoEvent::UNPAUSE_OK : ServoEvent::UNPAUSE_FAIL);
  state_dirty_ = true;
  RCLCPP_INFO(
    get_logger(), "[scan] target found -> %s", state_name(sm_.state()));
}

void ServoNode::finish_scan_timeout()
{
  // The worker only leaves SCANNING on a target, a cancel or a genuine
  // failure (ingress/trajectory generation); a completed sweep simply
  // starts the next one.
  call_trigger_sync(pause_client_, "pause_servo(scan-end)", kPauseTimeoutSec);
  std::lock_guard<std::mutex> lock(mutex_);
  scan_in_progress_.store(false);
  preposition_in_progress_.store(false);
  if (sm_.state() == ServoState::SCANNING) {
    sm_.handle(ServoEvent::SCAN_TIMEOUT);  // -> HOLD
  }
  state_dirty_ = true;
  RCLCPP_INFO(get_logger(), "[scan] ended (%s)", state_name(sm_.state()));
}

void ServoNode::cancel_scan()
{
  scan_cancel_.store(true);
  if (prepositioner_) {
    prepositioner_->cancel();
  }
}

}  // namespace lgw_bt_planner

namespace lgw_bt_planner
{

// Test hooks: drive the takeover/selection path without a ROS service call.
void ServoNode::set_latest_array_for_test(
  const yoloe_interfaces::msg::UnifiedTrackArray & arr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_array_ = std::make_shared<yoloe_interfaces::msg::UnifiedTrackArray>(arr);
}

bool ServoNode::takeover_for_test(
  uint64_t track_id, const std::string & class_filter, bool & scan_only)
{
  std::lock_guard<std::mutex> lock(mutex_);
  lgw_bt_planner::srv::Takeover::Request request;
  request.track_id = track_id;
  request.class_filter = class_filter;
  std::string reason;
  return try_takeover_locked(request, reason, scan_only);
}

bool ServoNode::takeover_service_for_test(
  uint64_t track_id, const std::string & class_filter, std::string & message)
{
  auto request = std::make_shared<lgw_bt_planner::srv::Takeover::Request>();
  request->track_id = track_id;
  request->class_filter = class_filter;
  auto response = std::make_shared<lgw_bt_planner::srv::Takeover::Response>();
  handle_takeover(request, response);
  message = response->message;
  return response->success;
}

}  // namespace lgw_bt_planner
