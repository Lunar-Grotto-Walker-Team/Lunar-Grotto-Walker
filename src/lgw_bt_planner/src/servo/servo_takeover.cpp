// src/servo/servo_takeover.cpp
//
// Takeover sequencing, the MoveGroup pre-position glue and the service
// handlers (including the SessionHost trampolines ServoSession calls back
// through). Split out of servo_node.hpp (2026-11-12); bodies moved verbatim
// apart from whitespace and the ServoNode:: qualifier.

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace lgw_bt_planner
{

bool ServoNode::maybe_preposition(std::string & error)
{
  if (!preposition_params_.enable) {
    return true;
  }
  if (!prepositioner_) {
    error = "preposition: move-group client not initialised";
    return false;
  }

  tf2::Vector3 target;
  std::array<double, kArmJointCount> q_now{};
  Pose6 tcp_now;
  PostureResult family;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_selected_snapshot_) {
      error = "preposition: no target selected";
      return false;
    }
    if (!measured_valid_) {
      error = "preposition: no joint state on pose_watch";
      return false;
    }
    if (!selected_snapshot_.geometry_valid) {
      error = "preposition: no drivable geometry for the selected target";
      return false;
    }
    target = selected_snapshot_.p_arm_base;
    q_now = measured_positions_;
    if (!lookup_tcp_locked(tcp_now)) {
      error = "preposition: tcp pose unavailable";
      return false;
    }
    try {
      family = solver_->solve(
        target, R_cam_in_tcp_, t_cam_in_tcp_, posture_params_);
    } catch (const std::exception & e) {
      error = std::string("preposition: ") + e.what();
      return false;
    }
  }

  const PrepositionDecision before = decide_preposition(
    family.joints, q_now, family.tcp_desired, tcp_now, preposition_params_);
  max_joint_delta_ = before.max_joint_delta;
  needs_preposition_ = before.needed;
  if (!before.needed) {
    RCLCPP_INFO(
      get_logger(),
      "[preposition] skipped (%s): joint_delta=%.3f q5_delta=%.3f "
      "limit_margin=%.3f tcp=%.3fm/%.3frad",
      before.reason.c_str(), before.max_joint_delta, before.q5_delta,
      before.min_limit_margin, before.tcp_distance, before.tcp_angle);
    return true;
  }

  RCLCPP_INFO(
    get_logger(),
    "[preposition] needed (%s): joint_delta=%.3f q5_delta=%.3f "
    "limit_margin=%.3f tcp=%.3fm/%.3frad -> moving to the family pose",
    before.reason.c_str(), before.max_joint_delta, before.q5_delta,
    before.min_limit_margin, before.tcp_distance, before.tcp_angle);

  std::array<std::string, kArmJointCount> names;
  for (std::size_t i = 0; i < kArmJointCount; ++i) {
    names[i] = kArmJoints[i];
  }
  const auto timeout = std::chrono::milliseconds(
    static_cast<int64_t>(preposition_timeout_s_ * 1000.0));

  RCLCPP_INFO(
    get_logger(),
    "[preposition] target q = [%.3f %.3f %.3f %.3f %.3f %.3f] (family ok=%d)",
    family.joints[0], family.joints[1], family.joints[2],
    family.joints[3], family.joints[4], family.joints[5],
    static_cast<int>(family.ok));
  preposition_in_progress_.store(true);
  const bool moved = prepositioner_->move_to(
    family.joints, names, preposition_velocity_scaling_,
    preposition_accel_scaling_, preposition_tolerance_rad_, timeout, error);
  preposition_in_progress_.store(false);
  if (!moved) {
    error = "preposition failed: " + error;
    return false;
  }

  // Let the controller settle before measuring. MoveGroup reports success
  // when the JTC finishes the trajectory, but the controller has a short
  // tail (and, in the mock, the state can still be settling); unpausing
  // Servo inside that window makes the first Servo tick see a stale or
  // reset state (measured 2026-10-31: Servo reported the arm at q = 0 right
  // after a move it had just completed).
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // The arm has tracking error of its own; re-measure before claiming the
  // family is reached.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q_now = measured_positions_;
    lookup_tcp_locked(tcp_now);
  }
  const PrepositionDecision after = decide_preposition(
    family.joints, q_now, family.tcp_desired, tcp_now, preposition_params_);
  max_joint_delta_ = after.max_joint_delta;
  if (after.needed) {
    error = "preposition: still outside the family after the move (" +
      after.reason + ")";
    return false;
  }
  return true;
}

void ServoNode::refuse_takeover_to_idle(const std::shared_ptr<lgw_bt_planner::srv::Takeover::Response> & response, const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sm_.handle(ServoEvent::RELEASE);
    state_dirty_ = true;
  }
  call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
  std::lock_guard<std::mutex> lock(mutex_);
  if (sm_.state() == ServoState::RELEASING) {
    sm_.handle(ServoEvent::PAUSE_OK);
    state_dirty_ = true;
  }
  response->success = false;
  response->message = message;
  response->state = state_name(sm_.state());
  RCLCPP_WARN(
    get_logger(), "takeover refused: %s — state -> %s", message.c_str(),
    state_name(sm_.state()));
}

void ServoNode::handle_takeover(const std::shared_ptr<lgw_bt_planner::srv::Takeover::Request> request, std::shared_ptr<lgw_bt_planner::srv::Takeover::Response> response)
{
  response->success = false;

  std::string reason;
  bool accepted = false;
  bool scan_only = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    accepted = try_takeover_locked(*request, reason, scan_only);
    response->state = state_name(sm_.state());
  }

  if (!accepted) {
    response->message = reason;
    RCLCPP_WARN(get_logger(), "takeover refused: %s", reason.c_str());
    return;
  }

  if (scan_only) {
    // BT start with no target: enter SCANNING immediately. The scan worker
    // pauses Servo (a no-op if it was never started) and drives MoveGroup.
    std::lock_guard<std::mutex> lock(mutex_);
    hold_reason_ = HoldReason::kTargetLost;
    hold_since_ns_ = now().nanoseconds();
    scan_session_active_ = true;
    if (!sm_.handle(ServoEvent::SCAN_START)) {
      response->message = "scan start rejected";
      return;
    }
    scan_in_progress_.store(true);
    scan_target_found_.store(false);
    scan_cancel_.store(false);
    scan_started_ns_ = now().nanoseconds();
    if (scan_thread_.joinable()) {
      scan_thread_.join();
    }
    scan_thread_ = std::thread(&ServoNode::scan_worker, this);
    response->success = true;
    response->message = "scanning (no target)";
    response->state = state_name(sm_.state());
    state_dirty_ = true;
    RCLCPP_INFO(
      get_logger(), "takeover: no target -> %s", state_name(sm_.state()));
    return;
  }

  // Pre-position (spec 2026-10-30 §3.8 rev3): the arm may be anywhere when
  // the session starts, including the other wrist branch where Servo would
  // integrate straight through q5 = 0. Decide, and if needed move the arm
  // BEFORE handing control to Servo. This blocks, so it must not hold the
  // control mutex — move_to() spins its own executor.
  {
    std::string preposition_error;
    if (!maybe_preposition(preposition_error)) {
      refuse_takeover_to_idle(response, preposition_error);
      return;
    }
  }

  // Re-seed Servo's cached command BEFORE handing it control
  // (docs/servo/2026-11-02-servo-state-monitor-stuck.md).
  //
  // moveit_servo fills `last_sent_command_` exactly once per
  // ServoCalcs::start(), from the state monitor at that instant, and
  // republishes it verbatim on every all-zero twist. Our boot path starts
  // Servo with the arm at home, so that cache is the all-zero home command;
  // the preposition above moves the arm to the posture family without
  // touching the cache, and the first all-zero twist after unpause then
  // commands the old zeros. The JTC executes them and yanks the arm back to
  // the wrist singularity. Re-running start_servo here (Servo is still
  // paused) re-seeds the cache from the post-preposition state.
  {
    const TriggerOutcome reseed =
      call_trigger_sync(reseed_client_, "start_servo(reseed)", kReseedTimeoutSec);
    if (!reseed.ok) {
      // start_servo unpauses Servo before it re-seeds; the helper pauses
      // and returns the state machine to IDLE.
      refuse_takeover_to_idle(response, reseed.detail);
      return;
    }

    // start_servo's response is always success (spec §7.9); prove the seed
    // while still in PREEMPT (the control law is silent).
    std::array<double, kArmJointCount> expected{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expected = measured_positions_;
    }
    std::string probe_error;
    if (!verify_servo_seed(expected, 0.10, probe_error)) {
      refuse_takeover_to_idle(
        response, "reseed verification failed: " + probe_error);
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Servo command cache re-seeded from the prepositioned pose");
  }

  // PREEMPT -> block on unpause -> TRACKING / FAULT.
  const TriggerOutcome outcome =
    call_trigger_sync(unpause_client_, "unpause_servo", kUnpauseTimeoutSec);

  std::lock_guard<std::mutex> lock(mutex_);
  state_dirty_ = true;
  if (outcome.ok) {
    sm_.handle(ServoEvent::UNPAUSE_OK);
    arm_saturation_window_locked();
    response->success = true;
    response->message = "tracking (" + reason + ")";
    RCLCPP_INFO(
      get_logger(), "takeover: track_id=%lu reason=%s -> %s",
      static_cast<unsigned long>(tracking_track_id_), reason.c_str(),
      state_name(sm_.state()));
  } else {
    sm_.handle(ServoEvent::UNPAUSE_FAIL);
    response->message = outcome.detail;
    RCLCPP_ERROR(
      get_logger(), "takeover failed: %s — state -> %s",
      outcome.detail.c_str(), state_name(sm_.state()));
  }
  response->state = state_name(sm_.state());
}

bool ServoNode::try_takeover_locked(
  const lgw_bt_planner::srv::Takeover::Request & request,
  std::string & reason, bool & scan_only)
{
  scan_only = false;
  if (sm_.state() == ServoState::FAULT) {
    reason = calibration_ok_ ? "fault (RESET requires a node restart)" :
      "calibration unavailable (fault)";
    return false;
  }
  if (sm_.state() != ServoState::IDLE) {
    reason = std::string("not idle (") + state_name(sm_.state()) + ")";
    return false;
  }
  sm_.set_move_group_busy(move_group_busy_locked());
  if (move_group_busy_locked()) {
    reason = "move_group busy";
    return false;
  }

  if (request.track_id != 0 || !request.class_filter.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Takeover.srv track_id/class_filter are deprecated and ignored; "
      "servo follows /perception/tracks/unified selected_track_id");
  }

  const uint64_t selected_id =
    latest_array_ ? latest_array_->selected_track_id : 0;

  // BT start with no selected target: enter SCANNING immediately. This path
  // deliberately does not require Servo to be started (the scan worker drives
  // MoveGroup, not Servo).
  if (selected_id == 0) {
    scan_only = true;
    if (!sm_.handle(ServoEvent::TAKEOVER)) {
      reason = "takeover rejected";
      return false;
    }
    return true;
  }

  // The selected id must exist before the Servo-started check, so callers can
  // tell "no such target" apart from "Servo not started".
  const yoloe_interfaces::msg::UnifiedTrack * selected = nullptr;
  if (latest_array_) {
    for (const auto & track : latest_array_->tracks) {
      if (track.track_id == selected_id) {
        selected = &track;
        break;
      }
    }
  }
  if (selected == nullptr) {
    reason = "selected_missing";
    return false;
  }

  if (start_state_ != StartState::STARTED) {
    reason = "servo not started";
    return false;
  }

  selected_snapshot_ = make_snapshot(*selected);
  has_selected_snapshot_ = true;
  tracking_track_id_ = selected_id;

  // New session: forget the control-law filter memory and the saturation
  // window.
  // The control-law filter and the feed-forward derivative both describe the
  // previous session's target; differentiating across the gap would command
  // its velocity on the first tick of the new one.
  loop_.reset();
  arm_saturation_window_locked();

  if (!sm_.handle(ServoEvent::TAKEOVER)) {
    reason = "takeover rejected";
    return false;
  }
  state_dirty_ = true;
  reason = "selected";
  return true;
}

void ServoNode::handle_release(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  const SessionResult result = servo_session_->release();
  response->success = result.success;
  response->message = result.message;
}

void ServoNode::handle_pause(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  const SessionResult result = servo_session_->pause();
  response->success = result.success;
  response->message = result.message;
}

void ServoNode::handle_resume(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  const SessionResult result = servo_session_->resume();
  response->success = result.success;
  response->message = result.message;
}

void ServoNode::under_lock(const std::function<void()> & critical)
{
  std::lock_guard<std::mutex> lock(mutex_);
  critical();
}

void ServoNode::enter_hold(HoldReason reason)
{
  // Called under under_lock(), exactly like the handler it came from.
  enter_hold_locked(reason, now().nanoseconds());
}

void ServoNode::cancel_preposition_unless_recovery()
{
  if (preposition_in_progress_.load() && !recovery_in_progress_.load() &&
    prepositioner_)
  {
    prepositioner_->cancel();
  }
}

std::array<double, 6> ServoNode::measured_positions()
{
  return measured_positions_;
}

bool ServoNode::verify_seed(const std::array<double, 6> & expected, double tolerance, std::string & error)
{
  return verify_servo_seed(expected, tolerance, error);
}

RpcOutcome ServoNode::rpc(ServoRpc which, const char * tag)
{
  TriggerOutcome outcome;
  switch (which) {
    case ServoRpc::kPause:
      outcome = call_trigger_sync(pause_client_, tag, kPauseTimeoutSec);
      break;
    case ServoRpc::kUnpause:
      outcome = call_trigger_sync(unpause_client_, tag, kUnpauseTimeoutSec);
      break;
    case ServoRpc::kReseed:
      outcome = call_trigger_sync(reseed_client_, tag, kReseedTimeoutSec);
      break;
  }
  RpcOutcome result;
  result.ok = outcome.ok;
  result.detail = outcome.detail;
  return result;
}

ServoStateMachine & ServoNode::sm()
{
  return sm_;
}

void ServoNode::clear_session()
{
  clear_session_locked();
}

void ServoNode::arm_saturation_window()
{
  arm_saturation_window_locked();
}

void ServoNode::mark_dirty()
{
  state_dirty_ = true;
}

void ServoNode::arm_recovery_cancel()
{
  recovery_cancel_.store(true);
}

}  // namespace lgw_bt_planner
