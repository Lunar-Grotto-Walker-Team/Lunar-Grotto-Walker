// src/servo/servo_tick.cpp
//
// The 100 Hz control tick: refusal rules, the look-at solve, twist assembly
// and the Servo start path. Split out of servo_node.hpp (2026-11-12); the
// bodies moved verbatim apart from whitespace and the ServoNode:: qualifier.

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace lgw_bt_planner
{

void ServoNode::tick(int64_t now_ns)
{
  const rclcpp::Time stamp(now_ns, RCL_ROS_TIME);
  ++tick_count_;

  // RELEASE / FAULT move the state away from SCANNING; make sure the scan
  // worker stops even if the release's pause RPC failed before
  // clear_session_locked() ran.
  if (sm_.state() != ServoState::SCANNING && scan_in_progress_.load()) {
    scan_cancel_.store(true);
    if (prepositioner_) {
      prepositioner_->cancel();
    }
  }

  bool need_pause = false;
  bool need_startup_pause = false;
  bool need_follower_resume = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (sm_.state() != ServoState::FAULT) {
      maybe_start_servo_locked(now_ns);
    }
    maybe_check_camera_tf_locked();
    if (pause_on_confirm_requested_) {
      pause_on_confirm_requested_ = false;
      need_startup_pause = true;
    }
    if (sm_.state() == ServoState::TRACKING || sm_.state() == ServoState::HOLD) {
      need_pause = update_tracking_locked(stamp, now_ns);
      need_follower_resume = follow_resume_requested_;
      follow_resume_requested_ = false;
    } else if (sm_.state() == ServoState::RECOVERING) {
      // Keep the RESUME latch warm from the live target: the worker's
      // RECOVER_OK only returns to TRACKING when the target is present.
      const bool stream_alive =
        !watchdog_.timed_out(now_ns, tracker_stream_timeout_ms_);
      if (calibration_ok_ && has_selected_snapshot_ && stream_alive &&
        selected_snapshot_.geometry_valid)
      {
        sm_.handle(ServoEvent::TARGET_OK);
      }
      // Overall recovery budget: cancel an overrunning MoveGroup goal.
      // A non-positive deadline means recovery is DISABLED (spec §9), so
      // there is nothing to cancel -- without this guard a deadline of 0
      // would cancel every move on its first tick.
      if (recovery_in_progress_.load() &&
        recovery_params_.deadline_ms > 0 &&
        now_ns - hold_since_ns_ > recovery_params_.deadline_ms * 1000000LL)
      {
        recovery_cancel_.store(true);
        if (prepositioner_) {
          prepositioner_->cancel();
        }
      }
    }
  }

  if (auto_resume_requested_.exchange(false)) {
    // HOLD + TARGET_OK + auto_resume: reuse the service's re-seed/unpause
    // sequence outside the control mutex.
    const SessionResult auto_resume = servo_session_->resume();
    if (!auto_resume.success) {
      RCLCPP_WARN(
        get_logger(), "auto-resume failed: %s", auto_resume.message.c_str());
    }
  }

  if (need_pause || need_startup_pause) {
    // Blocking, but outside mutex_ and on its own callback group.
    const TriggerOutcome outcome =
      call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!outcome.ok) {
      sm_.handle(ServoEvent::FAULT);
      if (need_startup_pause) {
        start_state_ = StartState::FAILED;
      }
      RCLCPP_ERROR(
        get_logger(), "%s — state -> FAULT (Servo not pausable)",
        outcome.detail.c_str());
    } else if (need_startup_pause) {
      // IDLE is now genuinely passive: Servo's calc thread is paused, so it
      // stops publishing to JTC until a takeover unpauses it (spec §5.3).
      start_state_ = StartState::STARTED;
      RCLCPP_INFO(
        get_logger(), "Servo started and parked (paused); takeover to drive it");
    }
  }

  if (need_follower_resume) {
    // Blocking call outside mutex_, same convention as pause_servo.
    const TriggerOutcome outcome = call_trigger_sync(
      unpause_client_, "unpause_servo(follower)", kUnpauseTimeoutSec);
    bool target_ok = false;
    bool resumed = false;
    bool still_holding = false;
    std::string state_after;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_ok = sm_.target_ok();
      if (outcome.ok && target_ok) {
        follow_resume_since_ns_ = 0;
        resumed = sm_.handle(ServoEvent::RESUME);
      }
      still_holding = sm_.state() == ServoState::HOLD;
      state_after = state_name(sm_.state());
      state_dirty_ = true;
    }
    switch (decide_follower_resume(
        still_holding, outcome.ok, target_ok, resumed))
    {
      case FollowerResumeOutcome::kResumed:
        RCLCPP_INFO(
          get_logger(), "follower_lag recovered: gap closed -> TRACKING");
        break;
      case FollowerResumeOutcome::kFault:
        {
          std::lock_guard<std::mutex> lock(mutex_);
          sm_.handle(ServoEvent::FAULT);
          state_dirty_ = true;
        }
        RCLCPP_ERROR(get_logger(), "follower_lag resume failed -> FAULT");
        break;
      case FollowerResumeOutcome::kCancelled:
        // A release took the session away while the unpause was in flight.
        // If the unpause landed after it, park Servo again so IDLE never
        // holds an unpaused Servo.
        if (outcome.ok) {
          call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
        }
        RCLCPP_INFO(
          get_logger(), "follower_lag resume cancelled: state %s",
          state_after.c_str());
        break;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  publish_state_locked(stamp);
}

bool ServoNode::update_tracking_locked(const rclcpp::Time & stamp, int64_t now_ns)
{
  const bool stream_alive = !watchdog_.timed_out(now_ns, tracker_stream_timeout_ms_);
  const bool pose_alive = pose_seen_ && (now_ns - pose_recv_ns_) <= kPoseWatchTimeoutNs;
  const bool busy = move_group_busy_locked();
  sm_.set_move_group_busy(busy);
  const bool servo_halted = servo_halt_persisted_locked(now_ns);

  const TrackSnapshot & snap = selected_snapshot_;
  const bool target_ok = calibration_ok_ && has_selected_snapshot_ &&
    stream_alive && snap.geometry_valid;
  // Following monitor (spec §5.5): Servo integrates on the JTC/mock domain, so
  // the real arm is a follower. If it stops following, the commanded pose no
  // longer describes the arm and the control law is driving blind.
  bool not_following = false;
  if (follow_check_enabled_) {
    const double gap = follow_gap_locked();
    const bool stale = commanded_valid_ &&
      (now_ns - commanded_recv_ns_) > kPoseWatchTimeoutNs;
    if (std::isfinite(gap) && gap > follow_tolerance_rad_) {
      if (follow_bad_since_ns_ == 0) {
        follow_bad_since_ns_ = now_ns;
      }
      not_following =
        (now_ns - follow_bad_since_ns_) >= follow_timeout_ms_ * 1'000'000LL;
      if (!not_following &&
        (now_ns - follow_warn_last_ns_) >= kFollowWarnPeriodNs)
      {
        follow_warn_last_ns_ = now_ns;
        RCLCPP_WARN(
          get_logger(),
          "arm following the commanded pose loosely: gap=%.3f rad (tolerance "
          "%.3f)", gap, follow_tolerance_rad_);
      }
    } else {
      follow_bad_since_ns_ = 0;
    }
    not_following = not_following || stale;
  }

  if (sm_.state() == ServoState::TRACKING) {
    if (!target_ok || busy || servo_halted || !pose_alive || not_following) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TRACKING -> HOLD (target_ok=%d busy=%d servo_halted=%d pose_alive=%d "
        "stream_alive=%d not_following=%d)", target_ok, busy, servo_halted,
        pose_alive, stream_alive, static_cast<int>(not_following));
      return hold_and_return(
        classify_hold(
          target_ok, stream_alive, busy, servo_halted, pose_alive,
          not_following), now_ns);
    }
    // Keeps the RESUME latch warm while the target is demonstrably present.
    sm_.handle(ServoEvent::TARGET_OK);
  } else {  // HOLD
    if (target_ok) {
      sm_.handle(ServoEvent::TARGET_OK);
    }
    if (auto_resume_on_target_ && target_ok &&
      hold_reason_ == HoldReason::kTargetLost)
    {
      // Only a reappearing target may auto-resume. follower_lag /
      // move_group_busy / servo_halt must keep their own recovery paths.
      auto_resume_requested_.store(true);
    }
    if (hold_reason_ == HoldReason::kFollowerLag) {
      update_follower_recovery_locked(now_ns);
    } else {
      maybe_start_recovery_locked(now_ns, target_ok, snap.p_arm_base);
      if (!target_ok) {
        maybe_start_scan_locked(now_ns, target_ok);
      }
    }
    return false;
  }

  // --- current tcp pose (TF, zero-timeout probe first) ---
  Pose6 tcp_now;
  if (!lookup_tcp_locked(tcp_now)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TF %s <- %s unavailable; HOLD", base_frame_.c_str(), tcp_frame_.c_str());
    // No TCP means no usable state for this node AND for MoveGroup.
    return hold_and_return(HoldReason::kPoseStale, now_ns);
  }

  // --- desired tcp pose -------------------------------------------------
  const tf2::Vector3 camera_now = tcp_now.p + tcp_now.R * t_cam_in_tcp_;

  Pose6 desired_tcp;
  const auto t_start = std::chrono::steady_clock::now();
  try {
    const PostureResult pr = solver_->solve(
      snap.p_arm_base, R_cam_in_tcp_, t_cam_in_tcp_, posture_params_);

    // A wrist branch change crosses j5 = 0, where Servo can hard-stop.
    // It is fed LIVE: routing every change through a planned move turned
    // out worse on a moving target (each ~3 s move lands behind the
    // target and the arm chases it: 14 moves vs 3 live stops in 120 s).
    // A hard stop is caught by the HOLD recovery. Logged so the rate can
    // be monitored; the switch itself needs no special handling here.
    if (pr.ok && last_sent_joints_valid_ &&
      std::abs(pr.joints[4] - last_sent_joints_[4]) > kBranchSwitchDeltaRad)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[posture] wrist branch change (live) j5 %+.2f -> %+.2f",
        last_sent_joints_[4], pr.joints[4]);
    }
    // Capture the previous j5 BEFORE overwriting it: the trace exists to
    // show the command step this tick, and reading last_sent_joints_
    // after the assignment made prev_j5 == j5 on every line.
    const double prev_j5 = last_sent_joints_valid_ ? last_sent_joints_[4] : 0.0;
    last_sent_joints_ = pr.joints;
    last_sent_joints_valid_ = true;
    desired_tcp = pr.tcp_desired;
    posture_last_status_ = pr.status;
    // Trace WHY this tick's wrist branch was chosen. Off by default: it
    // is a 5 Hz line and only useful when diagnosing a swing.
    if (posture_trace_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "[trace] j5=%+.3f branch=%+d run=%d relaxed=%d prev_j5=%+.3f "
        "switched=%d tgt=(%.2f %.2f %.2f)",
        pr.joints[4], pr.branch, pr.run_index, static_cast<int>(pr.relaxed),
        prev_j5,
        static_cast<int>(pr.branch_switched),
        snap.p_arm_base.x(), snap.p_arm_base.y(), snap.p_arm_base.z());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "posture setpoint unavailable (%s); HOLD", e.what());
    return hold_and_return(HoldReason::kTargetLost, now_ns);
  }
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();
  posture_solve_ms_sum_ += ms;
  posture_solve_ms_max_ = std::max(posture_solve_ms_max_, ms);
  if (++posture_solve_count_ >= 500) {
    RCLCPP_INFO(
      get_logger(),
      "[posture] solve mean %.2f ms max %.2f ms n=%d status=%s",
      posture_solve_ms_sum_ / posture_solve_count_,
      posture_solve_ms_max_, posture_solve_count_,
      posture_last_status_.c_str());
    posture_solve_ms_sum_ = 0.0;
    posture_solve_ms_max_ = 0.0;
    posture_solve_count_ = 0;
  }

  // Measured TCP twist for the damping term: J(q) * dq from the real arm's
  // 200 Hz joint velocities. This is the term that supplies the phase margin
  // the loop lost when it started closing on the real (lagging) pose.
  tf2::Vector3 v_meas(0, 0, 0);
  tf2::Vector3 w_meas(0, 0, 0);
  if (damping_active_ && measured_valid_ && measured_vel_valid_) {
    std::string twist_err;
    if (!posture_.tcpTwist(
        measured_positions_, measured_velocities_, v_meas, w_meas, twist_err))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "measured tcp twist unavailable (%s); damping off this tick",
        twist_err.c_str());
      v_meas.setValue(0, 0, 0);
      w_meas.setValue(0, 0, 0);
    }
  }

  // --- the control law --------------------------------------------------
  // Everything the law needs is an input from here on: no clock, no TF, no
  // message. That is what makes the refusals testable (tests/test_tracking_loop).
  TrackingInput loop_in;
  loop_in.now_ns = now_ns;
  loop_in.geometry_valid = snap.geometry_valid;
  loop_in.source_name = snap.source;
  loop_in.source_scale = arm_source_scale_;
  loop_in.tcp_now = tcp_now;
  loop_in.desired = desired_tcp;
  loop_in.p_arm_base = snap.p_arm_base;
  loop_in.joints_complete = measured_complete_locked();
  loop_in.v_meas = v_meas;
  loop_in.w_meas = w_meas;

  const TrackingOutput loop_out = loop_.step(loop_in, params_, session_);
  if (!loop_out.publish) {
    // Includes the non-finite case: a NaN must never reach Servo, and the
    // deadband path inside the law silently zeroes non-finite input, which
    // would look like convergence.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "tick refused (%s); HOLD",
      hold_reason_name(loop_out.hold_reason));
    return hold_and_return(loop_out.hold_reason, now_ns);
  }
  const TwistCommand & command = loop_out.twist;

  // --- reporting (ServoState) -------------------------------------------
  position_error_m_ = (desired_tcp.p - tcp_now.p).length();
  angular_error_rad_ = rotation_angle(desired_tcp.R, tcp_now.R);
  // Deadband hit AND finite inputs (compute_twist also zeroes non-finite
  // inputs into the deadband branch — never trust in_deadband alone).
  converged_ = command.in_deadband && pose_finite(desired_tcp) &&
    pose_finite(tcp_now);
  // Proximity from the CURRENT camera origin, never the desired pose
  // (spec §7.5 R9): cam_now = tcp_now.p + tcp_now.R * t_cam_in_tcp.
  target_distance_m_ = (snap.p_arm_base - camera_now).length();
  // Near-field gate (2026-11-17): XoY distance from the arm base origin, not
  // the camera-origin distance above (which is reporting only).  When the
  // platform closes in past this radius the arm holds; moving it cannot fix
  // a target that is inside the base footprint.
  const double target_xy_m = std::hypot(
    static_cast<double>(snap.p_arm_base.x()),
    static_cast<double>(snap.p_arm_base.y()));
  too_close_ = target_xy_m < min_observe_distance_m_;
  if (too_close_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "target XoY %.3f m < %.2f m; HOLD (too close)",
      target_xy_m, min_observe_distance_m_);
    return hold_and_return(HoldReason::kTooClose, now_ns);
  }
  geometry_source_ = snap.source;
  tracking_track_id_ = selected_snapshot_.track_id;

  // --- unreachable detection (spec §7.6: "e_ang 3 s 不减反增") -----------
  // Fire only when the error is actually getting WORSE, not when it improves
  // slowly: at low commanded speed a genuine 0.009 rad/s convergence failed
  // the old "must improve 0.5 deg every 3 s" rule and aborted a healthy run
  // (measured 2026-10-28). Best-so-far follows any improvement at all.
  // A SLOW follower is not an unreachable target. The measured error stays
  // high whenever the arm trails the command (measured 2026-10-30: with
  // speed_percent=20 + move_j the gap reached 1.2 rad and this guard fired at
  // 0.409 rad while Servo was still driving the command forward). So the
  // guard now also requires the COMMANDED joints to be standing still: if
  // /control is still advancing, the loop is making progress, just slowly.
  if (command.in_deadband) {
    sat_has_best_ = false;
    sat_cmd_still_ns_ = 0;
    sat_cmd_ref_ = commanded_positions_;
  } else {
    // Exactly one progress probe per tick: it also advances the reference,
    // so calling it twice would hide real progress.
    const bool cmd_advancing = commanded_advancing_locked();
    if (cmd_advancing) {
      // The command is still moving: re-arm at the current error.
      sat_cmd_still_ns_ = 0;
      sat_has_best_ = true;
      sat_best_rad_ = angular_error_rad_;
      sat_best_ns_ = now_ns;
    } else {
      if (sat_cmd_still_ns_ == 0) {
        sat_cmd_still_ns_ = now_ns;
      }
      if (!sat_has_best_ || angular_error_rad_ < sat_best_rad_) {
        sat_has_best_ = true;
        sat_best_rad_ = angular_error_rad_;
        sat_best_ns_ = now_ns;
      } else if (angular_error_rad_ > sat_best_rad_ + kSaturationImproveRad &&
        (now_ns - sat_best_ns_) >= kSaturationWindowNs &&
        (now_ns - sat_cmd_still_ns_) >= kSaturationWindowNs)
      {
        saturated_ = true;
        RCLCPP_WARN(
          get_logger(),
          "angular error stuck at %.3f rad for 3 s AND the commanded joints "
          "have not moved either; saturated -> HOLD", angular_error_rad_);
        return hold_and_return(HoldReason::kSaturated, now_ns, false);
      }
    }
  }

  // --- publish the twist (frame = BARE base_link) ------------------------
  publish_twist_locked(
    command.v.x(), command.v.y(), command.v.z(),
    command.w.x(), command.w.y(), command.w.z(),
    stamp.nanoseconds());

  return false;
}

void ServoNode::publish_twist_locked(double vx, double vy, double vz, double wx, double wy, double wz, int64_t stamp_ns)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = rclcpp::Time(stamp_ns);
  twist.header.frame_id = kTwistFrame;
  twist.twist.linear.x = vx;
  twist.twist.linear.y = vy;
  twist.twist.linear.z = vz;
  twist.twist.angular.x = wx;
  twist.twist.angular.y = wy;
  twist.twist.angular.z = wz;
  twist_pub_->publish(twist);
}

void ServoNode::maybe_start_servo_locked(int64_t now_ns)
{
  switch (start_state_) {
    case StartState::STARTED:
    case StartState::FAILED:
      return;

    case StartState::REQUESTED:
      // The service may block for up to 10 s inside waitForCompleteState();
      // a status message only proves the calc thread exists, which is the
      // weaker signal — keep waiting for the response itself.
      if (now_ns - start_request_ns_ > kServoStartResponseTimeoutNs) {
        start_state_ = StartState::FAILED;
        sm_.handle(ServoEvent::FAULT);
        RCLCPP_ERROR(
          get_logger(),
          "%s produced no response within %ld s — FAULT", kStartServoService,
          static_cast<long>(kServoStartResponseTimeoutNs / 1'000'000'000));
      }
      return;

    case StartState::CONFIRMING:
      if (servo_output_recv_ns_ > start_response_ns_) {
        // Servo really runs. Servo::start() left it UNPAUSED, so park it:
        // outside TRACKING this node must not leave Servo driving JTC.
        start_state_ = StartState::PAUSING;
        pause_on_confirm_requested_ = true;
        RCLCPP_INFO(
          get_logger(), "Servo confirmed running (%s output received); pausing it",
          kServoOutputTopic);
        return;
      }
      if (now_ns - start_response_ns_ > kServoStartVerifyNs) {
        start_state_ = StartState::FAILED;
        sm_.handle(ServoEvent::FAULT);
        RCLCPP_ERROR(
          get_logger(),
          "start_servo returned success but produced no %s within 300 ms — "
          "FAULT (Servo is not actually running)", kServoOutputTopic);
        return;
      }
      // Keep a fresh command in Servo's inbox: it only publishes while it
      // has a non-stale command (incoming_command_timeout = 0.3 s), and a
      // zero twist is the safe probe.
      if (now_ns - last_start_probe_ns_ >= kServoStartProbePeriodNs) {
        last_start_probe_ns_ = now_ns;
        publish_twist_locked(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, now_ns);
      }
      return;

    case StartState::PAUSING:
      // The blocking pause_servo call is performed by on_timer() outside
      // mutex_ (pause_on_confirm_requested_); this state only waits for it.
      return;

    case StartState::NOT_STARTED:
      break;
  }

  // NOT_STARTED: start exactly once, after the pose stream is alive.
  if (!pose_seen_) {
    return;
  }
  if (!start_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "waiting for %s before starting Servo", kStartServoService);
    return;
  }
  start_request_ns_ = now_ns;
  start_state_ = StartState::REQUESTED;
  RCLCPP_INFO(get_logger(), "calling %s (once per process)", kStartServoService);
  start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    std::bind(&ServoNode::on_start_response, this, std::placeholders::_1));
}

ServoNode::TriggerOutcome ServoNode::call_trigger_sync(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const char * name, double timeout_sec)
{
  std::lock_guard<std::mutex> call_lock(servo_call_mutex_);

  if (!client->service_is_ready() &&
    !client->wait_for_service(std::chrono::milliseconds(500)))
  {
    return {false, std::string(name) + ": service not available"};
  }

  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  rclcpp::executors::SingleThreadedExecutor local_executor;
  local_executor.add_callback_group(servo_sync_group_, get_node_base_interface());
  const auto timeout =
    std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0));
  const auto status = local_executor.spin_until_future_complete(future, timeout);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    return {false, std::string(name) + ": timeout"};
  }
  const auto response = future.get();
  if (!response || !response->success) {
    return {false, std::string(name) + ": success=false"};
  }
  return {true, std::string(name) + ": ok"};
}

std::optional<tf2::Transform> ServoNode::lookup_odom_base_locked()
{
  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_buffer_->canTransform(kOdomFrame, base_frame_, tf2::TimePointZero, zero)) {
    return std::nullopt;
  }
  try {
    const auto stamped = tf_buffer_->lookupTransform(
      kOdomFrame, base_frame_, tf2::TimePointZero, zero);
    tf2::Transform transform;
    transform.setOrigin(
      tf2::Vector3(
        stamped.transform.translation.x,
        stamped.transform.translation.y,
        stamped.transform.translation.z));
    tf2::Quaternion rotation(
      stamped.transform.rotation.x,
      stamped.transform.rotation.y,
      stamped.transform.rotation.z,
      stamped.transform.rotation.w);
    rotation.normalize();
    transform.setRotation(rotation);
    return transform;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

bool ServoNode::fk_tcp_locked(Pose6 & pose)
{
  if (!commanded_valid_) {
    return false;
  }
  std::string err;
  const tf2::Matrix3x3 I(1, 0, 0, 0, 1, 0, 0, 0, 1);
  const tf2::Vector3 zero(0, 0, 0);
  if (!posture_.cameraPose(
      commanded_positions_, I, zero, pose.p, pose.R, err))
  {
    return false;
  }
  return true;
}

bool ServoNode::lookup_tcp_locked(Pose6 & pose)
{
  if (tcp_pose_domain_command_ && fk_tcp_locked(pose)) {
    return true;
  }
  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_buffer_->canTransform(base_frame_, tcp_frame_, tf2::TimePointZero, zero)) {
    return false;
  }
  try {
    const auto stamped = tf_buffer_->lookupTransform(
      base_frame_, tcp_frame_, tf2::TimePointZero, zero);
    pose.p.setValue(
      stamped.transform.translation.x,
      stamped.transform.translation.y,
      stamped.transform.translation.z);
    tf2::Quaternion rotation(
      stamped.transform.rotation.x,
      stamped.transform.rotation.y,
      stamped.transform.rotation.z,
      stamped.transform.rotation.w);
    rotation.normalize();
    pose.R = tf2::Matrix3x3(rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void ServoNode::maybe_check_camera_tf_locked()
{
  if (camera_tf_checked_ || !camera_tf_check_enable_) {
    return;
  }
  if (!calibration_ok_ || !measured_complete_locked()) {
    return;
  }

  Pose6 fk;
  std::string error;
  if (!posture_.cameraPose(
      measured_positions_, R_cam_in_tcp_, t_cam_in_tcp_, fk.p, fk.R, error))
  {
    return;
  }

  // canTransform(0) first: a lookup on a missing frame would block for the
  // whole timeout and starve this 100 Hz tick.
  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_buffer_->canTransform(
      base_frame_, camera_tf_frame_, tf2::TimePointZero, zero))
  {
    return;  // tracker / realsense not up yet: retry on the next tick
  }

  tf2::Vector3 p_tf;
  tf2::Matrix3x3 R_tf;
  try {
    const auto stamped = tf_buffer_->lookupTransform(
      base_frame_, camera_tf_frame_, tf2::TimePointZero, zero);
    p_tf.setValue(
      stamped.transform.translation.x,
      stamped.transform.translation.y,
      stamped.transform.translation.z);
    tf2::Quaternion rotation(
      stamped.transform.rotation.x,
      stamped.transform.rotation.y,
      stamped.transform.rotation.z,
      stamped.transform.rotation.w);
    rotation.normalize();
    R_tf = tf2::Matrix3x3(rotation);
  } catch (const tf2::TransformException &) {
    return;
  }

  const double pos_err = (p_tf - fk.p).length();
  const double ang_err = rotation_angle(fk.R, R_tf);
  camera_tf_checked_ = true;
  if (pos_err > camera_tf_pos_tol_m_ || ang_err > camera_tf_ang_tol_rad_) {
    RCLCPP_WARN(
      get_logger(),
      "[calib] camera TF disagrees with FK: pos %.3f m, rot %.2f deg "
      "(tolerances %.3f m / %.2f deg). The tracker walks the TF chain while "
      "the control law uses FK + arm_calibration.json; check that both see "
      "the same URDF and calibration.",
      pos_err, ang_err * 180.0 / M_PI, camera_tf_pos_tol_m_,
      camera_tf_ang_tol_rad_ * 180.0 / M_PI);
  } else {
    RCLCPP_INFO(
      get_logger(), "[calib] camera TF matches FK: pos %.3f m, rot %.2f deg",
      pos_err, ang_err * 180.0 / M_PI);
  }
}

void ServoNode::publish_state_locked(const rclcpp::Time & stamp)
{
  const ServoState state = sm_.state();
  // HOLD keeps reporting the last known tier/errors so a field operator can
  // see WHY it is holding; IDLE/FAULT report nothing (no session).
  const bool session = session_active_locked();
  const std::string source = session ? geometry_source_ : std::string("none");

  const bool changed =
    state != last_published_state_ || source != last_published_source_ ||
    state_dirty_;
  if (!changed && (tick_count_ % kStatePublishEveryTicks) != 0U) {
    return;
  }

  ServoStateSnapshot snap;
  snap.stamp_ns = stamp.nanoseconds();
  snap.state = state;
  snap.session_active = session;
  snap.geometry_source = geometry_source_;
  snap.track_id = tracking_track_id_;
  snap.angular_error_rad = angular_error_rad_;
  snap.position_error_m = position_error_m_;
  snap.target_distance_m = target_distance_m_;
  snap.converged = converged_;
  snap.saturated = saturated_;
  snap.too_close = too_close_;
  snap.needs_preposition = needs_preposition_;
  snap.max_joint_delta = max_joint_delta_;
  snap.hold_reason = hold_reason_;
  snap.recovery_attempts = session_.recovery_attempts;
  snap.recovery_exhausted = session_.recovery_exhausted;
  snap.resumable = sm_.target_ok();
  snap.hold_since_ns = hold_since_ns_;
  state_pub_->publish(build_servo_state(snap));

  last_published_state_ = state;
  last_published_source_ = source;
  state_dirty_ = false;
}
}  // namespace lgw_bt_planner
