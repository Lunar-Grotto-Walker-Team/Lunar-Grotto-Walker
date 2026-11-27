// src/servo/servo_recovery.cpp
//
// HOLD entry/exit, the recovery decision glue and the recovery worker thread.
// Split out of servo_node.hpp (2026-11-12); bodies moved verbatim apart from
// whitespace and the ServoNode:: qualifier.

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace lgw_bt_planner
{

bool ServoNode::hold_and_return(HoldReason reason, int64_t now_ns, bool clear_geometry)
{
  enter_hold_locked(reason, now_ns);
  sm_.handle(ServoEvent::TARGET_LOST);
  if (clear_geometry) {
    geometry_source_ = "none";
  }
  return true;
}

void ServoNode::enter_hold_locked(HoldReason reason, int64_t now_ns)
{
  hold_reason_ = reason;
  hold_since_ns_ = now_ns;
  session_.recovery_attempts = 0;
  session_.recovery_exhausted = false;
  follow_bad_since_ns_ = 0;
  follow_resume_since_ns_ = 0;
  follow_resume_requested_ = false;
}

ServoNode::RecoveryTarget ServoNode::resolve_recovery_target_locked(int64_t now_ns, bool target_ok, const tf2::Vector3 & current_target)
{
  RecoveryTarget out;
  if (target_ok) {
    out.available = true;
    out.p_arm_base = current_target;
    out.age_ms = 0;
    return out;
  }
  if (!has_selected_snapshot_ || !selected_snapshot_.geometry_valid) {
    return out;
  }
  out.available = true;
  out.p_arm_base = selected_snapshot_.p_arm_base;
  out.age_ms = std::max<int64_t>(
    0, (now_ns - selected_snapshot_.last_seen_ns) / 1000000);
  return out;
}

void ServoNode::finish_recovery_locked(bool fatal, const std::string & detail)
{
  recovery_in_progress_.store(false);
  recovery_probe_armed_ns_ = 0;
  recovery_probe_got_ = false;
  session_.last_recovery_end_ns = now().nanoseconds();
  if (fatal) {
    sm_.handle(ServoEvent::FAULT);
  } else {
    // Do NOT latch here: the next tick's budget decides (attempts,
    // deadline, cooldown). max_attempts = 1 reproduces the old behaviour;
    // 0 means unlimited, with the cooldown spacing the real moves.
    sm_.handle(ServoEvent::RECOVER_FAIL);
  }
  state_dirty_ = true;
  if (!detail.empty()) {
    RCLCPP_ERROR(get_logger(), "[recovery] %s", detail.c_str());
  }
}

void ServoNode::finish_recovery_wait(const std::string & detail)
{
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[recovery] %s; waiting for a reachable target (attempt not consumed)",
    detail.c_str());
  std::lock_guard<std::mutex> lock(mutex_);
  recovery_in_progress_.store(false);
  recovery_probe_armed_ns_ = 0;
  recovery_probe_got_ = false;
  if (session_.recovery_attempts > 0) {
    --session_.recovery_attempts;
  }
  sm_.handle(ServoEvent::RECOVER_FAIL);
  state_dirty_ = true;
}

void ServoNode::finish_recovery_failed(const std::string & detail, bool fatal)
{
  if (fatal) {
    // Never leave Servo driving after a failed re-seed/probe.
    call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  finish_recovery_locked(fatal, detail);
}

bool ServoNode::recovery_abort_locked() const
{
  return recovery_cancel_.load() || sm_.state() != ServoState::RECOVERING;
}

bool ServoNode::worker_aborted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return recovery_abort_locked();
}

void ServoNode::finish_recovery_cancelled()
{
  bool still_recovering = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    still_recovering = sm_.state() == ServoState::RECOVERING;
    recovery_in_progress_.store(false);
    recovery_probe_armed_ns_ = 0;
    recovery_probe_got_ = false;
    state_dirty_ = true;
  }
  if (!still_recovering) {
    // RELEASE already moved the state to RELEASING; only clear our flags.
    return;
  }
  // Deadline cancel: no RELEASE moved the state, and start_servo may have
  // left Servo unpaused. Park it, then leave RECOVERING explicitly so the
  // node can never get stuck there (spec §6.4).
  call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
  std::lock_guard<std::mutex> lock(mutex_);
  if (sm_.state() == ServoState::RECOVERING) {
    session_.recovery_exhausted = true;
    sm_.handle(ServoEvent::RECOVER_FAIL);
    state_dirty_ = true;
    RCLCPP_WARN(
      get_logger(), "[recovery] cancelled -> HOLD(exhausted)");
  }
}

bool ServoNode::verify_servo_seed(const std::array<double, kArmJointCount> & expected, double tolerance_rad, std::string & error)
{
  int64_t before_ns = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    before_ns = servo_output_recv_ns_;
    recovery_probe_got_ = false;
    recovery_probe_armed_ns_ = now().nanoseconds();
    publish_twist_locked(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, now().nanoseconds());
  }
  bool got = false;
  for (int i = 0; i < 30; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::lock_guard<std::mutex> lock(mutex_);
    if (recovery_probe_got_ && servo_output_recv_ns_ > before_ns) {
      got = true;
      break;
    }
  }
  double worst = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (got) {
      // 参考必须取"指令域", 不能取实测域。Servo 的 joint_topic 是
      // /control/joint_states, 它播种和积分都在指令域; follow:=true 时真机
      // 臂本来就滞后指令 0.1-0.46 rad(2026-11-16 实测 gap=0.459), 拿实测域
      // 去比会把这段正常的跟随滞后误判成"播种没生效", 于是反复打成 FAULT。
      // 和 recovery 后置校验用的是同一条规则。
      const std::array<double, kArmJointCount> & reference =
        commanded_valid_ ? commanded_positions_ : expected;
      for (std::size_t i = 0; i < kArmJointCount; ++i) {
        worst = std::max(
          worst, std::fabs(recovery_probe_q_[i] - reference[i]));
      }
    }
    recovery_probe_armed_ns_ = 0;
    recovery_probe_got_ = false;
  }
  if (!got) {
    error = "no Servo output after the reseed probe";
    return false;
  }
  if (worst > tolerance_rad) {
    error = "re-seed did not take (probe pose mismatch)";
    return false;
  }
  return true;
}

void ServoNode::update_follower_recovery_locked(int64_t now_ns)
{
  if (!recovery_params_.enable) {
    return;
  }
  if (now_ns - hold_since_ns_ >
    recovery_params_.follower_timeout_ms * 1000000LL)
  {
    follow_resume_since_ns_ = 0;
    follow_resume_requested_ = false;
    RCLCPP_ERROR(
      get_logger(),
      "follower_lag not recovered within %ld ms (gap=%.3f) -> FAULT",
      static_cast<long>(recovery_params_.follower_timeout_ms),
      follow_gap_locked());
    sm_.handle(ServoEvent::FAULT);
    state_dirty_ = true;
    return;
  }

  const double gap = follow_gap_locked();
  if (!std::isfinite(gap) || gap > follow_resume_gap_rad_) {
    follow_resume_since_ns_ = 0;
    return;
  }
  if (follow_resume_since_ns_ == 0) {
    follow_resume_since_ns_ = now_ns;
  }
  // Resume only with a live target; TARGET_OK was armed earlier this tick.
  if (!sm_.target_ok()) {
    return;
  }
  if (follower_recovery_ready(
      gap, follow_resume_since_ns_, now_ns, follow_resume_gap_rad_,
      follow_resume_stable_ms_))
  {
    follow_resume_requested_ = true;
  }
}

void ServoNode::maybe_start_recovery_locked(int64_t now_ns, bool target_ok, const tf2::Vector3 & current_target)
{
  if (recovery_in_progress_.load()) {
    return;
  }
  if (recovery_thread_.joinable()) {
    recovery_thread_.join();  // previous run finished
  }

  const RecoveryTarget target =
    resolve_recovery_target_locked(now_ns, target_ok, current_target);

  RecoveryInput in;
  in.reason = hold_reason_;
  in.now_ns = now_ns;
  in.hold_since_ns = hold_since_ns_;
  in.target_ok = target_ok;
  in.move_group_busy = move_group_busy_locked();
  in.measured_valid = measured_valid_;
  Pose6 tcp_now;
  in.tcp_valid = lookup_tcp_locked(tcp_now);
  in.geometry_available = target.available;
  in.geometry_age_ms = target.age_ms;
  in.since_last_attempt_ms = session_.last_recovery_end_ns > 0 ?
    (now_ns - session_.last_recovery_end_ns) / 1000000 : -1;
  in.target = target.p_arm_base;

  // Only the node owns the solver, so reachability arrives as an input; the
  // orchestrator decides whether it matters.
  //
  // The family is initialised at startup (declare_and_load_parameters) and
  // is now the only setpoint rule, so it always applies. If its init failed
  // the node is in FAULT already, and solve() throws -- caught below, so a
  // broken URDF cannot turn every tick into an exception escape.
  in.family_applicable = true;
  bool family_ok = true;
  std::string family_error;
  if (in.family_applicable && solver_ != nullptr) {
    family_ok = false;   // cannot prove reachable without a solver
    try {
      family_ok = solver_->solve(
        target.p_arm_base, R_cam_in_tcp_, t_cam_in_tcp_,
        posture_params_).ok;
    } catch (const std::exception & e) {
      family_error = e.what();
    }
  }
  in.family_ok = family_ok;

  const bool was_exhausted = session_.recovery_exhausted;
  const RecoveryPlan plan =
    RecoveryOrchestrator::decide(in, params_, session_);
  if (session_.recovery_exhausted && !was_exhausted) {
    state_dirty_ = true;
    RCLCPP_WARN(
      get_logger(), "[recovery] budget exhausted for %s",
      hold_reason_name(hold_reason_));
    return;
  }
  if (!plan.needed) {
    if (!plan.detail.empty() && !session_.recovery_exhausted) {
      if (!family_error.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[recovery] %s (%s); waiting without consuming an attempt",
          plan.detail.c_str(), family_error.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[recovery] %s; waiting without consuming an attempt",
          plan.detail.c_str());
      }
    }
    return;
  }

  if (!sm_.handle(ServoEvent::RECOVER_START)) {
    // The orchestrator already counted the attempt; no move happened, so
    // give it back. (Before the split the increment came after this check.)
    --session_.recovery_attempts;
    return;
  }
  recovery_target_ = target.p_arm_base;
  recovery_cancel_.store(false);
  recovery_in_progress_.store(true);
  state_dirty_ = true;
  RCLCPP_INFO(
    get_logger(),
    "[recovery] %s -> RECOVERING (attempt %d/%d, geometry_age=%ldms)",
    hold_reason_name(hold_reason_), session_.recovery_attempts,
    recovery_params_.max_attempts, static_cast<long>(target.age_ms));
  recovery_thread_ = std::thread(&ServoNode::recovery_worker, this);
}

void ServoNode::recovery_worker()
{
  std::array<double, kArmJointCount> q_now{};
  Pose6 tcp_now;
  tf2::Vector3 target;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q_now = measured_positions_;
    target = recovery_target_;
    if (!lookup_tcp_locked(tcp_now)) {
      finish_recovery_locked(false, "tcp pose unavailable");
      return;
    }
  }
  if (worker_aborted()) {
    finish_recovery_cancelled();
    return;
  }

  PostureResult family;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      family = solver_->solve(
        target, R_cam_in_tcp_, t_cam_in_tcp_, posture_params_);
    } catch (const std::exception & e) {
      finish_recovery_locked(false, std::string("posture: ") + e.what());
      return;
    }
  }
  if (!family.ok) {
    // 2026-11-05: this used to log only "posture family not ok", which made
    // the close-range HOLDs undiagnosable — the family's answer depends on
    // the precise target geometry, and nothing recorded it.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[recovery] posture family not ok: target=(%+.3f, %+.3f, %+.3f) "
      "arm_base, status=%s aim=%.3f rad cond=%.1f, q_now=[%+.2f %+.2f "
      "%+.2f %+.2f %+.2f %+.2f]",
      target.x(), target.y(), target.z(), family.status.c_str(),
      family.aim_error_rad, family.cond,
      q_now[0], q_now[1], q_now[2], q_now[3], q_now[4], q_now[5]);
    finish_recovery_wait("posture family not ok");
    return;
  }

  const PrepositionDecision decision = decide_preposition(
    family.joints, q_now, family.tcp_desired, tcp_now, preposition_params_);
  RCLCPP_INFO(
    get_logger(),
    "[recovery] plan: family_ok=%d needed=%d joint_delta=%.3f q5=%.3f "
    "margin=%.3f tcp=%.3fm/%.3frad target=(%+.3f, %+.3f, %+.3f)",
    static_cast<int>(family.ok), static_cast<int>(decision.needed),
    decision.max_joint_delta, decision.q5_delta, decision.min_limit_margin,
    decision.tcp_distance, decision.tcp_angle,
    target.x(), target.y(), target.z());

  if (decision.needed) {
    // Re-check under the lock immediately before sending: the earlier
    // verdict was a snapshot, and an external MoveGroup goal may have
    // started since. (A goal that starts after this check cannot be
    // reserved against; MoveGroup serialises them itself.)
    bool send_abort = false;
    bool send_busy = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      send_abort = recovery_abort_locked();
      send_busy = move_group_busy_locked();
    }
    if (send_abort) {
      finish_recovery_cancelled();
      return;
    }
    if (send_busy) {
      finish_recovery_failed("move_group busy at send time", false);
      return;
    }

    // follow:=true: MoveGroup plans from /feedback while the JTC holds
    // /control. HOLD froze the JTC, so wait for the real arm to catch up;
    // otherwise the JTC rejects the trajectory on allowed_start_tolerance.
    const int64_t presettle_deadline_ns =
      now().nanoseconds() + recovery_presettle_timeout_ms_ * 1000000LL;
    int64_t settled_since_ns = 0;
    while (true) {
      bool abort = false;
      bool ready = true;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        abort = recovery_abort_locked();
        const double gap = follow_gap_locked();
        ready = !follow_check_enabled_ || !commanded_valid_ ||
          !measured_valid_ || gap <= recovery_presettle_gap_rad_;
      }
      if (abort) {
        finish_recovery_cancelled();
        return;
      }
      if (ready) {
        const int64_t now_settled_ns = now().nanoseconds();
        if (settled_since_ns == 0) {
          settled_since_ns = now_settled_ns;
        } else if (now_settled_ns - settled_since_ns >=
          recovery_presettle_stable_ms_ * 1000000LL)
        {
          break;
        }
      } else {
        settled_since_ns = 0;
      }
      if (now().nanoseconds() > presettle_deadline_ns) {
        finish_recovery_failed(
          "real arm did not settle before the recovery move", false);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Last abort check before the goal leaves the node: a release that
    // landed during the pre-settle must not start a move.
    if (worker_aborted()) {
      finish_recovery_cancelled();
      return;
    }
    std::array<std::string, kArmJointCount> names;
    for (std::size_t i = 0; i < kArmJointCount; ++i) {
      names[i] = kArmJoints[i];
    }
    const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(recovery_timeout_s_ * 1000.0));
    std::string error;
    preposition_in_progress_.store(true);
    const bool moved = prepositioner_->move_to(
      family.joints, names, recovery_velocity_scaling_,
      recovery_accel_scaling_, recovery_tolerance_rad_, timeout, error);
    preposition_in_progress_.store(false);
    if (worker_aborted()) {
      finish_recovery_cancelled();
      return;
    }
    if (!moved) {
      finish_recovery_failed("move failed: " + error, false);
      return;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(recovery_settle_ms_));
    if (worker_aborted()) {
      finish_recovery_cancelled();
      return;
    }

    // Verify the domain MoveGroup actually drove: the JTC/commanded state
    // (/control). In follow:=true the real arm follows over ~1 s, so
    // comparing /feedback here would reject a perfectly good move. Whether
    // the real arm keeps up is the following monitor's job after resume.
    double worst = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto & moved =
        commanded_valid_ ? commanded_positions_ : measured_positions_;
      for (std::size_t i = 0; i < kArmJointCount; ++i) {
        worst = std::max(
          worst, std::fabs(moved[i] - family.joints[i]));
      }
    }
    if (worst > recovery_tolerance_rad_) {
      finish_recovery_failed("post-move verify failed", false);
      return;
    }
  }

  if (worker_aborted()) {
    finish_recovery_cancelled();
    return;
  }

  // The move invalidated Servo's cached command: re-seed it now.
  const TriggerOutcome reseed =
    call_trigger_sync(reseed_client_, "start_servo(recovery)", kReseedTimeoutSec);
  if (!reseed.ok) {
    finish_recovery_failed("reseed failed: " + reseed.detail, true);
    return;
  }
  if (worker_aborted()) {
    finish_recovery_cancelled();
    return;
  }

  // Zero-twist probe BEFORE RECOVER_OK: while the state is still
  // RECOVERING the control law publishes nothing, so the first output is
  // the re-seeded cache. This is independent of the deadband.
  std::array<double, kArmJointCount> expected{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expected = measured_positions_;
  }
  std::string probe_error;
  if (!verify_servo_seed(expected, 0.10, probe_error)) {
    finish_recovery_failed(probe_error, true);
    return;
  }

  // Decide BEFORE unpausing: start_servo already unpaused Servo, so if the
  // target vanished (or a release arrived) it must be parked again before
  // leaving RECOVERING (spec §9, §6.4).
  bool abort = false;
  bool target_ok_now = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    abort = recovery_abort_locked();
    target_ok_now = sm_.target_ok();
  }
  if (abort) {
    finish_recovery_cancelled();
    return;
  }
  if (!target_ok_now) {
    call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
    std::lock_guard<std::mutex> lock(mutex_);
    session_.recovery_exhausted = true;
    recovery_in_progress_.store(false);
    sm_.handle(ServoEvent::RECOVER_FAIL);
    state_dirty_ = true;
    RCLCPP_WARN(get_logger(), "[recovery] target gone -> HOLD(exhausted)");
    return;
  }

  const TriggerOutcome unpause = call_trigger_sync(
    unpause_client_, "unpause_servo(recovery)", kUnpauseTimeoutSec);
  if (!unpause.ok) {
    finish_recovery_failed("unpause failed: " + unpause.detail, true);
    return;
  }
  if (worker_aborted()) {
    finish_recovery_cancelled();
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  recovery_in_progress_.store(false);
  session_.last_recovery_end_ns = now().nanoseconds();
  sm_.handle(ServoEvent::RECOVER_OK);
  state_dirty_ = true;
  RCLCPP_INFO(
    get_logger(), "[recovery] finished -> %s", state_name(sm_.state()));
}
}  // namespace lgw_bt_planner
