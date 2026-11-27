// src/hold_recovery_decision.cpp
//
// Implementation of spec 2026-11-03 §4. Pure C++17, no ROS.

#include "lgw_bt_planner/hold_recovery_decision.hpp"

namespace lgw_bt_planner
{
namespace
{
constexpr int64_t kNsPerMs = 1000000;
}  // namespace

const char * hold_reason_name(HoldReason reason)
{
  switch (reason) {
    case HoldReason::kNone:
      return "none";
    case HoldReason::kTargetLost:
      return "target_lost";
    case HoldReason::kTrackerStale:
      return "tracker_stale";
    case HoldReason::kPoseStale:
      return "pose_stale";
    case HoldReason::kMoveGroupBusy:
      return "move_group_busy";
    case HoldReason::kServoHalt:
      return "servo_halt";
    case HoldReason::kSaturated:
      return "saturated";
    case HoldReason::kFollowerLag:
      return "follower_lag";
    case HoldReason::kOperatorPause:
      return "operator_pause";
    case HoldReason::kBranchSwitch:
      return "branch_switch";
    case HoldReason::kJointStateIncomplete:
      return "joint_state_incomplete";
    case HoldReason::kTooClose:
      return "too_close";
  }
  return "unknown";
}

RecoveryAction decide_hold_recovery(
  const HoldRecoveryInput & input, const HoldRecoveryParams & params)
{
  if (!params.enable) {
    return RecoveryAction::kNone;
  }

  // A slow follower is not a posture problem: the only recovery is to stop
  // commanding and let the real arm close the gap.
  if (input.reason == HoldReason::kFollowerLag) {
    return RecoveryAction::kWaitFollower;
  }

  // MoveGroup cannot act without state, somebody else owns it, or a human
  // asked for the pause. Do NOT consume budget: these can clear by
  // themselves and the same HOLD may become recoverable.
  if (input.reason == HoldReason::kPoseStale ||
    input.reason == HoldReason::kMoveGroupBusy ||
    input.reason == HoldReason::kOperatorPause ||
    input.reason == HoldReason::kJointStateIncomplete ||
    input.reason == HoldReason::kTooClose ||
    input.reason == HoldReason::kNone)
  {
    return RecoveryAction::kNone;
  }

  if (!input.measured_valid || !input.tcp_valid || input.move_group_busy) {
    return RecoveryAction::kNone;
  }

  // max_attempts <= 0 removes the attempt cap (spec §9: "0 = 无限").
  // deadline_ms <= 0 DISABLES recovery outright (spec §9: "0 = 禁用") -- it
  // does NOT mean "wait forever". The two sentinels deliberately differ, and
  // the control tick relies on this: its overrun guard also skips the cancel
  // when the deadline is not positive, so a disabled deadline can never start
  // a move that is then cancelled 10 ms later.
  if (params.deadline_ms <= 0) {
    return RecoveryAction::kNone;
  }
  if ((params.max_attempts > 0 && input.attempts >= params.max_attempts) ||
    input.now_ns - input.hold_since_ns > params.deadline_ms * kNsPerMs)
  {
    return RecoveryAction::kExhausted;
  }

  // Space out real attempts: a MoveGroup move must not fire on every tick.
  if (params.retry_cooldown_ms > 0 && input.since_last_attempt_ms >= 0 &&
    input.since_last_attempt_ms < params.retry_cooldown_ms)
  {
    return RecoveryAction::kNone;
  }

  if (input.reason == HoldReason::kTargetLost ||
    input.reason == HoldReason::kTrackerStale)
  {
    if (!params.allow_target_lost) {
      return RecoveryAction::kNone;
    }
    if (!input.geometry_available ||
      input.geometry_age_ms > params.geometry_max_age_ms)
    {
      return RecoveryAction::kExhausted;
    }
  }

  return RecoveryAction::kMove;
}

RecoveryAction gate_recovery_on_family(
  RecoveryAction action, bool family_ok)
{
  if (action == RecoveryAction::kMove && !family_ok) {
    return RecoveryAction::kNone;
  }
  return action;
}

HoldReason classify_hold(
  bool target_ok, bool stream_alive, bool move_group_busy, bool servo_halted,
  bool pose_alive, bool not_following)
{
  if (!target_ok) {
    return stream_alive ? HoldReason::kTargetLost : HoldReason::kTrackerStale;
  }
  if (move_group_busy) {
    return HoldReason::kMoveGroupBusy;
  }
  if (servo_halted) {
    return HoldReason::kServoHalt;
  }
  if (!pose_alive) {
    return HoldReason::kPoseStale;
  }
  if (not_following) {
    return HoldReason::kFollowerLag;
  }
  return HoldReason::kNone;
}

bool follower_recovery_ready(
  double gap_rad, int64_t gap_ok_since_ns, int64_t now_ns,
  double gap_threshold_rad, int64_t stable_ms)
{
  if (gap_ok_since_ns == 0 || gap_rad > gap_threshold_rad) {
    return false;
  }
  return now_ns - gap_ok_since_ns >= stable_ms * 1000000;
}

FollowerResumeOutcome decide_follower_resume(
  bool still_holding, bool unpause_ok, bool target_ok, bool resume_accepted)
{
  if (resume_accepted) {
    return FollowerResumeOutcome::kResumed;
  }
  // The session left HOLD while the unpause call was in flight (release);
  // that is a clean cancellation, not a fault.
  if (!still_holding) {
    return FollowerResumeOutcome::kCancelled;
  }
  // Still HOLD and the resume was not accepted: unpause failed, the target
  // latch is empty, or the state machine rejected it. All are genuine
  // failures; the parameters are kept for logging/future rules.
  (void)unpause_ok;
  (void)target_ok;
  return FollowerResumeOutcome::kFault;
}

}  // namespace lgw_bt_planner
