// include/lgw_bt_planner/hold_recovery_decision.hpp
//
// "Should HOLD trigger a recovery move, and which recovery?" (spec
// 2026-11-03 §4).
//
// Pure function, no ROS: the servo node feeds it the reason it entered HOLD,
// the budget state and the available geometry, and gets back one action.
//
// The distinction that matters: follower_lag is a FOLLOWER problem. The real
// arm follows the JTC through agx_arm_ctrl, so a MoveGroup move cannot make it
// faster — it must freeze the command and wait for the gap to close. Only
// posture problems (servo halt at a singularity, unreachable target) get a
// MoveGroup recovery move.

#ifndef LGW_BT_PLANNER__HOLD_RECOVERY_DECISION_HPP_
#define LGW_BT_PLANNER__HOLD_RECOVERY_DECISION_HPP_

#include <cstdint>

namespace lgw_bt_planner
{

enum class HoldReason
{
  kNone,
  kTargetLost,
  kTrackerStale,
  kPoseStale,
  kMoveGroupBusy,
  kServoHalt,
  kSaturated,
  kFollowerLag,
  kOperatorPause,
  kBranchSwitch,
  // The follower's joint message did not carry the whole arm. Zero-filling
  // the missing joints would command a pose the arm cannot be assumed to
  // reach, so this is a HOLD condition, not a parsing detail.
  kJointStateIncomplete,
  // Target's XoY distance to the arm base fell under the near-field gate.
  // Moving the arm cannot fix this; the platform/target must separate.
  kTooClose,
};

// Stable string for logs and ServoState.msg; "none" / "target_lost" / ...
const char * hold_reason_name(HoldReason reason);

enum class RecoveryAction
{
  kNone,          // not eligible (disabled, reason not recoverable, no state)
  kMove,          // solve the posture family and run one MoveGroup move
  kWaitFollower,  // follower_lag: freeze and wait for the gap to close
  kExhausted,     // eligible, but budget / geometry says stop trying
};

struct HoldRecoveryParams
{
  bool enable{false};
  bool allow_target_lost{false};
  int max_attempts{1};
  int64_t deadline_ms{8000};
  int64_t geometry_max_age_ms{1500};
  int64_t follower_timeout_ms{3000};
  // Between two REAL recovery attempts (a MoveGroup move). Prevents a
  // persistent fault from issuing a move on every control tick.
  int64_t retry_cooldown_ms{2000};
};

// Grows a field at a time; the future walking-recovery gate adds
// `platform_motion_ok` here without changing decide_hold_recovery() (spec
// §14.1). Do not add it until the platform-motion input exists.
struct HoldRecoveryInput
{
  HoldReason reason{HoldReason::kNone};
  int64_t hold_since_ns{0};
  int64_t now_ns{0};
  int attempts{0};
  bool move_group_busy{false};
  bool measured_valid{false};
  bool tcp_valid{false};
  bool geometry_available{false};
  int64_t geometry_age_ms{0};
  // Milliseconds since the previous attempt ended; negative when none yet.
  int64_t since_last_attempt_ms{-1};
};

RecoveryAction decide_hold_recovery(
  const HoldRecoveryInput & input, const HoldRecoveryParams & params);

// The posture family can only serve a target it can currently solve. Starting
// a recovery for an unreachable target would consume the single attempt and
// latch HOLD until the operator takes over (real machine 2026-11-06: the dog
// walked over the target and the node never resumed after it moved away).
// Gate the move; with the budget untouched the caller retries as soon as the
// target becomes reachable again.
RecoveryAction gate_recovery_on_family(
  RecoveryAction action, bool family_ok);

// Fixed priority, matching the TRACKING->HOLD condition order in
// servo_node.cpp: target loss (or a dead tracker stream) wins over an
// external MoveGroup, which wins over a Servo halt, a stale pose and a
// lagging follower.
HoldReason classify_hold(
  bool target_ok, bool stream_alive, bool move_group_busy, bool servo_halted,
  bool pose_alive, bool not_following);

// follower_lag auto-resume. `gap_ok_since_ns == 0` means the current
// below-threshold run has not started yet.
bool follower_recovery_ready(
  double gap_rad, int64_t gap_ok_since_ns, int64_t now_ns,
  double gap_threshold_rad, int64_t stable_ms);

enum class FollowerResumeOutcome
{
  kResumed,
  kFault,
  kCancelled,
};

// Decides what a follower_lag auto-resume attempt should do. `still_holding`
// is sampled AFTER the RESUME event attempt, so a concurrent release (state
// left HOLD) is reported as kCancelled instead of a terminal FAULT.
FollowerResumeOutcome decide_follower_resume(
  bool still_holding, bool unpause_ok, bool target_ok, bool resume_accepted);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__HOLD_RECOVERY_DECISION_HPP_
