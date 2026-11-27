// src/servo/recovery_orchestrator.cpp
//
// Task 7 of the 2026-11-07 node-split plan. Pure decision: no rclcpp, no TF,
// no thread. The worker that actually moves the arm stays in the node.

#include "lgw_bt_planner/servo/recovery_orchestrator.hpp"

namespace lgw_bt_planner
{
namespace
{

// A posture problem is one the posture family can fix. A follower lag or an
// operator pause is not, and an external MoveGroup owning the arm is not ours
// to fight. Kept identical to the pre-refactor predicate.
bool is_posture_reason(HoldReason reason)
{
  return reason == HoldReason::kServoHalt ||
         reason == HoldReason::kSaturated ||
         reason == HoldReason::kBranchSwitch ||
         reason == HoldReason::kTargetLost ||
         reason == HoldReason::kTrackerStale;
}

}  // namespace

RecoveryPlan RecoveryOrchestrator::decide(
  const RecoveryInput & in, const ServoParams & params, SessionState & st)
{
  RecoveryPlan plan;

  if (!params.recovery.enable || st.recovery_exhausted) {
    return plan;
  }

  // The recovery MOVE is computed from the posture family (the worker solves
  // it for the live target). Without a family there is nothing to move to, so
  // recovery cannot run at all — say so once instead of probing an
  // uninitialised solver forever (real machine 2026-11-07: legacy mode logged
  // "target not reachable yet (solveFree() before initialize())" every 2 s).
  if (!in.family_applicable) {
    plan.detail = "recovery unavailable: no posture family in this look-at mode";
    return plan;
  }

  HoldRecoveryInput hri;
  // The HOLD entry reason is a snapshot. If the target has since vanished,
  // route the decision through the target-lost gate (allow flag + geometry
  // age) instead of inheriting, say, servo_halt and moving on stale geometry
  // the operator never authorised.
  hri.reason = in.target_ok ? in.reason : HoldReason::kTargetLost;
  hri.hold_since_ns = in.hold_since_ns;
  hri.now_ns = in.now_ns;
  hri.attempts = st.recovery_attempts;
  hri.move_group_busy = in.move_group_busy;
  hri.measured_valid = in.measured_valid;
  hri.tcp_valid = in.tcp_valid;
  hri.geometry_available = in.geometry_available;
  hri.geometry_age_ms = in.geometry_age_ms;
  hri.since_last_attempt_ms = in.since_last_attempt_ms;

  // Reachability FIRST: a target the posture family cannot serve right now is
  // a transient condition (the dog walked over it), not a recovery failure.
  // Checked before the budget so it never consumes an attempt or the deadline.
  if (is_posture_reason(hri.reason) && !in.family_ok) {
    plan.detail = "target not reachable yet";
    return plan;
  }

  const RecoveryAction action = decide_hold_recovery(hri, params.recovery);
  if (action == RecoveryAction::kExhausted) {
    st.recovery_exhausted = true;
    plan.detail = "budget exhausted";
    return plan;
  }
  if (action != RecoveryAction::kMove) {
    // kNone / kWaitFollower: nothing to start. The follower wait is driven by
    // the node's own follower path, not by a recovery move.
    plan.detail = "not eligible";
    return plan;
  }

  ++st.recovery_attempts;
  plan.needed = true;
  plan.target = in.target;
  return plan;
}

}  // namespace lgw_bt_planner
