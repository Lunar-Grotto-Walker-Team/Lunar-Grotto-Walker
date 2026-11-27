// tests/test_hold_recovery_decision.cpp
// Pure-logic unit tests for the HOLD recovery criteria (spec 2026-11-03 §4).
//
// No ROS runtime / executor / MoveIt / Servo needed: plain C++17 value
// types. Run the binary directly from build/lgw_bt_planner/
// (test_hold_recovery_decision) — colcon test is not available in this
// container (see CLAUDE.md).
//
// The rule that matters most: follower_lag NEVER returns kMove. The real
// arm follows the JTC through agx_arm_ctrl; MoveGroup cannot make it faster,
// so the only recovery is freezing the command and waiting for the gap to
// close.

#include "lgw_bt_planner/hold_recovery_decision.hpp"

#include <gtest/gtest.h>

namespace
{

using lgw_bt_planner::classify_hold;
using lgw_bt_planner::decide_follower_resume;
using lgw_bt_planner::decide_hold_recovery;
using lgw_bt_planner::FollowerResumeOutcome;
using lgw_bt_planner::follower_recovery_ready;
using lgw_bt_planner::HoldReason;
using lgw_bt_planner::HoldRecoveryInput;
using lgw_bt_planner::HoldRecoveryParams;
using lgw_bt_planner::RecoveryAction;

constexpr int64_t kMs = 1000000;

HoldRecoveryParams enabledParams()
{
  HoldRecoveryParams p;
  p.enable = true;
  p.allow_target_lost = true;
  return p;
}

// Every precondition satisfied: measured/tcp valid, target geometry fresh,
// no external MoveGroup goal, no budget consumed.
HoldRecoveryInput readyInput(HoldReason reason)
{
  HoldRecoveryInput in;
  in.reason = reason;
  in.hold_since_ns = 0;
  in.now_ns = 1000 * kMs;
  in.attempts = 0;
  in.move_group_busy = false;
  in.measured_valid = true;
  in.tcp_valid = true;
  in.geometry_available = true;
  in.geometry_age_ms = 100;
  return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// decide_hold_recovery
// ---------------------------------------------------------------------------

TEST(HoldRecoveryDecision, DisabledAlwaysNone)
{
  HoldRecoveryParams p;  // enable defaults to false
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kServoHalt), p),
            RecoveryAction::kNone);
}

// follower_lag is a follower problem, not a posture problem. It must never
// ask the node to move the arm, regardless of budget state.
TEST(HoldRecoveryDecision, FollowerLagWaits)
{
  const HoldRecoveryParams p = enabledParams();
  HoldRecoveryInput in = readyInput(HoldReason::kFollowerLag);
  in.attempts = 99;  // budget exhausted must not turn this into kExhausted
  EXPECT_EQ(decide_hold_recovery(in, p), RecoveryAction::kWaitFollower);
}

// A stale pose means MoveGroup has no state either. Do not consume budget:
// the pose stream may come back and the same HOLD may become recoverable.
TEST(HoldRecoveryDecision, PoseStaleNone)
{
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kPoseStale), enabledParams()),
            RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, MoveGroupBusyNone)
{
  HoldRecoveryInput in = readyInput(HoldReason::kServoHalt);
  in.move_group_busy = true;
  EXPECT_EQ(decide_hold_recovery(in, enabledParams()), RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, OperatorPauseNone)
{
  EXPECT_EQ(
    decide_hold_recovery(readyInput(HoldReason::kOperatorPause), enabledParams()),
    RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, ServoHaltMoves)
{
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kServoHalt), enabledParams()),
            RecoveryAction::kMove);
}

TEST(HoldRecoveryDecision, TooCloseNone)
{
  EXPECT_EQ(
    decide_hold_recovery(readyInput(HoldReason::kTooClose), enabledParams()),
    RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, TooCloseName)
{
  EXPECT_STREQ(
    lgw_bt_planner::hold_reason_name(HoldReason::kTooClose), "too_close");
}

TEST(HoldRecoveryDecision, SaturatedMoves)
{
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kSaturated), enabledParams()),
            RecoveryAction::kMove);
}

TEST(HoldRecoveryDecision, TargetLostRequiresFlag)
{
  HoldRecoveryParams p = enabledParams();
  p.allow_target_lost = false;
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kTargetLost), p),
            RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, TargetLostStaleGeometryExhausted)
{
  HoldRecoveryParams p = enabledParams();
  p.geometry_max_age_ms = 1500;
  HoldRecoveryInput in = readyInput(HoldReason::kTargetLost);
  in.geometry_age_ms = 2000;
  EXPECT_EQ(decide_hold_recovery(in, p), RecoveryAction::kExhausted);
}

TEST(HoldRecoveryDecision, TargetLostMovesWhenFresh)
{
  HoldRecoveryInput in = readyInput(HoldReason::kTargetLost);
  in.geometry_age_ms = 500;
  EXPECT_EQ(decide_hold_recovery(in, enabledParams()), RecoveryAction::kMove);
}

// The tracker-stale branch must behave exactly like target_lost: the allow
// flag gates it and stale geometry stops the retries.
TEST(HoldRecoveryDecision, TrackerStaleRequiresFlag)
{
  HoldRecoveryParams p = enabledParams();
  p.allow_target_lost = false;
  EXPECT_EQ(decide_hold_recovery(readyInput(HoldReason::kTrackerStale), p),
            RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, TrackerStaleMovesWhenFresh)
{
  EXPECT_EQ(
    decide_hold_recovery(readyInput(HoldReason::kTrackerStale), enabledParams()),
    RecoveryAction::kMove);
}

// No last-known contributor at all: there is nothing to aim at, so the
// session must give up rather than move the arm blind.
TEST(HoldRecoveryDecision, TargetLostWithoutGeometryExhausted)
{
  HoldRecoveryInput in = readyInput(HoldReason::kTargetLost);
  in.geometry_available = false;
  EXPECT_EQ(decide_hold_recovery(in, enabledParams()),
            RecoveryAction::kExhausted);
}

TEST(HoldRecoveryDecision, AttemptsExhausted)
{
  HoldRecoveryParams p = enabledParams();
  p.max_attempts = 1;
  HoldRecoveryInput in = readyInput(HoldReason::kServoHalt);
  in.attempts = 1;
  EXPECT_EQ(decide_hold_recovery(in, p), RecoveryAction::kExhausted);
}

TEST(HoldRecoveryDecision, DeadlineExceeded)
{
  HoldRecoveryParams p = enabledParams();
  p.deadline_ms = 8000;
  HoldRecoveryInput in = readyInput(HoldReason::kServoHalt);
  in.hold_since_ns = 0;
  in.now_ns = 9000 * kMs;
  EXPECT_EQ(decide_hold_recovery(in, p), RecoveryAction::kExhausted);
}

TEST(HoldRecoveryDecision, NoMeasuredStateNone)
{
  HoldRecoveryInput in = readyInput(HoldReason::kServoHalt);
  in.measured_valid = false;
  EXPECT_EQ(decide_hold_recovery(in, enabledParams()), RecoveryAction::kNone);
}

// ---------------------------------------------------------------------------
// classify_hold: fixed priority, matching the TRACKING->HOLD condition order
// in servo_node.cpp.
// ---------------------------------------------------------------------------

TEST(HoldReasonClassify, Priority)
{
  // Target loss wins over everything else.
  EXPECT_EQ(classify_hold(false, true, true, true, false, true),
            HoldReason::kTargetLost);
  // A dead tracker stream is reported as tracker_stale, not target_lost.
  EXPECT_EQ(classify_hold(false, false, true, true, false, true),
            HoldReason::kTrackerStale);
  // External MoveGroup wins over servo halt / pose / follower.
  EXPECT_EQ(classify_hold(true, true, true, true, false, true),
            HoldReason::kMoveGroupBusy);
  // Servo halt wins over pose / follower.
  EXPECT_EQ(classify_hold(true, true, false, true, false, true),
            HoldReason::kServoHalt);
  EXPECT_EQ(classify_hold(true, true, false, false, false, true),
            HoldReason::kPoseStale);
  EXPECT_EQ(classify_hold(true, true, false, false, true, true),
            HoldReason::kFollowerLag);
  EXPECT_EQ(classify_hold(true, true, false, false, true, false),
            HoldReason::kNone);
}

// ---------------------------------------------------------------------------
// follower_recovery_ready
// ---------------------------------------------------------------------------

TEST(FollowerRecovery, ReadyAfterStableWindow)
{
  // Started 300 ms ago (nonzero sentinel), exactly the 300 ms window.
  EXPECT_TRUE(follower_recovery_ready(0.05, 100 * kMs, 400 * kMs, 0.10, 300));
}

TEST(FollowerRecovery, NotReadyBelowWindow)
{
  EXPECT_FALSE(follower_recovery_ready(0.05, 100 * kMs, 200 * kMs, 0.10, 300));
}

TEST(FollowerRecovery, NotReadyWithLargeGap)
{
  EXPECT_FALSE(follower_recovery_ready(0.20, 100 * kMs, 500 * kMs, 0.10, 300));
}

TEST(FollowerRecovery, NotReadyBeforeFirstGoodTick)
{
  EXPECT_FALSE(follower_recovery_ready(0.05, 0, 500 * kMs, 0.10, 300));
}

// ---------------------------------------------------------------------------
// decide_follower_resume
// ---------------------------------------------------------------------------

TEST(FollowerResume, Resumed)
{
  EXPECT_EQ(decide_follower_resume(true, true, true, true),
            FollowerResumeOutcome::kResumed);
}

TEST(FollowerResume, FaultWhenUnpauseFails)
{
  EXPECT_EQ(decide_follower_resume(true, false, true, false),
            FollowerResumeOutcome::kFault);
}

TEST(FollowerResume, FaultWithoutTarget)
{
  EXPECT_EQ(decide_follower_resume(true, true, false, false),
            FollowerResumeOutcome::kFault);
}

// A release that lands while the unpause call is in flight must cancel the
// resume, not turn a clean release into a terminal FAULT (review #11).
TEST(FollowerResume, CancelledWhenStateLeftHold)
{
  EXPECT_EQ(decide_follower_resume(false, true, false, false),
            FollowerResumeOutcome::kCancelled);
  EXPECT_EQ(decide_follower_resume(false, false, true, false),
            FollowerResumeOutcome::kCancelled);
}

TEST(FollowerResume, FaultWhenResumeRejected)
{
  EXPECT_EQ(decide_follower_resume(true, true, true, false),
            FollowerResumeOutcome::kFault);
}

TEST(HoldRecoveryDecision, UnreachableTargetDoesNotStartRecovery)
{
  // Real machine 2026-11-06: the dog walked over the target, the single
  // recovery attempt failed inside the posture family, and the node latched
  // HOLD (recovery_exhausted) for the rest of the run even after the target
  // became reachable again. An unreachable target is transient: wait, do not
  // consume the attempt.
  EXPECT_EQ(
    lgw_bt_planner::gate_recovery_on_family(
      lgw_bt_planner::RecoveryAction::kMove, false),
    lgw_bt_planner::RecoveryAction::kNone);
  EXPECT_EQ(
    lgw_bt_planner::gate_recovery_on_family(
      lgw_bt_planner::RecoveryAction::kMove, true),
    lgw_bt_planner::RecoveryAction::kMove);
  // Other actions are untouched.
  EXPECT_EQ(
    lgw_bt_planner::gate_recovery_on_family(
      lgw_bt_planner::RecoveryAction::kWaitFollower, false),
    lgw_bt_planner::RecoveryAction::kWaitFollower);
  EXPECT_EQ(
    lgw_bt_planner::gate_recovery_on_family(
      lgw_bt_planner::RecoveryAction::kExhausted, false),
    lgw_bt_planner::RecoveryAction::kExhausted);
}

namespace
{
lgw_bt_planner::HoldRecoveryInput servo_halt_input(int attempts, int64_t hold_ms)
{
  lgw_bt_planner::HoldRecoveryInput in;
  in.reason = lgw_bt_planner::HoldReason::kServoHalt;
  in.attempts = attempts;
  in.hold_since_ns = 0;
  in.now_ns = hold_ms * 1000000;
  in.measured_valid = true;
  in.tcp_valid = true;
  in.geometry_available = true;
  return in;
}
}  // namespace

TEST(HoldRecoveryDecision, UnlimitedAttemptsDoNotExhaust)
{
  lgw_bt_planner::HoldRecoveryParams params;
  params.enable = true;
  params.max_attempts = 0;      // no cap
  // A POSITIVE deadline: 0 means "recovery disabled" (spec §9), not "wait
  // forever", so it cannot be used to express "no deadline" here.
  params.deadline_ms = 600000;
  params.retry_cooldown_ms = 0;
  const auto in = servo_halt_input(7, 60000);
  EXPECT_EQ(
    lgw_bt_planner::decide_hold_recovery(in, params),
    lgw_bt_planner::RecoveryAction::kMove);
}

TEST(HoldRecoveryDecision, ZeroAttemptsCapAndZeroDeadlineDisablesRecovery)
{
  lgw_bt_planner::HoldRecoveryParams params;
  params.enable = true;
  params.max_attempts = 0;
  params.deadline_ms = 0;       // disabled
  params.retry_cooldown_ms = 0;
  const auto in = servo_halt_input(7, 60000);
  EXPECT_EQ(
    lgw_bt_planner::decide_hold_recovery(in, params),
    lgw_bt_planner::RecoveryAction::kNone);
}

TEST(HoldRecoveryDecision, RetryCooldownSpacesRealMoves)
{
  lgw_bt_planner::HoldRecoveryParams params;
  params.enable = true;
  params.max_attempts = 3;
  params.deadline_ms = 600000;   // positive: 0 would disable recovery
  params.retry_cooldown_ms = 2000;
  auto in = servo_halt_input(1, 1500);
  in.since_last_attempt_ms = 500;   // too soon
  EXPECT_EQ(
    lgw_bt_planner::decide_hold_recovery(in, params),
    lgw_bt_planner::RecoveryAction::kNone);
  in.since_last_attempt_ms = 2500;  // cooldown elapsed
  EXPECT_EQ(
    lgw_bt_planner::decide_hold_recovery(in, params),
    lgw_bt_planner::RecoveryAction::kMove);
}

TEST(HoldRecoveryDecision, BranchSwitchIsRecoverable)
{
  lgw_bt_planner::HoldRecoveryParams params;
  params.enable = true;
  params.max_attempts = 1;
  params.deadline_ms = 8000;
  auto in = servo_halt_input(0, 100);
  in.reason = lgw_bt_planner::HoldReason::kBranchSwitch;
  EXPECT_EQ(
    lgw_bt_planner::decide_hold_recovery(in, params),
    lgw_bt_planner::RecoveryAction::kMove);
}
