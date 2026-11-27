// tests/test_recovery_orchestrator.cpp
//
// Task 7 of the 2026-11-07 node-split plan (S4). The rules that decide whether
// a HOLD turns into a MoveGroup attempt used to live inside a node method that
// needed a mutex, a thread and a TF buffer to reach. They are now a pure
// decision over an input struct and the session's memory.

#include <gtest/gtest.h>

#include "lgw_bt_planner/servo/recovery_orchestrator.hpp"

namespace
{

using lgw_bt_planner::HoldReason;
using lgw_bt_planner::RecoveryInput;
using lgw_bt_planner::RecoveryOrchestrator;
using lgw_bt_planner::SessionState;
using lgw_bt_planner::ServoParams;

ServoParams enabled_defaults()
{
  ServoParams p;
  // HoldRecoveryParams::enable is false in code; the shipped yaml turns it on.
  // Every test below states which side of that switch it is on.
  p.recovery.enable = true;
  p.recovery.max_attempts = 1;
  p.recovery.deadline_ms = 8000;
  p.recovery.retry_cooldown_ms = 0;
  p.recovery.allow_target_lost = false;
  return p;
}

RecoveryInput input_at(int64_t now_ns)
{
  RecoveryInput in;
  in.reason = HoldReason::kServoHalt;
  in.now_ns = now_ns;
  in.hold_since_ns = now_ns;
  in.target_ok = true;
  in.measured_valid = true;
  in.tcp_valid = true;
  in.geometry_available = true;
  in.geometry_age_ms = 100;
  in.family_ok = true;
  in.target.setValue(0.5, 0.0, 0.3);
  return in;
}

}  // namespace

TEST(RecoveryOrchestrator, RecoversWhenEverythingAllowsIt)
{
  ServoParams p = enabled_defaults();
  SessionState st;
  const auto plan = RecoveryOrchestrator::decide(input_at(1'000'000'000), p, st);

  EXPECT_TRUE(plan.needed);
  EXPECT_DOUBLE_EQ(plan.target.x(), 0.5);
  EXPECT_EQ(st.recovery_attempts, 1);   // the attempt is only counted when taken
  EXPECT_TRUE(plan.detail.empty());
}

TEST(RecoveryOrchestrator, IsOffUntilTheParameterIsSet)
{
  ServoParams p = enabled_defaults();
  p.recovery.enable = false;
  SessionState st;

  EXPECT_FALSE(RecoveryOrchestrator::decide(input_at(1'000'000'000), p, st).needed);
  EXPECT_EQ(st.recovery_attempts, 0);
}

TEST(RecoveryOrchestrator, ExhaustsTheBudgetAndStopsRetrying)
{
  SessionState st;
  st.recovery_attempts = 1;
  ServoParams p = enabled_defaults();
  p.recovery.max_attempts = 1;

  const auto plan = RecoveryOrchestrator::decide(input_at(1'000'000'000), p, st);

  EXPECT_FALSE(plan.needed);
  EXPECT_TRUE(st.recovery_exhausted);
  EXPECT_EQ(st.recovery_attempts, 1);
}

TEST(RecoveryOrchestrator, DeadlineElapsedStopsRetrying)
{
  // The shipped semantics: deadline_ms is the window from hold_since_ns.
  // deadline_ms <= 0 means "no deadline" (hold_recovery_decision.cpp), NOT
  // "recovery disabled" — the brief for this task assumed the opposite and
  // would have silently changed the parameter's meaning.
  ServoParams p = enabled_defaults();
  p.recovery.deadline_ms = 8000;
  SessionState st;
  RecoveryInput in = input_at(10'000'000'000);
  in.hold_since_ns = 1'000'000'000;     // held for 9 s > 8 s

  const auto plan = RecoveryOrchestrator::decide(in, p, st);

  EXPECT_FALSE(plan.needed);
  EXPECT_TRUE(st.recovery_exhausted);
}

TEST(RecoveryOrchestrator, ZeroDeadlineDisablesRecovery)
{
  // Spec §9: deadline_ms = 0 means DISABLED, not "wait forever". The control
  // tick's overrun guard skips the cancel for a non-positive deadline, so
  // this has to be decided here -- otherwise a deadline of 0 would start a
  // move and cancel it on the next tick.
  ServoParams p = enabled_defaults();
  p.recovery.deadline_ms = 0;
  SessionState st;
  RecoveryInput in = input_at(10'000'000'000);
  in.hold_since_ns = 1'000'000'000;

  const auto plan = RecoveryOrchestrator::decide(in, p, st);
  EXPECT_FALSE(plan.needed);
  EXPECT_EQ(st.recovery_attempts, 0);
  EXPECT_FALSE(st.recovery_exhausted);
}

TEST(RecoveryOrchestrator, WaitsWithoutConsumingAnAttemptWhenTheTargetIsUnreachable)
{
  // The dog walked over the target: transient, so the single attempt must
  // survive it. Measured on the real machine 2026-11-06.
  ServoParams p = enabled_defaults();
  SessionState st = SessionState{};
  RecoveryInput in = input_at(1'000'000'000);
  in.family_ok = false;

  const auto plan = RecoveryOrchestrator::decide(in, p, st);

  EXPECT_FALSE(plan.needed);
  EXPECT_FALSE(plan.detail.empty());
  EXPECT_EQ(st.recovery_attempts, 0);
  EXPECT_FALSE(st.recovery_exhausted);
}

TEST(RecoveryOrchestrator, IsUnavailableWithoutAPostureFamily)
{
  // Real machine 2026-11-07: with lookat_mode legacy the posture family is
  // never initialised, but the gate still asked it and got
  // "solveFree() before initialize()" every 2 s, forever. The worker would
  // throw on the same call, so skipping the gate alone would only move the
  // failure later and burn the single attempt. No family => no recovery.
  ServoParams p = enabled_defaults();
  SessionState st;
  RecoveryInput in = input_at(1'000'000'000);
  in.family_applicable = false;
  in.family_ok = false;

  const auto plan = RecoveryOrchestrator::decide(in, p, st);

  EXPECT_FALSE(plan.needed);
  EXPECT_FALSE(plan.detail.empty());
  EXPECT_EQ(st.recovery_attempts, 0);
  EXPECT_FALSE(st.recovery_exhausted);
}

TEST(RecoveryOrchestrator, RoutesAVanishedTargetThroughTheTargetLostGate)
{
  // Entered HOLD on servo_halt, but the target has since disappeared. Moving
  // on the last known geometry is opt-in, so the hold reason must not be
  // inherited from entry.
  ServoParams p = enabled_defaults();
  SessionState st;
  RecoveryInput in = input_at(1'000'000'000);
  in.reason = HoldReason::kServoHalt;
  in.target_ok = false;

  EXPECT_FALSE(RecoveryOrchestrator::decide(in, p, st).needed);
  EXPECT_EQ(st.recovery_attempts, 0);
}

TEST(RecoveryOrchestrator, CooldownSpacesOutRealMoves)
{
  ServoParams p = enabled_defaults();
  p.recovery.retry_cooldown_ms = 2000;
  SessionState st;
  RecoveryInput in = input_at(1'000'000'000);
  in.since_last_attempt_ms = 500;      // a move ended 0.5 s ago

  EXPECT_FALSE(RecoveryOrchestrator::decide(in, p, st).needed);
  EXPECT_EQ(st.recovery_attempts, 0);
}

TEST(RecoveryOrchestrator, StaysStoppedOnceTheBudgetIsDeclaredExhausted)
{
  ServoParams p = enabled_defaults();
  p.recovery.max_attempts = 5;         // budget would allow more
  SessionState st;
  st.recovery_exhausted = true;

  EXPECT_FALSE(RecoveryOrchestrator::decide(input_at(1'000'000'000), p, st).needed);
}
