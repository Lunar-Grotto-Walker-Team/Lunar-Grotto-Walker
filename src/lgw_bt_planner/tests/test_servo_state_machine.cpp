// tests/test_servo_state_machine.cpp
// Pure-logic unit tests for ServoStateMachine + StreamWatchdog (Task 5 of
// the 2026-10-26 servo-takeover plan, spec §7.1).
//
// No ROS runtime / executor / MoveIt / Servo needed: plain C++17 value
// types. Run the binary directly from build/lgw_bt_planner/
// (test_servo_state_machine) — colcon test is not available in this
// container (see CLAUDE.md).
//
// Transition table pinned here (authoritative, SDD ruling R2 — the
// architecture pauses/unpauses MoveIt Servo instead of switching
// controllers, hence UNPAUSE_OK/UNPAUSE_FAIL/PAUSE_OK):
//
//   IDLE      + TAKEOVER (move_group_busy == false) -> PREEMPT
//   PREEMPT   + UNPAUSE_OK                          -> TRACKING
//   PREEMPT   + UNPAUSE_FAIL                        -> FAULT
//   TRACKING  + TARGET_LOST                         -> HOLD
//   HOLD      + TARGET_LOST                         -> HOLD (recognised, stays)
//   TRACKING  + PAUSE                               -> HOLD
//   HOLD      + RESUME   (after TARGET_OK)          -> TRACKING
//   TRACKING/HOLD/PREEMPT + RELEASE                 -> RELEASING
//   IDLE      + RELEASE                             -> accepted no-op (R11)
//   RELEASING + RELEASE                             -> accepted no-op (R11)
//   RELEASING + PAUSE_OK                            -> IDLE
//   <non-FAULT> + FAULT                             -> FAULT
//   FAULT     + RELEASE                             -> rejected (stays FAULT)
//   FAULT     + RESET                               -> IDLE
//   anything else                                   -> rejected, state unchanged
//
// StreamWatchdog is deliberately independent of any track field: the
// tracker's own `time_since_last_obs_ms` freezes when the pipeline dies, so
// liveness must be measured from message ARRIVAL time (now_ns).

#include "lgw_bt_planner/servo_state_machine.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

using lgw_bt_planner::ServoEvent;
using lgw_bt_planner::ServoState;
using lgw_bt_planner::ServoStateMachine;
using lgw_bt_planner::StreamWatchdog;

constexpr int64_t kNsPerMs = 1000000;

}  // namespace

// IDLE -> PREEMPT -> TRACKING -> RELEASING -> IDLE (spec §7.1 happy path).
TEST(StateMachine, HappyPath)
{
  ServoStateMachine sm;
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);

  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);

  ASSERT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);

  ASSERT_TRUE(sm.handle(ServoEvent::PAUSE_OK));
  EXPECT_EQ(sm.state(), ServoState::IDLE);
}

// TAKEOVER is refused while MoveGroup is executing: Servo cannot be
// unpaused and we must not steal the controller. State stays IDLE, and the
// same event succeeds once MoveGroup is idle again.
TEST(StateMachine, TakeoverRejectedWhenMoveGroupBusy)
{
  ServoStateMachine sm;

  sm.set_move_group_busy(true);
  EXPECT_FALSE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  sm.set_move_group_busy(false);
  EXPECT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);
}

// A failed Servo unpause request is not recoverable in place: PREEMPT goes
// straight to FAULT and only RESET/RELEASE rules apply from there.
TEST(StateMachine, UnpauseFailureGoesFault)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_EQ(sm.state(), ServoState::PREEMPT);

  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_FAIL));
  EXPECT_EQ(sm.state(), ServoState::FAULT);

  // The sequence is dead: a late UNPAUSE_OK must not resurrect it.
  EXPECT_FALSE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_EQ(sm.state(), ServoState::FAULT);
}

// Target loss stops the command but keeps control: TRACKING -> HOLD, and
// HOLD -> TRACKING only once the target is back (TARGET_OK) and a RESUME is
// requested. No RELEASING in between (spec §7.1: "TRACKING → HOLD 只停指令,
// 不释放控制权").
TEST(StateMachine, HoldKeepsControlAndCanResume)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  // Still holding control: RESUME before the target is back is refused.
  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// FAULT is terminal: RELEASE is refused (Servo may not be pausable from
// there), RESET is the only way out and re-arms IDLE.
TEST(StateMachine, FaultNeedsReset)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));

  ASSERT_TRUE(sm.handle(ServoEvent::FAULT));
  EXPECT_EQ(sm.state(), ServoState::FAULT);

  EXPECT_FALSE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::FAULT);

  ASSERT_TRUE(sm.handle(ServoEvent::RESET));
  EXPECT_EQ(sm.state(), ServoState::IDLE);
}

// ---------------------------------------------------------------------------
// RECOVERING (2026-11-03 hold-recovery spec §6)
// ---------------------------------------------------------------------------

namespace
{
void driveToHold(ServoStateMachine & sm)
{
  sm.handle(ServoEvent::TAKEOVER);
  sm.handle(ServoEvent::UNPAUSE_OK);
  sm.handle(ServoEvent::TARGET_LOST);
}
}  // namespace

// A failed track leaves the session in HOLD; the node may start a bounded
// recovery move from there.
TEST(StateMachine, RecoverStartFromHold)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}

// Success only resumes tracking when the target is demonstrably present.
TEST(StateMachine, RecoverOkWithFreshTargetGoesTracking)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));

  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_OK));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// Recovered pose but no target: do not enter TRACKING only to fall straight
// back into HOLD.
TEST(StateMachine, RecoverOkWithoutTargetGoesHold)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_OK));
  EXPECT_EQ(sm.state(), ServoState::HOLD);
}

TEST(StateMachine, RecoverFailReturnsToHold)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_FAIL));
  EXPECT_EQ(sm.state(), ServoState::HOLD);
}

// Resume while MoveGroup is driving the JTC would create a second writer.
TEST(StateMachine, ResumeRefusedWhileRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));

  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}

TEST(StateMachine, PauseIsNoOpWhileRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  EXPECT_TRUE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}

// Release must be able to interrupt a recovery move (the node cancels the
// MoveGroup goal in its RELEASE handler).
TEST(StateMachine, ReleaseFromRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  ASSERT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);
}

TEST(StateMachine, TargetOkArmsLatchButStaysRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  EXPECT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
  EXPECT_TRUE(sm.target_ok());
}

TEST(StateMachine, TargetLostInRecoveringClearsLatch)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));

  EXPECT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
  EXPECT_FALSE(sm.target_ok());
}

TEST(StateMachine, TakeoverRefusedWhileRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  EXPECT_FALSE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}

TEST(StateMachine, FaultFromRecovering)
{
  ServoStateMachine sm;
  driveToHold(sm);
  ASSERT_TRUE(sm.handle(ServoEvent::RECOVER_START));

  ASSERT_TRUE(sm.handle(ServoEvent::FAULT));
  EXPECT_EQ(sm.state(), ServoState::FAULT);
}

// ---------------------------------------------------------------------------
// StreamWatchdog
// ---------------------------------------------------------------------------

// Arrival-time liveness: 500 ms default timeout, measured from the last
// on_track_array() call, independent of any field carried by the track.
TEST(StreamWatchdog, FiresAfterTimeout)
{
  StreamWatchdog wd;
  wd.on_track_array(0);

  EXPECT_FALSE(wd.timed_out(400 * kNsPerMs));
  EXPECT_TRUE(wd.timed_out(600 * kNsPerMs));

  // A fresh arrival re-arms the window.
  wd.on_track_array(1000 * kNsPerMs);
  EXPECT_FALSE(wd.timed_out(1200 * kNsPerMs));
  EXPECT_TRUE(wd.timed_out(1600 * kNsPerMs));

  // Custom timeout overrides the 500 ms default.
  EXPECT_FALSE(wd.timed_out(1050 * kNsPerMs, 100));
  EXPECT_TRUE(wd.timed_out(1200 * kNsPerMs, 100));
}

// No track array ever received == stream not alive.
TEST(StreamWatchdog, NeverReceivedTimesOut)
{
  StreamWatchdog wd;
  EXPECT_TRUE(wd.timed_out(0));
  EXPECT_TRUE(wd.timed_out(123456789));
}

// A backwards clock step must not fake a timeout (elapsed < 0).
TEST(StreamWatchdog, BackwardsClockDoesNotTimeOut)
{
  StreamWatchdog wd;
  wd.on_track_array(5000 * kNsPerMs);
  EXPECT_FALSE(wd.timed_out(4000 * kNsPerMs));
}

// A track loss is recognised in HOLD too (stays HOLD) and must clear the
// resume arm: otherwise a TARGET_OK taken inside HOLD would survive the
// loss and let RESUME re-enter TRACKING on a target that is gone.
TEST(StateMachine, HoldTargetLostClearsLatch)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));  // arms inside HOLD
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));  // recognised, no move
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));  // latch was cleared
  EXPECT_EQ(sm.state(), ServoState::HOLD);
}

// /lgw/servo/pause (ruling R10, spec §7.2 "保持控制权、转 HOLD"): PAUSE acts
// on TRACKING exactly like TARGET_LOST — HOLD, control retained, no
// RELEASING. Any other state refuses it. The TARGET_OK + RESUME path out of
// HOLD is unchanged.
TEST(StateMachine, PauseFromTrackingHolds)
{
  ServoStateMachine sm;

  // PAUSE means nothing in IDLE.
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);

  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);

  ASSERT_TRUE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  // Already holding: a second PAUSE is refused, still HOLD, still in control.
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  // Leaving HOLD still needs a fresh TARGET_OK first.
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// The TARGET_OK latch is cleared when entering HOLD: an arm taken in
// TRACKING must not survive TARGET_LOST, so RESUME is refused until a new
// TARGET_OK arrives while HOLD.
TEST(StateMachine, ResumeWithoutFreshTargetOkRejected)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));  // armed in TRACKING
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_EQ(sm.state(), ServoState::HOLD);

  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::HOLD);

  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_OK));  // re-armed while HOLD
  ASSERT_TRUE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// RELEASE is idempotent (ruling R11): a BT or service caller releasing an
// already-idle / already-releasing node gets success, not a failure, and no
// IDLE -> RELEASING hop that would wait for a PAUSE_OK that never comes.
// FAULT stays terminal.
TEST(StateMachine, ReleaseIsIdempotent)
{
  ServoStateMachine sm;

  // Already released.
  EXPECT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  // Normal release path, then a duplicate.
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);
  ASSERT_TRUE(sm.handle(ServoEvent::RELEASE));
  ASSERT_EQ(sm.state(), ServoState::RELEASING);
  EXPECT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);

  ASSERT_TRUE(sm.handle(ServoEvent::PAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::IDLE);

  // FAULT is the one state that refuses RELEASE.
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::FAULT));
  ASSERT_EQ(sm.state(), ServoState::FAULT);
  EXPECT_FALSE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::FAULT);
}

// ---------------------------------------------------------------------------
// Rejected events leave the state untouched
// ---------------------------------------------------------------------------

// Every event not listed in the transition table is refused, including the
// PAUSE/RESUME pair outside their legal states. (RELEASE is deliberately NOT
// in this list: it is idempotent from IDLE/RELEASING — see
// ReleaseIsIdempotent.)
TEST(StateMachine, IllegalEventsRejected)
{
  ServoStateMachine sm;

  // Nothing may happen from IDLE except TAKEOVER (and the RELEASE no-op).
  EXPECT_FALSE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_FALSE(sm.handle(ServoEvent::UNPAUSE_FAIL));
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE_OK));
  EXPECT_FALSE(sm.handle(ServoEvent::TARGET_LOST));
  EXPECT_FALSE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE));
  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_FALSE(sm.handle(ServoEvent::RESET));
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  // RESUME is only legal from HOLD (and only after TARGET_OK).
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(sm.state(), ServoState::TRACKING);
  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_FALSE(sm.handle(ServoEvent::PAUSE_OK));
  EXPECT_FALSE(sm.handle(ServoEvent::RESET));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// TARGET_OK is only meaningful while a track is (or was) being followed.
TEST(StateMachine, TargetOkOnlyInTrackingOrHold)
{
  ServoStateMachine sm;

  EXPECT_FALSE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::IDLE);

  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_FALSE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);

  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

// Scan is entered only from HOLD; finding a target goes back through
// PREEMPT so the unpause/verify path is reused.
TEST(StateMachine, ScanOnlyFromHoldAndReturnsToPreempt)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  EXPECT_EQ(sm.state(), ServoState::SCANNING);
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_FOUND));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

TEST(StateMachine, ScanTimeoutGoesHoldAndReleaseLeaves)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_TIMEOUT));
  EXPECT_EQ(sm.state(), ServoState::HOLD);
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  ASSERT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);
}

TEST(StateMachine, ScanRejectedWhileTracking)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));  // TRACKING
  EXPECT_FALSE(sm.handle(ServoEvent::SCAN_START));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

TEST(StateMachine, ScanAcceptsTargetLatchAndPauseNoop)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  EXPECT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_TRUE(sm.handle(ServoEvent::PAUSE));
  EXPECT_EQ(sm.state(), ServoState::SCANNING);
  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_FALSE(sm.handle(ServoEvent::TAKEOVER));
}

// BT start with no target: TAKEOVER -> PREEMPT -> SCAN_START -> SCANNING.
TEST(StateMachine, PreemptScanStartGoesScanning)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  EXPECT_EQ(sm.state(), ServoState::SCANNING);
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_FOUND));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);
}
