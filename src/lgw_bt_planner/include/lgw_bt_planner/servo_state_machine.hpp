// include/lgw_bt_planner/servo_state_machine.hpp
//
// Pure state machine + stream watchdog for the MoveIt Servo takeover stack
// (spec 2026-10-26 §7.1). Decides only whether a transition is legal; the
// ROS node that owns the Servo services (Task 7) is the caller.
//
// RULING R2 (SDD 2026-10-26): the architecture no longer switches
// controllers — it pauses / unpauses MoveIt Servo. The events are therefore
// named after Servo:
//   UNPAUSE_OK   = Servo unpause request succeeded  (was SWITCH_IN_OK)
//   UNPAUSE_FAIL = Servo unpause request failed     (was SWITCH_IN_FAIL)
//   PAUSE_OK     = Servo pause request succeeded    (was SWITCH_OUT_OK)
//
// TRANSITION TABLE (authoritative — this module implements exactly this and
// nothing more):
//
//   IDLE      + TAKEOVER (move_group_busy == false) -> PREEMPT
//   IDLE      + TAKEOVER (move_group_busy == true)  -> rejected, stays IDLE
//   PREEMPT   + UNPAUSE_OK                          -> TRACKING
//   PREEMPT   + UNPAUSE_FAIL                        -> FAULT
//   TRACKING  + TARGET_LOST                         -> HOLD
//   HOLD      + TARGET_LOST                         -> HOLD (recognised, stays)
//   TRACKING  + PAUSE                               -> HOLD
//   HOLD      + RESUME   (after TARGET_OK)          -> TRACKING
//   HOLD      + RECOVER_START                       -> RECOVERING
//   RECOVERING+ RECOVER_OK + TARGET_OK              -> TRACKING
//   RECOVERING+ RECOVER_OK without TARGET_OK        -> HOLD
//   RECOVERING+ RECOVER_FAIL                        -> HOLD
//   RECOVERING+ TARGET_OK / TARGET_LOST             -> RECOVERING (latch only)
//   RECOVERING+ PAUSE                               -> accepted no-op
//   RECOVERING+ RESUME / TAKEOVER                   -> rejected
//   HOLD      + SCAN_START                          -> SCANNING
//   SCANNING  + SCAN_FOUND                          -> PREEMPT
//   SCANNING  + SCAN_TIMEOUT / SCAN_FAIL            -> HOLD
//   SCANNING  + TARGET_OK / TARGET_LOST             -> SCANNING (latch only)
//   SCANNING  + PAUSE                               -> accepted no-op
//   SCANNING  + RESUME / TAKEOVER / SCAN_START      -> rejected
//   HOLD      + SCAN_START                          -> SCANNING
//   SCANNING  + SCAN_FOUND                          -> PREEMPT
//   SCANNING  + SCAN_TIMEOUT / SCAN_FAIL            -> HOLD
//   SCANNING  + TARGET_OK / TARGET_LOST             -> SCANNING (latch only)
//   SCANNING  + PAUSE                               -> accepted no-op
//   SCANNING  + RESUME / TAKEOVER / SCAN_START      -> rejected
//   RECOVERING+ RELEASE                             -> RELEASING
//   TRACKING  + RELEASE                             -> RELEASING
//   HOLD      + RELEASE                             -> RELEASING
//   PREEMPT   + RELEASE                             -> RELEASING
//   IDLE      + RELEASE                             -> accepted no-op, stays IDLE
//   RELEASING + RELEASE                             -> accepted no-op, stays RELEASING
//   RELEASING + PAUSE_OK                            -> IDLE
//   <non-FAULT> + FAULT                             -> FAULT
//   FAULT     + RELEASE                             -> rejected, stays FAULT
//   FAULT     + RESET                               -> IDLE
//   anything else                                   -> rejected, state unchanged
//
// Notes on the events that appear in the table only as conditions:
//   * TARGET_OK is the "a usable track is present" signal. It is accepted
//     (and only accepted) in TRACKING and HOLD, where it arms the RESUME
//     path, so HOLD + RESUME without a fresh TARGET_OK is refused —
//     "HOLD + RESUME (after TARGET_OK)".
//   * TARGET_LOST clears that arm in EVERY state and is recognised in HOLD
//     (stay HOLD) as well as in TRACKING (-> HOLD), so a TARGET_OK taken
//     while holding cannot survive a later loss and resurrect RESUME.
//     PAUSE clears it the same way.
//   * PAUSE is the `/lgw/servo/pause` request (ruling R10, spec §7.2 "保持
//     控制权、转 HOLD"): TRACKING -> HOLD, the same destination as
//     TARGET_LOST and likewise keeping control (no pause/unpause Servo call
//     is implied here — the node maps that). PAUSE in any other state is
//     refused, so a paused session still needs TARGET_OK + RESUME.
//   * RELEASE is idempotent (ruling R11): from IDLE and RELEASING it is an
//     accepted no-op (returns true, state unchanged), so a BT / service
//     caller releasing an already-released node does not see a failure and
//     IDLE never routes through RELEASING waiting for a PAUSE_OK that will
//     not come. FAULT still refuses RELEASE.
//
// handle() returns false for a rejected event and leaves the state
// untouched. Accepted FAULT/RESET/no-op events return true.
//
// StreamWatchdog measures liveness by MESSAGE ARRIVAL time only. The
// tracker's own `time_since_last_obs_ms` (and any other track field)
// freezes when the perception pipeline dies, so it cannot be used to detect
// a dead stream (spec §7.7 / risk R2). Pass the wall-clock arrival time of
// each UnifiedTrackArray to on_track_array(); timed_out() answers whether
// at least timeout_ms have elapsed since then.
//   - never received: timed_out() == true (the stream is not alive yet).
//   - timed out when elapsed >= timeout_ms.
//   - a backwards clock step (elapsed < 0) never fires.
//
// Pure C++17: no rclcpp, no ROS runtime, no BT, no Eigen.

#ifndef LGW_BT_PLANNER__SERVO_STATE_MACHINE_HPP_
#define LGW_BT_PLANNER__SERVO_STATE_MACHINE_HPP_

#include <cstdint>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Events fed to ServoStateMachine::handle. See the transition table above.
enum class ServoEvent
{
  TAKEOVER,
  UNPAUSE_OK,
  UNPAUSE_FAIL,
  RELEASE,
  PAUSE_OK,
  TARGET_LOST,
  TARGET_OK,
  PAUSE,
  RESUME,
  // HOLD recovery (spec 2026-11-03 §6):
  //   RECOVER_START = node decided to move the arm with MoveGroup
  //   RECOVER_OK    = move + re-seed + probe succeeded
  //   RECOVER_FAIL  = move/verify/start_servo failed, or the budget ran out
  RECOVER_START,
  RECOVER_OK,
  RECOVER_FAIL,
  // Scan (2026-11-13): only HOLD + kTargetLost may enter; the worker drives
  // the JTC through MoveGroup while Servo stays paused.
  SCAN_START,
  SCAN_FOUND,
  SCAN_TIMEOUT,
  SCAN_FAIL,
  FAULT,
  RESET,
};

class ServoStateMachine
{
public:
  ServoStateMachine() = default;

  ServoState state() const {return state_;}

  // Fresh-target latch. Exported for ServoState.resumable; the value itself
  // still gates HOLD + RESUME and RECOVERING + RECOVER_OK.
  bool target_ok() const {return target_ok_;}

  // Whether MoveGroup is executing a goal. TAKEOVER is only legal from
  // IDLE while this is false: Servo cannot be unpaused underneath a
  // running MoveGroup goal.
  void set_move_group_busy(bool busy) {move_group_busy_ = busy;}

  // Apply `event`. Returns true when the transition was accepted (the state
  // may be unchanged for accepted no-op-ish events such as TARGET_OK) and
  // false when the event is illegal in the current state (state unchanged).
  bool handle(ServoEvent event);

private:
  ServoState state_{ServoState::IDLE};
  bool move_group_busy_{false};
  // Set by TARGET_OK while TRACKING/HOLD, cleared by TARGET_LOST, PAUSE and
  // by leaving HOLD. Gates HOLD + RESUME.
  bool target_ok_{false};
};

// Arrival-time watchdog on the unified tracker stream.
class StreamWatchdog
{
public:
  StreamWatchdog() = default;

  // Record that a UnifiedTrackArray arrived at `now_ns`.
  void on_track_array(int64_t now_ns);

  // True when at least `timeout_ms` elapsed since the last arrival (or when
  // nothing ever arrived).
  bool timed_out(int64_t now_ns, int64_t timeout_ms = 500) const;

private:
  bool has_arrival_{false};
  int64_t last_arrival_ns_{0};
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO_STATE_MACHINE_HPP_
