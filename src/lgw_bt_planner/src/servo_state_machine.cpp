// src/servo_state_machine.cpp
//
// Implementation of the spec §7.1 takeover state machine and the tracker
// stream watchdog. See servo_state_machine.hpp for the authoritative
// transition table and the R2 event naming. Pure C++17, no ROS.

#include "lgw_bt_planner/servo_state_machine.hpp"

namespace lgw_bt_planner
{

bool ServoStateMachine::handle(ServoEvent event)
{
  // FAULT / RESET are handled ahead of the per-state switch because they
  // apply to several states at once:
  //   RESET is legal only from FAULT (and is the ONLY way out of it);
  //   FAULT is legal from every non-FAULT state.
  if (event == ServoEvent::RESET) {
    if (state_ != ServoState::FAULT) {
      return false;
    }
    state_ = ServoState::IDLE;
    target_ok_ = false;
    return true;
  }
  if (event == ServoEvent::FAULT) {
    if (state_ == ServoState::FAULT) {
      return false;  // already faulted; state unchanged
    }
    state_ = ServoState::FAULT;
    target_ok_ = false;
    return true;
  }
  // A track loss always invalidates the RESUME arm, in every state — most
  // importantly in HOLD, where a stale TARGET_OK would otherwise let RESUME
  // re-enter TRACKING on a target that is gone.
  if (event == ServoEvent::TARGET_LOST) {
    target_ok_ = false;
    switch (state_) {
      case ServoState::TRACKING:
        state_ = ServoState::HOLD;
        return true;
      case ServoState::HOLD:
        return true;  // recognised: still holding, no transition
      case ServoState::RECOVERING:
        return true;  // the move continues; the latch is cleared above
      case ServoState::SCANNING:
        return true;  // the scan continues; the latch is cleared above
      default:
        return false;
    }
  }
  // RELEASE is idempotent (ruling R11): releasing an already-idle or
  // already-releasing session is an accepted no-op, so a BT / service
  // caller never sees a spurious failure and IDLE never hops through
  // RELEASING waiting for a PAUSE_OK that will not come. FAULT refuses it.
  if (event == ServoEvent::RELEASE) {
    switch (state_) {
      case ServoState::IDLE:
      case ServoState::RELEASING:
        return true;  // already where RELEASE wants us
      case ServoState::PREEMPT:
      case ServoState::TRACKING:
      case ServoState::HOLD:
      case ServoState::RECOVERING:
      case ServoState::SCANNING:
        state_ = ServoState::RELEASING;
        target_ok_ = false;
        return true;
      case ServoState::FAULT:
        return false;  // terminal: only RESET gets out
    }
  }

  switch (state_) {
    case ServoState::IDLE:
      // Servo must not be unpaused underneath a running MoveGroup goal.
      if (event == ServoEvent::TAKEOVER && !move_group_busy_) {
        state_ = ServoState::PREEMPT;
        target_ok_ = false;
        return true;
      }
      return false;

    case ServoState::PREEMPT:
      if (event == ServoEvent::SCAN_START) {
        // BT started the session with no selectable target: skip the
        // unpause/verify path and go straight to the scan worker.
        state_ = ServoState::SCANNING;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::UNPAUSE_OK) {
        state_ = ServoState::TRACKING;
        return true;
      }
      if (event == ServoEvent::UNPAUSE_FAIL) {
        state_ = ServoState::FAULT;
        target_ok_ = false;
        return true;
      }
      return false;

    case ServoState::TRACKING:
      if (event == ServoEvent::PAUSE) {
        // /lgw/servo/pause (R10, spec §7.2): same destination as
        // TARGET_LOST — stop commanding, keep control. The latch is cleared
        // like TARGET_LOST, so leaving HOLD still needs a fresh TARGET_OK.
        state_ = ServoState::HOLD;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::TARGET_OK) {
        target_ok_ = true;
        return true;
      }
      return false;

    case ServoState::HOLD:
      if (event == ServoEvent::TARGET_OK) {
        target_ok_ = true;
        return true;
      }
      if (event == ServoEvent::RESUME && target_ok_) {
        state_ = ServoState::TRACKING;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::RECOVER_START) {
        // Entered only from HOLD (spec 2026-11-03 §6): Servo stays paused,
        // the node starts a bounded MoveGroup move.
        state_ = ServoState::RECOVERING;
        return true;
      }
      if (event == ServoEvent::SCAN_START) {
        // Entered only from HOLD (and the node gates it on kTargetLost):
        // Servo stays paused, MoveGroup sweeps the scan trajectory.
        state_ = ServoState::SCANNING;
        target_ok_ = false;
        return true;
      }
      return false;

    case ServoState::RECOVERING:
      // The recovery worker owns the JTC until it reports success/failure.
      if (event == ServoEvent::TARGET_OK) {
        target_ok_ = true;
        return true;
      }
      if (event == ServoEvent::PAUSE) {
        return true;  // already paused; accepted no-op
      }
      if (event == ServoEvent::RECOVER_OK) {
        // Only resume tracking when the target is demonstrably present;
        // otherwise stay in HOLD rather than bounce TRACKING->HOLD.
        state_ = target_ok_ ? ServoState::TRACKING : ServoState::HOLD;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::RECOVER_FAIL) {
        state_ = ServoState::HOLD;
        target_ok_ = false;
        return true;
      }
      // RESUME is refused: unpausing Servo while MoveGroup drives the JTC
      // would create a second writer. RELEASE is the safe interrupt.
      return false;

    case ServoState::SCANNING:
      if (event == ServoEvent::SCAN_FOUND) {
        // Cancel the scan, then run the normal unpause/verify path.
        state_ = ServoState::PREEMPT;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::SCAN_TIMEOUT || event == ServoEvent::SCAN_FAIL) {
        state_ = ServoState::HOLD;
        target_ok_ = false;
        return true;
      }
      if (event == ServoEvent::TARGET_OK) {
        target_ok_ = true;
        return true;
      }
      if (event == ServoEvent::PAUSE) {
        return true;  // Servo is already paused; accepted no-op
      }
      // RESUME / TAKEOVER / SCAN_START are refused: another writer or a
      // duplicate scan would race the worker.
      return false;

    case ServoState::RELEASING:
      if (event == ServoEvent::PAUSE_OK) {
        state_ = ServoState::IDLE;
        return true;
      }
      return false;

    case ServoState::FAULT:
      // Only RESET leaves FAULT; RELEASE in particular is refused.
      return false;
  }

  return false;  // unreachable for a valid enumerator
}

void StreamWatchdog::on_track_array(int64_t now_ns)
{
  last_arrival_ns_ = now_ns;
  has_arrival_ = true;
}

bool StreamWatchdog::timed_out(int64_t now_ns, int64_t timeout_ms) const
{
  if (!has_arrival_) {
    // No track array ever arrived: the stream is not alive.
    return true;
  }
  const int64_t elapsed_ns = now_ns - last_arrival_ns_;
  if (elapsed_ns < 0) {
    // Backwards clock step; never fabricate a timeout.
    return false;
  }
  return elapsed_ns >= timeout_ms * INT64_C(1000000);
}

}  // namespace lgw_bt_planner
