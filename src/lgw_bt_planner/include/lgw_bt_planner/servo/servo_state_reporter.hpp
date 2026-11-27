// include/lgw_bt_planner/servo/servo_state_reporter.hpp
//
// Task 8, step 4b of the 2026-11-07 node-split plan (spec S5). Building the
// /lgw/servo/state message was 40 lines inside the node, and its rules are
// invisible from the outside: which fields are zeroed when there is no
// session, which ones keep reporting during HOLD, and that `converged` is
// only ever true while TRACKING. Those rules now have a name and a test.
//
// The publish CADENCE stays in the node — it depends on the tick counter and
// on remembering the last published state, which is transport policy, not
// message content.

#ifndef LGW_BT_PLANNER__SERVO__SERVO_STATE_REPORTER_HPP_
#define LGW_BT_PLANNER__SERVO__SERVO_STATE_REPORTER_HPP_

#include <cstdint>
#include <string>

#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/msg/servo_state.hpp"
#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Everything the message says, gathered by whoever holds the values.
struct ServoStateSnapshot
{
  int64_t stamp_ns{0};
  ServoState state{ServoState::IDLE};
  // False in IDLE / FAULT / RELEASING: no session, so nothing to report.
  bool session_active{false};
  std::string geometry_source{"none"};
  uint64_t track_id{0};
  double angular_error_rad{0.0};
  double position_error_m{0.0};
  double target_distance_m{0.0};
  bool converged{false};
  bool saturated{false};
  bool too_close{false};
  bool needs_preposition{false};
  double max_joint_delta{0.0};
  HoldReason hold_reason{HoldReason::kNone};
  int recovery_attempts{0};
  bool recovery_exhausted{false};
  bool resumable{false};
  // 0 means "not holding".
  int64_t hold_since_ns{0};
};

// Rules applied here, all of them deliberate:
//   * without a session the geometry fields are reported empty, not stale:
//     a field operator reading "head, 0.02 m error" while the node is IDLE
//     would believe the arm is tracking something;
//   * HOLD DOES report the last known tier and errors — that is how the
//     operator sees why it is holding;
//   * converged is only ever true while TRACKING, because the deadband is
//     meaningless when nothing is being commanded.
msg::ServoState build_servo_state(const ServoStateSnapshot & snap);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SERVO_STATE_REPORTER_HPP_
