// src/servo/servo_state_reporter.cpp
//
// Task 8, step 4b of the 2026-11-07 node-split plan. See the header for the
// rules this applies and why each one exists.

#include "lgw_bt_planner/servo/servo_state_reporter.hpp"

namespace lgw_bt_planner
{

msg::ServoState build_servo_state(const ServoStateSnapshot & snap)
{
  const bool tracking = snap.state == ServoState::TRACKING;
  // HOLD keeps reporting the last known tier/errors so a field operator can
  // see WHY it is holding; IDLE/FAULT report nothing (no session).
  const std::string source =
    snap.session_active ? snap.geometry_source : std::string("none");

  msg::ServoState message;
  message.stamp.sec = static_cast<int32_t>(snap.stamp_ns / 1000000000LL);
  message.stamp.nanosec = static_cast<uint32_t>(snap.stamp_ns % 1000000000LL);
  message.state = std::string(state_name(snap.state));
  message.geometry_source = source;
  message.track_id = snap.session_active ? snap.track_id : 0U;
  message.angular_error_rad =
    snap.session_active ? snap.angular_error_rad : 0.0;
  message.position_error_m = snap.session_active ? snap.position_error_m : 0.0;
  message.target_distance_m = snap.session_active ? snap.target_distance_m : 0.0;
  message.converged = tracking && snap.converged;
  message.saturated = snap.saturated;
  message.too_close = snap.session_active && snap.too_close;
  message.needs_preposition = snap.needs_preposition;
  message.max_joint_delta = snap.max_joint_delta;
  message.hold_reason = std::string(hold_reason_name(snap.hold_reason));
  message.recovery_attempts = static_cast<uint32_t>(snap.recovery_attempts);
  message.recovery_exhausted = snap.recovery_exhausted;
  message.resumable = snap.resumable;
  if (snap.hold_since_ns > 0) {
    message.hold_since.sec = static_cast<int32_t>(snap.hold_since_ns / 1000000000LL);
    message.hold_since.nanosec =
      static_cast<uint32_t>(snap.hold_since_ns % 1000000000LL);
  }
  return message;
}

}  // namespace lgw_bt_planner
