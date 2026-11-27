// servo_types.hpp — Shared value types for the visual-servo takeover stack.
//
// Pure math / POD types only: no rclcpp, no ROS runtime, no Eigen in the
// public surface. Math is tf2 (header-only Vector3 / Matrix3x3).
//
// NAMING RULING (SDD 2026-10-26, R5): this header's `ServoState` is an
// `enum class` in namespace `lgw_bt_planner`; the ROS topic message is
// `lgw_bt_planner::msg::ServoState` (msg/ServoState.msg). They are
// deliberately distinct types that happen to share a name — never write
// `using namespace lgw_bt_planner::msg;` in translation units that include
// this header.

#ifndef LGW_BT_PLANNER__SERVO_TYPES_HPP_
#define LGW_BT_PLANNER__SERVO_TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace lgw_bt_planner
{

// Servo state machine (spec §7.1). Mirrored by the `state` string field of
// msg/ServoState.
enum class ServoState
{
  IDLE,
  PREEMPT,
  TRACKING,
  HOLD,
  // Active sub-state of HOLD: Servo stays paused and this node is driving
  // the JTC through MoveGroup (spec 2026-11-03 §6). Entered from HOLD only.
  RECOVERING,
  // Active sub-state of HOLD: Servo stays paused and a MoveGroup scan
  // trajectory sweeps the arm while the active target list is empty.
  SCANNING,
  RELEASING,
  FAULT,
};

// Rigid pose: translation + rotation.
struct Pose6
{
  tf2::Vector3 p{0, 0, 0};
  tf2::Matrix3x3 R;
};

// One published target as the servo consumes it.  The tracker owns all
// per-camera/fusion decisions; this is a single current point in arm base.
struct TrackSnapshot
{
  uint64_t track_id{0};
  std::string task_state;
  std::string source;              // arm / head / odom
  tf2::Vector3 p_odom{0, 0, 0};
  tf2::Vector3 p_arm_base{0, 0, 0};
  bool geometry_valid{false};
  int64_t last_seen_ns{0};
};

// Finiteness gates. A NaN that reaches Servo turns into a joint command, and
// deadband reporting is NOT evidence of convergence.
inline bool vector_finite(const tf2::Vector3 & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool rotation_finite(const tf2::Matrix3x3 & R)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(R[row][col])) {
        return false;
      }
    }
  }
  return true;
}

inline bool pose_finite(const Pose6 & pose)
{
  return vector_finite(pose.p) && rotation_finite(pose.R);
}

// Stable string for logs and msg/ServoState.
inline const char * state_name(ServoState state)
{
  switch (state) {
    case ServoState::IDLE:
      return "IDLE";
    case ServoState::PREEMPT:
      return "PREEMPT";
    case ServoState::TRACKING:
      return "TRACKING";
    case ServoState::HOLD:
      return "HOLD";
    case ServoState::RECOVERING:
      return "RECOVERING";
    case ServoState::SCANNING:
      return "SCANNING";
    case ServoState::RELEASING:
      return "RELEASING";
    case ServoState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO_TYPES_HPP_
