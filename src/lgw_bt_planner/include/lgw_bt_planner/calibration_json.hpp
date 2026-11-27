// include/lgw_bt_planner/calibration_json.hpp
//
// Hand-eye calibration loader, split into its own header so the light
// `lgw_posture` library can use it without dragging in the BT/ROS headers
// that compute_lookat_pose_node.hpp pulls (behaviortree_cpp, rclcpp).

#ifndef LGW_BT_PLANNER__CALIBRATION_JSON_HPP_
#define LGW_BT_PLANNER__CALIBRATION_JSON_HPP_

#include <string>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace lgw_bt_planner
{

// Read an arm/head hand-eye calibration JSON.
//
// Returns (R, t) where R maps camera-frame vectors into the parent frame and t
// is the camera origin in that frame. The quaternion in the file is xyzw.
// Throws std::runtime_error on any missing field, wrong length or zero-norm
// quaternion — a silently zeroed calibration would put a constant offset on
// every setpoint.
std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string & path);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__CALIBRATION_JSON_HPP_
