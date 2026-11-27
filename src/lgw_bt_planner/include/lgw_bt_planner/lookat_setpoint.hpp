// include/lgw_bt_planner/lookat_setpoint.hpp
//
// Pure lookat-setpoint math for the MoveIt Servo takeover stack
// (spec 2026-10-26 §7.5). Turns "target position in the arm base_link
// frame" into the desired tcp_link pose that the Servo loop drives to.
//
// Pure tf2 math only: no rclcpp, no ROS runtime, no BT, no Eigen.

#ifndef LGW_BT_PLANNER__LOOKAT_SETPOINT_HPP_
#define LGW_BT_PLANNER__LOOKAT_SETPOINT_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Where to put the end effector while looking at the target.
//   xoy_distance_m : radius of the orbit circle around base_link origin
//   z_fixed_m      : fixed end-effector height in base_link
//   ee_anchor      : fallback position when the target lies on the base z
//                    axis (its xoy direction is undefined)
struct LookatParams
{
  double xoy_distance_m{0.4};
  double z_fixed_m{0.3};
  tf2::Vector3 ee_anchor{0.4, 0.0, 0.2};
  // Roll about the camera OPTICAL axis, applied after the uprightness rule.
  //
  // Aiming the camera at the target only fixes 2 of the 3 orientation DOF:
  // the roll about the optical axis is free (the image rotates, the target
  // stays exactly on the optical axis). On a 6R arm the wrist joint q5 is
  // determined by the orientation, so an upright camera at a given standoff
  // can sit exactly on the wrist singularity (q5 = 0) — measured on this
  // arm: cond(J) = 311 with the upright rule at xoy 0.40 / z 0.30. Rolling
  // moves q5 off zero because q4/q6 absorb the rotation.
  //
  // Limits found by sweeping the real chain (scripts/lookat_reachability.py):
  // |roll| is capped near 35 deg by joint4's +/-1.745 rad limit, so roll is
  // a posture aid, not a free knob. Keep camera_roll_rad = 0 when the
  // downstream consumer needs a world-upright image.
  double camera_roll_rad{0.0};
};

struct LookatResult
{
  Pose6 tcp_desired;          // desired tcp_link pose = Servo reference point
  bool used_anchor{false};
};

// Compute the desired tcp_link pose that points the arm camera at
// p_target_base (all vectors in the arm base_link frame).
//
// 1. r_xy = hypot(p_target.x, p_target.y); r_xy < 1e-3 -> P_ee = ee_anchor
//    (used_anchor = true), else P_ee = (d*x/r_xy, d*y/r_xy, z_fixed).
// 2. direction = normalize(p_target - P_ee).
// 3. R_arm_in_base = build_lookat_orientation(direction, +z), then
//    apply_uprightness_flip() so cam_y points downward (same orientation
//    the BT lookat nodes command).
// 4. tcp_desired = compute_tcp_pose(P_ee, R_arm_in_base, camera calibration).
//
// No target distance is returned on purpose (controller ruling R9): the
// desired standoff is fixed, so a distance measured from THIS pose would
// always be >= xoy_distance_m and a too_close guard would never fire.
// Proximity must be measured from the CURRENT camera pose
// (cam_now = tcp_now.p + tcp_now.R * t_cam_in_tcp) against p_target, which
// only the servo node (Task 7) can do.
//
// R_cam_in_tcp / t_cam_in_tcp are the arm-camera hand-eye calibration
// (arm_calibration.json, the _tcp variant): rotation maps camera-frame
// vectors into tcp_link, translation is the camera origin in tcp_link.
//
// Throws std::runtime_error if p_target_base coincides with the chosen
// P_ee (look direction undefined — e.g. target exactly at the orbit
// point). Callers must catch it and treat that tick as "no setpoint"
// (HOLD); it must never reach Servo as a NaN orientation.
LookatResult compute_lookat_setpoint(
  const tf2::Vector3& p_target_base,
  const tf2::Matrix3x3& R_cam_in_tcp,
  const tf2::Vector3& t_cam_in_tcp,
  const LookatParams& params);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_SETPOINT_HPP_
