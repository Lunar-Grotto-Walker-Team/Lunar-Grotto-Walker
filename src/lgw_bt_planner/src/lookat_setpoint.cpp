// src/lookat_setpoint.cpp
// Pure lookat-setpoint math (servo takeover plan Task 2).
//
// Reuses build_lookat_orientation + compute_tcp_pose from
// compute_lookat_pose_node.hpp — do NOT reimplement the hand-eye chain
// here. In particular there is no link6 <-> tcp conversion: compute_tcp_pose
// already returns the tcp_link pose (SRDF `arm` group tip, spec §7.6).

#include "lgw_bt_planner/lookat_setpoint.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <stdexcept>

namespace lgw_bt_planner
{

LookatResult compute_lookat_setpoint(
  const tf2::Vector3& p_target_base,
  const tf2::Matrix3x3& R_cam_in_tcp,
  const tf2::Vector3& t_cam_in_tcp,
  const LookatParams& params)
{
  LookatResult out;

  // 1. Pick the end-effector (arm-camera-origin) position. On the base z
  //    axis the xoy orbit direction is undefined -> fall back to the
  //    configured anchor, keeping the legacy anchor pose.
  tf2::Vector3 P_ee_in_base;
  const double r_xy = std::hypot(p_target_base.x(), p_target_base.y());
  if (r_xy < 1e-3) {
    P_ee_in_base = params.ee_anchor;
    out.used_anchor = true;
  } else {
    P_ee_in_base.setValue(
      params.xoy_distance_m * p_target_base.x() / r_xy,
      params.xoy_distance_m * p_target_base.y() / r_xy,
      params.z_fixed_m);
  }

  // 2. Look direction. Guard explicitly before normalizing: a zero-norm
  //    vector would yield NaNs that slip past build_lookat_orientation's
  //    own norm check (NaN comparisons are always false), so we throw here
  //    instead of emitting a NaN rotation to Servo.
  const tf2::Vector3 delta = p_target_base - P_ee_in_base;
  if (delta.length() < 1e-6) {
    throw std::runtime_error(
      "compute_lookat_setpoint: target coincides with the chosen end-effector "
      "position; look direction is undefined");
  }
  const tf2::Vector3 direction = delta.normalized();

  // 3. Arm camera optical frame in base_link: cam_z = direction, then
  //    enforce camera uprightness (cam_y · world_up < 0) with the same
  //    shared helper the BT lookat nodes use, so Servo and the BT path
  //    command the same orientation.
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  tf2::Matrix3x3 R_arm_in_base = apply_uprightness_flip(
    build_lookat_orientation(direction, world_up), world_up);

  // 3b. Optional roll about the optical axis (cam_z). Rz is applied on the
  //     right, so cam_z (column 2) is untouched and the target stays exactly
  //     on the optical axis; only the image rotates. This is the only free
  //     orientation DOF once the camera is aimed, and the only way to move
  //     the wrist joint q5 off zero without pointing away from the target.
  if (params.camera_roll_rad != 0.0) {
    const double c = std::cos(params.camera_roll_rad);
    const double s = std::sin(params.camera_roll_rad);
    // Row-major: rotation about +z (the optical axis), NOT about x.
    const tf2::Matrix3x3 Rz(c, -s, 0.0,
      s, c, 0.0,
      0.0, 0.0, 1.0);
    R_arm_in_base = R_arm_in_base * Rz;
  }

  // 4. Desired tcp_link pose (camera standoff point + hand-eye chain).
  const geometry_msgs::msg::PoseStamped tcp_pose =
    compute_tcp_pose(P_ee_in_base, R_arm_in_base, R_cam_in_tcp, t_cam_in_tcp);

  out.tcp_desired.p.setValue(
    tcp_pose.pose.position.x,
    tcp_pose.pose.position.y,
    tcp_pose.pose.position.z);

  tf2::Quaternion q(
    tcp_pose.pose.orientation.x,
    tcp_pose.pose.orientation.y,
    tcp_pose.pose.orientation.z,
    tcp_pose.pose.orientation.w);
  q.normalize();
  // Pose6::R is uninitialized on default construction — assign it.
  out.tcp_desired.R = tf2::Matrix3x3(q);

  // No target distance here (controller ruling R9): it would be measured
  // from the *desired* pose, whose standoff is fixed at xoy_distance_m,
  // so it could never fall below a too_close threshold. The servo node
  // measures proximity from the CURRENT camera pose instead.

  return out;
}

}  // namespace lgw_bt_planner
