// include/lgw_bt_planner/servo/scan_roll_lock.hpp
//
// Keeps the arm camera's image upright during SCANNING (2026-11-15).
//
// The Lissajous moves j4/j5, which rolls the camera about its optical axis;
// a detector trained on upright images sees every object tilted.  The wrist
// roll joint (j6) rotates the camera about (almost) that same axis, so it can
// cancel the roll: for every trajectory sample we solve j6 such that the
// image roll matches the reference pose.
//
// Pure module: the node supplies the forward kinematics through a callback.

#ifndef LGW_BT_PLANNER__SERVO__SCAN_ROLL_LOCK_HPP_
#define LGW_BT_PLANNER__SERVO__SCAN_ROLL_LOCK_HPP_

#include <array>
#include <functional>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace lgw_bt_planner
{

// Roll of the image about the optical axis, in radians, with 0 meaning the
// image's up vector points at the world's up (projected into the image
// plane).  `R_cam_in_base` maps camera coordinates to base coordinates.
double image_roll(const tf2::Matrix3x3 & R_cam_in_base);

using JointRollFn = std::function<double(const std::array<double, 6> &)>;

// Rewrites joint index 5 of every sample so that image_roll() stays at
// `roll_ref`.  `roll_of` must return image_roll() for the given joints.
// j6 is clamped into [j6_min, j6_max]; if the compensation is unreachable
// the sample is left at the clamped best effort.
// Returns false when the trajectory is empty or the FK callback throws.
bool lock_camera_roll(
  trajectory_msgs::msg::JointTrajectory & traj,
  const JointRollFn & roll_of,
  double roll_ref,
  double j6_min, double j6_max);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SCAN_ROLL_LOCK_HPP_
