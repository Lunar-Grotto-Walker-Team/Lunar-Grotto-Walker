// include/lgw_bt_planner/servo/scan_trajectory.hpp
//
// Continuous joint-space scan (2026-11-15 rev5).  The scan's purpose is to
// enlarge the camera's effective field of view, so the arm holds j1 at a
// fixed angle and the observation posture (j2/j3), and only the wrist looks
// around: j4 and j5 trace a CIRCLE around the observation pose — the view
// direction sweeps a cone without ever passing back through the centre —
// while j6 keeps the image level (see scan_roll_lock).
//
// The module is pure (no ROS node, no clock): measured joints + parameters
// in, trajectory message out.

#ifndef LGW_BT_PLANNER__SERVO__SCAN_TRAJECTORY_HPP_
#define LGW_BT_PLANNER__SERVO__SCAN_TRAJECTORY_HPP_

#include <array>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace lgw_bt_planner
{

struct ScanCircleParams
{
  double j1_rad{0.0};              // sway centre
  double j1_radius_rad{0.349};     // +-20 deg sway, one cycle per circle
  double j2_rad{0.890};            // observation posture, held
  double j3_rad{-0.720};           // observation posture, held
  double j4_radius_rad{1.2};       // +-69 deg pan around the posture
  double j5_radius_rad{1.1};       // +-63 deg tilt (near the +-70 deg limit)
  double period_s{8.0};            // one full circle
  double duration_s{60.0};         // length of one published trajectory
  double sample_rate_hz{50.0};
  double max_velocity_rad_s{2.5};  // hard limit; a too-fast circle is slowed
};

// Starts on the circle (j4 = +radius, j5 = 0) and runs for whole periods so
// consecutive trajectories chain seamlessly.  j6 is left at the measured
// value here; the caller applies the roll lock.
trajectory_msgs::msg::JointTrajectory make_scan_circle(
  const std::array<double, 6> & measured,
  const ScanCircleParams & params,
  const builtin_interfaces::msg::Time & start);

// A constant-pose trajectory used to stop the scan immediately: the JTC
// preempts whatever it is executing when a new trajectory arrives.
trajectory_msgs::msg::JointTrajectory make_hold_trajectory(
  const std::array<double, 6> & measured,
  const std::array<std::string, 6> & names,
  double hold_s,
  const builtin_interfaces::msg::Time & start);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SCAN_TRAJECTORY_HPP_
