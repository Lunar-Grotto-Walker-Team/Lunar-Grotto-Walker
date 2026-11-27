// src/servo/scan_trajectory.cpp — see the header for the contract.

#include "lgw_bt_planner/servo/scan_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace lgw_bt_planner
{

namespace
{

constexpr std::size_t kDof = 6;

// The arm's joint names, in MoveIt's order.  Kept local so this module stays
// free of the servo node's headers; the names never change for this arm.
constexpr const char * kJointNames[kDof] = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

void set_duration(builtin_interfaces::msg::Duration & d, double seconds)
{
  const double total = std::max(0.0, seconds);
  d.sec = static_cast<int32_t>(std::floor(total));
  d.nanosec = static_cast<uint32_t>(
    std::llround((total - static_cast<double>(d.sec)) * 1e9));
  if (d.nanosec >= 1000000000u) {
    d.sec += 1;
    d.nanosec -= 1000000000u;
  }
}

}  // namespace

trajectory_msgs::msg::JointTrajectory make_scan_circle(
  const std::array<double, 6> & measured,
  const ScanCircleParams & params,
  const builtin_interfaces::msg::Time & start)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = start;
  for (std::size_t i = 0; i < kDof; ++i) {
    traj.joint_names.emplace_back(kJointNames[i]);
  }
  if (params.period_s <= 0.0 || params.duration_s <= 0.0 ||
    params.sample_rate_hz <= 0.0 || params.max_velocity_rad_s <= 0.0 ||
    (params.j4_radius_rad == 0.0 && params.j5_radius_rad == 0.0))
  {
    return traj;
  }

  // A circle that would break the velocity limit is stretched in time (same
  // shape, slower), never clipped.
  const double w_nominal = 2.0 * M_PI / params.period_s;
  const double peak_nominal = std::max(
    std::abs(params.j4_radius_rad), std::abs(params.j5_radius_rad)) * w_nominal;
  const double stretch = std::max(1.0, peak_nominal / params.max_velocity_rad_s);
  const double w = w_nominal / stretch;
  const double period = params.period_s * stretch;

  // Snap to whole circles so consecutive trajectories start and end at
  // exactly the same pose and velocity: the seam is continuous.
  const long circles = std::max(
    1L, static_cast<long>(std::lround(params.duration_s / period)));
  const double duration = static_cast<double>(circles) * period;

  const long steps = std::max(
    1L, static_cast<long>(std::lround(duration * params.sample_rate_hz)));
  const double dt = duration / static_cast<double>(steps);

  traj.points.reserve(static_cast<std::size_t>(steps) + 1);
  for (long k = 0; k <= steps; ++k) {
    const double t = dt * static_cast<double>(k);
    const double phase = w * t;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(kDof, 0.0);
    pt.velocities.assign(kDof, 0.0);

    pt.positions[0] = params.j1_rad + params.j1_radius_rad * std::sin(phase);
    pt.velocities[0] = params.j1_radius_rad * w * std::cos(phase);
    pt.positions[1] = params.j2_rad;
    pt.positions[2] = params.j3_rad;
    // A circle in the (j4, j5) plane around the observation pose: the view
    // sweeps a cone and never crosses its centre.
    pt.positions[3] = params.j4_radius_rad * std::cos(phase);
    pt.velocities[3] = -params.j4_radius_rad * w * std::sin(phase);
    pt.positions[4] = params.j5_radius_rad * std::sin(phase);
    pt.velocities[4] = params.j5_radius_rad * w * std::cos(phase);
    pt.positions[5] = measured[5];

    set_duration(pt.time_from_start, t);
    traj.points.push_back(std::move(pt));
  }
  return traj;
}

trajectory_msgs::msg::JointTrajectory make_hold_trajectory(
  const std::array<double, 6> & measured,
  const std::array<std::string, 6> & names,
  double hold_s,
  const builtin_interfaces::msg::Time & start)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = start;
  traj.joint_names.assign(names.begin(), names.end());

  auto point_at = [&measured, hold_s](double t) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.assign(measured.begin(), measured.end());
      pt.velocities.assign(kDof, 0.0);
      set_duration(pt.time_from_start, t);
      return pt;
    };
  traj.points.push_back(point_at(0.0));
  traj.points.push_back(point_at(std::max(0.0, hold_s)));
  return traj;
}

}  // namespace lgw_bt_planner
