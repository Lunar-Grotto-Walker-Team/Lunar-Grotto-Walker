// src/servo/scan_roll_lock.cpp — see the header for the contract.

#include "lgw_bt_planner/servo/scan_roll_lock.hpp"

#include <algorithm>
#include <cmath>

namespace lgw_bt_planner
{

namespace
{

constexpr double kEps = 1e-6;

// Wrap into (-pi, pi].
double wrap_pi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a <= -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double seconds_of(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

}  // namespace

double image_roll(const tf2::Matrix3x3 & R_cam_in_base)
{
  const tf2::Vector3 z_cam = R_cam_in_base.getColumn(2);      // optical axis
  const tf2::Vector3 image_up = -R_cam_in_base.getColumn(1);  // camera -y
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);

  // Project the world's up direction into the image plane.
  tf2::Vector3 u = world_up - z_cam * world_up.dot(z_cam);
  if (u.length() < kEps) {
    // Camera looking straight up/down: the horizon is undefined, fall back
    // to the base x axis so the estimate stays continuous.
    u = tf2::Vector3(1.0, 0.0, 0.0);
    u = u - z_cam * u.dot(z_cam);
    if (u.length() < kEps) {
      return 0.0;
    }
  }
  u.normalize();

  const tf2::Vector3 cross = image_up.cross(u);
  return std::atan2(cross.dot(z_cam), image_up.dot(u));
}

bool lock_camera_roll(
  trajectory_msgs::msg::JointTrajectory & traj,
  const JointRollFn & roll_of,
  double roll_ref,
  double j6_min, double j6_max)
{
  if (traj.points.empty() || !roll_of) {
    return false;
  }

  const double step = 0.05;   // rad used to measure d(roll)/d(j6) locally
  double previous = traj.points.front().positions[5];

  for (auto & pt : traj.points) {
    std::array<double, 6> q{};
    for (std::size_t i = 0; i < 6; ++i) {
      q[i] = pt.positions[i];
    }

    // Start from the previous sample so the branch (and the wrist) stays
    // continuous, then Newton-iterate: the roll is visibly non-linear in j6.
    double j6 = std::min(std::max(previous, j6_min), j6_max);
    q[5] = j6;
    double r = roll_of(q);
    for (int iteration = 0; iteration < 8; ++iteration) {
      const double error = wrap_pi(roll_ref - r);
      if (std::abs(error) < 1e-3) {
        break;
      }
      q[5] = j6 + step;
      const double r_step = roll_of(q);
      const double slope = wrap_pi(r_step - r) / step;
      if (std::abs(slope) < 1e-3) {
        break;  // j6 does not move the roll here; leave the best effort
      }
      j6 = std::min(std::max(j6 + error / slope, j6_min), j6_max);
      q[5] = j6;
      r = roll_of(q);
    }

    pt.positions[5] = j6;
    previous = j6;
  }

  // j6 velocity by central differences over the sample times.
  const std::size_t n = traj.points.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t a = (i == 0) ? 0 : i - 1;
    const std::size_t b = (i + 1 < n) ? i + 1 : n - 1;
    const double dt = seconds_of(traj.points[b].time_from_start) -
      seconds_of(traj.points[a].time_from_start);
    traj.points[i].velocities[5] = (dt > 1e-9) ?
      (traj.points[b].positions[5] - traj.points[a].positions[5]) / dt : 0.0;
  }
  return true;
}

}  // namespace lgw_bt_planner
