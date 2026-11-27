// tests/test_scan_roll_lock.cpp
// Image-roll estimation and the j6 roll lock used during SCANNING.
// Pure module: the forward kinematics arrives through a callback.

#include "lgw_bt_planner/servo/scan_roll_lock.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

builtin_interfaces::msg::Duration seconds(double s)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(s);
  d.nanosec = static_cast<uint32_t>((s - std::floor(s)) * 1e9);
  return d;
}

trajectory_msgs::msg::JointTrajectory make_traj(
  const std::vector<std::array<double, 6>> & samples)
{
  trajectory_msgs::msg::JointTrajectory traj;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(samples[i].begin(), samples[i].end());
    pt.velocities.assign(6, 0.0);
    pt.time_from_start = seconds(0.02 * static_cast<double>(i));
    traj.points.push_back(pt);
  }
  return traj;
}

// Camera looking along +y, image upright.
tf2::Matrix3x3 camera_looking_along_y()
{
  const double t = -kPi / 2.0;   // Rx(-90)
  return tf2::Matrix3x3(
    1, 0, 0,
    0, std::cos(t), -std::sin(t),
    0, std::sin(t), std::cos(t));
}

}  // namespace

TEST(ImageRoll, IsZeroWhenTheImageUpVectorIsWorldUp)
{
  EXPECT_NEAR(lgw_bt_planner::image_roll(camera_looking_along_y()), 0.0, 1e-9);
}

TEST(ImageRoll, FollowsARollAboutTheOpticalAxis)
{
  const double phi = 0.4;
  const tf2::Matrix3x3 roll(
    std::cos(phi), 0, std::sin(phi),
    0, 1, 0,
    -std::sin(phi), 0, std::cos(phi));
  const tf2::Matrix3x3 R = roll * camera_looking_along_y();
  EXPECT_NEAR(lgw_bt_planner::image_roll(R), -phi, 1e-9);
}

TEST(LockCameraRoll, DrivesJ6SoTheRollMatchesTheReference)
{
  // Fake kinematics: roll = 0.5*j4 + j6, so locking to 0 needs j6 = -0.5*j4.
  const auto roll_of = [](const std::array<double, 6> & q) {
      return 0.5 * q[3] + q[5];
    };
  std::vector<std::array<double, 6>> samples;
  for (double j4 : {-0.6, -0.2, 0.0, 0.3, 0.9}) {
    samples.push_back({0.0, 0.89, -0.72, j4, 0.1, 0.0});
  }
  auto traj = make_traj(samples);

  ASSERT_TRUE(lgw_bt_planner::lock_camera_roll(
      traj, roll_of, 0.0, -2.0, 2.0));

  ASSERT_EQ(traj.points.size(), samples.size());
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const std::array<double, 6> q{
      traj.points[i].positions[0], traj.points[i].positions[1],
      traj.points[i].positions[2], traj.points[i].positions[3],
      traj.points[i].positions[4], traj.points[i].positions[5]};
    EXPECT_NEAR(roll_of(q), 0.0, 1e-6)
      << "sample " << i << " is not roll-locked";
  }
}

TEST(LockCameraRoll, ConvergesForNonlinearRoll)
{
  // Real kinematics are not linear in j6; a single Newton step left ~10 deg
  // of error on the robot, so the solver must iterate.
  const auto roll_of = [](const std::array<double, 6> & q) {
      return q[5] + 0.8 * std::sin(q[5]) + 0.3 * q[3] + 0.1 * q[4] * q[4];
    };
  std::vector<std::array<double, 6>> samples;
  for (double j4 : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
    for (double j5 : {-1.0, 0.0, 1.0}) {
      samples.push_back({0.0, 0.89, -0.72, j4, j5, 0.0});
    }
  }
  auto traj = make_traj(samples);

  // Far from the starting j6, so one Newton step cannot land on target.
  const double target = 2.0;
  ASSERT_TRUE(lgw_bt_planner::lock_camera_roll(
      traj, roll_of, target, -2.0, 2.0));

  for (std::size_t i = 0; i < samples.size(); ++i) {
    std::array<double, 6> q{};
    for (std::size_t j = 0; j < 6; ++j) {
      q[j] = traj.points[i].positions[j];
    }
    EXPECT_NEAR(roll_of(q), target, 2e-3)
      << "sample " << i << " did not converge";
  }
}

TEST(LockCameraRoll, ClampsJ6ToItsLimits)
{
  const auto roll_of = [](const std::array<double, 6> & q) {
      return q[5];
    };
  std::vector<std::array<double, 6>> samples;
  samples.push_back({0.0, 0.89, -0.72, 0.0, 0.0, 0.0});
  auto traj = make_traj(samples);

  // Reference roll 2 rad is unreachable with j6 limited to +-0.5.
  ASSERT_TRUE(lgw_bt_planner::lock_camera_roll(
      traj, roll_of, 2.0, -0.5, 0.5));
  EXPECT_NEAR(traj.points[0].positions[5], 0.5, 1e-9);
}

TEST(LockCameraRoll, LeavesAnEmptyTrajectoryAlone)
{
  trajectory_msgs::msg::JointTrajectory traj;
  EXPECT_FALSE(lgw_bt_planner::lock_camera_roll(
      traj, [](const std::array<double, 6> &) {return 0.0;}, 0.0, -1.0, 1.0));
}
