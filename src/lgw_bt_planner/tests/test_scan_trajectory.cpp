// tests/test_scan_trajectory.cpp
// Circular scan generator (2026-11-15 rev5).  Pure module: no ROS node, no
// clock.  The tests pin: j1/j2/j3/j6 are immobile, j4/j5 trace a closed
// circle around the observation pose (never through its centre), the figure
// is periodic so trajectories chain, and the velocity limit holds.

#include "lgw_bt_planner/servo/scan_trajectory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace
{

const std::array<double, 6> kMeasured{0.10, 0.20, 0.30, 0.40, -0.30, 0.60};

lgw_bt_planner::ScanCircleParams small_params()
{
  lgw_bt_planner::ScanCircleParams p;
  p.j4_radius_rad = 1.0;
  p.j5_radius_rad = 0.5;
  p.period_s = 2.0;
  p.duration_s = 6.0;    // exactly three circles
  p.sample_rate_hz = 50.0;
  p.max_velocity_rad_s = 10.0;   // nominal figure; stretch has its own test
  return p;
}

double seconds_of(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

}  // namespace

TEST(ScanCircle, HoldsJ1J2J3AndJ6)
{
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, small_params(), builtin_interfaces::msg::Time());
  ASSERT_GT(traj.points.size(), 10u);
  for (const auto & p : traj.points) {
    EXPECT_NEAR(p.positions[0], 0.349, 1e-9) << "j1 is held at 20 deg";
    EXPECT_NEAR(p.positions[1], 0.890, 1e-9);
    EXPECT_NEAR(p.positions[2], -0.720, 1e-9);
    EXPECT_NEAR(p.positions[5], kMeasured[5], 1e-9) << "j6 is the roll lock's";
    EXPECT_NEAR(p.velocities[0], 0.0, 1e-9);
    EXPECT_NEAR(p.velocities[1], 0.0, 1e-9);
    EXPECT_NEAR(p.velocities[2], 0.0, 1e-9);
  }
}

TEST(ScanCircle, TracesACircleAroundTheObservationPose)
{
  const auto p = small_params();
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  ASSERT_FALSE(traj.points.empty());
  // Every sample sits on the ellipse (j4/A4)^2 + (j5/A5)^2 = 1 ...
  for (const auto & pt : traj.points) {
    const double u = pt.positions[3] / p.j4_radius_rad;
    const double v = pt.positions[4] / p.j5_radius_rad;
    EXPECT_NEAR(u * u + v * v, 1.0, 1e-6);
  }
  // ... and starts at (A4, 0), a quarter period later at (0, A5).
  EXPECT_NEAR(traj.points.front().positions[3], p.j4_radius_rad, 1e-9);
  EXPECT_NEAR(traj.points.front().positions[4], 0.0, 1e-9);
  const std::size_t quarter = static_cast<std::size_t>(
    std::llround(0.25 * p.period_s * p.sample_rate_hz));
  ASSERT_LT(quarter, traj.points.size());
  EXPECT_NEAR(traj.points[quarter].positions[3], 0.0, 1e-2);
  EXPECT_NEAR(traj.points[quarter].positions[4], p.j5_radius_rad, 1e-2);
}

TEST(ScanCircle, NeverPassesThroughTheCentre)
{
  const auto p = small_params();
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  for (const auto & pt : traj.points) {
    const double r = std::hypot(pt.positions[3], pt.positions[4]);
    EXPECT_GT(r, 0.4 * std::min(p.j4_radius_rad, p.j5_radius_rad))
      << "the circle must not collapse onto the centre";
  }
}

TEST(ScanCircle, IsPeriodicSoTrajectoriesChainSeamlessly)
{
  const auto p = small_params();
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  const std::size_t period_samples =
    static_cast<std::size_t>(std::llround(p.period_s * p.sample_rate_hz));
  ASSERT_GT(traj.points.size(), period_samples + 10u);
  for (std::size_t i = 0; i + period_samples < traj.points.size(); ++i) {
    for (std::size_t j : {3u, 4u}) {
      EXPECT_NEAR(
        traj.points[i + period_samples].positions[j],
        traj.points[i].positions[j], 1e-6)
        << "joint " << j << " sample " << i << " is not periodic";
    }
  }
}

TEST(ScanCircle, NeverExceedsTheVelocityLimit)
{
  const auto p = small_params();
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  for (const auto & pt : traj.points) {
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_LE(std::abs(pt.velocities[j]), p.max_velocity_rad_s + 1e-6);
    }
  }
}

TEST(ScanCircle, SlowsTheCircleWhenItWouldExceedTheLimit)
{
  auto p = small_params();
  p.max_velocity_rad_s = 1.0;   // the nominal circle peaks at pi rad/s
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  double peak = 0.0;
  for (const auto & pt : traj.points) {
    peak = std::max(peak, std::abs(pt.velocities[3]));
    peak = std::max(peak, std::abs(pt.velocities[4]));
  }
  EXPECT_LE(peak, p.max_velocity_rad_s + 1e-6);
  EXPECT_GT(seconds_of(traj.points.back().time_from_start), p.duration_s - 1e-6)
    << "an impossible period stretches the circle instead of speeding it up";
}

TEST(ScanCircle, SpansWholePeriodsAndTheRequestedDuration)
{
  const auto p = small_params();
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time());
  EXPECT_NEAR(seconds_of(traj.points.front().time_from_start), 0.0, 1e-6);
  EXPECT_NEAR(
    seconds_of(traj.points.back().time_from_start), p.duration_s,
    1.0 / p.sample_rate_hz);
  EXPECT_NEAR(
    static_cast<double>(traj.points.size()),
    p.duration_s * p.sample_rate_hz + 1.0, 2.0);
}

TEST(ScanCircle, RejectsDegenerateParameters)
{
  auto p = small_params();
  p.period_s = 0.0;
  EXPECT_TRUE(lgw_bt_planner::make_scan_circle(
    kMeasured, p, builtin_interfaces::msg::Time()).points.empty());

  auto q = small_params();
  q.j4_radius_rad = 0.0;
  q.j5_radius_rad = 0.0;
  EXPECT_TRUE(lgw_bt_planner::make_scan_circle(
    kMeasured, q, builtin_interfaces::msg::Time()).points.empty());

  auto r = small_params();
  r.duration_s = 0.0;
  EXPECT_TRUE(lgw_bt_planner::make_scan_circle(
    kMeasured, r, builtin_interfaces::msg::Time()).points.empty());
}

TEST(ScanCircle, NamesEveryJoint)
{
  const auto traj = lgw_bt_planner::make_scan_circle(
    kMeasured, small_params(), builtin_interfaces::msg::Time());
  ASSERT_EQ(traj.joint_names.size(), 6u);
  EXPECT_EQ(traj.joint_names[0], "joint1");
  EXPECT_EQ(traj.joint_names[5], "joint6");
}

TEST(HoldTrajectory, HoldsTheMeasuredPoseForTheRequestedTime)
{
  const std::array<double, 6> q{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const std::array<std::string, 6> names{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  builtin_interfaces::msg::Time start;
  const auto traj = lgw_bt_planner::make_hold_trajectory(
    q, names, 0.20, start);
  ASSERT_GE(traj.points.size(), 2u);
  EXPECT_NEAR(seconds_of(traj.points.back().time_from_start), 0.20, 1e-6);
  for (const auto & p : traj.points) {
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_NEAR(p.positions[j], q[j], 1e-9);
      EXPECT_NEAR(p.velocities[j], 0.0, 1e-9);
    }
  }
}
