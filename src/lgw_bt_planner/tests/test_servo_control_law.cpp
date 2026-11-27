// tests/test_servo_control_law.cpp
// Pure-function unit tests for compute_twist (Task 4 of the 2026-10-26
// servo-takeover plan, spec §7.6).
//
// No ROS runtime / executor / Servo / MoveIt needed: plain value types +
// tf2 LinearMath. Run the binary directly from build/lgw_bt_planner/
// (test_servo_control_law) — colcon test is not available in this
// container (see CLAUDE.md).
//
// Conventions pinned here:
//   * both poses are tcp_link poses in the arm base_link frame; there is no
//     link6 conversion and no 12cm offset in the control law;
//   * angular error is the rotation vector of R_des * R_cur^T (rotation
//     that takes current to desired);
//   * source_scale multiplies the CLAMPED output:
//     v = clamp(kp_pos * e_pos, v_max) * source_scale;
//   * the first-order low-pass runs on that scaled value, so the first
//     output is alpha * target with alpha = 1 - exp(-2*pi*fc/loop_rate);
//   * the deadband zeroes the output without touching the filter state.

#include "lgw_bt_planner/servo_control_law.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

lgw_bt_planner::Pose6 make_pose(const tf2::Vector3& p, const tf2::Matrix3x3& R)
{
  lgw_bt_planner::Pose6 pose;
  pose.p = p;
  pose.R = R;
  return pose;
}

tf2::Matrix3x3 identity_rotation()
{
  return tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

tf2::Matrix3x3 rotation_z(double angle)
{
  tf2::Matrix3x3 R;
  R.setRPY(0.0, 0.0, angle);
  return R;
}

tf2::Matrix3x3 rotation_x(double angle)
{
  tf2::Matrix3x3 R;
  R.setRPY(angle, 0.0, 0.0);
  return R;
}

// Same formula the implementation is required to use (spec §7.6).
double lowpass_alpha(const lgw_bt_planner::ControlLawParams& params)
{
  return 1.0 - std::exp(
    -2.0 * kPi * params.lowpass_cutoff_hz / params.loop_rate_hz);
}

}  // namespace

// A large error must saturate both commands at v_max / w_max, and the
// low-pass must converge to that saturated value (not stop short of it).
TEST(ControlLaw, SaturatesAtMax) {
  lgw_bt_planner::ControlLawParams params;  // v_max 0.15, w_max 0.6
  lgw_bt_planner::ControlLawState state;
  const auto current = make_pose(tf2::Vector3(0, 0, 0), identity_rotation());
  // 1 m away and 1 rad off about x: both raw proportional terms exceed
  // their clamps (2.0 > 0.15, 3.0 > 0.6).
  const auto desired = make_pose(tf2::Vector3(1.0, 0, 0), rotation_x(1.0));

  lgw_bt_planner::TwistCommand cmd;
  for (int i = 0; i < 2000; ++i) {
    cmd = lgw_bt_planner::compute_twist(desired, current, params, state, 1.0);
  }

  EXPECT_FALSE(cmd.in_deadband);
  EXPECT_NEAR(cmd.v.length(), params.v_max, 1e-9);
  EXPECT_NEAR(cmd.v.x(), params.v_max, 1e-9);   // pushes toward +x
  EXPECT_NEAR(cmd.w.length(), params.w_max, 1e-9);
  EXPECT_NEAR(cmd.w.x(), params.w_max, 1e-9);   // rotates about +x
}

// 1 degree / 5 mm is inside both deadbands: the output is exactly zero on
// this tick (so Servo may halt), but the filter memory must survive.
TEST(ControlLaw, DeadbandOutputsZero) {
  lgw_bt_planner::ControlLawParams params;
  lgw_bt_planner::ControlLawState state;
  // Pre-existing filter memory from earlier ticks.
  state.v_filtered = tf2::Vector3(0.05, 0, 0);
  state.w_filtered = tf2::Vector3(0, 0.1, 0);

  const auto current = make_pose(tf2::Vector3(0, 0, 0), identity_rotation());
  const auto desired = make_pose(
    tf2::Vector3(0.005, 0, 0), rotation_z(1.0 * kDegToRad));

  const auto cmd =
    lgw_bt_planner::compute_twist(desired, current, params, state, 1.0);

  EXPECT_TRUE(cmd.in_deadband);
  EXPECT_NEAR(cmd.v.length(), 0.0, 1e-15);
  EXPECT_NEAR(cmd.w.length(), 0.0, 1e-15);
  // Deadband must not reset (or otherwise move) the filter state.
  EXPECT_DOUBLE_EQ(state.v_filtered.x(), 0.05);
  EXPECT_DOUBLE_EQ(state.v_filtered.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.v_filtered.z(), 0.0);
  EXPECT_DOUBLE_EQ(state.w_filtered.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.w_filtered.y(), 0.1);
  EXPECT_DOUBLE_EQ(state.w_filtered.z(), 0.0);
}

// First tick starts from a zero filter state: output == alpha * target,
// and the second tick follows filtered += alpha * (target - filtered).
TEST(ControlLaw, LowpassFirstStep) {
  lgw_bt_planner::ControlLawParams params;  // fc 8 Hz, loop 100 Hz
  lgw_bt_planner::ControlLawState state;
  const auto current = make_pose(tf2::Vector3(0, 0, 0), identity_rotation());
  // 5 cm along x: outside the 1 cm deadband, target 0.1 m/s (unsaturated).
  const auto desired = make_pose(tf2::Vector3(0.05, 0, 0), identity_rotation());

  const double alpha = lowpass_alpha(params);
  EXPECT_NEAR(alpha, 0.3951, 1e-4);  // 1 - exp(-2*pi*8/100)
  const double target = params.kp_pos * 0.05;

  const auto first =
    lgw_bt_planner::compute_twist(desired, current, params, state, 1.0);
  EXPECT_NEAR(first.v.x(), alpha * target, 1e-12);
  EXPECT_NEAR(state.v_filtered.x(), alpha * target, 1e-12);
  EXPECT_NEAR(first.w.length(), 0.0, 1e-15);

  const auto second =
    lgw_bt_planner::compute_twist(desired, current, params, state, 1.0);
  const double expected_second = alpha * target + (1.0 - alpha) * (alpha * target);
  EXPECT_NEAR(second.v.x(), expected_second, 1e-12);
}

// source_scale multiplies the clamped output: with a saturated error the
// head tier (1.0) settles at v_max and the odom tier (0.5) at v_max/2 —
// i.e. scaling happens AFTER the clamp, not before it.
TEST(ControlLaw, SourceScaleApplied) {
  lgw_bt_planner::ControlLawParams params;
  const auto current = make_pose(tf2::Vector3(0, 0, 0), identity_rotation());

  // Saturated regime.
  const auto far = make_pose(tf2::Vector3(1.0, 0, 0), identity_rotation());
  lgw_bt_planner::ControlLawState head_state;
  lgw_bt_planner::ControlLawState odom_state;
  lgw_bt_planner::TwistCommand head;
  lgw_bt_planner::TwistCommand odom;
  for (int i = 0; i < 2000; ++i) {
    head = lgw_bt_planner::compute_twist(far, current, params, head_state, 1.0);
    odom = lgw_bt_planner::compute_twist(far, current, params, odom_state, 0.5);
  }
  EXPECT_NEAR(head.v.length(), params.v_max, 1e-9);
  EXPECT_NEAR(odom.v.length(), params.v_max * 0.5, 1e-9);
  EXPECT_NEAR(head.v.length(), 2.0 * odom.v.length(), 1e-9);

  // Unsaturated first tick halves as well (linear region).
  const auto near = make_pose(tf2::Vector3(0.03, 0, 0), identity_rotation());
  lgw_bt_planner::ControlLawState head_state_2;
  lgw_bt_planner::ControlLawState odom_state_2;
  const auto head_2 =
    lgw_bt_planner::compute_twist(near, current, params, head_state_2, 1.0);
  const auto odom_2 =
    lgw_bt_planner::compute_twist(near, current, params, odom_state_2, 0.5);
  EXPECT_NEAR(head_2.v.length(), 2.0 * odom_2.v.length(), 1e-12);
}

// A desired pose rotated +0.1 rad about z relative to current must produce
// a positive z angular command (R_err = R_des * R_cur^T).
TEST(ControlLaw, AngularSignFollowsError) {
  lgw_bt_planner::ControlLawParams params;
  lgw_bt_planner::ControlLawState state;
  const auto current = make_pose(
    tf2::Vector3(0.2, 0.1, 0.3), identity_rotation());
  // Same position, +0.1 rad (5.7 deg) about z: outside the 2 deg deadband,
  // raw angular term 0.3 rad/s so still below w_max.
  const auto desired = make_pose(
    tf2::Vector3(0.2, 0.1, 0.3), rotation_z(0.1));

  const auto cmd =
    lgw_bt_planner::compute_twist(desired, current, params, state, 1.0);

  EXPECT_FALSE(cmd.in_deadband);
  EXPECT_GT(cmd.w.z(), 0.0);
  EXPECT_NEAR(cmd.w.x(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.w.y(), 0.0, 1e-12);
  EXPECT_NEAR(cmd.w.z(), lowpass_alpha(params) * params.kp_ang * 0.1, 1e-9);
  // No positional error -> no translation command.
  EXPECT_NEAR(cmd.v.length(), 0.0, 1e-15);
}
