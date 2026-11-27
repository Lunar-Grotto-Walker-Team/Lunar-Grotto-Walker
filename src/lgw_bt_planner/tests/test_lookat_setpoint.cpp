// tests/test_lookat_setpoint.cpp
// Pure-function unit tests for compute_lookat_setpoint (Task 2 of the
// 2026-10-26 servo-takeover plan).
//
// No ROS runtime / executor needed: math only (tf2 LinearMath).
// Run the binary directly from build/lgw_bt_planner/
// (test_lookat_setpoint) — colcon test is not available in this container
// (see CLAUDE.md).

#include "lgw_bt_planner/lookat_setpoint.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <stdexcept>

namespace
{
// Column 2 of R is the camera optical +z axis (forward) expressed in the
// frame R maps to.
tf2::Vector3 optical_z(const tf2::Matrix3x3& R)
{
  return tf2::Vector3(R[0][2], R[1][2], R[2][2]);
}

// Column 1 of R is the camera optical +y axis.
tf2::Vector3 optical_y(const tf2::Matrix3x3& R)
{
  return tf2::Vector3(R[0][1], R[1][1], R[2][1]);
}

void expectMatrixNear(const tf2::Matrix3x3& a, const tf2::Matrix3x3& b,
                      double tol)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(a[i][j], b[i][j], tol) << "R[" << i << "][" << j << "]";
    }
  }
}
}  // namespace

TEST(LookatSetpoint, CameraOpticalAxisPointsAtTarget) {
  lgw_bt_planner::LookatParams p;  // 0.4 / 0.3
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.5, 0.0, 0.3}, R, {0, 0, 0}, p);
  tf2::Vector3 cam_z = optical_z(r.tcp_desired.R);
  EXPECT_NEAR(cam_z.dot(tf2::Vector3(1, 0, 0)), 1.0, 1e-6);
}

TEST(LookatSetpoint, TcpPoseIsTheSetpointItself) {
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.8, 0.0, 0.3}, R, {0, 0, 0}, p);
  // 无标定偏置时,期望位姿就是 standoff 点本身;任何额外 12cm 偏移都会打破这
  // 三条断言(tcp_link 是 Servo 的雅可比求值点,不能"换算"到 link6)。
  EXPECT_NEAR(r.tcp_desired.p.x(), 0.4, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.y(), 0.0, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.z(), 0.3, 1e-9);
}

TEST(LookatSetpoint, DegenerateTargetFallsBackToAnchor) {
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.0, 0.0, 1.0}, R, {0, 0, 0}, p);
  EXPECT_TRUE(r.used_anchor);
}

// Extra: the orbit radius / fixed height must come from LookatParams, not
// from the target's own z (the brief's pos test happens to use
// target.z == z_fixed_m, so it cannot catch a mixed-up z_fixed_m).
TEST(LookatSetpoint, OrbitPositionUsesConfiguredRadiusAndFixedZ) {
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.6, 0.8, 1.0}, R, {0, 0, 0}, p);
  EXPECT_FALSE(r.used_anchor);
  EXPECT_NEAR(r.tcp_desired.p.x(), 0.24, 1e-9);  // 0.4 * 0.6/1.0
  EXPECT_NEAR(r.tcp_desired.p.y(), 0.32, 1e-9);  // 0.4 * 0.8/1.0
  EXPECT_NEAR(r.tcp_desired.p.z(), 0.3, 1e-9);   // z_fixed_m, not target z
}

// Extra: a non-zero camera-in-tcp translation must shift the *tcp* setpoint
// back along the optical axis (this is compute_tcp_pose's job, exercised
// through the new module).
TEST(LookatSetpoint, TcpSetpointAccountsForCameraOffsetInTcp) {
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  // Camera sits 10 cm in front of tcp along the optical axis: the tcp
  // setpoint backs off from the 0.4 m standoff point to x = 0.3 (the
  // standoff point itself is the camera origin, not tcp_link).
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.5, 0.0, 0.3}, R, {0.0, 0.0, 0.1}, p);
  EXPECT_NEAR(r.tcp_desired.p.x(), 0.3, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.y(), 0.0, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.z(), 0.3, 1e-9);
}

// Extra: target exactly on the chosen orbit point leaves the look
// direction undefined. The module must signal that (throw) rather than
// emitting a NaN rotation to Servo.
TEST(LookatSetpoint, CoincidentTargetThrows) {
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  EXPECT_THROW(
    lgw_bt_planner::compute_lookat_setpoint({0.4, 0.0, 0.3}, R, {0, 0, 0}, p),
    std::runtime_error);
}

// === Uprightness flip ================================================
// NOTE on what the flip can and cannot do with world_up = +z:
// build_lookat_orientation builds cam_y = d × (d × up) = d·(d·up) − up,
// so cam_y · up = −sqrt(1 − dz²) < 0 for every NON-degenerate direction —
// the raw basis is already upright there and the flip is a no-op. The
// flip rule (`cam_y · up >= 0`) only fires on the degenerate ±z axis,
// where it is a 180° roll about the optical axis (the dot stays 0).
// We therefore test the *roll parity* with the BT path, not a sign flip.

TEST(LookatSetpoint, DegenerateDirectionGetsUprightnessFlip) {
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  // Anchor path (target on the base z axis) with the anchor at the base
  // origin: P_ee = (0,0,0), so direction = +z EXACTLY (no orbit / division
  // rounding), which puts the raw basis exactly on the flip boundary
  // (cam_y · up == 0) and makes the flip decision deterministic.
  lgw_bt_planner::LookatParams p;
  p.ee_anchor = tf2::Vector3(0.0, 0.0, 0.0);
  const tf2::Vector3 p_target(0.0, 0.0, 1.0);
  const tf2::Vector3 direction(0.0, 0.0, 1.0);

  // Precondition: the raw basis sits on the flip boundary (>= 0).
  const tf2::Matrix3x3 R_raw =
    lgw_bt_planner::build_lookat_orientation(direction, world_up);
  ASSERT_GE(optical_y(R_raw).dot(world_up), 0.0);

  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(p_target, R, {0, 0, 0}, p);
  ASSERT_TRUE(r.used_anchor);

  // Expected = raw basis rolled 180° about cam_z (cam_x / cam_y negated):
  // identical to what ComputeLookatPoseNode / ComputeGraspPose command.
  tf2::Matrix3x3 R_expected = R_raw;
  for (int i = 0; i < 3; ++i) {
    R_expected[i][0] = -R_expected[i][0];
    R_expected[i][1] = -R_expected[i][1];
  }
  expectMatrixNear(r.tcp_desired.R, R_expected, 1e-12);
  EXPECT_NEAR(r.tcp_desired.R.determinant(), 1.0, 1e-12);
  // The optical axis must still point along the direction.
  EXPECT_NEAR(optical_z(r.tcp_desired.R).dot(direction), 1.0, 1e-12);
}

TEST(LookatSetpoint, UprightDirectionIsNotFlipped) {
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.5, 0.0, 0.3}, R, {0, 0, 0}, p);

  // Identity calibration -> tcp R == arm R; an already-upright basis must
  // pass through the flip untouched.
  const tf2::Matrix3x3 R_expected =
    lgw_bt_planner::build_lookat_orientation(
      tf2::Vector3(1.0, 0.0, 0.0), world_up);
  expectMatrixNear(r.tcp_desired.R, R_expected, 1e-12);
  EXPECT_LT(optical_y(r.tcp_desired.R).dot(world_up), 0.0);
}

// === Camera roll =====================================================
// The roll exists to move the wrist joint off its singularity without
// pointing away from the target. Two properties matter and are tested here:
// the optical axis must NOT move (target stays centred), and the in-plane
// axes must actually rotate by the requested angle.

TEST(LookatSetpoint, CameraRollKeepsOpticalAxisOnTarget) {
  lgw_bt_planner::LookatParams p;
  p.xoy_distance_m = 0.35;
  p.z_fixed_m = 0.22;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  const tf2::Vector3 target(0.75, 0.0, -0.30);

  p.camera_roll_rad = 0.0;
  const auto upright = lgw_bt_planner::compute_lookat_setpoint(
    target, R, {0, 0, 0}, p);
  const tf2::Vector3 z_upright = optical_z(upright.tcp_desired.R);

  for (double roll : {-0.5236, -0.2, 0.2, 0.5236}) {
    p.camera_roll_rad = roll;
    const auto rolled = lgw_bt_planner::compute_lookat_setpoint(
      target, R, {0, 0, 0}, p);
    // Optical axis unchanged: the target is still exactly centred.
    EXPECT_NEAR(optical_z(rolled.tcp_desired.R).dot(z_upright), 1.0, 1e-12)
      << "roll=" << roll;
    // Still a proper rotation.
    EXPECT_NEAR(rolled.tcp_desired.R.determinant(), 1.0, 1e-12);
    // cam_y must actually turn by the requested angle (dot = cos(roll)).
    const tf2::Vector3 y_upright =
      tf2::Vector3(upright.tcp_desired.R[0][1], upright.tcp_desired.R[1][1],
        upright.tcp_desired.R[2][1]);
    const tf2::Vector3 y_rolled =
      tf2::Vector3(rolled.tcp_desired.R[0][1], rolled.tcp_desired.R[1][1],
        rolled.tcp_desired.R[2][1]);
    EXPECT_NEAR(y_upright.dot(y_rolled), std::cos(roll), 1e-12)
      << "roll=" << roll;
  }
}

TEST(LookatSetpoint, ZeroRollMatchesUprightRule) {
  // Default params must reproduce the pre-roll behaviour bit for bit: the
  // BT lookat path and every existing caller keep their orientation.
  lgw_bt_planner::LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  const auto r = lgw_bt_planner::compute_lookat_setpoint(
    {0.75, 0.0, -0.30}, R, {0.1, 0.0, 0.0}, p);
  // P_ee = (0.4, 0, 0.3) for target (0.75, 0, -0.30), so the look
  // direction is (0.35, 0, -0.6).
  const tf2::Matrix3x3 R_expected = lgw_bt_planner::apply_uprightness_flip(
    lgw_bt_planner::build_lookat_orientation(
      tf2::Vector3(0.35, 0.0, -0.6).normalized(), tf2::Vector3(0, 0, 1)),
    tf2::Vector3(0, 0, 1));
  expectMatrixNear(r.tcp_desired.R, R_expected, 1e-12);
}

TEST(UprightnessFlip, AlreadyUprightReturnedUnchanged) {
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  const tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(
    tf2::Vector3(1.0, 0.0, 0.0), world_up);
  ASSERT_LT(optical_y(R).dot(world_up), 0.0);
  expectMatrixNear(
    lgw_bt_planner::apply_uprightness_flip(R, world_up), R, 1e-12);
}

TEST(UprightnessFlip, Flips180AboutOpticalAxis) {
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  const tf2::Matrix3x3 R_up = lgw_bt_planner::build_lookat_orientation(
    tf2::Vector3(1.0, 0.0, 0.0), world_up);

  // Roll it 180° about the optical axis: cam_x / cam_y negated.
  tf2::Matrix3x3 R_down = R_up;
  for (int i = 0; i < 3; ++i) {
    R_down[i][0] = -R_down[i][0];
    R_down[i][1] = -R_down[i][1];
  }
  ASSERT_GE(optical_y(R_down).dot(world_up), 0.0);

  const tf2::Matrix3x3 R_fixed =
    lgw_bt_planner::apply_uprightness_flip(R_down, world_up);
  expectMatrixNear(R_fixed, R_up, 1e-12);
  EXPECT_NEAR(R_fixed.determinant(), 1.0, 1e-12);
}
