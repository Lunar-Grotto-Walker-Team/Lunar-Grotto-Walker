// tests/test_lookat_standoff_setpoint.cpp
// Pure-function unit tests for compute_standoff_setpoint (target-relative
// lookat standoff, spec 2026-10-29).
//
// No ROS runtime / executor needed: math only (tf2 LinearMath).
// Run the binary directly from build/lgw_bt_planner/
// (test_lookat_standoff_setpoint) — colcon test is not available in this
// container (see CLAUDE.md).

#include "lgw_bt_planner/lookat_standoff_setpoint.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>

namespace
{

const tf2::Matrix3x3 kIdentity = tf2::Matrix3x3::getIdentity();
const tf2::Vector3 kNoOffset(0.0, 0.0, 0.0);

lgw_bt_planner::StandoffParams make_params()
{
  lgw_bt_planner::StandoffParams p;
  p.reach_pref_m = 0.45;
  p.reach_min_m = 0.15;
  p.reach_max_m = 0.75;
  p.standoff_min_m = 0.25;
  p.standoff_max_m = 0.90;
  p.cam_min_height_m = 0.22;
  p.standoff_rate_limit_mps = 0.30;
  return p;
}

tf2::Vector3 optical_z(const tf2::Matrix3x3& R)
{
  return tf2::Vector3(R[0][2], R[1][2], R[2][2]);
}

// A camera position already at a sensible station, so the first tick starts
// from a realistic bearing rather than an arbitrary one.
tf2::Vector3 default_camera_now()
{
  return tf2::Vector3(0.38, 0.0, 0.24);
}

}  // namespace

// With identity calibration and no camera offset the tcp position IS the
// camera origin, which makes the geometry assertions below direct.
TEST(StandoffSetpoint, FeasibleTargetHonoursEveryConstraint) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 target(0.75, 0.0, -0.30);

  const auto r = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state);

  EXPECT_EQ(r.status, lgw_bt_planner::StandoffStatus::kOk)
    << lgw_bt_planner::standoff_status_name(r.status);
  const tf2::Vector3 p_cam = r.tcp_desired.p;
  const double standoff = (p_cam - target).length();
  EXPECT_GE(standoff, params.standoff_min_m - 1e-9);
  EXPECT_LE(standoff, params.standoff_max_m + 1e-9);
  EXPECT_NEAR(r.achieved_standoff_m, standoff, 1e-12);
  EXPECT_GE(r.reach_m, params.reach_min_m - 1e-9);
  EXPECT_LE(r.reach_m, params.reach_max_m + 1e-9);
  EXPECT_GE(p_cam.z(), params.cam_min_height_m - 1e-9);
}

// The whole point of the redesign: aiming is independent of where the camera
// sits, so constraints can never rotate the view off the target.
TEST(StandoffSetpoint, OpticalAxisStaysExactlyOnTarget) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 target(0.75, 0.0, -0.30);

  const auto r = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state);
  ASSERT_EQ(r.status, lgw_bt_planner::StandoffStatus::kOk);

  const tf2::Vector3 dir = (target - r.tcp_desired.p).normalized();
  EXPECT_NEAR(optical_z(r.tcp_desired.R).dot(dir), 1.0, 1e-12);
}

TEST(StandoffSetpoint, RollDoesNotMoveTheOpticalAxis) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state_a;
  lgw_bt_planner::StandoffState state_b;
  const tf2::Vector3 target(0.75, 0.0, -0.30);

  const auto plain = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state_a);

  params.camera_roll_rad = 0.5235988;  // +30 deg
  const auto rolled = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state_b);

  ASSERT_EQ(plain.status, lgw_bt_planner::StandoffStatus::kOk);
  ASSERT_EQ(rolled.status, lgw_bt_planner::StandoffStatus::kOk);
  EXPECT_NEAR(
    optical_z(rolled.tcp_desired.R).dot(optical_z(plain.tcp_desired.R)),
    1.0, 1e-12);
  // Same station: only the roll changed.
  EXPECT_NEAR(
    (rolled.tcp_desired.p - plain.tcp_desired.p).length(), 0.0, 1e-12);
}

// The floor must be met by raising the VIEW ELEVATION, not by clamping z
// afterwards (a z clamp would silently break the standoff and the reach).
TEST(StandoffSetpoint, HeightFloorIsHonouredByRaisingTheElevation) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 target(0.75, 0.0, -0.30);
  // Camera currently low and level with the target: the natural bearing would
  // put the standoff point under the floor.
  const tf2::Vector3 low_camera(0.40, 0.0, 0.05);

  const auto r = lgw_bt_planner::compute_standoff_setpoint(
    target, low_camera, 0.01, kIdentity, kNoOffset, params, state);

  ASSERT_TRUE(
    r.status == lgw_bt_planner::StandoffStatus::kOk ||
    r.status == lgw_bt_planner::StandoffStatus::kReachClamped)
    << lgw_bt_planner::standoff_status_name(r.status);
  EXPECT_GE(r.tcp_desired.p.z(), params.cam_min_height_m - 1e-9);
  // The standoff is still honoured within the band even after the tilt.
  EXPECT_GE(r.achieved_standoff_m, params.standoff_min_m - 1e-9);
  EXPECT_LE(r.achieved_standoff_m, params.standoff_max_m + 1e-9);
  EXPECT_LE(r.reach_m, params.reach_max_m + 1e-9);
}

// When the target moves beyond what the shell allows, the camera must HOLD its
// last feasible station and keep aiming at the CURRENT target — that is the
// agreed degradation (no HOLD state, the line of sight is never dropped).
TEST(StandoffSetpoint, HoldsLastFeasibleStationAndKeepsAiming) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 near_target(0.75, 0.0, -0.30);

  const auto first = lgw_bt_planner::compute_standoff_setpoint(
    near_target, default_camera_now(), 0.01, kIdentity, kNoOffset, params,
    state);
  ASSERT_EQ(first.status, lgw_bt_planner::StandoffStatus::kOk);
  const tf2::Vector3 held = first.tcp_desired.p;

  // Target runs away far beyond reach_max + standoff_max.
  const tf2::Vector3 far_target(3.0, 0.0, -0.30);
  const auto second = lgw_bt_planner::compute_standoff_setpoint(
    far_target, default_camera_now(), 0.01, kIdentity, kNoOffset, params,
    state);

  EXPECT_EQ(second.status, lgw_bt_planner::StandoffStatus::kHeldLast);
  EXPECT_NEAR((second.tcp_desired.p - held).length(), 0.0, 1e-12);
  // Still aimed at the *current* target from the held station.
  const tf2::Vector3 dir = (far_target - held).normalized();
  EXPECT_NEAR(optical_z(second.tcp_desired.R).dot(dir), 1.0, 1e-12);
}

// The slew limit is best-effort by design: feasibility beats smoothness. What
// must NEVER happen is a limited standoff that leaves the shell or dips under
// the floor, so the emitted station is checked rather than the limit itself.
TEST(StandoffSetpoint, RateLimitedTickStaysFeasible) {
  auto params = make_params();
  params.standoff_rate_limit_mps = 0.01;  // 0.1 mm per tick: limit will bind
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 target(0.75, 0.0, -0.30);

  const auto first = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state);
  ASSERT_TRUE(
    first.status == lgw_bt_planner::StandoffStatus::kOk ||
    first.status == lgw_bt_planner::StandoffStatus::kReachClamped);

  // Pretend the previous tick used a much longer standoff.
  state.last_standoff_m = params.standoff_max_m;

  const auto second = lgw_bt_planner::compute_standoff_setpoint(
    target, default_camera_now(), 0.01, kIdentity, kNoOffset, params, state);

  ASSERT_LE(second.reach_m, params.reach_max_m + 1e-9);
  EXPECT_GE(second.tcp_desired.p.z(), params.cam_min_height_m - 1e-9);
  EXPECT_GE(second.achieved_standoff_m, params.standoff_min_m - 1e-9);
}

// A target coincident with the camera has no bearing of its own; the
// configured fallback view is used and a finite pose is still produced.
TEST(StandoffSetpoint, DegenerateBearingUsesFallbackView) {
  auto params = make_params();
  lgw_bt_planner::StandoffState state;
  const tf2::Vector3 target(0.30, 0.0, 0.20);

  const auto r = lgw_bt_planner::compute_standoff_setpoint(
    target, target, 0.01, kIdentity, kNoOffset, params, state);

  EXPECT_TRUE(r.used_fallback_view);
  EXPECT_TRUE(
    r.status == lgw_bt_planner::StandoffStatus::kOk ||
    r.status == lgw_bt_planner::StandoffStatus::kReachClamped ||
    r.status == lgw_bt_planner::StandoffStatus::kHeldLast);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(r.tcp_desired.p[i]));
    for (int j = 0; j < 3; ++j) {
      EXPECT_TRUE(std::isfinite(r.tcp_desired.R[i][j]));
    }
  }
}

// Deep targets: the height floor forces a longer standoff
// (d >= cam_min_height - p_target.z), so this is where standoff_max_m earns
// its keep. A target on the floor with the dog standing is ~0.3-0.5 m below
// the arm base, which is exactly this regime.
TEST(StandoffSetpoint, DeepTargetsStayFeasibleWithAWideStandoffBand) {
  auto params = make_params();
  int feasible = 0;
  int total = 0;
  for (double depth = -0.20; depth >= -0.60; depth -= 0.05) {
    for (double dist = 0.55; dist <= 1.20; dist += 0.05) {
      lgw_bt_planner::StandoffState state;
      const tf2::Vector3 target(dist, 0.0, depth);
      const auto r = lgw_bt_planner::compute_standoff_setpoint(
        target, default_camera_now(), 0.0, kIdentity, kNoOffset, params,
        state);
      ++total;
      if (r.status == lgw_bt_planner::StandoffStatus::kHeldLast) {
        continue;
      }
      ++feasible;
      EXPECT_GE(r.tcp_desired.p.z(), params.cam_min_height_m - 1e-9)
        << "dist=" << dist << " depth=" << depth;
      EXPECT_LE(r.reach_m, params.reach_max_m + 1e-9)
        << "dist=" << dist << " depth=" << depth;
      EXPECT_LE(r.achieved_standoff_m, params.standoff_max_m + 1e-9)
        << "dist=" << dist << " depth=" << depth;
    }
  }
  // The corners of this grid are genuinely outside the workspace (a shallow
  // target that is close needs a short standoff the height floor forbids; a
  // deep target that is far needs a vertical view the reach forbids), so this
  // only asserts that the bulk of it is usable.
  EXPECT_GT(feasible, total / 2)
    << "feasible=" << feasible << "/" << total;
}

// The realistic operating band — target on the floor with the dog standing
// (~0.30 m below the arm base) — must be feasible end to end. This is the
// case the legacy xoy/z_fixed scheme could not serve at all.
TEST(StandoffSetpoint, StandingFloorTargetBandIsFullyFeasible) {
  auto params = make_params();
  for (double dist = 0.75; dist <= 1.05 + 1e-9; dist += 0.05) {
    lgw_bt_planner::StandoffState state;
    const tf2::Vector3 target(dist, 0.0, -0.30);
    const auto r = lgw_bt_planner::compute_standoff_setpoint(
      target, default_camera_now(), 0.0, kIdentity, kNoOffset, params, state);
    EXPECT_NE(r.status, lgw_bt_planner::StandoffStatus::kHeldLast)
      << "dist=" << dist;
    EXPECT_GE(r.tcp_desired.p.z(), params.cam_min_height_m - 1e-9)
      << "dist=" << dist;
  }
}

// Across the whole working range the standoff must stay in its band and the
// reach inside the shell, whenever the tick is feasible at all.
TEST(StandoffSetpoint, ConstraintsHoldAcrossTargetDistances) {
  auto params = make_params();
  for (double dist = 0.50; dist <= 1.35; dist += 0.05) {
    lgw_bt_planner::StandoffState state;
    const tf2::Vector3 target(dist, 0.0, -0.30);
    const auto r = lgw_bt_planner::compute_standoff_setpoint(
      target, default_camera_now(), 0.0, kIdentity, kNoOffset, params, state);
    if (r.status == lgw_bt_planner::StandoffStatus::kHeldLast) {
      continue;  // out of shell: the degradation path, covered elsewhere
    }
    EXPECT_LE(r.reach_m, params.reach_max_m + 1e-9) << "dist=" << dist;
    EXPECT_GE(r.reach_m, params.reach_min_m - 1e-9) << "dist=" << dist;
    EXPECT_GE(r.achieved_standoff_m, params.standoff_min_m - 1e-9)
      << "dist=" << dist;
    EXPECT_LE(r.achieved_standoff_m, params.standoff_max_m + 1e-9)
      << "dist=" << dist;
    EXPECT_GE(r.tcp_desired.p.z(), params.cam_min_height_m - 1e-9)
      << "dist=" << dist;
  }
}
