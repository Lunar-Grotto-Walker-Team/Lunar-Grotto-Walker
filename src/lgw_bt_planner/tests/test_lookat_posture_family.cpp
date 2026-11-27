// tests/test_lookat_posture_family.cpp
//
// Unit tests for the pinned-posture lookat family (plan 2026-10-31, spec
// 2026-10-30 rev2). Pure math: no rclcpp executor, no MoveIt runtime.
//
// The reference values come from scripts/lookat_posture_proto.py, which is
// the Python prototype the C++ implementation must match.

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <string>
#include <tuple>

#include "lgw_bt_planner/calibration_json.hpp"  // parse_calibration_json
#include "lgw_bt_planner/lookat_posture_family.hpp"

using lgw_bt_planner::PostureFamily;
using lgw_bt_planner::PostureParams;

namespace
{

// Tests build the chain to link6 and fold the tcp_offset (0, 0, 0.12) into the
// camera offset. The runtime node instead uses the launched `tcp_link`, which
// already carries that offset — same camera pose either way, but no xacro
// expansion is needed in a unit test.
std::string piperUrdf()
{
  return ament_index_cpp::get_package_share_directory("agx_arm_description") +
         "/agx_arm_urdf/piper/urdf/piper_description.urdf";
}

// arm_calibration.json is camera -> tcp_link. Tests stop the chain at link6,
// so the camera offset handed to the module is the calibration folded with
// the 0.12 m tcp offset.
std::pair<tf2::Matrix3x3, tf2::Vector3> camInLink6()
{
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = lgw_bt_planner::parse_calibration_json(
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/calibration/arm_calibration.json");
  return {R, t + tf2::Vector3(0.0, 0.0, 0.12)};
}

}  // namespace

TEST(PostureFamily, FkMatchesPrototype)
{
  PostureFamily fam;
  std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;

  const std::array<double, 6> q{-1.4064, 0.8898, -0.4841, 1.0463, 1.2338, -1.7653};
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();

  tf2::Vector3 p_cam;
  tf2::Matrix3x3 R_out;
  ASSERT_TRUE(fam.cameraPose(q, R_cam, t_cam, p_cam, R_out, err)) << err;

  // scripts/lookat_posture_proto.py measured on the same chain.
  EXPECT_NEAR(p_cam.x(), 0.162, 2e-3);
  EXPECT_NEAR(p_cam.y(), -0.099, 2e-3);
  EXPECT_NEAR(p_cam.z(), 0.303, 2e-3);
}

TEST(PostureFamily, AimsAtGroundTarget)
{
  // j3/j5 pinned, j2 = 0.96 (the prototype's solution); solve j1/j4/j6 only.
  PostureFamily fam;
  std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;

  PostureParams p;
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();

  // solveFree is profile-agnostic; use the fallback posture, which this floor
  // target was measured against (the primary -0.72/-0.80 cannot centre it).
  const tf2::Vector3 target(0.75, 0.0, -0.37);
  const std::array<double, 6> seed{
    0.0, 0.96, p.posture_j3_alt_rad, 0.0, p.posture_j5_alt_rad, 0.0};
  std::array<double, 6> q{};
  double aim = 0.0;
  double cond = 0.0;
  ASSERT_TRUE(fam.solveFree(target, R_cam, t_cam, seed, p, q, aim, cond, err)) << err;

  EXPECT_LT(aim, 0.02);                        // < 1.15 deg off the target
  EXPECT_EQ(q[2], p.posture_j3_alt_rad);       // j3 pinned
  EXPECT_EQ(q[4], p.posture_j5_alt_rad);       // j5 pinned
  EXPECT_LE(cond, p.cond_max);
}

// ---------------------------------------------------------------------------
// Task 3: j2 candidate search and guards via compute()
// ---------------------------------------------------------------------------

namespace
{

// A fresh family per solve would lose the warm start; share one, like the node.
PostureFamily & family()
{
  static PostureFamily fam = [] {
      PostureFamily f;
      std::string e;
      f.initialize(piperUrdf(), /*is_file=*/true, "base_link", "link6", e);
      return f;
    }();
  return fam;
}

lgw_bt_planner::PostureResult solveAt(const tf2::Vector3 & target)
{
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();
  return family().compute(target, R_cam, t_cam, PostureParams{});
}

}  // namespace

TEST(PostureFamily, GroundTargetsAreCovered)
{
  for (double d : {0.40, 0.75, 1.20}) {
    const auto r = solveAt(tf2::Vector3(d, 0.0, -0.37));
    EXPECT_TRUE(r.ok) << "d=" << d << " status=" << r.status;
    EXPECT_LT(r.aim_error_rad, 0.035) << "d=" << d;
    // Either profile is legal here (close/low targets need the fallback).
    const PostureParams def;
    EXPECT_TRUE(
      (r.joints[2] == def.posture_j3_rad && r.joints[4] == def.posture_j5_rad) ||
      (r.joints[2] == def.posture_j3_alt_rad &&
      r.joints[4] == def.posture_j5_alt_rad)) << "q=" << r.joints[2] << "," <<
      r.joints[4];
  }
}

TEST(PostureFamily, GuardsAlwaysHold)
{
  const PostureParams p;
  for (double az = -2.618; az <= 2.618; az += 0.10) {
    for (double d : {0.40, 0.75, 1.20, 2.0}) {
      const tf2::Vector3 t(d * std::cos(az), d * std::sin(az), -0.37);
      const auto r = solveAt(t);

      tf2::Matrix3x3 R_cam;
      tf2::Vector3 t_cam;
      std::tie(R_cam, t_cam) = camInLink6();
      tf2::Vector3 cam;
      tf2::Matrix3x3 R_out;
      std::string err;
      ASSERT_TRUE(family().cameraPose(r.joints, R_cam, t_cam, cam, R_out, err));

      EXPECT_GE(cam.z(), p.cam_min_height_m - 1e-6) << az << " " << d;
      EXPECT_LE(std::hypot(cam.x(), cam.y()), p.cam_max_radius_m + 1e-6);
      EXPECT_LE(r.cond, p.cond_max);
      EXPECT_GE(r.joints[3], -1.7453292 + p.joint_margin - 1e-6);
      EXPECT_LE(r.joints[3], 1.7453292 - p.joint_margin + 1e-6);
    }
  }
}

TEST(PostureFamily, WallTargetIsCoveredByPrimaryProfile)
{
  // 0.8 m ahead, 1.23 m above the floor (~47 deg up). The 2026-10-30 posture
  // could not centre it (aim 6.5 deg, cond 190); the -0.72/-0.80 primary
  // reaches it, so this is no longer an out-of-envelope case.
  const auto r = solveAt(tf2::Vector3(0.8, 0.0, 1.23));
  EXPECT_TRUE(r.ok) << "status=" << r.status << " aim=" << r.aim_error_rad;
  EXPECT_LT(r.aim_error_rad, PostureParams{}.aim_tol_rad);
}

TEST(PostureFamily, OutOfEnvelopeTargetIsStillFlagged)
{
  // Directly behind the arm: the optical axis would have to point backwards,
  // past the j1 limit. Must be flagged, not silently mis-aimed.
  const auto r = solveAt(tf2::Vector3(-0.75, 0.0, -0.37));
  EXPECT_TRUE(!r.ok || r.aim_error_rad > PostureParams{}.aim_tol_rad)
    << "status=" << r.status << " aim=" << r.aim_error_rad;
}

// ---------------------------------------------------------------------------
// Task 4: continuity, degeneracy, joint limits
// ---------------------------------------------------------------------------

TEST(PostureFamily, ContinuousAcrossBearing)
{
  // A target circling the robot at 0.75 m must not make the setpoint jump:
  // a jump here is what Servo turns into a large twist command.
  tf2::Vector3 prev_p;
  std::array<double, 6> prev_q{};
  bool first = true;
  // Coverage is asymmetric: the camera's optical axis sits ~78 deg off the
  // arm's +x, so j1 runs out first on the positive side (measured 2026-10-31:
  // ok up to +122 deg, still ok at -150 deg). Sweep the covered arc.
  int switches = 0;
  double worst_switch_jump = 0.0;
  // Warm-started solves run out at ~+119 deg (asymmetric: the negative side
  // still reaches -140). A FRESH family solves the same bearings out to
  // +-175 deg, so this is a solver-seed limitation (the previous pose seeds
  // j1/j4/j6 into a different basin), not a kinematic edge. Raising the 4 ms
  // solve budget to 8 ms does not help. The sweep stops at +115 deg.
  for (double az = -2.44; az <= 2.01; az += 0.0175) {   // -140..+115 deg
    const auto r = solveAt(
      tf2::Vector3(0.75 * std::cos(az), 0.75 * std::sin(az), -0.37));
    ASSERT_TRUE(r.ok) << "az=" << az << " status=" << r.status;
    if (!first) {
      const bool same_profile =
        r.joints[2] == prev_q[2] && r.joints[4] == prev_q[4];
      const double jump = (r.tcp_desired.p - prev_p).length();
      if (same_profile) {
        // With the j2 candidate grid the worst within-profile step is one
        // grid step (0.15 rad), which moves the tcp by ~28 mm. Tighter
        // continuity needs a finer posture_j2_step_rad (cost ~1/step).
        EXPECT_LT(jump, 0.035) << "az=" << az;
        EXPECT_LE(std::abs(r.joints[1] - prev_q[1]), 0.151) << "az=" << az;
      } else {
        // Two profiles: a boundary crossing is a deliberate wrist-branch
        // change. It must be rare (hysteresis keeps us on the current one)
        // and bounded, not a free jump.
        ++switches;
        worst_switch_jump = std::max(worst_switch_jump, jump);
      }
    }
    prev_p = r.tcp_desired.p;
    prev_q = r.joints;
    first = false;
  }
  EXPECT_LE(switches, 2);
  EXPECT_LT(worst_switch_jump, 0.30);
}

namespace
{

lgw_bt_planner::PostureResult solveFresh(const tf2::Vector3 & target)
{
  PostureFamily fresh;
  std::string err;
  fresh.initialize(piperUrdf(), /*is_file=*/true, "base_link", "link6", err);
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();
  return fresh.compute(target, R_cam, t_cam, PostureParams{});
}

}  // namespace

// 2026-11-05: a single -0.80 profile cannot centre targets closer than ~0.6 m
// at or below body height (measured aim 25.5 deg at 0.4 m / floor). The family
// must fall back to the +0.60 profile instead of returning out_of_envelope.
TEST(PostureFamily, AltProfileCoversCloseLowTargets)
{
  const auto r = solveFresh(tf2::Vector3(0.4, -0.2, -0.37));
  EXPECT_TRUE(r.ok) << "status=" << r.status;
  EXPECT_DOUBLE_EQ(r.joints[4], PostureParams{}.posture_j5_alt_rad);
}

// The fallback must not steal targets both profiles can solve: the primary
// has much better conditioning (measured cond 17 vs 26 at 0.75 m).
TEST(PostureFamily, PrimaryProfileWinsWhenBothSolve)
{
  const auto r = solveFresh(tf2::Vector3(0.75, -0.2, 0.0));
  EXPECT_TRUE(r.ok) << "status=" << r.status;
  EXPECT_DOUBLE_EQ(r.joints[4], PostureParams{}.posture_j5_rad);
}


TEST(PostureFamily, DegenerateTargetThrows)
{
  // Target on the base z axis: the look direction is undefined. Throwing is
  // the legacy contract (compute_lookat_setpoint) that the node maps to HOLD;
  // emitting a nearest-anything pose here would put a NaN in the control law.
  EXPECT_THROW(solveAt(tf2::Vector3(0.0, 0.0, -0.37)), std::runtime_error);
}

// Task 5: the node reads these from servo_node.yaml, so the struct defaults
// are the single source of truth for the spec's numbers. This is a drift
// guard, not a behaviour test — it fails if someone edits a default without
// meaning to.
TEST(PostureFamily, ParamsDefaultsMatchSpec)
{
  const PostureParams p;
  EXPECT_DOUBLE_EQ(p.posture_j3_rad, -0.720);
  EXPECT_DOUBLE_EQ(p.posture_j5_rad, -0.800);
  EXPECT_DOUBLE_EQ(p.posture_j3_alt_rad, -0.484);
  EXPECT_DOUBLE_EQ(p.posture_j5_alt_rad, 0.600);
  EXPECT_DOUBLE_EQ(p.posture_j2_step_rad, 0.150);
  EXPECT_DOUBLE_EQ(p.posture_j2_nominal_rad, 0.890);
  EXPECT_DOUBLE_EQ(p.posture_j2_min_rad, 0.150);
  EXPECT_DOUBLE_EQ(p.posture_j2_max_rad, 1.571);
  EXPECT_DOUBLE_EQ(p.cam_min_height_m, 0.200);
  EXPECT_DOUBLE_EQ(p.cam_max_radius_m, 0.400);
  EXPECT_DOUBLE_EQ(p.joint_margin, 0.150);
  EXPECT_DOUBLE_EQ(p.cond_max, 50.0);
}

// Final-review C1: with no legal candidate AND no previous pose the module
// used to return a default-constructed PostureResult, whose tf2 Pose6 members
// are deliberately uninitialised — a finite garbage pose straight into the
// control law. It must throw instead (the node turns that into HOLD).
TEST(PostureFamily, NoLegalCandidateThrows)
{
  PostureFamily fresh;
  std::string err;
  ASSERT_TRUE(fresh.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();

  PostureParams empty;
  empty.posture_j2_min_rad = 1.5;
  empty.posture_j2_max_rad = 0.2;      // candidate list is empty
  EXPECT_THROW(
    fresh.compute(tf2::Vector3(0.75, 0.0, -0.37), R_cam, t_cam, empty),
    std::runtime_error) << "returned a pose instead of throwing";
}

// Final-review I2: j2 may move at most 0.15 rad per tick. A one-way sweep
// never exposes a jump (previous-first is sticky); the return pass crosses
// the same candidate boundaries from the other side and does.
TEST(PostureFamily, J2NeverJumpsBetweenTicks)
{
  std::array<double, 6> prev{};
  bool first = true;
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i <= 200; ++i) {
      const double az = (pass == 0) ? -2.0 + 0.02 * i : 2.0 - 0.02 * i;
      const auto r = solveAt(
        tf2::Vector3(0.75 * std::cos(az), 0.75 * std::sin(az), -0.37));
      ASSERT_TRUE(r.ok) << "az=" << az;
      if (!first && r.joints[2] == prev[2] && r.joints[4] == prev[4]) {
        EXPECT_LE(std::abs(r.joints[1] - prev[1]), 0.15) << "az=" << az;
      }
      prev = r.joints;
      first = false;
    }
  }
}

// Final-review I1: a solve that exceeds its budget must return the previous
// pose flagged "overrun" instead of blocking the 100 Hz tick.
TEST(PostureFamily, TimeBudgetFallsBackToPrevious)
{
  PostureFamily fam;
  std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;
  tf2::Matrix3x3 R_cam;
  tf2::Vector3 t_cam;
  std::tie(R_cam, t_cam) = camInLink6();

  PostureParams p;
  const auto first = fam.compute(tf2::Vector3(0.75, 0.0, -0.37), R_cam, t_cam, p);
  ASSERT_TRUE(first.ok) << first.status;

  PostureParams tight = p;
  tight.max_solve_us = 0.0;            // budget exhausted before the first candidate
  const auto r = fam.compute(
    tf2::Vector3(0.95, 0.0, -0.37), R_cam, t_cam, tight);
  EXPECT_EQ(r.status, "overrun");
  EXPECT_NEAR((r.tcp_desired.p - first.tcp_desired.p).length(), 0.0, 1e-9);
}

TEST(PostureFamily, JointLimitsRespected)
{
  const PostureParams p;
  const double lo[6] = {
    -2.6179938, 0.0, -2.9670597, -1.7453292, -1.2217304, -2.0943951};
  const double hi[6] = {
    2.6179938, 3.1415926, 0.0, 1.7453292, 1.2217304, 2.0943951};
  for (double az = -2.44; az <= 2.44; az += 0.20) {
    for (double d : {0.40, 0.75, 1.20}) {
      const auto r = solveAt(
        tf2::Vector3(d * std::cos(az), d * std::sin(az), -0.37));
      for (int j = 0; j < 6; ++j) {
        EXPECT_GE(r.joints[j], lo[j] + p.joint_margin - 1e-6) << "j" << j + 1;
        EXPECT_LE(r.joints[j], hi[j] - p.joint_margin + 1e-6) << "j" << j + 1;
      }
    }
  }
}

// --- tcpTwist: joint velocities -> tcp twist in the base frame (2026-11-06) --
// The servo control law's damping term needs the REAL tcp velocity. The driver
// publishes 200 Hz joint velocities, so the mapping is J(q) * dq. Pinned
// against a finite difference of forward kinematics: an implementation that
// used the wrong reference link or dropped a joint would disagree here.
TEST(PostureFamily, TcpTwistMatchesFiniteDifferenceOfFk)
{
  PostureFamily fam;
  std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;

  const std::array<double, 6> q{-1.20, 0.75, -0.45, 0.90, 1.10, -1.40};
  const std::array<double, 6> dq{0.30, -0.20, 0.15, -0.25, 0.10, 0.20};

  tf2::Vector3 v;
  tf2::Vector3 w;
  ASSERT_TRUE(fam.tcpTwist(q, dq, v, w, err)) << err;

  // FK twice and compare: pose(q + h*dq) vs pose(q).
  const tf2::Matrix3x3 I(1, 0, 0, 0, 1, 0, 0, 0, 1);
  const tf2::Vector3 zero(0, 0, 0);
  constexpr double h = 1e-6;
  std::array<double, 6> q2{};
  for (int i = 0; i < 6; ++i) {
    q2[i] = q[i] + h * dq[i];
  }
  tf2::Vector3 p0;
  tf2::Vector3 p1;
  tf2::Matrix3x3 R0;
  tf2::Matrix3x3 R1;
  ASSERT_TRUE(fam.cameraPose(q, I, zero, p0, R0, err)) << err;
  ASSERT_TRUE(fam.cameraPose(q2, I, zero, p1, R1, err)) << err;

  const tf2::Vector3 v_fd = (p1 - p0) / h;
  const tf2::Matrix3x3 dR = R1 * R0.transpose();
  const tf2::Vector3 w_fd(
    (dR[2][1] - dR[1][2]) / (2.0 * h),
    (dR[0][2] - dR[2][0]) / (2.0 * h),
    (dR[1][0] - dR[0][1]) / (2.0 * h));

  EXPECT_NEAR(v.x(), v_fd.x(), 1e-3);
  EXPECT_NEAR(v.y(), v_fd.y(), 1e-3);
  EXPECT_NEAR(v.z(), v_fd.z(), 1e-3);
  EXPECT_NEAR(w.x(), w_fd.x(), 1e-3);
  EXPECT_NEAR(w.y(), w_fd.y(), 1e-3);
  EXPECT_NEAR(w.z(), w_fd.z(), 1e-3);
}
