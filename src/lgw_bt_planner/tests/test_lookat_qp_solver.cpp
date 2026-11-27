// tests/test_lookat_qp_solver.cpp
//
// QP lookat solver (plan 2026-11-05, Task 3).

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <tuple>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/calibration_json.hpp"
#include "lgw_bt_planner/lookat_qp_solver.hpp"
#include "lgw_bt_planner/piper_limits.hpp"

using lgw_bt_planner::PostureParams;
using lgw_bt_planner::QpPostureSolver;
using lgw_bt_planner::QpSolverParams;

namespace
{

std::string piperUrdf()
{
  return ament_index_cpp::get_package_share_directory("agx_arm_description") +
         "/agx_arm_urdf/piper/urdf/piper_description.urdf";
}

std::pair<tf2::Matrix3x3, tf2::Vector3> camInLink6()
{
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = lgw_bt_planner::parse_calibration_json(
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/calibration/arm_calibration.json");
  return {R, t + tf2::Vector3(0.0, 0.0, 0.12)};
}

// The solver answers an absolute pose per call; `ticks` only retries a call
// that came back out_of_envelope (it is a per-call retry, not a motion cap).
lgw_bt_planner::PostureResult solveTicks(
  QpPostureSolver & qp, const tf2::Vector3 & target,
  const tf2::Matrix3x3 & R, const tf2::Vector3 & t,
  const lgw_bt_planner::PostureParams & p, int ticks)
{
  lgw_bt_planner::PostureResult r;
  for (int i = 0; i < ticks; ++i) {
    try {
      r = qp.solve(target, R, t, p);
    } catch (const std::exception &) {
      // Same contract as the DLS family: no legal pose and no history throws.
      lgw_bt_planner::PostureResult thrown;
      thrown.ok = false;
      thrown.status = "degenerate";
      return thrown;
    }
    if (r.ok) {
      return r;
    }
  }
  return r;
}

void initSolver(QpPostureSolver & qp)
{
  std::string err;
  ASSERT_TRUE(qp.initialize(piperUrdf(), /*is_file=*/true, "base_link",
    "link6", err)) << err;
}

}  // namespace

TEST(QpSolver, AimsAtGroundTargetFirstSolve)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const auto r = qp.solve(tf2::Vector3(0.75, 0.0, -0.37), R, t, p);
  EXPECT_TRUE(r.ok) << "status=" << r.status << " aim=" << r.aim_error_rad
                    << " cond=" << r.cond;
  EXPECT_LT(r.aim_error_rad, p.aim_tol_rad);
  EXPECT_LE(r.cond, p.cond_max);
}

TEST(QpSolver, RespectsJointMarginsAndStep)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  // Two targets 0.6 m apart force the step limit to matter on the second call.
  const auto r1 = solveTicks(qp, tf2::Vector3(0.75, -0.2, 0.0), R, t, p, 10);
  ASSERT_TRUE(r1.ok) << r1.status;
  // One call is one SQP run (8 linearisations x 0.2 rad trust region). The
  // runtime calls again 10 ms later from the new iterate, so allow a few
  // calls here; there is no per-tick motion budget.
  lgw_bt_planner::PostureResult r2;
  for (int call = 0; call < 5; ++call) {
    r2 = qp.solve(tf2::Vector3(0.55, 0.35, 0.0), R, t, p);
    if (r2.ok) {
      break;
    }
  }
  ASSERT_TRUE(r2.ok) << r2.status;
  for (int i = 0; i < 6; ++i) {
    EXPECT_GE(r2.joints[i], lgw_bt_planner::kPiperLower[i] + p.joint_margin - 1e-9);
    EXPECT_LE(r2.joints[i], lgw_bt_planner::kPiperUpper[i] - p.joint_margin + 1e-9);
  }
}

TEST(QpSolver, SideTargetConvergesOverTicks)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double az = 150.0 * 3.14159265358979323846 / 180.0;
  const tf2::Vector3 target(0.75 * std::cos(az), 0.75 * std::sin(az), -0.37);
  double first_error = -1.0;
  double last_error = -1.0;
  bool ok = false;
  for (int i = 0; i < 20; ++i) {
    const auto r = qp.solve(target, R, t, p);
    if (i == 0) {
      first_error = r.aim_error_rad;
    }
    last_error = r.aim_error_rad;
    if (r.ok) {
      ok = true;
      break;
    }
  }
  EXPECT_TRUE(ok) << "first=" << first_error << " last=" << last_error;
}

TEST(QpSolver, NeverExitsTheCameraEnvelope)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  int checked = 0;
  for (double az : {-0.8, -0.3, 0.0, 0.4, 1.2}) {
    for (double d : {0.35, 0.55, 0.75, 1.0}) {
      for (double h : {0.1, -0.2, -0.37}) {
        const tf2::Vector3 target(
          d * std::cos(az), d * std::sin(az), h);
        const auto r = solveTicks(qp, target, R, t, p, 5);
        if (!r.ok) {
          continue;
        }
        ++checked;
        tf2::Vector3 cam;
        tf2::Matrix3x3 Rc;
        std::string err;
        ASSERT_TRUE(qp.cameraPose(r.joints, R, t, cam, Rc, err)) << err;
        EXPECT_GE(cam.z(), p.cam_min_height_m - 1e-6) << "az=" << az << " d=" << d;
        EXPECT_LE(std::hypot(cam.x(), cam.y()), p.cam_max_radius_m + 1e-6);
        EXPECT_LE(r.cond, p.cond_max);
      }
    }
  }
  // The assertion that matters is the zero-violation checks above; the count
  // only proves the loop exercised real solves. Coverage itself is measured
  // by the Task 6 A/B, not here.
  EXPECT_GT(checked, 10);
}

TEST(QpSolver, DegenerateTargetThrows)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  EXPECT_THROW(qp.solve(tf2::Vector3(0.0, 0.0, -0.37), R, t, PostureParams{}),
    std::runtime_error);
}

TEST(QpSolver, FailureReturnsOnlyGuardedPoses)
{
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  // Near the base axis and above it: no legal lookat posture.
  const auto front = qp.solve(tf2::Vector3(0.75, 0.0, 0.0), R, t, p);
  ASSERT_TRUE(front.ok) << front.status;
  const auto r = qp.solve(tf2::Vector3(0.10, 0.0, 0.30), R, t, p);
  ASSERT_FALSE(r.ok) << "status=" << r.status;
  if (r.status == "out_of_envelope") {
    tf2::Vector3 cam;
    tf2::Matrix3x3 Rc;
    std::string err;
    ASSERT_TRUE(qp.cameraPose(r.joints, R, t, cam, Rc, err)) << err;
    EXPECT_GE(cam.z(), p.cam_min_height_m - 1e-6);
    EXPECT_LE(std::hypot(cam.x(), cam.y()), p.cam_max_radius_m + 1e-6);
    EXPECT_LE(r.cond, p.cond_max);
  } else {
    EXPECT_EQ(r.status, "held_last");
    EXPECT_NEAR((r.tcp_desired.p - front.tcp_desired.p).length(), 0.0, 1e-9);
  }
}

TEST(QpSolver, WarmStartAdvancesAfterFailure)
{
  // Finding: a failed warm-started call used to keep the OLD seed, so every
  // following tick recomputed the same failure. After the fix the iterate
  // must advance even when no pose is legal yet. (Convergence quality for
  // side/rear targets is a separate, documented limitation.)
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const auto front = qp.solve(tf2::Vector3(0.75, 0.0, -0.2), R, t, p);
  ASSERT_TRUE(front.ok) << front.status;

  const double az = 120.0 * 3.14159265358979323846 / 180.0;
  const tf2::Vector3 side(0.75 * std::cos(az), 0.75 * std::sin(az), -0.2);
  const auto a = qp.solve(side, R, t, p);
  const auto b = qp.solve(side, R, t, p);
  double max_delta = 0.0;
  for (int i = 0; i < 6; ++i) {
    max_delta = std::max(max_delta, std::abs(a.joints[i] - b.joints[i]));
  }
  EXPECT_GT(max_delta, 1e-6)
    << "the second tick must start from the first tick's iterate";
}

TEST(QpSolver, NeverReportsAnAntiAlignedPose)
{
  // Found 2026-11-05: e_axis = R_z x u is zero at aim = 0 AND at aim = pi,
  // so the solver could converge to looking 180 deg away and report it as
  // the best legal pose. Measured before the fix: aim = 3.139 rad.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  for (double d : {0.35, 0.50}) {
    for (double h : {-0.20, -0.37}) {
      for (double azdeg : {0.0, 30.0}) {
        const double az = azdeg * 3.14159265358979323846 / 180.0;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        const auto r = solveTicks(qp, target, R, t, p, 5);
        // Without this the test can pass vacuously: solveTicks returns an
        // all-zero joint vector on an exception, and at the zero pose this
        // grid's aim is 0.61-0.93 rad, already under the 1.0 bound.
        ASSERT_TRUE(r.ok) << "d=" << d << " h=" << h << " az=" << azdeg
                          << " status=" << r.status;
        tf2::Vector3 cam;
        tf2::Matrix3x3 Rc;
        std::string err;
        ASSERT_TRUE(qp.cameraPose(r.joints, R, t, cam, Rc, err)) << err;
        const tf2::Vector3 u = (target - cam).normalized();
        const double aim = std::acos(std::min(1.0,
          std::max(-1.0, Rc.getColumn(2).dot(u))));
        EXPECT_LT(aim, 1.0) << "d=" << d << " h=" << h << " az=" << azdeg;
      }
    }
  }
}

TEST(QpSolver, InfeasibleSolveNeverReportsAPose)
{
  // An impossible envelope makes every iterate envelope-illegal. The cache
  // rule refuses to store one, so a solver that never produced a usable
  // iterate keeps throwing instead of reporting a held pose it did not
  // compute. (qpmad itself rejects the infeasible QP, so this pins the
  // "never invent a pose" contract end to end rather than the cache line in
  // isolation; see the ledger for why the isolated case is unreachable.)
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  PostureParams p;
  p.cam_min_height_m = 5.0;   // unreachable for every pose
  EXPECT_THROW(qp.solve(tf2::Vector3(0.75, 0.0, -0.2), R, t, p),
               std::runtime_error);
  EXPECT_THROW(qp.solve(tf2::Vector3(0.75, 0.0, -0.2), R, t, p),
               std::runtime_error);
}

TEST(QpSolver, CloseLowTargetsAreCovered)
{
  // The whole region the DLS covers only through its alt profile
  // (j3=-0.484, j5=+0.60): spec 10.2.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  for (double d : {0.35, 0.50, 0.60}) {
    for (double h : {-0.20, -0.37}) {
      for (double azdeg : {-60.0, 0.0, 60.0}) {
        const double az = azdeg * 3.14159265358979323846 / 180.0;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        initSolver(qp);   // isolate each target: intrinsic coverage
        const auto r = solveTicks(qp, target, R, t, p, 5);
        ASSERT_TRUE(r.ok) << "d=" << d << " h=" << h << " az=" << azdeg
                          << " status=" << r.status
                          << " aim=" << r.aim_error_rad
                          << " cond=" << r.cond;
        // A close target may need two calls to reach the strict tier's
        // cond <= 50 pose; the relaxed tier answers in one (cond 54-57 on
        // this grid), which is a deceleration pose, not a violation.
        EXPECT_LE(r.cond, 70.0 + 1e-9);
        EXPECT_LE(r.aim_error_rad, p.aim_tol_rad);
      }
    }
  }
}

TEST(QpSolver, WristBranchIsRespected)
{
  // Every reported pose (ok or failure) must sit in the branch the solver
  // declared, otherwise the HOLD diagnostic shows a singular pose.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  QpSolverParams q;
  q.j5_branch_min_rad = 0.60;
  qp.set_params(q);
  for (const tf2::Vector3 target :
    {tf2::Vector3(0.75, 0.0, -0.2), tf2::Vector3(0.50, 0.0, -0.37),
     tf2::Vector3(0.35, 0.0, -0.20)})
  {
    const auto r = solveTicks(qp, target, R, t, p, 5);
    EXPECT_GE(std::abs(r.joints[4]), 0.60 - 1e-6) << "status=" << r.status;
  }
}

TEST(QpSolver, TwoBranchSolveStaysInBudget)
{
  // Two branch runs double the SQP work. The node's solve budget is 4 ms and
  // the control tick is 10 ms, so the mean has to stay well under 1 ms.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  double total_ms = 0.0;
  double worst_ms = 0.0;
  const int n = 200;
  for (int i = 0; i < n; ++i) {
    const double az = (i % 37 - 18) * 10.0 * 3.14159265358979323846 / 180.0;
    const tf2::Vector3 target(0.75 * std::cos(az), 0.75 * std::sin(az), -0.2);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = qp.solve(target, R, t, p);
    (void)r;
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    total_ms += ms;
    worst_ms = std::max(worst_ms, ms);
  }
  EXPECT_LT(total_ms / n, 1.0);
  EXPECT_LT(worst_ms, 8.0);
}

TEST(QpSolver, BranchHysteresis)
{
  // Far target (negative branch) -> close target (positive branch) -> far
  // again. Each step must converge within five calls and stay legal.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  for (const tf2::Vector3 target :
    {tf2::Vector3(0.75, 0.0, -0.2), tf2::Vector3(0.45, 0.0, -0.37),
     tf2::Vector3(0.75, 0.0, -0.2)})
  {
    const auto r = solveTicks(qp, target, R, t, p, 5);
    ASSERT_TRUE(r.ok) << "status=" << r.status << " aim=" << r.aim_error_rad;
    EXPECT_LE(r.cond, p.cond_max);
  }
}

TEST(QpSolver, BoundaryTargetsUseTheRelaxedTier)
{
  // Rear bearings the DLS cannot reach at all. Their only solutions sit at
  // cond 57-68; the strict tier must fail and the relaxed tier must solve.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double kDeg = 3.14159265358979323846 / 180.0;
  // 0.75 m, not 0.35 m: with the QP's 0.20 joint margin (added after a real
  // Servo joint-limit halt), the closest rear bearings have no solution left
  // -- their only IK sits at |j1| ~ 2.47, inside Servo's halt slack.
  for (double d : {0.75}) {
    for (double h : {-0.20, -0.37}) {
      for (double azdeg : {-155.0, 155.0}) {
        const double az = azdeg * kDeg;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        initSolver(qp);   // isolate each target: intrinsic coverage
        const auto r = solveTicks(qp, target, R, t, p, 5);
        ASSERT_TRUE(r.ok) << "d=" << d << " h=" << h << " az=" << azdeg
                          << " status=" << r.status << " cond=" << r.cond;
        EXPECT_LE(r.cond, 70.0 + 1e-9);
      }
    }
  }
}

TEST(QpSolver, StrictTierKeepsTheGoodPoses)
{
  // Tiering must not change what already worked: every reachable target in
  // the mission range still comes from the strict tier, so cond <= 50.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double kDeg = 3.14159265358979323846 / 180.0;
  for (double d : {0.50, 0.75, 1.00, 1.30, 1.60}) {
    for (double h : {0.15, 0.0, -0.20, -0.37}) {
      for (double azdeg : {-60.0, 0.0, 60.0}) {
        const double az = azdeg * kDeg;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        initSolver(qp);   // isolate each target
        const auto r = solveTicks(qp, target, R, t, p, 5);
        ASSERT_TRUE(r.ok) << "d=" << d << " h=" << h << " az=" << azdeg;
        EXPECT_LE(r.cond, p.cond_max + 1e-9)
          << "fell through to the relaxed tier";
      }
    }
  }
}

TEST(QpSolver, BranchDoesNotFlipAcrossTicks)
{
  // d=0.60/h=-0.20/az=140 is solvable on BOTH branches. The hysteresis used
  // to compare `previous[4] > j5_branch_min_rad`, but an accepted positive
  // pose sits at exactly +0.600, so the next tick fell back to the negative
  // branch and the solver alternated every tick (16.5 cm tcp steps).
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double az = 140.0 * 3.14159265358979323846 / 180.0;
  const tf2::Vector3 target(0.60 * std::cos(az), 0.60 * std::sin(az), -0.20);
  double last_sign = 0.0;
  int switches = 0;
  int large_steps = 0;
  tf2::Vector3 prev;
  for (int i = 0; i < 10; ++i) {
    const auto r = qp.solve(target, R, t, p);
    ASSERT_TRUE(r.ok) << "tick " << i << " status=" << r.status;
    const double sign = r.joints[4] > 0.0 ? 1.0 : -1.0;
    if (last_sign != 0.0 && sign != last_sign) {
      ++switches;
    }
    last_sign = sign;
    if (i > 0 && (r.tcp_desired.p - prev).length() > 0.05) {
      ++large_steps;
    }
    prev = r.tcp_desired.p;
  }
  EXPECT_LE(switches, 1) << "the wrist branch must be sticky across ticks";
  // One acquisition switch is legitimate; a repeating 16 cm step is not.
  EXPECT_LE(large_steps, 1) << "the tcp must settle after the first switch";
}

TEST(QpSolver, RelaxedTierIsRearOnly)
{
  // The relaxed tier exists for rear bearings whose only IK solution sits at
  // cond 57-68. It must not fire for a target in the mission range: doing so
  // drops the pose into Servo's deceleration band on the very first call.
  QpPostureSolver qp;
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double kDeg = 3.14159265358979323846 / 180.0;
  for (double d : {0.35, 0.50}) {
    for (double h : {-0.20, -0.37}) {
      for (double azdeg : {-60.0, 0.0, 60.0}) {
        const double az = azdeg * kDeg;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        initSolver(qp);
        const auto r = solveTicks(qp, target, R, t, p, 5);
        ASSERT_TRUE(r.ok) << "d=" << d << " h=" << h << " az=" << azdeg
                          << " status=" << r.status;
        EXPECT_LE(r.cond, p.cond_max + 1e-9)
          << "mission-range target used the relaxed tier (d=" << d
          << " h=" << h << " az=" << azdeg << ")";
      }
    }
  }
}

TEST(QpSolver, KeepsClearOfTheServoJointLimitMargin)
{
  // Real-machine finding 2026-11-06: Servo halts at URDF_limit - 0.10, and a
  // QP setpoint at |j5| = 1.072 (our old 0.15 margin) left only 0.05 rad for
  // the tracking lag, so Servo halted every time the arm approached a limit.
  // The QP keeps its own 0.20 margin; assert at least 0.18 on every joint.
  QpPostureSolver qp;
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const double kDeg = 3.14159265358979323846 / 180.0;
  const double kClearance = 0.18;
  for (double d : {0.35, 0.50, 0.75, 1.00, 1.60}) {
    for (double h : {-0.37, -0.20, 0.15}) {
      for (double azdeg : {-140.0, -60.0, 0.0, 60.0, 140.0}) {
        const double az = azdeg * kDeg;
        const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
        initSolver(qp);
        const auto r = solveTicks(qp, target, R, t, p, 5);
        if (!r.ok) {
          continue;
        }
        for (int i = 0; i < 6; ++i) {
          EXPECT_GE(r.joints[i], lgw_bt_planner::kPiperLower[i] + kClearance - 1e-9)
            << "joint" << i + 1 << " d=" << d << " h=" << h << " az=" << azdeg;
          EXPECT_LE(r.joints[i], lgw_bt_planner::kPiperUpper[i] - kClearance + 1e-9)
            << "joint" << i + 1 << " d=" << d << " h=" << h << " az=" << azdeg;
        }
      }
    }
  }
}

TEST(QpSolver, ReportsWristBranchChanges)
{
  // A far target solves on the negative branch, a close one needs the
  // positive branch. Crossing j5 = 0 is where Servo hard-stops, so the
  // solver must tell the node to use the planned-move path.
  QpPostureSolver qp;
  initSolver(qp);
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();
  const PostureParams p;
  const auto far = qp.solve(tf2::Vector3(0.75, 0.0, -0.20), R, t, p);
  ASSERT_TRUE(far.ok) << far.status;
  EXPECT_FALSE(far.branch_switched) << "first solve has no history";
  const auto near = qp.solve(tf2::Vector3(0.35, 0.0, -0.37), R, t, p);
  ASSERT_TRUE(near.ok) << near.status;
  EXPECT_TRUE(near.branch_switched)
    << "far j5=" << far.joints[4] << " near j5=" << near.joints[4];
}
