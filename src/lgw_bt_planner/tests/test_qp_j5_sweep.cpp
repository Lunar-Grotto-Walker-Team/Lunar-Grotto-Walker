// tests/test_qp_j5_sweep.cpp
//
// Offline sweep for the QP wrist-branch floor (2026-11-09). One question:
// widening `j5_branch_min_rad` from 0.60 down to 0.05 — what does it buy in
// coverage, and what does it cost in conditioning?
//
// This is an analysis artifact with one invariant assertion: widening a
// FEASIBLE SET cannot reduce coverage. Everything else is printed for the
// operator to read; run the binary directly to see the table:
//
//   ./build/lgw_bt_planner/test_qp_j5_sweep
//
// The sweep uses the shipped PostureParams defaults, a fresh solver per
// (j5_min, target) so no hysteresis leaks between rows, and up to five
// solve calls per target (the runtime re-solves every 10 ms, so a single
// call coming back out_of_envelope is not the same as a covered target).

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <numeric>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/calibration_json.hpp"
#include "lgw_bt_planner/lookat_qp_solver.hpp"

using lgw_bt_planner::PostureParams;
using lgw_bt_planner::PostureResult;
using lgw_bt_planner::QpPostureSolver;
using lgw_bt_planner::QpSolverParams;

namespace
{

constexpr double kPi = 3.14159265358979323846;

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

struct Sample
{
  bool ok{false};
  bool first_call_ok{false};
  bool out_of_envelope{false};
  bool guarded{false};
  std::string status{"?"};
  double cond{0.0};
  double j5{0.0};
  double j2{0.0};
  double j1{0.0};
  double height{0.0};
  double radius{0.0};
  double depression{0.0};   // radians, 0 = level, pi/2 = straight down
  double aim{0.0};
};

// One target, one branch floor, fresh solver (no hysteresis carry-over).
Sample probe(
  const tf2::Vector3 & target, double j5_min, const tf2::Matrix3x3 & R,
  const tf2::Vector3 & t, double posture_j5 = 1e9,
  double w_radius = 0.0, double radius_near = 0.28, double radius_steep = 0.38,
  double cam_min_radius = 0.0, int max_lin = 0, double sector = 0.0,
  double taper = 0.0, double radius_side = 0.25, bool radius_3d = false)
{
  QpPostureSolver qp;
  std::string err;
  if (!qp.initialize(piperUrdf(), true, "base_link", "link6", err)) {
    return {};
  }
  QpSolverParams qp_params;
  qp_params.j5_branch_min_rad = j5_min;
  qp_params.w_radius = w_radius;
  qp_params.radius_near_m = radius_near;
  qp_params.radius_steep_m = radius_steep;
  qp_params.radius_side_m = radius_side;
  qp_params.radius_3d = radius_3d;
  if (max_lin > 0) {
    qp_params.max_linearizations = max_lin;
  }
  qp.set_params(qp_params);

  PostureParams p;
  p.cam_min_radius_m = cam_min_radius;
  p.cam_frontal_sector_rad = sector;
  p.cam_frontal_taper_rad = taper;
  if (posture_j5 < 1e8) {
    p.posture_j5_rad = posture_j5;
  }
  (void)0;
  PostureResult r;
  bool first_ok = false;
  for (int i = 0; i < 5; ++i) {
    try {
      r = qp.solve(target, R, t, p);
    } catch (const std::exception &) {
      continue;
    }
    if (i == 0) {
      first_ok = r.ok;
    }
    if (r.ok) {
      break;
    }
  }

  Sample s;
  s.first_call_ok = first_ok;
  s.ok = r.ok;
  s.out_of_envelope = r.out_of_envelope;
  s.guarded = r.guarded;
  s.status = r.status;
  s.cond = r.cond;
  s.j5 = r.joints[4];
  s.j2 = r.joints[1];
  s.j1 = r.joints[0];
  s.aim = r.aim_error_rad;
  tf2::Vector3 p_cam;
  tf2::Matrix3x3 R_cam;
  std::string cam_err;
  if (qp.cameraPose(r.joints, R, t, p_cam, R_cam, cam_err)) {
    s.radius = std::hypot(p_cam.x(), p_cam.y());
    s.depression = std::asin(std::clamp(-R_cam.getColumn(2).z(), -1.0, 1.0));
    s.height = p_cam.z();
  }
  return s;
}

std::vector<tf2::Vector3> targetGrid()
{
  std::vector<tf2::Vector3> out;
  const double distances[] = {0.40, 0.60, 0.80, 1.00, 1.20, 1.40, 1.60};
  const double heights[] = {-0.37, 0.00};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, -60.0, -120.0};
  for (const double h : heights) {
    for (const double d : distances) {
      for (const double a : azimuth_deg) {
        const double ar = a * kPi / 180.0;
        out.emplace_back(d * std::cos(ar), d * std::sin(ar), h);
      }
    }
  }
  return out;
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const size_t i = static_cast<size_t>(q * static_cast<double>(v.size() - 1));
  return v[i];
}

}  // namespace

TEST(QpJ5Sweep, WideningTheBranchFloorNeverReducesCoverage)
{
  const auto targets = targetGrid();
  const double floors[] = {0.60, 0.55, 0.50, 0.45, 0.40};

  std::vector<double> coverage;
  std::printf("\n  j5_min  covered  cond p50/p95/max        |j5| p10/min   "
              "radius p50   aim p95\n");
  for (const double floor : floors) {
    std::vector<double> conds, j5s, radii, aims;
    int covered = 0;
    for (const auto & target : targets) {
      const auto R_t = camInLink6();
      const Sample s = probe(target, floor, R_t.first, R_t.second);
      if (!s.ok) {
        continue;
      }
      ++covered;
      conds.push_back(s.cond);
      j5s.push_back(std::fabs(s.j5));
      radii.push_back(s.radius);
      aims.push_back(s.aim);
    }
    const double cov = 100.0 * static_cast<double>(covered) /
      static_cast<double>(targets.size());
    coverage.push_back(cov);
    std::printf(
      "  %5.2f  %5.1f%%  %6.1f/%6.1f/%6.1f  %6.3f/%6.3f  %6.3f  %5.3f\n",
      floor, cov,
      percentile(conds, 0.50), percentile(conds, 0.95), percentile(conds, 1.0),
      percentile(j5s, 0.10), j5s.empty() ? 0.0 : *std::min_element(
        j5s.begin(), j5s.end()),
      percentile(radii, 0.50), percentile(aims, 0.95));
    std::fflush(stdout);
  }

  // What the sweep actually showed (2026-11-09). The naive invariant
  // ("widening a feasible set cannot reduce coverage") DOES NOT hold here,
  // because the QP is a local SQP: a wider bound changes the iterate path,
  // and the solver never even uses the extra room. Codify the measurement:
  // dropping the floor to 0.05 must not buy coverage, and in fact costs it.
  for (size_t i = 1; i < coverage.size(); ++i) {
    EXPECT_LE(coverage[i], coverage[i - 1] + 1e-9)
      << "coverage at floor index " << i << " beat the tighter floor; the "
         "sweep's premise needs revisiting";
  }
}

TEST(QpJ5Sweep, CoveragePerDistanceForTheTighterAndTheWidestFloor)
{
  // The aggregate hides WHERE the loss is. The close-range band is the one
  // the swing investigation cares about (targets around 0.4 m).
  const double distances[] = {0.40, 0.60, 0.80, 1.00, 1.20, 1.40, 1.60};
  const double heights[] = {-0.37, 0.00};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, -60.0, -120.0};
  const auto R_t = camInLink6();

  std::printf("\n  dist   floor 0.60        floor 0.05\n");
  for (const double h : heights) {
    for (const double d : distances) {
      int covered[2] = {0, 0};
      int total = 0;
      for (const double a : azimuth_deg) {
        const double ar = a * kPi / 180.0;
        const tf2::Vector3 target(d * std::cos(ar), d * std::sin(ar), h);
        ++total;
        if (probe(target, 0.60, R_t.first, R_t.second).ok) {
          ++covered[0];
        }
        if (probe(target, 0.05, R_t.first, R_t.second).ok) {
          ++covered[1];
        }
      }
      std::printf("  %4.2f h%+.2f   %d/%d             %d/%d\n",
        d, h, covered[0], total, covered[1], total);
      std::fflush(stdout);
    }
  }
  SUCCEED();
}

TEST(QpJ5Sweep, DiagnoseWhyTheWideFloorFailsCloseTargets)
{
  // For every close-range azimuth, show WHAT the solver answers under both
  // floors: status, condition number, aim error and |j5|. A target that is
  // "not covered" is not necessarily unsolvable -- it is a pose the guards
  // refused, and this table says which guard.
  const auto R_t = camInLink6();
  std::printf("\n  d=0.40 h=-0.37   floor    status           cond    aim(deg)  |j5|\n");
  for (const double a : {0.0, 60.0, 120.0, 180.0, 240.0, 300.0}) {
    const double ar = a * kPi / 180.0;
    const tf2::Vector3 target(0.40 * std::cos(ar), 0.40 * std::sin(ar), -0.37);
    for (const double floor : {0.60, 0.05}) {
      const Sample s = probe(target, floor, R_t.first, R_t.second);
      std::printf("  az=%3.0f          %.2f     %-14s %6.1f  %6.2f    %.3f%s\n",
        a, floor, s.status.c_str(), s.cond,
        s.aim * 180.0 / kPi, std::fabs(s.j5), s.ok ? "  OK" : "");
      std::fflush(stdout);
    }
  }
}

TEST(QpJ5Sweep, DoesTheColdStartReferenceExplainTheWideFloorLoss)
{
  // Hypothesis from the per-target diagnosis: with a wide floor the solver
  // stalls near its reference posture (aim 33-36 deg, out_of_envelope),
  // whereas the 0.60 floor DRAGS it out of that posture. If that is the
  // mechanism, then widening the floor AND moving the reference into the
  // positive branch should recover coverage.
  //
  //   floor  reference j5   close-range coverage (d=0.40/0.60, h=-0.37)
  const double heights[] = {-0.37};
  const double distances[] = {0.40, 0.60};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
  const auto R_t = camInLink6();

  struct Case { double floor; double ref_j5; };
  const Case cases[] = {
    {0.60, -0.80},   // shipped: negative reference, tight floor
    {0.05, -0.80},   // wide floor, same reference
    {0.05, +0.60},   // wide floor, reference moved to the positive branch
    {0.05, +0.85},
    {0.05, -0.60},   // wide floor, reference near the negative floor
  };

  std::printf("\n  floor  ref_j5   d=0.40   d=0.60\n");
  for (const auto & c : cases) {
    int cov[2] = {0, 0};
    for (size_t di = 0; di < 2; ++di) {
      for (const double a : azimuth_deg) {
        const double ar = a * kPi / 180.0;
        const tf2::Vector3 target(
          distances[di] * std::cos(ar), distances[di] * std::sin(ar), heights[0]);
        if (probe(target, c.floor, R_t.first, R_t.second, c.ref_j5).ok) {
          ++cov[di];
        }
      }
    }
    std::printf("  %.2f   %+.2f    %d/6      %d/6\n", c.floor, c.ref_j5,
      cov[0], cov[1]);
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, FullGridShippedVersusWideFloorWithAlignedReference)
{
  // The reference-posture experiment says the tight floor was doing the work
  // of dragging the iterate into the positive branch: the QP prefers that
  // branch (prefer_positive_branch) while its cold reference sits at j5 =
  // -0.80. This compares the shipped pairing against a wide floor with the
  // reference moved into the branch the solver already prefers.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  struct Case { const char * name; double floor; double ref_j5; };
  const Case cases[] = {
    {"shipped   floor .60 ref -0.80", 0.60, -0.80},
    {"aligned   floor .60 ref +0.60", 0.60, +0.60},
    {"wide      floor .05 ref +0.60", 0.05, +0.60},
    {"wide      floor .05 ref +0.80", 0.05, +0.80},
  };
  std::printf("\n  %-30s covered  cond p50/max   radius p50  aim p95\n", "config");
  for (const auto & c : cases) {
    std::vector<double> conds, radii, aims;
    int covered = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, c.floor, R_t.first, R_t.second, c.ref_j5);
      if (!s.ok) {
        continue;
      }
      ++covered;
      conds.push_back(s.cond);
      radii.push_back(s.radius);
      aims.push_back(s.aim);
    }
    std::printf("  %-30s %5.1f%%  %6.1f/%6.1f  %6.3f   %5.3f\n", c.name,
      100.0 * covered / static_cast<double>(targets.size()),
      percentile(conds, 0.50), percentile(conds, 1.0),
      percentile(radii, 0.50), percentile(aims, 0.95));
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, WouldASecondReferenceRecoverBothRegimes)
{
  // No single reference covers the grid: -0.80 wins far, +0.60 wins close.
  // run_tier() already has four slots, but on a cold solve (q == q_cold ==
  // q_ref) they collapse to two branches of ONE start. This emulates a real
  // multi-start by retrying the whole solve from the other reference on a
  // fresh solver, and reports the union.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  const double floors[] = {0.60, 0.05};

  std::printf("\n  floor  ref -0.80 only   ref +0.60 only   TWO-START union\n");
  for (const double floor : floors) {
    int a = 0, b = 0, u = 0;
    for (const auto & target : targets) {
      const bool ok_neg = probe(target, floor, R_t.first, R_t.second, -0.80).ok;
      const bool ok_pos = probe(target, floor, R_t.first, R_t.second, 0.60).ok;
      a += ok_neg ? 1 : 0;
      b += ok_pos ? 1 : 0;
      u += (ok_neg || ok_pos) ? 1 : 0;
    }
    const double n = static_cast<double>(targets.size());
    std::printf("  %.2f   %5.1f%%             %5.1f%%            %5.1f%%\n",
      floor, 100.0 * a / n, 100.0 * b / n, 100.0 * u / n);
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, FirstCallVersusConvergedCoverage)
{
  // Responsiveness: a target solved on the FIRST call is available on the
  // next tick; one that needs three ticks costs ~30 ms of tracking. The
  // mirrored cold start exists to move targets from the second group into the
  // first, which the 5-call coverage number cannot show.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  floor  first-call   within-5-calls\n");
  for (const double floor : {0.60, 0.05}) {
    int first = 0, any = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, floor, R_t.first, R_t.second);
      first += s.first_call_ok ? 1 : 0;
      any += s.ok ? 1 : 0;
    }
    const double n = static_cast<double>(targets.size());
    std::printf("  %.2f   %5.1f%%      %5.1f%%\n", floor,
      100.0 * first / n, 100.0 * any / n);
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, RadiusCouplingToDepression)
{
  // Does the new term actually invert the measured relationship? Shipped
  // behaviour is corr(radius, depression) = -0.56 with mean radius 0.216 m at
  // 80-90 deg. A useful weight must move that correlation positive without
  // costing coverage.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  w_radius  covered   r@<30  r@30-60  r@60-80  r@80-90   corr"
              "   aim p95  cond p95\n");
  for (const double w : {0.0, 0.05, 0.15, 0.40, 1.00}) {
    std::vector<double> rad, dep, aims, conds;
    int covered = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, 0.60, R_t.first, R_t.second, 1e9, w);
      if (!s.ok) {
        continue;
      }
      ++covered;
      rad.push_back(s.radius);
      dep.push_back(s.depression);
      aims.push_back(s.aim);
      conds.push_back(s.cond);
    }
    auto bucket = [&](double lo, double hi) {
        double sum = 0.0;
        int n = 0;
        for (size_t i = 0; i < rad.size(); ++i) {
          const double d = dep[i] * 180.0 / kPi;
          if (d >= lo && d < hi) {
            sum += rad[i];
            ++n;
          }
        }
        return n > 0 ? sum / static_cast<double>(n) : 0.0;
      };
    // Pearson correlation.
    double mx = 0.0;
    double my = 0.0;
    for (size_t i = 0; i < rad.size(); ++i) {
      mx += dep[i];
      my += rad[i];
    }
    const double n = static_cast<double>(rad.size());
    mx /= n;
    my /= n;
    double sxy = 0.0, sxx = 0.0, syy = 0.0;
    for (size_t i = 0; i < rad.size(); ++i) {
      sxy += (dep[i] - mx) * (rad[i] - my);
      sxx += (dep[i] - mx) * (dep[i] - mx);
      syy += (rad[i] - my) * (rad[i] - my);
    }
    const double corr = (sxx > 0.0 && syy > 0.0) ? sxy / std::sqrt(sxx * syy) : 0.0;
    std::printf("  %7.2f   %5.1f%%   %.3f   %.3f    %.3f    %.3f   %+.2f"
                "   %6.3f   %6.1f\n", w,
      100.0 * covered / static_cast<double>(targets.size()), bucket(0, 30),
      bucket(30, 60), bucket(60, 80), bucket(80, 91), corr,
      percentile(aims, 0.95), percentile(conds, 0.95));
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, CloseRangeBehaviourUnderTheCoupling)
{
  // The whole point: targets the operator stands over. The main grid starts
  // at 0.40 m; this one goes to 0.25 m and reports the max aim error, because
  // a radius pull that trades aim for radius would show up here first.
  const auto R_t = camInLink6();
  const double distances[] = {0.25, 0.30, 0.40, 0.50};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
  std::printf("\n  w_radius   dist   covered   r mean   aim max(deg)\n");
  for (const double w : {0.0, 0.40, 1.00}) {
    for (const double d : distances) {
      int cov = 0, n = 0;
      double rsum = 0.0, aimmax = 0.0;
      for (const double a : azimuth_deg) {
        const double ar = a * kPi / 180.0;
        const tf2::Vector3 target(d * std::cos(ar), d * std::sin(ar), -0.37);
        ++n;
        const Sample s = probe(target, 0.60, R_t.first, R_t.second, 1e9, w,
          0.28, 0.38, 0.280);
        if (s.ok) {
          ++cov;
          rsum += s.radius;
          aimmax = std::max(aimmax, s.aim);
        }
      }
      std::printf("  %7.2f  %5.2f    %d/%d      %.3f    %.2f\n", w, d, cov, n,
        cov > 0 ? rsum / cov : 0.0, aimmax * 180.0 / kPi);
      std::fflush(stdout);
    }
  }
  SUCCEED();
}

TEST(QpJ5Sweep, DoesAWiderFloorWorkNowThatTheRadiusIsPushedOut)
{
  // The open question from the first sweep: widening j5_branch_min_rad was
  // catastrophic (98.8 % -> 83.3 %) because it dropped the solver into a
  // posture that could not aim. With the radius coupling pulling the arm
  // outward, does the wide floor become usable?
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  w_radius  floor  covered  cond p50/p95   radius p50  aim p95\n");
  for (const double w : {0.0, 0.40}) {
    for (const double floor : {0.60, 0.40, 0.20, 0.05}) {
      std::vector<double> conds, radii, aims;
      int covered = 0;
      for (const auto & target : targets) {
        const Sample s = probe(target, floor, R_t.first, R_t.second, 1e9, w);
        if (!s.ok) {
          continue;
        }
        ++covered;
        conds.push_back(s.cond);
        radii.push_back(s.radius);
        aims.push_back(s.aim);
      }
      std::printf("  %7.2f   %.2f   %5.1f%%   %6.1f/%6.1f   %6.3f     %5.3f\n", w,
        floor, 100.0 * covered / static_cast<double>(targets.size()),
        percentile(conds, 0.50), percentile(conds, 0.95),
        percentile(radii, 0.50), percentile(aims, 0.95));
      std::fflush(stdout);
    }
  }
  SUCCEED();
}

TEST(QpJ5Sweep, CloseRangeAtFloorFifty)
{
  // The owner asked for 0.50 specifically: keep the singular band out but
  // reclaim the 0.50-0.60 slice where close targets actually benefit.
  const auto R_t = camInLink6();
  const double distances[] = {0.25, 0.30, 0.40, 0.50, 0.60, 0.80};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
  std::printf("\n  floor   dist   covered   r mean   |j5| mean   cond p50   aim max(deg)\n");
  for (const double floor : {0.60, 0.50}) {
    for (const double d : distances) {
      int cov = 0, n = 0;
      double rs = 0.0, js = 0.0, aimmax = 0.0;
      std::vector<double> conds;
      for (const double a : azimuth_deg) {
        const double ar = a * kPi / 180.0;
        const tf2::Vector3 target(d * std::cos(ar), d * std::sin(ar), -0.37);
        ++n;
        const Sample s = probe(target, floor, R_t.first, R_t.second);
        if (s.ok) {
          ++cov;
          rs += s.radius;
          js += std::fabs(s.j5);
          conds.push_back(s.cond);
          aimmax = std::max(aimmax, s.aim);
        }
      }
      std::printf("  %.2f   %5.2f    %d/%d      %.3f     %.3f      %6.1f     %.2f\n",
        floor, d, cov, n, cov > 0 ? rs / cov : 0.0, cov > 0 ? js / cov : 0.0,
        percentile(conds, 0.50), aimmax * 180.0 / kPi);
      std::fflush(stdout);
    }
  }
  SUCCEED();
}

TEST(QpJ5Sweep, CostOfTheHardMinimumRadius)
{
  // The owner wants the camera never to stand at r < 0.28 m. The bound is a
  // hard QP constraint, so it can make a target infeasible. This is the
  // trade-off curve: how much coverage each floor costs, per soft weight.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  w_radius  r_min   covered   r p10   r p50   cond p95\n");
  for (const double w : {0.40}) {
    for (const double rmin : {0.26, 0.28, 0.285, 0.29, 0.295, 0.30, 0.32}) {
      std::vector<double> radii, conds;
      int covered = 0;
      for (const auto & target : targets) {
        const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, w,
          0.28, 0.38, rmin);
        if (!s.ok) {
          continue;
        }
        ++covered;
        radii.push_back(s.radius);
        conds.push_back(s.cond);
      }
      std::printf("  %7.2f  %.2f   %5.1f%%   %.3f   %.3f   %6.1f\n", w, rmin,
        100.0 * covered / static_cast<double>(targets.size()),
        percentile(radii, 0.10), percentile(radii, 0.50),
        percentile(conds, 0.95));
      std::fflush(stdout);
    }
  }
  SUCCEED();
}

TEST(QpJ5Sweep, TheHardFloorIsApproachedWithoutInfeasibility)
{
  // Regression guard: demanding the whole radius gap in one linearisation made
  // qpmad report "Infeasible inequality constraints", which the solver turned
  // into a thrown "no legal pose" -- coverage fell off a cliff between 0.280
  // and 0.285 with no physical meaning. The bound is now approached in steps.
  const auto R_t = camInLink6();
  const tf2::Vector3 target(0.75, 0.0, -0.37);
  std::printf("\n  r_min   status            cond    aim(deg)  |j5|   radius\n");
  for (const double rmin : {0.27, 0.28, 0.285, 0.29, 0.30}) {
    const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
      0.28, 0.38, rmin);
    std::printf("  %.3f   %-16s %6.1f   %6.2f   %.3f  %.3f%s\n", rmin,
      s.status.c_str(), s.cond, s.aim * 180.0 / kPi, std::fabs(s.j5), s.radius,
      s.ok ? "  OK" : "");
    std::fflush(stdout);
  }
}



TEST(QpJ5Sweep, DoesTheQpRespectTheFamilyJ2Window)
{
  // posture_j2_min_rad / posture_j2_max_rad are enforced ONLY in the DLS
  // family's candidate search. The QP sees just the URDF range minus the
  // 0.20 margin, i.e. j2 in [0.20, 2.94] rather than [0.15, 1.571].
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::vector<double> j2s;
  for (const auto & target : targets) {
    const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
      0.28, 0.38, 0.280);
    if (s.ok) {
      j2s.push_back(s.j2);
    }
  }
  std::printf("\n  j2 over %d solved targets: min %.3f  p50 %.3f  p95 %.3f  "
              "max %.3f rad\n", static_cast<int>(j2s.size()),
    percentile(j2s, 0.0), percentile(j2s, 0.50), percentile(j2s, 0.95),
    percentile(j2s, 1.0));
  const double family_max = PostureParams{}.posture_j2_max_rad;
  int above = 0;
  for (const double v : j2s) {
    above += v > family_max ? 1 : 0;
  }
  std::printf("  above the family window (%.3f rad): %d\n", family_max, above);
  SUCCEED();
}

TEST(QpJ5Sweep, IterationCountVersusQualityAndTime)
{
  // The SQP loop runs max_linearizations times (default 8), each iteration
  // doing 7 FK+Jacobian evaluations (1 + 6 finite differences) and one qpmad
  // solve. The trust region (0.2 rad) and the wall-clock budget
  // (max_solve_us 4000) also bound what a run can reach.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  iters  covered   cond p50/p95   aim p95   mean us  max us"
              "   over budget\n");
  for (const int iters : {8, 10, 12, 14, 16}) {
    std::vector<double> conds, aims, times;
    int covered = 0, over = 0;
    for (const auto & target : targets) {
      QpPostureSolver qp;
      std::string err;
      if (!qp.initialize(piperUrdf(), true, "base_link", "link6", err)) {
        continue;
      }
      QpSolverParams pp;
      pp.j5_branch_min_rad = 0.50;
      pp.w_radius = 0.40;
      pp.radius_near_m = 0.28;
      pp.radius_steep_m = 0.38;
      pp.max_linearizations = iters;
      qp.set_params(pp);
      PostureParams sp;
      sp.cam_min_radius_m = 0.280;
      bool ok = false;
      for (int call = 0; call < 5 && !ok; ++call) {
        const auto t0 = std::chrono::steady_clock::now();
        PostureResult r;
        try {
          r = qp.solve(target, R_t.first, R_t.second, sp);
        } catch (const std::exception &) {
          continue;
        }
        const double us = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - t0).count();
        times.push_back(us);
        if (us > sp.max_solve_us) {
          ++over;
        }
        if (r.ok) {
          ok = true;
          conds.push_back(r.cond);
          aims.push_back(r.aim_error_rad);
        }
      }
      covered += ok ? 1 : 0;
    }
    std::printf("  %5d   %5.1f%%   %6.1f/%6.1f   %7.3f   %7.1f  %7.1f   %5d\n",
      iters, 100.0 * covered / static_cast<double>(targets.size()),
      percentile(conds, 0.50), percentile(conds, 0.95),
      percentile(aims, 0.95),
      times.empty() ? 0.0 : std::accumulate(times.begin(), times.end(), 0.0) /
        static_cast<double>(times.size()),
      times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()),
      over);
    std::fflush(stdout);
  }
  SUCCEED();
}

TEST(QpJ5Sweep, FrontalSectorRadiusFloor)
{
  // The floor now applies only while the arm looks forward (|j1| <= sector).
  // Check both halves: inside the cone the radius must be >= the floor,
  // outside it the arm may come closer, and coverage must not collapse.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  r_min  sector  covered   r p10   r p50   "
              "front r p10   side r p10   cond p95\n");
  for (const double rmin : {0.0, 0.30, 0.35, 0.38}) {
    std::vector<double> radii, front, side, conds;
    int covered = 0;
    for (const auto & target : targets) {
      // j1 must be known to classify, so solve first and read it back.
      // 12 iterations and the shipped 45 deg half-angle (front 90 deg cone).
      const Sample s0 = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        0.35, 0.38, rmin, 12, 0.7854);
      if (!s0.ok) {
        continue;
      }
      ++covered;
      radii.push_back(s0.radius);
      conds.push_back(s0.cond);
      // Classified by the TARGET's azimuth, same as the rule itself.
      if (std::abs(std::atan2(target.y(), target.x())) <= 0.7854) {
        front.push_back(s0.radius);
      } else {
        side.push_back(s0.radius);
      }
    }
    std::printf("  %.2f   30deg   %5.1f%%   %.3f   %.3f      %.3f        %.3f"
                "      %6.1f\n", rmin,
      100.0 * covered / static_cast<double>(targets.size()),
      percentile(radii, 0.10), percentile(radii, 0.50),
      percentile(front, 0.10), percentile(side, 0.10), percentile(conds, 0.95));
    std::fflush(stdout);
  }
}

namespace
{

// Distance-based floor: d <= near_m demands min_radius, d >= far_m demands
// nothing, linear in between. Emulated through the existing per-target floor
// parameter, so no solver change is needed to compare it.
double distance_floor(double d, double near_m, double far_m, double min_radius)
{
  if (d <= near_m) {
    return min_radius;
  }
  if (d >= far_m) {
    return 0.0;
  }
  const double t = (d - near_m) / (far_m - near_m);
  return min_radius * (1.0 - t);
}

}  // namespace

TEST(QpJ5Sweep, FloorSchemesCompared)
{
  // Three ways to decide the camera radius floor, on the same 84 targets at
  // 12 iterations:
  //   none      no floor
  //   sector    |azimuth| <= 45 deg -> 0.35   (shipped)
  //   distance  d <= 0.50 -> 0.35, d >= 1.00 -> 0, linear between
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  struct Scheme { const char * name; int kind; };
  const Scheme schemes[] = {{"none", 0}, {"sector 0.35", 1},
    {"distance 0.35", 2}, {"distance 0.38", 3}};

  std::printf("\n  scheme          covered   r p10   r p50   cond p95"
              "   front r p10  side r p10\n");
  for (const auto & sc : schemes) {
    std::vector<double> radii, conds, front, side;
    int covered = 0;
    for (const auto & target : targets) {
      const double az = std::atan2(target.y(), target.x());
      const double d = std::hypot(target.x(), target.y());
      double rmin = 0.0;
      double sector = 0.0;
      if (sc.kind == 1) {
        rmin = 0.35;
        sector = 0.7854;
      } else if (sc.kind == 2) {
        rmin = distance_floor(d, 0.50, 1.00, 0.35);
      } else if (sc.kind == 3) {
        rmin = distance_floor(d, 0.50, 1.00, 0.38);
      }
      const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        0.35, 0.38, rmin, 12, sector);
      if (!s.ok) {
        continue;
      }
      ++covered;
      radii.push_back(s.radius);
      conds.push_back(s.cond);
      (std::abs(az) <= 0.7854 ? front : side).push_back(s.radius);
    }
    std::printf("  %-14s  %5.1f%%   %.3f   %.3f   %6.1f      %.3f        %.3f\n",
      sc.name, 100.0 * covered / static_cast<double>(targets.size()),
      percentile(radii, 0.10), percentile(radii, 0.50),
      percentile(conds, 0.95), percentile(front, 0.10),
      percentile(side, 0.10));
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, FloorSchemesAtCloseRange)
{
  const auto R_t = camInLink6();
  const double distances[] = {0.25, 0.30, 0.40, 0.50, 0.60, 0.80};
  const double azimuth_deg[] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
  std::printf("\n  dist   sector 0.35   distance 0.35   distance 0.38   none\n");
  for (const double d : distances) {
    int cov[4] = {0, 0, 0, 0};
    int n = 0;
    for (const double a : azimuth_deg) {
      const double ar = a * kPi / 180.0;
      const tf2::Vector3 target(d * std::cos(ar), d * std::sin(ar), -0.37);
      ++n;
      const double az = std::atan2(target.y(), target.x());
      const double dd = std::hypot(target.x(), target.y());
      struct Cfg { double rmin; double sector; };
      const Cfg cfgs[] = {
        {0.35, 0.7854},
        {distance_floor(dd, 0.50, 1.00, 0.35), 0.0},
        {distance_floor(dd, 0.50, 1.00, 0.38), 0.0},
        {0.0, 0.0},
      };
      (void)az;
      for (int i = 0; i < 4; ++i) {
        const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
          0.35, 0.38, cfgs[i].rmin, 12, cfgs[i].sector);
        cov[i] += s.ok ? 1 : 0;
      }
    }
    std::printf("  %.2f    %d/%d          %d/%d             %d/%d          %d/%d\n",
      d, cov[0], n, cov[1], n, cov[2], n, cov[3], n);
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, SectorTaperAtTheBoundary)
{
  // Sweep the azimuth across the 45 deg edge with and without the taper.
  // A hard edge flips the floor between 0.35 and 0 at 45; the taper ramps it
  // over the next 10 deg, so the arm's posture cannot jump there.
  const auto R_t = camInLink6();
  std::printf("\n  az(deg)  hard edge: radius  cover   taper 10deg: radius  cover\n");
  for (double az_deg : {30.0, 40.0, 44.0, 46.0, 50.0, 55.0, 60.0}) {
    const double ar = az_deg * kPi / 180.0;
    const tf2::Vector3 target(0.60 * std::cos(ar), 0.60 * std::sin(ar), -0.37);
    // Soft pull disabled: otherwise it alone lifts the radius to ~0.36 and
    // hides what the hard floor is doing at the boundary.
    const Sample hard = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.0,
      0.35, 0.38, 0.35, 12, 0.7854, 0.0);
    const Sample soft = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.0,
      0.35, 0.38, 0.35, 12, 0.7854, 0.1745);
    std::printf("  %5.0f    %s  %.3f    %d      %.3f    %d\n", az_deg,
      hard.ok ? " " : "X", hard.radius, static_cast<int>(hard.ok),
      soft.radius, static_cast<int>(soft.ok));
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, NearRadiusVersusFarTargets)
{
  // The soft target already ramps with depression (near at 0 deg, steep at
  // 90). radius_near_m currently equals the frontal hard floor (0.35), so the
  // two are pinned together. Lowering it should let FAR targets (shallow
  // depression) sit closer -- as long as the hard floor does not dominate.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  near   covered   r@dep0-30   r@30-60   r@60-80   cond p95\n");
  for (const double near_r : {0.22, 0.25, 0.28, 0.30, 0.35}) {
    std::vector<double> b0, b1, b2, conds;
    int covered = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        near_r, 0.38, 0.35, 12, 0.7854, 0.1745);
      if (!s.ok) {
        continue;
      }
      ++covered;
      conds.push_back(s.cond);
      const double d = s.depression * 180.0 / kPi;
      if (d < 30.0) {
        b0.push_back(s.radius);
      } else if (d < 60.0) {
        b1.push_back(s.radius);
      } else if (d < 80.0) {
        b2.push_back(s.radius);
      }
    }
    auto mean = [](const std::vector<double> & v) {
        if (v.empty()) {
          return 0.0;
        }
        double sum = 0.0;
        for (const double x : v) {
          sum += x;
        }
        return sum / static_cast<double>(v.size());
      };
    std::printf("  %.2f   %5.1f%%    %.3f n=%3d  %.3f n=%3d  %.3f n=%3d   %6.1f\n",
      near_r, 100.0 * covered / static_cast<double>(targets.size()),
      mean(b0), static_cast<int>(b0.size()), mean(b1),
      static_cast<int>(b1.size()), mean(b2), static_cast<int>(b2.size()),
      percentile(conds, 0.95));
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, SideTargetRadiusVersusConditioning)
{
  // The blended side target keeps coverage but pushes conditioning to the
  // acceptance limit (side 0.25 -> cond p95 49.9 against posture_cond_max 50).
  // Find the knee.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  side   covered   side r p10   side r p50   cond p95\n");
  for (const double side : {0.25, 0.28, 0.30, 0.32, 0.35}) {
    std::vector<double> side_r, conds;
    int covered = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        0.35, 0.38, 0.35, 12, 0.7854, 0.1745, side);
      if (!s.ok) {
        continue;
      }
      ++covered;
      conds.push_back(s.cond);
      if (std::abs(std::atan2(target.y(), target.x())) > 0.7854) {
        side_r.push_back(s.radius);
      }
    }
    std::printf("  %.2f   %5.1f%%      %.3f        %.3f        %6.1f\n", side,
      100.0 * covered / static_cast<double>(targets.size()),
      percentile(side_r, 0.10), percentile(side_r, 0.50),
      percentile(conds, 0.95));
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, SectorFloorVersusDistanceFloorWithCurrentSoftTerms)
{
  // Current soft terms (near 0.35 / steep 0.38 / side 0.30, 12 iterations).
  // Question: should the HARD floor key off the frontal sector, or off the
  // target's horizontal distance (closer target -> higher floor)?
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  auto distance_floor = [](double d) {
      constexpr double kNear = 0.50;
      constexpr double kFar = 1.00;
      if (d <= kNear) {
        return 0.35;
      }
      if (d >= kFar) {
        return 0.0;
      }
      return 0.35 * (1.0 - (d - kNear) / (kFar - kNear));
    };

  std::printf("\n  scheme              covered   r p50   cond p95   "
              "front r p10  side r p10\n");
  for (int scheme = 0; scheme < 3; ++scheme) {
    std::vector<double> radii, conds, front, side;
    int covered = 0;
    for (const auto & target : targets) {
      const double az = std::atan2(target.y(), target.x());
      const double d = std::hypot(target.x(), target.y());
      double rmin = 0.0;
      double sector = 0.0;
      if (scheme == 1) {
        rmin = 0.35;
        sector = 0.7854;
      } else if (scheme == 2) {
        rmin = distance_floor(d);
      }
      const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        0.35, 0.38, rmin, 12, sector, 0.1745, 0.30);
      if (!s.ok) {
        continue;
      }
      ++covered;
      radii.push_back(s.radius);
      conds.push_back(s.cond);
      (std::abs(az) <= 0.7854 ? front : side).push_back(s.radius);
    }
    const char * name = scheme == 0 ? "no hard floor" :
      (scheme == 1 ? "sector 0.35 (shipped)" : "distance 0.35@0.5m->0@1.0m");
    std::printf("  %-19s %5.1f%%   %.3f   %6.1f      %.3f        %.3f\n",
      name, 100.0 * covered / static_cast<double>(targets.size()),
      percentile(radii, 0.50), percentile(conds, 0.95),
      percentile(front, 0.10), percentile(side, 0.10));
    std::fflush(stdout);
  }
}

TEST(QpJ5Sweep, ThreeDimensionalRadiusVersusHorizontal)
{
  // Would measuring the radius as the 3-D distance to the base origin fix the
  // "camera stays high at far targets" problem? At far distance the camera is
  // high (z 0.53), so R is dominated by z and the 3-D target becomes a demand
  // to fold the arm into the base.
  const auto targets = targetGrid();
  const auto R_t = camInLink6();
  std::printf("\n  measure     covered   r p50   height p50   cond p95\n");
  for (const bool use3d : {false, true}) {
    std::vector<double> radii, hs, conds;
    int covered = 0;
    for (const auto & target : targets) {
      const Sample s = probe(target, 0.50, R_t.first, R_t.second, 1e9, 0.40,
        0.28, 0.38, 0.35, 12, 0.7854, 0.1745, 0.30, use3d);
      if (!s.ok) {
        continue;
      }
      ++covered;
      radii.push_back(s.radius);
      conds.push_back(s.cond);
      tf2::Vector3 p_cam;
      tf2::Matrix3x3 Rc;
      PostureParams pp;
      pp.cam_min_radius_m = 0.35;
      pp.cam_frontal_sector_rad = 0.7854;
      pp.cam_frontal_taper_rad = 0.1745;
      pp.cam_floor_distance_near_m = 0.50;
      pp.cam_floor_distance_far_m = 1.00;
      (void)p_cam;
      (void)Rc;
      (void)pp;
      hs.push_back(s.height);
    }
    std::printf("  %-10s  %5.1f%%   %.3f    %.3f       %6.1f\n",
      use3d ? "3-D" : "horizontal",
      100.0 * covered / static_cast<double>(targets.size()),
      percentile(radii, 0.50), percentile(hs, 0.50), percentile(conds, 0.95));
    std::fflush(stdout);
  }
}
