// tests/test_lookat_solver_ab.cpp
//
// Coverage/continuity A/B between the DLS posture family and the QP solver
// (plan 2026-11-05, Task 6). Prints one table row per solver and asserts the
// hard parts (zero envelope/cond violations, zero joint-margin violations).

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <tuple>
#include <vector>
#include <map>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/calibration_json.hpp"
#include "lgw_bt_planner/lookat_posture_family.hpp"
#include "lgw_bt_planner/lookat_qp_solver.hpp"
#include "lgw_bt_planner/lookat_solver.hpp"
#include "lgw_bt_planner/piper_limits.hpp"

using namespace lgw_bt_planner;  // NOLINT

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
  std::tie(R, t) = parse_calibration_json(
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/calibration/arm_calibration.json");
  return {R, t + tf2::Vector3(0.0, 0.0, 0.12)};
}

struct Stats
{
  int attempts{0};
  int ok{0};
  int out_of_envelope{0};
  int held_last{0};
  int thrown{0};
  double fail_aim_sum{0.0};
  int envelope_violations{0};
  int margin_violations{0};
  double worst_step_m{0.0};
  double cond_sum{0.0};
};

}  // namespace

TEST(LookatSolverAb, CoverageAndContinuity)
{
  const PostureParams p;
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();

  for (int which = 0; which < 3; ++which) {
    PostureFamily dls;
    QpPostureSolver qp;
    std::string err;
    ASSERT_TRUE(dls.initialize(piperUrdf(), true, "base_link", "link6", err));
    ASSERT_TRUE(qp.initialize(piperUrdf(), true, "base_link", "link6", err));
    LookatSolver * solver = which == 0 ? static_cast<LookatSolver *>(&dls) : &qp;
    const bool fresh_per_target = which == 2;

    Stats s;
    int multi_ok = 0;
    int az_bins[5] = {0, 0, 0, 0, 0};   // |az| bands: <60, 60-90, 90-120, 120-150, 150+
    std::map<std::string, int> band_fail;
    const double kDeg = 3.14159265358979323846 / 180.0;
    for (double h : {0.15, 0.0, -0.20, -0.37}) {
      for (double d : {0.35, 0.50, 0.75, 1.00, 1.30, 1.60}) {
        if (which == 1) {
          // The sweep jumps 350 deg between bands; the runtime never does.
          // Reset the warm start per band so azimuth is tracked continuously.
          std::string rerr;
          qp.initialize(piperUrdf(), true, "base_link", "link6", rerr);
        }
        tf2::Vector3 prev;
        bool have_prev = false;
        for (double azdeg = -175.0; azdeg <= 175.0; azdeg += 10.0) {
          const double az = azdeg * kDeg;
          const tf2::Vector3 target(d * std::cos(az), d * std::sin(az), h);
          // One call = one 10 ms control tick. Give every target up to five
          // ticks (50 ms) to converge, otherwise the QP's per-tick motion
          // limit would be scored as a coverage failure.
          // which==2: a brand-new QP per target, to separate the solver's
          // intrinsic coverage from warm-start contamination across targets.
          QpPostureSolver fresh;
          LookatSolver * active = solver;
          if (fresh_per_target) {
            std::string ferr;
            fresh.initialize(piperUrdf(), true, "base_link", "link6", ferr);
            active = &fresh;
          }
          PostureResult r;
          for (int tick = 0; tick < 5; ++tick) {
            try {
              r = active->solve(target, R, t, p);
            } catch (const std::exception &) {
              r.ok = false;
              r.status = "thrown";
              break;
            }
            if (r.ok) {
              break;
            }
          }
          ++s.attempts;
          if (r.ok) {
            ++s.ok;
            s.cond_sum += r.cond;
            if (have_prev) {
              s.worst_step_m = std::max(
                s.worst_step_m, (r.tcp_desired.p - prev).length());
            }
            prev = r.tcp_desired.p;
            have_prev = true;
            tf2::Vector3 cam;
            tf2::Matrix3x3 Rc;
            std::string ferr;
            ASSERT_TRUE(
              qp.cameraPose(r.joints, R, t, cam, Rc, ferr)) << ferr;
            if (cam.z() < p.cam_min_height_m - 1e-6 ||
              std::hypot(cam.x(), cam.y()) > p.cam_max_radius_m + 1e-6 ||
              r.cond > p.cond_max)
            {
              ++s.envelope_violations;
            }
            for (int i = 0; i < 6; ++i) {
              if (r.joints[i] < kPiperLower[i] + p.joint_margin - 1e-6 ||
                r.joints[i] > kPiperUpper[i] - p.joint_margin + 1e-6)
              {
                ++s.margin_violations;
                break;
              }
            }
          } else {
            if (which == 1) {
              // Diagnostic only: does the ALT reference recover the target?
              PostureParams alt = p;
              alt.posture_j3_rad = p.posture_j3_alt_rad;
              alt.posture_j5_rad = p.posture_j5_alt_rad;
              for (int tick = 0; tick < 5; ++tick) {
                try {
                  const auto r2 = solver->solve(target, R, t, alt);
                  if (r2.ok) {
                    ++multi_ok;
                    break;
                  }
                } catch (const std::exception &) {
                  break;
                }
              }
            }
            if (r.status == "out_of_envelope") {
              ++s.out_of_envelope;
            } else if (r.status == "held_last") {
              ++s.held_last;
            } else if (r.status == "thrown") {
              ++s.thrown;
            }
            s.fail_aim_sum += r.aim_error_rad;
            {
              const double a = std::abs(azdeg);
              const int bin = a < 60.0 ? 0 : (a < 90.0 ? 1 : (a < 120.0 ? 2 : (a < 150.0 ? 3 : 4)));
              ++az_bins[bin];
            }
            ++band_fail[std::string("h=") + std::to_string(h) + " d=" +
              std::to_string(d)];
            have_prev = false;
          }
        }
      }
    }
    const int fails = s.attempts - s.ok;
    std::printf(
      "[AB] %-3s attempts=%d ok=%d (%.0f%%) envelope_viol=%d margin_viol=%d "
      "worst_step=%.3fm mean_cond=%.1f | fail: ooe=%d held=%d thrown=%d "
      "mean_fail_aim=%.2frad\n",
      which == 0 ? "dls" : (which == 1 ? "qpw" : "qpf"), s.attempts, s.ok,
      100.0 * s.ok / std::max(1, s.attempts), s.envelope_violations,
      s.margin_violations, s.worst_step_m,
      s.ok > 0 ? s.cond_sum / s.ok : 0.0,
      s.out_of_envelope, s.held_last, s.thrown,
      fails > 0 ? s.fail_aim_sum / fails : 0.0);
    if (which == 1) {
      std::printf("[AB] alt-reference retry recovers %d of %d failures\n",
        multi_ok, fails);
      std::printf("[AB-az] |az|<60:%d 60-90:%d 90-120:%d 120-150:%d >150:%d\n",
        az_bins[0], az_bins[1], az_bins[2], az_bins[3], az_bins[4]);
    }
    if (which == 1) {
      for (const auto & kv : band_fail) {
        if (kv.second >= 8) {
          std::printf("[AB-band] %s fails=%d\n", kv.first.c_str(), kv.second);
        }
      }
    }
    EXPECT_EQ(s.envelope_violations, 0);
    EXPECT_EQ(s.margin_violations, 0);
  }
}
