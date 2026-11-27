// src/lookat_qp_solver.cpp — see the header for the formulation.

#include "lgw_bt_planner/lookat_qp_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include <qpmad/solver.h>

#include "lgw_bt_planner/lookat_posture_family.hpp"
#include "lgw_bt_planner/piper_limits.hpp"

namespace lgw_bt_planner
{

namespace
{

constexpr int kDofs = 6;
constexpr double kFdStep = 1e-4;
constexpr double kMinTargetNorm = 1e-3;
// Per-linearisation trust region (rad). Not a per-tick motion cap.
constexpr double kTrustRegionRad = 0.2;
constexpr double kHalfPi = 1.57079632679489661923;

// How strongly the frontal sector applies to this target: 1 inside
// |azimuth| <= sector, linear across the taper, 0 beyond. Judged on the TARGET
// rather than the solved j1 so it stays constant across the SQP iterations and
// across every candidate for that target. sector <= 0 means "everywhere".
double frontal_weight(const PostureParams & params, const tf2::Vector3 & target)
{
  if (params.cam_frontal_sector_rad <= 0.0) {
    return 1.0;
  }
  const double azimuth = std::abs(std::atan2(target.y(), target.x()));
  if (azimuth <= params.cam_frontal_sector_rad) {
    return 1.0;
  }
  const double taper = std::max(0.0, params.cam_frontal_taper_rad);
  if (taper <= 0.0 || azimuth >= params.cam_frontal_sector_rad + taper) {
    return 0.0;
  }
  return 1.0 - (azimuth - params.cam_frontal_sector_rad) / taper;
}

// Distance factor for the floor: 1 at cam_floor_distance_near_m, 0 at
// cam_floor_distance_far_m, linear between.
double distance_weight(const PostureParams & params, const tf2::Vector3 & target)
{
  const double near_d = params.cam_floor_distance_near_m;
  const double far_d = params.cam_floor_distance_far_m;
  if (far_d <= near_d) {
    return 1.0;
  }
  const double d = std::hypot(target.x(), target.y());
  if (d <= near_d) {
    return 1.0;
  }
  if (d >= far_d) {
    return 0.0;
  }
  return 1.0 - (d - near_d) / (far_d - near_d);
}

// Hard floor for this target, in metres: sector AND distance weighted.
double radius_floor(const PostureParams & params, const tf2::Vector3 & target)
{
  if (params.cam_min_radius_m <= 0.0) {
    return 0.0;
  }
  return params.cam_min_radius_m * frontal_weight(params, target) *
         distance_weight(params, target);
}

using Vec6 = std::array<double, kDofs>;

// Radius of the camera for this configuration: horizontal xoy by default, or
// the 3-D distance to the base origin when qp.radius_3d is set.
double camera_radius(const tf2::Vector3 & cam, bool use_3d)
{
  return use_3d ? cam.length() : std::hypot(cam.x(), cam.y());
}

// 4-vector task error: optical-axis misalignment (3) + weighted roll (1).
// Same shape and weights as PostureFamily::solveFree's error, but the axis
// term is the chord u - R_z instead of the cross product: the cross product
// has a second zero at 180 deg (see the .hpp note). Both are first-order
// equivalent near alignment, so the DLS solver is unaffected.
Eigen::VectorXd task_error(
  const tf2::Vector3 & cam, const tf2::Matrix3x3 & R_cam,
  const tf2::Vector3 & target, double roll_weight)
{
  const tf2::Vector3 raw = target - cam;
  if (!std::isfinite(raw.length()) || raw.length() < 1e-6) {
    // Same guard as the DLS family: a target on the camera origin has no look
    // direction; return a finite zero error so the caller's checks decide.
    return Eigen::VectorXd::Zero(4);
  }
  const tf2::Vector3 u = raw.normalized();
  // u - R_z is zero ONLY at alignment. The cross product is also zero at
  // 180 deg, which makes "looking away" a global minimum of the cost and was
  // the dominant close-target failure (spec 10.2). First-order identical to
  // the cross product near the solution.
  const tf2::Vector3 e_axis = u - R_cam.getColumn(2);
  const tf2::Vector3 world_z(0.0, 0.0, 1.0);
  tf2::Vector3 x_des = world_z.cross(u);
  if (x_des.length() < 1e-9) {
    x_des = tf2::Vector3(1.0, 0.0, 0.0);
  }
  x_des.normalize();
  const double e_roll = R_cam.getColumn(0).dot(u.cross(x_des));

  Eigen::VectorXd e(4);
  e << e_axis.x(), e_axis.y(), e_axis.z(), roll_weight * e_roll;
  return e;
}

double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace

struct QpPostureSolver::Impl
{
  PostureFamily chain;
  QpSolverParams qp;
  bool ready{false};
  bool have_previous{false};
  Vec6 previous{};          // warm start (may be a mid-travel iterate)
  bool have_held{false};
  Vec6 held{};              // last accepted pose, for the HOLD diagnostic
  int branch{0};            // last branch (+1/-1); 0 = none yet
};

QpPostureSolver::QpPostureSolver()
: impl_(std::make_unique<Impl>())
{
}

QpPostureSolver::~QpPostureSolver() = default;

bool QpPostureSolver::initialize(
  const std::string & urdf_path_or_xml, bool is_file,
  const std::string & base_link, const std::string & tip_link,
  std::string & error)
{
  impl_->ready = false;
  if (!impl_->chain.initialize(urdf_path_or_xml, is_file, base_link, tip_link,
    error))
  {
    return false;
  }
  impl_->ready = true;
  impl_->have_previous = false;
  impl_->have_held = false;
  impl_->branch = 0;
  return true;
}

void QpPostureSolver::set_params(const QpSolverParams & params)
{
  impl_->qp = params;
}

bool QpPostureSolver::tipPose(
  const std::array<double, 6> & joints,
  tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
  std::string & error) const
{
  if (!impl_->ready) {
    error = "qp lookat: tipPose() before initialize()";
    return false;
  }
  return impl_->chain.tipPose(joints, p_tip, R_tip, error);
}

bool QpPostureSolver::cameraPose(
  const std::array<double, 6> & joints,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
  std::string & error) const
{
  return impl_->chain.cameraPose(joints, R_cam_in_tip, t_cam_in_tip, p_cam,
           R_cam, error);
}

PostureResult QpPostureSolver::solve(
  const tf2::Vector3 & p_target,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  const PostureParams & params)
{
  if (!impl_->ready) {
    throw std::runtime_error("qp lookat: solve() before initialize()");
  }
  if (std::hypot(p_target.x(), p_target.y()) < kMinTargetNorm) {
    // Same contract as the DLS family: no well-defined look direction.
    throw std::runtime_error(
            "posture family: target on the base z axis, look direction "
            "undefined");
  }

  // The reference sits at camera radius 0.149 m, i.e. INSIDE any radius floor
  // of the kind cam_min_radius_m imposes, so a cold solve walks outward before
  // it can aim. Moving the reference itself inside the allowed region was
  // measured on 2026-11-09 and does NOT help: coverage fell monotonically as
  // ref j2 went 0.89 -> 1.40 (95.2 % -> 89.3 %). The nominal stays.
  const Vec6 q_ref{
    0.0, params.posture_j2_nominal_rad, params.posture_j3_rad, 0.0,
    params.posture_j5_rad, 0.0};

  // The QP keeps a wider joint margin than the DLS: Servo halts at
  // URDF_limit - 0.10, and the shared 0.15 left only 0.05 rad for the
  // tracking lag (real machine, 2026-11-06).
  const double jmargin = std::max(
    params.joint_margin, impl_->qp.joint_margin_rad);

  Vec6 q = impl_->have_previous ? impl_->previous : q_ref;
  for (int i = 0; i < kDofs; ++i) {
    q[i] = clampd(q[i], kPiperLower[i] + jmargin,
        kPiperUpper[i] - jmargin);
  }

  // Analytic j1 seed. The optical axis sits far off the arm's +x, so a j1=0
  // seed would need most of the QP's step budget just to turn towards the
  // target. Applied on the first solve AND whenever the current pose is
  // badly off (side/rear target): with a >57 deg aim error the linearisation
  // is meaningless and the Gauss-Newton step can walk the arm away.
  bool seed_j1_now = !impl_->have_previous;
  if (!seed_j1_now) {
    tf2::Vector3 cam_now;
    tf2::Matrix3x3 R_now;
    std::string seed_err;
    if (impl_->chain.cameraPose(q, R_cam_in_tip, t_cam_in_tip, cam_now, R_now,
      seed_err))
    {
      const tf2::Vector3 u = (p_target - cam_now).normalized();
      const double aim_now = std::acos(std::min(1.0,
        std::max(-1.0, R_now.getColumn(2).dot(u))));
      seed_j1_now = aim_now > 1.0;
    }
  }
  // Seed j1 from the target bearing (see above). Used for the warm start
  // when it is badly off and, unconditionally, for the cold fallback below.
  const auto seed_j1 = [&](Vec6 & qs) {
      Vec6 probe = qs;
      probe[0] = 0.0;
      tf2::Vector3 p0;
      tf2::Matrix3x3 R0;
      std::string probe_error;
      if (!impl_->chain.cameraPose(probe, R_cam_in_tip, t_cam_in_tip, p0, R0,
        probe_error))
      {
        return;
      }
      const tf2::Vector3 axis0 = R0.getColumn(2);
      const double phi0 = std::atan2(axis0.y(), axis0.x());
      const double to_target = std::atan2(
        p_target.y() - p0.y(), p_target.x() - p0.x());
      double j1 = to_target - phi0;
      j1 = std::atan2(std::sin(j1), std::cos(j1));
      qs[0] = clampd(j1, kPiperLower[0] + jmargin,
          kPiperUpper[0] - jmargin);
    };
  if (seed_j1_now) {
    seed_j1(q);
  }

  // Report-only pose when nothing passes this call (DLS returns previous_).
  Pose6 held_pose;
  bool have_held = false;
  Vec6 q_held{};
  if (impl_->have_held) {
    q_held = impl_->held;
    tf2::Vector3 cam_held;
    tf2::Matrix3x3 R_held;
    std::string held_err;
    if (impl_->chain.cameraPose(q_held, R_cam_in_tip, t_cam_in_tip, cam_held,
      R_held, held_err))
    {
      const tf2::Matrix3x3 R_tip = R_held * R_cam_in_tip.transpose();
      held_pose.R = R_tip;
      held_pose.p = cam_held - R_tip * t_cam_in_tip;
      have_held = true;
    }
  }

  // One branch run. Every pass re-linearises and re-solves the QP with the
  // wrist bounded to one side of |j5| = j5_branch_min_rad; the bound is
  // relaxed by the trust region, so a run starting on the other side can
  // travel towards its branch instead of becoming infeasible.
  const auto t_start = std::chrono::steady_clock::now();

  struct BranchOutcome
  {
    int branch{0};
    bool ok{false};
    PostureResult ok_result;
    PostureResult best_legal;
    bool have_best_legal{false};
    Vec6 cache{};
    bool cache_ok{false};
    double cache_aim{std::numeric_limits<double>::infinity()};
  };

  auto run_branch = [&](const Vec6 & q_start, int branch,
    double cond_limit) -> BranchOutcome {
      BranchOutcome out;
      out.branch = branch;
      Vec6 q_last = q_start;
      Vec6 q_cache = q_last;
      bool cache_ok = false;
      double cache_aim = std::numeric_limits<double>::infinity();
      std::string err;

      for (int iter = 0; iter < std::max(1, impl_->qp.max_linearizations);
        ++iter)
      {
        if (params.max_solve_us >= 0.0) {
          const double us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - t_start).count();
          if (us > params.max_solve_us) {
            break;
          }
        }

        tf2::Vector3 cam;
        tf2::Matrix3x3 R_cam;
        if (!impl_->chain.cameraPose(q_last, R_cam_in_tip, t_cam_in_tip, cam,
          R_cam, err))
        {
          throw std::runtime_error("qp lookat: " + err);
        }

        const Eigen::VectorXd e = task_error(
          cam, R_cam, p_target, params.roll_weight);

        Eigen::MatrixXd J(4, kDofs);
        Eigen::MatrixXd J_cam(3, kDofs);
        for (int i = 0; i < kDofs; ++i) {
          Vec6 qp = q_last;
          qp[i] += kFdStep;
          tf2::Vector3 cp;
          tf2::Matrix3x3 Rp;
          if (!impl_->chain.cameraPose(qp, R_cam_in_tip, t_cam_in_tip, cp, Rp,
            err))
          {
            throw std::runtime_error("qp lookat: " + err);
          }
          J.col(i) = (task_error(cp, Rp, p_target, params.roll_weight) - e) /
            kFdStep;
          J_cam(0, i) = (cp.x() - cam.x()) / kFdStep;
          J_cam(1, i) = (cp.y() - cam.y()) / kFdStep;
          J_cam(2, i) = (cp.z() - cam.z()) / kFdStep;
        }

        // min 0.5*x'Hx + g'x, x = dq; the task term is ||J*dq + e||^2.
        Eigen::MatrixXd H = 2.0 * (
          impl_->qp.w_aim * J.transpose() * J +
          (impl_->qp.w_smooth + impl_->qp.w_posture) *
          Eigen::MatrixXd::Identity(kDofs, kDofs));
        Eigen::VectorXd q_last_e(kDofs);
        Eigen::VectorXd q_ref_e(kDofs);
        for (int i = 0; i < kDofs; ++i) {
          q_last_e(i) = q_last[i];
          q_ref_e(i) = q_ref[i];
        }
        Eigen::VectorXd g = 2.0 * (
          impl_->qp.w_aim * J.transpose() * e +
          impl_->qp.w_posture * (q_last_e - q_ref_e));

        // Radius-vs-depression coupling: the camera should stand further out
        // the more steeply it looks down. Linearised like the aim term --
        // d(radius)/dq is the radial direction dotted with the camera
        // position Jacobian.
        if (impl_->qp.w_radius > 0.0) {
          const double r_now = camera_radius(cam, impl_->qp.radius_3d);
          if (r_now > 1e-9) {
            const tf2::Vector3 axis = R_cam.getColumn(2);
            const double dep = std::asin(std::clamp(-axis.z(), -1.0, 1.0));
            const double near_r = impl_->qp.radius_near_m;
            const double steep_r = impl_->qp.radius_steep_m;
            // The soft target must never sit BELOW the hard floor: doing so
            // pulls the solution down to it and the post-FK check then
            // rejects every tick (measured: r_min 0.285 -> 0 % coverage while
            // 0.280 -> 95 %, a cliff with no physical meaning).
            // Inside the cone the target grows with depression (stand back
            // when looking steeply down); outside it falls back to a closer
            // resting radius. One term, blended by the sector weight -- adding
            // a second well instead cost 4.8 points of coverage.
            const double frontal_target = near_r + (steep_r - near_r) *
              std::clamp(dep / kHalfPi, 0.0, 1.0);
            const double sector_w = frontal_weight(params, p_target);
            const double r_target = std::max(
              radius_floor(params, p_target),
              sector_w * frontal_target +
              (1.0 - sector_w) * impl_->qp.radius_side_m);
            Eigen::RowVectorXd n(3);
            n << cam.x() / r_now, cam.y() / r_now,
              (impl_->qp.radius_3d ? cam.z() / r_now : 0.0);
            const Eigen::RowVectorXd J_radius = n * J_cam;
            const double residual = r_now - r_target;
            H += 2.0 * impl_->qp.w_radius * J_radius.transpose() * J_radius;
            g += 2.0 * impl_->qp.w_radius * residual * J_radius.transpose();
          }
        }

        // Box bounds: joint limits with margin, plus a per-PASS trust region.
        // The trust region is not a speed limit -- it keeps each linearisation
        // valid (a QP is local). The solver answers "which pose looks at the
        // target"; how fast to get there is Servo's job.
        Eigen::VectorXd lb(kDofs);
        Eigen::VectorXd ub(kDofs);
        for (int i = 0; i < kDofs; ++i) {
          lb(i) = std::max(
            kPiperLower[i] + jmargin - q_last[i],
            -kTrustRegionRad);
          ub(i) = std::min(
            kPiperUpper[i] - jmargin - q_last[i],
            kTrustRegionRad);
        }

        // Wrist branch: keep |j5| >= j5_branch_min_rad. The bound applies to
        // the STEP, so travel across the singular band is limited by the
        // trust region instead of making the box empty.
        const double lo5 = kPiperLower[4] + jmargin;
        const double hi5 = kPiperUpper[4] - jmargin;
        // A floor at or beyond the joint margin makes both boxes degenerate
        // (every target would fail); clamp instead of silently disabling the
        // lookat when the yaml value is too large.
        const double branch_min = std::min(
          impl_->qp.j5_branch_min_rad, hi5 - 1e-3);
        if (branch_min > 0.0) {
          double j5_lb = lb(4);
          double j5_ub = ub(4);
          if (branch < 0) {
            const double ub_branch = std::min(hi5, -branch_min) - q_last[4];
            j5_ub = std::max(j5_lb, std::min(j5_ub, ub_branch));
          } else {
            const double lb_branch = std::max(lo5, branch_min) - q_last[4];
            j5_lb = std::min(j5_ub, std::max(j5_lb, lb_branch));
          }
          lb(4) = j5_lb;
          ub(4) = j5_ub;
        }
        // Keep the wrist off the joint limit: |j5| <= j5_branch_max_rad.
        // Trust-relaxed, so a pose already outside the cap moves back by at
        // most one step instead of making the box empty.
        const double j5_cap = impl_->qp.j5_branch_max_rad;
        if (j5_cap > 0.0) {
          double j5_ub_c = ub(4);
          double j5_lb_c = lb(4);
          j5_ub_c = std::min(j5_ub_c, std::max(j5_cap - q_last[4], j5_lb_c));
          j5_lb_c = std::max(j5_lb_c, std::min(-j5_cap - q_last[4], j5_ub_c));
          lb(4) = j5_lb_c;
          ub(4) = j5_ub_c;
        }

        // Linearised camera envelope (locally valid; FK-checked below).
        const double radius_now = camera_radius(cam, impl_->qp.radius_3d);
        Eigen::MatrixXd A(2, kDofs);
        Eigen::VectorXd Alb(2);
        Eigen::VectorXd Aub(2);
        A.row(0) = -J_cam.row(2);
        Alb(0) = -1e10;
        Aub(0) = cam.z() - params.cam_min_height_m;
        if (radius_now > 1e-9) {
          const double nx = cam.x() / radius_now;
          const double ny = cam.y() / radius_now;
          Eigen::RowVectorXd n(3);
          n << nx, ny, (impl_->qp.radius_3d ? cam.z() / radius_now : 0.0);
          A.row(1) = n * J_cam;
          // Lower bound too: the camera must not stand over the target. It is
          // approached in steps: demanding the whole gap in one linearisation
          // makes the QP infeasible whenever the iterate sits further inside
          // the floor than one trust-region step can cover (qpmad:
          // "Infeasible inequality constraints", measured 2026-11-09: floor
          // 0.285 -> 0 % coverage while 0.280 -> 95 %). The loop re-linearises
          // up to max_linearizations times, so the iterate walks out and the
          // per-iteration envelope check accepts it once it arrives.
          constexpr double kRadiusStepPerIteration = 0.03;
          const double r_demand = std::min(
            radius_floor(params, p_target),
            radius_now + kRadiusStepPerIteration);
          // n . cam must use the same vector as n itself: with the 3-D
          // measurement switch the radial direction has a z component, and
          // dropping it here made the bound weaker than intended.
          const double n_cam = impl_->qp.radius_3d ?
            radius_now : nx * cam.x() + ny * cam.y();
          Alb(1) = r_demand - n_cam;
          Aub(1) = params.cam_max_radius_m - n_cam;
        } else {
          A.row(1).setZero();
          Alb(1) = -1e10;
          Aub(1) = 1e10;
        }

        Eigen::VectorXd dq = Eigen::VectorXd::Zero(kDofs);
        qpmad::Solver solver;
        const auto status = solver.solve(dq, H, g, lb, ub, A, Alb, Aub);
        if (status != qpmad::Solver::ReturnStatus::OK) {
          break;
        }

        Vec6 q_new{};
        for (int i = 0; i < kDofs; ++i) {
          q_new[i] = q_last[i] + dq(i);
        }

        tf2::Vector3 cam_new;
        tf2::Matrix3x3 R_new;
        if (!impl_->chain.cameraPose(q_new, R_cam_in_tip, t_cam_in_tip,
          cam_new, R_new, err))
        {
          throw std::runtime_error("qp lookat: " + err);
        }
        const tf2::Vector3 u_raw = p_target - cam_new;
        if (!std::isfinite(u_raw.length()) || u_raw.length() < 1e-6) {
          throw std::runtime_error(
                  "qp lookat: target coincides with the camera");
        }
        const tf2::Vector3 u = u_raw.normalized();
        const double cos_aim = std::min(
          1.0, std::max(-1.0, R_new.getColumn(2).dot(u)));
        const double aim = std::acos(cos_aim);
        double cond = std::numeric_limits<double>::infinity();
        if (!impl_->chain.cond(q_new, cond, err)) {
          throw std::runtime_error("qp lookat: " + err);
        }
        const double radius = camera_radius(cam_new, impl_->qp.radius_3d);
        // `radius` may be 0 here when the linearised bound was degenerate, so
        // the lower bound is only enforced when it is meaningful.
        const double r_floor = radius_floor(params, p_target);
        const bool envelope = cam_new.z() >= params.cam_min_height_m - 1e-9 &&
          radius <= params.cam_max_radius_m + 1e-9 &&
          (radius <= 1e-9 || radius >= r_floor - 1e-9);
        const bool finite = std::isfinite(aim) && std::isfinite(cond);
        const bool branch_ok = (branch_min <= 0.0 ||
          (branch < 0 ? q_new[4] <= -branch_min + 1e-9 :
          q_new[4] >= branch_min - 1e-9)) &&
          (j5_cap <= 0.0 || std::abs(q_new[4]) <= j5_cap + 1e-9);
        // 2026-11-19: 硬约束 —— 画面不许倒转。roll 项是软权重, 在接近竖直
        // 或关节受限时仍可能收敛到"绕光轴滚 180 度"的解; 这里按与 roll 项
        // 相同的参照(x_des = z x u)做一次硬判定, 倒转的解一律不接受。
        const bool upright = camera_upright(cam_new, p_target, R_new);

        PostureResult candidate;
        candidate.joints = q_new;
        candidate.aim_error_rad = aim;
        candidate.cond = cond;
        candidate.guarded = false;
        {
          const tf2::Matrix3x3 R_tip = R_new * R_cam_in_tip.transpose();
          candidate.tcp_desired.R = R_tip;
          candidate.tcp_desired.p = cam_new - R_tip * t_cam_in_tip;
        }

        if (aim <= params.aim_tol_rad && cond <= cond_limit && envelope &&
          finite && branch_ok && upright)
        {
          candidate.ok = true;
          candidate.status = "ok";
          out.ok = true;
          out.ok_result = candidate;
          out.cache = q_new;
          out.cache_ok = true;
          return out;
        }

        if (envelope && cond <= cond_limit && finite && branch_ok && upright) {
          out.best_legal = candidate;
          out.have_best_legal = true;
        }

        // An anti-aligned iterate is not progress: it is the flip basin and
        // must never become the next call's start. Everything else (inside
        // the envelope) advances across ticks -- including a mid-travel
        // iterate in the singular band, which is how a target on the other
        // branch is reached (the analytic j1 seed only exists for the first
        // solve). `held` is separate: it only ever stores an accepted pose.
        // 倒转的迭代也不能当作下一拍的起点: 否则会一直困在翻转盆地里。
        if (finite && envelope && cos_aim > 0.0 && upright) {
          q_cache = q_new;
          cache_ok = true;
          cache_aim = aim;
        }

        q_last = q_new;
      }

      out.cache = q_cache;
      out.cache_ok = cache_ok;
      out.cache_aim = cache_aim;
      return out;
    };

  // Hysteresis: stay on the branch the last run used; a fresh solver starts
  // on the negative branch (the primary posture's side). The branch is
  // RECORDED, not inferred from j5: an accepted positive pose sits exactly at
  // +j5_branch_min_rad, so a strict `>` comparison sent the next tick back to
  // the negative branch and the solver alternated every tick.
  const int preferred =
    (impl_->have_previous && impl_->branch != 0) ? impl_->branch :
    (impl_->qp.prefer_positive_branch ? +1 : -1);

  // 2 tiers x 2 starts x 2 branches. Keep this in sync with run_tier().
  BranchOutcome runs[8];
  int n_runs = 0;
  int strict_runs = 0;
  const auto record = [&](const Vec6 & start, int branch, double cond_limit) {
      runs[n_runs] = run_branch(start, branch, cond_limit);
      return runs[n_runs++].ok;
    };
  const auto finish = [&](const BranchOutcome & out, bool relaxed_tier) {
      // Report a branch change before updating the state: the path between
      // the two branches crosses j5 = 0, where Servo hard-stops.
      PostureResult result = out.ok_result;
      result.branch_switched = impl_->have_previous && impl_->branch != 0 &&
        out.branch != impl_->branch;
      result.branch = out.branch;
      // Slot within the tier that won: 0 warm/preferred, 1 warm/other,
      // 2 cold/preferred, 3 cold/other.
      result.run_index = (n_runs - 1) - (relaxed_tier ? strict_runs : 0);
      result.relaxed = relaxed_tier;
      impl_->previous = out.cache;
      impl_->have_previous = true;
      impl_->branch = out.branch;
      if (out.ok_result.cond <= params.cond_max) {
        impl_->held = out.cache;
        impl_->have_held = true;
      }
      return result;
    };

  // Cold fallback: the wrist can sit at the far end of a branch (e.g. j5 at
  // its margin) and never reach the target's basin. Retry from the nominal
  // reference, which is exactly what a fresh solver does; this only runs
  // when both warm-started branches failed.
  Vec6 q_cold = q_ref;
  for (int i = 0; i < kDofs; ++i) {
    q_cold[i] = clampd(q_cold[i], kPiperLower[i] + jmargin,
        kPiperUpper[i] - jmargin);
  }
  seed_j1(q_cold);

  // One tier = (warm start, cold start) x (preferred branch, other branch).
  const auto run_tier = [&](double cond_limit) {
      if (record(q, preferred, cond_limit)) {
        return true;
      }
      if (record(q, -preferred, cond_limit)) {
        return true;
      }
      if (record(q_cold, preferred, cond_limit)) {
        return true;
      }
      if (record(q_cold, -preferred, cond_limit)) {
        return true;
      }
      return false;
    };

  if (run_tier(params.cond_max)) {
    return finish(runs[n_runs - 1], false);
  }
  strict_runs = n_runs;

  // Boundary tier: only reached when every strict run failed, so a target
  // that already solves under cond_max keeps exactly that pose.
  const double relaxed_limit =
    std::max(params.cond_max, impl_->qp.cond_max_relaxed);
  const double target_azimuth =
    std::abs(std::atan2(p_target.y(), p_target.x()));
  const bool relaxed_allowed =
    impl_->qp.relaxed_min_azimuth_rad <= 0.0 ||
    target_azimuth >= impl_->qp.relaxed_min_azimuth_rad;
  if (relaxed_allowed && relaxed_limit > params.cond_max &&
    run_tier(relaxed_limit))
  {
    return finish(runs[n_runs - 1], true);
  }

  // Everything failed. Advance the warm start with the iterate that got
  // CLOSEST to the target (the preferred branch may have made no progress
  // while another one crossed the wrist band); without one the previous
  // start is unchanged and a fresh solver stays fresh.
  const BranchOutcome * cache_src = nullptr;
  for (int i = 0; i < n_runs; ++i) {
    if (!runs[i].cache_ok) {
      continue;
    }
    if (!cache_src || runs[i].cache_aim < cache_src->cache_aim) {
      cache_src = &runs[i];
    }
  }
  if (cache_src) {
    impl_->previous = cache_src->cache;
    impl_->have_previous = true;
    impl_->branch = cache_src->branch;
  }

  // Best effort: the legal pose that aimed closest wins, but only from the
  // strict tier -- a 50-70 pose must never be reported as the diagnostic
  // "best pose" (Servo is already decelerating there).
  const BranchOutcome * best = nullptr;
  for (int i = 0; i < strict_runs; ++i) {
    if (!runs[i].have_best_legal) {
      continue;
    }
    if (!best || runs[i].best_legal.aim_error_rad <
      best->best_legal.aim_error_rad)
    {
      best = &runs[i];
    }
  }
  if (best) {
    PostureResult out = best->best_legal;
    out.ok = false;
    out.out_of_envelope = true;
    out.status = "out_of_envelope";
    return out;
  }

  PostureResult out;
  out.ok = false;
  out.out_of_envelope = true;
  out.status = have_held ? "held_last" : "degenerate";
  if (have_held) {
    out.tcp_desired = held_pose;
    out.joints = q_held;
  } else {
    throw std::runtime_error(
            "qp lookat: no legal pose and no previous solution");
  }
  return out;
}

// ---------------------------------------------------------------------------
// Generic task solve (2026-11-19)
//
// Same engine as solve(), with the task supplying residual / extra rows /
// acceptance. Deliberately a separate entry point: solve() above stays
// byte-identical so the Servo path cannot regress while the pose task is
// being brought up.
// ---------------------------------------------------------------------------
TaskSolveResult QpPostureSolver::solve_task(
  const SolveTask & task, const JointArray & seed,
  const TaskSolveParams & params)
{
  TaskSolveResult out;
  if (!impl_->ready) {
    out.status = "not_initialized";
    return out;
  }

  const auto t_start = std::chrono::steady_clock::now();
  const double jmargin = std::max(0.0, params.joint_margin_rad);
  const double trust = std::max(1e-3, params.trust_region_rad);

  Vec6 q = seed;
  for (int i = 0; i < kDofs; ++i) {
    q[i] = clampd(q[i], kPiperLower[i] + jmargin, kPiperUpper[i] - jmargin);
  }

  bool converged = false;
  int iters = 0;
  std::string err;

  for (iters = 0; iters < std::max(1, params.max_linearizations); ++iters) {
    if (params.max_solve_us >= 0.0) {
      const double us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - t_start).count();
      if (us > params.max_solve_us) {
        out.status = "overrun";
        break;
      }
    }

    const Eigen::VectorXd e = task.residual(q);
    const int n = static_cast<int>(e.size());
    if (n <= 0) {
      out.status = "bad_task";
      return out;
    }
    if (e.norm() <= params.converge_eps) {
      converged = true;
      break;
    }

    Eigen::MatrixXd J(n, kDofs);
    for (int i = 0; i < kDofs; ++i) {
      Vec6 qp = q;
      qp[i] += kFdStep;
      J.col(i) = (task.residual(qp) - e) / kFdStep;
    }

    // min 0.5*x'Hx + g'x, x = dq; the task term is ||J*dq + e||^2, and the
    // posture term pulls toward the seed (stay near the pose we started at).
    Eigen::MatrixXd H = 2.0 * (
      params.w_task * J.transpose() * J +
      (params.w_smooth + params.w_posture) *
      Eigen::MatrixXd::Identity(kDofs, kDofs));

    Eigen::VectorXd q_e(kDofs);
    Eigen::VectorXd q_ref_e(kDofs);
    for (int i = 0; i < kDofs; ++i) {
      q_e(i) = q[i];
      q_ref_e(i) = seed[i];
    }
    Eigen::VectorXd g = 2.0 * (
      params.w_task * J.transpose() * e +
      params.w_posture * (q_e - q_ref_e));

    Eigen::VectorXd lb(kDofs);
    Eigen::VectorXd ub(kDofs);
    for (int i = 0; i < kDofs; ++i) {
      lb(i) = std::max(kPiperLower[i] + jmargin - q[i], -trust);
      ub(i) = std::min(kPiperUpper[i] - jmargin - q[i], trust);
    }
    for (int i = 0; i < kDofs; ++i) {
      if (lb(i) > ub(i)) {
        // The seed sits outside the margin on this joint: hold it rather
        // than hand qpmad an infeasible box.
        lb(i) = ub(i) = 0.0;
      }
    }

    Eigen::MatrixXd A;
    Eigen::VectorXd Alb;
    Eigen::VectorXd Aub;
    task.add_constraints(q, A, Alb, Aub);
    if (A.rows() != Alb.size() || A.rows() != Aub.size() ||
      (A.rows() > 0 && A.cols() != kDofs))
    {
      out.status = "bad_constraints";
      return out;
    }
    for (int i = 0; i < A.rows(); ++i) {
      if (Alb(i) > Aub(i)) {
        std::swap(Alb(i), Aub(i));
      }
    }

    Eigen::VectorXd dq = Eigen::VectorXd::Zero(kDofs);
    qpmad::Solver solver;
    const auto status = (A.rows() > 0)
      ? solver.solve(dq, H, g, lb, ub, A, Alb, Aub)
      : solver.solve(dq, H, g, lb, ub);
    if (status != qpmad::Solver::ReturnStatus::OK) {
      out.status = "qp_failed";
      break;
    }

    for (int i = 0; i < kDofs; ++i) {
      q[i] = clampd(
        q[i] + dq(i), kPiperLower[i] + jmargin, kPiperUpper[i] - jmargin);
    }
  }

  out.q = q;
  out.metrics.iterations = iters + 1;
  out.metrics.solve_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t_start).count();
  double margin = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kDofs; ++i) {
    margin = std::min(
      margin, std::min(q[i] - kPiperLower[i], kPiperUpper[i] - q[i]));
  }
  out.metrics.joint_margin = std::isfinite(margin) ? margin : 0.0;
  out.metrics.residual_norm = task.residual(q).norm();
  double cond = 0.0;
  if (impl_->chain.cond(q, cond, err)) {
    out.metrics.cond = cond;
  }

  if (!converged) {
    if (out.status.empty()) {
      out.status = "no_converge";
    }
    return out;
  }

  std::string why;
  if (!task.accept(q, out.metrics, why)) {
    out.status = "rejected:" + why;
    return out;
  }

  out.ok = true;
  out.status = "ok";
  return out;
}

}  // namespace lgw_bt_planner
