// src/lookat_posture_family.cpp — see the header for the contract.

#include "lgw_bt_planner/lookat_posture_family.hpp"

#include "lgw_bt_planner/piper_limits.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace lgw_bt_planner
{

namespace
{

constexpr int kIterations = 8;
constexpr double kMaxStep = 0.2;        // rad per iteration
constexpr double kAimEpsilon = 1e-5;    // |cross(axis, want)| that counts as aimed
constexpr int kFree[3] = {0, 3, 5};     // j1, j4, j6

// Extra j4 starts besides the caller's seed. Damped least squares is a local
// method: from j4 = 0 it can lock onto the wrist branch on the far side of the
// singularity and stall (measured: 7.7 deg residual on a target the prototype
// centres). Trying both wrist sides costs at most a few extra runs and makes
// the result independent of where the caller happened to be.
//
// 2026-11-05: widened from {+-1.2} after the angular coverage sweep found
// cells at high elevation on the negative-azimuth side that only solve from
// j4 ~ +-1.4..1.5 (lookat_axis_coverage.py --pair=-0.72,-0.8). The original
// +-1.2 seeds are KEPT: the field-proven fallback posture reaches the far
// lateral bearings only from them, and dropping them regressed +-139 deg.
constexpr double kExtraJ4Starts[6] = {-1.5, -1.2, -0.8, 0.8, 1.2, 1.5};

}  // namespace

struct PostureFamily::Impl
{
  KDL::Chain chain;
  // JntToCart()/JntToJac() are non-const, and the node calls cameraPose() from
  // const context every tick. Creating a solver per call would be cheap but
  // noisy; mutable + single-threaded control tick is the simpler trade.
  mutable std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
  mutable std::unique_ptr<KDL::ChainJntToJacSolver> jac;
  bool ready{false};
};

PostureFamily::PostureFamily()
: impl_(std::make_shared<Impl>())
{
}

PostureFamily::~PostureFamily() = default;

bool PostureFamily::initialize(
  const std::string & urdf_path_or_xml, bool is_file,
  const std::string & base_link, const std::string & tip_link,
  std::string & error)
{
  impl_->ready = false;
  impl_->fk.reset();
  impl_->jac.reset();

  if (urdf_path_or_xml.empty()) {
    error = "posture family: empty URDF (robot_description not set?)";
    return false;
  }

  KDL::Tree tree;
  const bool parsed = is_file ?
    kdl_parser::treeFromFile(urdf_path_or_xml, tree) :
    kdl_parser::treeFromString(urdf_path_or_xml, tree);
  if (!parsed) {
    error = "posture family: failed to parse URDF";
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(base_link, tip_link, chain)) {
    error = "posture family: no chain " + base_link + " -> " + tip_link;
    return false;
  }
  if (chain.getNrOfJoints() != 6) {
    error = "posture family: chain " + base_link + " -> " + tip_link +
      " has " + std::to_string(chain.getNrOfJoints()) + " joints, expected 6";
    return false;
  }

  impl_->chain = chain;
  impl_->fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(impl_->chain);
  impl_->jac = std::make_unique<KDL::ChainJntToJacSolver>(impl_->chain);
  impl_->ready = true;
  return true;
}

bool PostureFamily::tipPose(
  const std::array<double, 6> & joints,
  tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
  std::string & error) const
{
  if (!impl_->ready) {
    error = "posture family: tipPose() before initialize()";
    return false;
  }

  KDL::JntArray q(6);
  for (std::size_t i = 0; i < 6; ++i) {
    q(static_cast<unsigned int>(i)) = joints[i];
  }

  KDL::Frame tip;
  if (impl_->fk->JntToCart(q, tip) < 0) {
    error = "posture family: FK failed";
    return false;
  }

  // Same quaternion round-trip as cameraPose(): KDL stores the rotation
  // column-major, tf2's 9-argument ctor is row-major.
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  tip.M.GetQuaternion(x, y, z, w);
  R_tip = tf2::Matrix3x3(tf2::Quaternion(x, y, z, w));
  p_tip = tf2::Vector3(tip.p.x(), tip.p.y(), tip.p.z());
  return true;
}

bool PostureFamily::cameraPose(
  const std::array<double, 6> & joints,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
  std::string & error) const
{
  if (!impl_->ready) {
    error = "posture family: cameraPose() before initialize()";
    return false;
  }

  KDL::JntArray q(6);
  for (std::size_t i = 0; i < 6; ++i) {
    q(static_cast<unsigned int>(i)) = joints[i];
  }

  KDL::Frame tip;
  if (impl_->fk->JntToCart(q, tip) < 0) {
    error = "posture family: FK failed";
    return false;
  }

  // Go through the quaternion rather than GetRot(): KDL's storage order is
  // column-major while tf2's 9-argument constructor is row-major, so copying
  // the raw array silently transposes the rotation.
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  tip.M.GetQuaternion(x, y, z, w);
  const tf2::Matrix3x3 R_tip(tf2::Quaternion(x, y, z, w));
  const tf2::Vector3 p_tip(tip.p.x(), tip.p.y(), tip.p.z());

  p_cam = p_tip + R_tip * t_cam_in_tip;
  R_cam = R_tip * R_cam_in_tip;
  return true;
}

bool PostureFamily::solveFree(
  const tf2::Vector3 & p_target,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  const std::array<double, 6> & seed,
  const PostureParams & params,
  std::array<double, 6> & q_out,
  double & aim_error_rad, double & cond,
  std::string & error) const
{
  if (!impl_->ready) {
    error = "posture family: solveFree() before initialize()";
    return false;
  }

  auto clamp_free = [&params](int idx, double value) {
      return std::min(
        std::max(value, kPiperLower[idx] + params.joint_margin),
        kPiperUpper[idx] - params.joint_margin);
    };

  const tf2::Vector3 world_z(0.0, 0.0, 1.0);
  const double h = 1e-4;

  struct Attempt
  {
    std::array<double, 6> q{};
    double aim{0.0};
    double cond{0.0};
  };

  // One damped-least-squares run. Returns false only on a hard failure
  // (FK / Jacobian / non-finite); a run that fails to converge still yields a
  // valid pose with a large aim error for the caller to judge.
  auto run = [&](const std::array<double, 6> & start, Attempt & out) -> bool {
      std::array<double, 6> q = start;
      for (int idx : kFree) {
        q[idx] = clamp_free(idx, q[idx]);
      }

      tf2::Vector3 cam;
      tf2::Matrix3x3 R_cam;

      for (int iter = 0; iter < kIterations; ++iter) {
        if (!cameraPose(q, R_cam_in_tip, t_cam_in_tip, cam, R_cam, error)) {
          return false;
        }
        const tf2::Vector3 u_raw = p_target - cam;
        if (!std::isfinite(u_raw.length()) || u_raw.length() < 1e-6) {
          error = "posture family: target coincides with the camera";
          return false;
        }
        const tf2::Vector3 u = u_raw.normalized();
        const tf2::Vector3 e_axis = R_cam.getColumn(2).cross(u);
        if (e_axis.length() < kAimEpsilon) {
          break;
        }

        const tf2::Vector3 x_des = world_z.cross(u).normalized();
        const double e_roll = R_cam.getColumn(0).dot(u.cross(x_des));

        Eigen::VectorXd e(4);
        e << e_axis.x(), e_axis.y(), e_axis.z(), params.roll_weight * e_roll;

        Eigen::MatrixXd J(4, 3);
        for (int k = 0; k < 3; ++k) {
          std::array<double, 6> qp = q;
          qp[kFree[k]] += h;
          tf2::Vector3 cp;
          tf2::Matrix3x3 Rp;
          if (!cameraPose(qp, R_cam_in_tip, t_cam_in_tip, cp, Rp, error)) {
            return false;
          }
          const tf2::Vector3 up = (p_target - cp).normalized();
          const tf2::Vector3 ea = Rp.getColumn(2).cross(up);
          const tf2::Vector3 xd = world_z.cross(up).normalized();
          const double er = Rp.getColumn(0).dot(up.cross(xd));
          J(0, k) = (ea.x() - e_axis.x()) / h;
          J(1, k) = (ea.y() - e_axis.y()) / h;
          J(2, k) = (ea.z() - e_axis.z()) / h;
          J(3, k) = params.roll_weight * (er - e_roll) / h;
        }

        Eigen::VectorXd dq = -J.completeOrthogonalDecomposition().solve(e);
        const double step = dq.norm();
        if (!std::isfinite(step)) {
          error = "posture family: non-finite step";
          return false;
        }
        if (step > kMaxStep) {
          dq *= kMaxStep / step;
        }
        for (int k = 0; k < 3; ++k) {
          q[kFree[k]] = clamp_free(kFree[k], q[kFree[k]] + dq(k));
        }
      }

      if (!cameraPose(q, R_cam_in_tip, t_cam_in_tip, cam, R_cam, error)) {
        return false;
      }
      const tf2::Vector3 u_end = (p_target - cam).normalized();
      const double cos_aim =
        std::min(1.0, std::max(-1.0, R_cam.getColumn(2).dot(u_end)));

      // Singularity metric over the WHOLE 6-joint chain, not just the free
      // ones: Servo scales on the full Jacobian and the pinned joints still
      // shape it.
      KDL::JntArray jq(6);
      for (std::size_t i = 0; i < 6; ++i) {
        jq(static_cast<unsigned int>(i)) = q[i];
      }
      KDL::Jacobian kdl_jac(6);
      if (impl_->jac->JntToJac(jq, kdl_jac) < 0) {
        error = "posture family: Jacobian failed";
        return false;
      }
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(kdl_jac.data);
      const auto sv = svd.singularValues();
      const double smallest = sv(sv.size() - 1);

      out.q = q;
      out.aim = std::acos(cos_aim);
      out.cond = smallest > 1e-12 ? sv(0) / smallest :
        std::numeric_limits<double>::infinity();
      return true;
    };

  Attempt best;
  bool have_best = false;

  Attempt first;
  if (!run(seed, first)) {
    return false;
  }
  best = first;
  have_best = true;

  for (double j4_start : kExtraJ4Starts) {
    if (best.aim < params.aim_tol_rad) {
      break;                      // the caller's seed already centred it
    }
    std::array<double, 6> alt = seed;
    alt[3] = j4_start;
    Attempt attempt;
    if (!run(alt, attempt)) {
      return false;
    }
    if (attempt.aim < best.aim) {
      best = attempt;
    }
  }

  if (!have_best) {
    error = "posture family: no solution attempt produced a pose";
    return false;
  }

  q_out = best.q;
  aim_error_rad = best.aim;
  cond = best.cond;
  return true;
}

PostureResult PostureFamily::compute(
  const tf2::Vector3 & p_target,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  const PostureParams & params)
{
  if (std::hypot(p_target.x(), p_target.y()) < 1e-3) {
    throw std::runtime_error(
            "posture family: target on the base z axis, look direction "
            "undefined");
  }

  // Candidate order is a fixed priority list, not an optimum search: the
  // previous j2 first (continuity AND a good warm start), then nominal, then
  // outward, then the bounds. Same input always yields the same pose.
  std::vector<double> candidates;
  const auto push_candidate = [&](double j2) {
      if (j2 < params.posture_j2_min_rad || j2 > params.posture_j2_max_rad) {
        return;
      }
      for (double existing : candidates) {
        if (std::abs(existing - j2) < 1e-6) {
          return;
        }
      }
      candidates.push_back(j2);
    };

  const double previous_j2 = have_previous_ ?
    previous_.joints[1] : params.posture_j2_nominal_rad;
  push_candidate(previous_j2);
  push_candidate(params.posture_j2_nominal_rad);
  // 0.15 rad is also the per-tick j2 change limit: a wider candidate would
  // let one tick move the arm further than the continuity contract allows.
  for (double delta : {0.15, -0.15}) {
    push_candidate(previous_j2 + delta);
  }
  push_candidate(params.posture_j2_min_rad);
  push_candidate(params.posture_j2_max_rad);
  // Uniform grid after the priority list. Six candidates cover the common
  // cases; the grid is what reaches the remaining poses (2026-11-05).
  if (params.posture_j2_step_rad > 1e-6) {
    for (double j2 = params.posture_j2_min_rad;
      j2 <= params.posture_j2_max_rad + 1e-9;
      j2 += params.posture_j2_step_rad)
    {
      push_candidate(std::min(j2, params.posture_j2_max_rad));
    }
  }

  // Posture profiles, primary first. The primary gets the whole candidate
  // list before the fallback runs, so a target both can solve keeps the
  // primary's much better conditioning. The fallback is a full (j3, j5) pair:
  // its elbow fold is what reaches the far lateral bearings.
  struct Profile
  {
    double j3;
    double j5;
  };
  std::vector<Profile> profiles;
  const auto add_profile = [&profiles](double j3, double j5) {
      for (const Profile & p : profiles) {
        if (std::abs(p.j3 - j3) < 1e-9 && std::abs(p.j5 - j5) < 1e-9) {
          return;
        }
      }
      profiles.push_back({j3, j5});
    };
  add_profile(params.posture_j3_rad, params.posture_j5_rad);
  add_profile(params.posture_j3_alt_rad, params.posture_j5_alt_rad);
  // Hysteresis: keep solving with the profile the previous tick used. A
  // switch is a wrist-branch change and the tcp setpoint jumps, so it must
  // happen once per boundary crossing, not every tick around it.
  if (have_previous_) {
    for (std::size_t i = 0; i < profiles.size(); ++i) {
      if (std::abs(previous_.joints[2] - profiles[i].j3) < 1e-9 &&
        std::abs(previous_.joints[4] - profiles[i].j5) < 1e-9)
      {
        std::rotate(
          profiles.begin(), profiles.begin() + static_cast<std::ptrdiff_t>(i),
          profiles.begin() + static_cast<std::ptrdiff_t>(i) + 1);
        break;
      }
    }
  }

  PostureResult best_legal;      // geometry-safe but short of the aim tolerance
  bool have_legal = false;
  bool any_guard_reject = false;
  bool budget_exceeded = false;
  std::string last_error;
  const auto solve_start = std::chrono::steady_clock::now();

  // Each profile gets an equal share of the solve budget. Without this the
  // primary's wider search could consume the whole budget and starve the
  // fallback, which is exactly the case that needs it (budget-exhausted
  // targets used to return out_of_envelope instead of the alt solution).
  const double share_us = params.max_solve_us / static_cast<double>(profiles.size());
  for (const Profile & profile : profiles) {
  const auto profile_start = std::chrono::steady_clock::now();
  for (double j2 : candidates) {
    if (have_previous_ && params.max_solve_us >= 0.0) {
      const double total_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - solve_start).count();
      const double profile_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - profile_start).count();
      if (total_us > params.max_solve_us) {
        budget_exceeded = true;
        break;
      }
      if (profile_us > share_us) {
        break;      // hand the rest of the budget to the next profile
      }
    }
    std::array<double, 6> seed{};
    if (have_previous_) {
      seed = previous_.joints;
    }
    seed[1] = j2;
    seed[2] = profile.j3;
    seed[4] = profile.j5;

    // Seed j1 from the target bearing. The camera's optical axis sits at a
    // large, posture-dependent azimuth offset from the arm's own +x (measured
    // ~78 deg at the nominal posture), so starting j1 at 0 leaves the DLS to
    // travel more than its whole step budget to reach a sideways target — it
    // then silently "fails" on every target past ~+-60 deg. One FK at j1 = 0
    // measures the offset and the seed lands within a few tenths of a rad.
    {
      std::array<double, 6> probe = seed;
      probe[0] = 0.0;
      tf2::Vector3 p0;
      tf2::Matrix3x3 R0;
      std::string probe_error;
      if (cameraPose(probe, R_cam_in_tip, t_cam_in_tip, p0, R0, probe_error)) {
        const tf2::Vector3 axis0 = R0.getColumn(2);
        const double phi0 = std::atan2(axis0.y(), axis0.x());
        const double to_target = std::atan2(
          p_target.y() - p0.y(), p_target.x() - p0.x());
        double j1 = to_target - phi0;
        j1 = std::atan2(std::sin(j1), std::cos(j1));   // wrap to [-pi, pi]
        seed[0] = j1;
      }
    }

    std::array<double, 6> q{};
    double aim = 0.0;
    double cond = 0.0;
    std::string error;
    if (!solveFree(
        p_target, R_cam_in_tip, t_cam_in_tip, seed, params, q, aim, cond, error))
    {
      last_error = error;
      continue;
    }

    tf2::Vector3 cam;
    tf2::Matrix3x3 R_cam;
    if (!cameraPose(q, R_cam_in_tip, t_cam_in_tip, cam, R_cam, error)) {
      last_error = error;
      continue;
    }

    const double cam_radius = std::hypot(cam.x(), cam.y());
    // Judged on the TARGET's azimuth in the arm base frame, not on the
    // candidate's j1: the sector is a property of what we are looking at, and
    // every candidate for the same target must fall on the same side of it.
    // Same taper as the QP's radius_floor: full inside the sector, linear to
    // zero across the taper, nothing beyond.
    const double target_azimuth =
      std::abs(std::atan2(p_target.y(), p_target.x()));
    double sector_f = 1.0;
    if (params.cam_frontal_sector_rad > 0.0) {
      sector_f = 0.0;
      if (target_azimuth <= params.cam_frontal_sector_rad) {
        sector_f = 1.0;
      } else {
        const double taper = std::max(0.0, params.cam_frontal_taper_rad);
        if (taper > 0.0 &&
          target_azimuth < params.cam_frontal_sector_rad + taper)
        {
          sector_f = 1.0 - (target_azimuth -
            params.cam_frontal_sector_rad) / taper;
        }
      }
    }
    double distance_f = 1.0;
    if (params.cam_floor_distance_far_m > params.cam_floor_distance_near_m) {
      const double d = std::hypot(p_target.x(), p_target.y());
      if (d <= params.cam_floor_distance_near_m) {
        distance_f = 1.0;
      } else if (d >= params.cam_floor_distance_far_m) {
        distance_f = 0.0;
      } else {
        distance_f = 1.0 - (d - params.cam_floor_distance_near_m) /
          (params.cam_floor_distance_far_m - params.cam_floor_distance_near_m);
      }
    }
    const double radius_floor =
      params.cam_min_radius_m * sector_f * distance_f;
    // 2026-11-19: 硬约束 —— 画面不许倒转。roll 项只是软权重, 关节受限或
    // 接近竖直时仍可能收敛到"绕光轴滚 180 度"的解; 这里按同一参照
    // (x_des = z x u) 硬判一次, 倒转的候选直接跳过。
    const bool upright = camera_upright(cam, p_target, R_cam);
    const bool geometry_ok =
      cam.z() >= params.cam_min_height_m &&
      cam_radius <= params.cam_max_radius_m &&
      cam_radius >= radius_floor &&
      upright;
    if (!geometry_ok) {
      any_guard_reject = true;
      continue;
    }

    PostureResult r;
    r.joints = q;
    r.aim_error_rad = aim;
    r.cond = cond;
    r.guarded = any_guard_reject;
    // tcp = camera with the hand-eye transform removed.
    const tf2::Matrix3x3 R_tip = R_cam * R_cam_in_tip.transpose();
    r.tcp_desired.R = R_tip;
    r.tcp_desired.p = cam - R_tip * t_cam_in_tip;

    if (aim <= params.aim_tol_rad && cond <= params.cond_max) {
      r.ok = true;
      r.status = "ok";
      previous_ = r;
      have_previous_ = true;
      return r;
    }

    if (cond <= params.cond_max &&
      (!have_legal || aim < best_legal.aim_error_rad))
    {
      best_legal = r;
      have_legal = true;
    }
  }
  if (budget_exceeded) {
    break;      // the whole budget is gone; no profile may start another search
  }
  }

  PostureResult out;
  out.tcp_desired.p.setValue(0.0, 0.0, 0.0);   // never leave a Pose6 unset
  out.tcp_desired.R.setIdentity();
  if (have_legal) {
    out = best_legal;
    out.ok = false;
    out.out_of_envelope = true;
    out.guarded = any_guard_reject;
    out.status = "out_of_envelope";
    // Not cached as previous_: it is geometry-legal but does not centre the
    // target, and seeding the next tick from it would drag the arm around.
    return out;
  }

  if (have_previous_) {
    // Budget spent or no geometry-legal candidate: hold the last good pose.
    // The guards on that pose still hold by construction.
    out = previous_;
    out.ok = false;
    out.out_of_envelope = !budget_exceeded;
    out.guarded = false;
    out.status = budget_exceeded ? "overrun" : "held_last";
    return out;
  }

  // Nothing usable and no previous pose. Returning a default-constructed
  // PostureResult here would hand the control law an uninitialised pose
  // (tf2's default constructors do not initialise), so throw: the node maps
  // this to HOLD and never sends a pose it did not compute.
  throw std::runtime_error(
          "posture family: no legal pose (" +
          (last_error.empty() ? std::string("every candidate rejected") :
          last_error) + ")");
}

bool PostureFamily::cond(
  const std::array<double, 6> & joints,
  double & cond, std::string & error) const
{
  if (!impl_->ready) {
    error = "posture family: cond() before initialize()";
    return false;
  }
  KDL::JntArray jq(6);
  for (std::size_t i = 0; i < 6; ++i) {
    jq(static_cast<unsigned int>(i)) = joints[i];
  }
  KDL::Jacobian kdl_jac(6);
  if (impl_->jac->JntToJac(jq, kdl_jac) < 0) {
    error = "posture family: Jacobian failed";
    return false;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(kdl_jac.data);
  const auto sv = svd.singularValues();
  const double smallest = sv(sv.size() - 1);
  cond = smallest > 1e-12 ? sv(0) / smallest :
    std::numeric_limits<double>::infinity();
  return true;
}

bool PostureFamily::tcpTwist(
  const std::array<double, 6> & joints,
  const std::array<double, 6> & joint_vel,
  tf2::Vector3 & v, tf2::Vector3 & w, std::string & error) const
{
  if (!impl_->ready) {
    error = "posture family: tcpTwist before initialize()";
    return false;
  }
  KDL::JntArray jq(6);
  KDL::JntArray jdq(6);
  for (std::size_t i = 0; i < 6; ++i) {
    jq(static_cast<unsigned int>(i)) = joints[i];
    jdq(static_cast<unsigned int>(i)) = joint_vel[i];
  }
  KDL::Jacobian kdl_jac(6);
  if (impl_->jac->JntToJac(jq, kdl_jac) < 0) {
    error = "posture family: Jacobian failed";
    return false;
  }
  // KDL stacks linear first, then angular, expressed in the chain root
  // (base_link) at the tip link.
  const Eigen::Matrix<double, Eigen::Dynamic, 1> tw = kdl_jac.data * Eigen::Map<
    const Eigen::Matrix<double, 6, 1>>(jdq.data.data());
  v.setValue(tw(0), tw(1), tw(2));
  w.setValue(tw(3), tw(4), tw(5));
  return true;
}

PostureResult PostureFamily::solve(
  const tf2::Vector3 & p_target,
  const tf2::Matrix3x3 & R_cam_in_tip,
  const tf2::Vector3 & t_cam_in_tip,
  const PostureParams & params)
{
  return compute(p_target, R_cam_in_tip, t_cam_in_tip, params);
}

}  // namespace lgw_bt_planner
