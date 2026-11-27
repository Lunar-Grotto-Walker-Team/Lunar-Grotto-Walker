// src/preposition_decision.cpp — see the header for the contract.

#include "lgw_bt_planner/preposition_decision.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>

#include "lgw_bt_planner/piper_limits.hpp"

namespace lgw_bt_planner
{

namespace
{

// Rotation angle of R_a^T * R_b, in [0, pi].
double rotation_angle(const tf2::Matrix3x3 & a, const tf2::Matrix3x3 & b)
{
  const tf2::Matrix3x3 delta = a.transpose() * b;
  const double trace = delta[0][0] + delta[1][1] + delta[2][2];
  return std::acos(std::min(1.0, std::max(-1.0, (trace - 1.0) * 0.5)));
}

}  // namespace

PrepositionDecision decide_preposition(
  const std::array<double, 6> & q_family,
  const std::array<double, 6> & q_now,
  const Pose6 & tcp_family,
  const Pose6 & tcp_now,
  const PrepositionParams & params)
{
  PrepositionDecision d;

  // Measurements first: the caller logs them either way, and a verdict
  // without its numbers is not actionable.
  for (std::size_t i = 0; i < 6; ++i) {
    d.max_joint_delta = std::max(d.max_joint_delta, std::fabs(q_now[i] - q_family[i]));
  }
  d.q5_delta = std::fabs(q_now[4] - q_family[4]);

  d.min_limit_margin = 1e9;
  for (std::size_t i = 0; i < 6; ++i) {
    d.min_limit_margin = std::min(
      d.min_limit_margin,
      std::min(q_now[i] - kPiperLower[i], kPiperUpper[i] - q_now[i]));
  }

  d.tcp_distance = (tcp_family.p - tcp_now.p).length();
  d.tcp_angle = rotation_angle(tcp_family.R, tcp_now.R);

  if (!params.enable) {
    d.reason = "disabled";
    return d;
  }

  // Evaluation order is by danger, not by the order the criteria are listed in
  // the spec: the wrist branch is what makes Servo integrate through a
  // singularity, so it is checked and reported first.
  //
  // "Other branch" is decided by which SIDE of q5 = 0 the arm is on, not by how
  // far q5 moved: -0.4 -> +0.6 is only 1.0 rad but crosses the singularity,
  // while +0.05 -> +0.6 is 0.55 rad and crosses nothing. `branch_delta_rad`
  // stays as a secondary "same side but implausibly far".
  const bool crosses_singularity =
    (q_now[4] < 0.0 && q_family[4] > 0.0) ||
    (q_now[4] > 0.0 && q_family[4] < 0.0);
  if (crosses_singularity || d.q5_delta > params.branch_delta_rad) {
    d.needed = true;
    d.reason = "branch";
  } else if (d.max_joint_delta > params.joint_delta_rad) {
    d.needed = true;
    d.reason = "joint_delta";
  } else if (d.min_limit_margin < params.limit_margin_rad) {
    d.needed = true;
    d.reason = "limit";
  } else if (d.tcp_distance > params.tcp_distance_m ||
    d.tcp_angle > params.tcp_angle_rad)
  {
    d.needed = true;
    d.reason = "tcp";
  }
  return d;
}

}  // namespace lgw_bt_planner
