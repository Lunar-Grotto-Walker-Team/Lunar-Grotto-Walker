// src/grasp_tasks.cpp — see the header for the constraint spec.

#include "lgw_bt_planner/grasp_tasks.hpp"

#include <algorithm>
#include <cmath>

#include "lgw_bt_planner/lookat_solver.hpp"   // camera_upright()

namespace lgw_bt_planner
{

namespace
{

// "Vertical" is the arm base_link z axis. The body is no longer tilted in
// later versions; while it still is, this makes the grasp follow the body.
const tf2::Vector3 kUp(0.0, 0.0, 1.0);

double clampCos(double v)
{
  return std::max(-1.0, std::min(1.0, v));
}

// Residual used when FK fails: large, finite, and pointing "away".
Eigen::VectorXd failedResidual(int n)
{
  Eigen::VectorXd e(n);
  e.setConstant(1e3);
  return e;
}

}  // namespace

// ---------------------------------------------------------------------------
// PreGraspTask
// ---------------------------------------------------------------------------
PreGraspTask::PreGraspTask(
  PoseFkFn fk, const tf2::Vector3 & p_target,
  const PreGraspTaskParams & params)
: fk_(std::move(fk)), p_target_(p_target), params_(params)
{
}

Eigen::VectorXd PreGraspTask::residual(const JointArray & q) const
{
  tf2::Vector3 p_tip, p_cam;
  tf2::Matrix3x3 R_tip, R_cam;
  std::string err;
  if (!fk_ || !fk_(q, p_tip, R_tip, p_cam, R_cam, err)) {
    return failedResidual(6);
  }

  const tf2::Vector3 raw = p_target_ - p_cam;
  const double range = raw.length();
  if (!std::isfinite(range) || range < 1e-6) {
    return failedResidual(6);
  }
  const tf2::Vector3 u = raw / range;              // camera -> target
  const tf2::Vector3 z_cam = R_cam.getColumn(2);   // optical axis

  Eigen::VectorXd e(6);
  // 1) look at the target (chord form: zero only when aligned, no 180 deg trap)
  const tf2::Vector3 aim = u - z_cam;
  e(0) = params_.w_aim * aim.x() / params_.tol_aim;
  e(1) = params_.w_aim * aim.y() / params_.tol_aim;
  e(2) = params_.w_aim * aim.z() / params_.tol_aim;
  // 2) camera -> target range
  e(3) = params_.w_standoff * (range - params_.d_ref_m) / params_.tol_standoff;
  // 3) as vertical as possible
  e(4) = params_.w_vert * (1.0 - (-z_cam).dot(kUp)) / params_.tol_vert;
  // 4) upright image (camera x-axis horizontal)
  e(5) = params_.w_roll * R_cam.getColumn(0).dot(kUp) / params_.tol_roll;
  return e;
}

bool PreGraspTask::accept(
  const JointArray & q, const SolveMetrics & m, std::string & why) const
{
  tf2::Vector3 p_tip, p_cam;
  tf2::Matrix3x3 R_tip, R_cam;
  std::string err;
  if (!fk_ || !fk_(q, p_tip, R_tip, p_cam, R_cam, err)) {
    why = "fk";
    return false;
  }
  const tf2::Vector3 raw = p_target_ - p_cam;
  const double range = raw.length();
  if (!std::isfinite(range) || range < 1e-6) {
    why = "degenerate_range";
    return false;
  }
  const tf2::Vector3 u = raw / range;
  const tf2::Vector3 z_cam = R_cam.getColumn(2);

  const double aim = std::acos(clampCos(u.dot(z_cam)));
  if (aim > params_.accept_aim_rad) {
    why = "aim";
    return false;
  }
  if (std::fabs(range - params_.d_ref_m) > params_.accept_standoff_m) {
    why = "standoff";
    return false;
  }
  const double cam_tilt = std::acos(clampCos((-z_cam).dot(kUp)));
  if (cam_tilt > params_.accept_max_cam_tilt_rad) {
    why = "cam_tilt";
    return false;
  }
  if (!camera_upright(p_cam, p_target_, R_cam)) {
    why = "roll";
    return false;
  }
  if (m.joint_margin < params_.min_joint_margin) {
    why = "joint_margin";
    return false;
  }
  return true;
}

bool PreGraspTask::tcpPose(
  const JointArray & q, tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
  std::string & err) const
{
  tf2::Vector3 p_cam;
  tf2::Matrix3x3 R_cam;
  return fk_ && fk_(q, p_tip, R_tip, p_cam, R_cam, err);
}

// ---------------------------------------------------------------------------
// GraspTask
// ---------------------------------------------------------------------------
GraspTask::GraspTask(
  PoseFkFn fk, const tf2::Vector3 & p_target,
  const GraspTaskParams & params)
: fk_(std::move(fk)), p_target_(p_target), params_(params)
{
}

Eigen::VectorXd GraspTask::residual(const JointArray & q) const
{
  tf2::Vector3 p_tip, p_cam;
  tf2::Matrix3x3 R_tip, R_cam;
  std::string err;
  if (!fk_ || !fk_(q, p_tip, R_tip, p_cam, R_cam, err)) {
    return failedResidual(7);
  }

  const tf2::Vector3 z_tool = R_tip.getColumn(2);
  const tf2::Vector3 down(0.0, 0.0, -1.0);

  Eigen::VectorXd e(7);
  const tf2::Vector3 dp = p_target_ - p_tip;
  e(0) = params_.w_pos * dp.x() / params_.tol_pos_m;
  e(1) = params_.w_pos * dp.y() / params_.tol_pos_m;
  e(2) = params_.w_pos * dp.z() / params_.tol_pos_m;
  // Tool axis as vertical as possible (direction only; the roll about it is
  // the next term).
  const tf2::Vector3 da = z_tool - down;
  e(3) = params_.w_axis * da.x() / params_.tol_axis;
  e(4) = params_.w_axis * da.y() / params_.tol_axis;
  e(5) = params_.w_axis * da.z() / params_.tol_axis;
  // Upright image: the camera x-axis stays horizontal.
  e(6) = params_.w_roll * R_cam.getColumn(0).dot(kUp) / params_.tol_roll;
  return e;
}

bool GraspTask::tiltRad(
  const JointArray & q, double & tilt, std::string & err) const
{
  tf2::Vector3 p_tip, p_cam;
  tf2::Matrix3x3 R_tip, R_cam;
  if (!fk_ || !fk_(q, p_tip, R_tip, p_cam, R_cam, err)) {
    return false;
  }
  tilt = std::acos(clampCos(-R_tip.getColumn(2).dot(kUp)));
  return true;
}

bool GraspTask::accept(
  const JointArray & q, const SolveMetrics & m, std::string & why) const
{
  tf2::Vector3 p_tip, p_cam;
  tf2::Matrix3x3 R_tip, R_cam;
  std::string err;
  if (!fk_ || !fk_(q, p_tip, R_tip, p_cam, R_cam, err)) {
    why = "fk";
    return false;
  }

  // "tcp on the target centre" is enforced as a hard acceptance threshold:
  // soft in the optimiser (so the QP stays well posed), hard in accept() (a
  // solution that misses the centre is never handed to MoveArmToPose).
  const tf2::Vector3 dp = p_target_ - p_tip;
  if (dp.length() > params_.accept_pos_m) {
    why = "position";
    return false;
  }

  const double tilt = std::acos(clampCos(-R_tip.getColumn(2).dot(kUp)));
  if (tilt > params_.max_tilt_rad) {
    why = "tilt";
    return false;
  }

  if (!camera_upright(p_cam, p_target_, R_cam)) {
    why = "roll";
    return false;
  }

  if (m.joint_margin < params_.min_joint_margin) {
    why = "joint_margin";
    return false;
  }
  return true;
}

bool GraspTask::tcpPose(
  const JointArray & q, tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
  std::string & err) const
{
  tf2::Vector3 p_cam;
  tf2::Matrix3x3 R_cam;
  return fk_ && fk_(q, p_tip, R_tip, p_cam, R_cam, err);
}

}  // namespace lgw_bt_planner
