#pragma once

// pose_error_gate.hpp — "did the arm actually get there?" contracts for the
// two arm-motion BT nodes.
//
// Why this exists (2026-11-17): ros2_control's arm_controller runs on
// FakeSystem, so a MoveGroup action reports SUCCEEDED as soon as the
// trajectory is handed to the controller — even when the real arm never
// moves or stops short. A full UnloadHandle run reported SUCCESS with
// ||pos_err|| = 0.29…0.37 m on every segment because the node only logged
// the error and always returned SUCCESS. A later run parked the arm with
// joint4 stalled 0.41 rad away from home and the tree still said SUCCESS,
// because MoveArmToNamed never verified anything at all.
//
// Keeping the predicates here (instead of inline in the action nodes) makes
// the semantics unit-testable without a BT.ROS2 / MoveIt harness.

#include <cmath>

namespace lgw_bt_planner
{

/// True when an error is inside its tolerance.
///
/// `max <= 0` disables the check entirely (legacy behaviour, for trees that
/// run without agx_arm_ctrl and therefore have no real feedback to compare
/// against). NaN in `err` is treated as a failure.
inline bool error_within_tolerance(double err, double max_err)
{
  return max_err <= 0.0 || err <= max_err;
}

/// Position-only gate, kept as the named entry point used by older callers.
inline bool pose_error_within_tolerance(double pos_err_m, double max_pos_err_m)
{
  return error_within_tolerance(pos_err_m, max_pos_err_m);
}

/// Angle (rad, always >= 0) between two unit quaternions given as xyzw.
///
/// q and -q denote the same rotation, so the dot product is taken as |q_a·q_b|
/// before the famous 2·acos(|dot|) formula. Clamped for |dot| slightly > 1
/// from floating-point round-off.
inline double rotation_angle_error_rad(const double qa[4], const double qb[4])
{
  double dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  dot = std::fabs(dot);
  if (dot > 1.0) {
    dot = 1.0;
  }
  return 2.0 * std::acos(dot);
}

/// Full pose gate: both position and orientation must be inside tolerance.
/// The two tolerances are independent — pass <= 0 to disable one of them.
inline bool pose_within_tolerance(
  double pos_err_m, double rot_err_rad,
  double max_pos_err_m, double max_rot_err_rad)
{
  return error_within_tolerance(pos_err_m, max_pos_err_m) &&
         error_within_tolerance(rot_err_rad, max_rot_err_rad);
}

}  // namespace lgw_bt_planner
