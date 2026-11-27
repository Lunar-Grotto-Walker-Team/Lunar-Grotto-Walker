// include/lgw_bt_planner/lookat_standoff_setpoint.hpp
//
// Target-relative lookat standoff (spec 2026-10-29, "lookat 站位重构").
//
// Why a second implementation instead of changing compute_lookat_setpoint():
// the existing one places the camera on a BASE-RELATIVE orbit
// (xoy_distance_m / z_fixed_m). That is what the BT lookat path and every
// field -tested behaviour are built on, so it stays untouched. This module is
// opt-in (servo param `lookat_mode: target_relative`) and can be dropped by
// flipping the parameter back.
//
// Three distances matter and they cannot all be pinned at once:
//
//   R = |p_target - P_base|   base -> target   (the platform moves it)
//   r = |P_cam - P_base|      base -> camera   (arm reach, MUST stay bounded)
//   d = |P_cam - p_target|    camera -> target (standoff, free within bounds)
//
// For a camera on the base->target ray, r = R - d. The legacy scheme pins r
// (xoy) and lets d float, which makes the standoff uncontrollable and turns
// xoy into a hidden reachability cliff (target closer than xoy => no IK) and
// z_fixed into a wrist-singularity knob (measured: cond(J) 24 / 88 / 29 for
// z = 0.20 / 0.25 / 0.35 at a 0.75 m target).
//
// This module instead pins r to a comfortable value by CHOOSING d, then
// projects the result into
//     { r_min <= r <= r_max }  ∩  { z >= z_min }
// so reachability and clearance are explicit, per-tick properties rather than
// side effects. When the requested point cannot be honoured, the caller gets
// the last feasible camera point (still aimed at the current target) instead
// of a bad command — the arm holds station and keeps the target in view.
//
// Orientation is the same look-at rule as the legacy path (Gram-Schmidt +
// uprightness flip) plus the optional optical-axis roll, so the target stays
// exactly centred in both modes.
//
// Pure tf2 math: no rclcpp, no ROS runtime, no Eigen.

#ifndef LGW_BT_PLANNER__LOOKAT_STANDOFF_SETPOINT_HPP_
#define LGW_BT_PLANNER__LOOKAT_STANDOFF_SETPOINT_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Why a tick did not produce the requested geometry. kOk is the only status
// that means "the requested standoff was honoured exactly".
enum class StandoffStatus
{
  kOk = 0,          // standoff and reach both as requested
  kReachClamped,    // r_req was outside [r_min, r_max]; standoff adjusted to fit
  kHeldLast,        // constraints unsatisfiable now; reusing the last feasible
                    // camera point (still aimed at the CURRENT target)
  kDegenerate,      // no usable look direction (target coincides with camera)
};

struct StandoffParams
{
  // Reach shell around the arm base. r_max must stay well below the chain's
  // real limit: measured max tcp reach over the joint limits is 0.871 m.
  double reach_pref_m{0.45};
  double reach_min_m{0.15};
  double reach_max_m{0.75};

  // Band the camera->target distance may float in. The standoff is NOT
  // driven directly: it follows from reach_pref_m (d = R - r on the
  // base->target ray), and these bounds only exist to keep the camera
  // sensible. standoff_max_m is what limits how far the target may be: once
  // d would exceed it, the camera is pushed out of the reach shell and the
  // tick degrades to hold-last. It also has to be large enough to let the
  // height floor be met for a low target (d >= cam_min_height - p_target.z).
  double standoff_min_m{0.25};
  double standoff_max_m{0.90};

  // Camera height floor in base_link (prevents the arm from dipping into the
  // platform). Set from the measured clearance under the arm mount.
  double cam_min_height_m{0.22};

  // Standoff slew limit, so a moving platform cannot make the setpoint jump.
  double standoff_rate_limit_mps{0.30};

  // Roll about the camera optical axis (same meaning as in LookatParams).
  double camera_roll_rad{0.0};

  // View direction used when the current camera position gives none
  // (target and camera coincident). Points FROM the target TO the camera.
  tf2::Vector3 fallback_view_dir{0.0, 0.0, 1.0};
};

// Carried across ticks: the last feasible camera point and standoff, plus the
// last emitted pose so a degenerate tick can repeat it instead of emitting a
// NaN rotation.
struct StandoffState
{
  bool has_last_feasible{false};
  tf2::Vector3 last_p_cam_base{0.0, 0.0, 0.0};

  bool has_last_standoff{false};
  double last_standoff_m{0.0};

  bool has_last_pose{false};
  Pose6 last_tcp_desired;
};

struct StandoffResult
{
  Pose6 tcp_desired;                 // desired tcp_link pose = Servo reference
  StandoffStatus status{StandoffStatus::kOk};
  bool used_fallback_view{false};
  double requested_standoff_m{0.0};  // d before reach clamping
  double achieved_standoff_m{0.0};   // |P_cam - p_target| actually used
  double reach_m{0.0};               // |P_cam - P_base|
};

// Compute the desired tcp_link pose for a target seen from an adaptive
// standoff. All vectors are in the arm base_link frame.
//
// p_cam_now_base is the CURRENT camera origin
// (tcp_now.p + tcp_now.R * t_cam_in_tcp): it supplies the view direction, so
// the camera keeps its bearing instead of swinging around the target when the
// platform translates. dt_s is used only for the standoff slew limit; pass 0
// to disable it.
//
// Never throws: every geometric degeneracy is reported through `status`, and a
// degenerate tick repeats the previous pose when one exists.
StandoffResult compute_standoff_setpoint(
  const tf2::Vector3& p_target_base,
  const tf2::Vector3& p_cam_now_base,
  double dt_s,
  const tf2::Matrix3x3& R_cam_in_tcp,
  const tf2::Vector3& t_cam_in_tcp,
  const StandoffParams& params,
  StandoffState& state);

// Human-readable status for logging.
const char* standoff_status_name(StandoffStatus status);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_STANDOFF_SETPOINT_HPP_
