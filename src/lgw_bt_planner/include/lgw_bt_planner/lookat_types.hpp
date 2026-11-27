// include/lgw_bt_planner/lookat_types.hpp
//
// Value types shared by every lookat solver implementation (DLS posture
// family, QP IK). Split out of lookat_posture_family.hpp so the solver
// interface can name them without pulling in the DLS class.

#ifndef LGW_BT_PLANNER__LOOKAT_TYPES_HPP_
#define LGW_BT_PLANNER__LOOKAT_TYPES_HPP_

#include <array>
#include <string>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Tuning for the pinned-posture family. Defaults are the spec §4 values —
// keep them in sync with config/servo_node.yaml.
struct PostureParams
{
  // Two pinned wrist profiles (2026-11-05, measured with
  // scripts/lookat_axis_coverage.py and scripts/lookat_j3j5_sweep.py):
  //   primary  (-0.80) looks forward/level and up to +45 deg, cond 13-19
  //            across 0.5-1.6 m; it CANNOT centre targets closer than ~0.6 m
  //            at or below body height.
  //   alt      (+0.60) centres those close/low targets; it blows the
  //            condition number out at far targets (65-575 at 1.0-1.6 m).
  // The two failure regions do not overlap: trying the primary first and
  // falling back to the alt covers the whole tested envelope (0 misses over
  // 32 distance/height targets).
  double posture_j3_rad{-0.720};        // primary elbow fold
  double posture_j5_rad{-0.800};        // primary wrist bend (forward/level/up)
  // Fallback keeps the field-proven 2026-10-30 posture.
  double posture_j3_alt_rad{-0.484};    // fallback elbow fold
  double posture_j5_alt_rad{0.600};     // fallback wrist bend (close/low, wide az)
  // Uniform j2 candidate grid step (rad). 0 disables the grid.
  double posture_j2_step_rad{0.150};
  double posture_j2_nominal_rad{0.890};
  double posture_j2_min_rad{0.150};     // URDF 0 + 0.15 (Servo halts at +0.10)
  double posture_j2_max_rad{1.571};     // 90 deg; more buys nothing (camera sinks)
  double cam_min_height_m{0.200};       // collision / platform-clearance floor
  // 0.40, not 0.35: raising the ceiling fixed 6 of 10 out-of-envelope cases
  // over the 0.35-1.6 m grid and LOWERED the worst condition number.
  double cam_max_radius_m{0.400};       // reach / centre-of-gravity ceiling
  // Floor on the horizontal camera radius, applied ONLY inside the frontal
  // sector below (2026-11-09). Looking forward the camera should stand back
  // and frame the target; looking to the side it may come closer. 0 disables.
  double cam_min_radius_m{0.0};
  // Half-width of the frontal sector, radians, measured as the TARGET's
  // azimuth in the arm base frame: inside |atan2(y, x)| <= this,
  // cam_min_radius_m applies; outside it there is no floor. 0 means "apply
  // everywhere". 45 deg = 0.7854, i.e. a 90 deg cone in front.
  //
  // Judging the target (not the solved j1) keeps the rule constant across the
  // SQP iterations, so the linearised bound and the envelope check cannot
  // disagree. The bound is still discontinuous at the sector edge: a target
  // sitting on it can flip between ticks as the platform rotates.
  double cam_frontal_sector_rad{0.0};
  // Width of the taper OUTSIDE the sector, radians. With a hard edge a target
  // crossing the boundary flips the floor between 0 and full, and the arm
  // re-solves into a different posture. With a taper the floor ramps linearly
  // from full, at the sector edge, down to zero at sector + taper.
  // 0 keeps the hard edge.
  double cam_frontal_taper_rad{0.0};
  // Inside the frontal sector the floor also fades with TARGET DISTANCE:
  // full at cam_floor_distance_near_m, zero at cam_floor_distance_far_m. A
  // far target does not need the camera to stand back the way a near one
  // does, and holding it there costs coverage. Set far <= near to disable.
  double cam_floor_distance_near_m{0.50};
  double cam_floor_distance_far_m{1.00};
  double joint_margin{0.150};           // 0.05 rad inside Servo's 0.10 halt line
  double cond_max{50.0};                // = servo_singularity_lower
  double aim_tol_rad{0.035};            // ~2 deg counts as centred
  double roll_weight{0.25};             // image-level term is secondary to aim
  // Wall-clock budget for one solve. Exceeding it returns the previous pose
  // flagged "overrun" rather than blocking the control tick; the first tick
  // (no previous pose yet) always runs to completion.
  double max_solve_us{4000.0};
};

// One solve's outcome. `status` is a stable string for logs and tests:
//   "ok"              solved at the first candidate that passed every guard
//   "out_of_envelope" no candidate centred the target; best-effort pose
//   "overrun"         solve budget exhausted; previous pose held
//   "degenerate"      no well-defined look direction (caller should HOLD)
struct PostureResult
{
  bool ok{false};
  bool out_of_envelope{false};
  bool guarded{false};
  Pose6 tcp_desired;
  std::array<double, 6> joints{};
  double aim_error_rad{0.0};
  double cond{0.0};
  // True when this call moved the wrist to the other side of |j5| (the QP
  // branch). The node feeds such a change LIVE -- routing it through a
  // planned move was measured worse on a moving target (2026-11-05: ~3 s
  // moves landing behind the target, 14 moves vs 3 live stops in 120 s). A
  // hard stop at the crossing is caught by the HOLD recovery instead. The
  // flag exists so the rate can be monitored.
  bool branch_switched{false};
  std::string status{"degenerate"};
  // Diagnostics only (2026-11-07: close-range wrist swing). Which J5 branch
  // the winning run used (+1 / -1), which run_tier() slot produced it
  // (0 = warm/preferred, 1 = warm/other, 2 = cold/preferred, 3 = cold/other),
  // and whether the relaxed cond tier had to be entered. No control logic
  // reads these; they exist so the node can trace WHY a branch was taken.
  int branch{0};
  int run_index{-1};
  bool relaxed{false};
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_TYPES_HPP_
