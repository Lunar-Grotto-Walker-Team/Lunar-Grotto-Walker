// include/lgw_bt_planner/lookat_qp_solver.hpp
//
// QP lookat solver (spec 2026-11-05). Solves, once per control tick:
//
//   min  w_aim*||J_task*dq + e||^2 + w_smooth*||dq||^2
//      + w_posture*||(q_prev + dq) - q_ref||^2
//   s.t. joint limits with margin,  |dq| <= step_max
//        (Task 4 adds the linearised camera height/radius constraints)
//
// The task error e has the same 4 components and weights as the DLS family
// (optical-axis misalignment + weighted image roll), but the axis term is
// the chord u - R_z, not the cross product: the cross product is also zero
// at 180 deg, which turns "looking away from the target" into a global
// minimum of the cost. J is e's finite-difference Jacobian over all six
// joints. qpmad solves the strictly convex QP (w_smooth > 0 makes the
// Hessian positive definite).
//
// Safety is NOT delegated to the QP: after every solve the result is FK'd
// and checked against the same guards the DLS family applies (aim, cond,
// camera envelope), with one re-linearisation before giving up.

#ifndef LGW_BT_PLANNER__LOOKAT_QP_SOLVER_HPP_
#define LGW_BT_PLANNER__LOOKAT_QP_SOLVER_HPP_

#include <array>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/lookat_solver.hpp"
#include "lgw_bt_planner/lookat_types.hpp"
#include "lgw_bt_planner/solve_task.hpp"

namespace lgw_bt_planner
{

struct QpSolverParams
{
  double w_aim{1.0};
  double w_smooth{0.01};
  double w_posture{0.001};
  // SQP passes: each re-linearises around the previous solution. The solver
  // answers an absolute pose (no motion budget — Servo owns the speed), so
  // this only bounds convergence effort.
  int max_linearizations{8};
  // Wrist branch: |j5| must stay at least this far from the straight-wrist
  // singularity. 0.50 since 2026-11-09: the offline sweep shows the grid keeps
  // its coverage while the wrist uses the extra 0.10 rad on close targets.
  // Below 0.45 the conditioning margin disappears; below 0.30 the band is
  // genuinely singular (cond 65-360 by the family's own kinematics), so the
  // room being reclaimed is 0.50-0.60, not the whole 60 degrees.
  double j5_branch_min_rad{0.50};
  // Boundary targets (|az| >= 155 deg) only exist at cond 57-68. The strict
  // pass keeps every pose it can solve under PostureParams::cond_max; this
  // second pass runs ONLY when all strict runs fail, so nothing that already
  // worked is replaced by a worse-conditioned pose.
  double cond_max_relaxed{70.0};
  // The relaxed tier is only for rear bearings (their only IK solution sits
  // at cond 57-68). Firing it in the mission range would put the pose in
  // Servo's deceleration band on the first call. Radians; <= 0 disables.
  double relaxed_min_azimuth_rad{2.4434609528};   // 140 deg
  // Joint-limit margin used by the QP. Servo halts at URDF_limit - 0.10, so
  // the 0.15 shared with the DLS left only 0.05 rad for the tracking lag and
  // Servo halted whenever a setpoint rode the limit (real machine
  // 2026-11-06). 0.20 keeps 0.10 rad of slack and measured 99.2% coverage.
  double joint_margin_rad{0.20};
  // Upper bound on |j5| inside a branch. Without it the solver can park the
  // wrist at the joint margin (|j5| ~ 1.07); 0.95 keeps it off the limit.
  double j5_branch_max_rad{0.95};
  // Radius-vs-depression coupling (2026-11-09). Measured on the robot with
  // the shipped settings: the camera radius CORRELATES NEGATIVELY with how
  // steeply it looks down (corr -0.56; at 80-90 deg depression the radius
  // averages 0.216 m, i.e. the camera stands over the target). This term
  // pulls the radius toward a target that grows with depression, while
  // cam_max_radius_m stays the hard ceiling.
  //
  // 0 disables the term (shipped behaviour). The weight is quadratic in
  // metres, so it competes with w_aim (radians^2): 0.05 is a weak pull,
  // 0.3 a strong one.
  double w_radius{0.0};
  // Measure the radius as the 3-D distance to the base origin instead of the
  // horizontal xoy radius. Measured 2026-11-09: it DOES lower the camera
  // (median height 0.417 -> 0.276) but coverage collapses 97.6 % -> 82.1 %,
  // because a far target demands a camera within ~0.28 m of the base origin
  // while still aiming at it. MEASUREMENT SWITCH ONLY: it is deliberately not
  // exposed as a node parameter, and it is exercised by
  // tests/test_qp_j5_sweep.cpp. The 97.6 % -> 82.1 % figure above was taken
  // with the linearised bound using only the horizontal projection of n.cam;
  // that omission is fixed, so re-run the sweep before quoting the number.
  bool radius_3d{false};
  // Soft target radius at 0 deg depression. Lower values pull the camera in
  // for level (far) targets; it is clamped up to the hard floor, so it can
  // never pull inward past it.
  double radius_near_m{0.28};    // target radius at 0 deg depression
  double radius_steep_m{0.38};   // target radius at 90 deg depression
  // Soft target radius OUTSIDE the frontal sector (no depression ramp there:
  // looking sideways at a steep angle does not call for standing back the way
  // the frontal cone does). Blended with the frontal target across the taper,
  // so the pull stays a single continuous term.
  double radius_side_m{0.25};
  // Which branch to try first when there is no history. The rest of the time
  // the previous branch wins (hysteresis). Positive is the default: its
  // posture is the one that reaches close/low targets, which is where the
  // arm spends most of a follow.
  bool prefer_positive_branch{true};
};

class QpPostureSolver final : public LookatSolver
{
public:
  QpPostureSolver();
  ~QpPostureSolver() override;

  bool initialize(
    const std::string & urdf_path_or_xml, bool is_file,
    const std::string & base_link, const std::string & tip_link,
    std::string & error);

  void set_params(const QpSolverParams & params);

  // FK of the tip (tcp) frame. 2026-11-19: grasp/pre-grasp pose generation
  // needs the tip pose; delegation to the underlying chain.
  bool tipPose(
    const std::array<double, 6> & joints,
    tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
    std::string & error) const;

  // FK of a solved joint vector (same camera frame the DLS family returns).
  // Exposed so callers/tests can verify a result against the envelope.
  bool cameraPose(
    const std::array<double, 6> & joints,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
    std::string & error) const;

  PostureResult solve(
    const tf2::Vector3 & p_target,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    const PostureParams & params) override;

  // Generic task solve (2026-11-19): the same SQP/QP machinery (finite
  // difference Jacobian, qpmad, joint-limit box with margin, per-pass trust
  // region, wall-clock budget) with the task supplying its own residual,
  // extra hard rows and acceptance. Used by the grasp / pre-grasp pose
  // generation; the look-at path above is deliberately NOT routed through it
  // yet, so Servo behaviour is bit-identical.
  //
  // `seed` is both the starting point and the posture regulariser reference
  // (the solution stays near the pose it started from unless the task term
  // pulls harder).
  TaskSolveResult solve_task(
    const SolveTask & task, const JointArray & seed,
    const TaskSolveParams & params);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_QP_SOLVER_HPP_
