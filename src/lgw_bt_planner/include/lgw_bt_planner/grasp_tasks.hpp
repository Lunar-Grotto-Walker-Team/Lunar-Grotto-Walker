// include/lgw_bt_planner/grasp_tasks.hpp
//
// Two SolveTask implementations for the collect mission (2026-11-19).
//
// The user's constraint spec (all "vertical" = the arm base_link z axis; the
// body is no longer tilted in later versions):
//
//   pre-grasp:  the arm camera looks at the target, the viewing direction as
//               close to vertical as possible, the image is upright, and the
//               camera-target range stays at d_ref (0.35 m).  The POSITION is
//               free - the solver picks where the tcp ends up.
//   grasp:      the tcp sits on the target centre, the tool axis as close to
//               vertical as possible, the image upright.  "As vertical as
//               possible" is a soft term with a hard cap (max_tilt_rad), so a
//               target that cannot be reached straight down still gets the
//               least-tilted feasible pose.
//
// Hard constraints: joint limits with margin (the engine's box) only.  The
// user explicitly removed the table-clearance and reach-radius rows: whatever
// the solver returns is handed to MoveArmToPose, and "the pose must be
// solvable" is the only requirement.
//
// Both tasks hand back a Cartesian tcp pose, so the BT node keeps using
// MoveArmToPose; solve_task() doubles as the feasibility pre-check that keeps
// impossible poses away from move_group (the -27 class).

#ifndef LGW_BT_PLANNER__GRASP_TASKS_HPP_
#define LGW_BT_PLANNER__GRASP_TASKS_HPP_

#include <functional>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/solve_task.hpp"

namespace lgw_bt_planner
{

// FK callback: tip (tcp) pose and camera optical pose for one joint vector.
// The caller builds it from the solver (tipPose + cameraPose); tests can pass
// a fake.
using PoseFkFn = std::function<bool(
    const JointArray & q,
    tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
    tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
    std::string & err)>;

struct PreGraspTaskParams
{
  double d_ref_m{0.35};          // camera -> target range
  double tol_aim{0.03};          // rad, chord error normalisation
  double tol_vert{0.05};         // normalisation of (1 - verticality)
  double tol_roll{0.05};
  double tol_standoff{0.02};     // m
  double w_aim{10.0};            // must look at the target
  double w_standoff{5.0};        // keep the D435 in its good range
  double w_vert{1.0};            // as vertical as possible
  double w_roll{0.5};            // upright image
  // Acceptance
  double accept_aim_rad{0.05};         // ~3 deg
  double accept_standoff_m{0.04};
  double accept_max_cam_tilt_rad{0.87}; // 50 deg from vertical
  double min_joint_margin{0.15};
};

struct GraspTaskParams
{
  double tol_pos_m{0.01};
  double tol_axis{0.03};
  double tol_roll{0.05};
  double w_pos{10.0};            // tcp on the target centre
  double w_axis{1.0};            // tool axis vertical
  double w_roll{0.5};            // upright image
  // Acceptance
  double accept_pos_m{0.005};    // hard: centre error below 5 mm
  double max_tilt_rad{0.61};     // hard cap: 35 deg from vertical
  double min_joint_margin{0.15};
};

class PreGraspTask : public SolveTask
{
public:
  PreGraspTask(
    PoseFkFn fk, const tf2::Vector3 & p_target,
    const PreGraspTaskParams & params);

  Eigen::VectorXd residual(const JointArray & q) const override;
  bool accept(
    const JointArray & q, const SolveMetrics & m,
    std::string & why) const override;

  // tcp pose of a solved joint vector (what the caller writes to the
  // blackboard for MoveArmToPose).
  bool tcpPose(
    const JointArray & q, tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
    std::string & err) const;

  const PreGraspTaskParams & params() const {return params_;}

private:
  PoseFkFn fk_;
  tf2::Vector3 p_target_;
  PreGraspTaskParams params_;
};

class GraspTask : public SolveTask
{
public:
  GraspTask(
    PoseFkFn fk, const tf2::Vector3 & p_target,
    const GraspTaskParams & params);

  Eigen::VectorXd residual(const JointArray & q) const override;
  bool accept(
    const JointArray & q, const SolveMetrics & m,
    std::string & why) const override;

  bool tcpPose(
    const JointArray & q, tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
    std::string & err) const;

  // Tool-axis tilt from vertical at `q`, radians. Used by the caller to rank
  // candidates and by tests.
  bool tiltRad(const JointArray & q, double & tilt, std::string & err) const;

  const GraspTaskParams & params() const {return params_;}

private:
  PoseFkFn fk_;
  tf2::Vector3 p_target_;
  GraspTaskParams params_;
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__GRASP_TASKS_HPP_
