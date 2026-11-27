// include/lgw_bt_planner/solve_task.hpp
//
// Task abstraction for the QP IK engine (2026-11-19).
//
// Background: QpPostureSolver's SQP loop (finite-difference Jacobian ->
// qpmad QP -> step -> re-linearise) is task-agnostic. Only three things are
// task specific:
//
//   1) the soft objective  e(q)                      -> residual()
//   2) extra hard rows     lb <= A*dq <= ub          -> add_constraints()
//   3) the acceptance test applied to a converged q  -> accept()
//
// The look-at (servo posture) task and the pose task (grasp / pre-grasp
// generation) differ in ALL THREE, so they each provide their own
// implementation; the engine below is shared.
//
// Deliberately additive: QpPostureSolver::solve() (the look-at path the Servo
// uses every tick) is NOT routed through this interface yet. It keeps its own
// implementation until the pose path has proven itself on the robot; the
// existing look-at test suites are the regression gate for that later merge.

#ifndef LGW_BT_PLANNER__SOLVE_TASK_HPP_
#define LGW_BT_PLANNER__SOLVE_TASK_HPP_

#include <array>
#include <string>

#include <Eigen/Dense>

namespace lgw_bt_planner
{

using JointArray = std::array<double, 6>;

// Metrics the engine computes for a candidate. Tasks use them in their own
// accept(); callers (grasp candidate scoring) use them to rank solutions.
struct SolveMetrics
{
  double residual_norm{0.0};   // ||e(q)||, in the task's normalised units
  double joint_margin{0.0};    // min distance to a joint limit, rad
  double cond{0.0};            // chain condition number at q
  int iterations{0};           // linearisations actually run
  double solve_us{0.0};        // wall clock of the whole solve
};

class SolveTask
{
public:
  virtual ~SolveTask() = default;

  // 1) Soft objective. Dimension is task-defined; the engine only needs it to
  // size the Jacobian.
  virtual Eigen::VectorXd residual(const JointArray & q) const = 0;

  // 2) Extra hard rows, appended to the joint-limit/trust-region box:
  //      lb <= A * dq <= ub
  // Called once per linearisation with the current q. Default: no extra rows
  // (the look-at task's camera envelope lives here; the grasp task's TCP
  // height / approach-angle / reach rows will live here).
  virtual void add_constraints(
    const JointArray & /*q*/,
    Eigen::MatrixXd & /*A*/, Eigen::VectorXd & /*lb*/,
    Eigen::VectorXd & /*ub*/) const {}

  // 3) Acceptance of the converged solution. `why` is filled on rejection.
  virtual bool accept(
    const JointArray & q, const SolveMetrics & m, std::string & why) const = 0;
};

struct TaskSolveParams
{
  double w_task{1.0};
  double w_smooth{0.01};
  double w_posture{0.001};
  int max_linearizations{16};
  // Keeps each linearisation valid. 0.30 rad x 16 passes = 4.8 rad of travel,
  // enough for a seed that is far away (the servo uses 0.20 x 8 because it
  // re-solves every tick).
  double trust_region_rad{0.30};
  double joint_margin_rad{0.20};
  double max_solve_us{8000.0};
  double converge_eps{2e-3};   // ||e|| below this counts as converged
};

struct TaskSolveResult
{
  bool ok{false};
  // "ok" | "not_initialized" | "fk_failed" | "overrun" | "no_converge" |
  // "rejected:<why>"
  std::string status;
  JointArray q{};
  SolveMetrics metrics{};
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SOLVE_TASK_HPP_
