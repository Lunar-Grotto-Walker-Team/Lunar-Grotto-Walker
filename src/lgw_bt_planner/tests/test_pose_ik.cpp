// tests/test_pose_ik.cpp
//
// Generic task solve (2026-11-19). The engine (SQP + qpmad + joint-limit box
// + trust region) is shared with the look-at solver; these tests pin the
// pose-task behaviour the grasp generation relies on:
//
//   * a reachable 6-DoF pose is solved back to within tolerance,
//   * the solution stays inside the joint-limit margin,
//   * an unreachable target is rejected instead of returning a wrong pose,
//   * a task-provided hard row reaches the QP (no "bad_constraints").
//
// The look-at path is NOT exercised here — test_lookat_qp_solver and friends
// remain its regression gate.

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/lookat_qp_solver.hpp"
#include "lgw_bt_planner/solve_task.hpp"

using lgw_bt_planner::JointArray;
using lgw_bt_planner::QpPostureSolver;
using lgw_bt_planner::SolveMetrics;
using lgw_bt_planner::SolveTask;
using lgw_bt_planner::TaskSolveParams;

namespace
{

std::string piperUrdf()
{
  return ament_index_cpp::get_package_share_directory("agx_arm_description") +
         "/agx_arm_urdf/piper/urdf/piper_description.urdf";
}

// FK of the tip (link6). The solver exposes cameraPose(); feeding it an
// identity hand-eye makes the camera frame identical to the tip frame.
bool tipPoseOf(
  const QpPostureSolver & fk, const JointArray & q,
  tf2::Vector3 & p, tf2::Matrix3x3 & R, std::string & err)
{
  return fk.cameraPose(
    q, tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0), p, R, err);
}

// Rotation-vector (axis * angle), the orientation error the production pose
// task will use.
Eigen::Vector3d logMap(const tf2::Matrix3x3 & R)
{
  tf2::Quaternion q;
  R.getRotation(q);
  q.normalize();
  const tf2::Vector3 axis = q.getAxis();
  return Eigen::Vector3d(axis.x(), axis.y(), axis.z()) * q.getAngle();
}

class PoseTask : public SolveTask
{
public:
  PoseTask(
    const QpPostureSolver & fk, const tf2::Vector3 & p_tgt,
    const tf2::Matrix3x3 & R_tgt, double tol_pos, double tol_rot)
  : fk_(fk), p_tgt_(p_tgt), R_tgt_(R_tgt), tol_pos_(tol_pos), tol_rot_(tol_rot)
  {}

  Eigen::VectorXd residual(const JointArray & q) const override
  {
    tf2::Vector3 p;
    tf2::Matrix3x3 R;
    std::string err;
    Eigen::VectorXd e(6);
    if (!tipPoseOf(fk_, q, p, R, err)) {
      e.setConstant(1e3);   // FK failed: push the solver away
      return e;
    }
    e.head<3>() << (p_tgt_.x() - p.x()) / tol_pos_,
      (p_tgt_.y() - p.y()) / tol_pos_,
      (p_tgt_.z() - p.z()) / tol_pos_;
    const tf2::Matrix3x3 dR = R_tgt_ * R.transpose();
    e.tail<3>() = logMap(dR) / tol_rot_;
    return e;
  }

  bool accept(
    const JointArray &, const SolveMetrics & m, std::string & why) const override
  {
    if (m.residual_norm > 1.0) {
      why = "residual_norm";
      return false;
    }
    return true;
  }

private:
  const QpPostureSolver & fk_;
  tf2::Vector3 p_tgt_;
  tf2::Matrix3x3 R_tgt_;
  double tol_pos_;
  double tol_rot_;
};

const JointArray kSeed{0.0, 0.9, -0.72, 0.0, -0.80, 0.0};

}  // namespace

TEST(PoseIk, SolvesReachablePoseWithinTolerance)
{
  QpPostureSolver qp;
  std::string err;
  ASSERT_TRUE(qp.initialize(piperUrdf(), true, "base_link", "link6", err))
  << err;

  // Target = FK of a DIFFERENT configuration, so the solve has to move.
  const JointArray goal{0.25, 0.60, -0.50, 0.10, -0.60, 0.20};
  tf2::Vector3 p;
  tf2::Matrix3x3 R;
  ASSERT_TRUE(tipPoseOf(qp, goal, p, R, err)) << err;

  PoseTask task(qp, p, R, 0.01, 0.05);
  TaskSolveParams params;
  const auto res = qp.solve_task(task, kSeed, params);

  EXPECT_TRUE(res.ok) << res.status;
  EXPECT_LT(res.metrics.residual_norm, 1.0);
  EXPECT_GE(res.metrics.joint_margin, 0.19);   // param default 0.20 rad
}

TEST(PoseIk, RejectsUnreachableTarget)
{
  QpPostureSolver qp;
  std::string err;
  ASSERT_TRUE(qp.initialize(piperUrdf(), true, "base_link", "link6", err))
  << err;

  tf2::Vector3 p;
  tf2::Matrix3x3 R;
  ASSERT_TRUE(tipPoseOf(qp, kSeed, p, R, err)) << err;

  const tf2::Vector3 far(2.0, 0.0, 0.0);   // far outside the arm's reach
  PoseTask task(qp, far, R, 0.01, 0.05);
  TaskSolveParams params;
  const auto res = qp.solve_task(task, kSeed, params);
  EXPECT_FALSE(res.ok);
}

TEST(PoseIk, TaskConstraintRowsReachTheQp)
{
  QpPostureSolver qp;
  std::string err;
  ASSERT_TRUE(qp.initialize(piperUrdf(), true, "base_link", "link6", err))
  << err;

  tf2::Vector3 p;
  tf2::Matrix3x3 R;
  ASSERT_TRUE(tipPoseOf(qp, kSeed, p, R, err)) << err;

  // Deliberately slack row: proves the plumbing (A/lb/ub reach qpmad) without
  // changing the optimum.
  class WithSlackRow : public PoseTask
  {
  public:
    using PoseTask::PoseTask;
    void add_constraints(
      const JointArray &, Eigen::MatrixXd & A, Eigen::VectorXd & lb,
      Eigen::VectorXd & ub) const override
    {
      A = Eigen::MatrixXd::Zero(1, 6);
      lb = Eigen::VectorXd::Constant(1, -1.0);
      ub = Eigen::VectorXd::Constant(1, 1.0);
    }
  };

  WithSlackRow task(qp, p, R, 0.01, 0.05);
  TaskSolveParams params;
  const auto res = qp.solve_task(task, kSeed, params);
  EXPECT_TRUE(res.ok) << res.status;
  EXPECT_NE(res.status, "bad_constraints");
}
