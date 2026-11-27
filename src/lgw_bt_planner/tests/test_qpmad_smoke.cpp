// tests/test_qpmad_smoke.cpp
//
// Proves the vendored qpmad builds and solves the exact problem shape the
// QP lookat solver needs (box bounds + one general linear inequality).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <qpmad/solver.h>

TEST(QpmadSmoke, MinimisesWithBoxAndLinearConstraint)
{
  // min 0.5*x'x  s.t.  0 <= x <= 2,  x0 + x1 >= 1  ->  optimum (0.5, 0.5)
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lb(2);
  lb << 0.0, 0.0;
  Eigen::VectorXd ub(2);
  ub << 2.0, 2.0;
  Eigen::MatrixXd A(1, 2);
  A << 1.0, 1.0;
  Eigen::VectorXd Alb(1);
  Alb << 1.0;
  Eigen::VectorXd Aub(1);
  Aub << 1e10;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  qpmad::Solver solver;
  const auto status = solver.solve(x, H, h, lb, ub, A, Alb, Aub);
  ASSERT_EQ(status, qpmad::Solver::ReturnStatus::OK);
  EXPECT_NEAR(x(0), 0.5, 1e-6);
  EXPECT_NEAR(x(1), 0.5, 1e-6);
}
