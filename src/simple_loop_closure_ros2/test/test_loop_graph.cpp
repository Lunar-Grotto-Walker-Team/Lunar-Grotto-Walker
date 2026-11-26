#include <gtest/gtest.h>

#include <cmath>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "simple_loop_closure/factors.hpp"
#include "simple_loop_closure/loop_noise.hpp"

namespace {

double optimize_z(bool constrain_z) {
  gtsam::NonlinearFactorGraph g;
  g.addPrior(0, gtsam::Pose3(),
             gtsam::noiseModel::Diagonal::Variances(
                 (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6)
                     .finished()));
  const auto odom = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished());
  g.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, gtsam::Pose3(), odom));
  g.add(gtsam::BetweenFactor<gtsam::Pose3>(1, 2, gtsam::Pose3(), odom));

  // The loop says T0 is 1 m above T2; the odometry chain says they are level.
  const auto loop_noise = gtsam::noiseModel::Diagonal::Variances(
      simple_loop_closure::makeLoopVariances(
          1e-6, 1.0, 1e-6, constrain_z, /*constrain_attitude=*/false, 1e4));
  g.add(gtsam::BetweenFactor<gtsam::Pose3>(
      2, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1.0)),
      loop_noise));

  gtsam::Values init;
  for (int i = 0; i < 3; ++i) init.insert(i, gtsam::Pose3());
  return gtsam::LevenbergMarquardtOptimizer(g, init)
      .optimize()
      .at<gtsam::Pose3>(2)
      .z();
}

}  // namespace

TEST(LoopGraph, ThreeDofIgnoresVerticalLoopError) {
  const double z6 = optimize_z(/*constrain_z=*/true);
  const double z3 = optimize_z(/*constrain_z=*/false);
  EXPECT_GT(std::abs(z6), 0.1);   // 6-DOF really is pulled by the loop
  EXPECT_LT(std::abs(z3), 1e-3);  // 3-DOF leaves z on the odometry
  EXPECT_GT(std::abs(z6 - z3), 0.1);
}

TEST(LoopGraph, LegZDiffMovesSecondPose) {
  // Wiring test: pose 1 is constrained only by the leg-z difference.  A
  // competing 1 cm odometry edge would fight the 20 cm measurement and the
  // Cauchy kernel would down-weight it, making the assertion meaningless.
  gtsam::NonlinearFactorGraph g;
  g.addPrior(0, gtsam::Pose3(),
             gtsam::noiseModel::Diagonal::Variances(
                 (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6)
                     .finished()));
  g.add(boost::make_shared<simple_loop_closure::ZDiffFactor>(
      0, 1, 0.20, simple_loop_closure::robustNoise1(4e-4, 3.0)));
  gtsam::Values init;
  init.insert(0, gtsam::Pose3());
  init.insert(1, gtsam::Pose3());
  const auto out = gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
  EXPECT_NEAR(out.at<gtsam::Pose3>(1).z(), 0.20, 0.02);
}

namespace {

// Straight path along world -y (body forward = body x).  A gravity-like
// roll prior varies along the path; the tight relative-rotation edge then
// forces yaw adjustments, which rotate the body-frame odometry translation.
// World-frame translation must keep the path straight.
gtsam::Values solve_odom_with_roll_prior(bool world_trans) {
  constexpr int kN = 120;
  std::vector<gtsam::Pose3> poses;
  poses.reserve(kN);
  for (int i = 0; i < kN; ++i) {
    const double wobble = 0.01 * std::sin(0.7 * i);
    poses.emplace_back(gtsam::Rot3::Ypr(-M_PI / 2 + wobble, 0.0, 0.0),
                       gtsam::Point3(0.0, -0.25 * i, 0.0));
  }
  gtsam::NonlinearFactorGraph g;
  g.addPrior(0, poses[0],
             gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
  for (int i = 1; i < kN; ++i) {
    const gtsam::Pose3 rel = poses[i - 1].between(poses[i]);
    if (world_trans) {
      gtsam::Vector6 rv;
      rv << 1e-4, 1e-4, 1e-4, 1e4, 1e4, 1e4;
      g.add(gtsam::BetweenFactor<gtsam::Pose3>(
          i - 1, i, gtsam::Pose3(rel.rotation(), gtsam::Point3::Zero()),
          gtsam::noiseModel::Diagonal::Variances(rv)));
      const gtsam::Vector3 dp =
          poses[i].translation() - poses[i - 1].translation();
      g.add(boost::make_shared<simple_loop_closure::WorldTransDiffFactor>(
          i - 1, i, dp,
          gtsam::noiseModel::Diagonal::Variances(
              (gtsam::Vector3() << 1e-4, 1e-4, 1e-4).finished())));
    } else {
      gtsam::Vector6 v;
      v << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
      g.add(gtsam::BetweenFactor<gtsam::Pose3>(
          i - 1, i, rel, gtsam::noiseModel::Diagonal::Variances(v)));
    }
    const gtsam::Pose3 target(
        gtsam::Rot3::Ypr(poses[i].rotation().ypr()(2), 0.0,
                         0.05 * std::sin(0.13 * i)),
        poses[i].translation());
    g.add(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        i, target,
        simple_loop_closure::makeRollPitchPriorNoise(9e-4, 1e4, 3.0)));
  }
  gtsam::Values init;
  for (int i = 0; i < kN; ++i) init.insert(i, poses[i]);
  return gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
}

}  // namespace

TEST(WorldOdomGraph, AttitudePriorDoesNotBendXy) {
  const auto body = solve_odom_with_roll_prior(false);
  const auto world = solve_odom_with_roll_prior(true);
  double body_x = 0.0;
  double world_x = 0.0;
  for (int i = 0; i < 120; ++i) {
    body_x = std::max(body_x, std::abs(body.at<gtsam::Pose3>(i).x()));
    world_x = std::max(world_x, std::abs(world.at<gtsam::Pose3>(i).x()));
  }
  EXPECT_LT(world_x, 0.01);
  EXPECT_GT(body_x, 0.05);  // body-frame edge bends ~0.20 m
  EXPECT_GT(body_x, 5.0 * world_x);
}
