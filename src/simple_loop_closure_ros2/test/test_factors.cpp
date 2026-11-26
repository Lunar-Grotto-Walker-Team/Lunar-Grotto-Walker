#include <gtest/gtest.h>

#include <boost/make_shared.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "simple_loop_closure/factors.hpp"

TEST(OdomZDiff, LegZSuppressesContinuity) {
  EXPECT_FALSE(simple_loop_closure::useOdomZDiffWithLegZ(
      true, 1e-3, false, false, false, false));
  EXPECT_TRUE(simple_loop_closure::useOdomZDiffWithLegZ(
      false, 1e-3, false, false, false, false));
}

TEST(ZPriorFactor, MovesPoseZToMeasurement) {
  gtsam::NonlinearFactorGraph g;
  // Weak full-pose prior (sigma 0.5) + strong z prior (sigma 1e-3):
  // the z residual must win, otherwise the test would only prove the
  // full-pose prior is stronger.
  g.addPrior(0,
             gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 1.0)),
             gtsam::noiseModel::Isotropic::Sigma(6, 0.5));
  auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(1) << 1e-6).finished());
  g.add(boost::make_shared<simple_loop_closure::ZPriorFactor>(0, 0.0, noise));

  gtsam::Values init;
  init.insert(0, gtsam::Pose3(gtsam::Rot3::Identity(),
                              gtsam::Point3(0, 0, 1.0)));
  const auto result = gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
  EXPECT_NEAR(result.at<gtsam::Pose3>(0).z(), 0.0, 1e-3);
}

namespace {

double numericZDerivative(const gtsam::NoiseModelFactor1<gtsam::Pose3>& factor,
                          const gtsam::Pose3& pose, int k) {
  gtsam::Vector6 d = gtsam::Vector6::Zero();
  d(k) = 1e-6;
  const double ep = factor.evaluateError(pose.retract(d))(0);
  const double em = factor.evaluateError(pose.retract(-d))(0);
  return (ep - em) / 2e-6;
}

}  // namespace

TEST(ZPriorFactor, JacobianMatchesNumericalAtNonIdentityRotation) {
  const gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.25, 0.2),
                          gtsam::Point3(0.5, -0.2, 1.0));
  const auto factor = boost::make_shared<simple_loop_closure::ZPriorFactor>(
      0, 0.3, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  gtsam::Matrix H;
  factor->evaluateError(pose, H);
  for (int k = 0; k < 6; ++k) {
    EXPECT_NEAR(H(0, k), numericZDerivative(*factor, pose, k), 1e-6)
        << "tangent index " << k;
  }
}

TEST(ZDiffFactor, JacobianMatchesNumericalAtNonIdentityRotation) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.1, 0.25, 0.2),
                        gtsam::Point3(0.5, -0.2, 1.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(-0.15, 0.05, -0.1),
                        gtsam::Point3(-0.4, 0.3, 0.7));
  const auto factor = boost::make_shared<simple_loop_closure::ZDiffFactor>(
      0, 1, 0.2, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  gtsam::Matrix H1, H2;
  factor->evaluateError(p1, p2, H1, H2);
  for (int k = 0; k < 6; ++k) {
    gtsam::Vector6 d = gtsam::Vector6::Zero();
    d(k) = 1e-6;
    const double e1p = factor->evaluateError(p1.retract(d), p2)(0);
    const double e1m = factor->evaluateError(p1.retract(-d), p2)(0);
    const double e2p = factor->evaluateError(p1, p2.retract(d))(0);
    const double e2m = factor->evaluateError(p1, p2.retract(-d))(0);
    EXPECT_NEAR(H1(0, k), (e1p - e1m) / 2e-6, 1e-6) << "H1 index " << k;
    EXPECT_NEAR(H2(0, k), (e2p - e2m) / 2e-6, 1e-6) << "H2 index " << k;
  }
}

TEST(ZDiffFactor, EnforcesRelativeZWithoutAbsoluteAnchor) {
  gtsam::NonlinearFactorGraph g;
  g.addPrior(0, gtsam::Pose3(),
             gtsam::noiseModel::Isotropic::Sigma(6, 0.5));
  const auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(1) << 1e-6).finished());
  g.add(boost::make_shared<simple_loop_closure::ZDiffFactor>(0, 1, 0.5, noise));
  gtsam::Values init;
  init.insert(0, gtsam::Pose3());
  init.insert(1, gtsam::Pose3());
  const auto result = gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
  EXPECT_NEAR(result.at<gtsam::Pose3>(1).z() -
                  result.at<gtsam::Pose3>(0).z(),
              0.5, 1e-3);
}

TEST(WorldTransDiffFactor, ResidualAndJacobians) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.4, -0.2, 0.1),
                        gtsam::Point3(1.0, -2.0, 0.3));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(-0.3, 0.5, 0.2),
                        gtsam::Point3(2.5, -1.0, 0.8));
  const gtsam::Vector3 dp = p2.translation() - p1.translation();
  const auto factor =
      boost::make_shared<simple_loop_closure::WorldTransDiffFactor>(
          0, 1, dp, gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  gtsam::Matrix H1, H2;
  const gtsam::Vector e = factor->evaluateError(p1, p2, H1, H2);
  EXPECT_LT(e.norm(), 1e-12);
  for (int k = 0; k < 6; ++k) {
    gtsam::Vector6 d = gtsam::Vector6::Zero();
    d(k) = 1e-6;
    const gtsam::Vector e1p = factor->evaluateError(p1.retract(d), p2);
    const gtsam::Vector e1m = factor->evaluateError(p1.retract(-d), p2);
    const gtsam::Vector e2p = factor->evaluateError(p1, p2.retract(d));
    const gtsam::Vector e2m = factor->evaluateError(p1, p2.retract(-d));
    EXPECT_TRUE(H1.col(k).isApprox((e1p - e1m) / 2e-6, 1e-6))
        << "H1 index " << k;
    EXPECT_TRUE(H2.col(k).isApprox((e2p - e2m) / 2e-6, 1e-6))
        << "H2 index " << k;
  }
}

TEST(WorldTransDiffFactor, AttitudeChangeDoesNotMoveTranslationResidual) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.0, 0.1, 0.0),
                        gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(0.0, -0.1, 0.0),
                        gtsam::Point3(0.0, -1.0, 0.0));
  const gtsam::Vector3 dp = p2.translation() - p1.translation();
  const auto factor =
      boost::make_shared<simple_loop_closure::WorldTransDiffFactor>(
          0, 1, dp, gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  // A pure roll/pitch change on either pose must not change the residual.
  gtsam::Vector6 d = gtsam::Vector6::Zero();
  d(0) = 0.2;  // roll
  d(1) = -0.15;  // pitch
  EXPECT_LT(factor->evaluateError(p1.retract(d), p2).norm(), 1e-12);
  EXPECT_LT(factor->evaluateError(p1, p2.retract(d)).norm(), 1e-12);
}

TEST(Go2AttitudePrior, TargetKeepsLioYawAndTranslation) {
  const gtsam::Pose3 lio(gtsam::Rot3::Ypr(0.3, 0.1, 0.2),
                         gtsam::Point3(1, 2, 3));
  const auto target = simple_loop_closure::targetPoseFromGo2Attitude(
      lio, Eigen::Quaterniond::Identity(), 0.0, 0.0);
  const auto rpy = target.rotation().rpy();
  EXPECT_NEAR(rpy(0), 0.0, 1e-6);  // roll from Go2
  EXPECT_NEAR(rpy(1), 0.0, 1e-6);  // pitch from Go2
  EXPECT_NEAR(rpy(2), 0.3, 1e-6);  // yaw kept from LIO
  EXPECT_NEAR(target.translation()(0), 1.0, 1e-9);
  EXPECT_NEAR(target.translation()(1), 2.0, 1e-9);
  EXPECT_NEAR(target.translation()(2), 3.0, 1e-9);
}

TEST(WorldXYPhiFactor, ZeroResidualAtMeasurement) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.05, -0.1),
                        gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(0.5, 0.05, -0.1),
                        gtsam::Point3(1.0, 2.0, 0.7));
  const auto factor = boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
      0, 1, Eigen::Vector2d(1.0, 2.0), 0.3,
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  const gtsam::Vector e = factor->evaluateError(p1, p2);
  EXPECT_NEAR(e(0), 0.0, 1e-9);
  EXPECT_NEAR(e(1), 0.0, 1e-9);
  EXPECT_NEAR(e(2), 0.0, 1e-9);
}

TEST(WorldXYPhiFactor, IgnoresZRollPitchButNotYawXY) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.0, 0.0),
                        gtsam::Point3(0.0, 0.0, 0.0));
  const auto factor = boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
      0, 1, Eigen::Vector2d(1.0, 2.0), 0.3,
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));

  const gtsam::Pose3 base(gtsam::Rot3::Ypr(0.5, 0.0, 0.0),
                          gtsam::Point3(1.0, 2.0, 0.0));
  const gtsam::Pose3 perturbed(gtsam::Rot3::Ypr(0.5, 0.25, 0.2),
                               gtsam::Point3(1.0, 2.0, 1.5));
  EXPECT_NEAR((factor->evaluateError(p1, base) -
               factor->evaluateError(p1, perturbed)).norm(), 0.0, 1e-9);

  const gtsam::Pose3 shifted(gtsam::Rot3::Ypr(0.5, 0.0, 0.0),
                             gtsam::Point3(1.1, 2.0, 0.0));
  EXPECT_GT(std::abs(factor->evaluateError(p1, shifted)(1)), 0.05);
}

TEST(WorldXYPhiFactor, JacobianMatchesNumericalAtNonIdentity) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.1, -0.05),
                        gtsam::Point3(0.5, -0.2, 1.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(-0.3, 0.15, 0.2),
                        gtsam::Point3(-0.4, 0.3, 0.7));
  const auto factor = boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
      0, 1, Eigen::Vector2d(0.3, -0.1), 0.2,
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  gtsam::Matrix H1, H2;
  factor->evaluateError(p1, p2, H1, H2);
  for (int k = 0; k < 6; ++k) {
    gtsam::Vector6 d = gtsam::Vector6::Zero();
    d(k) = 1e-6;
    const gtsam::Vector e1p = factor->evaluateError(p1.retract(d), p2);
    const gtsam::Vector e1m = factor->evaluateError(p1.retract(-d), p2);
    const gtsam::Vector e2p = factor->evaluateError(p1, p2.retract(d));
    const gtsam::Vector e2m = factor->evaluateError(p1, p2.retract(-d));
    for (int r = 0; r < 3; ++r) {
      EXPECT_NEAR(H1(r, k), (e1p(r) - e1m(r)) / 2e-6, 1e-5)
          << "H1 " << r << "," << k;
      EXPECT_NEAR(H2(r, k), (e2p(r) - e2m(r)) / 2e-6, 1e-5)
          << "H2 " << r << "," << k;
    }
  }
}

TEST(WorldXYPhiFactor, YawWrapsAcrossPi) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(3.0, 0.0, 0.0),
                        gtsam::Point3::Zero());
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(-3.0, 0.0, 0.0),
                        gtsam::Point3::Zero());
  // -3.0 - 3.0 = -6.0, wraps to -6.0 + 2*pi = +0.283 rad.
  const auto factor = boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
      0, 1, Eigen::Vector2d::Zero(), 0.283185307179586,
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  EXPECT_NEAR(factor->evaluateError(p1, p2)(0), 0.0, 1e-9);
}

TEST(WorldXYPhiFactor, DoesNotPullZOrRollPitch) {
  gtsam::NonlinearFactorGraph g;
  g.addPrior(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
  const auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());
  g.add(boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
      0, 1, Eigen::Vector2d(1.0, 0.0), 0.0, noise));
  gtsam::Values init;
  init.insert(0, gtsam::Pose3());
  init.insert(1, gtsam::Pose3(gtsam::Rot3::Ypr(0.0, 0.3, 0.2),
                              gtsam::Point3(0.5, 0.0, 1.2)));
  const auto result = gtsam::LevenbergMarquardtOptimizer(g, init).optimize();
  const auto p1 = result.at<gtsam::Pose3>(1);
  EXPECT_NEAR(p1.x(), 1.0, 1e-3);
  EXPECT_NEAR(p1.z(), 1.2, 1e-6);       // z untouched
  EXPECT_NEAR(p1.rotation().roll(), 0.2, 1e-6);
  EXPECT_NEAR(p1.rotation().pitch(), 0.3, 1e-6);
}

TEST(PoseNormalFactor, ZeroWhenWorldNormalsAgree) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.1, 0.0), gtsam::Point3::Zero());
  const gtsam::Pose3 p2 = p1;  // same rotation
  const auto f = boost::make_shared<simple_loop_closure::PoseNormalFactor>(
      0, 1, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1),
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  EXPECT_NEAR(f->evaluateError(p1, p2).norm(), 0.0, 1e-12);
}

TEST(PoseNormalFactor, JacobianMatchesNumerical) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.1, -0.05),
                        gtsam::Point3(0.5, -0.2, 1.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(-0.3, 0.15, 0.2),
                        gtsam::Point3(-0.4, 0.3, 0.7));
  const Eigen::Vector3d n1 = Eigen::Vector3d(0.1, -0.2, 0.97).normalized();
  const Eigen::Vector3d n2 = Eigen::Vector3d(-0.05, 0.1, 0.99).normalized();
  const auto f = boost::make_shared<simple_loop_closure::PoseNormalFactor>(
      0, 1, n1, n2, gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  gtsam::Matrix H1, H2;
  f->evaluateError(p1, p2, H1, H2);
  for (int k = 0; k < 6; ++k) {
    gtsam::Vector6 d = gtsam::Vector6::Zero();
    d(k) = 1e-6;
    const gtsam::Vector e1p = f->evaluateError(p1.retract(d), p2);
    const gtsam::Vector e1m = f->evaluateError(p1.retract(-d), p2);
    const gtsam::Vector e2p = f->evaluateError(p1, p2.retract(d));
    const gtsam::Vector e2m = f->evaluateError(p1, p2.retract(-d));
    for (int r = 0; r < 3; ++r) {
      EXPECT_NEAR(H1(r, k), (e1p(r) - e1m(r)) / 2e-6, 1e-5)
          << "H1 " << r << "," << k;
      EXPECT_NEAR(H2(r, k), (e2p(r) - e2m(r)) / 2e-6, 1e-5)
          << "H2 " << r << "," << k;
    }
  }
}

TEST(PoseNormalFactor, TranslationDoesNotMatter) {
  const gtsam::Pose3 p1(gtsam::Rot3::Ypr(0.2, 0.1, 0.0),
                        gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 p2(gtsam::Rot3::Ypr(0.3, 0.0, 0.1),
                        gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Pose3 p2_shifted(p2.rotation(),
                                p2.translation() + gtsam::Point3(1, 0, 0));
  const auto f = boost::make_shared<simple_loop_closure::PoseNormalFactor>(
      0, 1, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1),
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  EXPECT_NEAR((f->evaluateError(p1, p2) - f->evaluateError(p1, p2_shifted))
                  .norm(),
              0.0, 1e-12);
}

TEST(LoopFactorMode, ProjectedOnlyForPure3Dof) {
  EXPECT_TRUE(
      simple_loop_closure::useProjected3Dof("projected_3dof", false, false));
  EXPECT_FALSE(
      simple_loop_closure::useProjected3Dof("projected_3dof", true, false));
  EXPECT_FALSE(
      simple_loop_closure::useProjected3Dof("projected_3dof", false, true));
  EXPECT_FALSE(
      simple_loop_closure::useProjected3Dof("diagonal", false, false));
}

TEST(OdomZContinuity, OnlyWithoutAttitudeOrGo2Z) {
  EXPECT_TRUE(simple_loop_closure::useOdomZDiff(1e-3, false, false, false,
                                                false));
  EXPECT_FALSE(simple_loop_closure::useOdomZDiff(1e-3, true, false, false,
                                                 false));
  EXPECT_FALSE(simple_loop_closure::useOdomZDiff(1e-3, false, true, false,
                                                 false));
  EXPECT_FALSE(
      simple_loop_closure::useOdomZDiff(0.0, false, false, false, false));
}

TEST(GravityFlags, Go2AttitudeSkippedWhenGravityOn) {
  EXPECT_TRUE(simple_loop_closure::useGo2Attitude(true, false));
  EXPECT_FALSE(simple_loop_closure::useGo2Attitude(true, true));
  EXPECT_FALSE(simple_loop_closure::useGo2Attitude(false, false));
}

TEST(GravityFlags, ZDiffSkippedWithGravityUnlessAllowed) {
  EXPECT_FALSE(
      simple_loop_closure::useOdomZDiff(1e-3, false, false, true, false));
  EXPECT_TRUE(
      simple_loop_closure::useOdomZDiff(1e-3, false, false, true, true));
}

TEST(GravityPrior, TargetKeepsLioYawAndTakesGravityRollPitch) {
  const gtsam::Pose3 lio(gtsam::Rot3::Ypr(0.3, 0.2, 0.1),
                         gtsam::Point3(1, 2, 3));
  // Level gravity in the body frame: the target must be roll=0, pitch=0.
  const auto target = simple_loop_closure::targetPoseFromGravity(
      lio, Eigen::Vector3d(0, 0, 1), 0.0, 0.0);
  const auto rpy = target.rotation().rpy();
  EXPECT_NEAR(rpy(0), 0.0, 1e-9);
  EXPECT_NEAR(rpy(1), 0.0, 1e-9);
  EXPECT_NEAR(rpy(2), 0.3, 1e-9);
  EXPECT_NEAR(target.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(target.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(target.translation().z(), 3.0, 1e-12);
}

TEST(GravityPrior, TargetAddsOffsetDegrees) {
  const gtsam::Pose3 lio(gtsam::Rot3::Ypr(0.0, 0.0, 0.0), gtsam::Point3::Zero());
  const auto target = simple_loop_closure::targetPoseFromGravity(
      lio, Eigen::Vector3d(0, 0, 1), 1.5, -2.0);
  const auto rpy = target.rotation().rpy();
  EXPECT_NEAR(rpy(0) * 180.0 / M_PI, 1.5, 1e-9);
  EXPECT_NEAR(rpy(1) * 180.0 / M_PI, -2.0, 1e-9);
}

TEST(Go2AttitudePrior, CorrectsRollPitchNotYaw) {
  const gtsam::Pose3 init(gtsam::Rot3::Ypr(0.3, 0.05, 0.1),
                          gtsam::Point3(0, 0, 0));
  const auto target = simple_loop_closure::targetPoseFromGo2Attitude(
      init, Eigen::Quaterniond::Identity(), 0.0, 0.0);
  gtsam::NonlinearFactorGraph g;
  g.add(gtsam::PriorFactor<gtsam::Pose3>(
      0, target,
      simple_loop_closure::makeRollPitchPriorNoise(1e-6, 1e4)));
  gtsam::Values values;
  values.insert(0, init);
  const auto result = gtsam::LevenbergMarquardtOptimizer(g, values).optimize();
  const auto rpy = result.at<gtsam::Pose3>(0).rotation().rpy();
  EXPECT_NEAR(rpy(0), 0.0, 1e-3);  // roll corrected
  EXPECT_NEAR(rpy(1), 0.0, 1e-3);  // pitch corrected
  EXPECT_NEAR(rpy(2), 0.3, 1e-3);  // yaw left free
}

TEST(WorldXYPhiPriorFactor, PullsPoseToMeasurement) {
  gtsam::NonlinearFactorGraph g;
  // Weak full-pose prior (sigma 0.5) + strong unary prior (sigma 1e-3):
  // the xy/yaw residual must win, otherwise the test only proves the
  // full-pose prior is stronger.
  g.addPrior(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 0.5));
  auto noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());  // yaw, x, y
  g.add(boost::make_shared<simple_loop_closure::WorldXYPhiPriorFactor>(
      0, Eigen::Vector2d(1.0, 2.0), 0.3, noise));

  gtsam::Values init;
  init.insert(0, gtsam::Pose3());
  const gtsam::Values out =
      gtsam::LevenbergMarquardtOptimizer(g, init).optimize();

  const gtsam::Pose3 p = out.at<gtsam::Pose3>(0);
  EXPECT_NEAR(p.x(), 1.0, 1e-3);
  EXPECT_NEAR(p.y(), 2.0, 1e-3);
  EXPECT_NEAR(p.rotation().yaw(), 0.3, 1e-3);
  EXPECT_NEAR(p.z(), 0.0, 1e-6);  // z is not in the factor, must not move
}
