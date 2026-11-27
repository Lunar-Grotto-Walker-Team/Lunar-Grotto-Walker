#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "simple_loop_closure/ground_diagnostics.hpp"

namespace {

// Plane a*x + b*y + c*z = d sampled on a grid in x/y, with z solved.  The
// Point-LIO body frame is x forward, y left, z UP.  The floor does NOT have
// to be horizontal in the body frame: the L2 is mounted with a pitch and the
// pose drifts, so on the corridor the body-frame floor normal is ~15 deg off
// z.  The fit must therefore be a constrained plane fit (RANSAC), not a z
// band.
std::vector<Eigen::Vector3d> gridPlane(double a, double b, double c, double d,
                                       double xmin, double xmax, double ymin,
                                       double ymax, int nx, int ny) {
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(static_cast<size_t>(nx * ny));
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double x = xmin + (xmax - xmin) * i / (nx - 1);
      const double y = ymin + (ymax - ymin) * j / (ny - 1);
      const double z = (d - a * x - b * y) / c;
      pts.emplace_back(x, y, z);
    }
  }
  return pts;
}

}  // namespace

TEST(GroundPlane, FlatFloorAtOneMetre) {
  // z = -1: a horizontal floor one metre below the sensor.
  const auto pts = gridPlane(0.0, 0.0, 1.0, -1.0, -3, 3, -3, 3, 60, 60);
  const auto g = simple_loop_closure::fitGroundPlane(pts,
                                                     Eigen::Matrix3d::Identity());
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.height, 1.0, 1e-6);
  EXPECT_NEAR(g.pitch_err_deg, 0.0, 1e-3);
  EXPECT_NEAR(g.roll_err_deg, 0.0, 1e-3);
  EXPECT_GT(g.inliers, 100u);
}

// The corridor case: the floor is tilted 15 deg in the body frame.  A z-band
// fit smears it over ~2 m and finds nothing; the plane fit must recover it.
TEST(GroundPlane, BodyTiltedFloorFifteenDegrees) {
  const double t = 15.0 * M_PI / 180.0;
  // n = (0, -sin t, cos t) through (0, 0, -1).
  const auto pts = gridPlane(0.0, -std::sin(t), std::cos(t), -std::cos(t),
                             -3, 3, -3, 3, 60, 60);
  const auto g = simple_loop_closure::fitGroundPlane(pts,
                                                     Eigen::Matrix3d::Identity());
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.height, std::cos(t), 1e-4);
  EXPECT_NEAR(g.normal_body.y(), -std::sin(t), 1e-3);
  EXPECT_NEAR(g.normal_body.z(), std::cos(t), 1e-3);
  EXPECT_GT(g.inliers, 300u);
}

TEST(GroundPlane, TiltedFloorGivesPitchError) {
  // z = -1 + 0.05 x  ->  -0.05 x + z = -1, up-normal ~ (-0.05, 0, 1).
  const auto pts = gridPlane(-0.05, 0.0, 1.0, -1.0, -3, 3, -3, 3, 60, 60);
  const auto g = simple_loop_closure::fitGroundPlane(pts,
                                                     Eigen::Matrix3d::Identity());
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.pitch_err_deg, std::atan2(0.05, 1.0) * 180.0 / M_PI, 0.1);
  EXPECT_NEAR(g.roll_err_deg, 0.0, 0.1);
}

TEST(GroundPlane, UsesBodyToWorldRotation) {
  const auto pts = gridPlane(-0.05, 0.0, 1.0, -1.0, -3, 3, -3, 3, 60, 60);
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto g = simple_loop_closure::fitGroundPlane(pts, R);
  ASSERT_TRUE(g.valid);
  const Eigen::Vector3d nw =
      (R * Eigen::Vector3d(-0.05, 0.0, 1.0).normalized()).normalized();
  EXPECT_NEAR(g.pitch_err_deg, std::atan2(-nw.x(), nw.z()) * 180.0 / M_PI, 0.1);
  EXPECT_NEAR(g.roll_err_deg, std::atan2(nw.y(), nw.z()) * 180.0 / M_PI, 0.1);
}

TEST(GroundPlane, RejectsTooFewPoints) {
  const auto pts = gridPlane(0.0, 0.0, 1.0, -1.0, -1, 1, -1, 1, 5, 5);
  const auto g = simple_loop_closure::fitGroundPlane(pts,
                                                     Eigen::Matrix3d::Identity());
  EXPECT_FALSE(g.valid);
}

TEST(GroundPlane, RejectsVerticalWall) {
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j < 60; ++j) {
      const double y = -2.0 + 4.0 * i / 59.0;
      const double z = -3.0 + 6.0 * j / 59.0;
      pts.emplace_back(0.0, y, z);
    }
  }
  const auto g = simple_loop_closure::fitGroundPlane(pts,
                                                     Eigen::Matrix3d::Identity());
  EXPECT_FALSE(g.valid);
}

TEST(GroundPlane, RejectsSteepFortyDegreeFloor) {
  const double t = 40.0 * M_PI / 180.0;
  const auto pts = gridPlane(0.0, -std::sin(t), std::cos(t), -std::cos(t),
                             -3, 3, -3, 3, 60, 60);
  EXPECT_FALSE(
      simple_loop_closure::fitGroundPlane(pts, Eigen::Matrix3d::Identity())
          .valid);
}

// Review Focus 5: a pose that is itself tilted >25 deg must not make its
// keyframe's floor unusable for the factor.  The body-frame fit is the real
// measurement; only the diagnostic world gate may reject it.
TEST(GroundPlane, WorldGateRejectsTiltedPoseButFactorPathAccepts) {
  const auto pts = gridPlane(0.0, 0.0, 1.0, -1.0, -3, 3, -3, 3, 60, 60);
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  const auto diag =
      simple_loop_closure::fitGroundPlane(pts, R, 0.30, 80, true);
  EXPECT_FALSE(diag.valid);

  const auto factor =
      simple_loop_closure::fitGroundPlane(pts, R, 0.30, 80, false);
  ASSERT_TRUE(factor.valid);
  EXPECT_NEAR(factor.height, 1.0, 1e-6);
  EXPECT_NEAR(factor.normal_body.z(), 1.0, 1e-3);
}
