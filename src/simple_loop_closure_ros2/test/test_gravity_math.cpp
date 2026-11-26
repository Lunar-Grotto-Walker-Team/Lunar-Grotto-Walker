#include <cmath>

#include <gtest/gtest.h>

#include "simple_loop_closure/gravity_math.hpp"

// Point-LIO body frame = the /livox/imu IMU frame: x forward, y left, z UP
// (verified on the corridor bags: /livox/imu reads (0,0,+1)g at rest and the
// registered cloud's floor normal is +z).  roll is about x, pitch about y,
// yaw about z, matching gtsam::Rot3::rpy().
TEST(GravityMath, LevelGivesZeroRollPitch) {
  const auto g = simple_loop_closure::gravityToRollPitch({0, 0, 1});
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.roll_deg, 0.0, 1e-9);
  EXPECT_NEAR(g.pitch_deg, 0.0, 1e-9);
}

TEST(GravityMath, PitchTenDegrees) {
  const double t = 10.0 * M_PI / 180.0;
  const auto g =
      simple_loop_closure::gravityToRollPitch({-std::sin(t), 0.0, std::cos(t)});
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.pitch_deg, 10.0, 1e-6);
  EXPECT_NEAR(g.roll_deg, 0.0, 1e-6);
}

TEST(GravityMath, RollFiveDegrees) {
  const double t = 5.0 * M_PI / 180.0;
  const auto g =
      simple_loop_closure::gravityToRollPitch({0.0, std::sin(t), std::cos(t)});
  ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.roll_deg, 5.0, 1e-6);
}

TEST(GravityMath, RejectsUpsideDown) {
  EXPECT_FALSE(simple_loop_closure::gravityToRollPitch({0, 0, -1}).valid);
}

TEST(GravityMath, RestGravityExtrinsicMapsToUp) {
  const double t = 8.0 * M_PI / 180.0;
  const Eigen::Vector3d rest(std::sin(t), 0.0, std::cos(t));
  const Eigen::Matrix3d E = simple_loop_closure::extrinsicFromRestGravity(rest);
  EXPECT_NEAR((E * rest.normalized() - Eigen::Vector3d(0, 0, 1)).norm(), 0.0,
              1e-9);
}

TEST(GravityMath, WorldAttitudeExtrinsicIsTheRotation) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  const Eigen::Matrix3d E = simple_loop_closure::extrinsicFromWorldAttitude(q);
  EXPECT_NEAR((E - q.toRotationMatrix()).norm(), 0.0, 1e-12);
}

TEST(GravityMath, UtlidarTransformMatchesCalibImu) {
  const double t = 15.1 * M_PI / 180.0;
  const Eigen::Vector3d z = simple_loop_closure::utlidarAccelToZUp({0, 0, 1});
  EXPECT_NEAR(z.x(), std::sin(t), 1e-9);
  EXPECT_NEAR(z.y(), 0.0, 1e-9);
  EXPECT_NEAR(z.z(), -std::cos(t), 1e-9);
}

// The calibrated /utlidar/imu stream is gravity-DOWN (static mean
// (1.41, 0.05, -9.79)); the prior needs the specific force (up).
TEST(GravityMath, UtlidarSpecificForcePointsUp) {
  const Eigen::Vector3d g = simple_loop_closure::utlidarSpecificForce({0, 0, 1});
  EXPECT_GT(g.z(), 0.3);
  EXPECT_TRUE(simple_loop_closure::gravityToRollPitch(g).valid);
}
