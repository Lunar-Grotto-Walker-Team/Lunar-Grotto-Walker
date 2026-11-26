#include <cmath>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "simple_loop_closure/loop_quality.hpp"

using simple_loop_closure::isDuplicateLoop;
using simple_loop_closure::motionScale;
using simple_loop_closure::overlapRatio;
using simple_loop_closure::yawDelta;

TEST(LoopQuality, OverlapIdentical) {
  std::vector<Eigen::Vector3d> c = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  EXPECT_DOUBLE_EQ(overlapRatio(c, c, Eigen::Affine3d::Identity(), 0.2), 1.0);
}

TEST(LoopQuality, OverlapFar) {
  std::vector<Eigen::Vector3d> src = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3d> dst = {{0, 0, 0}, {1, 0, 0}};
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = Eigen::Vector3d(5, 0, 0);
  EXPECT_DOUBLE_EQ(overlapRatio(src, dst, T, 0.2), 0.0);
}

TEST(LoopQuality, OverlapHalf) {
  std::vector<Eigen::Vector3d> src = {
      {0, 0, 0}, {1, 0, 0}, {10, 0, 0}, {11, 0, 0}};
  std::vector<Eigen::Vector3d> dst = {{0, 0, 0}, {1, 0, 0}};
  EXPECT_DOUBLE_EQ(
      overlapRatio(src, dst, Eigen::Affine3d::Identity(), 0.2), 0.5);
}

TEST(LoopQuality, YawDeltaWraps) {
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d rz01 =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(yawDelta(rz01, I), 0.1, 1e-9);

  // 3.0 - (-3.0) = 6.0 rad, which wraps to 6.0 - 2*pi = -0.283 rad.
  const Eigen::Matrix3d r3 =
      Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d rm3 =
      Eigen::AngleAxisd(-3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_NEAR(std::abs(yawDelta(r3, rm3)), 0.2831853, 1e-6);
}

TEST(LoopQuality, DuplicateWindow) {
  std::unordered_map<int, int> last_query;
  EXPECT_FALSE(isDuplicateLoop(100, 50, last_query, 20));
  last_query[50] = 95;
  EXPECT_TRUE(isDuplicateLoop(100, 50, last_query, 20));
  EXPECT_FALSE(isDuplicateLoop(116, 50, last_query, 20));  // diff = 21
}

TEST(LoopQuality, MotionScale) {
  EXPECT_DOUBLE_EQ(motionScale(0.15, 0.15), 1.0);
  EXPECT_DOUBLE_EQ(motionScale(0.5, 0.15), 0.5 / 0.15);
  EXPECT_DOUBLE_EQ(motionScale(0.1, 0.15), 1.0);
}
