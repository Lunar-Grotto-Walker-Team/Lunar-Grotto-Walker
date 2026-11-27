// tests/test_select_arm_observation_pose_geom.cpp
// Pure-function unit tests for select_arm_observation_pose_geom.
//
// No ROS runtime needed (math only). Run from build/lgw_bt_planner/
// test_select_arm_observation_pose_geom directly — colcon test is not
// available in this container (see CLAUDE.md / bt-tech-summary §8).

#include "lgw_bt_planner/select_arm_observation_pose_geom.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr double kEps = 1e-9;
constexpr double kEpsCoarse = 1e-6;

// Validate that (R, q) pair produces a proper rotation:
//   R * R^T = I,  det(R) = +1
void ExpectRotationFromQuaternion(
  const tf2::Quaternion& q,
  const char* ctx = "")
{
  const tf2::Matrix3x3 R(q);
  const tf2::Matrix3x3 Rt = R.transpose();
  const tf2::Matrix3x3 RRt = R * Rt;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(RRt[i][j], expected, kEps)
        << "R*R^T[" << i << "][" << j << "]  " << ctx;
    }
  }
  EXPECT_NEAR(R.determinant(), 1.0, kEps) << "  " << ctx;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}
}  // namespace
