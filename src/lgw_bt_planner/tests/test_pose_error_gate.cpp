// test_pose_error_gate.cpp — contract for the arm-motion arrival gates.
//
// Regression: on 2026-11-17 an UnloadHandle run reported SUCCESS while the
// real tcp pose was 0.29–0.37 m away from every goal, and a later park left
// joint4 0.41 rad off while MoveArmToNamed returned SUCCESS without looking
// at any feedback. Both nodes now compare feedback against the command.

#include <gtest/gtest.h>

#include "lgw_bt_planner/pose_error_gate.hpp"

using lgw_bt_planner::error_within_tolerance;
using lgw_bt_planner::pose_error_within_tolerance;
using lgw_bt_planner::pose_within_tolerance;
using lgw_bt_planner::rotation_angle_error_rad;

namespace
{
constexpr double kPi = 3.14159265358979323846;
// at_handle orientation from unload_handle.xml, as xyzw.
const double kQa[4] = {-0.034175, 0.998510, -0.040224, -0.013880};
const double kNQa[4] = {0.034175, -0.998510, 0.040224, 0.013880};
const double kIdentity[4] = {0.0, 0.0, 0.0, 1.0};
}  // namespace

TEST(PoseErrorGate, AcceptsErrorInsideTolerance)
{
  EXPECT_TRUE(pose_error_within_tolerance(0.0, 0.005));
  EXPECT_TRUE(pose_error_within_tolerance(0.004, 0.005));
  EXPECT_TRUE(pose_error_within_tolerance(0.005, 0.005));
}

TEST(PoseErrorGate, RejectsErrorOutsideTolerance)
{
  EXPECT_FALSE(pose_error_within_tolerance(0.0051, 0.005));
  EXPECT_FALSE(pose_error_within_tolerance(0.29, 0.005));
  EXPECT_FALSE(pose_error_within_tolerance(0.37, 0.005));
}

TEST(PoseErrorGate, ZeroAndNegativeDisableTheGate)
{
  EXPECT_TRUE(pose_error_within_tolerance(1.0, 0.0));
  EXPECT_TRUE(pose_error_within_tolerance(1.0, -1.0));
  EXPECT_TRUE(error_within_tolerance(1.0, 0.0));
  EXPECT_TRUE(error_within_tolerance(1.0, -1.0));
}

TEST(PoseErrorGate, MatchesToleranceIsInclusiveAtTypicalThresholds)
{
  EXPECT_TRUE(pose_error_within_tolerance(0.001, 0.001));
  EXPECT_FALSE(pose_error_within_tolerance(0.0011, 0.001));
  EXPECT_TRUE(error_within_tolerance(0.05, 0.05));
  EXPECT_FALSE(error_within_tolerance(0.411, 0.05));
}

TEST(PoseErrorGate, RotationAngleIsZeroForIdenticalQuaternion)
{
  EXPECT_NEAR(rotation_angle_error_rad(kQa, kQa), 0.0, 1e-9);
}

TEST(PoseErrorGate, RotationAngleIgnoresQuaternionSign)
{
  // q and -q are the same rotation; a sign flip must not read as 360 deg.
  EXPECT_NEAR(rotation_angle_error_rad(kQa, kNQa), 0.0, 1e-6);
}

TEST(PoseErrorGate, RotationAngleOfNinetyDegreesAboutZ)
{
  const double qz90[4] = {0.0, 0.0, 0.7071067811865476, 0.7071067811865476};
  EXPECT_NEAR(rotation_angle_error_rad(kIdentity, qz90), kPi / 2.0, 1e-6);
}

TEST(PoseErrorGate, RotationAngleIsSymmetric)
{
  const double qz33[4] = {0.0, 0.0, 0.284015, 0.958820};  // ~33 deg about z
  EXPECT_NEAR(
    rotation_angle_error_rad(kIdentity, qz33),
    rotation_angle_error_rad(qz33, kIdentity), 1e-12);
}

TEST(PoseErrorGate, PoseGateRequiresBothPositionAndOrientation)
{
  // Small position error, big rotation error (the 33 deg wrist taught pose).
  const double qz33[4] = {0.0, 0.0, 0.284015, 0.958820};
  const double rot = rotation_angle_error_rad(kIdentity, qz33);
  EXPECT_GT(rot, 0.05);
  EXPECT_FALSE(pose_within_tolerance(0.001, rot, 0.005, 0.05));

  // Both inside.
  EXPECT_TRUE(pose_within_tolerance(0.001, 0.02, 0.005, 0.05));

  // Position out, orientation fine.
  EXPECT_FALSE(pose_within_tolerance(0.30, 0.01, 0.005, 0.05));
}

TEST(PoseErrorGate, PoseGateCanDisableOrientationOnly)
{
  // max_rot_err_rad = 0 disables the orientation leg, position still gates.
  EXPECT_TRUE(pose_within_tolerance(0.001, kPi, 0.005, 0.0));
  EXPECT_FALSE(pose_within_tolerance(0.30, 0.0, 0.005, 0.0));
}
