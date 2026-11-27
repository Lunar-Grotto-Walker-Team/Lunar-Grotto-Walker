// tests/test_servo_state_reporter.cpp
//
// Task 8, step 4b of the 2026-11-07 node-split plan. /lgw/servo/state is what
// a field operator reads; the rules about what it may show when there is no
// session were buried in a node method and had no test.

#include <gtest/gtest.h>

#include "lgw_bt_planner/servo/servo_state_reporter.hpp"

namespace
{

using lgw_bt_planner::HoldReason;
using lgw_bt_planner::ServoState;
using lgw_bt_planner::ServoStateSnapshot;
using lgw_bt_planner::build_servo_state;

ServoStateSnapshot tracking_snapshot()
{
  ServoStateSnapshot s;
  s.stamp_ns = 1'500'000'250;
  s.state = ServoState::TRACKING;
  s.session_active = true;
  s.geometry_source = "head";
  s.track_id = 42;
  s.angular_error_rad = 0.11;
  s.position_error_m = 0.02;
  s.target_distance_m = 0.61;
  s.converged = true;
  s.too_close = false;
  s.hold_since_ns = 0;
  return s;
}

}  // namespace

TEST(ServoStateReporter, CarriesTheSessionFieldsWhileTracking)
{
  const auto msg = build_servo_state(tracking_snapshot());

  EXPECT_EQ(msg.stamp.sec, 1);
  EXPECT_EQ(msg.stamp.nanosec, 500000250U);
  EXPECT_EQ(msg.state, "TRACKING");
  EXPECT_EQ(msg.geometry_source, "head");
  EXPECT_EQ(msg.track_id, 42U);
  EXPECT_DOUBLE_EQ(msg.angular_error_rad, 0.11);
  EXPECT_DOUBLE_EQ(msg.position_error_m, 0.02);
  EXPECT_DOUBLE_EQ(msg.target_distance_m, 0.61);
  EXPECT_TRUE(msg.converged);
}

TEST(ServoStateReporter, IdleReportsNoGeometryEvenIfStaleValuesRemain)
{
  // The node keeps its last values around; the message must not leak them.
  ServoStateSnapshot s = tracking_snapshot();
  s.state = ServoState::IDLE;
  s.session_active = false;

  const auto msg = build_servo_state(s);

  EXPECT_EQ(msg.geometry_source, "none");
  EXPECT_EQ(msg.track_id, 0U);
  EXPECT_DOUBLE_EQ(msg.angular_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(msg.position_error_m, 0.0);
  EXPECT_DOUBLE_EQ(msg.target_distance_m, 0.0);
  EXPECT_FALSE(msg.converged);
  EXPECT_FALSE(msg.too_close);
}

TEST(ServoStateReporter, HoldKeepsReportingWhyItIsHolding)
{
  ServoStateSnapshot s = tracking_snapshot();
  s.state = ServoState::HOLD;
  s.converged = true;             // the deadband value, but nothing is commanded
  s.hold_reason = HoldReason::kTargetLost;
  s.hold_since_ns = 2'000'000'000;
  s.recovery_attempts = 1;

  const auto msg = build_servo_state(s);

  EXPECT_EQ(msg.state, "HOLD");
  EXPECT_EQ(msg.geometry_source, "head");    // still the last known tier
  EXPECT_FALSE(msg.converged);               // HOLD never reports convergence
  EXPECT_EQ(msg.hold_reason, "target_lost");
  EXPECT_EQ(msg.hold_since.sec, 2);
  EXPECT_EQ(msg.recovery_attempts, 1U);
}

TEST(ServoStateReporter, ConvergedRequiresTrackingEvenIfTheFlagIsSet)
{
  ServoStateSnapshot s = tracking_snapshot();
  s.state = ServoState::PREEMPT;
  s.converged = true;

  EXPECT_FALSE(build_servo_state(s).converged);
}

TEST(ServoStateReporter, ZeroHoldSinceLeavesTheStampEmpty)
{
  ServoStateSnapshot s = tracking_snapshot();
  s.hold_since_ns = 0;

  const auto msg = build_servo_state(s);

  EXPECT_EQ(msg.hold_since.sec, 0);
  EXPECT_EQ(msg.hold_since.nanosec, 0U);
}

TEST(ServoStateReporter, JointStateIncompleteHasAStableName)
{
  ServoStateSnapshot s = tracking_snapshot();
  s.state = ServoState::HOLD;
  s.hold_reason = HoldReason::kJointStateIncomplete;

  EXPECT_EQ(build_servo_state(s).hold_reason, "joint_state_incomplete");
}
