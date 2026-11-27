// tests/test_tracking_loop.cpp
//
// Task 6 of the 2026-11-07 node-split plan: the per-tick control pipeline
// becomes a class with no rclcpp, no TF and no clock. Everything that used to
// be "obvious from being inside the node" — which inputs are acceptable, what
// the deadband means, how the feed-forward remembers the previous setpoint —
// is pinned here instead.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "lgw_bt_planner/servo/servo_params.hpp"
#include "lgw_bt_planner/servo/tracking_loop.hpp"

namespace
{

using lgw_bt_planner::ServoParams;
using lgw_bt_planner::SessionState;
using lgw_bt_planner::TrackingInput;
using lgw_bt_planner::TrackingLoop;
using lgw_bt_planner::HoldReason;

ServoParams defaults() {return ServoParams{};}

lgw_bt_planner::Pose6 pose_at(double x, double y, double z)
{
  lgw_bt_planner::Pose6 p;
  p.p.setValue(x, y, z);
  p.R.setIdentity();
  return p;
}

// A tick that would drive the arm if nothing were wrong: fresh target, finite
// geometry, the whole joint set present.
TrackingInput valid_input()
{
  TrackingInput in;
  in.now_ns = 1'000'000'000;
  in.geometry_valid = true;
  in.source_name = "head";
  in.source_scale = 1.0;
  in.p_arm_base.setValue(0.5, 0.0, 0.3);
  in.tcp_now = pose_at(0.3, 0.0, 0.3);
  in.desired = pose_at(0.4, 0.0, 0.3);   // 10 cm out: well outside the deadband
  return in;
}

}  // namespace

TEST(TrackingLoop, EmitsNoTwistWhenTheTargetIsLost)
{
  TrackingInput in;
  in.now_ns = 1'000'000'000;
  in.source_name = "none";
  in.source_scale = 0.0;
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(in, defaults(), st);

  EXPECT_FALSE(out.publish);
  EXPECT_EQ(out.source_name, "none");
  EXPECT_EQ(out.hold_reason, HoldReason::kTargetLost);
}

TEST(TrackingLoop, RejectsNonFinitePoseWithoutEmittingNaN)
{
  TrackingInput in = valid_input();
  in.tcp_now.p.setValue(std::nan(""), 0.0, 0.0);
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(in, defaults(), st);

  EXPECT_FALSE(out.publish);
  EXPECT_EQ(out.hold_reason, HoldReason::kPoseStale);
  EXPECT_TRUE(std::isfinite(out.twist.v.x()));
  EXPECT_TRUE(std::isfinite(out.twist.w.z()));
}

TEST(TrackingLoop, RejectsNonFiniteDesiredPose)
{
  TrackingInput in = valid_input();
  in.desired.R[0][0] = std::numeric_limits<double>::quiet_NaN();
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(in, defaults(), st);

  EXPECT_FALSE(out.publish);
  EXPECT_EQ(out.hold_reason, HoldReason::kPoseStale);
}

TEST(TrackingLoop, GeometryInvalidDegradesToHold)
{
  TrackingInput in = valid_input();
  in.geometry_valid = false;
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(in, defaults(), st);

  EXPECT_FALSE(out.publish);
  EXPECT_EQ(st.geometry_source, "none");
  EXPECT_EQ(out.hold_reason, HoldReason::kTrackerStale);
}

TEST(TrackingLoop, IgnoresJointsMissingFromTheMessage)
{
  TrackingInput in = valid_input();
  in.joints_complete = false;                     // e.g. only joint1 + joint3
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(in, defaults(), st);

  // A partial joint message must never be zero-filled into a command.
  EXPECT_FALSE(out.publish);
  EXPECT_EQ(out.hold_reason, HoldReason::kJointStateIncomplete);
}

TEST(TrackingLoop, EmitsTheCommandWhenEverythingIsFresh)
{
  TrackingLoop loop;
  SessionState st;

  const auto out = loop.step(valid_input(), defaults(), st);

  EXPECT_TRUE(out.publish);
  EXPECT_EQ(out.source_name, "head");
  EXPECT_EQ(st.geometry_source, "head");
  EXPECT_GT(out.twist.v.x(), 0.0);                // target ahead: move +x
  EXPECT_NEAR(out.twist.v.y(), 0.0, 1e-12);
}

TEST(TrackingLoop, PinnedTargetInsideTheDeadbandPublishesZeroAtTheSteadyState)
{
  TrackingInput in = valid_input();
  in.desired = in.tcp_now;                        // exactly on target
  TrackingLoop loop;
  SessionState st;
  ServoParams p = defaults();
  p.ff_enable = false;                            // isolate the P term

  const auto out = loop.step(in, p, st);

  // Publishing a zero twist is not the same as not publishing: Servo treats a
  // silent publisher as dead, so the deadband still emits.
  EXPECT_TRUE(out.publish);
  EXPECT_TRUE(out.twist.in_deadband);
  EXPECT_NEAR(out.twist.v.length(), 0.0, 1e-12);
}

TEST(TrackingLoop, FeedForwardCarriesTheMovingSetpointBetweenTicks)
{
  TrackingLoop loop;
  SessionState st;
  ServoParams p = defaults();
  p.control.kp_pos = 0.0;                         // feed-forward only
  p.control.kp_ang = 0.0;
  p.control.kd_pos = 0.0;
  p.control.kd_ang = 0.0;

  TrackingInput in = valid_input();
  loop.step(in, p, st);                           // first tick: derivative unknown

  in.now_ns += 20'000'000;                        // 20 ms later
  in.geometry_valid = true;
  in.desired.p.setValue(0.42, 0.0, 0.3);          // setpoint moved +2 cm
  const auto out = loop.step(in, p, st);

  EXPECT_TRUE(out.publish);
  EXPECT_GT(out.twist.v.x(), 0.0);
}

TEST(TrackingLoop, MeasuredTwistOpposesTheCommand)
{
  TrackingInput in = valid_input();
  in.v_meas.setValue(0.5, 0.0, 0.0);              // arm already moving at 0.5 m/s
  TrackingLoop loop;
  SessionState st;
  ServoParams p = defaults();
  p.ff_enable = false;

  const auto damped = loop.step(in, p, st);

  TrackingLoop undamped_loop;
  SessionState undamped_st;
  ServoParams q = defaults();
  q.ff_enable = false;
  q.control.kd_pos = 0.0;
  const auto undamped = undamped_loop.step(in, q, undamped_st);

  EXPECT_LT(damped.twist.v.x(), undamped.twist.v.x());
}

TEST(TrackingLoop, ResetForgetsTheFeedForwardMemory)
{
  TrackingLoop loop;
  SessionState st;
  ServoParams p = defaults();
  p.control.kp_pos = 0.0;
  p.control.kp_ang = 0.0;

  TrackingInput in = valid_input();
  loop.step(in, p, st);
  in.now_ns += 20'000'000;
  in.geometry_valid = true;
  in.desired.p.setValue(0.42, 0.0, 0.3);
  const auto moving = loop.step(in, p, st);
  ASSERT_GT(moving.twist.v.x(), 0.0);

  loop.reset();
  in.now_ns += 20'000'000;
  in.geometry_valid = true;
  in.desired.p.setValue(0.44, 0.0, 0.3);
  const auto after = loop.step(in, p, st);

  // Without history there is no derivative on the first tick after a reset.
  EXPECT_NEAR(after.twist.v.x(), 0.0, 1e-12);
}
