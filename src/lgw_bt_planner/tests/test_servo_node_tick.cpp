// tests/test_servo_node_tick.cpp
//
// Task 1 of the 2026-11-07 node-split plan: the control tick must be drivable
// from a test with an explicit timestamp. Everything the tick does keys off
// `now_ns`, so the replay harness (Task 2) can step eight scenarios through the
// real class instead of wall-clock time.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace
{

rclcpp::NodeOptions idle_options()
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("frame_prefix", "arm1/")});
  return opts;
}

}  // namespace

TEST(ServoNodeTick, TicksWithoutSessionAndStaysIdle)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  EXPECT_EQ(node->state_for_test(), "IDLE");
  node->tick(1'000'000'000);
  EXPECT_EQ(node->state_for_test(), "IDLE");
}

TEST(ServoNodeTick, RepeatedTicksWithSteppingTimeDoNotLeaveIdle)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  for (int i = 0; i < 10; ++i) {
    node->tick(1'000'000'000 + static_cast<int64_t>(i) * 10'000'000);
  }
  EXPECT_EQ(node->state_for_test(), "IDLE");
}

TEST(ServoNodeTick, SnapshotUsesPublishedArmBaseAndGeometryValid)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());

  yoloe_interfaces::msg::UnifiedTrack track;
  track.track_id = 7;
  track.task_state = "active";
  track.geometry_frame = "arm1/base_link";
  track.geometry_valid = true;
  track.source = "arm";
  track.position.x = 1.0;
  track.position_arm_base.x = 0.5;
  track.position_arm_base.z = 0.4;
  track.last_observation_stamp = node->now();

  const auto snap = node->make_snapshot_for_test(track);
  EXPECT_EQ(snap.track_id, 7u);
  EXPECT_TRUE(snap.geometry_valid);
  EXPECT_NEAR(snap.p_arm_base.x(), 0.5, 1e-9);
  EXPECT_NEAR(snap.p_arm_base.z(), 0.4, 1e-9);
  EXPECT_EQ(snap.source, "arm");
}
