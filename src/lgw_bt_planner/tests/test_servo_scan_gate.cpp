// tests/test_servo_scan_gate.cpp
// SCANNING gate: only HOLD + kTargetLost may start the worker.  Operator
// pause and other HOLD reasons must never move the arm.
// 2026-11-17: servo follows UnifiedTrackArray.selected_track_id; the local
// target selector is gone.

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

TEST(ServoNodeScanGate, DoesNotScanOnOperatorPause)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  node->force_hold_for_test(lgw_bt_planner::HoldReason::kOperatorPause);
  node->tick(node->now().nanoseconds());
  EXPECT_FALSE(node->scan_in_progress_for_test());
}

TEST(ServoNodeScanGate, DoesNotScanOnFollowerLag)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  node->force_hold_for_test(lgw_bt_planner::HoldReason::kFollowerLag);
  node->tick(node->now().nanoseconds());
  EXPECT_FALSE(node->scan_in_progress_for_test());
}

TEST(ServoNodeScanGate, TakeoverWithoutSelectedTargetIsScanOnly)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray empty;
  node->set_latest_array_for_test(empty);

  bool scan_only = false;
  EXPECT_TRUE(node->takeover_for_test(0, "", scan_only));
  EXPECT_TRUE(scan_only);
  EXPECT_EQ(node->state_for_test(), "PREEMPT");
}

TEST(ServoNodeScanGate, TakeoverIgnoresRequestedTrackId)
{
  // selected_track_id == 0 时，即使请求里带非零 track_id / class_filter
  // 也必须走 scan-only（旧代码会按 requested_missing 直接拒绝）。
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray empty;
  node->set_latest_array_for_test(empty);

  bool scan_only = false;
  EXPECT_TRUE(node->takeover_for_test(9, "wood_block", scan_only));
  EXPECT_TRUE(scan_only);
}

TEST(ServoNodeScanGate, TakeoverWithSelectedIdMissingFails)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 9;
  node->set_latest_array_for_test(arr);

  bool scan_only = false;
  EXPECT_FALSE(node->takeover_for_test(0, "", scan_only));
  EXPECT_FALSE(scan_only);
}

TEST(ServoNodeScanGate, ReselectFollowsSelectedId)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 5;
  yoloe_interfaces::msg::UnifiedTrack track;
  track.track_id = 5;
  track.task_state = "active";
  arr.tracks.push_back(track);
  node->set_latest_array_for_test(arr);

  node->reselect_for_test();
  EXPECT_EQ(node->selected_track_id_for_test(), 5u);
}

TEST(ServoNodeScanGate, ReselectClearsWhenSelectedMissing)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 9;
  node->set_latest_array_for_test(arr);

  node->reselect_for_test();
  EXPECT_EQ(node->selected_track_id_for_test(), 0u);
}

TEST(ServoNodeScanGate, ReselectResetsLoopAcrossGap)
{
  // A -> selected 0 (HOLD) -> B must still reset the control-law memory:
  // B must not inherit A's low-pass filter / feed-forward state.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());

  yoloe_interfaces::msg::UnifiedTrackArray a;
  a.selected_track_id = 5;
  yoloe_interfaces::msg::UnifiedTrack track_a;
  track_a.track_id = 5;
  track_a.task_state = "active";
  a.tracks.push_back(track_a);
  node->set_latest_array_for_test(a);
  node->reselect_for_test();
  EXPECT_EQ(node->reselect_loop_reset_count_for_test(), 1u);

  yoloe_interfaces::msg::UnifiedTrackArray gap;
  node->set_latest_array_for_test(gap);
  node->reselect_for_test();
  EXPECT_EQ(node->reselect_loop_reset_count_for_test(), 1u);

  yoloe_interfaces::msg::UnifiedTrackArray b;
  b.selected_track_id = 6;
  yoloe_interfaces::msg::UnifiedTrack track_b;
  track_b.track_id = 6;
  track_b.task_state = "active";
  b.tracks.push_back(track_b);
  node->set_latest_array_for_test(b);
  node->reselect_for_test();
  EXPECT_EQ(node->reselect_loop_reset_count_for_test(), 2u);
}
