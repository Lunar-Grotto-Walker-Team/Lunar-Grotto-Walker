// tests/test_evaluate_detection.cpp
// Pure-function unit tests for lgw_bt_planner::evaluate_detection.
// No ROS runtime needed; runs under build/<target> directly (per
// bt-tech-summary.md §8: this container lacks ament_cmake_test).

#include "lgw_bt_planner/evaluate_detection.hpp"

#include <gtest/gtest.h>
#include <yoloe_interfaces/msg/observation_result.hpp>
#include <yoloe_interfaces/msg/yoloe_detection3_d.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include <optional>

namespace
{
// Build a CONFIRMED ObservationResult with one synthetic detection and
// the given header.stamp. Returns nullopt if you want to test the
// "no message yet" branch.
yoloe_interfaces::msg::ObservationResult make_confirmed_msg(
    rclcpp::Time stamp,
    bool include_detection = true)
{
  yoloe_interfaces::msg::ObservationResult msg;
  msg.header.stamp = stamp;
  msg.status = yoloe_interfaces::msg::ObservationResult::CONFIRMED;
  msg.image_width = 640;
  msg.image_height = 480;
  if (include_detection) {
    yoloe_interfaces::msg::YoloeDetection3D det;
    det.class_id = 0;  // red_block
    det.class_name = "red_block";
    det.confidence = 0.05f;
    det.position_valid = true;
    det.center_m.x = 0.1;
    det.center_m.y = 0.2;
    det.center_m.z = 0.3;
    msg.detections.push_back(det);
  }
  return msg;
}
}  // namespace

// Case 1: never received a message yet → FAILURE
TEST(EvaluateDetection, NoMessageYetReturnsFailure)
{
  const auto result = lgw_bt_planner::evaluate_detection(
    std::nullopt, /*max_age_sec=*/2.5, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.ok);
}

// Case 2: status == NO_TARGET → FAILURE
TEST(EvaluateDetection, NoTargetStatusReturnsFailure)
{
  auto msg = make_confirmed_msg(rclcpp::Time(0, 0, RCL_ROS_TIME));
  msg.status = yoloe_interfaces::msg::ObservationResult::NO_TARGET;
  const auto result = lgw_bt_planner::evaluate_detection(
    msg, 2.5, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.ok);
}

// Case 3: status == ERROR → FAILURE (covers ERROR / TIMEOUT / CANCELLED too)
TEST(EvaluateDetection, ErrorStatusReturnsFailure)
{
  auto msg = make_confirmed_msg(rclcpp::Time(0, 0, RCL_ROS_TIME));
  msg.status = yoloe_interfaces::msg::ObservationResult::ERROR;
  const auto result = lgw_bt_planner::evaluate_detection(
    msg, 2.5, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.ok);
}

// Case 4: CONFIRMED but detections empty → FAILURE
TEST(EvaluateDetection, ConfirmedEmptyDetectionsReturnsFailure)
{
  auto msg = make_confirmed_msg(rclcpp::Time(0, 0, RCL_ROS_TIME),
                                /*include_detection=*/false);
  const auto result = lgw_bt_planner::evaluate_detection(
    msg, 2.5, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.ok);
}

// Case 5: CONFIRMED + detection + age 1.0s + max_age 2.5s → SUCCESS
//
// Note: rclcpp::Time(int32_t, uint32_t, rcl_clock_type_t) takes seconds
// (not nanoseconds), so `rclcpp::Time(1, 0, RCL_ROS_TIME)` = epoch+1 s.
// With msg_stamp = 0 and now = 1 s, age = 1 s, which is well under
// max_age_sec = 2.5.
TEST(EvaluateDetection, FreshConfirmedDetectionReturnsSuccess)
{
  auto msg = make_confirmed_msg(rclcpp::Time(0, 0, RCL_ROS_TIME));
  const auto result = lgw_bt_planner::evaluate_detection(
    msg, 2.5, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_TRUE(result.ok);
}

// Case 6: CONFIRMED + detection + age 3.0s + max_age 2.5s → FAILURE
TEST(EvaluateDetection, StaleConfirmedDetectionReturnsFailure)
{
  auto msg = make_confirmed_msg(rclcpp::Time(0, 0, RCL_ROS_TIME));
  const auto result = lgw_bt_planner::evaluate_detection(
    msg, 2.5, rclcpp::Time(3, 0, RCL_ROS_TIME));
  EXPECT_FALSE(result.ok);
}
