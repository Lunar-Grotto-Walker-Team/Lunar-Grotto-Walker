// tests/test_servo_params.cpp
//
// V4 parameter snapshot (2026-11-07 node-split plan, Task 3b). Pins the values
// the shipped servo_node.yaml actually delivers, so a default that silently
// moves is caught here instead of on the robot.

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "lgw_bt_planner/servo/servo_params.hpp"

namespace
{

// servo_node.yaml is organised by node name, so the probe node must carry the
// real name or every override is silently ignored.
rclcpp::Node::SharedPtr node_with_params_file(const std::string & yaml)
{
  return std::make_shared<rclcpp::Node>(
    "lgw_servo_node",
    rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", yaml}));
}

}  // namespace

TEST(ServoParams, ScalarDefaultsMatchTheSource)
{
  // Plain defaults, no params file: these are the values the node would use if
  // the yaml vanished. Update deliberately, never accidentally.
  lgw_bt_planner::ServoParams p;
  EXPECT_EQ(p.frame_prefix, "arm1/");
  EXPECT_EQ(p.pose_watch_topic, "/control/joint_states");
  EXPECT_FALSE(p.follow_check_enabled);
  EXPECT_DOUBLE_EQ(p.follow_tolerance_rad, 0.15);
  EXPECT_EQ(p.follow_timeout_ms, 300);
  EXPECT_DOUBLE_EQ(p.control.kp_pos, 2.0);
  EXPECT_DOUBLE_EQ(p.control.kp_ang, 3.0);
  EXPECT_DOUBLE_EQ(p.control.kd_pos, 0.30);
  EXPECT_DOUBLE_EQ(p.control.kd_ang, 0.30);
}

TEST(ServoParams, ShippedYamlOverridesReachTheStruct)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  // Resolve through the ament index: a hardcoded install path would follow
  // whichever checkout happens to sit there, and a stale copy would make this
  // test pass against the wrong file.
  const auto yaml =
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/config/servo_node.yaml";
  auto node = node_with_params_file(yaml);

  lgw_bt_planner::ServoParams p;
  // The node assigns through reference members today; emulate that by reading
  // the parameters the same way the node does.
  p.control.kp_pos = node->declare_parameter<double>("kp_pos", p.control.kp_pos);
  p.control.kp_ang = node->declare_parameter<double>("kp_ang", p.control.kp_ang);
  p.control.kd_pos = node->declare_parameter<double>("kd_pos", p.control.kd_pos);
  p.follow_timeout_ms =
    node->declare_parameter<int>("follow_timeout_ms", p.follow_timeout_ms);

  EXPECT_DOUBLE_EQ(p.control.kp_pos, 3.0);
  EXPECT_DOUBLE_EQ(p.control.kp_ang, 5.0);
  EXPECT_DOUBLE_EQ(p.control.kd_pos, 0.30);
  EXPECT_EQ(p.follow_timeout_ms, 8000);
}
