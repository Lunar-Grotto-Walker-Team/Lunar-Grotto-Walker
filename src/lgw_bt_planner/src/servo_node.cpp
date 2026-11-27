// src/servo_node.cpp
//
// Entry point only. The node class lives in
// include/lgw_bt_planner/servo/servo_node.hpp so tests can link it and drive
// the control tick directly (Task 1 of the 2026-11-07 node-split plan).

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "lgw_bt_planner/servo/servo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lgw_bt_planner::ServoNode>();
  // Two threads: the 100 Hz control timer plus services/subscriptions. A
  // callback group is never added to two executors (servo_sync_group_ is
  // driven by its own local executor, see call_trigger_sync()).
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2, false);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
