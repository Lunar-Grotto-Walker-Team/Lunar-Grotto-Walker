// wait_for_user_input_bt_node.hpp — snapshot-style BT.ROS2
// RosTopicSubNode<Bool> leaf. See .cpp for semantics.

#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace lgw_bt_planner
{

// Snapshot-style node: on each Reactive* tick the node is reset, onStart
// runs and immediately returns SUCCESS if the latest /bt_user_input
// std_msgs/Bool has data==true, FAILURE otherwise. Never RUNNING — the
// caller must be a reactive control node (e.g. ReactiveFallback) that
// re-evaluates children each tick; a static Sequence/Sequence would lock on
// the first FAILURE and never re-enter it.
class WaitForUserInputActionNode
  : public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  WaitForUserInputActionNode(const std::string& name,
                             const BT::NodeConfig& cfg,
                             const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, cfg, params) {}

  WaitForUserInputActionNode() = delete;
  WaitForUserInputActionNode(const WaitForUserInputActionNode&) = delete;
  WaitForUserInputActionNode& operator=(const WaitForUserInputActionNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(
    const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
};

}  // namespace lgw_bt_planner