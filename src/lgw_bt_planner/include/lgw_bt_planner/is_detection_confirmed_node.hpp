// is_detection_confirmed_node.hpp — two BT.ROS2 RosTopicSubNode
// leaf nodes that condition on YOLOE head/arm ObservationResult.
//
// Both inherit BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>
// so BT.ROS2 manages subscription lifecycle and last_msg caching.
// onTick() defers the four-condition check to
// lgw_bt_planner::evaluate_detection() (Task 1), then writes the
// observation to the blackboard's `last_observation` port on SUCCESS.
//
// NOTE on onTick signature: BT.ROS2 v0.3.0's RosTopicSubNode declares
//   virtual NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg) = 0;
// (see bt_topic_sub_node.hpp:142). The original Task 2 brief wrote
// std::optional<...> which would not override the base; we follow the
// BT.ROS2 contract and convert to std::optional inside the body before
// calling evaluate_detection().
//
// NOTE on providedPorts: per the same header, subscribers look up the
// "topic_name" InputPort only when providedPorts() routes through
// providedBasicPorts({...}). Without it, the per-instance topic_name
// remap (used to differentiate /yoloe/head vs /yoloe/arm) is silently
// ignored and every node would subscribe to params.default_port_value.

#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <memory>
#include <optional>
#include <string>

namespace lgw_bt_planner
{

class IsHeadDetectionConfirmed
  : public BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>
{
public:
  IsHeadDetectionConfirmed(const std::string& name,
                           const BT::NodeConfig& config,
                           const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>(
        name, config, params) {}

  IsHeadDetectionConfirmed() = delete;
  IsHeadDetectionConfirmed(const IsHeadDetectionConfirmed&) = delete;
  IsHeadDetectionConfirmed& operator=(const IsHeadDetectionConfirmed&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(
    const std::shared_ptr<yoloe_interfaces::msg::ObservationResult>& last_msg)
    override;
};

class IsArmDetectionConfirmed
  : public BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>
{
public:
  IsArmDetectionConfirmed(const std::string& name,
                          const BT::NodeConfig& config,
                          const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>(
        name, config, params) {}

  IsArmDetectionConfirmed() = delete;
  IsArmDetectionConfirmed(const IsArmDetectionConfirmed&) = delete;
  IsArmDetectionConfirmed& operator=(const IsArmDetectionConfirmed&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(
    const std::shared_ptr<yoloe_interfaces::msg::ObservationResult>& last_msg)
    override;
};

}  // namespace lgw_bt_planner