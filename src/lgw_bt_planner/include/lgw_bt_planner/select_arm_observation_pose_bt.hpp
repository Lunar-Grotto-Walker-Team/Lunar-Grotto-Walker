// select_arm_observation_pose_bt.hpp — BT thin shell.
//
// 通过 /select_arm_observation_pose service 调独立 service 节点做 arm
// 观测位姿选择;本类只负责 setRequest / onResponseReceived 两个回调 +
// 端口声明。
//
// 设计依据:docs/superpowers/specs/2026-10-09-select-arm-observation-pose-spec.md §4.8

#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>

#include "lgw_bt_planner/srv/select_arm_observation_pose.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <string>

namespace lgw_bt_planner
{

class SelectArmObservationPoseBTNode
  : public BT::RosServiceNode<
      lgw_bt_planner::srv::SelectArmObservationPose>
{
public:
  using Service = lgw_bt_planner::srv::SelectArmObservationPose;

  SelectArmObservationPoseBTNode(const std::string& name,
                                  const BT::NodeConfig& cfg,
                                  const BT::RosNodeParams& params);

  SelectArmObservationPoseBTNode() = delete;
  SelectArmObservationPoseBTNode(
    const SelectArmObservationPoseBTNode&) = delete;
  SelectArmObservationPoseBTNode& operator=(
    const SelectArmObservationPoseBTNode&) = delete;

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr& req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr& resp) override;
};

}  // namespace lgw_bt_planner
