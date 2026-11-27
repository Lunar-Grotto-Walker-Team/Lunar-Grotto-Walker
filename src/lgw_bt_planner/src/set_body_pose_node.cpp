// set_body_pose_node.cpp
//
// BT thin shell over body_pose_bridge::srv::SetBodyPose. Pairs with
// body_pose_bridge_node. See header for design rationale.
//
// 2026-10-13 v7 简化:Go2 固件对 BodyHeight api(1013)bh_off 不响应
// (真机验证),body_height 字段从 .srv 移除,本节点也不再提供该 port。

#include "lgw_bt_planner/set_body_pose_node.hpp"

#include <rclcpp/logging.hpp>

namespace lgw_bt_planner
{

SetBodyPoseNode::SetBodyPoseNode(
  const std::string& name, const BT::NodeConfig& cfg,
  const BT::RosNodeParams& params)
: BT::RosServiceNode<Service>(name, cfg, params)
{
}

BT::PortsList SetBodyPoseNode::providedPorts()
{
  // providedBasicPorts() is REQUIRED — exposes `service_name` port
  // used by BT.ROS2 v0.3.0 RosServiceNode (see header line 94 in
  // bt_service_node.hpp; real-hardware 2026-10-09 first-send_goal
  // crash without it).
  return providedBasicPorts({
    BT::InputPort<double>("roll", 0.0,
      "Target roll (rad). Range [-0.75, +0.75]; out-of-range CLAMPED."),
    BT::InputPort<double>("pitch", 0.0,
      "Target pitch (rad). Range [-0.75, +0.75]; CLAMPED. "
      "Go2 real-hardware convention: pitch>0 = nose DOWN (INVERTED "
      "from standard right-hand rule, verified 2026-10-10)."),
    BT::InputPort<double>("yaw", 0.0,
      "Target yaw (rad). Range [-pi, +pi]; CLAMPED."),
    BT::InputPort<bool>("clear", false,
      "If true, bridge cancels its 50Hz republish and recenters the "
      "dog to (0,0,0). rpy fields are IGNORED when clear=true. "
      "Use before StandDown to release the /api/sport/request channel."),
    BT::InputPort<double>("tolerance_rpy", 0.05,
      "Per-axis |current - target| threshold for rpy (rad). "
      "Passed through to bridge; 0 or negative disables."),
  });
}

bool SetBodyPoseNode::setRequest(
  typename Service::Request::SharedPtr& req)
{
  // BT.CPP InputPort default "" trap (see feedback_bt_inputport_default.md):
  // getInput<T>() returns Optional<T>{value} when default provided. For
  // numeric types (double / bool) the Optional<T> predicate is truthy as
  // long as the port exists in the XML, so we can rely on `if (p)` here.
  if (auto p = getInput<double>("roll"); p) {
    req->roll = *p;
  }
  if (auto p = getInput<double>("pitch"); p) {
    req->pitch = *p;
  }
  if (auto p = getInput<double>("yaw"); p) {
    req->yaw = *p;
  }
  if (auto p = getInput<bool>("clear"); p) {
    req->clear = *p;
  }
  // tolerance_rpy 字段当前 bridge 暂时用默认(0.05),不传入。
  // 真机回归稳定后再加 user-tunable 透传(参见 spec §4.3 端口表注释)。
  return true;
}

BT::NodeStatus SetBodyPoseNode::onResponseReceived(
  const typename Service::Response::SharedPtr& resp)
{
  if (!resp->success) {
    RCLCPP_ERROR(rclcpp::get_logger("lgw_bt_planner.set_body_pose"),
      "service success=false, message='%s'", resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("lgw_bt_planner.set_body_pose"),
    "service success=true, message='%s'", resp->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner