// src/track_service_nodes.cpp — see the header for the contracts.

#include "lgw_bt_planner/track_service_nodes.hpp"

#include <rclcpp/logging.hpp>

#include <cstdint>

namespace lgw_bt_planner
{

// ---------------------------------------------------------------------------
// ServoTakeover
// ---------------------------------------------------------------------------

ServoTakeoverNode::ServoTakeoverNode(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<Service>(name, config, params)
{
  if (auto node = params.nh.lock()) {
    logger_ = node->get_logger();
  }
}

BT::PortsList ServoTakeoverNode::providedPorts()
{
  return providedBasicPorts({});
}

bool ServoTakeoverNode::setRequest(typename Service::Request::SharedPtr & req)
{
  // The servo ignores these fields (2026-11-17): it follows
  // UnifiedTrackArray.selected_track_id. Populate them for old callers.
  req->track_id = 0;
  req->class_filter.clear();
  return true;
}

BT::NodeStatus ServoTakeoverNode::onResponseReceived(
  const typename Service::Response::SharedPtr & resp)
{
  if (!resp->success) {
    RCLCPP_WARN(
      logger_, "[%s] takeover rejected: %s",
      name().c_str(), resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    logger_, "[%s] takeover accepted: state=%s",
    name().c_str(), resp->state.c_str());
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// ServoRelease
// ---------------------------------------------------------------------------

ServoReleaseNode::ServoReleaseNode(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<Service>(name, config, params)
{
  if (auto node = params.nh.lock()) {
    logger_ = node->get_logger();
  }
}

BT::PortsList ServoReleaseNode::providedPorts()
{
  return providedBasicPorts({});
}

bool ServoReleaseNode::setRequest(typename Service::Request::SharedPtr & /*req*/)
{
  return true;
}

BT::NodeStatus ServoReleaseNode::onResponseReceived(
  const typename Service::Response::SharedPtr & resp)
{
  if (!resp->success) {
    RCLCPP_WARN(
      logger_, "[%s] release rejected: %s",
      name().c_str(), resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    logger_, "[%s] release accepted: %s",
    name().c_str(), resp->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// MarkTrack
// ---------------------------------------------------------------------------

MarkTrackNode::MarkTrackNode(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::RosServiceNode<Service>(name, config, params)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("MarkTrackNode: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  cache_.init(node.get());
}

BT::PortsList MarkTrackNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int64_t>(
      "track_id", 0, "0 = use the tracker's current selected_track_id"),
    BT::InputPort<int>(
      "action", 1,
      "1=consume, 2=ignore, 4=request (track_id=0 releases), "
      "5=clear arm-camera hit history (state unchanged), "
      "6=pin base circle centre at the robot's current odom pose"),
  });
}

bool MarkTrackNode::setRequest(typename Service::Request::SharedPtr & req)
{
  int action = 1;
  if (!getInput("action", action)) {
    RCLCPP_ERROR(logger_, "[%s] required input 'action' missing", name().c_str());
    return false;
  }
  // Action 6 = PIN_BASE_CENTER: 不需要 track_id, 也不涉及选中目标。
  if (action == 6) {
    req->track_id = 0;
    req->action = static_cast<uint8_t>(action);
    return true;
  }
  int64_t track_id = 0;
  getInput("track_id", track_id);
  if (track_id == 0) {
    yoloe_interfaces::msg::UnifiedTrackArray arr;
    if (cache_.snapshot(&arr)) {
      track_id = static_cast<int64_t>(arr.selected_track_id);
    }
  }
  // Action 4 with track_id 0 is the explicit "release selection" request.
  if (track_id == 0 && action != 4) {
    RCLCPP_WARN(
      logger_, "[%s] no track_id and no selected track; nothing to mark",
      name().c_str());
    return false;
  }
  req->track_id = static_cast<uint64_t>(track_id);
  req->action = static_cast<uint8_t>(action);
  return true;
}

BT::NodeStatus MarkTrackNode::onResponseReceived(
  const typename Service::Response::SharedPtr & resp)
{
  if (!resp->success) {
    RCLCPP_WARN(
      logger_, "[%s] mark rejected: %s",
      name().c_str(), resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    logger_, "[%s] %s", name().c_str(), resp->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
