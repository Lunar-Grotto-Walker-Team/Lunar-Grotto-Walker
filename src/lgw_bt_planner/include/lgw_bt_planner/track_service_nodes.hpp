// include/lgw_bt_planner/track_service_nodes.hpp
//
// Thin BT service shells for the track-collect validation chain (2026-11-17):
//   ServoTakeover  -> /lgw/servo/takeover  (servo follows selected_track_id)
//   ServoRelease   -> /lgw/servo/release
//   MarkTrack      -> /lgw/tracker/mark_track (1=consume, 2=ignore, 4=request)
#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>

#include <lgw_bt_planner/srv/takeover.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yoloe_interfaces/srv/mark_track.hpp>

#include <rclcpp/logger.hpp>

#include <string>

#include "lgw_bt_planner/target_track_nodes.hpp"

namespace lgw_bt_planner
{

class ServoTakeoverNode
  : public BT::RosServiceNode<lgw_bt_planner::srv::Takeover>
{
public:
  using Service = lgw_bt_planner::srv::Takeover;

  ServoTakeoverNode(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr & req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr & resp) override;

private:
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.servo_takeover")};
};

class ServoReleaseNode
  : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  using Service = std_srvs::srv::Trigger;

  ServoReleaseNode(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr & req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr & resp) override;

private:
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.servo_release")};
};

class MarkTrackNode
  : public BT::RosServiceNode<yoloe_interfaces::srv::MarkTrack>
{
public:
  using Service = yoloe_interfaces::srv::MarkTrack;

  MarkTrackNode(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr & req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr & resp) override;

private:
  TrackArrayCache cache_;
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.mark_track")};
};

}  // namespace lgw_bt_planner
