#include "lgw_bt_planner/set_pose_stamped_node.hpp"

#include "lgw_bt_planner/pose_string_parser.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>

namespace lgw_bt_planner
{

BT::NodeStatus SetPoseStampedNode::tick()
{
  std::string output_key;
  if (!getInput("output_key", output_key)) {
    throw BT::RuntimeError("SetPoseStamped: missing port [output_key]");
  }
  std::string value_str;
  if (!getInput("value", value_str)) {
    throw BT::RuntimeError("SetPoseStamped: missing port [value]");
  }

  geometry_msgs::msg::PoseStamped pose;
  try {
    pose = parse_pose_string(value_str);
  } catch (const std::exception& e) {
    throw BT::RuntimeError(
      std::string("SetPoseStamped: failed to parse [") + value_str +
      "]: " + e.what());
  }

  config().blackboard->set(output_key, pose);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetPoseStampedNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "value",
      "PoseStamped literal: \"frame;x;y;z;qx;qy;qz;qw\" (semicolon-separated, "
      "missing trailing fields default to 0/identity)."),
    BT::BidirectionalPort<std::string>(
      "output_key",
      "Blackboard key under which the typed PoseStamped is written.")
  };
}

}  // namespace lgw_bt_planner