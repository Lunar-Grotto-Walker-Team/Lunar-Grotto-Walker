#pragma once

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace lgw_bt_planner
{

// SyncActionNode that parses a PoseStamped literal (format:
//   "frame_id;x;y;z;qx;qy;qz;qw", missing fields default to identity frame +
//   zero translation + identity orientation) and writes the typed
// geometry_msgs::msg::PoseStamped onto the blackboard under output_key.
//
// This exists because BT::SetBlackboardNode cannot produce a typed
// geometry_msgs::msg::PoseStamped from a string literal in this build:
// convertFromString<PoseStamped> is a template specialization in
// lgw_bt_planner_nodes.so but SetBlackboardNode::tick() is already
// compiled in libbehaviortree_cpp.so with the default throwing
// specialization, and the user's spec does not cross the .so boundary.
//
// SetPoseStampedNode sidesteps that by parsing the string itself and
// calling blackboard->set(key, typed_pose) directly.
class SetPoseStampedNode : public BT::SyncActionNode
{
public:
  SetPoseStampedNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};

}  // namespace lgw_bt_planner