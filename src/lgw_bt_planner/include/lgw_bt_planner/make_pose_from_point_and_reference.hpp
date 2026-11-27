// make_pose_from_point_and_reference.hpp — trivial BT node that composes a
// PoseStamped from a world-frame Point and a reference PoseStamped,
// inheriting the reference's orientation. Used by
// head_arm_independent_lookat.xml Phase B and Phase C to drive tcp to
// head_target_world / arm_target_world while preserving Phase A's lookat
// orientation (so the user can compare endpoint positions under a fixed
// gripper pose, isolating the geometric bias from the orientation recompute
// that ComputeGraspPose would otherwise apply).
//
// Pure data-shape node — no transforms, no math. Always SUCCESS unless the
// `point` or `reference_pose` port is unset.

#pragma once

#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace lgw_bt_planner
{

class MakePoseFromPointAndReferenceNode : public BT::SyncActionNode
{
public:
  MakePoseFromPointAndReferenceNode(const std::string& name,
                                    const BT::NodeConfig& config);

  MakePoseFromPointAndReferenceNode() = delete;
  MakePoseFromPointAndReferenceNode(const MakePoseFromPointAndReferenceNode&) = delete;
  MakePoseFromPointAndReferenceNode& operator=(
    const MakePoseFromPointAndReferenceNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace lgw_bt_planner
