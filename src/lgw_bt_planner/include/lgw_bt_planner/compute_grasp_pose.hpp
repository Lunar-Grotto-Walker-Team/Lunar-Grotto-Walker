// compute_grasp_pose.hpp — BT node that computes a tcp_link pose for the
// grasp step: tcp position == arm_target_world (Point from the previous
// VerifyHeadArmConsistency node's blackboard write), tcp orientation ==
// the lookat orientation (arm optical axis pointing at the target) rebuilt
// around the new anchor (ee_position_world → arm_target_world).
//
// Why not reuse lookat_pose.orientation directly? Because lookat_pose was
// computed for `ee_position_world` as the camera anchor; arm optical is
// `R_tcp · t_a2t + t_tcp` offset from tcp, so reusing lookat_pose.orientation
// while moving tcp to arm_target_world would mis-point the camera. We
// recompute lookat direction with the same anchor (ee_position_world) so
// the camera stays aimed at arm_target_world, then take R_tcp = R_arm·R_a2t⁻¹
// as the tcp orientation.
//
// Like VerifyHeadArmConsistency, loads `head_calibration.json` and
// `arm_calibration.json` (the latter MUST be the _tcp.json variant) once
// at construction; on load failure, sets `initialized_ = false` and
// `onTick` always returns FAILURE.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <string>

namespace lgw_bt_planner
{

class ComputeGraspPoseNode : public BT::SyncActionNode
{
public:
  ComputeGraspPoseNode(const std::string& name,
                       const BT::NodeConfig& config);

  ComputeGraspPoseNode() = delete;
  ComputeGraspPoseNode(const ComputeGraspPoseNode&) = delete;
  ComputeGraspPoseNode& operator=(const ComputeGraspPoseNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  // arm optical in tcp (only R_a2t used; t_a2t loaded for symmetry / debug
  // but unused in the grasp math — tcp goes directly to arm_target_world).
  tf2::Matrix3x3 R_h2b_;
  tf2::Vector3   t_h2b_;
  tf2::Matrix3x3 R_a2t_;
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner
