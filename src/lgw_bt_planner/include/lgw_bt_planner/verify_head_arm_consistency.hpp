// verify_head_arm_consistency.hpp — BT node that verifies whether the
// head-camera ObservationResult and the arm-camera ObservationResult point
// at the same world-frame target, by transforming both centers via the
// hand-eye calibration JSONs into the world (== base_link) frame and
// comparing Euclidean distance against a configurable threshold.
//
// Loads `head_calibration.json` (T_base_link → camera_head_color_optical_frame)
// and `arm_calibration.json` (T_tcp_link → camera_arm_color_optical_frame,
// _tcp.json variant) once at construction; on any load failure, sets
// `initialized_ = false` and `onTick` always returns FAILURE (matches the
// "degrade gracefully under Repeat num_cycles=-1" pattern of
// ComputeLookatPoseNode).
//
// On SUCCESS, writes two keys to the global blackboard for downstream
// nodes (ComputeGraspPose):
//   - "arm_target_world"  (geometry_msgs/Point) — arm center in world frame
//   - "consistency_error_m" (double)            — ||head_world − arm_world||

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <string>

namespace lgw_bt_planner
{

class VerifyHeadArmConsistencyNode : public BT::SyncActionNode
{
public:
  VerifyHeadArmConsistencyNode(const std::string& name,
                               const BT::NodeConfig& config);

  VerifyHeadArmConsistencyNode() = delete;
  VerifyHeadArmConsistencyNode(const VerifyHeadArmConsistencyNode&) = delete;
  VerifyHeadArmConsistencyNode& operator=(const VerifyHeadArmConsistencyNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  // head optical in base
  tf2::Matrix3x3 R_h2b_;
  tf2::Vector3   t_h2b_;
  // arm optical in tcp
  tf2::Matrix3x3 R_a2t_;
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner
