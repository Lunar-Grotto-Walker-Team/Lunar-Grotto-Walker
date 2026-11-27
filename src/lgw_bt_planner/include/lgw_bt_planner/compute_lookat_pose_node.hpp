// include/lgw_bt_planner/compute_lookat_pose_node.hpp
//
// ComputeLookatPose BT node + pure lookat math utilities.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <memory>
#include <string>
#include <utility>

namespace lgw_bt_planner
{

// === Pure functions (Tasks 2-5) ===

std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& path);

tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& direction,
  const tf2::Vector3& world_up);

// Ensures cam_y · world_up < 0 (camera upright) by rotating 180° about
// the optical axis (column 2). Returns R unchanged when already upright.
// Used by ComputeLookatPoseNode / ComputeGraspPose and the Servo
// lookat-setpoint path (they must all command the same orientation).
tf2::Matrix3x3 apply_uprightness_flip(
  const tf2::Matrix3x3& R,
  const tf2::Vector3& world_up);

geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& P_ee_in_base,
  const tf2::Matrix3x3& R_arm_in_base,
  const tf2::Matrix3x3& R_arm_in_tcp,
  const tf2::Vector3& t_arm_in_tcp);

// === BT node (Task 6) ===

// SyncActionNode that takes a head ObservationResult + the desired
// arm-camera-origin world position + 2 handeye calibration JSONs
// (loaded at construction time) and writes a PoseStamped for the
// tcp_link onto the blackboard.
//
// Construction: loads 2 calibration JSONs once. On any load failure,
// sets `initialized_ = false` and `onTick` always returns FAILURE.
// This keeps the node safe under Repeat num_cycles="-1" — it
// degrades gracefully instead of crashing.
class ComputeLookatPoseNode : public BT::SyncActionNode
{
public:
  ComputeLookatPoseNode(const std::string& name,
                        const BT::NodeConfig& config);

  ComputeLookatPoseNode() = delete;
  ComputeLookatPoseNode(const ComputeLookatPoseNode&) = delete;
  ComputeLookatPoseNode& operator=(const ComputeLookatPoseNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  tf2::Matrix3x3 R_h2b_;     // head optical in base
  tf2::Vector3   t_h2b_;
  tf2::Matrix3x3 R_a2t_;     // arm optical in tcp
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner
