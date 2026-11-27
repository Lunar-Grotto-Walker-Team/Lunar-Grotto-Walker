// compute_arm_target_world.hpp — BT node that transforms an arm-camera
// ObservationResult (center_m in camera_arm_color_optical_frame) into
// the world frame using:
//   1. T_camera -> tcp_link from arm_calibration.json (applied directly,
//      NOT inverted — see convention note in the .cpp file-top comment)
//   2. T_tcp_in_world supplied externally as tcp_pose_in_world (PoseStamped
//      from blackboard, e.g. {head_pose} written by an earlier
//      MoveArmToPose+lookat step)
//
// Math: P_arm_in_tcp = R_a2t · P_arm_optical + t_a2t
//       P_arm_world   = R_tcp · P_arm_in_tcp + t_tcp
//
// Output: writes geometry_msgs/msg/Point to blackboard under
// "arm_target_world" (overridable via output_key). Always SUCCESS unless
// calibration load failed or any input is empty/malformed.
//
// Used by head_arm_independent_lookat.xml step 2. The "tcp_pose_in_world"
// is the assumed tcp pose at the moment this node ticks — typically the
// result of step 1's MoveArmToPose, which is the head-perspective lookat
// pose. By feeding step 1's pose here, step 2 can express the arm-camera
// observation in the world frame anchored at step 1's physical location.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <string>

namespace lgw_bt_planner
{

class ComputeArmTargetWorldNode : public BT::SyncActionNode
{
public:
  ComputeArmTargetWorldNode(const std::string& name,
                            const BT::NodeConfig& config);

  ComputeArmTargetWorldNode() = delete;
  ComputeArmTargetWorldNode(const ComputeArmTargetWorldNode&) = delete;
  ComputeArmTargetWorldNode& operator=(const ComputeArmTargetWorldNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  // T_camera_arm_color_optical_frame -> tcp_link (the _tcp.json variant,
  // which is the arm_calibration.json file in share/calibration/). The
  // _meta.transformation field used to read "tcp_link -> camera_..." but
  // that was inconsistent with the actual data; corrected 2026-10-09.
  tf2::Matrix3x3 R_a2t_;
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner
