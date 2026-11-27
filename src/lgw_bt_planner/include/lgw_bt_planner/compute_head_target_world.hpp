// compute_head_target_world.hpp — BT node that transforms a head-camera
// ObservationResult (center_m in camera_head_color_optical_frame) into
// the world frame using T_base → head_optical from head_calibration.json
// (parse_calibration_json via compute_lookat_pose_node.hpp).
//
// Output: writes a geometry_msgs/msg/Point to the blackboard under
// "head_target_world" (overridable via output_key port). Always SUCCESS
// unless calibration failed at construction or the observation has no
// detections — in which case FAILURE.
//
// Used by head_arm_independent_lookat.xml: lets the tree drive tcp
// straight to the head-camera's view of the target without any arm-side
// cross-check (the user judges geometric errors by visually inspecting
// where tcp ends up).

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/point.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <string>

namespace lgw_bt_planner
{

class ComputeHeadTargetWorldNode : public BT::SyncActionNode
{
public:
  ComputeHeadTargetWorldNode(const std::string& name,
                             const BT::NodeConfig& config);

  ComputeHeadTargetWorldNode() = delete;
  ComputeHeadTargetWorldNode(const ComputeHeadTargetWorldNode&) = delete;
  ComputeHeadTargetWorldNode& operator=(const ComputeHeadTargetWorldNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  // T_base_link → camera_head_color_optical_frame
  tf2::Matrix3x3 R_h2b_;
  tf2::Vector3   t_h2b_;
};

}  // namespace lgw_bt_planner
