// include/lgw_bt_planner/compute_grasp_poses_from_track.hpp
//
// Two tcp poses for the collect step, expressed in the MoveIt model frame
// (default `base_link`, unprefixed) as of 2026-11-18 — TF keeps the arm side
// prefixed (`arm1/base_link`) to stay distinct from Go2's base_link, but
// move_group's URDF model has no prefix, so a prefixed frame in a goal
// constraint makes the planner report "Invalid goal state".
//
//
//   grasp_pose      tcp position = selected target's position_arm_base
//                   orientation  = the tcp orientation the servo held while
//                                  looking at it (current TF tcp_link)
//   pre_grasp_pose  same orientation, position = target + pre_grasp_distance
//                   along the arm camera's optical axis, on the camera side
//
// The optical axis is R_tcp * R_cam_tcp * (0,0,1); the hand-eye JSON shipped
// with the package stores camera->tcp, so no extra calibration is needed.
//
// Uses TF (arm_base <- tcp) rather than /feedback/tcp_pose: the driver's
// topic has an empty frame_id and SDK timestamps (spec 2026-10-26 D19).
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yoloe_interfaces/msg/unified_track_array.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace lgw_bt_planner
{

class ComputeGraspPosesFromTrack : public BT::SyncActionNode
{
public:
  ComputeGraspPosesFromTrack(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void on_track_array(
    const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg);

  std::shared_ptr<rclcpp::Subscription<
      yoloe_interfaces::msg::UnifiedTrackArray>> sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  yoloe_interfaces::msg::UnifiedTrackArray latest_;
  bool seen_{false};

  // camera -> tcp rotation from arm_calibration.json (hand-eye JSON).
  tf2::Matrix3x3 R_cam_tcp_;
  bool calibration_ok_{false};

  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.compute_grasp_poses_from_track")};
};

}  // namespace lgw_bt_planner
