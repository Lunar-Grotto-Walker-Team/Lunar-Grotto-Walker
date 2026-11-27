// include/lgw_bt_planner/compute_grasp_poses_sampled.hpp
//
// 2026-11-20: sampled grasp / pre-grasp pose generation.
//
// Replaces the "one analytic pose" of ComputeGraspPosesFromTrack with a small
// deterministic search:
//
//   yaw    = azimuth of the base -> target horizontal line (+ perturbations)
//   pitch  = tilt of the tool axis away from straight down, scanned over a
//            grid (0..150 deg). Below-base targets are tried vertical-first;
//            above-base targets are tried "pointing at the target"-first.
//   roll   = fixed by construction so the jaw axis stays horizontal (the
//            hand-eye calibration has camera-x perpendicular to the tool axis,
//            so the image stays upright for every yaw at pitch 0).
//   position: grasp = target centre; pre-grasp = target - d * pre_distance.
//
// Each (pitch, yaw) candidate costs two /compute_ik calls (~1 ms each) with
// avoid_collisions=true, so the goal state is also checked against MoveIt's
// planning scene. The first candidate whose BOTH poses solve is written to
// the blackboard; the tree still executes it with MoveArmToPose (MoveIt stays
// the authority on the path).
//
// If nothing solves the node returns FAILURE and the tree's absent branch
// does the recovery. No planning calls are made here: a single -27 style goal
// is what wedges move_group, and the IK check is what keeps it away.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yoloe_interfaces/msg/unified_track_array.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lgw_bt_planner
{

class ComputeGraspPosesSampled : public BT::SyncActionNode
{
public:
  ComputeGraspPosesSampled(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  using IkSrv = moveit_msgs::srv::GetPositionIK;
  using MoveGroupAction = moveit_msgs::action::MoveGroup;

  void on_track_array(
    const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg);
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // One /compute_ik call. Returns true when MoveIt reports SUCCESS.
  bool solveIk(
    const tf2::Vector3 & p, const tf2::Matrix3x3 & R,
    const std::string & frame);

  // plan_only MoveGroup check for one tcp pose. Cheap-ish (0.1-1 s) and the
  // only way to know the PATH is feasible, not just the goal state.
  bool planOnly(
    const tf2::Vector3 & p, const tf2::Matrix3x3 & R,
    const std::string & frame);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<
      yoloe_interfaces::msg::UnifiedTrackArray>> track_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> js_sub_;
  rclcpp::Client<IkSrv>::SharedPtr ik_client_;
  rclcpp::CallbackGroup::SharedPtr ik_cb_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ik_executor_;
  rclcpp_action::Client<MoveGroupAction>::SharedPtr plan_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  yoloe_interfaces::msg::UnifiedTrackArray latest_;
  bool seen_{false};
  sensor_msgs::msg::JointState seed_js_;
  bool have_js_{false};

  // Per-tick inputs (ports) cached for solveIk().
  int ik_timeout_ms_{60};
  int plan_check_k_{2};
  int plan_timeout_ms_{1000};
  std::string ik_group_{"arm"};
  // 2026-11-20: /compute_ik hands ik_link_name straight to the KDL plugin,
  // whose chain ends at the group tip (link6) — asking for tcp_link is
  // rejected with INVALID_LINK_NAME. The request therefore carries the LINK6
  // pose (tcp pose minus the fixed 12 cm tcp offset); the MoveGroup plan
  // check below still uses tcp_link, because MoveGroup resolves that itself.
  std::string ik_link_{"link6"};
  double tcp_offset_z_m_{0.12};
  std::string plan_link_{"tcp_link"};

  // 2026-11-20 diagnostics: every /compute_ik reply's error_code, counted per
  // tick. Without this "no IK solution for 77 candidates" is unactionable
  // (NO_IK_SOLUTION vs INVALID_LINK_NAME vs GOAL_IN_COLLISION are entirely
  // different problems). Cleared at the start of each tick().
  std::map<int, int> ik_code_counts_;
  int ik_last_code_{0};
  bool ik_timeout_seen_{false};

  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.compute_grasp_poses_sampled")};
};

}  // namespace lgw_bt_planner
