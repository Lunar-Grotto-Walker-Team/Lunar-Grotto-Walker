#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <mutex>

namespace lgw_bt_planner
{

class MoveArmToPoseActionNode
  : public BT::RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
  MoveArmToPoseActionNode(const std::string& name, const BT::NodeConfig& cfg,
                          const BT::RosNodeParams& params)
    : BT::RosActionNode<moveit_msgs::action::MoveGroup>(name, cfg, params) {}

  bool setGoal(moveit_msgs::action::MoveGroup::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  // 2026-11-18: ABORTED / CANCELED / GOAL_REJECTED_BY_SERVER / SEND_GOAL_TIMEOUT
  // 走的是 onFailure() 而不是 onResultReceived()。bt_ros2 基类只 return
  // FAILURE 不打日志，于是 MoveIt 的规划失败原因（moveit_error_code）被完全
  // 吞掉 —— 真机上表现为"抓取那一步空转 5 秒然后静默失败"。
  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult>& wr) override;
  static BT::PortsList providedPorts();

private:
  // Lazy-init the joint-state subscription so setGoal() can seed IK with
  // the latest joint positions. Without a seed the KDL solver converges to
  // whichever basin it lands in, producing different IK solutions across
  // BT cycles and visible joint drift between repeated goal-sends.
  void ensure_joint_state_sub();

  // Lazy-init the /feedback/tcp_pose subscription. Caches every 200 Hz
  // PoseStamped from agx_arm_ctrl_single_node into latest_tcp_pose_. After
  // MoveGroup reports SUCCEEDED, onResultReceived() blocks for
  // kSettleWindowMs then samples the latest cached tcp_pose and writes
  // it to blackboard under output_key (default 'actual_tcp_pose'). This
  // lets downstream nodes (e.g. ComputeArmTargetWorld.tcp_pose_in_world)
  // consume the *actual* tcp pose — not the setGoal target — eliminating
  // the 1-5 cm IK-error + the 8-35 cm "Phase A goal reused in Phase C"
  // discrepancies documented in
  // docs/superpowers/plans/2026-10-08-tcp-pose-from-feedback.md.
  void ensure_tcp_pose_sub();

  // True iff BT_PLANNER_DEBUG=1 in the environment. Cached once per process
  // so we don't pay getenv() on every setGoal() call.
  static bool is_debug_enabled();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_ready_ = false;
  bool sub_init_attempted_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_sub_;
  std::mutex tcp_pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_tcp_pose_;
  std::atomic<bool> tcp_pose_ready_{false};
  bool tcp_sub_init_attempted_ = false;

  // Settle window after MoveGroup SUCCEEDED — sampled from a wall-clock
  // wall before reading latest_tcp_pose_. arm SDK keeps publishing at
  // 200 Hz even after a stop command for ~100 ms (mechanical damping),
  // so 300 ms gives the arm enough time to physically stabilize.
  static constexpr int kSettleWindowMs = 300;
};

}  // namespace lgw_bt_planner
