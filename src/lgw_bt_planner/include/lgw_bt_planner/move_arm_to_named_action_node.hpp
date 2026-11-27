#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "lgw_bt_planner/named_target_loader.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace lgw_bt_planner
{

class MoveArmToNamedActionNode
  : public BT::RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
  MoveArmToNamedActionNode(
    const std::string& name, const BT::NodeConfig& cfg,
    const BT::RosNodeParams& params)
    : BT::RosActionNode<moveit_msgs::action::MoveGroup>(name, cfg, params) {}

  bool setGoal(moveit_msgs::action::MoveGroup::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  // 同 MoveArmToPose: ABORTED / CANCELED / REJECTED 走 onFailure()，基类实现
  // 静默返回 FAILURE，MoveIt 失败原因会丢失（2026-11-18）。
  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult>& wr) override;
  static BT::PortsList providedPorts();

private:
  // Lazy-init /feedback/joint_states (200 Hz from agx_arm_ctrl_single_node).
  // Used to prove the arm actually arrived: MoveGroup runs on FakeSystem and
  // returns SUCCEEDED even when a joint never moves (2026-11-17: park with
  // joint4 stalled 0.41 rad away, tree still reported SUCCESS).
  void ensure_joint_state_sub();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::atomic<bool> joint_state_ready_{false};
  bool sub_init_attempted_ = false;

  // Joint-value goal captured in setGoal(); empty for unknown targets.
  std::map<std::string, double> target_joints_;

  // Let the arm physically settle before sampling the same way MoveArmToPose
  // does (the SDK keeps damping ~100 ms after the last setpoint).
  static constexpr int kSettleWindowMs = 300;
};

}  // namespace lgw_bt_planner
