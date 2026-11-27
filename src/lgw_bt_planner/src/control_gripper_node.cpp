// src/control_gripper_node.cpp — see the header for the contract.

#include "lgw_bt_planner/control_gripper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace lgw_bt_planner
{

namespace
{
constexpr char kGripperJoint[] = "gripper";
constexpr char kTrajTopic[] = "/gripper_controller/joint_trajectory";
constexpr char kStatusTopic[] = "/feedback/gripper_status";
// URDF: prismatic, 0 .. 0.1 m.
constexpr double kWidthMin = 0.0;
constexpr double kWidthMax = 0.1;
}  // namespace

ControlGripperNode::ControlGripperNode(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("ControlGripperNode: RosNodeParams::nh expired");
  }

  traj_pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    kTrajTopic, rclcpp::QoS(1));
  status_sub_ = node->create_subscription<agx_arm_msgs::msg::GripperStatus>(
    kStatusTopic, rclcpp::QoS(10),
    std::bind(&ControlGripperNode::on_status, this, std::placeholders::_1));
}

void ControlGripperNode::on_status(
  const agx_arm_msgs::msg::GripperStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  width_m_ = msg->width;
  force_n_ = msg->force;
  status_seen_ = true;
}

BT::PortsList ControlGripperNode::providedPorts()
{
  return {
    // 目标开合量, 单位 m (0 = 全闭, 0.1 = 全开)。
    BT::InputPort<double>("target_width", 0.0, "Target opening in metres"),
    // 运动用时(s)。速度 = |target-current| / duration。
    BT::InputPort<double>("duration", 0.5, "Trajectory duration in seconds"),
    // >0 时把力当作完成判据: |force| 达到该值即 SUCCESS(夹住即返回)。
    // 取绝对值: 实测夹住时 force 为负(约 -effort), 空载约 +0.07 N。
    BT::InputPort<double>(
      "max_force", 0.0, "0 = ignore force; >0 = success when |force| reaches it"),
    // 抓取重试用: 走到目标开合量但始终没建起接触力 → FAILURE(空夹)。
    BT::InputPort<bool>(
      "require_force", false,
      "true = reaching target_width without |force| >= max_force is FAILURE"),
    // 位置到位判据(m)。
    BT::InputPort<double>("tolerance", 0.004, "Position tolerance in metres"),
    // 整体超时(s), 超过判 FAILURE。
    BT::InputPort<double>("timeout", 2.0, "Give up after this many seconds"),
  };
}

BT::NodeStatus ControlGripperNode::onStart()
{
  if (!getInput("target_width", target_width_)) {
    RCLCPP_ERROR(logger_, "ControlGripper: required input 'target_width' missing");
    return BT::NodeStatus::FAILURE;
  }
  if (target_width_ < kWidthMin || target_width_ > kWidthMax) {
    RCLCPP_ERROR(
      logger_, "ControlGripper: target_width %.3f outside [%.1f, %.1f] m",
      target_width_, kWidthMin, kWidthMax);
    return BT::NodeStatus::FAILURE;
  }
  getInput("duration", duration_s_);
  getInput("max_force", max_force_n_);
  getInput("require_force", require_force_);
  getInput("tolerance", tolerance_m_);
  getInput("timeout", timeout_s_);
  if (duration_s_ <= 0.0) {
    duration_s_ = 0.5;  // 0 会让 JTC 拿到起点=终点的退化轨迹
  }

  double now_width = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_seen_) {
      RCLCPP_ERROR(
        logger_, "ControlGripper: no %s yet, cannot build a trajectory", kStatusTopic);
      return BT::NodeStatus::FAILURE;
    }
    now_width = width_m_;
  }
  start_width_ = now_width;

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {kGripperJoint};

  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {now_width};
  first.time_from_start.sec = 0;
  first.time_from_start.nanosec = 0;

  trajectory_msgs::msg::JointTrajectoryPoint last;
  last.positions = {target_width_};
  const double total = duration_s_;
  last.time_from_start.sec = static_cast<int32_t>(std::floor(total));
  last.time_from_start.nanosec = static_cast<uint32_t>(
    std::llround((total - std::floor(total)) * 1e9));

  traj.points.push_back(first);
  traj.points.push_back(last);

  traj_pub_->publish(traj);
  started_at_ = clock_.now();

  RCLCPP_INFO(
    logger_, "ControlGripper: %.3f -> %.3f m in %.2f s (max_force=%.2fN)",
    now_width, target_width_, duration_s_, max_force_n_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ControlGripperNode::onRunning()
{
  double width = 0.0;
  double force = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    width = width_m_;
    force = force_n_;
  }

  // 力优先: 夹到东西比"走到目标"更有意义(目标可能被物体挡住)。
  //
  // 用绝对值比较: 2026-11-17 真机实测, /feedback/gripper_status.force 在
  // 空载时约 +0.07 N, 夹住物体时变成 -1.05 N(量级正好等于当时的
  // gripper_default_effort=1.0 N)。符号是负的, 所以直接比 force >= 阈值
  // 永远不会成立 —— 必须取 |force|。
  if (max_force_n_ > 0.0 && std::fabs(force) >= max_force_n_) {
    RCLCPP_INFO(
      logger_, "ControlGripper: grip force |%.2f|N >= %.2fN at width %.3f m",
      force, max_force_n_, width);
    return BT::NodeStatus::SUCCESS;
  }

  if (std::fabs(width - target_width_) <= tolerance_m_) {
    // 空夹判定: 到位但没接触力, 说明对象不在指间(抓取时)或本来就没夹到。
    if (require_force_ && max_force_n_ > 0.0) {
      RCLCPP_WARN(
        logger_,
        "ControlGripper: reached %.3f m (target %.3f) without force (%.2fN < %.2fN)"
        " — empty grasp", width, target_width_, std::fabs(force), max_force_n_);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      logger_, "ControlGripper: reached %.3f m (target %.3f)", width, target_width_);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = (clock_.now() - started_at_).seconds();
  if (elapsed > timeout_s_) {
    RCLCPP_ERROR(
      logger_, "ControlGripper: timeout after %.2fs (width %.3f, target %.3f, force %.2fN)",
      elapsed, width, target_width_, force);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void ControlGripperNode::onHalted()
{
  // 停在当前开合量即可; 不回原点也不重发。
  RCLCPP_WARN(logger_, "ControlGripper: halted");
}

}  // namespace lgw_bt_planner
