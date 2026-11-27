// include/lgw_bt_planner/control_gripper_node.hpp
//
// ControlGripper —— 夹爪开合/速度/力判据的规范 BT 节点(2026-11-16)。
//
// 为什么不用 SRDF named state:那种只能离散"开/合",给不了开合量。
// 为什么不用 action:历史里有个 ControlGripper.action,但从没有服务端
// (见 0a4711e 的删除记录),是个空壳。
//
// 实际下发通道:`/gripper_controller/joint_trajectory`
//   (trajectory_msgs/JointTrajectory, joint = "gripper", 0..0.1 m)
//   - 开合大小 = 终点位置
//   - 速度     = |target - current| / duration
// 起点的 X 值必须写当前实测值, 否则 JTC 的 allowed_start_tolerance 会拒收。
//
// 力的说明:逐次力在驱动层是支持的(`/control/joint_states` 的 effort[]),
// 但 JTC 以 200 Hz 发同一话题且 effort 为空,会把我们写的值覆盖掉。
// 所以这里的 force 端口是**判据**而不是命令:
//   max_force > 0 时, /feedback/gripper_status.force 达到该值即视为夹住,
//   立即返回 SUCCESS(用于"夹到把手"这类只需要知道夹紧了的场景)。
// 想改实际握力用驱动的 `gripper_default_effort` 参数。
//
// require_force(2026-11-18,抓取重试用):闭合指令下,如果夹爪一路走到目标
// 开合量却始终没建立起接触力(空夹),说明对象没被夹到 —— 此时返回 FAILURE
// 而不是"到位即成功",让外层 RetryUntilSuccessful 触发重抓。只有在
// max_force > 0 时才有意义。

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace lgw_bt_planner
{

class ControlGripperNode : public BT::StatefulActionNode
{
public:
  ControlGripperNode(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void on_status(const agx_arm_msgs::msg::GripperStatus::SharedPtr msg);

  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>>
  traj_pub_;
  std::shared_ptr<rclcpp::Subscription<agx_arm_msgs::msg::GripperStatus>>
  status_sub_;

  mutable std::mutex mutex_;
  bool status_seen_{false};
  double width_m_{0.0};
  double force_n_{0.0};

  // Ports captured at onStart.
  double start_width_{0.0};
  double target_width_{0.0};
  double duration_s_{0.5};
  double max_force_n_{0.0};
  double tolerance_m_{0.004};
  double timeout_s_{2.0};
  bool require_force_{false};

  rclcpp::Time started_at_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  rclcpp::Logger logger_{rclcpp::get_logger("lgw_bt_planner.control_gripper")};
};

}  // namespace lgw_bt_planner
