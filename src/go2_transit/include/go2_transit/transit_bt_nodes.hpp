// transit_bt_nodes.hpp — go2_transit 导出的 BehaviorTree.CPP 节点。
//
// 只放"把树上的字面量翻译成动作调用"的薄壳，控制逻辑全在 transit_node 里。

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "go2_transit/action/transit_to_pose.hpp"
#include "go2_transit/transit_types.hpp"

namespace go2_transit
{

// 把"树启动那一刻的位姿"当作原点，走到局部坐标 (X, Y)，再把朝向摆回原点朝向。
//
// 端口：
//   in  goal            geometry_msgs::PoseStamped，frame_id 必须是 "local"
//   in  planning_frame  默认 odom
//   in  base_frame      默认 base_footprint
//   in  xy_tolerance    默认 0（交给 transit 节点的默认值）
//   in  yaw_tolerance   默认 0（同上）
//   in  ready_wait_s    等 TF / 动作服务器的上限，默认 10 s
//   out result_reason   结果字符串（ok / unreachable / ...），便于日志排查
class TransitToLocalPoint : public BT::StatefulActionNode
{
public:
  TransitToLocalPoint(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using TransitToPose = go2_transit::action::TransitToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<TransitToPose>;

  enum class Phase { WaitReady, WaitResult };

  bool ready();
  bool capture_origin();
  void send_goal();
  double elapsed_s() const;
  BT::NodeStatus fail_with(const std::string & reason);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<TransitToPose>::SharedPtr client_;

  geometry_msgs::msg::PoseStamped goal_local_;
  std::string planning_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  double xy_tolerance_{0.0};
  double yaw_tolerance_{0.0};
  double ready_wait_s_{10.0};
  double result_timeout_s_{420.0};

  Phase phase_{Phase::WaitReady};
  bool origin_captured_{false};
  // keep_origin (2026-11-20): halt 之后不作废局部原点。
  // 用途: 巡逻腿"前进 3 m 途中被采集任务打断", 恢复时要接着把**剩下的**
  // 路程走完 —— 需要的是同一个原点 + 同一个 (3, 0), 而不是从当前位置再走
  // 3 m。默认 false = 旧行为(每次 onStart 重新定位, 适用于一次性摆渡)。
  bool keep_origin_{false};
  Pose2D origin_{};
  std::chrono::steady_clock::time_point started_at_{};
  std::chrono::steady_clock::time_point goal_sent_at_{};

  std::shared_future<GoalHandle::SharedPtr> goal_response_;
  std::shared_future<typename GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;

  rclcpp::Logger logger_{rclcpp::get_logger("go2_transit.bt_transit_to_local_point")};
};

}  // namespace go2_transit
