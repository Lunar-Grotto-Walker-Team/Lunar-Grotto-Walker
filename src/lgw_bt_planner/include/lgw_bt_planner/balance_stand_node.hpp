// balance_stand_node.hpp
//
// BT::StatefulActionNode that publishes a BalanceStand command
// (api_id=1002) to /api/sport/request once and reports SUCCESS immediately —
// no state polling, no deadline. Just enough to deliver the command so the
// Sequence can move on to StandDown.
//
// Why so minimal: see commit message / work.log 2026-10-06. Go2 真机
// lf/sportmodestate.mode 字段在 BalanceStand 1002 后未跳到 1 (真机观测
// mode=0 持续 2s+),mode==1 判据不可靠。直接发布一次 1002 让 Go2 进入
// BalanceStand 模式,Sequence 走完到 StandDown。StandDown 用 body_height
// 阈值,不依赖 mode。

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/logging.hpp>
#include <unitree_api/msg/request.hpp>

#include <memory>

namespace lgw_bt_planner
{

class BalanceStandNode : public BT::StatefulActionNode
{
public:
  BalanceStandNode(const std::string& name, const BT::NodeConfig& config,
                   const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("lgw_bt_planner.balance_stand")};
};

}  // namespace lgw_bt_planner