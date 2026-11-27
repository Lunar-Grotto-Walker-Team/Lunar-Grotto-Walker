// stand_down_node.hpp
//
// BT::StatefulActionNode that publishes a StandDown command
// (api_id=1005) to /api/sport/request and waits until
// lf/sportmodestate.body_height < kLyingHeight (0.20 m) for
// kStableFrames (5) consecutive ticks. Fails after
// kStandDownTimeoutSec (5 s). See spec §4.2.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace lgw_bt_planner
{

class StandDownNode : public BT::StatefulActionNode
{
public:
  StandDownNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  std::shared_ptr<rclcpp::Subscription<unitree_go::msg::SportModeState>>
      subscriber_;
  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  bool state_received_{false};
  int stable_count_{0};
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Logger logger_{rclcpp::get_logger("lgw_bt_planner.stand_down")};
};

}  // namespace lgw_bt_planner