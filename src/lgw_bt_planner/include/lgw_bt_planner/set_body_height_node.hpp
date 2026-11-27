// set_body_height_node.hpp
//
// BT::StatefulActionNode that publishes a BodyHeight command
// (api_id=1013) to /api/sport/request with parameter `{"data": H}`
// where H is a relative offset in meters (Go2 spec range
// [-0.18, +0.03]). Sign convention (verified on real hardware
// 2026-10-06): positive = DROP (body lowers below default),
// negative = RAISE (body lifts above default). Waits until
// lf/sportmodestate.body_height is within
// `kDefaultGo2BodyHeight − offset` ± `tolerance` for kStableFrames
// (5) consecutive ticks. Fails after kBodyHeightTimeoutSec (5 s).
// StopMove (1003) on halt.
//
// Per the Go2 BodyHeight service description, the adjustment only
// works when the dog is in a balanced standing or walking state.
// The caller is responsible for having sent a BalanceStand (1002)
// before this node runs.

#pragma once

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logging.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace lgw_bt_planner
{

class SetBodyHeightNode : public BT::StatefulActionNode
{
public:
  SetBodyHeightNode(const std::string& name, const BT::NodeConfig& config,
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
  // Cached from onStart so onRunning uses the same target the publish used.
  float target_height_{sport_api::kDefaultGo2BodyHeight};
  float tolerance_{sport_api::kDefaultBodyHeightTol};
  rclcpp::Logger logger_{rclcpp::get_logger("lgw_bt_planner.set_body_height")};
};

}  // namespace lgw_bt_planner