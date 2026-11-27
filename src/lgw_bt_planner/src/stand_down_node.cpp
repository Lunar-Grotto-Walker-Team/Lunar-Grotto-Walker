// stand_down_node.cpp
//
// Implements StandDownNode: publish api_id=1005 once in onStart, then
// poll lf/sportmodestate.body_height < 0.20 for 5 consecutive
// onRunning ticks. StopMove (1003) on halt. Mirror of StandUpNode —
// same pattern, opposite threshold direction. See spec §4.2 + §5.1.

#include "lgw_bt_planner/stand_down_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

namespace lgw_bt_planner
{

StandDownNode::StandDownNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "StandDownNode: RosNodeParams::nh weak_ptr expired");
  }

  logger_ = node->get_logger();

  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  subscriber_ = node->create_subscription<unitree_go::msg::SportModeState>(
    sport_api::kSportModeStateTopic,
    rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool first = !state_received_;
      latest_state_ = *msg;
      state_received_ = true;
      if (first) {
        RCLCPP_INFO(logger_,
          "[%s] first state callback: body_height=%.3f mode=%u",
          this->name().c_str(), msg->body_height, msg->mode);
      }
    });

  RCLCPP_INFO(logger_,
    "[%s] StandDownNode initialized: pub=/api/sport/request sub=lf/sportmodestate",
    this->name().c_str());
}

BT::PortsList StandDownNode::providedPorts()
{
  return {};
}

BT::NodeStatus StandDownNode::onStart()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStandDown;
  publisher_->publish(req);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = unitree_go::msg::SportModeState{};
    state_received_ = false;
  }
  stable_count_ = 0;
  // steady_clock::duration is nanoseconds (long int); duration<double>
  // does not implicitly convert. Cast to long-int seconds to match
  // steady_clock's representation. (Timeout values are integer seconds
  // in sport_api_constants.hpp so precision loss is irrelevant.)
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::duration<double>(
                  sport_api::kStandDownTimeoutSec));

  RCLCPP_INFO(logger_,
    "[%s] onStart: published StandDown (api_id=1005), deadline=%.0fs, waiting body_height < 0.20 for 5 frames",
    this->name().c_str(), sport_api::kStandDownTimeoutSec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StandDownNode::onRunning()
{
  unitree_go::msg::SportModeState state;
  bool received;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    received = state_received_;
  }

  // Threshold: body_height < 0.20 for 5 consecutive frames (spec §4.2).
  if (state.body_height < sport_api::kLyingHeight) {
    if (++stable_count_ >= sport_api::kStableFrames) {
      RCLCPP_INFO(logger_,
        "[%s] SUCCESS: body_height=%.3f mode=%u stable_count=%d (>= %d)",
        this->name().c_str(), state.body_height, state.mode,
        stable_count_, sport_api::kStableFrames);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_count_ = 0;
  }

  // Log every 5th tick (500ms at 10Hz).
  static thread_local int tick_count = 0;
  if (tick_count++ % 5 == 0) {
    RCLCPP_INFO(logger_,
      "[%s] tick: body_height=%.3f mode=%u stable_count=%d state_received=%s",
      this->name().c_str(), state.body_height, state.mode,
      stable_count_, received ? "true" : "false");
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    RCLCPP_WARN(logger_,
      "[%s] FAILURE: deadline exceeded (timeout=%.0fs), last body_height=%.3f mode=%u state_received=%s",
      this->name().c_str(), sport_api::kStandDownTimeoutSec,
      state.body_height, state.mode, received ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void StandDownNode::onHalted()
{
  // Spec §5.3: StandDown halt publishes StopMove (1003), symmetric
  // with StandUp.
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStopMove;
  publisher_->publish(req);
  RCLCPP_INFO(logger_,
    "[%s] onHalted: published StopMove (api_id=1003)",
    this->name().c_str());
}

}  // namespace lgw_bt_planner