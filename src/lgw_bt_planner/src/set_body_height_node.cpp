// set_body_height_node.cpp
//
// Implements SetBodyHeightNode: publish api_id=1013 once in onStart
// with parameter {"data": offset}, then poll body_height ≈ target
// for kStableFrames (5) consecutive ticks. StopMove (1003) on halt.
// See hpp for spec rationale and the Go2 /body_height service
// semantics.

#include "lgw_bt_planner/set_body_height_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace lgw_bt_planner
{

SetBodyHeightNode::SetBodyHeightNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "SetBodyHeightNode: RosNodeParams::nh weak_ptr expired");
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
    "[%s] SetBodyHeightNode initialized: pub=/api/sport/request sub=lf/sportmodestate",
    this->name().c_str());
}

BT::PortsList SetBodyHeightNode::providedPorts()
{
  return {
    BT::InputPort<double>("height",
      0.0,
      "Relative body height offset (m). Sign convention: positive = DROP "
      "(body lowers below default 0.33 m), negative = RAISE (body lifts "
      "above default). Go2 spec range [-0.18, +0.03]; out-of-range values "
      "are CLAMPED. Polling target is computed as 0.33 - offset."),
    BT::InputPort<double>("tolerance",
      static_cast<double>(sport_api::kDefaultBodyHeightTol),
      "Acceptable |body_height - target| (m). 0 or negative disables.")
  };
}

BT::NodeStatus SetBodyHeightNode::onStart()
{
  double height_d = 0.0;
  double tolerance_d = static_cast<double>(sport_api::kDefaultBodyHeightTol);
  if (!getInput("height", height_d)) {
    RCLCPP_WARN(logger_,
      "[%s] port 'height' not provided, defaulting to 0.0",
      this->name().c_str());
    height_d = 0.0;
  }
  if (!getInput("tolerance", tolerance_d)) {
    RCLCPP_WARN(logger_,
      "[%s] port 'tolerance' not provided, defaulting to %.3f",
      this->name().c_str(), sport_api::kDefaultBodyHeightTol);
    tolerance_d = static_cast<double>(sport_api::kDefaultBodyHeightTol);
  }

  // Clip offset to Go2 spec range. Out-of-range values are clamped at
  // the boundary rather than rejected, so the operator sees the actual
  // command we sent (and the WARN log makes the truncation obvious).
  //
  // Sign convention (verified on real hardware 2026-10-06): the
  // /body_height service treats positive offset as DROP (body lowers
  // below default 0.33 m) and negative offset as RAISE (body lifts
  // above default). Empirically, offset=+0.02 sent while the dog
  // was at body_h=0.329 drove the dog DOWN to body_h=0.314 — i.e.
  // actual target = 0.33 − offset. So we compute the polling target
  // here as default − offset to match the physical semantics.
  const float offset = static_cast<float>(std::clamp(
    height_d,
    static_cast<double>(sport_api::kBodyHeightOffsetMin),
    static_cast<double>(sport_api::kBodyHeightOffsetMax)));
  tolerance_ = static_cast<float>(std::max(tolerance_d, 0.0));
  target_height_ = sport_api::kDefaultGo2BodyHeight - offset;

  if (offset != static_cast<float>(height_d)) {
    RCLCPP_WARN(logger_,
      "[%s] offset %.3f outside Go2 spec range [%.2f, %.2f], clamped to %.3f",
      this->name().c_str(), height_d,
      sport_api::kBodyHeightOffsetMin, sport_api::kBodyHeightOffsetMax,
      offset);
  }

  // Build parameter JSON `{"data": <offset>}` manually to avoid pulling
  // nlohmann/json into this node (no shared helpers per spec §3).
  const std::string param = "{\"data\": " + std::to_string(offset) + "}";

  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdBodyHeight;
  req.parameter = param;
  publisher_->publish(req);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = unitree_go::msg::SportModeState{};
    state_received_ = false;
  }
  stable_count_ = 0;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::duration<double>(
                  sport_api::kBodyHeightTimeoutSec));

  RCLCPP_INFO(logger_,
    "[%s] onStart: published BodyHeight (api_id=1013) offset=%.3f target=%.3f ± %.3f, deadline=%.0fs",
    this->name().c_str(), offset, target_height_, tolerance_,
    sport_api::kBodyHeightTimeoutSec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetBodyHeightNode::onRunning()
{
  unitree_go::msg::SportModeState state;
  bool received;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    received = state_received_;
  }

  const float diff = std::fabs(state.body_height - target_height_);
  if (diff <= tolerance_) {
    if (++stable_count_ >= sport_api::kStableFrames) {
      RCLCPP_INFO(logger_,
        "[%s] SUCCESS: body_height=%.3f (target=%.3f ± %.3f, diff=%.3f)",
        this->name().c_str(), state.body_height, target_height_,
        tolerance_, diff);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_count_ = 0;
  }

  static thread_local int tick_count = 0;
  if (tick_count++ % 5 == 0) {
    RCLCPP_INFO(logger_,
      "[%s] tick: body_height=%.3f (target=%.3f ± %.3f, diff=%.3f) stable_count=%d state_received=%s",
      this->name().c_str(), state.body_height, target_height_,
      tolerance_, diff, stable_count_, received ? "true" : "false");
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    RCLCPP_WARN(logger_,
      "[%s] FAILURE: deadline exceeded (timeout=%.0fs), last body_height=%.3f (target=%.3f ± %.3f, diff=%.3f) state_received=%s",
      this->name().c_str(), sport_api::kBodyHeightTimeoutSec,
      state.body_height, target_height_, tolerance_, diff,
      received ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SetBodyHeightNode::onHalted()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStopMove;
  publisher_->publish(req);
  RCLCPP_INFO(logger_,
    "[%s] onHalted: published StopMove (api_id=1003)",
    this->name().c_str());
}

}  // namespace lgw_bt_planner