// balance_stand_node.cpp
//
// Implements BalanceStandNode: publish api_id=1002 once in onStart, then
// return SUCCESS immediately. No state polling, no deadline. See hpp for the
// rationale (Go2 mode field is not a reliable completion signal).
//
// onHalted is a no-op per spec §5.3: sending StopMove here would knock the
// dog's mode back to 0, undoing what we just achieved. With onStart →
// SUCCESS, BT.CPP normally won't reach onHalted at all; the only way to
// reach here is a Sequence-level halt (e.g. a later sibling FAILURE), which
// is the case where StopMove would be wrong.

#include "lgw_bt_planner/balance_stand_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

namespace lgw_bt_planner
{

BalanceStandNode::BalanceStandNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "BalanceStandNode: RosNodeParams::nh weak_ptr expired");
  }

  logger_ = node->get_logger();

  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  RCLCPP_INFO(logger_,
    "[%s] BalanceStandNode initialized (pub=/api/sport/request, no sub)",
    this->name().c_str());
}

BT::PortsList BalanceStandNode::providedPorts()
{
  return {};
}

BT::NodeStatus BalanceStandNode::onStart()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdBalanceStand;
  publisher_->publish(req);

  RCLCPP_INFO(logger_,
    "[%s] onStart: published BalanceStand (api_id=1002), SUCCESS (no mode check)",
    this->name().c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BalanceStandNode::onRunning()
{
  // After onStart returns SUCCESS, BT.CPP never calls onRunning again for
  // this node instance. Kept as a defensive no-op so a future change that
  // re-introduces polling won't silently skip a body. (Also catches
  // accidental logic regressions.)
  RCLCPP_WARN(logger_,
    "[%s] onRunning called unexpectedly (should not happen — onStart returns SUCCESS)",
    this->name().c_str());
  return BT::NodeStatus::SUCCESS;
}

void BalanceStandNode::onHalted()
{
  // Spec §5.3: BalanceStand onHalted is a no-op. Sending StopMove here
  // would knock state.mode back to 0, which is the opposite of what
  // we just achieved. Leave the dog in whatever state it reached.
  RCLCPP_INFO(logger_,
    "[%s] onHalted: no-op (spec §5.3, do not StopMove here)",
    this->name().c_str());
}

}  // namespace lgw_bt_planner