// wait_for_user_input_bt_node.cpp — see header for design.
//
// Snapshot semantics (no RUNNING):
//   - RosTopicSubNode caches the latest msg; onTick receives it as
//     shared_ptr (nullptr == no msg received since subscription start).
//   - We never return RUNNING — the parent Reactive* node is expected to
//     re-tick us every BT tick, and onTick returns SUCCESS/FAILURE based
//     on the cached msg.
//
// Why this is intentionally simpler than the historical piper_bt_planner
// StatefulActionNode version: BT.ROS2 v0.3.0's RosTopicSubNode already
// manages the subscription, callback group, executor, and last_msg cache.
// Re-implementing those (~70 lines of mutex + sub) would duplicate BT.ROS2
// internals for no behavior gain.

#include "lgw_bt_planner/wait_for_user_input_bt_node.hpp"

namespace lgw_bt_planner
{

BT::PortsList WaitForUserInputActionNode::providedPorts()
{
  // providedBasicPorts() adds the "topic_name" InputPort that BT.ROS2's
  // constructor inspects to create the subscriber. Without this call the
  // per-instance <topic_name> remap in BT XML is silently dropped (same
  // reasoning as is_detection_confirmed_node.hpp §NOTES).
  return providedBasicPorts({});
}

BT::NodeStatus WaitForUserInputActionNode::onTick(
  const std::shared_ptr<std_msgs::msg::Bool>& last_msg)
{
  // No message received yet since subscription started.
  if (!last_msg) {
    return BT::NodeStatus::FAILURE;
  }
  // data == true  → SUCCESS
  // data == false → FAILURE (a "false" pulse is not a trigger)
  return last_msg->data ? BT::NodeStatus::SUCCESS
                        : BT::NodeStatus::FAILURE;
}

}  // namespace lgw_bt_planner