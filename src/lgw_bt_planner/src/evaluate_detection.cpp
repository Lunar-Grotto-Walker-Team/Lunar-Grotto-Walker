// evaluate_detection.cpp — see header for semantics.

#include "lgw_bt_planner/evaluate_detection.hpp"

namespace lgw_bt_planner
{

DetectionCheckResult evaluate_detection(
  const std::optional<yoloe_interfaces::msg::ObservationResult>& last_msg,
  double max_age_sec,
  rclcpp::Time now)
{
  // Condition 1: at least one message received
  if (!last_msg.has_value()) {
    return {false};
  }
  const auto& msg = *last_msg;

  // Condition 2: status == CONFIRMED. NO_TARGET / TIMEOUT / CANCELLED /
  // ERROR all fall through to FAILURE per spec §3.3.
  if (msg.status != yoloe_interfaces::msg::ObservationResult::CONFIRMED) {
    return {false};
  }

  // Condition 3: at least one detection
  if (msg.detections.empty()) {
    return {false};
  }

  // Condition 4: age <= max_age_sec. rclcpp::Time is signed; if the
  // header stamp is somehow in the future (clock weirdness) the
  // resulting age is negative and 0 fails the check, which is the
  // conservative behavior we want.
  const auto msg_stamp = rclcpp::Time(msg.header.stamp);
  const auto age = now - msg_stamp;
  if (age.seconds() > max_age_sec) {
    return {false};
  }

  return {true};
}

}  // namespace lgw_bt_planner
