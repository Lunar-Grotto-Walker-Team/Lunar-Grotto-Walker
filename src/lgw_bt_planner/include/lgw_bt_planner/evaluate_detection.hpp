// evaluate_detection.hpp — pure-function predicate for the
// detection-condition BT leaf nodes (IsHead/ArmDetectionConfirmed).
//
// Lives in lgw_bt_planner so both BT leaf nodes (Task 2) and the
// gtest binary (Task 1) can call it without dragging in BT.ROS2.

#pragma once

#include <yoloe_interfaces/msg/observation_result.hpp>

#include <rclcpp/time.hpp>

#include <optional>

namespace lgw_bt_planner
{

// Result of evaluating one cached detection against the four-condition
// check described in the spec. ok == true iff all four conditions pass.
//
// Currently only `ok` is exposed; the observation is already in the
// caller's hand (it passed the std::optional in). We keep a struct
// (rather than bool) so future fields (e.g. "matched_class_id") can
// be added without breaking callers.
struct DetectionCheckResult
{
  bool ok;
};

DetectionCheckResult evaluate_detection(
  const std::optional<yoloe_interfaces::msg::ObservationResult>& last_msg,
  double max_age_sec,
  rclcpp::Time now);

}  // namespace lgw_bt_planner
