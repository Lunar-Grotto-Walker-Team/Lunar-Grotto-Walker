// is_detection_confirmed_node.cpp — see header. The two classes share
// providedPorts() / onTick() logic; only the class identity differs
// (and that comes from RosNodeParams at registration time).

#include "lgw_bt_planner/is_detection_confirmed_node.hpp"

#include "lgw_bt_planner/evaluate_detection.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace lgw_bt_planner
{

namespace
{
// Shared port declaration. Two InputPort-shaped ports:
//   - max_age_sec: double (default 2.5, per detection.txt §11)
//   - last_observation: ObservationResult (written on SUCCESS only,
//     per spec §3.2)
constexpr double kDefaultMaxAgeSec = 2.5;
constexpr const char* kMaxAgePort = "max_age_sec";
constexpr const char* kLastObsPort = "last_observation";
// 2026-10-09: depth-range gating on IsArmDetectionConfirmed — head_arm_independent_lookat
// BT cycles observed the arm camera flipping between a near wood_block
// (~0.48 m depth) and a far one (~1.37 m) across cycles, even though the
// head camera always picks the same physical block. Default [0, +inf) is
// fully backward-compatible (no filter). Default [0.40, 0.65] matches the
// workspace head→arm-target distance the lookat step already proved
// reachable (P_tcp ≈ 0.46 m from base, head→target ≈ 0.54 m).
constexpr double kDefaultArmDepthMin = 0.0;
constexpr double kDefaultArmDepthMax = 1e9;
constexpr const char* kArmDepthMinPort = "arm_depth_min_m";
constexpr const char* kArmDepthMaxPort = "arm_depth_max_m";
}  // namespace

// IsHeadDetectionConfirmed ------------------------------------------------

BT::PortsList IsHeadDetectionConfirmed::providedPorts()
{
  // providedBasicPorts() adds the "topic_name" InputPort that BT.ROS2's
  // constructor inspects to create the subscriber. Without this call the
  // per-instance <topic_name> remap in BT XML is silently dropped.
  return providedBasicPorts({
    BT::InputPort<double>(kMaxAgePort, kDefaultMaxAgeSec,
                          "Maximum age (s) of the cached detection."),
    BT::OutputPort<yoloe_interfaces::msg::ObservationResult>(
      kLastObsPort,
      "Last CONFIRMED ObservationResult; written only on SUCCESS."),
  });
}

BT::NodeStatus IsHeadDetectionConfirmed::onTick(
  const std::shared_ptr<yoloe_interfaces::msg::ObservationResult>& last_msg)
{
  // BT.ROS2's RosTopicSubNode passes the cached last message as a
  // shared_ptr; nullptr means "no message received yet since last tick".
  // evaluate_detection() takes std::optional, so wrap here. RosTopicSubNode
  // exposes the node handle as a protected `node_` weak_ptr — lock it to
  // access now() (same pattern as bt_topic_sub_node.hpp:252).
  const std::optional<yoloe_interfaces::msg::ObservationResult> opt_msg =
    last_msg ? std::optional<yoloe_interfaces::msg::ObservationResult>(*last_msg)
             : std::nullopt;
  const auto max_age_sec = getInput<double>(kMaxAgePort).value();
  const auto node_shared = node_.lock();
  const auto result = evaluate_detection(opt_msg, max_age_sec, node_shared->now());
  if (result.ok) {
    // Spec §3.2: write only on SUCCESS. The dereference is safe: ok
    // implies last_msg was non-null per evaluate_detection()'s logic.
    setOutput(kLastObsPort, *last_msg);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// IsArmDetectionConfirmed -------------------------------------------------

BT::PortsList IsArmDetectionConfirmed::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>(kMaxAgePort, kDefaultMaxAgeSec,
                          "Maximum age (s) of the cached detection."),
    BT::InputPort<double>(kArmDepthMinPort, kDefaultArmDepthMin,
                          "Minimum allowed depth (m) of detections.front()."
                          "center_m.z; default 0 disables the lower bound."),
    BT::InputPort<double>(kArmDepthMaxPort, kDefaultArmDepthMax,
                          "Maximum allowed depth (m) of detections.front()."
                          "center_m.z; default 1e9 disables the upper bound."),
    BT::OutputPort<yoloe_interfaces::msg::ObservationResult>(
      kLastObsPort,
      "Last CONFIRMED ObservationResult; written only on SUCCESS."),
  });
}

BT::NodeStatus IsArmDetectionConfirmed::onTick(
  const std::shared_ptr<yoloe_interfaces::msg::ObservationResult>& last_msg)
{
  const std::optional<yoloe_interfaces::msg::ObservationResult> opt_msg =
    last_msg ? std::optional<yoloe_interfaces::msg::ObservationResult>(*last_msg)
             : std::nullopt;
  const auto max_age_sec = getInput<double>(kMaxAgePort).value();
  const auto node_shared = node_.lock();
  const auto result = evaluate_detection(opt_msg, max_age_sec, node_shared->now());
  if (!result.ok) {
    return BT::NodeStatus::FAILURE;
  }
  // Depth-range gate (2026-10-09): only after freshness/confidence checks
  // pass do we ask "is the chosen detection actually the one we expect?"
  // empty detections → already caught by evaluate_detection; defensively
  // bail here so the .front() below is always safe.
  if (last_msg->detections.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto depth_min = getInput<double>(kArmDepthMinPort).value();
  const auto depth_max = getInput<double>(kArmDepthMaxPort).value();
  const double depth_z = last_msg->detections.front().center_m.z;
  if (depth_z < depth_min || depth_z > depth_max) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("IsArmDetectionConfirmed"),
      *node_shared, 2000,
      "arm detection center_m.z=%.3f outside [%g, %g] m → FAILURE",
      depth_z, depth_min, depth_max);
    return BT::NodeStatus::FAILURE;
  }
  setOutput(kLastObsPort, *last_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner