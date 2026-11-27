// compute_head_target_world.cpp — see header for semantics.
//
// Algorithm (pure transform, no consistency check):
//   P_target_head  = head_obs.detections.front().center_m
//   P_target_world = R_h2b · P_target_head + t_h2b
//   blackboard.set(output_key, Point{P_target_world})
//
// Companion to compute_arm_target_world.cpp. The two are split into
// separate nodes (rather than one) so that the head_arm_independent_lookat
// BT can wire them into two distinct Sequence children — the user inspects
// tcp at each independently computed world position to localize which
// transform (head→world vs. arm→tcp→world) carries the bias.

#include "lgw_bt_planner/compute_head_target_world.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // parse_calibration_json

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>

#include <stdexcept>
#include <string>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultOutputKey[]  = "head_target_world";
constexpr char kDefaultHeadCalibName[] = "head_calibration.json";

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}
}  // namespace

ComputeHeadTargetWorldNode::ComputeHeadTargetWorldNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  std::string head_path;
  if (auto p = getInput<std::string>("head_calibration_path"); p && !p->empty()) {
    head_path = *p;
  } else {
    head_path = resolve_default_calib_path(kDefaultHeadCalibName);
  }

  try {
    std::tie(R_h2b_, t_h2b_) = parse_calibration_json(head_path);
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ComputeHeadTargetWorld"),
      "Loaded head calibration: %s", head_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeHeadTargetWorld"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList ComputeHeadTargetWorldNode::providedPorts()
{
  return {
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "head_observation",
      "Last CONFIRMED head ObservationResult (e.g. {last_head_obs})"),
    BT::InputPort<std::string>(
      "head_calibration_path", "",
      "Path to head handeye JSON; empty = default share/.../head_calibration.json"),
    BT::InputPort<std::string>(
      "output_key", kDefaultOutputKey,
      "Blackboard key for the output Point (default 'head_target_world')"),
    BT::OutputPort<geometry_msgs::msg::Point>(
      "head_target_world", "World-frame target Point; also written to blackboard"),
  };
}

BT::NodeStatus ComputeHeadTargetWorldNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  auto obs_in = getInput<yoloe_interfaces::msg::ObservationResult>(
    "head_observation");
  if (!obs_in || obs_in->detections.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeHeadTargetWorld"),
      "No head detection (obs empty / no detections); FAILURE");
    return BT::NodeStatus::FAILURE;
  }
  const auto& det = obs_in->detections.front();

  // P_target_world = R_h2b · center_m + t_h2b
  tf2::Vector3 P_target_head(det.center_m.x, det.center_m.y, det.center_m.z);
  tf2::Vector3 P_target_world = R_h2b_ * P_target_head + t_h2b_;

  geometry_msgs::msg::Point pt;
  pt.x = P_target_world.x();
  pt.y = P_target_world.y();
  pt.z = P_target_world.z();

  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_key"); p && !p->empty()) {
    output_key = *p;
  }
  setOutput("head_target_world", pt);
  config().blackboard->set<geometry_msgs::msg::Point>(output_key, pt);

  RCLCPP_INFO(rclcpp::get_logger("ComputeHeadTargetWorld"),
    "det.class_name='%s' center_m=(%.4f, %.4f, %.4f) -> "
    "P_target_world=(%.4f, %.4f, %.4f) [key='%s']",
    det.class_name.c_str(),
    det.center_m.x, det.center_m.y, det.center_m.z,
    pt.x, pt.y, pt.z, output_key.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
