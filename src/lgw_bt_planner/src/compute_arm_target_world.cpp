// compute_arm_target_world.cpp — see header for semantics.
//
// Algorithm (pure transform, no consistency check vs. head):
//   P_arm_optical  = arm_obs.detections.front().center_m
//   P_arm_in_tcp   = R_a2t · P_arm_optical + t_a2t            (tcp frame)
//   P_arm_world    = R_tcp · P_arm_in_tcp + t_tcp             (world frame)
//   blackboard.set(output_key, Point{P_arm_world})
//
// NOTE on the arm_calibration.json convention: although _meta.transformation
// originally read "tcp_link -> camera_arm_color_optical_frame", the actual
// numeric data in that file represents T_camera -> tcp_link (i.e. R_a2t
// maps camera-frame vectors to tcp-frame vectors; t_a2t is the camera
// origin expressed in the tcp frame). Empirically verified 2026-10-09 by
// back-solving the lookat_pose step (P_tcp = (0.4007,-0.0036,0.1857) only
// reproduces with the camera->tcp interpretation, not tcp->camera). The
// _meta field has been corrected to match the data; downstream code must
// apply the formula directly (no .inverse(), no (P - t)) — see bug
// head_arm_independent_lookat 2026-10-08 25 cm world-frame error.
//
// Identical to the math inside verify_head_arm_consistency.cpp step 3-4
// (P_arm_world branch), but without the distance comparison + FAILURE
// return. The head_arm_independent_lookat BT feeds this node's output
// into ComputeGraspPose as if it were a confirmed "arm_target_world",
// letting the arm physically drive to P_arm_world while the user
// visually compares it against the actual wood_block on the table.

#include "lgw_bt_planner/compute_arm_target_world.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // parse_calibration_json, quat_rotate_vector

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>
#include <string>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultOutputKey[] = "arm_target_world";
constexpr char kDefaultArmCalibName[] = "arm_calibration.json";

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}

// Pure-quaternion rotate-vector (mirrors compute_lookat_pose_node.cpp).
// tf2 in Humble does not expose rotate(Vector3) on Quaternion directly.
tf2::Vector3 quat_rotate_vector(const tf2::Quaternion& q, const tf2::Vector3& v)
{
  tf2::Quaternion vq(v.x(), v.y(), v.z(), 0.0);
  tf2::Quaternion r = q * vq * q.inverse();
  return tf2::Vector3(r.x(), r.y(), r.z());
}
}  // namespace

ComputeArmTargetWorldNode::ComputeArmTargetWorldNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  std::string arm_path;
  if (auto p = getInput<std::string>("arm_calibration_path"); p && !p->empty()) {
    arm_path = *p;
  } else {
    arm_path = resolve_default_calib_path(kDefaultArmCalibName);
  }

  try {
    std::tie(R_a2t_, t_a2t_) = parse_calibration_json(arm_path);
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ComputeArmTargetWorld"),
      "Loaded arm calibration: %s", arm_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeArmTargetWorld"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList ComputeArmTargetWorldNode::providedPorts()
{
  return {
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "arm_observation",
      "Last CONFIRMED arm ObservationResult (e.g. {last_arm_obs})"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "tcp_pose_in_world",
      "Assumed current tcp pose in world (PoseStamped from blackboard, "
      "e.g. {head_pose} written by an earlier step's MoveArmToPose)"),
    BT::InputPort<std::string>(
      "arm_calibration_path", "",
      "Path to arm handeye JSON (must be _tcp.json variant); "
      "empty = default share/.../arm_calibration.json"),
    BT::InputPort<std::string>(
      "output_key", kDefaultOutputKey,
      "Blackboard key for the output Point (default 'arm_target_world')"),
    BT::OutputPort<geometry_msgs::msg::Point>(
      "arm_target_world", "World-frame target Point; also written to blackboard"),
  };
}

BT::NodeStatus ComputeArmTargetWorldNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  auto arm_in = getInput<yoloe_interfaces::msg::ObservationResult>(
    "arm_observation");
  if (!arm_in || arm_in->detections.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeArmTargetWorld"),
      "No arm detection; FAILURE");
    return BT::NodeStatus::FAILURE;
  }
  auto tcp_in = getInput<geometry_msgs::msg::PoseStamped>("tcp_pose_in_world");
  if (!tcp_in) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeArmTargetWorld"),
      "Missing tcp_pose_in_world; FAILURE");
    return BT::NodeStatus::FAILURE;
  }
  const auto& det = arm_in->detections.front();

  // P_arm_in_tcp = R_a2t · center + t_a2t
  // (calibration is T_camera -> tcp_link, so applying R_a2t + t_a2t
  // directly takes a camera-frame point into the tcp frame; do NOT
  // .inverse() / subtract — see file-top comment.)
  tf2::Vector3 arm_center(det.center_m.x, det.center_m.y, det.center_m.z);
  tf2::Vector3 P_arm_in_tcp = R_a2t_ * arm_center + t_a2t_;

  // P_arm_world = R_tcp · P_arm_in_tcp + t_tcp
  tf2::Quaternion q_tcp;
  tf2::fromMsg(tcp_in->pose.orientation, q_tcp);
  tf2::Vector3 P_arm_world =
    quat_rotate_vector(q_tcp, P_arm_in_tcp) +
    tf2::Vector3(tcp_in->pose.position.x, tcp_in->pose.position.y,
                 tcp_in->pose.position.z);

  geometry_msgs::msg::Point pt;
  pt.x = P_arm_world.x();
  pt.y = P_arm_world.y();
  pt.z = P_arm_world.z();

  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_key"); p && !p->empty()) {
    output_key = *p;
  }
  setOutput("arm_target_world", pt);
  config().blackboard->set<geometry_msgs::msg::Point>(output_key, pt);

  RCLCPP_INFO(rclcpp::get_logger("ComputeArmTargetWorld"),
    "det.class_name='%s' center_m=(%.4f, %.4f, %.4f)  "
    "tcp.pos=(%.4f, %.4f, %.4f) -> P_arm_world=(%.4f, %.4f, %.4f) [key='%s']",
    det.class_name.c_str(),
    det.center_m.x, det.center_m.y, det.center_m.z,
    tcp_in->pose.position.x, tcp_in->pose.position.y, tcp_in->pose.position.z,
    pt.x, pt.y, pt.z, output_key.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
