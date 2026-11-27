// verify_head_arm_consistency.cpp — see header for semantics.
//
// Algorithm:
//   P_head_world = R_h2b · head.center_m + t_h2b
//   P_arm_in_tcp = R_a2t · arm.center_m + t_a2t       (arm optical → tcp)
//   P_arm_world  = R_tcp · P_arm_in_tcp + t_tcp        (tcp → world, from
//                                                       `tcp_pose_in_world`)
//   error_m      = ||P_head_world − P_arm_world||
//   SUCCESS iff error_m ≤ max_distance_m
//
// NOTE on the arm_calibration.json convention: although _meta.transformation
// originally read "tcp_link -> camera_arm_color_optical_frame", the actual
// numeric data in that file represents T_camera -> tcp_link (i.e. R_a2t
// maps camera-frame vectors to tcp-frame vectors; t_a2t is the camera
// origin expressed in the tcp frame). The formula above applies R_a2t +
// t_a2t directly. If you see the "R_a2t.inverse() * (center - t_a2t)"
// version, that's the pre-2026-10-09 bug that produced ~25 cm world-frame
// error in head_arm_independent_lookat.
//
// Pure helpers (`parse_calibration_json`, `quat_rotate_vector`) are duplicated
// locally — the originals live in the anonymous namespace of
// compute_lookat_pose_node.cpp and aren't part of any public header. Reusing
// them would require a refactor (extract into a lookat_math_helpers.hpp) that
// touches already-stable code; YAGNI here.

#include "lgw_bt_planner/verify_head_arm_consistency.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // parse_calibration_json

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultWorldFrame[]    = "world";
constexpr char kDefaultMaxDistance[]   = "0.05";
constexpr char kDefaultHeadCalibName[] = "head_calibration.json";
constexpr char kDefaultArmCalibName[]  = "arm_calibration.json";
constexpr char kArmTargetWorldKey[]    = "arm_target_world";
constexpr char kConsistencyErrorKey[]  = "consistency_error_m";

// Pure-quaternion rotate-vector: v' = q ⊗ (0, v) ⊗ q*. Mirrors the helper
// in compute_lookat_pose_node.cpp's anonymous namespace; duplicated here
// rather than refactored to a public header (see file-top comment).
tf2::Vector3 quat_rotate_vector(const tf2::Quaternion& q, const tf2::Vector3& v)
{
  tf2::Quaternion vq(v.x(), v.y(), v.z(), 0.0);
  tf2::Quaternion r = q * vq * q.inverse();
  return tf2::Vector3(r.x(), r.y(), r.z());
}

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}
}  // namespace

VerifyHeadArmConsistencyNode::VerifyHeadArmConsistencyNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  // Same default-fallback idiom as ComputeLookatPoseNode: BT.CPP
  // providedPorts default "" makes getInput truthy with empty string, so
  // the explicit !empty() check is required to trigger fallback (see
  // feedback_bt_inputport_default memory).
  std::string head_path, arm_path;
  if (auto p = getInput<std::string>("head_calibration_path"); p && !p->empty()) {
    head_path = *p;
  } else {
    head_path = resolve_default_calib_path(kDefaultHeadCalibName);
  }
  if (auto p = getInput<std::string>("arm_calibration_path"); p && !p->empty()) {
    arm_path = *p;
  } else {
    arm_path = resolve_default_calib_path(kDefaultArmCalibName);
  }

  try {
    std::tie(R_h2b_, t_h2b_) = parse_calibration_json(head_path);
    std::tie(R_a2t_, t_a2t_) = parse_calibration_json(arm_path);
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("VerifyHeadArmConsistency"),
      "Loaded calibration: head=%s arm=%s", head_path.c_str(),
      arm_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VerifyHeadArmConsistency"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList VerifyHeadArmConsistencyNode::providedPorts()
{
  return {
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "head_observation", "Last CONFIRMED head ObservationResult"),
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "arm_observation", "Last CONFIRMED arm ObservationResult"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "tcp_pose_in_world",
      "Current tcp_link pose in world frame, from "
      "ComputeLookatPose blackboard 'lookat_pose'"),
    BT::InputPort<std::string>(
      "max_distance_m", kDefaultMaxDistance,
      "Max allowed world-frame distance between head/arm targets (meters)"),
    BT::InputPort<std::string>(
      "head_calibration_path", "",
      "Path to head handeye JSON; empty = default share/.../head_calibration.json"),
    BT::InputPort<std::string>(
      "arm_calibration_path", "",
      "Path to arm handeye JSON (must be _tcp.json variant); "
      "empty = default share/.../arm_calibration.json"),
  };
}

BT::NodeStatus VerifyHeadArmConsistencyNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  // 1. Read inputs.
  auto head_in = getInput<yoloe_interfaces::msg::ObservationResult>(
    "head_observation");
  if (!head_in || head_in->detections.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  auto arm_in = getInput<yoloe_interfaces::msg::ObservationResult>(
    "arm_observation");
  if (!arm_in || arm_in->detections.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  auto tcp_in = getInput<geometry_msgs::msg::PoseStamped>("tcp_pose_in_world");
  if (!tcp_in) {
    return BT::NodeStatus::FAILURE;
  }

  double max_distance_m = 0.05;
  if (auto p = getInput<std::string>("max_distance_m"); p && !p->empty()) {
    try {
      max_distance_m = std::stod(*p);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("VerifyHeadArmConsistency"),
        "Bad max_distance_m '%s': %s", p->c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto& head_det = head_in->detections.front();
  const auto& arm_det  = arm_in->detections.front();

  // 2. P_head_world = R_h2b · head.center_m + t_h2b.
  tf2::Vector3 P_target_head(head_det.center_m.x, head_det.center_m.y,
                             head_det.center_m.z);
  // Rh_p = R_h2b · P_target_head (intermediate, useful for log trace)
  tf2::Vector3 Rh_p = R_h2b_ * P_target_head;
  tf2::Vector3 P_head_world = Rh_p + t_h2b_;

  // 3. P_arm_in_tcp = R_a2t · arm.center_m + t_a2t
  //    (calibration is T_camera -> tcp_link, applied directly; do NOT
  //    .inverse() / subtract — see file-top comment.)
  tf2::Vector3 arm_center(arm_det.center_m.x, arm_det.center_m.y,
                          arm_det.center_m.z);
  tf2::Vector3 P_arm_in_tcp = R_a2t_ * arm_center + t_a2t_;

  // 4. P_arm_world = R_tcp · P_arm_in_tcp + t_tcp.
  //    Quaternion path: q_tcp from tcp_in.pose.orientation, then
  //    rotate P_arm_in_tcp by q_tcp and add tcp position.
  tf2::Quaternion q_tcp;
  tf2::fromMsg(tcp_in->pose.orientation, q_tcp);
  // Rtcp_Parm = q_tcp · P_arm_in_tcp (intermediate, before + t_tcp)
  tf2::Vector3 Rtcp_Parm = quat_rotate_vector(q_tcp, P_arm_in_tcp);
  tf2::Vector3 P_arm_world =
    Rtcp_Parm +
    tf2::Vector3(tcp_in->pose.position.x, tcp_in->pose.position.y,
                 tcp_in->pose.position.z);

  // 5. error = ||P_head_world − P_arm_world||.
  tf2::Vector3 diff = P_head_world - P_arm_world;
  double error_m = diff.length();

  // 6. Write blackboard — arm_target_world + consistency_error_m.
  //    Always write (even on FAILURE) so downstream tools / Groot2 can
  //    inspect the last comparison; the SUCCESS/FAILURE semantics only
  //    gate the next Sequence child.
  geometry_msgs::msg::Point arm_target_world;
  arm_target_world.x = P_arm_world.x();
  arm_target_world.y = P_arm_world.y();
  arm_target_world.z = P_arm_world.z();
  config().blackboard->set<geometry_msgs::msg::Point>(
    kArmTargetWorldKey, arm_target_world);
  config().blackboard->set<double>(kConsistencyErrorKey, error_m);

  // 7. Detailed trace log — every intermediate vector so the chain
  //    arm_optical → tcp → world can be reconstructed and compared against
  //    ComputeLookatPose's head-side trace (P_target_base = same as our
  //    P_head_world here). Unconditional (matches existing INFO convention
  //    in this file); remove or gate behind BT_PLANNER_DEBUG once root cause
  //    is identified.
  const auto& log = rclcpp::get_logger("VerifyHeadArmConsistency");
  RCLCPP_INFO(log, "=== consistency cycle start ===");
  RCLCPP_INFO(log,
    "  HEAD obs: n_det=%zu  frame_id='%s'  status=%u",
    head_in->detections.size(),
    head_in->header.frame_id.c_str(),
    head_in->status);
  RCLCPP_INFO(log,
    "  HEAD det.front: instance_id=%u  class_id=%d  class_name='%s'  "
    "confidence=%.3f  position_valid=%d  valid_depth_pts=%u  "
    "mask_area_px=%u",
    head_det.instance_id, head_det.class_id,
    head_det.class_name.c_str(), head_det.confidence,
    head_det.position_valid ? 1 : 0,
    head_det.valid_depth_points,
    head_det.mask_area_px);
  RCLCPP_INFO(log,
    "  HEAD center_m=(%.4f, %.4f, %.4f)",
    head_det.center_m.x, head_det.center_m.y, head_det.center_m.z);
  RCLCPP_INFO(log,
    "  HEAD step1: R_h2b*center=(%.4f, %.4f, %.4f) + t_h2b=(%.4f, %.4f, %.4f)"
    " -> P_head_world=(%.4f, %.4f, %.4f)",
    Rh_p.x(), Rh_p.y(), Rh_p.z(),
    t_h2b_.x(), t_h2b_.y(), t_h2b_.z(),
    P_head_world.x(), P_head_world.y(), P_head_world.z());
  RCLCPP_INFO(log,
    "  ARM  obs: n_det=%zu  frame_id='%s'  status=%u",
    arm_in->detections.size(),
    arm_in->header.frame_id.c_str(),
    arm_in->status);
  RCLCPP_INFO(log,
    "  ARM  det.front: instance_id=%u  class_id=%d  class_name='%s'  "
    "confidence=%.3f  position_valid=%d  valid_depth_pts=%u  "
    "mask_area_px=%u",
    arm_det.instance_id, arm_det.class_id,
    arm_det.class_name.c_str(), arm_det.confidence,
    arm_det.position_valid ? 1 : 0,
    arm_det.valid_depth_points,
    arm_det.mask_area_px);
  RCLCPP_INFO(log,
    "  ARM  center_m=(%.4f, %.4f, %.4f)",
    arm_det.center_m.x, arm_det.center_m.y, arm_det.center_m.z);
  RCLCPP_INFO(log,
    "  ARM  step2: R_a2t*center + t_a2t -> P_arm_in_tcp=(%.4f, %.4f, %.4f)",
    P_arm_in_tcp.x(), P_arm_in_tcp.y(), P_arm_in_tcp.z());
  RCLCPP_INFO(log,
    "  TCP  pose_in_world: pos=(%.4f, %.4f, %.4f)  frame_id='%s'  "
    "quat_xyzw=(%.4f, %.4f, %.4f, %.4f)",
    tcp_in->pose.position.x, tcp_in->pose.position.y, tcp_in->pose.position.z,
    tcp_in->header.frame_id.c_str(),
    tcp_in->pose.orientation.x, tcp_in->pose.orientation.y,
    tcp_in->pose.orientation.z, tcp_in->pose.orientation.w);
  RCLCPP_INFO(log,
    "  ARM  step3: R_tcp*P_arm_in_tcp=(%.4f, %.4f, %.4f) + t_tcp -> "
    "P_arm_world=(%.4f, %.4f, %.4f)",
    Rtcp_Parm.x(), Rtcp_Parm.y(), Rtcp_Parm.z(),
    P_arm_world.x(), P_arm_world.y(), P_arm_world.z());
  RCLCPP_INFO(log,
    "  RESULT: P_head_world=(%.4f, %.4f, %.4f)  P_arm_world=(%.4f, %.4f, %.4f)"
    "  error_m=%.4f  threshold=%.4f  -> %s",
    P_head_world.x(), P_head_world.y(), P_head_world.z(),
    P_arm_world.x(), P_arm_world.y(), P_arm_world.z(),
    error_m, max_distance_m,
    (error_m <= max_distance_m) ? "SUCCESS" : "FAILURE");

  return (error_m <= max_distance_m) ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::FAILURE;
}

}  // namespace lgw_bt_planner
