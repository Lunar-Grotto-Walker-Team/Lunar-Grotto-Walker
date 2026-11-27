// compute_grasp_pose.cpp — see header for semantics.
//
// Algorithm (re-anchored lookat math):
//   P_target_world = arm_target_world         (Point from blackboard)
//   P_ee_world     = parse_semicolon_xyz(ee_position_world)
//   direction      = (P_target_world − P_ee_world).normalized()
//   R_arm_in_base  = build_lookat_orientation(direction) + uprightness flip
//   R_tcp_in_base  = R_arm_in_base · R_a2t⁻¹
//   q_tcp_in_base  = quaternion from R_tcp_in_base       (grasp orientation)
//   P_tcp_target   = P_target_world                       (tcp at target)
//   output PoseStamped(P_tcp_target, q_tcp_in_base) → blackboard 'grasp_pose'

#include "lgw_bt_planner/compute_grasp_pose.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // build_lookat_orientation, parse_calibration_json

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultEePosition[] = "0.5;0.0;0.3";
constexpr char kDefaultOutputKey[]  = "grasp_pose";
constexpr char kDefaultHeadCalibName[] = "head_calibration.json";
constexpr char kDefaultArmCalibName[]  = "arm_calibration.json";

tf2::Vector3 parse_semicolon_xyz(const std::string& s)
{
  tf2::Vector3 v;
  size_t p1 = s.find(';');
  size_t p2 = s.find(';', p1 + 1);
  if (p1 == std::string::npos || p2 == std::string::npos) {
    throw std::runtime_error(
      "parse_semicolon_xyz: expected 'x;y;z' format, got '" + s + "'");
  }
  v.setX(std::stod(s.substr(0, p1)));
  v.setY(std::stod(s.substr(p1 + 1, p2 - p1 - 1)));
  v.setZ(std::stod(s.substr(p2 + 1)));
  return v;
}

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}
}  // namespace

ComputeGraspPoseNode::ComputeGraspPoseNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
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
    RCLCPP_INFO(rclcpp::get_logger("ComputeGraspPose"),
      "Loaded calibration: head=%s arm=%s", head_path.c_str(),
      arm_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeGraspPose"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList ComputeGraspPoseNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>(
      "arm_target_world",
      "Arm-camera target center in world frame, from "
      "VerifyHeadArmConsistency blackboard 'arm_target_world'"),
    BT::InputPort<std::string>(
      "ee_position_world", kDefaultEePosition,
      "Camera-anchor in world frame for direction recomputation ('x;y;z')"),
    BT::InputPort<std::string>(
      "world_frame", "world",
      "frame_id for output PoseStamped"),
    BT::InputPort<std::string>(
      "head_calibration_path", "",
      "Path to head handeye JSON; empty = default share/.../head_calibration.json"),
    BT::InputPort<std::string>(
      "arm_calibration_path", "",
      "Path to arm handeye JSON (must be _tcp.json variant); "
      "empty = default share/.../arm_calibration.json"),
    BT::InputPort<std::string>(
      "output_pose_key", kDefaultOutputKey,
      "Blackboard key for the output PoseStamped (default 'grasp_pose')"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "grasp_pose", "Computed tcp_link grasp pose; also written to blackboard"),
  };
}

BT::NodeStatus ComputeGraspPoseNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  // 1. Read arm_target_world from blackboard (must come from prior
  //    VerifyHeadArmConsistency SUCCESS in the same Sequence).
  auto pt_in = getInput<geometry_msgs::msg::Point>("arm_target_world");
  if (!pt_in) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeGraspPose"),
      "Missing arm_target_world; VerifyHeadArmConsistency must run first");
    return BT::NodeStatus::FAILURE;
  }
  tf2::Vector3 P_target_world(pt_in->x, pt_in->y, pt_in->z);

  // 2. Read ee_position_world (anchor for direction recomputation).
  std::string ee_str = kDefaultEePosition;
  if (auto p = getInput<std::string>("ee_position_world"); p && !p->empty()) {
    ee_str = *p;
  }
  std::string world_frame = "world";
  if (auto p = getInput<std::string>("world_frame"); p && !p->empty()) {
    world_frame = *p;
  }
  tf2::Vector3 P_ee_world;
  try {
    P_ee_world = parse_semicolon_xyz(ee_str);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeGraspPose"),
      "Bad ee_position_world '%s': %s", ee_str.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  // 3. Recompute direction with arm_target_world as the target.
  tf2::Vector3 raw_dir = P_target_world - P_ee_world;
  if (raw_dir.length() < 1e-3) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeGraspPose"),
      "arm_target_world and ee_position_world coincide; geometric degeneracy");
    return BT::NodeStatus::FAILURE;
  }
  tf2::Vector3 direction = raw_dir.normalized();

  // 4. build_lookat_orientation + uprightness flip (same as
  //    ComputeLookatPoseNode step 3).
  tf2::Matrix3x3 R_arm_in_base;
  try {
    R_arm_in_base = build_lookat_orientation(direction,
                                             tf2::Vector3(0, 0, 1));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeGraspPose"),
      "build_lookat_orientation failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  {
    const tf2::Vector3 base_z(0.0, 0.0, 1.0);
    tf2::Vector3 cam_y(R_arm_in_base[0][1], R_arm_in_base[1][1],
                       R_arm_in_base[2][1]);
    if (cam_y.dot(base_z) >= 0.0) {
      // Flip cam_x (col 0) and cam_y (col 1); cam_z (col 2, = direction)
      // untouched. det = +1 (right-handed). See ComputeLookatPoseNode
      // step3 for the rationale.
      for (int i = 0; i < 3; ++i) {
        R_arm_in_base[i][0] = -R_arm_in_base[i][0];
        R_arm_in_base[i][1] = -R_arm_in_base[i][1];
      }
    }
  }

  // 5. R_tcp = R_arm · R_a2t⁻¹; convert to quaternion (grasp orientation).
  tf2::Quaternion q_arm_in_base, q_a2t;
  R_arm_in_base.getRotation(q_arm_in_base);
  R_a2t_.getRotation(q_a2t);
  tf2::Quaternion q_tcp_in_base = q_arm_in_base * q_a2t.inverse();

  // 6. Output: tcp at arm_target_world, orientation = lookat orientation.
  //    Crucially, NO t_a2t compensation here — the spec says tcp goes
  //    straight to arm_target_world, not to (arm_target_world − R_tcp·t_a2t).
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = world_frame;
  pose.pose.position.x = P_target_world.x();
  pose.pose.position.y = P_target_world.y();
  pose.pose.position.z = P_target_world.z();
  pose.pose.orientation.x = q_tcp_in_base.x();
  pose.pose.orientation.y = q_tcp_in_base.y();
  pose.pose.orientation.z = q_tcp_in_base.z();
  pose.pose.orientation.w = q_tcp_in_base.w();
  pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_pose_key"); p && !p->empty()) {
    output_key = *p;
  }
  setOutput("grasp_pose", pose);
  config().blackboard->set(output_key, pose);

  RCLCPP_INFO(rclcpp::get_logger("ComputeGraspPose"),
    "P_target_world=(%.4f, %.4f, %.4f)  P_tcp_target=(%.4f, %.4f, %.4f)  "
    "q_tcp_xyzw=(%.4f, %.4f, %.4f, %.4f)",
    P_target_world.x(), P_target_world.y(), P_target_world.z(),
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    q_tcp_in_base.x(), q_tcp_in_base.y(), q_tcp_in_base.z(),
    q_tcp_in_base.w());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
