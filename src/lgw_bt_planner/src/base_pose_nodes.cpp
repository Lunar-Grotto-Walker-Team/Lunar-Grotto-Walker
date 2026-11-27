// src/base_pose_nodes.cpp — see the header for the contract.

#include "lgw_bt_planner/base_pose_nodes.hpp"

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <utility>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultBaseFrame[] = "odom";
// TF 侧臂基座是带前缀的; MoveIt 模型才是不带前缀的 base_link。
constexpr char kDefaultRobotFrame[] = "arm1/base_link";
constexpr char kDefaultCenterKey[] = "base_center";
}  // namespace

// ---------------------------------------------------------------------------
// RecordBasePose
// ---------------------------------------------------------------------------

RecordBasePose::RecordBasePose(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("RecordBasePose: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_, node.get(), true);
}

BT::PortsList RecordBasePose::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "base_frame", kDefaultBaseFrame, "Frame the base centre is stored in"),
    BT::InputPort<std::string>(
      "robot_frame", kDefaultRobotFrame,
      "Robot frame sampled as the base centre (TF side, prefixed)"),
    BT::InputPort<std::string>(
      "output_key", kDefaultCenterKey, "Blackboard key for the Point"),
  };
}

BT::NodeStatus RecordBasePose::tick()
{
  std::string base_frame = kDefaultBaseFrame;
  std::string robot_frame = kDefaultRobotFrame;
  std::string key = kDefaultCenterKey;
  getInput("base_frame", base_frame);
  getInput("robot_frame", robot_frame);
  getInput("output_key", key);

  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_->canTransform(
      base_frame, robot_frame, tf2::TimePointZero, zero))
  {
    RCLCPP_WARN(
      logger_, "[%s] no TF %s <- %s yet; base centre NOT recorded",
      name().c_str(), base_frame.c_str(), robot_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto stamped = tf_->lookupTransform(
      base_frame, robot_frame, tf2::TimePointZero, zero);
    geometry_msgs::msg::Point p;
    p.x = stamped.transform.translation.x;
    p.y = stamped.transform.translation.y;
    p.z = stamped.transform.translation.z;
    config().blackboard->set(key, p);
    RCLCPP_INFO(
      logger_, "[%s] base centre pinned: x=%.3f y=%.3f z=%.3f (frame=%s)",
      name().c_str(), p.x, p.y, p.z, base_frame.c_str());
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN(logger_, "[%s] TF lookup failed: %s", name().c_str(), exc.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ---------------------------------------------------------------------------
// IsWithinBaseRadius
// ---------------------------------------------------------------------------

IsWithinBaseRadius::IsWithinBaseRadius(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError("IsWithinBaseRadius: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_, node.get(), true);
}

BT::PortsList IsWithinBaseRadius::providedPorts()
{
  return {
    BT::InputPort<std::string>("base_frame", kDefaultBaseFrame),
    BT::InputPort<std::string>("robot_frame", kDefaultRobotFrame),
    BT::InputPort<std::string>("center_key", kDefaultCenterKey,
                              "Blackboard key holding the base centre Point"),
    BT::InputPort<double>("radius_m", 0.5, "XoY radius around the centre"),
  };
}

BT::NodeStatus IsWithinBaseRadius::tick()
{
  std::string base_frame = kDefaultBaseFrame;
  std::string robot_frame = kDefaultRobotFrame;
  std::string key = kDefaultCenterKey;
  double radius = 0.5;
  getInput("base_frame", base_frame);
  getInput("robot_frame", robot_frame);
  getInput("center_key", key);
  getInput("radius_m", radius);

  geometry_msgs::msg::Point center;
  if (!config().blackboard->get(key, center)) {
    // 还没记录基地圆心: 当作"不在基地", 由外层决定是否继续采集。
    return BT::NodeStatus::FAILURE;
  }

  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_->canTransform(
      base_frame, robot_frame, tf2::TimePointZero, zero))
  {
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto stamped = tf_->lookupTransform(
      base_frame, robot_frame, tf2::TimePointZero, zero);
    const double dx = stamped.transform.translation.x - center.x;
    const double dy = stamped.transform.translation.y - center.y;
    const double d = std::hypot(dx, dy);
    RCLCPP_INFO_THROTTLE(
      logger_, log_clock_, 2000,
      "[%s] distance to base centre: %.2f m (radius %.2f m)",
      name().c_str(), d, radius);
    return (d <= radius) ? BT::NodeStatus::SUCCESS
                         : BT::NodeStatus::FAILURE;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN(logger_, "[%s] TF lookup failed: %s", name().c_str(), exc.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace lgw_bt_planner
