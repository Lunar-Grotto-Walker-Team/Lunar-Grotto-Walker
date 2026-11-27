// make_pose_from_point_and_reference.cpp — see header for semantics.
//
// Algorithm (pure data composition, no transforms):
//   pose.position    = point
//   pose.orientation = reference_pose.pose.orientation
//   pose.header.frame_id = world_frame (default "world")
//   blackboard.set(output_key, PoseStamped)
//
// This is intentionally trivial — it exists as a separate BT node so the
// XML can spell out "take head_target_world point + lookat_pose orientation
// → head_pose" as a single declarative step, rather than burying the same
// logic inside ComputeGraspPose (which re-anchors the lookat direction to
// the new target, defeating the cross-phase orientation-preservation the
// user wants to keep constant for visual comparison).

#include "lgw_bt_planner/make_pose_from_point_and_reference.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultOutputKey[]  = "composed_pose";
constexpr char kDefaultWorldFrame[] = "world";
}  // namespace

MakePoseFromPointAndReferenceNode::MakePoseFromPointAndReferenceNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList MakePoseFromPointAndReferenceNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>(
      "point",
      "World-frame target Point (e.g. {head_target_world} or "
      "{arm_target_world}); becomes pose.position"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "reference_pose",
      "Reference PoseStamped whose .pose.orientation is reused "
      "(e.g. {lookat_pose}); only orientation is read, position is ignored"),
    BT::InputPort<std::string>(
      "output_key", kDefaultOutputKey,
      "Blackboard key for the composed PoseStamped (default 'composed_pose')"),
    BT::InputPort<std::string>(
      "world_frame", kDefaultWorldFrame,
      "frame_id stamped on the output PoseStamped (default 'world')"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "composed_pose", "Composed PoseStamped; also written to blackboard"),
  };
}

BT::NodeStatus MakePoseFromPointAndReferenceNode::tick()
{
  auto pt_in = getInput<geometry_msgs::msg::Point>("point");
  if (!pt_in) {
    RCLCPP_WARN(rclcpp::get_logger("MakePoseFromPointAndReference"),
      "Missing 'point' input; FAILURE");
    return BT::NodeStatus::FAILURE;
  }
  auto ref_in = getInput<geometry_msgs::msg::PoseStamped>("reference_pose");
  if (!ref_in) {
    RCLCPP_WARN(rclcpp::get_logger("MakePoseFromPointAndReference"),
      "Missing 'reference_pose' input; FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  std::string world_frame = kDefaultWorldFrame;
  if (auto p = getInput<std::string>("world_frame"); p && !p->empty()) {
    world_frame = *p;
  }
  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_key"); p && !p->empty()) {
    output_key = *p;
  }

  geometry_msgs::msg::PoseStamped out;
  // Leave stamp as zero — MoveArmToPose only reads pose, not stamp;
  // world_frame is the contract that matters for the MoveGroup goal.
  out.header.stamp    = rclcpp::Time(0, 0, RCL_ROS_TIME);
  out.header.frame_id = world_frame;
  out.pose.position.x = pt_in->x;
  out.pose.position.y = pt_in->y;
  out.pose.position.z = pt_in->z;
  out.pose.orientation = ref_in->pose.orientation;

  setOutput("composed_pose", out);
  config().blackboard->set<geometry_msgs::msg::PoseStamped>(output_key, out);

  RCLCPP_INFO(rclcpp::get_logger("MakePoseFromPointAndReference"),
    "point=(%.4f, %.4f, %.4f)  ref.quat=(%.4f, %.4f, %.4f, %.4f)  "
    "frame='%s'  [key='%s']",
    pt_in->x, pt_in->y, pt_in->z,
    ref_in->pose.orientation.x, ref_in->pose.orientation.y,
    ref_in->pose.orientation.z, ref_in->pose.orientation.w,
    world_frame.c_str(), output_key.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
