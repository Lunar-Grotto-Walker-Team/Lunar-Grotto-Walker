// src/compute_grasp_poses_from_track.cpp — see the header for the contract.

#include "lgw_bt_planner/compute_grasp_poses_from_track.hpp"

#include "lgw_bt_planner/calibration_json.hpp"
#include "lgw_bt_planner/compute_lookat_pose_node.hpp"
#include "lgw_bt_planner/target_track_query.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <utility>

namespace lgw_bt_planner
{

namespace
{
constexpr char kTrackTopic[] = "/perception/tracks/unified";
constexpr char kDefaultArmCalib[] = "arm_calibration.json";
constexpr char kDefaultPreGraspKey[] = "pre_grasp_pose";
constexpr char kDefaultGraspKey[] = "grasp_pose";

geometry_msgs::msg::PoseStamped make_pose(
  const std::string & frame, const tf2::Vector3 & p,
  const tf2::Quaternion & q)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = p.x();
  pose.pose.position.y = p.y();
  pose.pose.position.z = p.z();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}
}  // namespace

ComputeGraspPosesFromTrack::ComputeGraspPosesFromTrack(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
            "ComputeGraspPosesFromTrack: RosNodeParams::nh expired");
  }
  logger_ = node->get_logger();
  sub_ = node->create_subscription<yoloe_interfaces::msg::UnifiedTrackArray>(
    kTrackTopic, rclcpp::QoS(10),
    [this](const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg) {
      on_track_array(msg);
    });

  tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_, node.get(), true);

  // Camera -> tcp rotation for the optical-axis direction.  The JSON is the
  // _tcp variant (12 cm tcp offset already applied, see its _meta).
  try {
    const std::string path =
      ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
      "/calibration/" + kDefaultArmCalib;
    std::tie(R_cam_tcp_, std::ignore) = parse_calibration_json(path);
    calibration_ok_ = true;
    RCLCPP_INFO(
      logger_, "[%s] loaded arm hand-eye calibration: %s",
      name.c_str(), path.c_str());
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      logger_, "[%s] calibration load failed (%s); node always FAILURE",
      name.c_str(), exc.what());
  }
}

void ComputeGraspPosesFromTrack::on_track_array(
  const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = *msg;
  seen_ = true;
}

BT::PortsList ComputeGraspPosesFromTrack::providedPorts()
{
  return {
    // 输出位姿的坐标系必须是 MoveIt 规划模型认的那个。
    // 2026-11-18 真机: TF 里臂侧是前缀化的 arm1/base_link (与 Go2 的 base_link
    // 区分), 但 MoveIt 的 URDF 模型里仍然是无前缀的 base_link。用 arm1/base_link
    // 输出会让 move_group 打 "getFrameTransform() did not find a frame with
    // name arm1/base_link" -> "Invalid goal state" -> 抓取位永远规划不出来。
    BT::InputPort<std::string>(
      "arm_base_frame", "base_link", "MoveIt model frame for both output poses"),
    // tracker 的 position_arm_base 所在坐标系(TF 侧, 带前缀)。
    BT::InputPort<std::string>(
      "track_frame", "arm1/base_link",
      "Frame of UnifiedTrack.position_arm_base (TF side, prefixed)"),
    BT::InputPort<std::string>(
      "tcp_frame", "arm1/tcp_link", "Current tcp frame for the look-at pose"),
    BT::InputPort<double>(
      "pre_grasp_distance_m", 0.100,
      "Pre-grasp offset along the camera optical axis, camera side"),
    BT::InputPort<std::string>(
      "output_pre_grasp_key", kDefaultPreGraspKey,
      "Blackboard key for the pre-grasp pose"),
    BT::InputPort<std::string>(
      "output_grasp_key", kDefaultGraspKey,
      "Blackboard key for the grasp pose"),
  };
}

BT::NodeStatus ComputeGraspPosesFromTrack::tick()
{
  if (!calibration_ok_) {
    return BT::NodeStatus::FAILURE;
  }

  std::string arm_base = "base_link";
  std::string track_frame = "arm1/base_link";
  std::string tcp_frame = "arm1/tcp_link";
  std::string pre_key = kDefaultPreGraspKey;
  std::string grasp_key = kDefaultGraspKey;
  double pre_distance = 0.100;
  getInput("arm_base_frame", arm_base);
  getInput("track_frame", track_frame);
  getInput("tcp_frame", tcp_frame);
  getInput("pre_grasp_distance_m", pre_distance);
  getInput("output_pre_grasp_key", pre_key);
  getInput("output_grasp_key", grasp_key);

  yoloe_interfaces::msg::UnifiedTrackArray arr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!seen_) {
      RCLCPP_WARN(logger_, "[%s] no track array yet", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    arr = latest_;
  }
  const auto * track = track_query::selected_track(arr);
  if (track == nullptr) {
    RCLCPP_WARN(logger_, "[%s] no selected target", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const tf2::Duration zero = tf2::durationFromSec(0.0);
  if (!tf_->canTransform(
      arm_base, tcp_frame, tf2::TimePointZero, zero))
  {
    RCLCPP_WARN(
      logger_, "[%s] no TF %s <- %s", name().c_str(),
      arm_base.c_str(), tcp_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }
  tf2::Transform T_tcp;
  try {
    const auto stamped = tf_->lookupTransform(
      arm_base, tcp_frame, tf2::TimePointZero, zero);
    tf2::Quaternion q(
      stamped.transform.rotation.x, stamped.transform.rotation.y,
      stamped.transform.rotation.z, stamped.transform.rotation.w);
    q.normalize();
    T_tcp.setRotation(q);
    T_tcp.setOrigin(tf2::Vector3(
      stamped.transform.translation.x, stamped.transform.translation.y,
      stamped.transform.translation.z));
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN(logger_, "[%s] TF lookup failed: %s", name().c_str(), exc.what());
    return BT::NodeStatus::FAILURE;
  }

  tf2::Vector3 p_target(
    track->position_arm_base.x, track->position_arm_base.y,
    track->position_arm_base.z);
  // tracker 给的是 TF 侧坐标系(默认 arm1/base_link)下的点, 输出位姿要求
  // 在 MoveIt 模型坐标系(默认 base_link)下, 按 TF 变换过去。两者今天重合
  // (tracker 发 base_link→arm1/world 恒等 + RSP 挂载固定), 但按变换写才在
  // 将来改挂载时不会错。
  if (track_frame != arm_base) {
    if (!tf_->canTransform(arm_base, track_frame, tf2::TimePointZero, zero)) {
      RCLCPP_WARN(
        logger_, "[%s] no TF %s <- %s (track frame -> output frame)",
        name().c_str(), arm_base.c_str(), track_frame.c_str());
      return BT::NodeStatus::FAILURE;
    }
    try {
      const auto t = tf_->lookupTransform(
        arm_base, track_frame, tf2::TimePointZero, zero);
      const tf2::Quaternion q(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w);
      const tf2::Vector3 o(
        t.transform.translation.x, t.transform.translation.y,
        t.transform.translation.z);
      p_target = tf2::Transform(q, o) * p_target;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN(
        logger_, "[%s] TF %s <- %s failed: %s", name().c_str(),
        arm_base.c_str(), track_frame.c_str(), exc.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  // 2026-11-19 真机: 姿态不再复用"冻结的伺服 TCP 姿态", 改为解析构造。
  //
  // 旧做法(已废弃): R_tcp = 当前 TCP 姿态, axis = R_tcp·R_cam_tcp·z。
  //   - 伺服在 SCANNING 时冻结到的是扫描姿态, 不是瞄准目标的姿态;
  //   - 伺服求解器是在**它自己那个位置**选腕部姿态, 沿光轴后退 25 cm 后
  //     同一个姿态可能已顶到腕部限位 -> MoveIt -27 GOAL_STATE_INVALID;
  //   - 后退方向用的是相机光轴, 不是工具轴。
  //
  // 新做法: 方向 = 当前 TCP 位置(只取平移, 不取旋转) -> 目标点, 与
  // ComputeGraspPose 节点同一套数学(build_lookat_orientation +
  // apply_uprightness_flip), 再乘手眼 R_cam_tcp⁻¹ 得到 TCP 姿态。
  // 结果只依赖目标点和"手当前在哪", 与伺服状态/腕部姿态无关。
  const tf2::Vector3 p_ref = T_tcp.getOrigin();
  tf2::Vector3 axis = p_target - p_ref;
  if (axis.length() < 1e-3) {
    RCLCPP_WARN(
      logger_, "[%s] target coincides with the tcp position; no approach "
      "direction", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  axis.normalize();

  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  tf2::Quaternion q_tcp;
  try {
    const tf2::Matrix3x3 R_cam = apply_uprightness_flip(
      build_lookat_orientation(axis, world_up), world_up);
    const tf2::Matrix3x3 R_tcp = R_cam * R_cam_tcp_.inverse();
    R_tcp.getRotation(q_tcp);
    q_tcp.normalize();
  } catch (const std::exception & exc) {
    RCLCPP_WARN(
      logger_, "[%s] orientation build failed: %s", name().c_str(), exc.what());
    return BT::NodeStatus::FAILURE;
  }

  const tf2::Vector3 p_grasp = p_target;
  // Back off from the target toward the camera (axis points camera -> target).
  const tf2::Vector3 p_pre = p_target - axis * pre_distance;

  // setOutput() only writes keys this node declares as OutputPorts; these two
  // keys are dynamic (they come from the output_*_key input ports), so write
  // the blackboard directly -- and CHECK the result.
  //
  // 2026-11-18 real-robot lesson: the unchecked setOutput() silently did
  // nothing, MoveArmToPose's getInput<PoseStamped>("goal") returned nullopt,
  // setGoal() returned false and BT::RosActionNode reported FAILURE without
  // logging anything -- three grasp attempts died in 4 ms each with no error.
  // ComputeLookatPoseNode avoided this by writing both the declared port and
  // config().blackboard->set(output_key, ...).
  const auto pre_pose = make_pose(arm_base, p_pre, q_tcp);
  const auto grasp_pose = make_pose(arm_base, p_grasp, q_tcp);
  // Blackboard::set() returns void in this BT.CPP version, so read back the
  // same typed path MoveArmToPose's getInput<PoseStamped> will use.
  config().blackboard->set(pre_key, pre_pose);
  config().blackboard->set(grasp_key, grasp_pose);
  geometry_msgs::msg::PoseStamped verify;
  if (!config().blackboard->get(pre_key, verify) ||
      !config().blackboard->get(grasp_key, verify))
  {
    RCLCPP_ERROR(
      logger_, "[%s] blackboard write failed (pre='%s' grasp='%s')",
      name().c_str(), pre_key.c_str(), grasp_key.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    logger_,
    "[%s] target id=%lu class=%s p=(%.3f, %.3f, %.3f) axis=(%.3f, %.3f, %.3f) "
    "pre=(%.3f, %.3f, %.3f) frame=%s",
    name().c_str(), static_cast<unsigned long>(track->track_id),
    track->class_name.c_str(),
    p_target.x(), p_target.y(), p_target.z(),
    axis.x(), axis.y(), axis.z(),
    p_pre.x(), p_pre.y(), p_pre.z(), arm_base.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
