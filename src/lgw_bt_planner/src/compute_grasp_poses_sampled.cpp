// src/compute_grasp_poses_sampled.cpp — see the header for the contract.

#include "lgw_bt_planner/compute_grasp_poses_sampled.hpp"

#include "lgw_bt_planner/target_track_query.hpp"

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/logging.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace lgw_bt_planner
{

namespace
{
constexpr char kTrackTopic[] = "/perception/tracks/unified";
constexpr char kJointStateTopic[] = "/feedback/joint_states";
constexpr char kIkService[] = "/compute_ik";
constexpr char kMoveAction[] = "/move_action";
constexpr char kDefaultPreGraspKey[] = "pre_grasp_pose";
constexpr char kDefaultGraspKey[] = "grasp_pose";

geometry_msgs::msg::PoseStamped make_pose(
  const std::string & frame, const tf2::Vector3 & p, const tf2::Matrix3x3 & R)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = p.x();
  pose.pose.position.y = p.y();
  pose.pose.position.z = p.z();
  tf2::Quaternion q;
  R.getRotation(q);
  q.normalize();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

std::vector<double> parse_semicolon_doubles(const std::string & s)
{
  std::vector<double> out;
  std::size_t start = 0;
  while (start <= s.size()) {
    const std::size_t end = s.find(';', start);
    const std::string token =
      s.substr(start, end == std::string::npos ? std::string::npos : end - start);
    try {
      if (!token.empty()) {
        out.push_back(std::stod(token));
      }
    } catch (const std::exception &) {
      // ignore malformed tokens: the caller falls back to the defaults
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return out;
}

double deg2rad(double d) {return d * M_PI / 180.0;}

}  // namespace

ComputeGraspPosesSampled::ComputeGraspPosesSampled(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw BT::RuntimeError("ComputeGraspPosesSampled: nh expired");
  }

  track_sub_ = node_->create_subscription<yoloe_interfaces::msg::UnifiedTrackArray>(
    kTrackTopic, rclcpp::QoS(10),
    [this](const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg) {
      on_track_array(msg);
    });
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    kJointStateTopic, rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      on_joint_state(msg);
    });

  // Same pattern as BT.ROS2's RosServiceNode: the group is NOT auto-added to
  // the node's executor, so only the local executor spins it.
  ik_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  ik_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ik_executor_->add_callback_group(
    ik_cb_group_, node_->get_node_base_interface());
  ik_client_ = node_->create_client<IkSrv>(
    kIkService, rmw_qos_profile_services_default, ik_cb_group_);
  // Plan-only validation client: shares the dedicated callback group, so the
  // same local executor waits on it without touching the node's executor.
  plan_client_ = rclcpp_action::create_client<MoveGroupAction>(
    node_, kMoveAction, ik_cb_group_);

  tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, node_, true);
}

BT::PortsList ComputeGraspPosesSampled::providedPorts()
{
  return {
    // Frame the target point and every candidate pose live in: the ARM base
    // link, i.e. the same frame the tracker writes position_arm_base in.
    // 2026-11-20: was "base_link" (the DOG body) — that made every request
    // numerically wrong by the arm mount offset and stamped a frame MoveIt
    // then had to resolve through TF.
    BT::InputPort<std::string>(
      "arm_base_frame", "arm1/base_link",
      "Frame the target and the poses are computed in (arm base, TF side)"),
    BT::InputPort<std::string>(
      "track_frame", "arm1/base_link",
      "Frame of UnifiedTrack.position_arm_base (TF side, prefixed)"),
    // Frame name STAMPED on the /compute_ik request and on the plan_only
    // constraints / output poses. Must be the MoveIt model frame.
    //
    // 2026-11-20, why this is not arm1/base_link: /compute_ik transforms the
    // request pose into the model frame with MoveGroupCapability::
    // performTransform(), which goes through the TF buffer. The arm RSP runs
    // with frame_prefix="arm1/", so TF only ever has "arm1/world" and never
    // "world" — every request died with -21 FRAME_TRANSFORM_FAILURE before
    // KDL was even called. Stamping the model frame name makes
    // performTransform() return immediately (frame_id == target_frame).
    //
    // This is numerically sound because the URDF's world_to_base_link joint
    // is the identity ("0 0 0" / "0 0 0"), so arm1/base_link coordinates ARE
    // model-frame coordinates.
    BT::InputPort<std::string>(
      "model_frame", "world",
      "header.frame_id for /compute_ik + plan_only + output poses "
      "(MoveIt model frame; identity with the arm base link)"),
    BT::InputPort<double>(
      "pre_grasp_distance_m", 0.25,
      "Pre-grasp offset along the tool axis, on the approach side"),
    // Tool-axis tilt away from straight down, degrees. Below-base targets are
    // tried vertical-first; above-base targets pointing-at-target-first.
    BT::InputPort<std::string>(
      "pitch_deg_list", "0;15;30;45;60;75;90;105;120;135;150",
      "Semicolon-separated tilt grid (deg from straight down)"),
    BT::InputPort<std::string>(
      "yaw_offsets_deg", "0;15;-15;30;-30;45;-45",
      "Semicolon-separated yaw perturbations around the base->target azimuth"),
    BT::InputPort<double>(
      "min_horizontal_m", 0.05,
      "Below this horizontal base->target distance the azimuth is undefined "
      "and base +x is used"),
    BT::InputPort<int>("ik_timeout_ms", 60, "Per-call /compute_ik budget"),
    // 2026-11-20: after the IK filter, the first K feasible candidates are
    // validated with a plan_only MoveGroup goal (path feasibility). 0 disables
    // the plan check and accepts the first IK-feasible candidate.
    BT::InputPort<int>(
      "plan_check_k", 2,
      "How many IK-feasible candidates to plan-validate (0 = skip)"),
    BT::InputPort<int>(
      "plan_timeout_ms", 1000, "Per-candidate plan_only budget"),
    BT::InputPort<std::string>("ik_group", "arm", "MoveIt planning group"),
    // /compute_ik -> KDL plugin: the chain ends at the group tip, so the
    // request must name link6 and carry the link6 pose (tcp pose shifted by
    // the fixed tcp offset along the tool axis).
    BT::InputPort<std::string>(
      "ik_link", "link6", "Link the /compute_ik pose is expressed for (link6)"),
    BT::InputPort<double>(
      "tcp_offset_z_m", 0.12,
      "Fixed tcp_link offset along link6 +z (URDF tcp_offset_xyz)"),
    // plan_only uses MoveGroup, which resolves tcp_link itself.
    BT::InputPort<std::string>(
      "plan_link", "tcp_link", "Link the plan_only goal constraint uses"),
    BT::InputPort<std::string>(
      "output_pre_grasp_key", kDefaultPreGraspKey, "Blackboard key"),
    BT::InputPort<std::string>(
      "output_grasp_key", kDefaultGraspKey, "Blackboard key"),
  };
}

void ComputeGraspPosesSampled::on_track_array(
  const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = *msg;
  seen_ = true;
}

void ComputeGraspPosesSampled::on_joint_state(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  seed_js_ = *msg;
  have_js_ = true;
}

bool ComputeGraspPosesSampled::solveIk(
  const tf2::Vector3 & p, const tf2::Matrix3x3 & R, const std::string & frame)
{
  if (!ik_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[%s] %s not ready", name().c_str(), kIkService);
    return false;
  }

  // tcp pose -> link6 pose: same rotation, shifted 12 cm back along the tool
  // axis (tcp_link is link6's fixed child at +z). Solving the link6 pose gives
  // exactly the joint solution MoveIt would compute internally for a tcp_link
  // constraint, but the KDL plugin accepts it.
  const tf2::Vector3 p_link6 =
    p - R * tf2::Vector3(0.0, 0.0, tcp_offset_z_m_);

  auto req = std::make_shared<IkSrv::Request>();
  req->ik_request.group_name = ik_group_;
  req->ik_request.ik_link_name = ik_link_;
  req->ik_request.avoid_collisions = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_js_) {
      req->ik_request.robot_state.joint_state = seed_js_;
    }
  }
  req->ik_request.pose_stamped = make_pose(frame, p_link6, R);
  req->ik_request.timeout.sec = 0;
  req->ik_request.timeout.nanosec =
    static_cast<uint32_t>(std::max(1, ik_timeout_ms_) * 1000000);

  auto future = ik_client_->async_send_request(req);
  const auto ret = ik_executor_->spin_until_future_complete(
    future, std::chrono::milliseconds(std::max(1, ik_timeout_ms_)));
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    ik_timeout_seen_ = true;
    return false;
  }
  const auto resp = future.get();
  if (!resp) {
    ik_timeout_seen_ = true;
    return false;
  }
  ik_last_code_ = resp->error_code.val;
  ++ik_code_counts_[ik_last_code_];
  return resp->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool ComputeGraspPosesSampled::planOnly(
  const tf2::Vector3 & p, const tf2::Matrix3x3 & R, const std::string & frame)
{
  if (!plan_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[%s] %s not ready; accepting the IK result without a plan check",
      name().c_str(), kMoveAction);
    return true;   // fail open: the tree's MoveArmToPose is the real run
  }

  MoveGroupAction::Goal goal;
  goal.request.group_name = ik_group_;
  goal.request.num_planning_attempts = 1;
  const double budget_s = std::max(0.1, plan_timeout_ms_ / 1000.0);
  goal.request.allowed_planning_time = budget_s * 0.8;
  goal.request.max_velocity_scaling_factor = 0.3;
  goal.request.max_acceleration_scaling_factor = 0.3;

  auto & gc = goal.request.goal_constraints.emplace_back();
  // Position: 1 mm box (the executed goal uses MoveArmToPose's own box; this
  // one only answers "is there a path to roughly here").
  auto & pc = gc.position_constraints.emplace_back();
  pc.header.frame_id = frame;
  pc.link_name = plan_link_;
  pc.constraint_region.primitives.resize(1);
  pc.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  pc.constraint_region.primitives[0].dimensions = {0.001, 0.001, 0.001};
  pc.constraint_region.primitive_poses.resize(1);
  pc.constraint_region.primitive_poses[0] = make_pose(frame, p, R).pose;
  pc.weight = 1.0;

  auto & oc = gc.orientation_constraints.emplace_back();
  oc.header.frame_id = frame;
  oc.link_name = plan_link_;
  tf2::Quaternion q;
  R.getRotation(q);
  q.normalize();
  oc.orientation.x = q.x();
  oc.orientation.y = q.y();
  oc.orientation.z = q.z();
  oc.orientation.w = q.w();
  oc.absolute_x_axis_tolerance = 2e-2;
  oc.absolute_y_axis_tolerance = 2e-2;
  oc.absolute_z_axis_tolerance = 2e-2;
  oc.weight = 1.0;

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  const auto budget = std::chrono::milliseconds(plan_timeout_ms_);
  auto goal_future = plan_client_->async_send_goal(goal);
  if (ik_executor_->spin_until_future_complete(goal_future, budget) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  auto handle = goal_future.get();
  if (!handle) {
    return false;
  }
  auto result_future = plan_client_->async_get_result(handle);
  if (ik_executor_->spin_until_future_complete(result_future, budget) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  const auto wrapped = result_future.get();
  return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
         wrapped.result->error_code.val ==
         moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

BT::NodeStatus ComputeGraspPosesSampled::tick()
{
  std::string arm_base = "arm1/base_link";
  std::string track_frame = "arm1/base_link";
  std::string model_frame = "world";
  std::string pre_key = kDefaultPreGraspKey;
  std::string grasp_key = kDefaultGraspKey;
  double pre_distance = 0.25;
  double min_horizontal = 0.05;
  std::string pitch_list = "0;15;30;45;60;75;90;105;120;135;150";
  std::string yaw_list = "0;15;-15;30;-30;45;-45";
  getInput("arm_base_frame", arm_base);
  getInput("track_frame", track_frame);
  getInput("model_frame", model_frame);
  getInput("output_pre_grasp_key", pre_key);
  getInput("output_grasp_key", grasp_key);
  getInput("pre_grasp_distance_m", pre_distance);
  getInput("min_horizontal_m", min_horizontal);
  getInput("pitch_deg_list", pitch_list);
  getInput("yaw_offsets_deg", yaw_list);
  getInput("ik_timeout_ms", ik_timeout_ms_);
  getInput("plan_check_k", plan_check_k_);
  getInput("plan_timeout_ms", plan_timeout_ms_);
  getInput("ik_group", ik_group_);
  getInput("ik_link", ik_link_);
  getInput("tcp_offset_z_m", tcp_offset_z_m_);
  getInput("plan_link", plan_link_);

  ik_code_counts_.clear();
  ik_last_code_ = 0;
  ik_timeout_seen_ = false;

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

  // Target point in the arm base frame. Numerically this IS the MoveIt model
  // frame (URDF world_to_base_link is the identity); the two are only kept as
  // separate STRINGS because one is what TF calls the frame and the other is
  // what MoveIt/URDF calls it.
  tf2::Vector3 p_target(
    track->position_arm_base.x, track->position_arm_base.y,
    track->position_arm_base.z);
  if (track_frame != arm_base) {
    const tf2::Duration zero = tf2::durationFromSec(0.0);
    if (!tf_->canTransform(arm_base, track_frame, tf2::TimePointZero, zero)) {
      RCLCPP_WARN(
        logger_, "[%s] no TF %s <- %s", name().c_str(),
        arm_base.c_str(), track_frame.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto t = tf_->lookupTransform(
      arm_base, track_frame, tf2::TimePointZero, zero);
    const tf2::Quaternion q(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w);
    const tf2::Vector3 o(
      t.transform.translation.x, t.transform.translation.y,
      t.transform.translation.z);
    p_target = tf2::Transform(q, o) * p_target;
  }

  // Nominal azimuth: base -> target, projected on the horizontal plane.
  const double hx = p_target.x();
  const double hy = p_target.y();
  const double hnorm = std::hypot(hx, hy);
  double psi_nom = 0.0;
  if (hnorm >= std::max(1e-3, min_horizontal)) {
    psi_nom = std::atan2(hy, hx);
  }

  // Candidate ordering.
  std::vector<double> pitches = parse_semicolon_doubles(pitch_list);
  if (pitches.empty()) {
    pitches = {0.0};
  }
  std::vector<double> yaws = parse_semicolon_doubles(yaw_list);
  if (yaws.empty()) {
    yaws = {0.0};
  }
  if (p_target.z() < 0.0) {
    std::sort(pitches.begin(), pitches.end());   // vertical first
  } else {
    // Above the base: prefer pointing at the target.
    const double u_norm = p_target.length();
    const double tilt_best =
      (u_norm > 1e-6) ?
      std::acos(std::max(-1.0, std::min(1.0, -p_target.z() / u_norm))) * 180.0 /
      M_PI :
      0.0;
    std::sort(
      pitches.begin(), pitches.end(),
      [tilt_best](double a, double b) {
        return std::fabs(a - tilt_best) < std::fabs(b - tilt_best);
      });
  }
  std::sort(
    yaws.begin(), yaws.end(),
    [](double a, double b) {return std::fabs(a) < std::fabs(b);});

  const tf2::Vector3 up(0.0, 0.0, 1.0);
  const int k = std::max(0, plan_check_k_);
  const int want = std::max(1, k);   // stop the IK sweep once K are feasible

  struct Candidate
  {
    tf2::Vector3 p_pre;
    tf2::Vector3 p_grasp;
    tf2::Matrix3x3 R;
    double pitch_deg{0.0};
    double yaw_deg{0.0};
  };
  std::vector<Candidate> feasible;
  int tried = 0;
  const auto t_start = std::chrono::steady_clock::now();

  for (const double pitch_deg : pitches) {
    const double theta = deg2rad(pitch_deg);
    for (const double yaw_deg : yaws) {
      const double psi = psi_nom + deg2rad(yaw_deg);
      const tf2::Vector3 h(std::cos(psi), std::sin(psi), 0.0);

      // Tool axis: down at pitch 0, horizontal at 90, up at 180.
      tf2::Vector3 d = up * (-std::cos(theta)) + h * std::sin(theta);
      d.normalize();

      // Jaw axis: horizontal and perpendicular to the approach. Degenerate
      // when the tool is exactly vertical -> use the radial direction.
      tf2::Vector3 x_t = up.cross(d);
      if (x_t.length() < 1e-3) {
        x_t = h;
      } else {
        x_t.normalize();
      }
      const tf2::Vector3 y_t = d.cross(x_t);
      const tf2::Matrix3x3 R(
        x_t.x(), y_t.x(), d.x(),
        x_t.y(), y_t.y(), d.y(),
        x_t.z(), y_t.z(), d.z());

      const tf2::Vector3 p_grasp = p_target;
      const tf2::Vector3 p_pre = p_target - d * pre_distance;

      ++tried;
      if (solveIk(p_pre, R, model_frame) && solveIk(p_grasp, R, model_frame)) {
        feasible.push_back({p_pre, p_grasp, R, pitch_deg, yaw_deg});
        if (static_cast<int>(feasible.size()) >= want) {
          break;
        }
      }
    }
    if (static_cast<int>(feasible.size()) >= want) {
      break;
    }
  }

  if (feasible.empty()) {
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
    std::string codes;
    for (const auto & kv : ik_code_counts_) {
      if (!codes.empty()) {
        codes += ",";
      }
      codes += std::to_string(kv.first) + "x" + std::to_string(kv.second);
    }
    if (ik_timeout_seen_) {
      if (!codes.empty()) {
        codes += ",";
      }
      codes += "timeout";
    }
    RCLCPP_WARN(
      logger_,
      "[%s] no IK solution for id=%lu p=(%.3f, %.3f, %.3f) "
      "(%d candidates, %.1f ms, ik_codes={%s}, last=%d)",
      name().c_str(), static_cast<unsigned long>(track->track_id),
      p_target.x(), p_target.y(), p_target.z(), tried, ms,
      codes.c_str(), ik_last_code_);
    return BT::NodeStatus::FAILURE;
  }

  // Path validation: plan_only for the approach pose, in preference order.
  for (const Candidate & c : feasible) {
    if (k > 0 && !planOnly(c.p_pre, c.R, model_frame)) {
      RCLCPP_INFO(
        logger_, "[%s] pitch=%.0f yaw=%+.0f deg: IK ok but plan_only failed",
        name().c_str(), c.pitch_deg, c.yaw_deg);
      continue;
    }
    const auto pre_pose = make_pose(model_frame, c.p_pre, c.R);
    const auto grasp_pose = make_pose(model_frame, c.p_grasp, c.R);
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
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
    RCLCPP_INFO(
      logger_,
      "[%s] target id=%lu class=%s p=(%.3f, %.3f, %.3f) "
      "pitch=%.0f deg yaw=%+.0f deg pre=(%.3f, %.3f, %.3f) "
      "ik_tried=%d ik_ok=%zu plan_checked=%s elapsed=%.1f ms frame=%s",
      name().c_str(), static_cast<unsigned long>(track->track_id),
      track->class_name.c_str(),
      p_target.x(), p_target.y(), p_target.z(),
      c.pitch_deg, c.yaw_deg, c.p_pre.x(), c.p_pre.y(), c.p_pre.z(),
      tried, feasible.size(), (k > 0) ? "yes" : "no", ms, model_frame.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_WARN(
    logger_,
    "[%s] %zu IK-feasible candidate(s) all failed the plan check "
    "(id=%lu, %d IK calls, %.1f ms)",
    name().c_str(), feasible.size(),
    static_cast<unsigned long>(track->track_id), tried, ms);
  return BT::NodeStatus::FAILURE;
}

}  // namespace lgw_bt_planner
