#include "lgw_bt_planner/move_arm_to_pose_action_node.hpp"
#include "lgw_bt_planner/pose_error_gate.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace lgw_bt_planner
{

namespace
{
constexpr double kJointToleranceAbove = 0.01;
constexpr double kJointToleranceBelow = 0.01;
// /feedback/joint_states is the topic agx_arm_ctrl_single_node publishes
// (see agx_arm_ctrl launch remappings). move_group also subscribes to this
// topic via its own joint_states:=feedback/joint_states remap, so using the
// same source here keeps the IK seed consistent with what move_group sees.
constexpr char kJointStateTopic[] = "/feedback/joint_states";

// Default OMPL position-goal box half-extents (metres). Matches MoveIt
// MoveGroupInterface::setPoseTarget() default (~1e-5 m = exact match).
// Wider boxes admit multiple IK basins on a 6-DOF arm and drive joint
// drift between BT cycles, so this stays as the tight default; callers
// can override via the 'pos_box' input port (semicolon-separated x;y;z).
constexpr double kDefaultPosBox = 1e-5;

// Parse "a;b;c" into {a,b,c}. Falls back to {kDefaultPosBox,...} on
// malformed input. Used to read the 'pos_box' input port without
// forcing BT.CPP to handle std::vector<double> blackboard values.
std::vector<double> parse_semicolon_doubles(const std::string& s)
{
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ';')) {
    if (item.empty()) {continue;}
    try {
      out.push_back(std::stod(item));
    } catch (const std::exception&) {
      return {kDefaultPosBox, kDefaultPosBox, kDefaultPosBox};
    }
  }
  if (out.size() != 3) {
    return {kDefaultPosBox, kDefaultPosBox, kDefaultPosBox};
  }
  return out;
}
}  // namespace

bool MoveArmToPoseActionNode::is_debug_enabled()
{
  static const bool cached = []() {
    const char* env = std::getenv("BT_PLANNER_DEBUG");
    return env && std::string(env) == "1";
  }();
  return cached;
}

void MoveArmToPoseActionNode::ensure_joint_state_sub()
{
  if (sub_init_attempted_) {
    return;
  }
  sub_init_attempted_ = true;

  auto node = this->node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToPose"),
      "node handle unavailable; IK seed will not be applied");
    return;
  }

  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    kJointStateTopic, 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(joint_state_mutex_);
      latest_joint_state_ = *msg;
      joint_state_ready_ = true;
    });
  RCLCPP_INFO(rclcpp::get_logger("MoveArmToPose"),
    "IK seed subscription established on %s", kJointStateTopic);
}

void MoveArmToPoseActionNode::ensure_tcp_pose_sub()
{
  if (tcp_sub_init_attempted_) {
    return;
  }
  tcp_sub_init_attempted_ = true;

  auto node = this->node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToPose"),
      "node handle unavailable; /feedback/tcp_pose will not be cached");
    return;
  }

  // agx_arm_ctrl_single_node publishes at pub_rate=200Hz with TCP pose
  // stamped by SDK flange-pose timestamp. frame_id is empty (SDK
  // provides base_link-relative pose via FK), so downstream consumers
  // should treat the pose as base_link frame regardless.
  tcp_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/feedback/tcp_pose",
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(tcp_pose_mutex_);
      latest_tcp_pose_ = *msg;
      tcp_pose_ready_.store(true, std::memory_order_release);
    });
  RCLCPP_INFO(rclcpp::get_logger("MoveArmToPose"),
    "tcp_pose subscription established on /feedback/tcp_pose");
}

bool MoveArmToPoseActionNode::setGoal(moveit_msgs::action::MoveGroup::Goal& goal)
{
  ensure_joint_state_sub();

  auto target = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!target) {
    return false;
  }

  const std::string frame = target->header.frame_id.empty()
    ? "base_link"
    : target->header.frame_id;

  goal.request.group_name = "arm";
  goal.request.workspace_parameters.header.frame_id = frame;
  goal.request.workspace_parameters.header.stamp = builtin_interfaces::msg::Time();

  auto& gc = goal.request.goal_constraints.emplace_back();

  // Position constraint: tight by default (~1e-5 m = exact match,
  // MoveGroupInterface::setPoseTarget()). Wider boxes admit multiple IK
  // basins on a 6-DOF arm and drive joint drift between BT cycles, so the
  // default stays tight; callers can override via the 'pos_box' input port
  // (semicolon-separated x;y;z in metres). Diagnostic trees (e.g.
  // head_arm_independent_lookat) historically loosen to 1e-3 (1 mm) when
  // OMPL's goal sampler can't hit the 1e-5 m box inside the planning
  // timeout — see "Unable to sample any valid states for goal tree".
  auto& pc = gc.position_constraints.emplace_back();
  pc.header.frame_id = frame;
  pc.link_name = "tcp_link";
  pc.target_point_offset.x = 0.0;
  pc.target_point_offset.y = 0.0;
  pc.target_point_offset.z = 0.0;
  pc.constraint_region.primitives.resize(1);
  pc.constraint_region.primitives[0].type =
    shape_msgs::msg::SolidPrimitive::BOX;
  // Resolve pos_box from XML input port; default to 1e-5 m on missing /
  // empty / malformed input (preserves pre-existing tight behaviour for
  // every tree that doesn't opt in).
  std::vector<double> pos_box = {kDefaultPosBox, kDefaultPosBox, kDefaultPosBox};
  if (auto pb = getInput<std::string>("pos_box");
      pb && !pb->empty()) {
    pos_box = parse_semicolon_doubles(*pb);
  }
  // shape_msgs/SolidPrimitive.dimensions is a BoundedVector<double,3>, so
  // assign via initializer_list (the only overload that accepts a
  // runtime-built triple; std::vector does not implicitly convert).
  pc.constraint_region.primitives[0].dimensions = {
    pos_box[0], pos_box[1], pos_box[2]};
  pc.constraint_region.primitive_poses.resize(1);
  pc.constraint_region.primitive_poses[0] = target->pose;
  pc.weight = 1.0;

  // Orientation constraint: 1e-2 rad per axis (~0.57°). Loosened from 1e-3
  // rad on 2026-10-06 after the isolation test (lookat_pose_isolated_test
  // target_in_head_frame=(0.5,0,0)) showed ompl "Unable to sample any valid
  // states for goal tree" — KDL Newton iteration's near-exact solutions
  // (within basin boundaries) can drift ~0.005–0.02 rad from the target
  // lookat quat under wide joint space, and 1e-3 rad rejects them all.
  // Still well below basin-to-basin orientation differences (~0.1+ rad on
  // a 6-DOF arm) so a single IK basin remains forced.
  auto& oc = gc.orientation_constraints.emplace_back();
  oc.header.frame_id = frame;
  oc.link_name = "tcp_link";
  oc.orientation = target->pose.orientation;
  oc.absolute_x_axis_tolerance = 1e-2;
  oc.absolute_y_axis_tolerance = 1e-2;
  oc.absolute_z_axis_tolerance = 1e-2;
  oc.weight = 1.0;

  goal.request.planner_id = getInput<std::string>("planner_id").value_or("");
  goal.request.allowed_planning_time =
    getInput<double>("planning_time").value_or(5.0);
  (void)getInput<bool>("plan_only");

  // OMPL RRT start_state (NOT the KDL IK seed — see note below).
  //
  // NOTE: The KDL Newton iteration initial guess for inverse kinematics is
  // sourced by MoveIt internally via KinematicsBase::searchPositionIK(
  // ik_seed_state, ...), where the default fallback is
  // current_state_monitor_->getCurrentState() (see
  // moveit_kinematics/kdl_kinematics_plugin.cpp). User-supplied
  // goal.request.start_state feeds only the OMPL RRT root; it does NOT
  // feed back into KDL's seed. So omitting the assignment below does NOT
  // disable the IK seed — MoveIt continues to seed KDL with the live
  // current state. Likewise, joint_constraints constrain the planning
  // pipeline, not KDL's solver.
  //
  // To re-enable either, uncomment the corresponding block.
  {
    std::lock_guard<std::mutex> lk(joint_state_mutex_);
    if (joint_state_ready_) {
      // OMPL RRT root state (NOT KDL IK seed — see note above).
      // goal.request.start_state.joint_state = latest_joint_state_;
      // goal.request.start_state.is_diff = false;

      // MoveIt joint_constraints (NOT KDL IK seed — see note above).
      // kNoConstraintTol = M_PI means "do not actually constrain".
      /*
      auto add_joint_constraint =
        [&gc](const sensor_msgs::msg::JointState& js,
              const std::string& name, double tol_above, double tol_below) {
          auto it = std::find(js.name.begin(), js.name.end(), name);
          if (it == js.name.end()) {return;}
          const auto idx = std::distance(js.name.begin(), it);
          if (idx < 0 ||
              idx >= static_cast<std::ptrdiff_t>(js.position.size())) {return;}
          auto& jc = gc.joint_constraints.emplace_back();
          jc.joint_name = name;
          jc.position = js.position[idx];
          jc.tolerance_above = tol_above;
          jc.tolerance_below = tol_below;
          jc.weight = 1.0;
        };
      constexpr double kNoConstraintTol = M_PI;
      add_joint_constraint(latest_joint_state_, "joint6",
                           kNoConstraintTol, kNoConstraintTol);
      add_joint_constraint(latest_joint_state_, "joint4",
                           kNoConstraintTol, kNoConstraintTol);
      */
    } else {
      RCLCPP_WARN(rclcpp::get_logger("MoveArmToPose"),
        "no %s received yet; OMPL start_state will fall back to MoveIt's "
        "current_state_monitor snapshot (KDL IK seed is unaffected)",
        kJointStateTopic);
    }
  }

  if (is_debug_enabled()) {
    auto log = rclcpp::get_logger("MoveArmToPose");
    const auto& p = target->pose.position;
    const auto& q = target->pose.orientation;
    RCLCPP_INFO(log,
      "DBG setGoal target frame=%s pos=[%.4f,%.4f,%.4f] quat=[%.4f,%.4f,%.4f,%.4f]",
      frame.c_str(), p.x, p.y, p.z, q.x, q.y, q.z, q.w);
    RCLCPP_INFO(log,
      "DBG setGoal workspace frame=%s pos_box=[%.3f,%.3f,%.3f]m "
      "ori_tol=[%.3f,%.3f,%.3f]rad planner_id='%s' plan_time=%.2fs plan_only=%s",
      frame.c_str(),
      pc.constraint_region.primitives[0].dimensions[0],
      pc.constraint_region.primitives[0].dimensions[1],
      pc.constraint_region.primitives[0].dimensions[2],
      oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance,
      oc.absolute_z_axis_tolerance,
      goal.request.planner_id.c_str(),
      goal.request.allowed_planning_time,
      getInput<bool>("plan_only").value_or(false) ? "true" : "false");
    if (!gc.joint_constraints.empty()) {
      std::ostringstream jc_oss;
      for (const auto& jc : gc.joint_constraints) {
        jc_oss << " " << jc.joint_name << ":" << jc.position
               << " +/-(" << jc.tolerance_above << "/" << jc.tolerance_below << ");";
      }
      RCLCPP_INFO(log, "DBG setGoal joint_constraints:%s", jc_oss.str().c_str());
    } else {
      RCLCPP_INFO(log, "DBG setGoal joint_constraints: <none>");
    }
    if (!goal.request.start_state.joint_state.name.empty()) {
      const auto& ss = goal.request.start_state.joint_state;
      std::ostringstream ss_oss;
      for (size_t i = 0; i < ss.name.size() && i < ss.position.size(); ++i) {
        ss_oss << " " << ss.name[i] << ":" << ss.position[i] << ";";
      }
      RCLCPP_INFO(log, "DBG setGoal start_state:%s", ss_oss.str().c_str());
    } else {
      RCLCPP_INFO(log, "DBG setGoal start_state: <empty>");
    }
  }

  return true;
}

BT::NodeStatus MoveArmToPoseActionNode::onFailure(
  BT::ActionNodeErrorCode error, const std::optional<WrappedResult> & wr)
{
  // 2026-11-18: 这里以前是 bt_ros2 的默认实现，静默 return FAILURE。
  // 真机表现：抓取那一步空转 ~5 s(planning_time) 后失败，日志一个字没有，
  // 只能靠 move_group 的日志反推。现在把错误码和 MoveIt 错误码打出来。
  auto log = rclcpp::get_logger("MoveArmToPose");

  const char * code_str = "no-result";
  int32_t moveit_err = -1;
  if (wr) {
    switch (wr->code) {
      case rclcpp_action::ResultCode::SUCCEEDED: code_str = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED: code_str = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED: code_str = "CANCELED"; break;
      case rclcpp_action::ResultCode::UNKNOWN: code_str = "UNKNOWN"; break;
    }
    if (wr->result) {
      moveit_err = wr->result->error_code.val;
    }
  }

  std::string target = "<no goal port>";
  if (auto g = getInput<geometry_msgs::msg::PoseStamped>("goal"); g) {
    char buf[160];
    std::snprintf(
      buf, sizeof(buf),
      "pos=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] frame=%s",
      g->pose.position.x, g->pose.position.y, g->pose.position.z,
      g->pose.orientation.x, g->pose.orientation.y, g->pose.orientation.z,
      g->pose.orientation.w, g->header.frame_id.c_str());
    target = buf;
  }

  RCLCPP_ERROR(
    log,
    "action FAILED: error=%s result_code=%s moveit_error_code=%d target %s "
    "(moveit: 1=SUCCESS -1=FAILURE -2=PLANNING_FAILED -3=INVALID_MOTION_PLAN "
    "-4=PLAN_INVALIDATED -12=COLLISION_CHECKING_UNAVAILABLE "
    "-14=NO_IK_SOLUTION -21=START_STATE_IN_COLLISION "
    "-22=INVALID_GOAL_CONSTRAINTS -24=FRAME_TRANSFORM_LOOKUP_ERROR_NO_DATA)",
    BT::toStr(error), code_str, moveit_err, target.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveArmToPoseActionNode::onResultReceived(const WrappedResult& wr)
{
  if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
    // Diagnostic: log MoveGroup result code + MoveItErrorCodes value so
    // we can distinguish IK failure / collision / planning-timeout / no
    // pipeline path / unknown. Without this the BT just sees a bare
    // FAILURE and we have to grep move_group's own stderr.
    auto code_str = [c = wr.code]() {
      switch (c) {
        case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
        case rclcpp_action::ResultCode::ABORTED:   return "ABORTED";
        case rclcpp_action::ResultCode::CANCELED:  return "CANCELED";
        case rclcpp_action::ResultCode::UNKNOWN:   return "UNKNOWN";
      }
      return "?";
    }();
    int32_t moveit_err = -1;
    if (wr.result) {
      moveit_err = wr.result->error_code.val;
    }
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToPose"),
      "MoveGroup result != SUCCEEDED: result_code=%s "
      "moveit_error_code.val=%d (see MoveItErrorCodes.msg: "
      "1=SUCCESS, -1=FAILURE, -2=PLANNING_FAILED, -3=INVALID_MOTION_PLAN, "
      "-4=MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE, -5=CONTROL_FAILED, "
      "-6=UNABLE_TO_AQUIRE_SENSOR_DATA, -7=ROBOT_STUCK, -8=INVALID_INDEX, "
      "-9=INVALID_GROUP_NAME, -10=INVALID_LINK_NAME, -11=FRAME_TRANSFORM_FAILURE, "
      "-12=COLLISION_CHECKING_UNAVAILABLE, -13=OK但是 robot state 错误, "
      "-14=NO_IK_SOLUTION, -15=LOOKUP_ERROR, -16=INCOMPLETE_ROBOT_STATE, "
      "-17=INVALID_ROBOT_STATE, -18=INVALID_PLAN, -19=EMPTY_PLAN, -21=START_STATE_IN_COLLISION, "
      "-22=INVALID_GOAL_CONSTRAINTS, -23=INVALID_OBJECT_NAME, "
      "-24=FRAME_TRANSFORM_LOOKUP_ERROR_NO_DATA, ...)",
      code_str, moveit_err);
    return BT::NodeStatus::FAILURE;
  }

  // Sample the latest /feedback/tcp_pose (200 Hz from
  // agx_arm_ctrl_single_node) after a short wall-clock window — the arm
  // keeps damping for ~100 ms after a stop command, and IK joints may
  // also be settling on joint-limit solutions. The window lets the
  // pose stabilize before we cache it as 'actual_tcp_pose'.
  ensure_tcp_pose_sub();
  std::this_thread::sleep_for(std::chrono::milliseconds(kSettleWindowMs));

  geometry_msgs::msg::PoseStamped actual;
  bool ready = false;
  {
    std::lock_guard<std::mutex> lk(tcp_pose_mutex_);
    actual = latest_tcp_pose_;
    ready = tcp_pose_ready_.load(std::memory_order_acquire);
  }

  // Resolve the output_key port (default 'actual_tcp_pose'). xml can
  // override per-instance to write to a BT-scoped key, e.g.
  // 'actual_tcp_pose_phaseA' vs 'actual_tcp_pose_phaseB'.
  std::string output_key = "actual_tcp_pose";
  if (auto p = getInput<std::string>("output_actual_tcp_pose_key");
      p && !p->empty()) {
    output_key = *p;
  }

  // Read the setGoal target (lookat_pose) — logged next to the actual
  // pose so the user can see IK error per BT cycle.
  std::string goal_label = "none";
  geometry_msgs::msg::PoseStamped goal_pose;
  bool goal_present = false;
  if (auto g = getInput<geometry_msgs::msg::PoseStamped>("goal"); g) {
    goal_pose = *g;
    goal_label = g->header.frame_id.empty() ? "world" : g->header.frame_id;
    goal_present = true;
  }

  if (ready) {
    // SDK publishes with empty header.frame_id but pose is base_link
    // (FK root); stamp it explicitly for downstream clarity.
    actual.header.frame_id = "base_link";
    setOutput("actual_tcp_pose", actual);
    config().blackboard->set<geometry_msgs::msg::PoseStamped>(output_key, actual);

    const auto& ap = actual.pose;
    if (goal_present) {
      const auto& gp = goal_pose.pose;
      const double pos_err = std::hypot(
        gp.position.x - ap.position.x,
        gp.position.y - ap.position.y,
        gp.position.z - ap.position.z);
      RCLCPP_INFO(rclcpp::get_logger("MoveArmToPose"),
        "MoveGroup SUCCEEDED -> sampled actual_tcp_pose after %dms settle "
        "(blackboard='%s'). lookat_pose frame=%s pos=[%.4f, %.4f, %.4f] "
        "quat=[%.4f, %.4f, %.4f, %.4f]; actual_tcp_pose frame=%s "
        "pos=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f]; "
        "||pos_err||=%.4f m",
        kSettleWindowMs, output_key.c_str(),
        goal_label.c_str(),
        gp.position.x, gp.position.y, gp.position.z,
        gp.orientation.x, gp.orientation.y, gp.orientation.z, gp.orientation.w,
        actual.header.frame_id.c_str(),
        ap.position.x, ap.position.y, ap.position.z,
        ap.orientation.x, ap.orientation.y, ap.orientation.z, ap.orientation.w,
        pos_err);

      // 到位判据: MoveGroup 在 FakeSystem 上必然 SUCCEEDED, 真机是否
      // 跟上只有 tcp 反馈说了算。位置与姿态任一超差直接 FAILURE, 不再让
      // 树带着误差往下走一整轮。
      const double qa[4] = {
        ap.orientation.x, ap.orientation.y, ap.orientation.z, ap.orientation.w};
      const double qg[4] = {
        gp.orientation.x, gp.orientation.y, gp.orientation.z, gp.orientation.w};
      const double rot_err = rotation_angle_error_rad(qa, qg);

      double max_pos_err_m = 0.005;
      double max_rot_err_rad = 0.05;
      getInput("max_pos_err_m", max_pos_err_m);
      getInput("max_rot_err_rad", max_rot_err_rad);
      if (!pose_within_tolerance(
            pos_err, rot_err, max_pos_err_m, max_rot_err_rad))
      {
        RCLCPP_ERROR(rclcpp::get_logger("MoveArmToPose"),
          "pose not reached: ||pos_err||=%.4f m (max %.4f), rot_err=%.4f rad "
          "(max %.4f) (target pos=[%.4f, %.4f, %.4f], actual pos=[%.4f, %.4f, %.4f]). "
          "MoveGroup reported SUCCEEDED, so the trajectory was accepted but "
          "the real arm did not follow — check agx_arm_ctrl "
          "(arm_status.motion_status, joint feedback) before retrying.",
          pos_err, max_pos_err_m, rot_err, max_rot_err_rad,
          gp.position.x, gp.position.y, gp.position.z,
          ap.position.x, ap.position.y, ap.position.z);
        return BT::NodeStatus::FAILURE;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MoveArmToPose"),
        "MoveGroup SUCCEEDED -> sampled actual_tcp_pose after %dms settle "
        "(blackboard='%s'). actual_tcp_pose frame=%s pos=[%.4f, %.4f, %.4f] "
        "quat=[%.4f, %.4f, %.4f, %.4f] (no goal port to log)",
        kSettleWindowMs, output_key.c_str(),
        actual.header.frame_id.c_str(),
        ap.position.x, ap.position.y, ap.position.z,
        ap.orientation.x, ap.orientation.y, ap.orientation.z, ap.orientation.w);
    }
  } else {
    // No /feedback/tcp_pose has arrived yet (e.g. agx_arm_ctrl not
    // running). Log loudly so the user sees the gap, but still
    // SUCCESS — the action itself completed, downstream nodes will
    // see an unset actual_tcp_pose and can decide how to react.
    RCLCPP_WARN(rclcpp::get_logger("MoveArmToPose"),
      "MoveGroup SUCCEEDED but no /feedback/tcp_pose received within "
      "%dms settle; blackboard='%s' not written. check that "
      "agx_arm_ctrl_single_node is running.",
      kSettleWindowMs, output_key.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveArmToPoseActionNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "workspace target pose"),
    BT::InputPort<std::string>("planner_id", "", "planning pipeline / planner id"),
    BT::InputPort<double>("planning_time", 5.0, "max planning time (s)"),
    BT::InputPort<std::string>("pos_box", "1e-5;1e-5;1e-5",
      "OMPL position-goal tolerance box half-extents (metres); "
      "semicolon-separated x;y;z. Default 1e-5 m matches MoveIt "
      "setPoseTarget(). Loosen (e.g. 0.001;0.001;0.001 = 1 mm) for "
      "diagnostic trees where OMPL reports \"Unable to sample any "
      "valid states for goal tree\" with the tight default."),
    BT::InputPort<bool>("plan_only", false, "plan only, do not execute"),
    // 位置到位判据(2026-11-17 加)。MoveGroup 在 FakeSystem 上永远报
    // SUCCEEDED, 真机没到位也照样成功 —— 2026-11-17 一次 UnloadHandle
    // 全程 ||pos_err|| = 0.29~0.37 m 仍被判 SUCCESS。现在采样完
    // /feedback/tcp_pose 后会比对, 超过该值直接 FAILURE。
    // <=0 关闭判据(留给没有 agx_arm_ctrl、拿不到 tcp 反馈的场景)。
    BT::InputPort<double>("max_pos_err_m", 0.005,
      "Fail the node when ||actual_tcp_pose - goal|| exceeds this many "
      "metres. Default 5 mm. <=0 disables the check."),
    // 姿态到位判据(2026-11-17 加): 只看位置会漏掉"到位但转角偏了"的情况。
    // 角度取 2*acos(|q1·q2|), 也就是两个四元数的测地夹角。默认 0.05 rad
    // (2.9°) —— 实测正常到位时约 1.0~1.3°, 留了约 2 倍余量。
    // <=0 关闭姿态判据(退化成只判位置)。
    BT::InputPort<double>("max_rot_err_rad", 0.05,
      "Fail the node when the angle between actual_tcp_pose and goal "
      "orientation exceeds this many radians. Default 0.05 (2.9 deg). "
      "<=0 disables the orientation check."),
    BT::InputPort<std::string>("output_actual_tcp_pose_key",
      "actual_tcp_pose",
      "Blackboard key for the actual tcp pose sampled from /feedback/tcp_pose "
      "after a 300 ms settle window. Override to namespace per BT step."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "actual_tcp_pose",
      "Actual tcp pose sampled from /feedback/tcp_pose after settle; "
      "also written to blackboard under output_actual_tcp_pose_key."),
  });
}

}  // namespace lgw_bt_planner
