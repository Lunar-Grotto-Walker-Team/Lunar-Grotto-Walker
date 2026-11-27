#include "lgw_bt_planner/move_arm_to_named_action_node.hpp"
#include "lgw_bt_planner/pose_error_gate.hpp"

#include <moveit_msgs/msg/joint_constraint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace lgw_bt_planner
{

namespace
{
constexpr double kJointToleranceAbove = 0.01;
constexpr double kJointToleranceBelow = 0.01;
}  // namespace

bool MoveArmToNamedActionNode::setGoal(moveit_msgs::action::MoveGroup::Goal& goal)
{
  // SRDF target table is injected via the BT global blackboard by
  // bt_runner_node (BT.CPP t16 idiom). The XML port
  // `named_targets="@{named_targets}"` registers a port remap on the tree's
  // root blackboard; getInput<T>() follows the remap up to the global
  // blackboard and returns BT::Expected<T>. Null shared_ptr means either
  // the key was never set, or it was set to nullptr.
  auto named_targets_ptr =
    getInput<std::shared_ptr<TargetTable>>("named_targets");
  if (!named_targets_ptr || !named_targets_ptr.value()) {
    throw std::runtime_error(
      "MoveArmToNamed: blackboard port 'named_targets' not set; bt_runner must "
      "inject a non-null shared_ptr<TargetTable> before ticking");
  }
  const auto& named_targets = *named_targets_ptr.value();

  auto target_name = getInput<std::string>("named_target");
  if (!target_name) {
    return false;
  }

  auto it = named_targets.find(target_name.value());
  if (it == named_targets.end()) {
    return false;  // unknown target → BT FAILURE
  }

  const auto& nt = it->second;
  // Route to the SRDF-declared planning group ("arm" / "gripper" / "hand")
  // so OMPL plans against joints that exist in that group.
  goal.request.group_name = nt.group;
  auto& gc = goal.request.goal_constraints.emplace_back();

  for (const auto& [joint_name, value] : nt.joints) {
    auto& jc = gc.joint_constraints.emplace_back();
    jc.joint_name = joint_name;
    jc.position = value;
    jc.tolerance_above = kJointToleranceAbove;
    jc.tolerance_below = kJointToleranceBelow;
    jc.weight = 1.0;
  }

  goal.request.planner_id = getInput<std::string>("planner_id").value_or("");
  goal.request.allowed_planning_time =
    getInput<double>("planning_time").value_or(5.0);
  (void)getInput<bool>("plan_only");

  // Remember the commanded joint values for the post-execution check.
  target_joints_ = nt.joints;

  return true;
}

void MoveArmToNamedActionNode::ensure_joint_state_sub()
{
  if (sub_init_attempted_) {
    return;
  }
  sub_init_attempted_ = true;
  auto node = this->node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToNamed"),
      "node handle unavailable; /feedback/joint_states will not be cached");
    return;
  }
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/feedback/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(joint_state_mutex_);
      latest_joint_state_ = *msg;
      joint_state_ready_.store(true, std::memory_order_release);
    });
}

BT::NodeStatus MoveArmToNamedActionNode::onFailure(
  BT::ActionNodeErrorCode error, const std::optional<WrappedResult> & wr)
{
  auto log = rclcpp::get_logger("MoveArmToNamed");

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

  const std::string named =
    getInput<std::string>("named_target").value_or(std::string("<unset>"));

  RCLCPP_ERROR(
    log,
    "action FAILED: error=%s result_code=%s moveit_error_code=%d target '%s' "
    "(moveit: 1=SUCCESS -1=FAILURE -2=PLANNING_FAILED -3=INVALID_MOTION_PLAN "
    "-14=NO_IK_SOLUTION -21=START_STATE_IN_COLLISION)",
    BT::toStr(error), code_str, moveit_err, named.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveArmToNamedActionNode::onResultReceived(const WrappedResult& wr)
{
  if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return BT::NodeStatus::FAILURE;
  }

  // 到位判据(2026-11-17 加). 目标是关节值(SRDF group_state), 所以这里逐关节
  // 比对 /feedback/joint_states —— 对关节目标来说这比位姿更直接也更严格
  // (位姿还得先调 FK 服务求一次正解, 平白多一个失败点). 2026-11-17 的 park
  // 事故正是 joint4 差了 0.41 rad 而 MoveGroup 仍报 SUCCEEDED。
  double max_joint_err_rad = 0.05;
  getInput("max_joint_err_rad", max_joint_err_rad);
  if (max_joint_err_rad <= 0.0 || target_joints_.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  ensure_joint_state_sub();
  std::this_thread::sleep_for(std::chrono::milliseconds(kSettleWindowMs));

  sensor_msgs::msg::JointState actual;
  bool ready = false;
  {
    std::lock_guard<std::mutex> lk(joint_state_mutex_);
    actual = latest_joint_state_;
    ready = joint_state_ready_.load(std::memory_order_acquire);
  }
  if (!ready) {
    // 拿不到反馈就没法证明到位; 保持与 MoveArmToPose 一致的做法: 大声告警,
    // 但不判失败(没有 agx_arm_ctrl 的场景仍然可用, 把 max_joint_err_rad
    // 设为 0 可彻底关掉)。
    RCLCPP_WARN(rclcpp::get_logger("MoveArmToNamed"),
      "MoveGroup SUCCEEDED but no /feedback/joint_states within %d ms settle; "
      "arrival not verified (is agx_arm_ctrl running?)", kSettleWindowMs);
    return BT::NodeStatus::SUCCESS;
  }

  std::map<std::string, double> actual_by_name;
  for (std::size_t i = 0; i < actual.name.size() && i < actual.position.size(); ++i) {
    actual_by_name[actual.name[i]] = actual.position[i];
  }

  // 只校验反馈里真实存在的关节。SRDF 目标表里可能带进别的臂型分支的关节
  // (named_target_loader 解析的是 agx_arm.srdf.xacro 模板, 里面 home 在
  // 每个臂型分支各有一份, 只有 nero 分支带 joint7, 而驱动反馈里没有这个
  // 名字)。漏掉的关节跳过并告警; 一个都校验不了才算失败。
  double worst = 0.0;
  std::string worst_joint;
  std::size_t verified = 0;
  for (const auto& [joint_name, target] : target_joints_) {
    auto it = actual_by_name.find(joint_name);
    if (it == actual_by_name.end()) {
      RCLCPP_WARN(rclcpp::get_logger("MoveArmToNamed"),
        "skipping joint '%s' from named target: not present in "
        "/feedback/joint_states", joint_name.c_str());
      continue;
    }
    ++verified;
    const double err = std::fabs(it->second - target);
    if (err > worst) {
      worst = err;
      worst_joint = joint_name;
    }
  }
  if (verified == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToNamed"),
      "cannot verify named target: none of its joints appear in "
      "/feedback/joint_states");
    return BT::NodeStatus::FAILURE;
  }

  if (!error_within_tolerance(worst, max_joint_err_rad)) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveArmToNamed"),
      "named target not reached: worst joint '%s' off by %.4f rad > "
      "max_joint_err_rad=%.4f. MoveGroup reported SUCCEEDED, so the "
      "trajectory was accepted but the real arm did not follow — check "
      "agx_arm_ctrl (arm_status.motion_status, joint feedback) before retrying.",
      worst_joint.c_str(), worst, max_joint_err_rad);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("MoveArmToNamed"),
    "named target reached: worst joint '%s' off by %.4f rad (max %.4f)",
    worst_joint.c_str(), worst, max_joint_err_rad);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveArmToNamedActionNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("named_target", "SRDF group_state name"),
    BT::InputPort<std::string>("planner_id", "", "planning pipeline / planner id"),
    BT::InputPort<double>("planning_time", 5.0, "max planning time (s)"),
    BT::InputPort<bool>("plan_only", false, "plan only, do not execute"),
    // 到位判据(2026-11-17 加): 逐关节比对 /feedback/joint_states。默认
    // 0.05 rad(2.9°) —— 正常到位约 0.01 rad 量级; 2026-11-17 park 事故是
    // joint4 差 0.41 rad。<=0 关闭判据。
    BT::InputPort<double>("max_joint_err_rad", 0.05,
      "Fail the node when any commanded joint is further than this from "
      "/feedback/joint_states after a 300 ms settle. Default 0.05 rad. "
      "<=0 disables the check."),
    // SRDF named-targets table injected from the global blackboard. The XML
    // wires this port as `named_targets="@{named_targets}"` so getInput()
    // resolves via parent-blackboard traversal. Inner type is TargetTable
    // (not const) because BT.CPP's InputPort type erasure strips inner
    // const, and set<T>() on the global BB uses typeid(T) for matching.
    BT::InputPort<std::shared_ptr<TargetTable>>(
      "named_targets", "SRDF named-targets table (from global blackboard)"),
  });
}

}  // namespace lgw_bt_planner
