// transit_node.cpp — ROS 接线层实现。
//
// 设计要点（见 docs/superpowers/specs/2026-11-17-cave-base-entrance-transit-design.md）：
//   * 单线程执行器 + 20 Hz 定时器驱动，任何回调都不阻塞；
//   * TF 一律零超时查询，拿不到就保持上一位姿，绝不用 {0,0,0} 兜底；
//   * 运动只通过 nav2 的 FollowPath / Spin，从不直接发 cmd_vel。

#include "go2_transit/transit_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace go2_transit
{
namespace
{

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

TransitNode::TransitNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("transit_node", options)
{
  planning_frame_ = declare_parameter<std::string>("planning_frame", planning_frame_);
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  default_xy_tolerance_ =
    declare_parameter<double>("default_xy_tolerance", default_xy_tolerance_);
  default_yaw_tolerance_ =
    declare_parameter<double>("default_yaw_tolerance", default_yaw_tolerance_);
  path_spacing_m_ = declare_parameter<double>("path_spacing_m", path_spacing_m_);
  pre_align_threshold_rad_ =
    declare_parameter<double>("pre_align_threshold_rad", pre_align_threshold_rad_);
  pre_align_fallback_ = declare_parameter<bool>("pre_align_fallback", pre_align_fallback_);
  follow_path_timeout_s_ =
    declare_parameter<double>("follow_path_timeout_s", follow_path_timeout_s_);
  spin_timeout_s_ = declare_parameter<double>("spin_timeout_s", spin_timeout_s_);
  total_timeout_s_ = declare_parameter<double>("total_timeout_s", total_timeout_s_);
  ready_wait_s_ = declare_parameter<double>("ready_wait_s", ready_wait_s_);
  max_retries_ = declare_parameter<int>("max_retries", max_retries_);
  controller_id_ = declare_parameter<std::string>("controller_id", controller_id_);
  goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", goal_checker_id_);
  dry_run_ = declare_parameter<bool>("dry_run", dry_run_);
  status_rate_hz_ = declare_parameter<double>("status_rate_hz", status_rate_hz_);
  action_namespace_ =
    declare_parameter<std::string>("action_namespace", action_namespace_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
  spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");

  status_pub_ = create_publisher<std_msgs::msg::String>(
    "/" + action_namespace_ + "/status", 10);
  stop_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/" + action_namespace_ + "/stop", 10,
    std::bind(&TransitNode::on_stop, this, std::placeholders::_1));

  transit_server_ = rclcpp_action::create_server<TransitToPose>(
    this,
    "/" + action_namespace_ + "/transit_to_pose",
    std::bind(
      &TransitNode::handle_transit_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&TransitNode::handle_transit_cancel, this, std::placeholders::_1),
    std::bind(&TransitNode::handle_transit_accepted, this, std::placeholders::_1));

  rotate_server_ = rclcpp_action::create_server<RotateToYaw>(
    this,
    "/" + action_namespace_ + "/rotate_to_yaw",
    std::bind(
      &TransitNode::handle_rotate_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&TransitNode::handle_rotate_cancel, this, std::placeholders::_1),
    std::bind(&TransitNode::handle_rotate_accepted, this, std::placeholders::_1));

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&TransitNode::tick, this));

  const double rate = status_rate_hz_ > 0.0 ? status_rate_hz_ : 2.0;
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
    std::bind(&TransitNode::publish_status, this));

  RCLCPP_INFO(
    get_logger(),
    "go2_transit 就绪：planning_frame=%s base_frame=%s dry_run=%s",
    planning_frame_.c_str(), base_frame_.c_str(), dry_run_ ? "true" : "false");
}

// ------------------------------------------------------------ 参数与工具

TransitParams TransitNode::machine_params() const
{
  TransitParams p;
  p.default_xy_tolerance = default_xy_tolerance_;
  p.default_yaw_tolerance = default_yaw_tolerance_;
  p.path_spacing_m = path_spacing_m_;
  p.pre_align_threshold_rad = pre_align_threshold_rad_;
  p.pre_align_fallback = pre_align_fallback_;
  p.max_retries = max_retries_;
  return p;
}

double TransitNode::elapsed_since_goal_start() const
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - goal_started_at_).count();
}

bool TransitNode::nav2_ready() const
{
  return follow_path_client_->action_server_is_ready() &&
         spin_client_->action_server_is_ready();
}

std::optional<Pose2D> TransitNode::lookup_pose()
{
  if (!tf_buffer_->canTransform(
      planning_frame_, base_frame_, tf2::TimePointZero))
  {
    return std::nullopt;
  }
  try {
    const auto tf =
      tf_buffer_->lookupTransform(planning_frame_, base_frame_, tf2::TimePointZero);
    Pose2D pose;
    pose.x = tf.transform.translation.x;
    pose.y = tf.transform.translation.y;
    pose.yaw = tf2::getYaw(tf.transform.rotation);
    return pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "TF 查询失败: %s", ex.what());
    return std::nullopt;
  }
}

Pose2D TransitNode::current_target_pose() const
{
  return target_;
}

bool TransitNode::resolve_transit_goal()
{
  geometry_msgs::msg::PoseStamped in = raw_target_;
  if (in.header.frame_id.empty()) {
    in.header.frame_id = planning_frame_;
  }

  geometry_msgs::msg::PoseStamped out = in;
  if (in.header.frame_id != planning_frame_) {
    if (!tf_buffer_->canTransform(
        planning_frame_, in.header.frame_id, tf2::TimePointZero))
    {
      return false;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(
        planning_frame_, in.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(in, out, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "目标位姿换算失败: %s", ex.what());
      return false;
    }
  }

  if (!std::isfinite(out.pose.position.x) || !std::isfinite(out.pose.position.y)) {
    return false;
  }

  target_.x = out.pose.position.x;
  target_.y = out.pose.position.y;
  target_.yaw = tf2::getYaw(out.pose.orientation);
  return true;
}

// ------------------------------------------------------------ 动作服务器

rclcpp_action::GoalResponse TransitNode::handle_transit_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const TransitToPose::Goal> goal)
{
  if (!std::isfinite(goal->target.pose.position.x) ||
    !std::isfinite(goal->target.pose.position.y))
  {
    RCLCPP_WARN(get_logger(), "拒绝目标：坐标不是有限值");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TransitNode::handle_transit_cancel(
  const std::shared_ptr<TransitGoalHandle>)
{
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TransitNode::handle_transit_accepted(
  const std::shared_ptr<TransitGoalHandle> goal_handle)
{
  start_transit(goal_handle);
}

rclcpp_action::GoalResponse TransitNode::handle_rotate_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const RotateToYaw::Goal> goal)
{
  if (!std::isfinite(goal->target_yaw)) {
    RCLCPP_WARN(get_logger(), "拒绝目标：target_yaw 不是有限值");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TransitNode::handle_rotate_cancel(
  const std::shared_ptr<RotateGoalHandle>)
{
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TransitNode::handle_rotate_accepted(
  const std::shared_ptr<RotateGoalHandle> goal_handle)
{
  start_rotate(goal_handle);
}

void TransitNode::preempt_active()
{
  if (active_ == ActiveKind::NONE) {
    return;
  }
  RCLCPP_WARN(get_logger(), "新目标到达，抢占当前任务");
  cancel_in_flight();
  if (machine_) {
    machine_->cancel();
  }
  finish_active_goal();
}

void TransitNode::start_transit(
  const std::shared_ptr<TransitGoalHandle> & goal_handle)
{
  preempt_active();

  active_ = ActiveKind::TRANSIT;
  rotate_goal_ = false;
  transit_handle_ = goal_handle;
  rotate_handle_.reset();

  machine_ = std::make_unique<TransitStateMachine>(machine_params(), TaskMode::TRANSIT);

  const auto goal = goal_handle->get_goal();
  raw_target_ = goal->target;
  goal_xy_tolerance_ = goal->xy_tolerance;
  goal_align_yaw_ = goal->align_yaw;
  goal_yaw_tolerance_ = goal->yaw_tolerance;
  goal_resolved_ = false;

  goal_started_at_ = std::chrono::steady_clock::now();
  last_dispatched_id_ = 0;
  // dispatch_epoch_ 绝不在这里归零：它必须跨 goal 单调递增，否则被抢占任务
  // 的旧 epoch 会和下一个任务第一次派发的 epoch 相等，旧 result 就会回流。
  ready_reported_ = false;
  cancel_requested_ = false;
  sim_pending_cmd_.reset();
  fp_in_flight_ = false;

  RCLCPP_INFO(
    get_logger(), "TransitToPose: target=(%.2f, %.2f) frame=%s align_yaw=%d",
    raw_target_.pose.position.x, raw_target_.pose.position.y,
    raw_target_.header.frame_id.c_str(), static_cast<int>(goal_align_yaw_));
}

void TransitNode::start_rotate(
  const std::shared_ptr<RotateGoalHandle> & goal_handle)
{
  preempt_active();

  active_ = ActiveKind::ROTATE;
  rotate_goal_ = true;
  rotate_handle_ = goal_handle;
  transit_handle_.reset();

  machine_ = std::make_unique<TransitStateMachine>(machine_params(), TaskMode::ROTATE);
  const auto goal = goal_handle->get_goal();
  machine_->set_rotate_goal(goal->target_yaw, goal->yaw_tolerance);
  // 只为了 result / feedback 里能算角度误差；ROTATE 模式不使用 x/y。
  target_ = Pose2D{0.0, 0.0, goal->target_yaw};

  goal_started_at_ = std::chrono::steady_clock::now();
  last_dispatched_id_ = 0;
  // 同 start_transit：epoch 不归零，保证跨 goal 永不复用。
  ready_reported_ = false;
  cancel_requested_ = false;
  sim_pending_cmd_.reset();
  fp_in_flight_ = false;
  goal_resolved_ = true;

  RCLCPP_INFO(get_logger(), "RotateToYaw: target_yaw=%.3f", goal->target_yaw);
}

// ------------------------------------------------------------ 主循环

void TransitNode::tick()
{
  if (active_ == ActiveKind::NONE || !machine_) {
    return;
  }

  if (elapsed_since_goal_start() > total_timeout_s_) {
    RCLCPP_WARN(get_logger(), "任务总超时 %.1f s", total_timeout_s_);
    machine_->report_timeout();
    finish_active_goal();
    return;
  }

  if (cancel_requested_) {
    cancel_requested_ = false;
    cancel_in_flight();
    machine_->cancel();
    finish_active_goal();
    return;
  }

  check_follow_path_timeout();

  const bool tf_ok =
    tf_buffer_->canTransform(planning_frame_, base_frame_, tf2::TimePointZero);
  // dry_run 的目的是"不依赖 nav2/TF 也能把状态机走一遍"，所以它直接算就绪；
  // 拿不到位姿时状态机用默认位姿 {0,0,0}，不影响路径生成是否正确的判断。
  const bool ready = dry_run_ || (tf_ok && nav2_ready());
  machine_->set_ready(ready);

  if (!ready) {
    if (!ready_reported_ && elapsed_since_goal_start() > ready_wait_s_) {
      ready_reported_ = true;
      // TF 不可用优先报 tf_unavailable：两者都没有时提示先查数据源。
      machine_->fail(tf_ok ? FailReason::NAV2_UNAVAILABLE : FailReason::TF_UNAVAILABLE);
      finish_active_goal();
      return;
    }
    machine_->next_command();
    return;
  }

  // dry_run 下只在第一次拿到真实位姿时播种，之后位姿由模拟推进。
  // 否则每个 tick 都会把模拟出来的位置覆盖回真实值，SPIN 阶段之后 VERIFY
  // 永远判定"没到位"，dry_run 会一路报 unreachable。
  if (!dry_run_ || !dry_run_pose_seeded_) {
    if (auto pose = lookup_pose()) {
      last_pose_ = *pose;
      have_pose_ = true;
      dry_run_pose_seeded_ = dry_run_;
    }
  }
  // 拿不到 TF 时只保持上一位姿，绝不用 {0,0,0} 兜底。
  if (have_pose_) {
    machine_->set_pose(last_pose_);
  }

  if (dry_run_ && sim_pending_cmd_) {
    const MachineCommand simulated = *sim_pending_cmd_;
    sim_pending_cmd_.reset();
    if (simulated.kind == MachineCommand::Kind::SPIN) {
      Pose2D pose = have_pose_ ? last_pose_ : Pose2D{};
      pose.yaw = simulated.spin_target_yaw;
      last_pose_ = pose;
      have_pose_ = true;
      machine_->set_pose(pose);
    } else if (
      simulated.kind == MachineCommand::Kind::FOLLOW_PATH && !simulated.path.empty())
    {
      last_pose_ = simulated.path.back();
      have_pose_ = true;
      machine_->set_pose(last_pose_);
    }
    machine_->report_action_done(true);
  }

  if (!goal_resolved_) {
    if (!resolve_transit_goal()) {
      RCLCPP_WARN(get_logger(), "目标位姿无法换算到 %s", planning_frame_.c_str());
      machine_->fail(FailReason::INVALID_GOAL);
      finish_active_goal();
      return;
    }
    machine_->set_transit_goal(
      target_, goal_xy_tolerance_, goal_align_yaw_, goal_yaw_tolerance_);
    goal_resolved_ = true;
  }

  const MachineCommand cmd = machine_->next_command();
  if (cmd.id == last_dispatched_id_) {
    return;
  }
  last_dispatched_id_ = cmd.id;

  switch (cmd.kind) {
    case MachineCommand::Kind::WAIT_READY:
      break;
    case MachineCommand::Kind::SPIN:
    case MachineCommand::Kind::FOLLOW_PATH:
      dispatch(cmd);
      break;
    case MachineCommand::Kind::SUCCEED:
    case MachineCommand::Kind::FAIL:
      finish_active_goal();
      break;
  }
}

void TransitNode::dispatch(const MachineCommand & cmd)
{
  if (dry_run_) {
    RCLCPP_INFO(
      get_logger(), "[dry_run] %s id=%lu target_yaw=%.3f path_points=%zu",
      cmd.kind == MachineCommand::Kind::SPIN ? "SPIN" : "FOLLOW_PATH",
      static_cast<unsigned long>(cmd.id), cmd.spin_target_yaw, cmd.path.size());
    sim_pending_cmd_ = cmd;
    return;
  }

  if (cmd.kind == MachineCommand::Kind::SPIN) {
    dispatch_spin(cmd);
  } else {
    dispatch_follow_path(cmd);
  }
}

void TransitNode::dispatch_follow_path(const MachineCommand & cmd)
{
  if (!follow_path_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "follow_path 动作服务器不在线");
    machine_->fail(FailReason::NAV2_UNAVAILABLE);
    finish_active_goal();
    return;
  }

  const uint64_t epoch = ++dispatch_epoch_;

  nav_msgs::msg::Path path;
  path.header.frame_id = planning_frame_;
  path.header.stamp = now();
  path.poses.reserve(cmd.path.size());
  for (const auto & p : cmd.path) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose.position.x = p.x;
    pose_stamped.pose.position.y = p.y;
    pose_stamped.pose.orientation = quat_from_yaw(p.yaw);
    path.poses.push_back(pose_stamped);
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id_;
  goal.goal_checker_id = goal_checker_id_;

  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.goal_response_callback =
    [this, epoch](const FollowPathGoalHandle::SharedPtr & handle) {
      if (epoch != dispatch_epoch_) {
        // 派发后到拿到句柄之前发生了抢占/取消：句柄来晚了，但对端已经在跑。
        // 必须立刻取消它，否则会留下一个没人能取消的孤儿动作，和新动作
        // 一起驱动底盘。
        if (handle) {
          follow_path_client_->async_cancel_goal(handle);
        }
        return;
      }
      fp_handle_ = handle;
    };
  options.result_callback =
    [this, epoch](const FollowPathGoalHandle::WrappedResult & wrapped) {
      if (epoch != dispatch_epoch_ || !machine_) {
        return;
      }
      fp_handle_.reset();
      fp_in_flight_ = false;
      machine_->report_action_done(
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  RCLCPP_INFO(
    get_logger(), "FollowPath: %zu 个路径点 → (%.2f, %.2f)",
    path.poses.size(), cmd.path.back().x, cmd.path.back().y);

  fp_in_flight_ = true;
  fp_dispatched_at_ = std::chrono::steady_clock::now();
  follow_path_client_->async_send_goal(goal, options);
}

void TransitNode::dispatch_spin(const MachineCommand & cmd)
{
  if (!spin_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "spin 动作服务器不在线");
    machine_->fail(FailReason::NAV2_UNAVAILABLE);
    finish_active_goal();
    return;
  }

  if (!have_pose_) {
    RCLCPP_ERROR(get_logger(), "没有位姿，无法把绝对目标朝向换算成相对转角");
    machine_->fail(FailReason::TF_UNAVAILABLE);
    finish_active_goal();
    return;
  }

  const uint64_t epoch = ++dispatch_epoch_;

  // nav2 的 Spin 把 target_yaw 当**相对转角**，不是绝对朝向：
  //   spin.cpp: cmd_yaw_ = command->target_yaw;
  //             remaining_yaw = |cmd_yaw_| - |relative_yaw_|
  // 直接送绝对角会多转一大截（真机实测：目标 +20° 被理解成再转 148°）。
  const double relative_delta =
    normalize_angle(cmd.spin_target_yaw - last_pose_.yaw);

  Spin::Goal goal;
  goal.target_yaw = static_cast<float>(relative_delta);
  goal.time_allowance =
    rclcpp::Duration::from_seconds(spin_timeout_s_);

  auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
  options.goal_response_callback =
    [this, epoch](const SpinGoalHandle::SharedPtr & handle) {
      if (epoch != dispatch_epoch_) {
        if (handle) {
          spin_client_->async_cancel_goal(handle);
        }
        return;
      }
      spin_handle_ = handle;
    };
  options.result_callback =
    [this, epoch](const SpinGoalHandle::WrappedResult & wrapped) {
      if (epoch != dispatch_epoch_ || !machine_) {
        return;
      }
      spin_handle_.reset();
      machine_->report_action_done(
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  RCLCPP_INFO(
    get_logger(), "Spin: 绝对目标 %.3f rad（当前 %.3f）→ 相对转角 %+.3f rad",
    cmd.spin_target_yaw, last_pose_.yaw, relative_delta);
  spin_client_->async_send_goal(goal, options);
}

// DWB 卡住时 FollowPath 会一直挂着；没有这个看门狗就只能等总超时，
// 中间有界重试的机会根本不存在。
void TransitNode::check_follow_path_timeout()
{
  if (!fp_in_flight_) {
    return;
  }
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - fp_dispatched_at_)
    .count();
  if (elapsed <= follow_path_timeout_s_) {
    return;
  }
  RCLCPP_WARN(
    get_logger(), "FollowPath 超时 %.1f s，按失败处理", follow_path_timeout_s_);
  fp_in_flight_ = false;
  cancel_in_flight();
  machine_->report_action_done(false);
}

void TransitNode::cancel_in_flight()
{
  // 让在飞的 result 回调失效，避免把取消结果当成任务失败喂回状态机。
  ++dispatch_epoch_;
  fp_in_flight_ = false;
  if (fp_handle_) {
    follow_path_client_->async_cancel_goal(fp_handle_);
    fp_handle_.reset();
  }
  if (spin_handle_) {
    spin_client_->async_cancel_goal(spin_handle_);
    spin_handle_.reset();
  }
  sim_pending_cmd_.reset();
}

void TransitNode::fill_feedback()
{
  if (!machine_) {
    return;
  }
  const bool ok = machine_->result_reason() == FailReason::OK;
  (void)ok;

  if (active_ == ActiveKind::TRANSIT && transit_handle_) {
    if (transit_handle_->is_active()) {
      auto feedback = std::make_shared<TransitToPose::Feedback>();
      feedback->phase = to_string(machine_->phase());
      feedback->distance_remaining = static_cast<float>(
        have_pose_ ? distance_2d(last_pose_, target_) : 0.0);
      const double bearing = have_pose_
        ? std::atan2(target_.y - last_pose_.y, target_.x - last_pose_.x)
        : 0.0;
      feedback->heading_error = static_cast<float>(
        have_pose_ ? shortest_angular_distance(last_pose_.yaw, bearing) : 0.0);
      transit_handle_->publish_feedback(feedback);
    }
  } else if (active_ == ActiveKind::ROTATE && rotate_handle_) {
    if (rotate_handle_->is_active()) {
      auto feedback = std::make_shared<RotateToYaw::Feedback>();
      feedback->angular_remaining = 0.0F;
      rotate_handle_->publish_feedback(feedback);
    }
  }
}

void TransitNode::finish_active_goal()
{
  if (active_ == ActiveKind::NONE) {
    return;
  }

  const FailReason reason = machine_ ? machine_->result_reason() : FailReason::OK;
  const bool success = reason == FailReason::OK;

  if (active_ == ActiveKind::TRANSIT && transit_handle_) {
    auto result = std::make_shared<TransitToPose::Result>();
    result->success = success;
    result->reason = to_string(reason);
    result->final_xy_error = static_cast<float>(
      have_pose_ ? distance_2d(last_pose_, target_) : 0.0);
    result->final_yaw_error = static_cast<float>(
      have_pose_ ? shortest_angular_distance(last_pose_.yaw, target_.yaw) : 0.0);

    if (transit_handle_->is_canceling()) {
      transit_handle_->canceled(result);
    } else if (success) {
      transit_handle_->succeed(result);
    } else {
      transit_handle_->abort(result);
    }
    RCLCPP_INFO(
      get_logger(), "TransitToPose 结束: reason=%s xy_err=%.2f yaw_err=%.2f",
      result->reason.c_str(), result->final_xy_error, result->final_yaw_error);
  } else if (active_ == ActiveKind::ROTATE && rotate_handle_) {
    auto result = std::make_shared<RotateToYaw::Result>();
    result->success = success;
    result->reason = to_string(reason);
    result->final_yaw_error = static_cast<float>(
      have_pose_ ? shortest_angular_distance(last_pose_.yaw, target_.yaw) : 0.0);

    if (rotate_handle_->is_canceling()) {
      rotate_handle_->canceled(result);
    } else if (success) {
      rotate_handle_->succeed(result);
    } else {
      rotate_handle_->abort(result);
    }
    RCLCPP_INFO(get_logger(), "RotateToYaw 结束: reason=%s", result->reason.c_str());
  }

  active_ = ActiveKind::NONE;
  transit_handle_.reset();
  rotate_handle_.reset();
  machine_.reset();
  last_dispatched_id_ = 0;
  // dispatch_epoch_ 不归零：跨 goal 单调递增是"旧结果不回流"的唯一保证。
  fp_in_flight_ = false;
  fp_handle_.reset();
  spin_handle_.reset();
  sim_pending_cmd_.reset();
  ready_reported_ = false;
  goal_resolved_ = false;
  have_pose_ = false;
  cancel_requested_ = false;
}

void TransitNode::publish_status()
{
  std_msgs::msg::String msg;

  std::ostringstream os;
  if (active_ == ActiveKind::NONE || !machine_) {
    os << "phase=idle;task=none;dist=0.00;heading=0.00";
  } else {
    const double dist =
      (have_pose_ && !rotate_goal_) ? distance_2d(last_pose_, target_) : 0.0;
    double heading = 0.0;
    if (have_pose_ && !rotate_goal_) {
      const double bearing =
        std::atan2(target_.y - last_pose_.y, target_.x - last_pose_.x);
      heading = shortest_angular_distance(last_pose_.yaw, bearing);
    }
    os << "phase=" << to_string(machine_->phase())
       << ";task=" << (rotate_goal_ ? "rotate" : "transit")
       << ";dist=" << std::fixed << std::setprecision(2) << dist
       << ";heading=" << std::fixed << std::setprecision(2) << heading;
    if (machine_->finished()) {
      os << ";reason=" << to_string(machine_->result_reason());
    }
    fill_feedback();
  }
  msg.data = os.str();
  status_pub_->publish(msg);
}

void TransitNode::cancel_all_on_shutdown(
  rclcpp::executors::SingleThreadedExecutor & executor)
{
  if (active_ == ActiveKind::NONE) {
    return;
  }
  RCLCPP_WARN(get_logger(), "进程退出：取消在飞的 nav2 动作");

  // 先把状态机摘掉，再发取消。
  // 否则下面这 600 ms 的 spin 会把 CANCELED 结果喂回状态机，状态机按重试
  // 逻辑又派发一条新 goal —— 进程都要退了，狗却收到新指令继续走（真机踩过）。
  active_ = ActiveKind::NONE;
  machine_.reset();
  transit_handle_.reset();
  rotate_handle_.reset();
  sim_pending_cmd_.reset();

  follow_path_client_->async_cancel_all_goals();
  spin_client_->async_cancel_all_goals();

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  fp_in_flight_ = false;
}

void TransitNode::on_stop(const std_msgs::msg::Empty::ConstSharedPtr)
{
  if (active_ != ActiveKind::NONE) {
    RCLCPP_WARN(get_logger(), "收到 /go2_transit/stop，打断当前任务");
    cancel_requested_ = true;
  }
}

}  // namespace go2_transit
