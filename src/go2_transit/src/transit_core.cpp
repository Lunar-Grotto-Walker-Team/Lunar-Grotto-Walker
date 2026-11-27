// transit_core.cpp — 纯逻辑实现（角度数学 / 路径生成 / 状态机）。

#include "go2_transit/transit_core.hpp"

#include <cmath>

namespace go2_transit
{

double normalize_angle(double angle_rad)
{
  // 值域 (-pi, pi]。±pi 是同一个朝向，调用方一律取 fabs 比较。
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double shortest_angular_distance(double from_rad, double to_rad)
{
  return normalize_angle(to_rad - from_rad);
}

double distance_2d(const Pose2D & a, const Pose2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool within_xy_tolerance(
  const Pose2D & current, const Pose2D & goal, double tolerance_m)
{
  return distance_2d(current, goal) <= tolerance_m;
}

bool within_yaw_tolerance(
  double current_yaw, double target_yaw, double tolerance_rad)
{
  return std::fabs(shortest_angular_distance(current_yaw, target_yaw)) <=
         tolerance_rad;
}

std::vector<Pose2D> build_straight_path(
  const Pose2D & start, const Pose2D & goal, double spacing_m)
{
  const double distance = distance_2d(start, goal);

  // 已在目标上：给两个重合点，DWB 拿到非空路径但不需要走。
  if (distance < 1e-9) {
    return {start, Pose2D{goal.x, goal.y, start.yaw}};
  }

  int steps = 1;
  if (spacing_m > 0.0) {
    steps = static_cast<int>(std::ceil(distance / spacing_m));
    if (steps < 1) {
      steps = 1;
    }
  }

  const double yaw = std::atan2(goal.y - start.y, goal.x - start.x);
  std::vector<Pose2D> path;
  path.reserve(static_cast<size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    path.push_back(Pose2D{
      start.x + (goal.x - start.x) * t,
      start.y + (goal.y - start.y) * t,
      yaw});
  }
  return path;
}

// ---------------------------------------------------------------- 局部坐标系

Pose2D local_to_planning(const Pose2D & origin, double local_x, double local_y)
{
  const double c = std::cos(origin.yaw);
  const double s = std::sin(origin.yaw);
  Pose2D out;
  out.x = origin.x + local_x * c - local_y * s;
  out.y = origin.y + local_x * s + local_y * c;
  out.yaw = origin.yaw;
  return out;
}

bool is_local_frame(const std::string & frame_id)
{
  return frame_id == "local";
}

// ---------------------------------------------------------------- 状态机

TransitStateMachine::TransitStateMachine(const TransitParams & params, TaskMode mode)
: params_(params), mode_(mode)
{
  last_.kind = MachineCommand::Kind::WAIT_READY;
  last_.id = 0;
}

void TransitStateMachine::set_transit_goal(
  const Pose2D & target, double xy_tolerance, bool align_yaw, double yaw_tolerance)
{
  target_ = target;
  align_yaw_ = align_yaw;
  xy_tolerance_ = xy_tolerance > 0.0 ? xy_tolerance : params_.default_xy_tolerance;
  yaw_tolerance_ = yaw_tolerance > 0.0 ? yaw_tolerance : params_.default_yaw_tolerance;
}

void TransitStateMachine::set_rotate_goal(double target_yaw, double yaw_tolerance)
{
  target_yaw_ = target_yaw;
  yaw_tolerance_ = yaw_tolerance > 0.0 ? yaw_tolerance : params_.default_yaw_tolerance;
}

void TransitStateMachine::set_ready(bool ready)
{
  ready_ = ready;
}

void TransitStateMachine::set_pose(const Pose2D & pose)
{
  pose_ = pose;
}

void TransitStateMachine::report_timeout()
{
  fail(FailReason::TIMEOUT);
}

void TransitStateMachine::cancel()
{
  fail(FailReason::CANCELED);
}

void TransitStateMachine::fail(FailReason reason)
{
  if (finished_) {
    return;
  }
  issue_terminal(MachineCommand::Kind::FAIL, reason);
}

bool TransitStateMachine::at_target_xy() const
{
  return within_xy_tolerance(pose_, target_, xy_tolerance_);
}

bool TransitStateMachine::at_target_yaw() const
{
  const double wanted = (mode_ == TaskMode::ROTATE) ? target_yaw_ : target_.yaw;
  return within_yaw_tolerance(pose_.yaw, wanted, yaw_tolerance_);
}

bool TransitStateMachine::consume_retry_or_fail()
{
  ++retries_;
  if (retries_ > params_.max_retries) {
    issue_terminal(MachineCommand::Kind::FAIL, FailReason::UNREACHABLE);
    return false;
  }
  return true;
}

void TransitStateMachine::finish_ok()
{
  issue_terminal(MachineCommand::Kind::SUCCEED, FailReason::OK);
}

void TransitStateMachine::issue_terminal(MachineCommand::Kind kind, FailReason reason)
{
  finished_ = true;
  result_reason_ = reason;
  phase_ = TransitPhase::DONE;
  pending_ = Pending::NONE;

  last_.kind = kind;
  last_.id = ++next_id_;
  last_.spin_target_yaw = 0.0;
  last_.path.clear();
  last_.fail_reason = reason;
}

void TransitStateMachine::issue_spin(double yaw)
{
  pending_ = Pending::SPIN;
  last_.kind = MachineCommand::Kind::SPIN;
  last_.id = ++next_id_;
  last_.spin_target_yaw = yaw;
  last_.path.clear();
  last_.fail_reason = FailReason::OK;
}

void TransitStateMachine::issue_follow_path()
{
  pending_ = Pending::FOLLOW_PATH;
  last_.kind = MachineCommand::Kind::FOLLOW_PATH;
  last_.id = ++next_id_;
  last_.spin_target_yaw = 0.0;
  last_.path = build_straight_path(pose_, target_, params_.path_spacing_m);
  last_.fail_reason = FailReason::OK;
}

void TransitStateMachine::enter_pre_align(double bearing)
{
  phase_ = TransitPhase::PRE_ALIGN;
  issue_spin(bearing);
}

void TransitStateMachine::enter_traverse()
{
  phase_ = TransitPhase::TRAVERSE;
  issue_follow_path();
}

void TransitStateMachine::enter_final_align()
{
  phase_ = TransitPhase::FINAL_ALIGN;
  issue_spin((mode_ == TaskMode::ROTATE) ? target_yaw_ : target_.yaw);
}

void TransitStateMachine::verify()
{
  phase_ = TransitPhase::VERIFY;

  const bool ok = (mode_ == TaskMode::ROTATE)
    ? at_target_yaw()
    : (at_target_xy() && (!align_yaw_ || at_target_yaw()));

  if (ok) {
    finish_ok();
    return;
  }
  if (consume_retry_or_fail()) {
    advance_entry();
  }
}

void TransitStateMachine::advance_entry()
{
  if (mode_ == TaskMode::ROTATE) {
    if (at_target_yaw()) {
      finish_ok();
    } else {
      enter_final_align();
    }
    return;
  }

  // 已经在目标附近：不派发任何运动指令。
  if (at_target_xy()) {
    if (!align_yaw_ || at_target_yaw()) {
      finish_ok();
    } else {
      enter_final_align();
    }
    return;
  }

  const double bearing = std::atan2(target_.y - pose_.y, target_.x - pose_.x);
  const bool need_pre_align =
    !pre_align_done_ && !fallback_used_ &&
    params_.pre_align_threshold_rad > 0.0 &&
    std::fabs(shortest_angular_distance(pose_.yaw, bearing)) >
    params_.pre_align_threshold_rad;

  if (need_pre_align) {
    enter_pre_align(bearing);
    return;
  }
  enter_traverse();
}

MachineCommand TransitStateMachine::next_command()
{
  if (finished_) {
    return last_;
  }
  if (phase_ == TransitPhase::WAIT_READY) {
    if (!ready_) {
      return last_;
    }
    advance_entry();
  }
  return last_;
}

void TransitStateMachine::report_action_done(bool success)
{
  if (finished_) {
    return;
  }

  const Pending was = pending_;
  pending_ = Pending::NONE;

  if (was == Pending::SPIN) {
    if (phase_ == TransitPhase::PRE_ALIGN) {
      if (success) {
        pre_align_done_ = true;
        advance_entry();
        return;
      }
      // 转不动时默认退化成"让 DWB 边走边拐"，且不消耗重试次数。
      if (params_.pre_align_fallback) {
        fallback_used_ = true;
        advance_entry();
        return;
      }
      if (consume_retry_or_fail()) {
        advance_entry();
      }
      return;
    }

    // FINAL_ALIGN
    if (success) {
      verify();
      return;
    }
    if (consume_retry_or_fail()) {
      advance_entry();
    }
    return;
  }

  if (was == Pending::FOLLOW_PATH) {
    if (success) {
      // 2026-11-27 真机: 以前无条件 FINAL_ALIGN, 即使 yaw 早已在容差内也要
      // 原地转一下。四足转 5° 会把 base_footprint 平移 ~2 cm, 正好把刚
      // 达标的 XY 撞出容差 -> verify 失败 -> 重试。
      // 只有"XY 已到、只剩朝向"时才 spin; 位置没到就直接 verify
      // (失败 -> 消耗重试 -> 重新 traverse), 不白花一次 spin。
      if (align_yaw_ && at_target_xy() && !at_target_yaw()) {
        enter_final_align();
      } else {
        verify();
      }
      return;
    }
    if (consume_retry_or_fail()) {
      advance_entry();
    }
  }
}

const char * to_string(TransitPhase phase)
{
  switch (phase) {
    case TransitPhase::WAIT_READY:
      return "wait_ready";
    case TransitPhase::PRE_ALIGN:
      return "pre_align";
    case TransitPhase::TRAVERSE:
      return "traverse";
    case TransitPhase::FINAL_ALIGN:
      return "final_align";
    case TransitPhase::VERIFY:
      return "verify";
    case TransitPhase::DONE:
      return "done";
  }
  return "unknown";
}

const char * to_string(FailReason reason)
{
  switch (reason) {
    case FailReason::OK:
      return "ok";
    case FailReason::TIMEOUT:
      return "timeout";
    case FailReason::UNREACHABLE:
      return "unreachable";
    case FailReason::NAV2_UNAVAILABLE:
      return "nav2_unavailable";
    case FailReason::TF_UNAVAILABLE:
      return "tf_unavailable";
    case FailReason::INVALID_GOAL:
      return "invalid_goal";
    case FailReason::CANCELED:
      return "canceled";
  }
  return "unknown";
}

}  // namespace go2_transit
