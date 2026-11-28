/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <limits>

namespace frontier_exploration_ros2
{

MissionState FrontierExplorerCore::mission_state() const
{
  return mission_state_;
}

std::string FrontierExplorerCore::mission_state_name() const
{
  switch (mission_state_) {
    case MissionState::IDLE:
      return "idle";
    case MissionState::EXPLORING:
      return "exploring";
    case MissionState::BACKTRACKING:
      return "backtracking";
    case MissionState::PATROLLING:
      return "patrolling";
    case MissionState::PAUSED:
      return "paused";
    case MissionState::HOLDING:
      return "holding";
    case MissionState::RESUMING:
      return "resuming";
    case MissionState::RETURNING:
      return "returning";
    case MissionState::COMPLETE:
      return "complete";
  }
  return "idle";
}

const std::string & FrontierExplorerCore::mission_reason() const
{
  return mission_reason_;
}

void FrontierExplorerCore::set_mission_state(MissionState state, const std::string & reason)
{
  mission_state_ = state;
  mission_reason_ = reason;
  if (callbacks.on_mission_state_changed) {
    callbacks.on_mission_state_changed();
  }
}

bool FrontierExplorerCore::mission_paused() const
{
  return mission_state_ == MissionState::PAUSED || mission_state_ == MissionState::HOLDING;
}

std::size_t FrontierExplorerCore::junction_depth() const
{
  return junction_stack_.size();
}

bool FrontierExplorerCore::detect_junction(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!params.cave_mode_enabled || frontiers.empty()) {
    return false;
  }
  const double base_yaw = base_yaw_for_policy(current_pose);
  const double gap_rad = params.junction_angle_sep_deg * 3.14159265358979323846 / 180.0;
  std::vector<double> relative_angles;
  relative_angles.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    const auto point = frontier.goal_point.value_or(frontier.centroid);
    relative_angles.push_back(
      DirectionPolicy::relative_angle(
        point.first - current_pose.position.x,
        point.second - current_pose.position.y,
        base_yaw));
  }
  return direction_policy_.cluster_count(relative_angles, gap_rad) >= 2U;
}

void FrontierExplorerCore::update_junction_memory(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose)
{
  if (!params.cave_mode_enabled || frontiers.empty()) {
    return;
  }
  if (!detect_junction(frontiers, current_pose)) {
    return;
  }
  junction_stack_.push_or_merge(
    current_pose.position.x,
    current_pose.position.y,
    base_yaw_for_policy(current_pose),
    callbacks.now_ns ? callbacks.now_ns() : 0);
}

SuppressedBranchDecision FrontierExplorerCore::note_suppressed_branch()
{
  ++suppressed_branch_streak_;
  const int limit = std::max(1, params.suppressed_branch_retries);
  return suppressed_branch_streak_ >= limit ?
         SuppressedBranchDecision::EXHAUSTED : SuppressedBranchDecision::RETRY;
}

void FrontierExplorerCore::reset_suppressed_branch()
{
  suppressed_branch_streak_ = 0;
}

bool FrontierExplorerCore::try_backtrack()
{
  if (!params.cave_mode_enabled) {
    return false;
  }
  const auto robot = callbacks.get_current_pose();
  while (auto node = junction_stack_.pop_next()) {
    if (
      robot.has_value() &&
      std::hypot(node->x - robot->position.x, node->y - robot->position.y) <
      params.junction_dedup_m)
    {
      // Never send a goal to where the robot already is: it would succeed
      // instantly and bounce between BACKTRACKING and EXPLORING.
      continue;
    }

    clear_suppression_around(
      node->x, node->y,
      std::max(params.suppression_clear_radius_m, params.sensor_effective_range_m));

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = params.global_frame;
    goal.pose.position.x = node->x;
    goal.pose.position.y = node->y;
    goal.pose.orientation.z = std::sin(node->yaw / 2.0);
    goal.pose.orientation.w = std::cos(node->yaw / 2.0);

    const bool sent = send_pose_goal(
      goal,
      "backtrack_junction",
      std::nullopt,
      FrontierSequence{},
      "backtrack to junction");
    if (!sent) {
      // Keep the node for the next attempt when the action server is not ready.
      junction_stack_.push_or_merge(node->x, node->y, node->yaw, node->stamp_ns);
      return false;
    }
    set_mission_state(MissionState::BACKTRACKING, "backtrack");
    return true;
  }
  return false;
}

void FrontierExplorerCore::on_goal_rejected(
  const std::string & goal_kind,
  const geometry_msgs::msg::Pose & goal_pose)
{
  if (goal_kind == "target_standoff") {
    on_target_standoff_result(false);
    return;
  }
  if (goal_kind == "resume_center") {
    on_resume_center_result(false);
    return;
  }
  if (goal_kind == "backtrack_junction") {
    junction_stack_.mark_failed(
      goal_pose.position.x, goal_pose.position.y, params.junction_dedup_m);
    set_mission_state(MissionState::EXPLORING, "backtrack_rejected");
    return;
  }
  if (goal_kind == "return_waypoint" || goal_kind == "return_entrance") {
    ++return_consecutive_failures_;
    if (goal_kind == "return_waypoint") {
      ++return_index_;
    }
    if (return_consecutive_failures_ >= params.return_max_consecutive_failures) {
      return_to_start_completed = true;
      set_mission_state(MissionState::COMPLETE, "return_rejected");
      callbacks.on_exploration_complete();
    } else {
      dispatch_next_return_waypoint();
    }
  }
}

bool FrontierExplorerCore::has_line_of_sight(
  double from_x, double from_y, double to_x, double to_y) const
{
  if (!costmap.has_value()) {
    return false;
  }
  const double resolution = costmap->map().info.resolution;
  const double distance = std::hypot(to_x - from_x, to_y - from_y);
  if (distance <= 0.0) {
    return true;
  }
  const double step_m = std::max(0.05, resolution * 0.5);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step_m)));
  for (int index = 1; index < steps; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(steps);
    int mx = 0;
    int my = 0;
    if (
      !costmap->worldToMapNoThrow(
        from_x + (to_x - from_x) * fraction,
        from_y + (to_y - from_y) * fraction, mx, my))
    {
      return false;
    }
    if (costmap->getCost(mx, my) != static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
      return false;
    }
  }
  return true;
}

void FrontierExplorerCore::clear_suppression_around(double x, double y, double radius_m)
{
  if (frontier_suppression_) {
    frontier_suppression_->clear_near(x, y, radius_m);
  }
}

double FrontierExplorerCore::clearance_score_at(double x, double y) const
{
  if (!costmap.has_value()) {
    return 0.0;
  }
  const double resolution = costmap->map().info.resolution;
  if (resolution <= 0.0) {
    return 0.0;
  }
  int mx = 0;
  int my = 0;
  if (!costmap->worldToMapNoThrow(x, y, mx, my)) {
    return 0.0;
  }
  // Occupied or unknown cells are not standoff candidates.
  if (costmap->getCost(mx, my) != static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
    return 0.0;
  }

  const int max_ring = std::max(
    2, static_cast<int>(std::ceil(2.0 * params.clearance_cap_m / resolution)));
  for (int ring = 1; ring <= max_ring; ++ring) {
    for (int dx = -ring; dx <= ring; ++dx) {
      for (int dy = -ring; dy <= ring; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != ring) {
          continue;
        }
        const int cx = mx + dx;
        const int cy = my + dy;
        if (cx < 0 || cy < 0 || cx >= costmap->getSizeX() || cy >= costmap->getSizeY()) {
          return static_cast<double>(ring - 1) * resolution;
        }
        if (costmap->getCost(cx, cy) != static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
          return static_cast<double>(ring - 1) * resolution;
        }
      }
    }
  }
  return static_cast<double>(max_ring) * resolution;
}

std::optional<geometry_msgs::msg::PoseStamped> FrontierExplorerCore::compute_standoff_pose(
  double target_x,
  double target_y) const
{
  if (!costmap.has_value()) {
    return std::nullopt;
  }
  const auto robot_pose = callbacks.get_current_pose();
  const geometry_msgs::msg::Pose robot = robot_pose.value_or(geometry_msgs::msg::Pose{});
  const double base_yaw = base_yaw_for_policy(robot);

  const double step_rad = params.target_standoff_step_deg * 3.14159265358979323846 / 180.0;
  const double step = std::max(step_rad, 0.01);
  std::optional<geometry_msgs::msg::PoseStamped> best;
  double best_score = -std::numeric_limits<double>::infinity();

  for (double angle = 0.0; angle < 2.0 * 3.14159265358979323846; angle += step) {
    const double px = target_x + params.target_standoff_radius_m * std::cos(angle);
    const double py = target_y + params.target_standoff_radius_m * std::sin(angle);
    const double clearance = clearance_score_at(px, py);
    if (clearance <= 0.0) {
      continue;
    }
    if (!has_line_of_sight(px, py, target_x, target_y)) {
      continue;
    }

    // Soft saturation keeps the score monotonic even when the cave is uniformly open.
    double score = clearance / (clearance + params.clearance_cap_m);
    const auto nearest = trajectory_.nearest_accepted(
      px, py, [](const TrajectoryPoint &) {return true;});
    if (nearest.has_value()) {
      const double path_distance = std::hypot(nearest->x - px, nearest->y - py);
      score += std::exp(-path_distance / params.path_center_sigma_m) * params.path_center_weight;
    }
    const double bearing = DirectionPolicy::relative_angle(
      px - robot.position.x, py - robot.position.y, base_yaw);
    score -= std::abs(bearing) / 3.14159265358979323846 * params.heading_penalty;

    if (score > best_score) {
      best_score = score;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = params.global_frame;
      pose.pose.position.x = px;
      pose.pose.position.y = py;
      const double yaw = std::atan2(target_y - py, target_x - px);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      best = pose;
    }
  }
  return best;
}

void FrontierExplorerCore::ingest_target(double x, double y, int64_t stamp_ns)
{
  if (!params.cave_mode_enabled) {
    return;
  }
  const bool exploring = mission_state_ == MissionState::EXPLORING;
  const bool patrolling = mission_state_ == MissionState::PATROLLING;
  if (!exploring && !patrolling) {
    if (callbacks.log_debug) {
      callbacks.log_debug(
        "Ignoring target in mission state " + mission_state_name());
    }
    return;
  }
  pause_was_patrol_ = patrolling;

  if (has_last_target_) {
    const double distance = std::hypot(x - last_target_.first, y - last_target_.second);
    const double elapsed_ns = static_cast<double>(stamp_ns - last_target_ns_);
    if (distance < params.target_dedup_m && elapsed_ns < params.target_cooldown_s * 1e9) {
      return;
    }
  }

  const auto standoff = compute_standoff_pose(x, y);
  if (!standoff.has_value()) {
    if (callbacks.log_warn) {
      callbacks.log_warn("No free standoff pose around the detected target; ignoring it");
    }
    return;
  }

  has_last_target_ = true;
  last_target_ = {x, y};
  last_target_ns_ = stamp_ns;
  active_target_ = {x, y};

  if (goal_in_progress) {
    request_active_goal_cancel("target");
  }

  const bool sent = send_pose_goal(
    *standoff, "target_standoff", std::nullopt, FrontierSequence{}, "target standoff");
  if (!sent) {
    active_target_.reset();
    return;
  }
  paused_by_user_ = false;
  set_mission_state(MissionState::PAUSED, "target");
  if (callbacks.log_info) {
    callbacks.log_info("Target preemption: driving to the standoff pose");
  }
}

void FrontierExplorerCore::on_target_standoff_result(bool success)
{
  if (!success) {
    active_target_.reset();
    if (callbacks.log_warn) {
      callbacks.log_warn("Target standoff goal failed; resuming exploration");
    }
    if (return_requested_) {
      start_returning();
    } else if (pause_was_patrol_) {
      pause_was_patrol_ = false;
      start_patrolling();
    } else {
      set_mission_state(MissionState::EXPLORING, "target_unreachable");
    }
    return;
  }

  holding_started_ns_ = callbacks.now_ns ? callbacks.now_ns() : 0;

  bool heading_aligned = true;
  if (active_goal_pose.has_value()) {
    const double goal_yaw = yaw_from_quaternion(active_goal_pose->pose.orientation);
    const auto robot = callbacks.get_current_pose();
    if (robot.has_value()) {
      const double error = std::abs(
        DirectionPolicy::normalize_angle(
          goal_yaw - yaw_from_quaternion(robot->orientation)));
      const double tolerance = params.standoff_yaw_tolerance_deg * 3.14159265358979323846 / 180.0;
      heading_aligned = error <= tolerance;
      if (!heading_aligned && callbacks.spin_to_yaw) {
        // Fire-and-forget by design (single-threaded executor); the state reason
        // still reports the unaligned case to the grasp side.
        if (!callbacks.spin_to_yaw(goal_yaw, params.spin_time_allowance_s) && callbacks.log_warn) {
          callbacks.log_warn("Standoff heading not aligned; entering HOLDING anyway");
        }
      }
    }
  }

  set_mission_state(
    MissionState::HOLDING,
    heading_aligned ? "target_reached" : "heading_not_aligned");
  if (callbacks.log_info) {
    callbacks.log_info("Holding at the target standoff pose");
  }
}

void FrontierExplorerCore::on_grab_done()
{
  if (mission_state_ != MissionState::HOLDING) {
    if (callbacks.log_debug) {
      callbacks.log_debug("Ignoring grab_done in mission state " + mission_state_name());
    }
    return;
  }
  if (return_requested_) {
    start_returning();
    return;
  }
  dispatch_resume_goal();
}

bool FrontierExplorerCore::dispatch_resume_goal()
{
  const auto robot = callbacks.get_current_pose();
  if (!robot.has_value()) {
    set_mission_state(MissionState::EXPLORING, "resume_no_pose");
    return false;
  }

  const auto point = trajectory_.nearest_accepted(
    robot->position.x, robot->position.y,
    [this](const TrajectoryPoint & candidate) {
      return clearance_score_at(candidate.x, candidate.y) >= params.resume_clearance_m;
    });
  if (!point.has_value()) {
    set_mission_state(MissionState::EXPLORING, "resume_no_point");
    return false;
  }
  if (
    std::hypot(point->x - robot->position.x, point->y - robot->position.y) <
    params.standoff_reach_tolerance_m)
  {
    set_mission_state(MissionState::EXPLORING, "resume_already_centered");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = params.global_frame;
  goal.pose.position.x = point->x;
  goal.pose.position.y = point->y;
  goal.pose.orientation.z = std::sin(point->yaw / 2.0);
  goal.pose.orientation.w = std::cos(point->yaw / 2.0);

  const bool sent = send_pose_goal(
    goal, "resume_center", std::nullopt, FrontierSequence{}, "resume at path center");
  if (!sent) {
    set_mission_state(MissionState::EXPLORING, "resume_dispatch_failed");
    return false;
  }
  set_mission_state(MissionState::RESUMING, "grab_done");
  return true;
}

void FrontierExplorerCore::on_resume_center_result(bool success)
{
  active_target_.reset();
  no_frontier_streak = 0;
  if (return_requested_) {
    start_returning();
    return;
  }
  if (pause_was_patrol_) {
    pause_was_patrol_ = false;
    start_patrolling();
    return;
  }
  set_mission_state(MissionState::EXPLORING, success ? "resumed" : "resume_failed");
}

void FrontierExplorerCore::tick_mission()
{
  const int64_t now = callbacks.now_ns ? callbacks.now_ns() : 0;

  if (mission_state_ == MissionState::PATROLLING) {
    if (
      params.patrol_timeout_s > 0.0 &&
      static_cast<double>(now - patrol_started_ns_) >= params.patrol_timeout_s * 1e9)
    {
      if (callbacks.log_warn) {
        callbacks.log_warn("Patrol timeout reached; returning to the entrance");
      }
      request_return();
    }
    return;
  }

  if (mission_state_ != MissionState::HOLDING) {
    return;
  }
  const double elapsed_ns = static_cast<double>(now - holding_started_ns_);
  if (elapsed_ns >= params.grab_done_timeout_s * 1e9) {
    if (callbacks.log_warn) {
      callbacks.log_warn("grab_done timeout; resuming exploration");
    }
    if (return_requested_) {
      start_returning();
    } else {
      dispatch_resume_goal();
    }
  }
}

std::optional<std::pair<double, double>> FrontierExplorerCore::active_target() const
{
  return active_target_;
}

double FrontierExplorerCore::frontier_selection_cost(
  const FrontierCandidate & frontier,
  const geometry_msgs::msg::Pose & current_pose) const
{
  const auto point = frontier.goal_point.value_or(frontier.centroid);
  const double distance = std::hypot(
    point.first - current_pose.position.x,
    point.second - current_pose.position.y);
  const double gain = std::max(1.0, static_cast<double>(frontier.size));
  return distance / std::pow(gain, params.weight_gain_ws);
}

bool FrontierExplorerCore::should_switch_to_frontier(
  const FrontierCandidate & candidate,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!active_goal_cost_.has_value()) {
    return true;
  }
  const double threshold = std::clamp(params.goal_hysteresis_threshold, 0.0, 0.99);
  return frontier_selection_cost(candidate, current_pose) <= *active_goal_cost_ * (1.0 - threshold);
}

bool FrontierExplorerCore::start_patrolling()
{
  if (!params.cave_mode_enabled || !params.patrol_enabled || return_requested_) {
    return start_returning();
  }
  patrol_waypoints_ = trajectory_.reverse_waypoints(params.patrol_waypoint_spacing_m);
  patrol_waypoints_.erase(
    std::remove_if(
      patrol_waypoints_.begin(), patrol_waypoints_.end(),
      [this](const TrajectoryPoint & point) {
        return cave_geometry_.is_within_entrance_radius(point.x, point.y);
      }),
    patrol_waypoints_.end());
  if (patrol_waypoints_.size() < 2U) {
    // Too little recorded route to patrol; go home instead.
    return start_returning();
  }
  patrol_index_ = 0;
  patrol_direction_ = 1;
  patrol_dispatch_failures_ = 0;
  patrol_started_ns_ = callbacks.now_ns ? callbacks.now_ns() : 0;
  set_mission_state(MissionState::PATROLLING, "exploration_complete");
  return dispatch_next_patrol_waypoint();
}

bool FrontierExplorerCore::dispatch_next_patrol_waypoint()
{
  if (patrol_waypoints_.empty()) {
    return false;
  }
  const int count = static_cast<int>(patrol_waypoints_.size());
  if (patrol_index_ >= count) {
    patrol_direction_ = -1;
    patrol_index_ = count - 2;
  } else if (patrol_index_ < 0) {
    patrol_direction_ = 1;
    patrol_index_ = 1;
  }
  if (patrol_index_ < 0 || patrol_index_ >= count) {
    return false;
  }

  const auto & point = patrol_waypoints_[static_cast<std::size_t>(patrol_index_)];
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = params.global_frame;
  goal.pose.position.x = point.x;
  goal.pose.position.y = point.y;
  goal.pose.orientation.z = std::sin(point.yaw / 2.0);
  goal.pose.orientation.w = std::cos(point.yaw / 2.0);
  const bool sent = send_pose_goal(
    goal, "patrol_waypoint", std::nullopt, FrontierSequence{}, "patrol waypoint");
  if (sent) {
    patrol_dispatch_failures_ = 0;
  }
  return sent;
}

void FrontierExplorerCore::on_patrol_waypoint_result(bool success)
{
  if (mission_state_ != MissionState::PATROLLING) {
    return;
  }
  if (!success) {
    ++patrol_dispatch_failures_;
    if (patrol_dispatch_failures_ > params.return_max_consecutive_failures) {
      callbacks.log_warn("Patrol waypoints keep failing; returning to the entrance");
      start_returning();
      return;
    }
  }
  patrol_index_ += patrol_direction_;
  dispatch_next_patrol_waypoint();
}

void FrontierExplorerCore::request_return()
{
  if (mission_state_ == MissionState::COMPLETE || mission_state_ == MissionState::IDLE) {
    return;
  }
  return_requested_ = true;
  if (mission_state_ == MissionState::PAUSED || mission_state_ == MissionState::HOLDING) {
    if (callbacks.log_info) {
      callbacks.log_info("Return requested; will return after the current target task");
    }
    return;
  }
  if (goal_in_progress) {
    request_active_goal_cancel("return_requested");
  }
  start_returning();
}

bool FrontierExplorerCore::start_returning()
{
  if (!params.cave_mode_enabled || !cave_geometry_.has_entrance()) {
    return false;
  }
  return_waypoints_ = trajectory_.reverse_waypoints(params.return_waypoint_spacing_m);
  return_waypoints_.erase(
    std::remove_if(
      return_waypoints_.begin(), return_waypoints_.end(),
      [this](const TrajectoryPoint & point) {
        return cave_geometry_.is_within_entrance_radius(point.x, point.y);
      }),
    return_waypoints_.end());
  return_index_ = 0;
  return_consecutive_failures_ = 0;
  return_dispatch_failures_ = 0;
  set_mission_state(MissionState::RETURNING, "exploration_complete");
  return dispatch_next_return_waypoint();
}

bool FrontierExplorerCore::dispatch_next_return_waypoint()
{
  if (!cave_geometry_.has_entrance()) {
    return false;
  }
  while (
    return_index_ < return_waypoints_.size() &&
    cave_geometry_.is_within_entrance_radius(
      return_waypoints_[return_index_].x, return_waypoints_[return_index_].y))
  {
    ++return_index_;
  }

  if (return_index_ < return_waypoints_.size()) {
    const auto & point = return_waypoints_[return_index_];
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = params.global_frame;
    goal.pose.position.x = point.x;
    goal.pose.position.y = point.y;
    goal.pose.orientation.z = std::sin(point.yaw / 2.0);
    goal.pose.orientation.w = std::cos(point.yaw / 2.0);
    const bool sent = send_pose_goal(goal, "return_waypoint", std::nullopt, {}, "return waypoint");
    if (sent) {
      return_dispatch_failures_ = 0;
    }
    return sent;
  }

  const auto & entrance = cave_geometry_.entrance();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = params.global_frame;
  goal.pose.position.x = entrance.x;
  goal.pose.position.y = entrance.y;
  goal.pose.orientation.z = std::sin(entrance.heading / 2.0);
  goal.pose.orientation.w = std::cos(entrance.heading / 2.0);
  const bool sent = send_pose_goal(
    goal, "return_entrance", std::nullopt, {}, "return to entrance");
  if (sent) {
    return_dispatch_failures_ = 0;
  }
  return sent;
}

void FrontierExplorerCore::record_entrance_pose(const geometry_msgs::msg::Pose & current_pose)
{
  if (cave_geometry_.has_entrance()) {
    return;
  }
  cave_geometry_.set_entrance(
    current_pose.position.x,
    current_pose.position.y,
    yaw_from_quaternion(current_pose.orientation));
}

std::optional<EntranceGeometry> FrontierExplorerCore::entrance() const
{
  if (!cave_geometry_.has_entrance()) {
    return std::nullopt;
  }
  return cave_geometry_.entrance();
}

void FrontierExplorerCore::record_robot_trajectory(const geometry_msgs::msg::Pose & current_pose)
{
  const bool have_now = static_cast<bool>(callbacks.now_ns);
  const int64_t now = have_now ? callbacks.now_ns() : 0;
  if (have_now && params.traj_record_rate_hz > 0.0 && last_traj_record_ns_.has_value()) {
    const double min_interval_ns = 1e9 / params.traj_record_rate_hz;
    if (static_cast<double>(now - *last_traj_record_ns_) < min_interval_ns) {
      return;
    }
  }
  last_traj_record_ns_ = now;
  trajectory_.add(
    current_pose.position.x,
    current_pose.position.y,
    yaw_from_quaternion(current_pose.orientation),
    now);
}

const TrajectoryBuffer & FrontierExplorerCore::trajectory() const
{
  return trajectory_;
}

double FrontierExplorerCore::base_yaw_for_policy(
  const geometry_msgs::msg::Pose & current_pose) const
{
  const auto tangent = trajectory_.tangent_yaw();
  if (tangent.has_value()) {
    return *tangent;
  }
  return yaw_from_quaternion(current_pose.orientation);
}

FrontierSequence FrontierExplorerCore::filter_by_cave_geometry(
  const FrontierSequence & frontiers) const
{
  if (!params.cave_mode_enabled || !cave_geometry_.has_entrance()) {
    return frontiers;
  }
  FrontierSequence kept;
  kept.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    const auto point = frontier.goal_point.value_or(frontier.centroid);
    if (!cave_geometry_.is_outside_cave(point.first, point.second)) {
      kept.push_back(frontier);
    }
  }
  return kept;
}

FrontierSequence FrontierExplorerCore::apply_direction_policy(
  const FrontierSequence & frontiers,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!params.cave_mode_enabled || !params.direction_policy_enabled || frontiers.empty()) {
    return frontiers;
  }
  const double base_yaw = base_yaw_for_policy(current_pose);
  std::vector<double> relative_angles;
  relative_angles.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    const auto point = frontier.goal_point.value_or(frontier.centroid);
    relative_angles.push_back(
      DirectionPolicy::relative_angle(
        point.first - current_pose.position.x,
        point.second - current_pose.position.y,
        base_yaw));
  }

  const auto selected = direction_policy_.select_bucket(relative_angles);
  FrontierSequence kept;
  kept.reserve(selected.size());
  for (const auto index : selected) {
    kept.push_back(frontiers[index]);
  }
  return kept;
}

void FrontierExplorerCore::pause_exploration(const std::string & reason)
{
  const bool pausable =
    mission_state_ == MissionState::EXPLORING ||
    mission_state_ == MissionState::BACKTRACKING ||
    mission_state_ == MissionState::RESUMING ||
    mission_state_ == MissionState::RETURNING;
  if (!pausable) {
    if (callbacks.log_debug) {
      callbacks.log_debug("Ignoring pause request in mission state " + mission_state_name());
    }
    return;
  }

  paused_by_user_ = true;
  if (goal_in_progress) {
    request_active_goal_cancel(reason);
  }
  set_mission_state(MissionState::PAUSED, reason);
  if (callbacks.log_info) {
    callbacks.log_info("Exploration paused: " + reason);
  }
}

void FrontierExplorerCore::resume_exploration()
{
  if (mission_state_ != MissionState::PAUSED) {
    if (callbacks.log_debug) {
      callbacks.log_debug("Ignoring resume request in mission state " + mission_state_name());
    }
    return;
  }

  if (!paused_by_user_) {
    // Target preemption pause: operator override abandons the target detour.
    if (goal_in_progress) {
      request_active_goal_cancel("resume_override");
    }
    active_target_.reset();
  }
  paused_by_user_ = false;
  no_frontier_streak = 0;
  if (return_requested_) {
    start_returning();
    return;
  }
  if (pause_was_patrol_) {
    pause_was_patrol_ = false;
    start_patrolling();
    return;
  }
  set_mission_state(MissionState::EXPLORING, "resume");
  if (callbacks.log_info) {
    callbacks.log_info("Exploration resumed");
  }
}

}  // namespace frontier_exploration_ros2
