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

#include "frontier_explorer_core_detail.hpp"

#include <action_msgs/msg/goal_status.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::log_frontier_stages(
  std::size_t snapshot_count,
  std::size_t cave_count,
  std::size_t suppressed_count,
  std::size_t policy_count,
  bool selected,
  const FrontierSearchStats & stats)
{
  if (!callbacks.log_info) {
    return;
  }
  std::ostringstream oss;
  oss << "frontier stages: snapshot=" << snapshot_count
      << " cave=" << cave_count
      << " suppressed=" << suppressed_count
      << " policy=" << policy_count
      << " selected=" << (selected ? 1 : 0)
      << " | clusters=" << stats.clusters
      << " candidates=" << stats.candidates
      << " reject(small=" << stats.rejected_small
      << " no_goal=" << stats.rejected_no_goal
      << " clearance=" << stats.rejected_clearance
      << " reachability=" << stats.rejected_reachability
      << " blocked=" << stats.rejected_blocked
      << ") backoff_saved=" << stats.backoff_saved;
  const std::string key = oss.str();
  const int64_t now = callbacks.now_ns ? callbacks.now_ns() : 0;
  if (key == last_stage_log_key_ && now - last_stage_log_ns_ < 5000000000LL) {
    return;
  }
  last_stage_log_key_ = key;
  last_stage_log_ns_ = now;
  callbacks.log_info(key);
}

void FrontierExplorerCore::try_send_next_goal()
{
  // Main scheduler entrypoint: exits early unless maps, pose, and lifecycle gates are ready.
  if (!exploration_enabled) {
    return;
  }

  if (mission_paused()) {
    // Paused for a target preemption or by an operator: keep maps and state but
    // do not dispatch any exploration goal.
    return;
  }

  if (mission_state_ == MissionState::PATROLLING) {
    // Patrol progress is driven by result callbacks; this is the retry/timeout
    // safety net when a waypoint never gets dispatched.
    if (!goal_in_progress) {
      ++patrol_dispatch_failures_;
      if (patrol_dispatch_failures_ > params.return_max_consecutive_failures) {
        callbacks.log_warn("Patrol waypoint could not be dispatched; returning to the entrance");
        start_returning();
      } else {
        dispatch_next_patrol_waypoint();
      }
    }
    return;
  }

  if (mission_state_ == MissionState::RETURNING) {
    const auto pose = callbacks.get_current_pose();
    if (
      pose.has_value() && cave_geometry_.has_entrance() &&
      cave_geometry_.is_within_entrance_radius(pose->position.x, pose->position.y))
    {
      return_to_start_completed = true;
      set_mission_state(MissionState::COMPLETE, "returned_to_entrance");
      callbacks.on_exploration_complete();
      callbacks.log_info("Returned to the cave entrance");
    } else if (!goal_in_progress) {
      // No return goal in flight: either the first dispatch failed or a segment
      // ended without producing the next one. Retry with a bounded budget.
      ++return_dispatch_failures_;
      if (return_dispatch_failures_ > params.return_max_consecutive_failures) {
        return_to_start_completed = true;
        set_mission_state(MissionState::COMPLETE, "return_dispatch_failed");
        callbacks.on_exploration_complete();
        callbacks.log_warn("Return goal could not be dispatched; completing in place");
      } else {
        dispatch_next_return_waypoint();
      }
    }
    return;
  }

  if (mission_state_ == MissionState::COMPLETE) {
    return;
  }

  if (return_to_start_completed) {
    // Exploration is fully finished.
    return;
  }

  if (goal_in_progress || !map.has_value() || !costmap.has_value()) {
    // Cannot dispatch a new goal while one is active, or before required maps arrive.
    return;
  }

  commit_deferred_costmap_search_input_updates();

  if (awaiting_map_refresh && !post_goal_settle_ready()) {
    // Settle window is still active after previous goal completion.
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    // Skip scheduling until TF provides a valid robot pose.
    return;
  }

  if (awaiting_map_refresh) {
    clear_post_goal_wait_state();
  }

  record_start_pose(*current_pose);
  record_entrance_pose(*current_pose);

  if (!pending_frontier_sequence.empty()) {
    // Replacement/preemption path already computed a concrete next target.
    dispatch_pending_frontier_goal(current_pose);
    return;
  }

  FrontierSnapshot snapshot;
  const double min_goal_distance = frontier_snapshot_min_goal_distance_for_pose(*current_pose);
  try {
    snapshot = get_frontier_snapshot(
      *current_pose,
      min_goal_distance);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier update: ") + exc.what());
    return;
  }

  const FrontierSequence & frontiers = snapshot.frontiers;
  const FrontierSequence cave_frontiers = filter_by_cave_geometry(frontiers);
  update_junction_memory(cave_frontiers, *current_pose);
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(cave_frontiers);
  // 早于任何 return 打一次：候选在 cave/suppression 被吃光时也能看到计数。
  log_frontier_stages(
    frontiers.size(), cave_frontiers.size(), filtered_frontiers.size(),
    0U, false, snapshot.search_stats);
  bool escape_mode_active = false;
  if (filtered_frontiers.empty() && !cave_frontiers.empty()) {
    no_frontiers_reported = false;
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);
    if (!all_frontiers_suppressed_reported) {
      callbacks.log_info("All currently detected frontiers are temporarily suppressed");
      all_frontiers_suppressed_reported = true;
    }
    // Cave mode: a branch whose frontiers were all tried and failed is a dead
    // end, not a reason to park. Backtrack first; when the stack is empty,
    // escalate a bounded number of times and then treat the branch as exhausted
    // instead of falling into the legacy "stay" behavior forever.
    if (params.cave_mode_enabled) {
      if (try_backtrack()) {
        return;
      }
      if (note_suppressed_branch() == SuppressedBranchDecision::RETRY) {
        callbacks.log_info(
          "All in-cave frontiers are temporarily suppressed; retrying before treating this branch as exhausted");
        return;
      }
      reset_suppressed_branch();
    } else {
      handle_all_frontiers_suppressed(*current_pose);
      return;
    }
  }

  if (filtered_frontiers.empty() && cave_frontiers.empty() && params.escape_enabled &&
    min_goal_distance > 0.0)
  {
    FrontierSnapshot escape_snapshot;
    try {
      escape_snapshot = get_frontier_snapshot(*current_pose, 0.0);
    } catch (const std::out_of_range & exc) {
      callbacks.log_warn(std::string("Skipping escape-mode frontier update: ") + exc.what());
      return;
    }

    const FrontierSequence escape_cave_frontiers =
      filter_by_cave_geometry(escape_snapshot.frontiers);
    FrontierSequence escape_filtered_frontiers =
      filter_frontiers_for_suppression(escape_cave_frontiers);
    if (escape_filtered_frontiers.empty() && !escape_cave_frontiers.empty()) {
      no_frontiers_reported = false;
      no_reachable_frontier_reported = false;
      publish_frontier_markers(escape_filtered_frontiers);
      if (!all_frontiers_suppressed_reported) {
        callbacks.log_info("All currently detected frontiers are temporarily suppressed");
        all_frontiers_suppressed_reported = true;
      }
      handle_all_frontiers_suppressed(*current_pose);
      return;
    }

    if (!escape_filtered_frontiers.empty()) {
      filtered_frontiers = std::move(escape_filtered_frontiers);
      escape_mode_active = true;
      callbacks.log_info(
        "No frontier survived the active minimum-distance filters (min_goal_distance=" +
        detail::format_meters(min_goal_distance) +
        "); retrying in escape mode with min-distance search and dispatch bypassed");
    }
  }

  all_frontiers_suppressed_reported = false;
  suppressed_return_to_start_started = false;
  if (filtered_frontiers.empty()) {
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);

    // Cave mode: no in-cave frontier left at this pose means the branch ended.
    // Return to the most recent junction before considering completion.
    if (try_backtrack()) {
      return;
    }

    // 2026-11-19: 候选为空 ≠ 探索完成。raw map 上若仍有"未知且 8 邻接可通行"
    // 的格, 说明 frontier 只是被 costmap/size/backoff/suppression 挡住或到不了,
    // 此时只报告并保持探索, 绝不允许累加完成计数。
    if (params.exploration_complete_raw_boundary_guard) {
      const std::size_t raw_boundary = raw_unknown_boundary_cell_count();
      if (raw_boundary > 0U) {
        no_frontier_streak = 0;
        no_frontier_streak_map_generation = -1;
        if (!raw_boundary_blocks_completion_reported) {
          callbacks.log_info(
            "frontiers exist but unreachable/suppressed (raw unknown boundary cells=" +
            std::to_string(raw_boundary) + "); not declaring completion");
          raw_boundary_blocks_completion_reported = true;
        }
        return;
      }
      raw_boundary_blocks_completion_reported = false;
    }

    // 2026-11-11: 完成判定去抖。地图瞬时为空/断裂时不再一次就锁存"探索完成"。
    // 2026-11-19: 只有拿到新的一帧 /exploration_map 才计一次 scan。本函数由
    // map(2 Hz) / global costmap(2 Hz) / local costmap(5 Hz) 三个回调共同调用,
    // 之前同一帧被重复计数, `exploration_complete_confirm_scans: 10` 实测
    // 4.1 s 就满足(2026-11-19 真机日志), 任何 1~2 s 的 frontier 抖动都会被
    // 直接锁存成永久完成。
    if (no_frontier_streak_map_generation != map_generation) {
      no_frontier_streak += 1;
      no_frontier_streak_map_generation = map_generation;
    }
    const int confirm_scans = std::max(1, params.exploration_complete_confirm_scans);
    if (no_frontier_streak < confirm_scans) {
      if (!no_frontiers_reported) {
        callbacks.log_info(
          "No frontiers in this scan (" + std::to_string(no_frontier_streak) + "/" +
          std::to_string(confirm_scans) + "); waiting for confirmation before declaring completion");
        no_frontiers_reported = true;
      }
      callbacks.log_debug(
        "no frontier streak " + std::to_string(no_frontier_streak) + "/" +
        std::to_string(confirm_scans));
      return;
    }

    if (!map_known_cells_reach(params.exploration_complete_min_known_cells)) {
      // 地图还太小(启动瞬间/SLAM 刚重置), 不认为"探索完了"。
      if (!no_frontier_small_map_reported) {
        callbacks.log_info(
          "No frontiers for " + std::to_string(confirm_scans) +
          " consecutive scans, but the map is still too small (known cells < " +
          std::to_string(params.exploration_complete_min_known_cells) +
          "); keeping exploration alive");
        no_frontier_small_map_reported = true;
      }
      return;
    }

    callbacks.log_info(
      "Exploration finished: no frontier for " + std::to_string(confirm_scans) +
      " consecutive scans");
    no_frontiers_reported = true;
    handle_exploration_complete(*current_pose);
    return;
  }

  no_frontier_streak = 0;
  no_frontier_streak_map_generation = -1;
  no_frontiers_reported = false;
  no_frontier_small_map_reported = false;
  raw_boundary_blocks_completion_reported = false;
  reset_suppressed_branch();
  const FrontierSequence policy_frontiers = apply_direction_policy(filtered_frontiers, *current_pose);
  auto selection = select_frontier(policy_frontiers, *current_pose);
  if (escape_mode_active && selection.frontier.has_value()) {
    selection.mode = "escape";
  }
  publish_frontier_markers(policy_frontiers);
  log_frontier_stages(
    frontiers.size(), cave_frontiers.size(), filtered_frontiers.size(),
    policy_frontiers.size(), selection.frontier.has_value(),
    snapshot.search_stats);

  if (!selection.frontier.has_value()) {
    if (!no_reachable_frontier_reported) {
      callbacks.log_info(
        "No reachable frontier candidate is available right now; waiting for a map update");
      no_reachable_frontier_reported = true;
    }
    return;
  }

  no_reachable_frontier_reported = false;
  const auto frontier_sequence = select_frontier_sequence(
    policy_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    return;
  }

  send_frontier_goal(
    frontier_sequence,
    *current_pose,
    "Sending frontier goal (" + selection.mode + "): " + describe_frontier(frontier_sequence.front()),
    escape_mode_active);
}

void FrontierExplorerCore::reset_exploration_runtime_state(bool clear_maps)
{
  clear_post_goal_wait_state();
  clear_active_goal_progress_state();

  no_frontier_streak = 0;
  no_frontier_streak_map_generation = -1;
  no_frontiers_reported = false;
  no_frontier_small_map_reported = false;
  no_reachable_frontier_reported = false;
  all_frontiers_suppressed_reported = false;
  raw_boundary_map_generation = -1;
  raw_boundary_cell_count_cache = 0;
  raw_boundary_blocks_completion_reported = false;
  return_to_start_started = false;
  return_to_start_completed = false;
  suppressed_return_to_start_started = false;
  cancel_request_in_progress = false;
  pending_cancel_reason.reset();
  pending_frontier_sequence.clear();
  pending_frontier_selection_mode.clear();
  pending_frontier_dispatch_context.clear();
  active_goal_blocked_reason.reset();
  pending_costmap_search_input_update = false;
  pending_local_costmap_search_input_update = false;
  distance_completed_frontier.reset();
  last_low_gain_reselection_time_ns.reset();
  reset_replacement_candidate_tracking();
  last_published_frontier_signature.reset();
  frontier_snapshot.reset();
  raw_frontier_debug_cache.reset();
  mrtsp_order_cache.reset();
  frontier_suppression_.reset();
  frontier_suppression_activation_ns_.reset();

  if (clear_maps) {
    map.reset();
    decision_map.reset();
    decision_map_msg.reset();
    decision_map_workspace = DecisionMapWorkspace{};
    costmap.reset();
    local_costmap.reset();
    map_generation = 0;
    decision_map_generation = 0;
    costmap_generation = 0;
    local_costmap_generation = 0;
    decision_map_dirty = false;
    decision_map_cache_hits = 0;
    decision_map_cache_misses = 0;
    frontier_snapshot_cache_hits = 0;
    frontier_snapshot_cache_misses = 0;
    mrtsp_order_cache_hits = 0;
    mrtsp_order_cache_misses = 0;
  }
}

void FrontierExplorerCore::start_exploration_session()
{
  reset_exploration_runtime_state(true);
  exploration_enabled = true;
  paused_by_user_ = false;
  if (params.frontier_suppression_enabled) {
    frontier_suppression_activation_ns_ =
      callbacks.now_ns() +
      static_cast<int64_t>(params.frontier_suppression_startup_grace_period_s * 1e9);
  }
  set_mission_state(MissionState::EXPLORING, "start");
}

void FrontierExplorerCore::stop_exploration_session(const std::string & reason)
{
  exploration_enabled = false;
  paused_by_user_ = false;
  patrol_waypoints_.clear();
  patrol_index_ = -1;
  patrol_direction_ = 1;
  patrol_dispatch_failures_ = 0;
  return_dispatch_failures_ = 0;
  return_requested_ = false;
  pause_was_patrol_ = false;
  reset_exploration_runtime_state(true);
  if (goal_in_progress) {
    request_active_goal_cancel(reason);
  }
  set_mission_state(MissionState::IDLE, reason);
}

bool FrontierExplorerCore::ready_for_shutdown() const
{
  return !goal_in_progress && !cancel_request_in_progress;
}

void FrontierExplorerCore::set_goal_state(GoalLifecycleState state)
{
  // Keep enum state and convenience bool in sync from one write location.
  goal_state = state;
  // IDLE <-> non-IDLE mapping kept centralized to avoid inconsistent flags.
  goal_in_progress = state != GoalLifecycleState::IDLE;
}

void FrontierExplorerCore::mark_dispatch_state(int dispatch_id, GoalLifecycleState state)
{
  // Tracks lifecycle by dispatch id for stale callback filtering and diagnostics.
  dispatch_states[dispatch_id] = state;
}

void FrontierExplorerCore::reset_replacement_candidate_tracking()
{
  // Clear both candidate payload and debounce counter together to avoid stale "stable" hits.
  replacement_candidate_frontier.reset();
  replacement_candidate_hits = 0;
}

bool FrontierExplorerCore::has_stable_replacement_candidate(const FrontierSequence & frontier_sequence)
{
  if (frontier_sequence.empty()) {
    reset_replacement_candidate_tracking();
    return false;
  }

  const FrontierLike & selected_frontier = frontier_sequence.front();
  if (replacement_candidate_frontier.has_value() &&
    are_frontiers_equivalent(selected_frontier, replacement_candidate_frontier))
  {
    // Debounce only the selected replacement frontier. Later MRTSP order entries may change
    // without changing the actual replacement target, and Python reference behavior treats
    // that case as stable.
    replacement_candidate_hits += 1;
  } else {
    // New candidate observed; restart stability counter.
    replacement_candidate_frontier = selected_frontier;
    replacement_candidate_hits = 1;
  }

  // Candidate is considered stable only after configurable repeated hits.
  return replacement_candidate_hits >= replacement_required_hits;
}

std::optional<std::pair<double, double>> FrontierExplorerCore::active_goal_target_point() const
{
  if (active_goal_pose.has_value()) {
    return std::pair<double, double>{
      active_goal_pose->pose.position.x,
      active_goal_pose->pose.position.y,
    };
  }

  if (active_goal_frontier.has_value()) {
    return frontier_position(*active_goal_frontier);
  }

  return std::nullopt;
}

FrontierLike FrontierExplorerCore::frontier_with_goal_point(
  const FrontierLike & frontier,
  const std::pair<double, double> & goal_point) const
{
  FrontierLike updated_frontier = frontier;
  updated_frontier.goal_point = goal_point;
  return updated_frontier;
}

std::optional<double> FrontierExplorerCore::active_goal_visible_reveal_length() const
{
  if (!map.has_value() || !costmap.has_value() || !active_goal_frontier.has_value()) {
    return std::nullopt;
  }

  // Model the sensor at the dispatched target pose, not at the robot's current pose.
  // This keeps the gain estimate aligned with "what would be revealed if we finish this goal?"
  const auto goal_point = active_goal_target_point();
  if (!goal_point.has_value()) {
    return std::nullopt;
  }
  const auto reference_point = frontier_reference_point(*active_goal_frontier);
  double heading = 0.0;
  const double dx = reference_point.first - goal_point->first;
  const double dy = reference_point.second - goal_point->second;
  if (std::hypot(dx, dy) > 1e-6) {
    // Preferred heading points from the dispatched goal toward the frontier reference geometry.
    heading = std::atan2(dy, dx);
  } else if (active_goal_pose.has_value()) {
    // Degenerate frontier geometry falls back to the yaw of the already-dispatched goal pose.
    heading = detail::yaw_from_quaternion(active_goal_pose->pose.orientation);
  }

  const double yaw_offset_rad = params.goal_preemption_lidar_yaw_offset_deg * (detail::kPi / 180.0);
  geometry_msgs::msg::Pose sensor_pose;
  sensor_pose.position.x = goal_point->first;
  sensor_pose.position.y = goal_point->second;
  sensor_pose.orientation = detail::quaternion_from_yaw(heading + yaw_offset_rad);

  // The helper returns a local, occlusion-aware reveal length estimate around the target pose.
  const auto visible_reveal_bounds =
    active_goal_frontier->visible_reveal_bounds;

  const auto visible_gain = compute_visible_reveal_gain(
    sensor_pose,
    *map,
    *costmap,
    local_costmap,
    params.goal_preemption_lidar_range_m,
    params.goal_preemption_lidar_fov_deg,
    params.goal_preemption_lidar_ray_step_deg,
    visible_reveal_bounds);
  if (!visible_gain.has_value()) {
    return std::nullopt;
  }

  return visible_gain->visible_reveal_length_m;
}

double FrontierExplorerCore::frontier_snapshot_min_goal_distance_for_pose(
  const geometry_msgs::msg::Pose & current_pose)
{
  if (!distance_completed_frontier.has_value() ||
    params.goal_preemption_complete_if_within_m <= 0.0)
  {
    return params.frontier_candidate_min_goal_distance_m;
  }

  const auto completed_goal = frontier_position(*distance_completed_frontier);
  const double completed_distance = std::hypot(
    completed_goal.first - current_pose.position.x,
    completed_goal.second - current_pose.position.y);
  if (completed_distance > params.goal_preemption_complete_if_within_m) {
    distance_completed_frontier.reset();
    return params.frontier_candidate_min_goal_distance_m;
  }

  return std::max(
    params.frontier_candidate_min_goal_distance_m,
    params.goal_preemption_complete_if_within_m);
}

void FrontierExplorerCore::consider_preempt_active_goal(const std::string & trigger_source)
{
  // Costmap-triggered calls only update blocked reason; map-triggered calls may reseat goals.
  const bool completion_distance_enabled = params.goal_preemption_complete_if_within_m > 0.0;
  const bool preemption_allowed = (
    params.goal_preemption_enabled ||
    params.goal_skip_on_blocked_goal ||
    completion_distance_enabled);
  if (!preemption_allowed) {
    return;
  }

  if (!map.has_value() || !costmap.has_value() || !goal_in_progress || !active_goal_frontier.has_value()) {
    // Preemption logic requires an active frontier goal plus current map context.
    return;
  }

  if (!goal_handle) {
    // Goal has not been accepted yet, so cancel/reselect actions cannot be issued.
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    // Pose uncertainty: defer preemption decision.
    return;
  }

  const auto active_goal_cost_status = params.goal_skip_on_blocked_goal ?
    frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};
  if (
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled &&
    !completion_distance_enabled)
  {
    // Nothing to do when all active-goal update triggers are effectively inactive.
    return;
  }

  if (trigger_source != "map") {
    // Costmap/local-costmap callbacks intentionally skip full frontier reselection work.
    if (active_goal_cost_status.has_value()) {
      active_goal_blocked_reason = *active_goal_cost_status;
    }
    return;
  }

  if (cancel_request_in_progress) {
  // Avoid duplicate cancel requests while previous cancel handshake is in flight.
  return;
} 

if (!active_goal_cost_status.has_value()) {
  active_goal_blocked_reason.reset();
}

  const auto active_goal_point = active_goal_target_point();
  if (!active_goal_point.has_value()) {
    return;
  }
  const double active_goal_distance = std::hypot(
    active_goal_point->first - current_pose->position.x,
    active_goal_point->second - current_pose->position.y);
  std::optional<double> visible_reveal_length;
  // Near-goal completion is independent from visible-gain preemption; it is a close-enough guard.
  const bool revealed_completion_distance_reached =
    !active_goal_cost_status.has_value() &&
    completion_distance_enabled &&
    active_goal_distance <= params.goal_preemption_complete_if_within_m;
  const std::string completion_distance_reason =
    "active frontier considered complete by goal_preemption_complete_if_within_m (distance=" +
    detail::format_meters(active_goal_distance) + ", threshold=" +
    detail::format_meters(params.goal_preemption_complete_if_within_m) + ")";

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled)
  {
    return;
  }

  if (revealed_completion_distance_reached) {
    distance_completed_frontier = frontier_with_goal_point(
      *active_goal_frontier,
      *active_goal_point);
    request_active_goal_cancel(completion_distance_reason);
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_sent_time_ns.has_value() &&
    !active_goal_cost_status.has_value())
  {
    // Without send timestamp we cannot evaluate time-based visible-gain preemption gate.
    return;
  }

  const int64_t now_ns = callbacks.now_ns();
  const double elapsed = active_goal_sent_time_ns.has_value() ?
    static_cast<double>(now_ns - *active_goal_sent_time_ns) / 1e9 : 0.0;

  // Time-based gate for visible-gain preemption to avoid immediate churn.
  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    elapsed < params.goal_preemption_min_interval_s)
  {
    return;
  }

  // Visible-gain gate is only meaningful on the map-triggered revealed-preemption path.
  const bool visible_gain_gate_active =
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled;
  bool visible_reveal_gain_exhausted = false;

  if (visible_gain_gate_active && !revealed_completion_distance_reached) {
    visible_reveal_length = active_goal_visible_reveal_length();
    if (!visible_reveal_length.has_value()) {
      callbacks.log_warn(
        "Skipping visible reveal gain gate for the active goal; falling back to frontier snapshot reselection");
    } else if (
      map.has_value() &&
      static_cast<double>(frontier_size(*active_goal_frontier)) * map->map().info.resolution <=
      *visible_reveal_length)
    {
      // Small frontier clusters can look fully revealable even with low absolute gain thresholds.
      // If the current visible reveal already covers the whole cluster-size proxy, skip preemption.
      reset_replacement_candidate_tracking();
      return;
    } else if (*visible_reveal_length >= params.goal_preemption_lidar_min_reveal_length_m) {
      // Keep current goal while target pose can still reveal enough unexplored boundary on arrival.
      reset_replacement_candidate_tracking();
      return;
    } else {
      // Low visible gain alone is not enough to preempt; the refreshed frontier snapshot still decides.
      visible_reveal_gain_exhausted = true;
    }
  }

  if (
    visible_reveal_gain_exhausted &&
    last_low_gain_reselection_time_ns.has_value() &&
    params.goal_preemption_min_interval_s > 0.0)
  {
    const double elapsed_since_low_gain_reselection =
      static_cast<double>(now_ns - *last_low_gain_reselection_time_ns) / 1e9;
    if (elapsed_since_low_gain_reselection < params.goal_preemption_min_interval_s) {
      return;
    }
  }
  if (visible_reveal_gain_exhausted) {
    last_low_gain_reselection_time_ns = now_ns;

    const std::string visible_gain_preemption_reason =
    visible_reveal_length.has_value() ?
    "active frontier visible reveal gain exhausted at target pose (visible=" +
    detail::format_meters(*visible_reveal_length) + ", required=" +
    detail::format_meters(params.goal_preemption_lidar_min_reveal_length_m) +
    "); canceling active goal and deferring frontier reselection until the next map refresh" :
    "active frontier visible reveal gain could not be evaluated; canceling active goal and deferring frontier reselection until the next map refresh";

  request_active_goal_cancel(visible_gain_preemption_reason);
  return;
  }

  // THIS PART MIGHT BE PROBLEMATIC FOR CPU USAGE
  // CHECK IN THE FUTURE

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_candidate_min_goal_distance_m);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier reselection: ") + exc.what());
    return;
  }

  const FrontierSequence & frontiers = snapshot.frontiers;
  const FrontierSequence cave_frontiers = filter_by_cave_geometry(frontiers);
  update_junction_memory(cave_frontiers, *current_pose);
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(cave_frontiers);
  // 早于任何 return 打一次：候选在 cave/suppression 被吃光时也能看到计数。
  log_frontier_stages(
    frontiers.size(), cave_frontiers.size(), filtered_frontiers.size(),
    0U, false, snapshot.search_stats);
  if (filtered_frontiers.empty() && !cave_frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    callbacks.log_info("All replacement frontiers are temporarily suppressed");
    if (active_goal_cost_status.has_value()) {
      // If current goal is blocked and no alternative frontier exists, cancel proactively.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no unsuppressed replacement frontier is available");
    }
    return;
  }

  if (filtered_frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    if (active_goal_cost_status.has_value()) {
      // Blocked-goal path cancels proactively when no replacement target exists at all.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no replacement frontier is available");
    }
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    frontier_exists_in_set(active_goal_frontier, filtered_frontiers))
  {
    // Snapshot membership remains the final authority for revealed-preemption.
    // If the frontier still exists in the refreshed set, keep the active goal.
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    return;
  }

  const FrontierSequence policy_frontiers = apply_direction_policy(filtered_frontiers, *current_pose);
  const auto selection = select_frontier(policy_frontiers, *current_pose);
  publish_frontier_markers(policy_frontiers);
  log_frontier_stages(
    frontiers.size(), cave_frontiers.size(), filtered_frontiers.size(),
    policy_frontiers.size(), selection.frontier.has_value(),
    snapshot.search_stats);
  const auto frontier_sequence = select_frontier_sequence(
    policy_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    // Selection produced no dispatchable target.
    reset_replacement_candidate_tracking();
    return;
  }

  if (
    !active_goal_cost_status.has_value() && selection.frontier.has_value() &&
    !should_switch_to_frontier(*selection.frontier, *current_pose))
  {
    // Goal hysteresis: the replacement is not meaningfully better than the active
    // goal, so keep the current one to avoid flip-flopping between neighbours.
    reset_replacement_candidate_tracking();
    publish_frontier_markers(policy_frontiers);
    return;
  }

  if (are_frontier_sequences_equivalent(frontier_sequence, active_goal_frontiers)) {
    // Replacement candidate is effectively same as active target.
    reset_replacement_candidate_tracking();
    return;
  }

  const std::string revealed_preemption_reason = active_goal_cost_status.value_or(
    visible_reveal_gain_exhausted && visible_reveal_length.has_value() ?
    "active frontier revealed and visible reveal gain exhausted at target pose (visible=" +
    detail::format_meters(*visible_reveal_length) + ", required=" +
    detail::format_meters(params.goal_preemption_lidar_min_reveal_length_m) +
    "); preempting to the next frontier" :
    "active frontier no longer appears in the latest frontier snapshot after reveal updates; preempting to the next frontier");

  const std::string goal_update_log_prefix = active_goal_cost_status.has_value() ?
    "Skipping blocked frontier goal" :
    "Preempting active frontier goal";

  request_frontier_reselection(
    frontier_sequence,
    *current_pose,
    selection.mode,
    revealed_preemption_reason,
    goal_update_log_prefix);
}

void FrontierExplorerCore::request_frontier_reselection(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & selection_mode,
  const std::string & reselection_reason,
  const std::string & goal_update_log_prefix)
{
  if (!goal_handle || frontier_sequence.empty()) {
    // Reselection is meaningful only with an active accepted goal and a non-empty replacement.
    return;
  }

  if (!pending_frontier_sequence.empty() &&
    are_frontier_sequences_equivalent(frontier_sequence, pending_frontier_sequence))
  {
    // Keep queue compact by suppressing duplicate pending replacement requests.
    return;
  }

  if (!has_stable_replacement_candidate(frontier_sequence)) {
    // Require repeated observation before replacing active target.
    return;
  }

  pending_frontier_sequence = frontier_sequence;
  pending_frontier_selection_mode = selection_mode;
  pending_frontier_dispatch_context = "reselected";
  callbacks.log_info(
    goal_update_log_prefix + ": " + reselection_reason);
  // Use Nav2's native action preemption path by sending the replacement goal directly.
  // Post-goal settle remains reserved for terminal result paths, not live replacements.
  dispatch_pending_frontier_goal(current_pose);
}

void FrontierExplorerCore::request_active_goal_cancel(const std::string & reason)
{
  // Keep earliest cancel reason; subsequent requests should not erase context.
  if (!pending_cancel_reason.has_value()) {
    pending_cancel_reason = reason;
  }

  if (!goal_handle || cancel_request_in_progress) {
    // Cancel will be attempted later when handle exists and no request is in progress.
    return;
  }

  issue_active_goal_cancel();
}

void FrontierExplorerCore::issue_active_goal_cancel()
{
  if (!goal_handle || cancel_request_in_progress) {
    return;
  }

  const std::string reason = pending_cancel_reason.value_or("Canceling active goal");
  pending_cancel_reason.reset();
  cancel_request_in_progress = true;
  set_goal_state(GoalLifecycleState::CANCELING);
  callbacks.log_debug(reason);

  const int dispatch_id = current_dispatch_id;
  // Bind cancel response to current dispatch to ignore late/stale acknowledgements.
  goal_handle->cancel_goal_async(
    [this, dispatch_id](bool accepted, const std::string & error_message) {
      cancel_response_callback(dispatch_id, accepted, error_message);
    });
}

void FrontierExplorerCore::cancel_response_callback(
  int dispatch_id,
  bool cancel_accepted,
  const std::string & error_message)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore cancel responses from superseded goals.
    return;
  }

  if (!cancel_request_in_progress || goal_state != GoalLifecycleState::CANCELING) {
    // Result handling or a newer dispatch may have already closed this cancel path.
    // Late cancel responses must not resurrect an active goal after cleanup.
    return;
  }

  if (!error_message.empty()) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Failed to cancel active goal: " + error_message);
    return;
  }

  if (!cancel_accepted) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Active goal cancel request was not accepted");
  }
}

bool FrontierExplorerCore::dispatch_pending_frontier_goal(
  const std::optional<geometry_msgs::msg::Pose> & current_pose)
{
  if (pending_frontier_sequence.empty()) {
    // Nothing queued by preemption/reselection logic.
    return false;
  }

  std::optional<geometry_msgs::msg::Pose> pose = current_pose;
  if (!pose.has_value()) {
    // Keep sequence pending if TF is transiently unavailable; caller retries later.
    pose = callbacks.get_current_pose();
    if (!pose.has_value()) {
      return true;
    }
  }

  const FrontierSequence frontier_sequence = pending_frontier_sequence;
  const std::string selection_mode = pending_frontier_selection_mode.empty() ?
    "mrtsp" : pending_frontier_selection_mode;

  const bool dispatched = send_frontier_goal(
    frontier_sequence,
    *pose,
    "Sending updated frontier goal (" + selection_mode + "): " +
    describe_frontier(frontier_sequence.front()));

  if (dispatched) {
    // Consume pending sequence only after successful dispatch to preserve at-least-once intent.
    pending_frontier_sequence.clear();
    pending_frontier_selection_mode.clear();
    pending_frontier_dispatch_context.clear();
    active_goal_blocked_reason.reset();
    reset_replacement_candidate_tracking();
    clear_post_goal_wait_state();
  }

  return true;
}

void FrontierExplorerCore::handle_exploration_complete(const geometry_msgs::msg::Pose & current_pose)
{
  if (params.dry_run) {
    // 2026-11-11: 干跑不锁存"探索完成"。离线回放刚开始时地图基本是空的,
    // 一旦走到这里就会把 return_to_start_completed 置位, 之后再也不选点
    // (现场表现为 /explore/frontiers 一直为空)。
    return;
  }

  if (params.cave_mode_enabled && cave_geometry_.has_entrance()) {
    if (mission_state_ == MissionState::COMPLETE) {
      return;
    }
    if (cave_geometry_.is_within_entrance_radius(
        current_pose.position.x, current_pose.position.y))
    {
      return_to_start_completed = true;
      set_mission_state(MissionState::COMPLETE, "returned_to_entrance");
      callbacks.on_exploration_complete();
      return;
    }
    const bool dispatched = (params.patrol_enabled && !return_requested_) ?
      start_patrolling() : start_returning();
    if (!dispatched) {
      // No dispatchable goal right now; the scheduler retries on the next tick.
      callbacks.log_warn("Unable to dispatch the cave patrol/return goal yet");
    }
    return;
  }

  callbacks.on_exploration_complete();

  if (return_to_start_completed) {
    return;
  }

  if (!params.return_to_start_on_complete || !start_pose.has_value())
  {
    // Mark completion when return-to-start is disabled (or unavailable) to stop repeated no-frontier scans.
    return_to_start_completed = true;
    return;
  }

  if (is_pose_within_xy_tolerance(current_pose, start_pose->pose)) {
    // Exploration already ended near start; avoid issuing a redundant return goal.
    return_to_start_completed = true;
    callbacks.log_info("Exploration finished at the start pose");
    return;
  }

  if (!return_to_start_started) {
    // Latch this flag before dispatch to prevent duplicate return goals on repeated callbacks.
    return_to_start_started = true;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Exploration finished, returning to start pose: ("
        << start_pose->pose.position.x << ", "
        << start_pose->pose.position.y << ")";

    send_pose_goal(
      *start_pose,
      "return_to_start",
      std::nullopt,
      {},
      oss.str());
  }
}

bool FrontierExplorerCore::is_pose_within_xy_tolerance(
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double tolerance) const
{
  // XY-only proximity check intentionally ignores yaw for exploration completion decisions.
  return std::hypot(
    goal_pose.position.x - current_pose.position.x,
    goal_pose.position.y - current_pose.position.y) <= tolerance;
}

bool FrontierExplorerCore::send_pose_goal(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  if (!callbacks.wait_for_action_server(1.0)) {
    // Keep core non-blocking: schedule retry on future updates instead of waiting indefinitely.
    callbacks.log_info(
      "Action '" + params.navigate_to_pose_action_name + "' not ready yet, waiting...");
    if (goal_kind == "return_to_start") {
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    return false;
  }

  dispatch_goal_request(
    params.navigate_to_pose_action_name,
    goal_pose,
    goal_kind,
    frontier,
    frontier_sequence,
    description);
  return true;
}

bool FrontierExplorerCore::send_frontier_goal(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & description,
  bool bypass_min_distance_dispatch)
{
  if (frontier_sequence.empty()) {
    return false;
  }

  std::size_t dispatch_index = 0U;
  for (; dispatch_index < frontier_sequence.size(); ++dispatch_index) {
    const auto & candidate_frontier = frontier_sequence[dispatch_index];
    if (!params.goal_skip_on_blocked_goal) {
      break;
    }

    const auto cost_status = frontier_cost_status(std::optional<FrontierLike>{candidate_frontier});
    if (!cost_status.has_value()) {
      break;
    }

    callbacks.log_info(
      "Skipping blocked frontier goal before dispatch: " + *cost_status +
      "; " + describe_frontier(candidate_frontier));
  }

  if (dispatch_index >= frontier_sequence.size()) {
    callbacks.log_info(
      "All selected frontier goals are blocked before dispatch; waiting for updated costmap or frontier data");
    return false;
  }

  FrontierSequence dispatch_sequence;
  dispatch_sequence.reserve(frontier_sequence.size() - dispatch_index);
  for (std::size_t index = dispatch_index; index < frontier_sequence.size(); ++index) {
    dispatch_sequence.push_back(frontier_sequence[index]);
  }

const auto goal_pose = build_dispatch_goal_pose(
      dispatch_sequence.front(),
    current_pose,
    bypass_min_distance_dispatch);
  if (debug_outputs_enabled()) {
    callbacks.publish_selected_frontier_pose(goal_pose);
  }
  const std::string dispatch_description = dispatch_index == 0U ?
    description :
    "Sending frontier goal after blocked-goal skip: " + describe_frontier(dispatch_sequence.front());
  // Frontier mode dispatches only the first element from the selected sequence.
  const bool sent = send_pose_goal(
    goal_pose,
    "frontier",
    dispatch_sequence.front(),
    dispatch_sequence,
    dispatch_description);
  if (sent) {
    active_goal_cost_ = frontier_selection_cost(dispatch_sequence.front(), current_pose);
  }
  return sent;
}

void FrontierExplorerCore::dispatch_goal_request(
  const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  if (params.dry_run) {
    // 2026-11-11 离线干跑: 只把选中的目标发布到 /explore/selected_frontier 并记日志,
    // 不进入 SENDING/ACTIVE 生命周期, 因此下一个地图 tick 会重新选点。
    callbacks.publish_selected_frontier_pose(goal_pose);
    const std::pair<double, double> goal_xy{
      goal_pose.pose.position.x, goal_pose.pose.position.y};
    const bool changed =
      !last_dry_run_goal_.has_value() ||
      std::hypot(goal_xy.first - last_dry_run_goal_->first,
                 goal_xy.second - last_dry_run_goal_->second) > 0.10;
    if (changed) {
      last_dry_run_goal_ = goal_xy;
      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss.precision(2);
      oss << "[DRY-RUN] select goal=(" << goal_xy.first << ", " << goal_xy.second << ")";
      if (frontier.has_value()) {
        oss << " " << describe_frontier(*frontier);
      }
      oss << " via " << action_name << " (" << description << ")";
      callbacks.log_info(oss.str());
    }
    return;
  }

  // Dispatch ids make out-of-order callbacks safe when old and new goals overlap.
  if (goal_in_progress && dispatch_states.find(current_dispatch_id) != dispatch_states.end()) {
    mark_dispatch_state(current_dispatch_id, GoalLifecycleState::SUPERSEDED);
  }

  if (pending_cancel_reason.has_value()) {
    // That pending cancel targeted the goal being superseded now; Nav2 replaces
    // it, so carrying the reason over would cancel the new goal on acceptance.
    pending_cancel_reason.reset();
  }

  current_dispatch_id += 1;
  const int dispatch_id = current_dispatch_id;

  active_goal_kind = goal_kind;
  active_goal_pose = goal_pose;
  active_goal_frontier = frontier;
  active_goal_frontiers = frontier_sequence;
  active_action_name = action_name;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::SENDING);
  set_goal_state(GoalLifecycleState::SENDING);
  goal_handle.reset();
  // Send timestamp anchors min-interval preemption gating.
  active_goal_sent_time_ns = callbacks.now_ns();
  last_low_gain_reselection_time_ns.reset();
  awaiting_map_refresh = false;
  callbacks.log_info(description);

  dispatch_contexts[dispatch_id] = DispatchContext{
    goal_kind,
    frontier,
    frontier_sequence,
    action_name,
  };

  callbacks.dispatch_goal_request(GoalDispatchRequest{
    dispatch_id,
    action_name,
    goal_kind,
    goal_pose,
    frontier,
    frontier_sequence,
    description,
  });
}

void FrontierExplorerCore::clear_active_goal_state()
{
  // Clears all active lifecycle fields after terminal result/cancel.
  set_goal_state(GoalLifecycleState::IDLE);
  goal_handle.reset();
  active_goal_pose.reset();
  active_goal_frontier.reset();
  active_goal_frontiers.clear();
  active_goal_kind.clear();
  active_action_name.clear();
  active_goal_sent_time_ns.reset();
  active_goal_blocked_reason.reset();
  active_goal_cost_.reset();
  last_low_gain_reselection_time_ns.reset();
  clear_active_goal_progress_state();
  dispatch_states.clear();
  dispatch_contexts.clear();
}

std::optional<FrontierExplorerCore::DispatchContext> FrontierExplorerCore::dispatch_context_for(
  int dispatch_id) const
{
  // Dispatch context is best-effort metadata; stale callbacks may legitimately miss this entry.
  auto it = dispatch_contexts.find(dispatch_id);
  if (it == dispatch_contexts.end()) {
    return std::nullopt;
  }
  return it->second;
}

void FrontierExplorerCore::goal_response_callback(
  int dispatch_id,
  const std::shared_ptr<GoalHandleInterface> & received_goal_handle,
  bool accepted,
  const std::string & error_message)
{
  if (shutdown_requested) {
    // Ignore callbacks after teardown requested.
    return;
  }

  const auto context = dispatch_context_for(dispatch_id);

  if (!accepted) {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (context.has_value() && context->goal_kind == "return_to_start") {
      return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "frontier") {
      record_failed_frontier_attempt(context->frontier);
    }
    const std::string rejected_kind = context.has_value() ? context->goal_kind : "";
    const geometry_msgs::msg::Pose rejected_pose =
      active_goal_pose.has_value() ? active_goal_pose->pose : geometry_msgs::msg::Pose{};
    clear_active_goal_state();
    on_goal_rejected(rejected_kind, rejected_pose);
    callbacks.log_error(error_message.empty() ? "Frontier goal was rejected" : error_message);
    return;
  }

  if (dispatch_id != current_dispatch_id) {
    // Late response from superseded dispatch is ignored; Nav2 handles active-goal preemption.
    return;
  }

  goal_handle = received_goal_handle;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::ACTIVE);
  set_goal_state(GoalLifecycleState::ACTIVE);
  if (context.has_value() && context->goal_kind == "return_to_start") {
    callbacks.log_info("Return-to-start goal accepted");
  } else if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
    callbacks.log_info("Temporary return-to-start goal accepted while frontiers are suppressed");
  } else {
    callbacks.log_info("Frontier goal accepted");
  }
  start_active_goal_progress_tracking();

  if (pending_cancel_reason.has_value()) {
    // If cancel was requested while sending, execute cancel immediately after accept.
    issue_active_goal_cancel();
  }
}

void FrontierExplorerCore::get_result_callback(
  int dispatch_id,
  int status,
  int error_code,
  const std::string & error_message,
  const std::string & exception_text)
{
  const auto context = dispatch_context_for(dispatch_id);
  // Cache derived values early so later cleanup cannot invalidate log/context decisions.
  const std::string goal_kind = context.has_value() ? context->goal_kind : "";
  const FrontierSequence frontier_sequence = context.has_value() ? context->frontier_sequence : FrontierSequence{};

  if (exception_text.empty()) {
    if (dispatch_id != current_dispatch_id) {
      // Result belongs to an older superseded dispatch.
      return;
    }

    if (goal_kind == "patrol_waypoint") {
      if (status != action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        on_patrol_waypoint_result(status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
      }
    } else if (goal_kind == "resume_center") {
      if (status != action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        on_resume_center_result(
          status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
      }
    } else if (goal_kind == "target_standoff") {
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        on_target_standoff_result(true);
      } else if (status != action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        on_target_standoff_result(false);
      }
    } else if (goal_kind == "return_waypoint" || goal_kind == "return_entrance") {
      if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        // A pause or preemption canceled the return goal; keep the mission state.
      } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        return_consecutive_failures_ = 0;
        if (goal_kind == "return_waypoint") {
          ++return_index_;
          dispatch_next_return_waypoint();
        } else {
          return_to_start_completed = true;
          set_mission_state(MissionState::COMPLETE, "returned");
          callbacks.on_exploration_complete();
          callbacks.log_info("Returned to the cave entrance");
        }
      } else {
        ++return_consecutive_failures_;
        if (goal_kind == "return_waypoint") {
          ++return_index_;
        }
        if (return_consecutive_failures_ >= params.return_max_consecutive_failures) {
          return_to_start_completed = true;
          set_mission_state(MissionState::COMPLETE, "return_failed");
          callbacks.on_exploration_complete();
          callbacks.log_warn("Segmented return failed repeatedly; completing in place");
        } else {
          dispatch_next_return_waypoint();
        }
      }
    } else if (goal_kind == "backtrack_junction") {
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        set_mission_state(MissionState::EXPLORING, "backtrack_arrived");
        callbacks.log_info("Backtrack junction reached");
      } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        set_mission_state(MissionState::EXPLORING, "backtrack_canceled");
      } else {
        if (active_goal_pose.has_value()) {
          junction_stack_.mark_failed(
            active_goal_pose->pose.position.x,
            active_goal_pose->pose.position.y,
            params.junction_dedup_m);
        }
        set_mission_state(MissionState::EXPLORING, "backtrack_failed");
        callbacks.log_warn("Backtrack junction goal failed");
      }
    } else if (
      status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
      goal_kind == "return_to_start")
    {
      // Return path uses dedicated completion latch consumed by scheduler.
      return_to_start_completed = true;
      callbacks.log_info("Returned to start pose");
    } else if (
      status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
      goal_kind == "suppressed_return_to_start")
    {
      callbacks.log_info("Reached start pose while frontiers remain temporarily suppressed");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      callbacks.log_info("Frontier goal reached");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      // Expected during explicit preemption/cancel paths.
      callbacks.log_debug(goal_kind + " was canceled before completion");
      if (
        goal_kind == "frontier" &&
        frontier_suppression_ &&
        frontier_suppression_->progress_timeout_cancel_requested())
      {
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
    } else {
      if (goal_kind == "return_to_start") {
        // Failed/aborted return-to-start should permit a future retry.
        return_to_start_started = false;
      } else if (goal_kind == "suppressed_return_to_start") {
        suppressed_return_to_start_started = false;
      }
      if (goal_kind == "frontier") {
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
      callbacks.log_warn(
        goal_kind + " finished with status " + detail::status_to_string(status) +
        ", error_code=" + std::to_string(error_code) +
        ", error_msg='" + error_message + "'");
    }
  } else {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (goal_kind == "return_to_start") {
      // Exceptions in result wait path should also reopen return-to-start retry path.
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    callbacks.log_error("Failed while waiting for result: " + exception_text);
  }

  if (dispatch_id != current_dispatch_id) {
    // Ignore stale results from superseded dispatches.
    return;
  }

  cancel_request_in_progress = false;
  // Cancel reason must not leak to the next goal lifecycle.
  pending_cancel_reason.reset();
  clear_active_goal_state();

  if (!exploration_enabled) {
    clear_post_goal_wait_state();
    return;
  }

  // Replacement/preemption dispatch should never be blocked by the normal post-goal cooldown.
  if (!pending_frontier_sequence.empty()) {
    if (dispatch_pending_frontier_goal()) {
      return;
    }
  }

  if (goal_kind != "frontier") {
    clear_post_goal_wait_state();
    return;
  }

  if (params.post_goal_settle_enabled) {
    start_post_goal_settle();
    return;
  }

  clear_post_goal_wait_state();
  try_send_next_goal();
}

void FrontierExplorerCore::feedback_callback(double distance_remaining, int dispatch_id)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore feedback from stale dispatches.
    return;
  }

  note_active_goal_progress(distance_remaining);

  std::ostringstream oss;
  // Keep distance precision stable to ease visual comparison in debug logs.
  oss << std::fixed << std::setprecision(3)
      << "Distance remaining: " << distance_remaining;
  callbacks.log_debug(oss.str());
}

void FrontierExplorerCore::publish_frontier_markers(const FrontierSequence & frontiers)
{
  const auto signature = frontier_signature(frontiers);
  if (last_published_frontier_signature.has_value() &&
    *last_published_frontier_signature == signature)
  {
    // Skip duplicate publish to reduce RViz/topic noise.
    return;
  }

  callbacks.publish_frontier_markers(frontiers);
  last_published_frontier_signature = signature;
}

void FrontierExplorerCore::request_shutdown()
{
  // Prevent future callback-side state transitions and release current goal handle.
  shutdown_requested = true;
  goal_handle.reset();
}

}  // namespace frontier_exploration_ros2
