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

namespace frontier_exploration_ros2
{
namespace
{

bool same_costmap_search_input(
  const std::optional<OccupancyGrid2d> & previous,
  const OccupancyGrid2d & next)
{
  if (!previous.has_value()) {
    return false;
  }

  const auto & previous_map = previous->map();
  const auto & next_map = next.map();
  const auto & previous_info = previous_map.info;
  const auto & next_info = next_map.info;

  return (
    previous_info.width == next_info.width &&
    previous_info.height == next_info.height &&
    previous_info.resolution == next_info.resolution &&
    previous_info.origin.position.x == next_info.origin.position.x &&
    previous_info.origin.position.y == next_info.origin.position.y &&
    previous_info.origin.orientation.x == next_info.origin.orientation.x &&
    previous_info.origin.orientation.y == next_info.origin.orientation.y &&
    previous_info.origin.orientation.z == next_info.origin.orientation.z &&
    previous_info.origin.orientation.w == next_info.origin.orientation.w &&
    previous_map.data == next_map.data);
}

}  // namespace

void FrontierExplorerCore::refresh_decision_map()
{
  if (!map.has_value()) {
    return;
  }

  const auto previous_decision_map_msg = decision_map_msg;
  const DecisionMapBuildStatus status = build_decision_map(
    *map,
    decision_map_config(),
    decision_map_workspace);
  decision_map_msg = decision_map_workspace.optimized_map_msg;
  if (
    !decision_map.has_value() ||
    previous_decision_map_msg != decision_map_msg ||
    status.geometry_changed)
  {
    decision_map = OccupancyGrid2d(decision_map_msg);
  }

  if (status.output_changed || !decision_map.has_value()) {
    decision_map_generation += 1;
    decision_map_cache_misses += 1;
    frontier_snapshot.reset();
    decision_map_dirty = false;
    if (debug_outputs_enabled() && decision_map_msg) {
      callbacks.publish_optimized_map(*decision_map_msg);
      callbacks.log_debug(
        "decision_map: miss, raw_generation=" + std::to_string(map_generation) +
        ", decision_generation=" + std::to_string(decision_map_generation) +
        ", optimization_enabled=" +
        std::string(frontier_map_optimization_enabled() ? "true" : "false"));
    }
    return;
  }

  decision_map_cache_hits += 1;
  decision_map_dirty = false;
  if (debug_outputs_enabled()) {
    callbacks.log_debug(
      "decision_map: hit, raw_generation=" + std::to_string(map_generation) +
      ", decision_generation=" + std::to_string(decision_map_generation) +
      ", optimization_enabled=" +
      std::string(frontier_map_optimization_enabled() ? "true" : "false"));
  }
}

bool FrontierExplorerCore::map_known_cells_reach(int threshold) const
{
  // 2026-11-11: "探索完成"的最小地图门限。启动瞬间或 SLAM 刚重置时地图可能
  // 只有几个自由格，此时没有 frontier 不代表探索结束。
  if (threshold <= 0) {
    return true;
  }
  if (!map.has_value()) {
    return false;
  }
  const auto & grid = map->map();
  if (grid.data.empty()) {
    return false;
  }
  const auto unknown = static_cast<int8_t>(OccupancyGrid2d::CostValues::NoInformation);
  std::size_t known = 0;
  for (const auto value : grid.data) {
    if (value != unknown && ++known >= static_cast<std::size_t>(threshold)) {
      return true;
    }
  }
  return false;
}

std::size_t FrontierExplorerCore::raw_unknown_boundary_cell_count(std::size_t stop_after)
{
  if (!map.has_value() || stop_after == 0U) {
    return 0U;
  }
  if (raw_boundary_map_generation == map_generation) {
    return raw_boundary_cell_count_cache;
  }

  const auto & grid = map->map();
  const auto unknown = static_cast<int8_t>(OccupancyGrid2d::CostValues::NoInformation);
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const bool skip_outside_cave = params.cave_mode_enabled && cave_geometry_.has_entrance();
  static constexpr int kNeighbourOffsets[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

  std::size_t count = 0U;
  for (int y = 0; y < height && count < stop_after; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      if (grid.data[index] != unknown) {
        continue;
      }
      if (skip_outside_cave) {
        const auto world = map->mapToWorld(x, y);
        if (cave_geometry_.is_outside_cave(world.first, world.second)) {
          continue;
        }
      }
      bool passable_neighbour = false;
      for (const auto & offset : kNeighbourOffsets) {
        const int nx = x + offset[0];
        const int ny = y + offset[1];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const int cost = static_cast<int>(
          grid.data[static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(nx)]);
        if (cost >= 0 && cost < params.occ_threshold) {
          passable_neighbour = true;
          break;
        }
      }
      if (passable_neighbour) {
        ++count;
        if (count >= stop_after) {
          break;
        }
      }
    }
  }

  raw_boundary_map_generation = map_generation;
  raw_boundary_cell_count_cache = count;
  return count;
}

void FrontierExplorerCore::ingestRawMapUpdate(const OccupancyGrid2d & map_msg)
{
  map = map_msg;
  map_generation += 1;
  decision_map_dirty = true;
}

bool FrontierExplorerCore::active_frontier_goal_in_progress() const
{
  return goal_in_progress && active_goal_kind == "frontier";
}

void FrontierExplorerCore::commit_deferred_costmap_search_input_updates()
{
  bool committed_update = false;

  if (pending_costmap_search_input_update) {
    costmap_generation += 1;
    pending_costmap_search_input_update = false;
    committed_update = true;
  }

  if (pending_local_costmap_search_input_update) {
    local_costmap_generation += 1;
    pending_local_costmap_search_input_update = false;
    committed_update = true;
  }

  if (committed_update) {
    frontier_snapshot.reset();
    raw_frontier_debug_cache.reset();
  }
}

void FrontierExplorerCore::handleUrgentRawMapUpdateForActiveGoal()
{
  // if (goal_in_progress && active_goal_kind == "frontier") {
  //   consider_preempt_active_goal("map");
  //   return;
  // }

  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
  }
}

void FrontierExplorerCore::processPendingMapUpdate()
{
  const bool had_dirty_map = decision_map_dirty;

  // if (decision_map_dirty) {
  //   refresh_decision_map();
  // }

  if (goal_in_progress) {
    if (had_dirty_map && active_goal_kind == "frontier") {
      consider_preempt_active_goal("map");
    } 
    
    else if (had_dirty_map && active_goal_kind == "suppressed_return_to_start") {
      consider_cancel_suppressed_return_to_start();
    }

    return;
  }

  if (!had_dirty_map) {
    return;
  }

  try_send_next_goal();
}

void FrontierExplorerCore::occupancyGridCallback(const OccupancyGrid2d & map_msg)
{
  ingestRawMapUpdate(map_msg);
  refresh_decision_map();
  if (goal_in_progress && active_goal_kind == "frontier") {
    // While navigating a frontier, map updates are routed through preemption policy first.
    consider_preempt_active_goal("map");
    return;
  }
  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    consider_cancel_suppressed_return_to_start();
    return;
  }
  try_send_next_goal();
}

void FrontierExplorerCore::costmapCallback(const OccupancyGrid2d & map_msg)
{
  if (active_frontier_goal_in_progress()) {
    costmap = map_msg;
    pending_costmap_search_input_update = true;

    const auto active_goal_cost_status = params.goal_skip_on_blocked_goal ?
      frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};

    if (active_goal_cost_status.has_value()) {
      active_goal_blocked_reason = *active_goal_cost_status;
      request_active_goal_cancel(*active_goal_cost_status);
    } else {
      active_goal_blocked_reason.reset();
    }

    return;
  }

  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    costmap = map_msg;
    pending_costmap_search_input_update = true;
    consider_cancel_suppressed_return_to_start();
    return;
  }

  const bool costmap_changed = !same_costmap_search_input(costmap, map_msg);
  costmap = map_msg;

  if (costmap_changed) {
    pending_costmap_search_input_update = true;
  }

  commit_deferred_costmap_search_input_updates();

  if (params.map_processing_rate_hz <= 0.0 || !decision_map_dirty) {
    try_send_next_goal();
  }
}

void FrontierExplorerCore::localCostmapCallback(const OccupancyGrid2d & map_msg)
{
  if (active_frontier_goal_in_progress()) {
    local_costmap = map_msg;
    pending_local_costmap_search_input_update = true;

    const auto active_goal_cost_status = params.goal_skip_on_blocked_goal ?
      frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};

    if (active_goal_cost_status.has_value()) {
      active_goal_blocked_reason = *active_goal_cost_status;
      request_active_goal_cancel(*active_goal_cost_status);
    } else {
      active_goal_blocked_reason.reset();
    }

    return;
  }

  if (goal_in_progress && active_goal_kind == "suppressed_return_to_start") {
    local_costmap = map_msg;
    pending_local_costmap_search_input_update = true;
    consider_cancel_suppressed_return_to_start();
    return;
  }

  const bool local_costmap_changed = !same_costmap_search_input(local_costmap, map_msg);
  local_costmap = map_msg;

  if (local_costmap_changed) {
    pending_local_costmap_search_input_update = true;
  }

  commit_deferred_costmap_search_input_updates();

  if (params.map_processing_rate_hz <= 0.0 || !decision_map_dirty) {
    try_send_next_goal();
  }
}

}  // namespace frontier_exploration_ros2
