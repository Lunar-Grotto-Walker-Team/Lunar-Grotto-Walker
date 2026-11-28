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

#include "frontier_exploration_ros2/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdint>
#include <limits>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// 8-connected neighborhood plus center cell for local scans.
constexpr int kNeighborOffsets[9][2] = {
  {-1, -1},
  {-1, 0},
  {-1, 1},
  {0, -1},
  {0, 0},
  {0, 1},
  {1, -1},
  {1, 0},
  {1, 1},
};

constexpr int classification_flag(PointClassification classification) noexcept
{
  // Enum values are bit-flags; convert once for branch-friendly bit operations.
  return static_cast<int>(classification);
}

inline bool has_classification(
  const FrontierPoint * point,
  PointClassification classification) noexcept
{
  // Standard mask test: true if the requested bit is set.
  return (point->classification & classification_flag(classification)) != 0;
}

inline void set_classification(
  FrontierPoint * point,
  PointClassification classification) noexcept
{
  // Monotonic set operation; flags are never toggled off inside one pass.
  point->classification |= classification_flag(classification);
}

template<typename Fn>
void for_each_neighbor(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache,
  Fn && fn)
{
  // Neighbor iteration stays allocation-free in hot paths.
  const int size_x = occupancy_map.getSizeX();
  const int size_y = occupancy_map.getSizeY();
  for (const auto & offset : kNeighborOffsets) {
    const int x = point->mapX + offset[0];
    const int y = point->mapY + offset[1];
    if (x < 0 || y < 0 || x >= size_x || y >= size_y) {
      continue;
    }
    fn(frontier_cache.getPoint(x, y));
  }
}

std::vector<FrontierPoint *> iter_neighbors(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  std::vector<FrontierPoint *> neighbors;
  neighbors.reserve(9);
  for_each_neighbor(point, occupancy_map, frontier_cache, [&neighbors](FrontierPoint * neighbor) {
    neighbors.push_back(neighbor);
  });
  return neighbors;
}

}  // namespace

FrontierSearchContext::FrontierSearchContext(
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierSearchOptions search_options)
: occupancy_map_(occupancy_map),
  costmap_(costmap),
  local_costmap_(local_costmap),
  size_x_(occupancy_map.getSizeX()),
  size_y_(occupancy_map.getSizeY()),
  global_costmap_aligned_(occupancy_map.isGeometryAlignedWith(costmap)),
  local_costmap_aligned_(
    local_costmap.has_value() && occupancy_map.isGeometryAlignedWith(*local_costmap)),
  search_options_(std::move(search_options))
{
  // Caches are sized once per pass and reused via generation stamps.
  const std::size_t cell_count =
    static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_);
  map_to_world_cache_.assign(cell_count, {0.0, 0.0});
  map_to_world_cache_stamp_.assign(cell_count, 0);
  global_cost_blocked_cache_.assign(cell_count, 0);
  global_cost_blocked_cache_stamp_.assign(cell_count, 0);
  local_cost_blocked_cache_.assign(cell_count, 0);
  local_cost_blocked_cache_stamp_.assign(cell_count, 0);
  frontier_eligibility_cache_.assign(cell_count, 0);
  frontier_eligibility_cache_stamp_.assign(cell_count, 0);
  accessible_cell_stamp_.assign(cell_count, 0);
  visible_frontier_cell_stamp_.assign(cell_count, 0);
}

std::size_t FrontierSearchContext::cell_index(int map_x, int map_y) const
{
  return static_cast<std::size_t>(map_y) * static_cast<std::size_t>(size_x_) +
         static_cast<std::size_t>(map_x);
}

void FrontierSearchContext::advance_generation(
  std::vector<uint32_t> & stamps,
  uint32_t & generation)
{
  // Generation bump invalidates previous cache without touching all elements.
  if (generation == std::numeric_limits<uint32_t>::max()) {
    // Rare wrap-around path: explicit clear to restore stamp invariants.
    std::fill(stamps.begin(), stamps.end(), 0);
    generation = 1;
    return;
  }
  ++generation;
}

void FrontierSearchContext::begin_candidate_accessible_scan()
{
  advance_generation(accessible_cell_stamp_, accessible_cell_generation_);
}

bool FrontierSearchContext::mark_accessible_cell_once(int map_x, int map_y)
{
  const std::size_t idx = cell_index(map_x, map_y);
  // Duplicate suppression inside one candidate-materialization pass.
  if (accessible_cell_stamp_[idx] == accessible_cell_generation_) {
    return false;
  }
  accessible_cell_stamp_[idx] = accessible_cell_generation_;
  return true;
}

void FrontierSearchContext::begin_visible_frontier_scan()
{
  advance_generation(visible_frontier_cell_stamp_, visible_frontier_cell_generation_);
}

bool FrontierSearchContext::mark_visible_frontier_cell_once(int map_x, int map_y)
{
  const std::size_t idx = cell_index(map_x, map_y);
  if (visible_frontier_cell_stamp_[idx] == visible_frontier_cell_generation_) {
    return false;
  }
  visible_frontier_cell_stamp_[idx] = visible_frontier_cell_generation_;
  return true;
}

bool FrontierSearchContext::global_cost_obstacle(int map_x, int map_y)
{
  // 0..100 是代价, 253/254 是内切/致命, 255 是 NO_INFORMATION。
  // 只有"真实障碍/高代价格"才算障碍; 未知格(255)不算。
  if (!costmap_.has_value()) {
    return false;
  }
  int cost_x = 0;
  int cost_y = 0;
  const auto world = world_point(map_x, map_y);
  if (!costmap_->worldToMapNoThrow(world.first, world.second, cost_x, cost_y)) {
    // 视口外单元格无从判断, 不当作障碍(rolling costmap 常见)。
    return false;
  }
  const int cost = costmap_->getCost(cost_x, cost_y);
  return cost != 255 && cost >= search_options_.occ_threshold;
}

bool FrontierSearchContext::goal_reachable_from(
  int map_x,
  int map_y,
  int start_map_x,
  int start_map_y)
{
  // 2026-11-06: 路径可达性检查。之前只校验 frontier 格本身在 costmap 上未被阻塞，
  // 但"格本身干净"不等于"从机器人走得到"（门洞被膨胀封死、free 区被墙隔断等情况）。
  // 这里在 map 网格上做一次 4 邻域 BFS，通行判据与候选筛选一致：
  // map 格必须是 FreeSpace 且 global costmap 未阻塞（cost < occ_threshold）。
  // local costmap 不参与：它是随机器人滚动的，远处的格子不可见会误判成不可达。
  if (map_x < 0 || map_y < 0 || map_x >= size_x_ || map_y >= size_y_ ||
    start_map_x < 0 || start_map_y < 0 ||
    start_map_x >= size_x_ || start_map_y >= size_y_)
  {
    return false;
  }

  if (
    !reachable_ready_ || reachable_start_x_ != start_map_x ||
    reachable_start_y_ != start_map_y)
  {
    reachable_from_robot_.assign(
      static_cast<std::size_t>(size_x_) * static_cast<std::size_t>(size_y_), 0);
    reachable_start_x_ = start_map_x;
    reachable_start_y_ = start_map_y;
    reachable_ready_ = true;

    const auto passable = [this](int x, int y) {
        if (x < 0 || y < 0 || x >= size_x_ || y >= size_y_) {
          return false;
        }
        if (
          occupancy_map_.getCost(x, y) !=
          static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
        {
          return false;
        }
        return !global_cost_blocked(x, y);
      };

    if (passable(start_map_x, start_map_y)) {
      std::deque<int> queue;
      const int start_idx = static_cast<int>(cell_index(start_map_x, start_map_y));
      reachable_from_robot_[static_cast<std::size_t>(start_idx)] = 1;
      queue.push_back(start_idx);
      while (!queue.empty()) {
        const int idx = queue.front();
        queue.pop_front();
        const int x = idx % size_x_;
        const int y = idx / size_x_;
        const int neighbors[4][2] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
        for (const auto & nb : neighbors) {
          if (!passable(nb[0], nb[1])) {
            continue;
          }
          const std::size_t nb_idx = cell_index(nb[0], nb[1]);
          if (reachable_from_robot_[nb_idx] != 0) {
            continue;
          }
          reachable_from_robot_[nb_idx] = 1;
          queue.push_back(static_cast<int>(nb_idx));
        }
      }
    }
  }

  return reachable_from_robot_[cell_index(map_x, map_y)] != 0;
}

std::pair<double, double> FrontierSearchContext::world_point(int map_x, int map_y)
{
  // map_to_world_cache_[i] is valid iff stamp[i] == current generation.
  const std::size_t idx = cell_index(map_x, map_y);
  if (map_to_world_cache_stamp_[idx] == map_to_world_generation_) {
    return map_to_world_cache_[idx];
  }

  map_to_world_cache_stamp_[idx] = map_to_world_generation_;
  map_to_world_cache_[idx] = occupancy_map_.mapToWorld(map_x, map_y);
  return map_to_world_cache_[idx];
}

bool FrontierSearchContext::global_cost_blocked(int map_x, int map_y)
{
  // Cost-blocked cache stores bool-as-byte; stamp controls validity.
  const std::size_t idx = cell_index(map_x, map_y);
  if (global_cost_blocked_cache_stamp_[idx] == global_cost_blocked_generation_) {
    return global_cost_blocked_cache_[idx] != 0;
  }

  const bool blocked = is_cost_blocked(costmap_, global_costmap_aligned_, map_x, map_y);
  global_cost_blocked_cache_stamp_[idx] = global_cost_blocked_generation_;
  global_cost_blocked_cache_[idx] = blocked ? 1 : 0;
  return blocked;
}

bool FrontierSearchContext::local_cost_blocked(int map_x, int map_y)
{
  const std::size_t idx = cell_index(map_x, map_y);
  if (local_cost_blocked_cache_stamp_[idx] == local_cost_blocked_generation_) {
    return local_cost_blocked_cache_[idx] != 0;
  }

  const bool blocked = is_cost_blocked(local_costmap_, local_costmap_aligned_, map_x, map_y);
  local_cost_blocked_cache_stamp_[idx] = local_cost_blocked_generation_;
  local_cost_blocked_cache_[idx] = blocked ? 1 : 0;
  return blocked;
}

bool FrontierSearchContext::is_cost_blocked(
  const std::optional<OccupancyGrid2d> & costmap,
  bool aligned_with_occupancy,
  int map_x,
  int map_y)
{
  if (!costmap.has_value()) {
    // Missing costmap is treated as "no additional blocking information".
    return false;
  }

  if (aligned_with_occupancy) {
    // Fast path when occupancy and costmap share geometry.
    if (
      map_x < 0 || map_y < 0 ||
      map_x >= costmap->getSizeX() || map_y >= costmap->getSizeY())
    {
      return true;
    }
    return costmap->getCost(map_x, map_y) >= search_options_.occ_threshold;
  }

  const auto world = world_point(map_x, map_y);
  // Fallback for mismatched map geometry: map cell -> world -> costmap cell.
  int cost_x = 0;
  int cost_y = 0;
  if (!costmap->worldToMapNoThrow(world.first, world.second, cost_x, cost_y)) {
    return true;
  }
  return costmap->getCost(cost_x, cost_y) >= search_options_.occ_threshold;
}

std::pair<double, double> centroid(const std::vector<std::pair<double, double>> & arr)
{
  // Centroid equation:
  //   cx = (1/N) * sum(x_i), cy = (1/N) * sum(y_i)
  const std::size_t length = arr.size();
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & point : arr) {
    sum_x += point.first;
    sum_y += point.second;
  }
  return {sum_x / static_cast<double>(length), sum_y / static_cast<double>(length)};
}

double squared_distance(
  const std::pair<double, double> & first_point,
  const std::pair<double, double> & second_point)
{
  // d^2 = (x1 - x2)^2 + (y1 - y2)^2
  // Using squared distance avoids sqrt in ranking comparisons.
  const double dx = first_point.first - second_point.first;
  const double dy = first_point.second - second_point.second;
  return (dx * dx) + (dy * dy);
}

std::optional<int> world_point_cost(
  const std::optional<OccupancyGrid2d> & costmap,
  const std::pair<double, double> & world_point)
{
  if (!costmap.has_value()) {
    return std::nullopt;
  }

  int costmap_x = 0;
  int costmap_y = 0;
  if (!costmap->worldToMapNoThrow(world_point.first, world_point.second, costmap_x, costmap_y)) {
    return std::nullopt;
  }
  return costmap->getCost(costmap_x, costmap_y);
}

bool is_world_point_blocked(
  const std::vector<std::optional<OccupancyGrid2d>> & costmaps,
  const std::pair<double, double> & world_point,
  int occ_threshold)
{
  // Multi-costmap policy: blocked if any map reports occupied over threshold.
  for (const auto & costmap : costmaps) {
    const auto cost = world_point_cost(costmap, world_point);
    if (cost.has_value() && *cost > occ_threshold) {
      return true;
    }
  }

  return false;
}

std::optional<std::pair<double, double>> choose_accessible_frontier_goal(
  const std::pair<double, double> & frontier_centroid,
  const std::vector<std::pair<int, int>> & accessible_cells,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<geometry_msgs::msg::Pose> & current_pose,
  double min_robot_distance)
{
  if (accessible_cells.empty()) {
    return std::nullopt;
  }

  // When a minimum robot distance is requested, prefer the closest far-enough goal.
  const bool apply_min_distance = current_pose.has_value() && min_robot_distance > 0.0;
  const double min_robot_distance_sq = min_robot_distance * min_robot_distance;

  std::optional<std::pair<double, double>> best_any_point;
  // Distances are squared to avoid sqrt while preserving ordering.
  double best_any_distance = std::numeric_limits<double>::infinity();
  std::optional<std::pair<double, double>> best_far_point;
  double best_far_distance = std::numeric_limits<double>::infinity();

  for (const auto & [map_x, map_y] : accessible_cells) {
    const auto world = occupancy_map.mapToWorld(map_x, map_y);
    const double centroid_distance = squared_distance(world, frontier_centroid);
    if (centroid_distance < best_any_distance) {
      best_any_distance = centroid_distance;
      best_any_point = world;
    }

    if (apply_min_distance) {
      const double robot_dx = world.first - current_pose->position.x;
      const double robot_dy = world.second - current_pose->position.y;
      if ((robot_dx * robot_dx) + (robot_dy * robot_dy) >= min_robot_distance_sq &&
        centroid_distance < best_far_distance)
      {
        // Keep the closest point to the centroid among points outside the robot-distance exclusion radius.
        best_far_distance = centroid_distance;
        best_far_point = world;
      }
    }
  }

  if (best_far_point.has_value()) {
    // Prefer far-enough candidate when min-distance gating is active.
    return best_far_point;
  }

  // Otherwise use best unconstrained candidate.
  return best_any_point;
}

namespace
{

// 2026-11-06: 一个格能否作为导航 goal 的落点。
// 与旧逻辑保持同一套通行判据（map free + global costmap 未阻塞），
// 另外可叠加 clearance 邻域检查和"从机器人真的走得到"的 BFS 检查。
enum class GoalRejectReason
{
  None,
  Blocked,
  Clearance,
  Reachability,
};

bool goal_cell_acceptable(
  int map_x,
  int map_y,
  const OccupancyGrid2d & occupancy_map,
  FrontierSearchContext * context,
  const FrontierSearchOptions & options,
  const std::pair<int, int> & start_cell,
  GoalRejectReason * reason = nullptr)
{
  if (reason != nullptr) {
    *reason = GoalRejectReason::None;
  }
  if (
    map_x < 0 || map_y < 0 || map_x >= occupancy_map.getSizeX() ||
    map_y >= occupancy_map.getSizeY())
  {
    return false;
  }
  if (
    occupancy_map.getCost(map_x, map_y) !=
    static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
  {
    if (reason != nullptr) {
      *reason = GoalRejectReason::Blocked;
    }
    return false;
  }
  if (context->global_cost_blocked(map_x, map_y)) {
    if (reason != nullptr) {
      *reason = GoalRejectReason::Blocked;
    }
    return false;
  }

  if (options.goal_clearance_check_enabled && options.goal_clearance_radius_cells > 0) {
    const int radius = options.goal_clearance_radius_cells;
    for (int y = map_y - radius; y <= map_y + radius; ++y) {
      for (int x = map_x - radius; x <= map_x + radius; ++x) {
        if (
          x < 0 || y < 0 || x >= occupancy_map.getSizeX() ||
          y >= occupancy_map.getSizeY())
        {
          // 贴着地图边缘的点不作为回退落点：无法确认外侧是否可通行。
          if (reason != nullptr) {
            *reason = GoalRejectReason::Clearance;
          }
          return false;
        }
        if (
          occupancy_map.getCost(x, y) ==
          static_cast<int>(OccupancyGrid2d::CostValues::LethalObstacle))
        {
          if (reason != nullptr) {
            *reason = GoalRejectReason::Clearance;
          }
          return false;
        }
        // 未知格(255)不算障碍: goal 本来就贴着未知区, 否则所有候选都会被否掉。
        if (context->global_cost_obstacle(x, y)) {
          if (reason != nullptr) {
            *reason = GoalRejectReason::Clearance;
          }
          return false;
        }
      }
    }
  }

  if (
    options.goal_reachability_check_enabled &&
    !context->goal_reachable_from(map_x, map_y, start_cell.first, start_cell.second))
  {
    if (reason != nullptr) {
      *reason = GoalRejectReason::Reachability;
    }
    return false;
  }

  return true;
}

// 把候选 goal 沿"远离未知 / 靠近机器人"的方向推开一段距离，再吸附到最近的
// 合格格。回退失败时退回原始 goal（只要它自身通过检查）。
std::optional<std::pair<double, double>> adjust_candidate_goal_point(
  const std::pair<double, double> & goal_point,
  const geometry_msgs::msg::Pose & current_pose,
  const OccupancyGrid2d & occupancy_map,
  FrontierSearchContext * context,
  const FrontierSearchOptions & options,
  const std::pair<int, int> & start_cell,
  GoalRejectReason * original_reject_reason = nullptr,
  bool * backoff_used = nullptr)
{
  if (backoff_used != nullptr) {
    *backoff_used = false;
  }
  const auto original_ok = [&]() -> std::optional<std::pair<double, double>> {
      int map_x = 0;
      int map_y = 0;
      if (!occupancy_map.worldToMapNoThrow(goal_point.first, goal_point.second, map_x, map_y)) {
        if (original_reject_reason != nullptr) {
          *original_reject_reason = GoalRejectReason::Blocked;
        }
        return std::nullopt;
      }
      if (!goal_cell_acceptable(
          map_x, map_y, occupancy_map, context, options, start_cell,
          original_reject_reason))
      {
        return std::nullopt;
      }
      if (original_reject_reason != nullptr) {
        *original_reject_reason = GoalRejectReason::None;
      }
      return goal_point;
    };

  if (options.goal_backoff_from_frontier_m <= 0.0) {
    // 只做检查，不回退。
    return original_ok();
  }

  double dir_x = goal_point.first - current_pose.position.x;
  double dir_y = goal_point.second - current_pose.position.y;
  const double norm = std::hypot(dir_x, dir_y);
  if (norm < 1e-6) {
    return original_ok();
  }
  dir_x /= norm;
  dir_y /= norm;

  // 回退不能把落点推到机器人身后：距离机器人至少留 candidate_min_goal_distance
  // （默认 0.5 m）的余量，否则 goal 会跑到狗背后，规划反而更容易失败。
  const double min_hold_m =
    std::max(0.5, options.candidate_min_goal_distance_m);
  const double backoff =
    std::min(options.goal_backoff_from_frontier_m, std::max(0.0, norm - min_hold_m));
  if (backoff <= 1e-6) {
    return original_ok();
  }

  // dir 是"机器人 -> goal"的方向；回退是往机器人方向走，所以取负号。
  const double desired_x = goal_point.first - dir_x * backoff;
  const double desired_y = goal_point.second - dir_y * backoff;

  int desired_map_x = 0;
  int desired_map_y = 0;
  if (!occupancy_map.worldToMapNoThrow(desired_x, desired_y, desired_map_x, desired_map_y)) {
    return original_ok();
  }

  const std::pair<double, double> desired_point{desired_x, desired_y};
  const int radius = std::max(0, options.goal_backoff_search_radius_cells);
  std::optional<std::pair<double, double>> best_point;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int map_x = desired_map_x + dx;
      const int map_y = desired_map_y + dy;
      if (!goal_cell_acceptable(map_x, map_y, occupancy_map, context, options, start_cell)) {
        continue;
      }
      const auto world = occupancy_map.mapToWorld(map_x, map_y);
      const double distance_sq = squared_distance(world, desired_point);
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point = world;
      }
    }
  }

  if (best_point.has_value()) {
    // 回退点不能比原 goal 更靠近未知区：至少要和原 goal 保持同等距离。
    if (backoff_used != nullptr) {
      *backoff_used = true;
    }
    return best_point;
  }

  return original_ok();
}

}  // namespace

std::optional<FrontierCandidate> build_frontier_candidate(
  const std::vector<FrontierPoint *> & new_frontier,
  const std::pair<int, int> & start_cell,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  const geometry_msgs::msg::Pose & current_pose,
  double min_goal_distance,
  const FrontierSearchOptions & options,
  FrontierSearchContext * search_context,
  const std::pair<int, int> & robot_cell,
  FrontierSearchStats * stats)
{
  const int min_frontier_size = std::max(1, options.min_frontier_size_cells);
  if (new_frontier.size() < static_cast<std::size_t>(min_frontier_size)) {
    // Reject tiny clusters; they are often noise at unknown/free boundaries.
    if (stats != nullptr) {
      ++stats->rejected_small;
    }
    return std::nullopt;
  }

  FrontierSearchContext * context = search_context;
  std::optional<FrontierSearchContext> owned_context;
  if (context == nullptr) {
    if (!costmap.has_value()) {
      // Candidate construction depends on cost checks; no global costmap means no candidate.
      return std::nullopt;
    }
    owned_context.emplace(occupancy_map, *costmap, local_costmap, options);
    context = &(*owned_context);
  }

  // Frontier geometry is computed once here so dispatch, preemption, and MRTSP scoring
  // all share one candidate model.
  std::vector<std::pair<double, double>> frontier_world_points;
  frontier_world_points.reserve(new_frontier.size());
  double centroid_sum_x = 0.0;
  double centroid_sum_y = 0.0;
  for (auto * frontier_point : new_frontier) {
    const auto world = context->world_point(frontier_point->mapX, frontier_point->mapY);
    frontier_world_points.push_back(world);
    centroid_sum_x += world.first;
    centroid_sum_y += world.second;
  }

  const std::pair<double, double> frontier_centroid{
    centroid_sum_x / static_cast<double>(new_frontier.size()),
    centroid_sum_y / static_cast<double>(new_frontier.size()),
  };

  int min_frontier_x = new_frontier.front()->mapX;
  int max_frontier_x = new_frontier.front()->mapX;
  int min_frontier_y = new_frontier.front()->mapY;
  int max_frontier_y = new_frontier.front()->mapY;
  for (auto * frontier_point : new_frontier) {
    min_frontier_x = std::min(min_frontier_x, frontier_point->mapX);
    max_frontier_x = std::max(max_frontier_x, frontier_point->mapX);
    min_frontier_y = std::min(min_frontier_y, frontier_point->mapY);
    max_frontier_y = std::max(max_frontier_y, frontier_point->mapY);
  }
  const FrontierCandidate::CellBounds visible_reveal_bounds{
    std::max(0, min_frontier_x - 1),
    std::max(0, min_frontier_y - 1),
    std::min(occupancy_map.getSizeX() - 1, max_frontier_x + 1),
    std::min(occupancy_map.getSizeY() - 1, max_frontier_y + 1),
  };

  std::pair<int, int> center_cell = {new_frontier.front()->mapX, new_frontier.front()->mapY};
  std::pair<double, double> center_point = frontier_world_points.front();
  FrontierPoint * center_frontier_point = new_frontier.front();
  double center_distance_sq = std::numeric_limits<double>::infinity();
  constexpr double kCenterTieEpsilon = 1e-12;
  for (std::size_t i = 0; i < new_frontier.size(); ++i) {
    const double distance_sq = squared_distance(frontier_world_points[i], frontier_centroid);
    const std::pair<int, int> candidate_cell{new_frontier[i]->mapX, new_frontier[i]->mapY};
    const bool lexicographically_earlier =
      std::pair<int, int>{candidate_cell.second, candidate_cell.first} <
      std::pair<int, int>{center_cell.second, center_cell.first};
    if (
      distance_sq + kCenterTieEpsilon < center_distance_sq ||
      (std::abs(distance_sq - center_distance_sq) <= kCenterTieEpsilon && lexicographically_earlier))
    {
      center_distance_sq = distance_sq;
      center_cell = candidate_cell;
      center_point = frontier_world_points[i];
      center_frontier_point = new_frontier[i];
    }
  }

  std::optional<double> robot_center_distance_m;
  if (center_frontier_point->robot_distance_m >= 0.0) {
    robot_center_distance_m = center_frontier_point->robot_distance_m;
  }

  const auto start_world_point = context->world_point(start_cell.first, start_cell.second);
  const double effective_candidate_min_goal_distance = std::max(
    min_goal_distance,
    options.candidate_min_goal_distance_m);
  const bool apply_min_goal_distance = effective_candidate_min_goal_distance > 0.0;
  const double min_goal_distance_sq =
    effective_candidate_min_goal_distance * effective_candidate_min_goal_distance;

  context->begin_candidate_accessible_scan();

  std::optional<std::pair<double, double>> best_any_goal_point;
  std::optional<std::pair<double, double>> best_far_goal_point;
  double best_any_distance_sq = std::numeric_limits<double>::infinity();
  double best_far_distance_sq = std::numeric_limits<double>::infinity();

  for (auto * frontier_point : new_frontier) {
    // Candidate goal points are chosen from free, unblocked neighbors around frontier cells.
    for_each_neighbor(frontier_point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (!context->mark_accessible_cell_once(neighbor->mapX, neighbor->mapY)) {
        // Same accessible neighbor can be touched by multiple frontier cells.
        return;
      }

      if (
        occupancy_map.getCost(neighbor->mapX, neighbor->mapY) !=
        static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
      {
        // Only free neighbors are admissible navigation endpoints.
        return;
      }

      if (
        context->global_cost_blocked(neighbor->mapX, neighbor->mapY))
      {
        // A blocked neighbor cannot be used as frontier goal candidate.
        return;
      }

      const auto world = context->world_point(neighbor->mapX, neighbor->mapY);
      const double centroid_distance_sq = squared_distance(world, frontier_centroid);
      if (centroid_distance_sq < best_any_distance_sq) {
        best_any_distance_sq = centroid_distance_sq;
        best_any_goal_point = world;
      }

      if (apply_min_goal_distance) {
        // Goal-distance gating uses squared radius:
        //   (x - xr)^2 + (y - yr)^2 >= min_goal_distance^2
        const double dx = world.first - current_pose.position.x;
        const double dy = world.second - current_pose.position.y;
        const double robot_distance_sq = (dx * dx) + (dy * dy);
        if (robot_distance_sq >= min_goal_distance_sq && centroid_distance_sq < best_far_distance_sq) {
          best_far_distance_sq = centroid_distance_sq;
          best_far_goal_point = world;
        }
      }
    });
  }

  std::optional<std::pair<double, double>> goal_point =
    best_far_goal_point.has_value() ? best_far_goal_point : best_any_goal_point;
  if (!goal_point.has_value()) {
    // Cluster has no reachable free neighbor after cost filtering.
    if (stats != nullptr) {
      ++stats->rejected_no_goal;
    }
    return std::nullopt;
  }

  // 2026-11-06: goal 原来紧贴已知/未知交界（frontier 的 free 邻格）。
  // 开回退 / clearance / 可达性后，把它推进已知自由区，并把不满足条件的
  // frontier 直接丢弃，避免派发注定失败的导航目标。
  if (
    options.goal_backoff_from_frontier_m > 0.0 ||
    options.goal_clearance_check_enabled ||
    options.goal_reachability_check_enabled)
  {
    GoalRejectReason reject_reason = GoalRejectReason::None;
    bool backoff_used = false;
    const auto adjusted = adjust_candidate_goal_point(
      *goal_point, current_pose, occupancy_map, context, options,
      robot_cell.first >= 0 ? robot_cell : start_cell,
      &reject_reason, &backoff_used);
    if (!adjusted.has_value()) {
      if (stats != nullptr) {
        switch (reject_reason) {
          case GoalRejectReason::Clearance:
            ++stats->rejected_clearance;
            break;
          case GoalRejectReason::Reachability:
            ++stats->rejected_reachability;
            break;
          case GoalRejectReason::Blocked:
          case GoalRejectReason::None:
          default:
            ++stats->rejected_blocked;
            break;
        }
      }
      return std::nullopt;
    }
    if (backoff_used && stats != nullptr) {
      ++stats->backoff_saved;
    }
    goal_point = adjusted;
  }

  return FrontierCandidate{
    frontier_centroid,
    center_point,
    center_cell,
    start_cell,
    start_world_point,
    *goal_point,
    static_cast<int>(new_frontier.size()),
    visible_reveal_bounds,
    robot_center_distance_m,
  };
}

std::pair<int, int> find_free(int mx, int my, const OccupancyGrid2d & occupancy_map)
{
  FrontierCache frontier_cache(occupancy_map.getSizeX(), occupancy_map.getSizeY());
  return find_free_with_cache(mx, my, occupancy_map, frontier_cache);
}

std::pair<int, int> find_free_with_cache(
  int mx,
  int my,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  // If robot starts in unknown/occupied cell, recover the closest reachable free seed for BFS.
  std::deque<FrontierPoint *> bfs;
  bfs.push_back(frontier_cache.getPoint(mx, my));

  while (!bfs.empty()) {
    FrontierPoint * location = bfs.front();
    bfs.pop_front();

    if (occupancy_map.getCost(location->mapX, location->mapY) == static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
      // First free hit in BFS radius is the closest by grid distance.
      return {location->mapX, location->mapY};
    }

    for_each_neighbor(location, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (!has_classification(neighbor, PointClassification::MapClosed)) {
        // MapClosed acts as visited marker for this recovery BFS.
        set_classification(neighbor, PointClassification::MapClosed);
        bfs.push_back(neighbor);
      }
    });
  }

  return {mx, my};
}

FrontierSearchResult get_frontier(
  const geometry_msgs::msg::Pose & current_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double min_goal_distance,
  bool return_robot_cell,
  const FrontierSearchOptions & options,
  FrontierSearchStats * stats)
{
  (void)return_robot_cell;

  FrontierSearchStats local_stats;

  // Two-level BFS:
  // 1) map queue expands navigable map area
  // 2) frontier queue grows each connected frontier cluster
  FrontierCache frontier_cache(occupancy_map.getSizeX(), occupancy_map.getSizeY());
  FrontierSearchContext search_context(occupancy_map, costmap, local_costmap, options);

  const auto [mx, my] = occupancy_map.worldToMap(current_pose.position.x, current_pose.position.y);
  // If robot projects into unknown space, find_free_with_cache nudges start to the closest free seed.
  const auto free_point = find_free_with_cache(mx, my, occupancy_map, frontier_cache);
  FrontierPoint * start = frontier_cache.getPoint(free_point.first, free_point.second);
  // Seed map BFS from a guaranteed free (or best effort) cell.
  start->classification = classification_flag(PointClassification::MapOpen);
  start->robot_distance_m = 0.0;
  const double resolution_m = occupancy_map.map().info.resolution;

  std::deque<FrontierPoint *> map_point_queue;
  map_point_queue.push_back(start);
  std::vector<FrontierCandidate> frontiers;

  while (!map_point_queue.empty()) {
    FrontierPoint * point = map_point_queue.front();
    map_point_queue.pop_front();

    if (has_classification(point, PointClassification::MapClosed)) {
      // Already fully processed from map queue.
      continue;
    }

      if (is_frontier_point(
        point,
        occupancy_map,
        costmap,
        local_costmap,
        frontier_cache,
        &search_context))
    {
      // Collect one connected frontier cluster.
      set_classification(point, PointClassification::FrontierOpen);
      const std::pair<int, int> frontier_start{point->mapX, point->mapY};
      std::deque<FrontierPoint *> frontier_queue;
      frontier_queue.push_back(point);
      std::vector<FrontierPoint *> new_frontier;

      while (!frontier_queue.empty()) {
        FrontierPoint * candidate = frontier_queue.front();
        frontier_queue.pop_front();

        if (
          has_classification(candidate, PointClassification::MapClosed) ||
          has_classification(candidate, PointClassification::FrontierClosed))
        {
          // Candidate already consumed by map or frontier phase.
          continue;
        }

        if (is_frontier_point(
            candidate,
            occupancy_map,
            costmap,
            local_costmap,
            frontier_cache,
            &search_context))
        {
          // Frontier BFS only expands cells that remain frontier-eligible under current maps.
          new_frontier.push_back(candidate);

          for_each_neighbor(candidate, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
            if (
              !has_classification(neighbor, PointClassification::FrontierOpen) &&
              !has_classification(neighbor, PointClassification::FrontierClosed) &&
              !has_classification(neighbor, PointClassification::MapClosed))
            {
              if (candidate->robot_distance_m >= 0.0) {
                neighbor->robot_distance_m = candidate->robot_distance_m + resolution_m;
              }
              // FrontierOpen marks queued frontier candidates.
              set_classification(neighbor, PointClassification::FrontierOpen);
              frontier_queue.push_back(neighbor);
            }
          });
        }

        // FrontierClosed marks node as fully processed in this cluster-growth phase.
        set_classification(candidate, PointClassification::FrontierClosed);
      }

      for (auto * frontier_point : new_frontier) {
        // Prevent re-expansion from map queue once frontier cells are consumed.
        set_classification(frontier_point, PointClassification::MapClosed);
      }

      ++local_stats.clusters;
      const auto frontier_candidate = build_frontier_candidate(
        new_frontier,
        frontier_start,
        occupancy_map,
        costmap,
        local_costmap,
        frontier_cache,
        current_pose,
        min_goal_distance,
        options,
        &search_context,
        free_point,
        &local_stats);

      if (frontier_candidate.has_value()) {
        ++local_stats.candidates;
        // Preserve discovery order to keep output deterministic for downstream policy.
        frontiers.push_back(*frontier_candidate);
      }
    }

    // Continue map-space BFS through cells adjacent to free space.
    for_each_neighbor(point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
      if (
        !has_classification(neighbor, PointClassification::MapOpen) &&
        !has_classification(neighbor, PointClassification::MapClosed))
      {
        bool has_free_neighbor = false;
        for_each_neighbor(neighbor, occupancy_map, frontier_cache, [&](FrontierPoint * candidate) {
          if (
            occupancy_map.getCost(candidate->mapX, candidate->mapY) ==
            static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
          {
            has_free_neighbor = true;
          }
        });

        if (has_free_neighbor) {
          if (point->robot_distance_m >= 0.0) {
            neighbor->robot_distance_m = point->robot_distance_m + resolution_m;
          }
          // MapOpen marks node eligible for future map-queue expansion.
          set_classification(neighbor, PointClassification::MapOpen);
          map_point_queue.push_back(neighbor);
        }
      }
    });

    // MapClosed marks map-queue completion for this point.
    set_classification(point, PointClassification::MapClosed);
  }

  local_stats.candidates = frontiers.size();
  FrontierSearchResult result;
  result.frontiers = std::move(frontiers);
  result.robot_map_cell = {mx, my};
  result.stats = local_stats;
  if (stats != nullptr) {
    *stats = local_stats;
  }
  return result;
}

std::vector<FrontierPoint *> get_neighbors(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  FrontierCache & frontier_cache)
{
  return iter_neighbors(point, occupancy_map, frontier_cache);
}

bool is_frontier_point(
  FrontierPoint * point,
  const OccupancyGrid2d & occupancy_map,
  const std::optional<OccupancyGrid2d> & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  FrontierCache & frontier_cache,
  FrontierSearchContext * search_context)
{
  FrontierSearchContext * context = search_context;
  std::optional<FrontierSearchContext> owned_context;
  if (context == nullptr) {
    if (!costmap.has_value()) {
      return false;
    }
    owned_context.emplace(occupancy_map, *costmap, local_costmap, FrontierSearchOptions{});
    context = &(*owned_context);
  }

  // Frontier eligibility is cached per cell for the current search generation.
  const std::size_t cache_idx = context->cell_index(point->mapX, point->mapY);
  const auto cache_frontier_eligibility = [context, cache_idx](bool eligible) {
      context->frontier_eligibility_cache_stamp_[cache_idx] = context->frontier_eligibility_generation_;
      context->frontier_eligibility_cache_[cache_idx] = eligible ? 1 : 0;
    };

  if (context->frontier_eligibility_cache_stamp_[cache_idx] == context->frontier_eligibility_generation_) {
    // Fast path: frontier eligibility was already computed in this generation.
    return context->frontier_eligibility_cache_[cache_idx] != 0;
  }

  if (occupancy_map.getCost(point->mapX, point->mapY) != static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
    // Frontier candidates must be unknown cells bordering known free space.
    cache_frontier_eligibility(false);
    return false;
  }

  // A frontier point must touch free map space and must not be blocked by costmaps.
  bool blocked_neighbor = false;
  bool has_free_neighbor = false;
  for_each_neighbor(point, occupancy_map, frontier_cache, [&](FrontierPoint * neighbor) {
    if (blocked_neighbor) {
      return;
    }

    const int map_cost = occupancy_map.getCost(neighbor->mapX, neighbor->mapY);

    if (context->global_cost_blocked(neighbor->mapX, neighbor->mapY)) {
      // Any blocked adjacent cell disqualifies this frontier candidate.
      blocked_neighbor = true;
      return;
    }

    if (map_cost == static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
      has_free_neighbor = true;
    }
  });

  if (blocked_neighbor) {
    // Blocked adjacency takes precedence over free-neighbor condition.
    cache_frontier_eligibility(false);
    return false;
  }

  cache_frontier_eligibility(has_free_neighbor);
  return has_free_neighbor;
}

std::optional<VisibleRevealGain> compute_visible_reveal_gain(
  const geometry_msgs::msg::Pose & sensor_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & costmap,
  const std::optional<OccupancyGrid2d> & local_costmap,
  double range_m,
  double fov_deg,
  double ray_step_deg,
  const std::optional<FrontierCandidate::CellBounds> & visible_reveal_bounds)
{
  // Invalid geometry means the caller should skip this optimization and keep the fallback path.
  if (range_m < 0.1 || fov_deg < 1.0 || fov_deg > 360.0 || ray_step_deg < 0.25 || ray_step_deg > 45.0) {
    return std::nullopt;
  }

  int origin_x = 0;
  int origin_y = 0;
  // Gain is defined only for target poses that land on the known occupancy map.
  if (!occupancy_map.worldToMapNoThrow(sensor_pose.position.x, sensor_pose.position.y, origin_x, origin_y)) {
    return std::nullopt;
  }

  FrontierCache frontier_cache(occupancy_map.getSizeX(), occupancy_map.getSizeY());
  FrontierSearchContext search_context(occupancy_map, costmap, local_costmap);
  // Dedup stamps keep multiple rays from counting the same frontier cell more than once.
  search_context.begin_visible_frontier_scan();

  const double resolution = occupancy_map.map().info.resolution;
  // March slightly sub-cell to avoid skipping thin structures due to coarse whole-cell stepping.
  const double step_distance_m = std::max(resolution * 0.5, 1e-6);
  const double heading = std::atan2(
    2.0 * (
      sensor_pose.orientation.w * sensor_pose.orientation.z +
      sensor_pose.orientation.x * sensor_pose.orientation.y),
    1.0 - 2.0 * (
      sensor_pose.orientation.y * sensor_pose.orientation.y +
      sensor_pose.orientation.z * sensor_pose.orientation.z));
  const int ray_count = std::max(1, static_cast<int>(std::ceil(fov_deg / ray_step_deg)));
  const double fov_rad = fov_deg * (kPi / 180.0);

  int visible_reveal_cell_count = 0;
  for (int ray_index = 0; ray_index < ray_count; ++ray_index) {
    double ray_angle = heading;
    // 360-degree scans distribute evenly around the pose; partial FOV scans sweep the sector edges.
    if (fov_deg >= 360.0) {
      const double full_step = (2.0 * kPi) / static_cast<double>(ray_count);
      ray_angle = heading + full_step * static_cast<double>(ray_index);
    } else if (ray_count > 1) {
      const double ray_step_rad = fov_rad / static_cast<double>(ray_count - 1);
      ray_angle = heading - (fov_rad * 0.5) + ray_step_rad * static_cast<double>(ray_index);
    }

    int last_map_x = origin_x;
    int last_map_y = origin_y;
    for (double distance_m = step_distance_m; distance_m <= range_m; distance_m += step_distance_m) {
      const double sample_x = sensor_pose.position.x + std::cos(ray_angle) * distance_m;
      const double sample_y = sensor_pose.position.y + std::sin(ray_angle) * distance_m;

      int map_x = 0;
      int map_y = 0;
      if (!occupancy_map.worldToMapNoThrow(sample_x, sample_y, map_x, map_y)) {
        // Leaving the map bounds is equivalent to exhausting that ray.
        break;
      }
      if (visible_reveal_bounds.has_value() &&
        (map_x < visible_reveal_bounds->min_x ||
        map_x > visible_reveal_bounds->max_x ||
        map_y < visible_reveal_bounds->min_y ||
        map_y > visible_reveal_bounds->max_y))
      {
        // Treat the local frontier-cluster bounds as a virtual wall for visible-gain estimation.
        break;
      }

      if (map_x == last_map_x && map_y == last_map_y) {
        continue;
      }
      last_map_x = map_x;
      last_map_y = map_y;

      const int map_cost = occupancy_map.getCost(map_x, map_y);
      if (map_cost > OCC_THRESHOLD) {
        // Occupied map cells terminate line of sight; anything behind them is ignored.
        break;
      }

      if (map_cost != static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
        continue;
      }

      // Only the first unknown cell reached by a ray can contribute gain, and only if it
      // is a true frontier under the same map/costmap rules used by the main search.
      FrontierPoint * point = frontier_cache.getPoint(map_x, map_y);
      if (is_frontier_point(
          point,
          occupancy_map,
          costmap,
          local_costmap,
          frontier_cache,
          &search_context) &&
        search_context.mark_visible_frontier_cell_once(map_x, map_y))
      {
        visible_reveal_cell_count += 1;
      }

      break;
    }
  }

  return VisibleRevealGain{
    visible_reveal_cell_count,
    // Convert deduplicated reveal cells into a map-resolution-scaled length estimate.
    static_cast<double>(visible_reveal_cell_count) * resolution,
  };
}

}  // namespace frontier_exploration_ros2
