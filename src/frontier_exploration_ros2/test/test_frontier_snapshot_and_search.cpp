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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Validates frontier extraction primitives plus snapshot/marker cache behavior in core.

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = std::cos(yaw * 0.5);
  pose.orientation.z = std::sin(yaw * 0.5);
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  // Builds compact synthetic occupancy/cost grids for deterministic unit scenarios.
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

void set_cells(
  nav_msgs::msg::OccupancyGrid & msg,
  const std::vector<std::pair<int, int>> & cells,
  int value)
{
  const int width = static_cast<int>(msg.info.width);
  for (const auto & [x, y] : cells) {
    msg.data[static_cast<std::size_t>(y * width + x)] = static_cast<int8_t>(value);
  }
}

std::pair<OccupancyGrid2d, OccupancyGrid2d> simple_frontier_world(
  const std::vector<std::pair<int, int>> & free_cells,
  const std::vector<std::pair<int, int>> & blocked_cells = {})
{
  // Occupancy starts unknown, then carve free islands and optional blocked cost cells.
  auto map_msg = build_grid(10, 10, -1);
  set_cells(map_msg, free_cells, 0);

  auto costmap_msg = build_grid(10, 10, 0);
  set_cells(costmap_msg, blocked_cells, 100);

  return {OccupancyGrid2d(map_msg), OccupancyGrid2d(costmap_msg)};
}

FrontierCandidate dummy_frontier(double x = 1.0, double y = 1.0)
{
  return FrontierCandidate{{x, y}, {x, y}, 8};
}

FrontierCandidate make_frontier(double x, double y, int size = 1)
{
  return FrontierCandidate{{x, y}, {x, y}, size};
}

void expect_frontier_results_equal(
  const FrontierSearchResult & expected,
  const FrontierSearchResult & actual)
{
  ASSERT_EQ(expected.frontiers.size(), actual.frontiers.size());
  ASSERT_EQ(expected.robot_map_cell, actual.robot_map_cell);
  for (std::size_t i = 0; i < expected.frontiers.size(); ++i) {
    EXPECT_EQ(expected.frontiers[i].centroid, actual.frontiers[i].centroid);
    EXPECT_EQ(expected.frontiers[i].goal_point, actual.frontiers[i].goal_point);
    EXPECT_EQ(expected.frontiers[i].robot_center_distance_m, actual.frontiers[i].robot_center_distance_m);
    EXPECT_EQ(expected.frontiers[i].size, actual.frontiers[i].size);
  }
}

std::unique_ptr<FrontierExplorerCore> make_snapshot_core(
  FrontierExplorerCoreParams params = {})
{
  // Creates a core with stable clocks/maps so cache invalidation is fully controlled by tests.
  FrontierExplorerCoreCallbacks callbacks;

  auto now_ns = std::make_shared<int64_t>(10'000'000'000);
  callbacks.now_ns = [now_ns]() {
      *now_ns += 1'000'000;
      return *now_ns;
    };
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);

  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);

  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->local_costmap = OccupancyGrid2d(costmap_msg);
  core->map_generation = 1;
  core->costmap_generation = 2;
  core->local_costmap_generation = 3;
  core->frontier_stats_log_throttle_seconds = 0.0;
  core->params.frontier_visit_tolerance = 0.3;

  return core;
}

// Frontier extraction behavior.
TEST(FrontierSearchTests, FrontierExtractionWhenRobotIsOnFreeCell)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
  for (const auto & frontier : result.frontiers) {
    ASSERT_TRUE(frontier.robot_center_distance_m.has_value());
    EXPECT_GE(*frontier.robot_center_distance_m, 0.0);
  }
}

TEST(FrontierSearchTests, FrontierExtractionWhenRobotStartsUnknownUsesFindFree)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  auto result = get_frontier(make_pose(0.0, 0.0), occupancy_map, costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, GlobalCostmapBlockingEliminatesFrontiers)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells, free_cells);
  auto result = get_frontier(make_pose(4.0, 4.0), occupancy_map, costmap);

  EXPECT_TRUE(result.frontiers.empty());
}

TEST(FrontierSearchTests, LocalCostmapBlockingDoesNotEliminateFrontierExtraction)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 3; x < 7; ++x) {
    for (int y = 3; y < 7; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto local_costmap_msg = build_grid(10, 10, 0);
  set_cells(local_costmap_msg, free_cells, 100);
  auto local_costmap = OccupancyGrid2d(local_costmap_msg);

  auto result = get_frontier(
    make_pose(4.0, 4.0),
    occupancy_map,
    costmap,
    local_costmap);

  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierSearchTests, FrontierChoiceDeterministicAcrossIdenticalRuns)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 1; x < 4; ++x) {
    for (int y = 1; y < 4; ++y) {
      free_cells.emplace_back(x, y);
    }
  }
  for (int x = 6; x < 9; ++x) {
    for (int y = 6; y < 9; ++y) {
      free_cells.emplace_back(x, y);
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);

  auto first = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);
  auto second = get_frontier(make_pose(2.0, 2.0), occupancy_map, costmap);

  ASSERT_EQ(first.frontiers.size(), second.frontiers.size());
  for (std::size_t i = 0; i < first.frontiers.size(); ++i) {
    EXPECT_EQ(first.frontiers[i].centroid, second.frontiers[i].centroid);
    EXPECT_EQ(first.frontiers[i].goal_point, second.frontiers[i].goal_point);
    EXPECT_EQ(first.frontiers[i].size, second.frontiers[i].size);
  }
}

TEST(FrontierSearchTests, ConcurrentSearchProducesDeterministicEquivalentResults)
{
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 2; x < 8; ++x) {
    for (int y = 2; y < 8; ++y) {
      if (!(x == 4 && y == 4)) {
        free_cells.emplace_back(x, y);
      }
    }
  }

  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  const auto baseline = get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);

  auto future_one = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });
  auto future_two = std::async(std::launch::async, [&]() {
      return get_frontier(make_pose(3.0, 3.0), occupancy_map, costmap);
    });

  const auto result_one = future_one.get();
  const auto result_two = future_two.get();
  expect_frontier_results_equal(baseline, result_one);
  expect_frontier_results_equal(baseline, result_two);
}

TEST(FrontierSearchTests, VisibleRevealGainStopsAtOccupiedWall)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 5; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{5, 6}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto visible_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(visible_gain.has_value());
  EXPECT_EQ(visible_gain->visible_reveal_cell_count, 0);
  EXPECT_DOUBLE_EQ(visible_gain->visible_reveal_length_m, 0.0);
}

TEST(FrontierSearchTests, VisibleRevealGainRespectsYawAndFov)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 4; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }
  set_cells(map_msg, {{3, 5}}, 100);

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto east_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);
  const auto north_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, -1.57079632679),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);

  ASSERT_TRUE(east_gain.has_value());
  ASSERT_TRUE(north_gain.has_value());
  EXPECT_GT(east_gain->visible_reveal_cell_count, 0);
  EXPECT_EQ(north_gain->visible_reveal_cell_count, 0);
}

TEST(FrontierSearchTests, VisibleRevealGainStopsAtVirtualClusterBounds)
{
  auto map_msg = build_grid(12, 12, -1);
  for (int x = 2; x <= 4; ++x) {
    set_cells(map_msg, {{x, 6}}, 0);
  }

  auto costmap_msg = build_grid(12, 12, 0);
  const OccupancyGrid2d occupancy_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);

  const auto unclipped_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0);
  const auto clipped_gain = compute_visible_reveal_gain(
    make_pose(3.0, 6.0, 0.0),
    occupancy_map,
    costmap,
    std::nullopt,
    6.0,
    1.0,
    1.0,
    FrontierCandidate::CellBounds{2, 6, 4, 6});

  ASSERT_TRUE(unclipped_gain.has_value());
  ASSERT_TRUE(clipped_gain.has_value());
  EXPECT_GT(unclipped_gain->visible_reveal_cell_count, 0);
  EXPECT_EQ(clipped_gain->visible_reveal_cell_count, 0);
  EXPECT_DOUBLE_EQ(clipped_gain->visible_reveal_length_m, 0.0);
}

TEST(FrontierSearchTests, ExplorationCompleteCallbackRunsWhenNoFrontiersRemain)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;
  // 本用例只验证"无 frontier 会触发完成回调", 关掉去抖与地图门限。
  core->params.exploration_complete_confirm_scans = 1;
  core->params.exploration_complete_min_known_cells = 0;

  int completion_calls = 0;
  int frontier_search_calls = 0;
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  core->try_send_next_goal();
  core->try_send_next_goal();

  EXPECT_EQ(completion_calls, 1);
  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

TEST(FrontierSearchTests, ExplorationCompleteRequiresConsecutiveEmptyScans)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;
  core->params.exploration_complete_confirm_scans = 3;
  core->params.exploration_complete_min_known_cells = 0;

  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  int completion_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };

  // 前两次空扫描只累计 streak, 不能判定完成。
  // 每次调用之间必须有一帧新地图(map_generation 变化), 否则同一次扫描会被
  // costmap 回调重复调用而不计数。
  core->try_send_next_goal();
  core->map_generation += 1;
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 0);
  EXPECT_FALSE(core->return_to_start_completed);

  // 第三次达到 confirm_scans 才完成。
  core->map_generation += 1;
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

// 2026-11-19 真机回归: map/global costmap/local costmap 三个回调都会调
// try_send_next_goal(), 没有新地图帧时不能推进完成计数, 否则 10 次确认
// 实测只要 4.1 s(而不是 10 帧地图), 任何瞬时 frontier 抖动都会被锁存成完成。
TEST(FrontierSearchTests, RepeatedCallbacksOnSameMapDoNotAdvanceCompletionStreak)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;
  core->params.exploration_complete_confirm_scans = 3;
  core->params.exploration_complete_min_known_cells = 0;

  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  int completion_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };

  // 同一帧地图被回调重复命中 10 次: 只算 1 次 scan。
  for (int i = 0; i < 10; ++i) {
    core->try_send_next_goal();
  }
  EXPECT_EQ(core->no_frontier_streak, 1);
  EXPECT_EQ(completion_calls, 0);
  EXPECT_FALSE(core->return_to_start_completed);

  // 再来两帧新地图, 凑满 3 次。
  core->map_generation += 1;
  core->try_send_next_goal();
  EXPECT_EQ(core->no_frontier_streak, 2);
  core->map_generation += 1;
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

TEST(FrontierSearchTests, ExplorationCompleteBlockedBySmallMapThreshold)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;
  core->params.exploration_complete_confirm_scans = 1;
  // 20x20 的测试地图只有 400 格已知, 门限设大就永远不判完成。
  core->params.exploration_complete_min_known_cells = 10000;

  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 2.0));
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {2, 2};
      return result;
    };

  int completion_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };

  core->try_send_next_goal();
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 0);
  EXPECT_FALSE(core->return_to_start_completed);
}

// 2026-11-19 真机回归: 候选 frontier 为空不等于探索完成。raw map 上仍有
// "未知且 8 邻接可通行"的边界时, 只说明 frontier 到不了/被压制, 不能锁存完成。
TEST(FrontierSearchTests, RawUnknownBoundaryBlocksCompletionEvenWhenNoCandidateIsFound)
{
  auto core = make_snapshot_core();
  core->params.return_to_start_on_complete = false;
  core->params.exploration_complete_confirm_scans = 1;
  core->params.exploration_complete_min_known_cells = 0;

  auto map_msg = build_grid(10, 10, -1);
  std::vector<std::pair<int, int>> free_cells;
  for (int y = 3; y < 6; ++y) {
    for (int x = 3; x < 6; ++x) {
      free_cells.emplace_back(x, y);
    }
  }
  set_cells(map_msg, free_cells, 0);

  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(4.0, 4.0));
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &, const OccupancyGrid2d &,
    const OccupancyGrid2d &, const std::optional<OccupancyGrid2d> &,
    double, bool)
    {
      FrontierSearchResult result;   // 模拟"有未知区但候选全被过滤/压制"
      result.robot_map_cell = {4, 4};
      return result;
    };
  int completion_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };

  core->ingestRawMapUpdate(OccupancyGrid2d(map_msg));
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 0);
  EXPECT_FALSE(core->return_to_start_completed);
  EXPECT_EQ(core->no_frontier_streak, 0);
  EXPECT_GT(core->raw_unknown_boundary_cell_count(), 0U);

  // 只有把未知区全部填成 free 后, 才允许判定完成。
  core->ingestRawMapUpdate(OccupancyGrid2d(build_grid(10, 10, 0)));
  core->try_send_next_goal();
  EXPECT_EQ(completion_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

TEST(FrontierSearchTests, RawBoundaryBehindEntranceGateIsIgnored)
{
  FrontierExplorerCoreParams params;
  params.cave_mode_enabled = true;
  params.entrance_radius_m = 1.0;
  params.entrance_gate_enabled = true;
  params.entrance_gate_margin_m = 0.5;
  auto core = make_snapshot_core(params);
  core->record_entrance_pose(make_pose(5.0, 5.0, 0.0));

  // 只有门外 (x 中心 < 4.5) 的未知-自由交界。
  auto outside_msg = build_grid(10, 10, 100);
  set_cells(outside_msg, {{1, 5}, {2, 5}, {3, 5}}, 0);
  set_cells(outside_msg, {{1, 4}, {2, 4}, {2, 6}, {3, 4}}, -1);
  core->ingestRawMapUpdate(OccupancyGrid2d(outside_msg));
  EXPECT_EQ(core->raw_unknown_boundary_cell_count(), 0U);

  // 门内有未知-自由交界。
  auto inside_msg = build_grid(10, 10, 100);
  set_cells(inside_msg, {{8, 5}}, 0);
  set_cells(inside_msg, {{7, 5}, {8, 4}}, -1);
  core->ingestRawMapUpdate(OccupancyGrid2d(inside_msg));
  EXPECT_GT(core->raw_unknown_boundary_cell_count(), 0U);
}

TEST(FrontierSearchTests, RawBoundaryTreatsSoftTerrainAsPassable)
{
  auto core = make_snapshot_core();
  // 核心参数默认 occ_threshold=50, 必须显式改为现场值 65, 否则 soft=50 不被视为可通行。
  core->params.occ_threshold = 65;
  auto map_msg = build_grid(10, 10, 100);
  set_cells(map_msg, {{1, 5}}, 0);
  set_cells(map_msg, {{2, 5}, {3, 5}, {4, 5}}, 50);   // SOFT
  set_cells(map_msg, {{5, 5}}, -1);                   // 唯一可通行邻居是 (4,5)=50
  core->ingestRawMapUpdate(OccupancyGrid2d(map_msg));
  EXPECT_EQ(core->raw_unknown_boundary_cell_count(), 1U);
}

// 2026-11-19 端到端回归: 地形层的 SOFT(128) 经 relay 变成 50 后, 决策图把
// cost < occ_threshold 折成 free, 3 格软地形带不再截断地图 BFS; 对照组用 100
// 必须截断。锁定 "relay 50 → 决策图 free → frontier 搜索" 的契约。
TEST(FrontierSearchTests, SoftTerrainBandDoesNotCutFrontierSearch)
{
  // 1-cell-wide corridor at y=1; 3-cell band at x=4..6; unknown at x=10..12.
  auto map_msg = build_grid(14, 3, 100);
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 1; x <= 9; ++x) {
    free_cells.emplace_back(x, 1);
  }
  set_cells(map_msg, free_cells, 0);
  set_cells(map_msg, {{4, 1}, {5, 1}, {6, 1}}, 50);      // SOFT
  set_cells(map_msg, {{10, 1}, {11, 1}, {12, 1}}, -1);   // unknown beyond
  auto costmap_msg = build_grid(14, 3, 0);

  DecisionMapConfig config;
  config.optimization_enabled = false;   // 隔离 soft -> free 的转换
  config.occ_threshold = 65;
  DecisionMapResult decision = build_decision_map(OccupancyGrid2d(map_msg), config);

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  auto result = get_frontier(
    make_pose(2.0, 1.0), decision.decision_map, OccupancyGrid2d(costmap_msg),
    std::nullopt, 0.0, false, options);

  ASSERT_FALSE(result.frontiers.empty());
  EXPECT_GT(result.frontiers.front().centroid.first, 8.0);

  // 对照组: 同样的带子用 100(硬障碍), BFS 被截断, 远端没有 frontier。
  auto hard_msg = map_msg;
  set_cells(hard_msg, {{4, 1}, {5, 1}, {6, 1}}, 100);
  DecisionMapResult hard = build_decision_map(OccupancyGrid2d(hard_msg), config);
  auto hard_result = get_frontier(
    make_pose(2.0, 1.0), hard.decision_map, OccupancyGrid2d(costmap_msg),
    std::nullopt, 0.0, false, options);
  EXPECT_TRUE(hard_result.frontiers.empty());
}

// Frontier snapshot cache behavior.
TEST(FrontierSnapshotTests, SnapshotCacheHitsOnSameGenerationsAndRobotCell)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  const auto first = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  const auto second = core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(first.signature, second.signature);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, SnapshotReusesCacheWhenOnlyRawMapGenerationChanges)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(core->frontier_snapshot_cache_hits, 1);
}

TEST(FrontierSnapshotTests, RefreshDecisionMapSkipsGenerationBumpWhenOutputIsUnchanged)
{
  auto core = make_snapshot_core();
  core->params.frontier_map_optimization_enabled = true;

  auto map_msg = build_grid(20, 20, -1);
  set_cells(map_msg, {{4, 4}, {4, 5}, {5, 4}, {5, 5}}, 0);
  core->map = OccupancyGrid2d(map_msg);

  core->refresh_decision_map();
  const int generation_after_first_refresh = core->decision_map_generation;
  const int misses_after_first_refresh = core->decision_map_cache_misses;

  core->refresh_decision_map();

  EXPECT_EQ(core->decision_map_generation, generation_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_misses, misses_after_first_refresh);
  EXPECT_EQ(core->decision_map_cache_hits, 1);
}

TEST(FrontierSnapshotTests, DebugOutputsDoNotChangeSnapshotOrSelection)
{
  auto debug_core = make_snapshot_core();
  auto quiet_core = make_snapshot_core();
  debug_core->params.frontier_map_optimization_enabled = true;
  quiet_core->params.frontier_map_optimization_enabled = true;
  debug_core->callbacks.debug_outputs_enabled = []() {return true;};
  quiet_core->callbacks.debug_outputs_enabled = []() {return false;};

  auto map_msg = build_grid(20, 20, -1);
  set_cells(
    map_msg,
    {
      {3, 3}, {3, 4}, {4, 3}, {4, 4},
      {10, 10}, {10, 11}, {11, 10}, {11, 11},
    },
    0);
  auto costmap_msg = build_grid(20, 20, 0);

  debug_core->map = OccupancyGrid2d(map_msg);
  quiet_core->map = OccupancyGrid2d(map_msg);
  debug_core->costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->costmap = OccupancyGrid2d(costmap_msg);
  debug_core->local_costmap = OccupancyGrid2d(costmap_msg);
  quiet_core->local_costmap = OccupancyGrid2d(costmap_msg);
  debug_core->map_generation = 10;
  quiet_core->map_generation = 10;
  debug_core->costmap_generation = 11;
  quiet_core->costmap_generation = 11;
  debug_core->local_costmap_generation = 12;
  quiet_core->local_costmap_generation = 12;

  auto frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.robot_map_cell = {3, 3};
      result.frontiers.push_back(FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 4});
      result.frontiers.push_back(FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 9});
      return result;
    };
  debug_core->callbacks.frontier_search = frontier_search;
  quiet_core->callbacks.frontier_search = frontier_search;

  const auto debug_snapshot = debug_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);
  const auto quiet_snapshot = quiet_core->get_frontier_snapshot(make_pose(3.0, 3.0), 0.0);

  ASSERT_EQ(debug_snapshot.frontiers.size(), quiet_snapshot.frontiers.size());
  EXPECT_EQ(debug_snapshot.signature, quiet_snapshot.signature);
  for (std::size_t i = 0; i < debug_snapshot.frontiers.size(); ++i) {
    const auto & debug_candidate = debug_snapshot.frontiers[i];
    const auto & quiet_candidate = quiet_snapshot.frontiers[i];
    EXPECT_EQ(debug_candidate.centroid, quiet_candidate.centroid);
    EXPECT_EQ(debug_candidate.goal_point, quiet_candidate.goal_point);
    EXPECT_EQ(debug_candidate.robot_center_distance_m, quiet_candidate.robot_center_distance_m);
    EXPECT_EQ(debug_candidate.size, quiet_candidate.size);
  }

  const auto debug_selection = debug_core->select_frontier(debug_snapshot.frontiers, make_pose(3.0, 3.0));
  const auto quiet_selection = quiet_core->select_frontier(quiet_snapshot.frontiers, make_pose(3.0, 3.0));

  ASSERT_TRUE(debug_selection.frontier.has_value());
  ASSERT_TRUE(quiet_selection.frontier.has_value());
  const auto & debug_selected = *debug_selection.frontier;
  const auto & quiet_selected = *quiet_selection.frontier;
  EXPECT_EQ(debug_selected.centroid, quiet_selected.centroid);
  EXPECT_EQ(debug_selected.goal_point, quiet_selected.goal_point);
  EXPECT_EQ(debug_selected.robot_center_distance_m, quiet_selected.robot_center_distance_m);
  EXPECT_EQ(debug_selected.size, quiet_selected.size);
}

TEST(FrontierSnapshotTests, DecisionMapOptimizationFollowsParameterInMrtspOnlyCore)
{
  auto core = make_snapshot_core();
  core->params.frontier_map_optimization_enabled = false;

  const auto config = core->decision_map_config();

  EXPECT_FALSE(core->frontier_map_optimization_enabled());
  EXPECT_FALSE(config.optimization_enabled);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnDecisionMapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->decision_map_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnCostmapGenerationChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->costmap_generation += 1;
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, CostmapCallbacksOnlyBumpGenerationWhenSearchInputChanges)
{
  auto core = make_snapshot_core();
  const int initial_costmap_generation = core->costmap_generation;
  const int initial_local_costmap_generation = core->local_costmap_generation;

  auto unchanged_costmap = build_grid(20, 20, 0);
  core->costmapCallback(OccupancyGrid2d(unchanged_costmap));
  core->localCostmapCallback(OccupancyGrid2d(unchanged_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation);

  auto changed_costmap = unchanged_costmap;
  set_cells(changed_costmap, {{3, 3}}, 100);
  core->costmapCallback(OccupancyGrid2d(changed_costmap));
  core->localCostmapCallback(OccupancyGrid2d(changed_costmap));

  EXPECT_EQ(core->costmap_generation, initial_costmap_generation + 1);
  EXPECT_EQ(core->local_costmap_generation, initial_local_costmap_generation + 1);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnRobotCellChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose & pose,
    const OccupancyGrid2d & occupancy,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = occupancy.worldToMap(pose.position.x, pose.position.y);
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(2.0, 2.0), 0.3);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, SnapshotInvalidatesOnMinGoalDistanceChange)
{
  auto core = make_snapshot_core();
  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.4);

  EXPECT_EQ(call_count, 2);
}

TEST(FrontierSnapshotTests, GetFrontierSnapshotRefreshesDirtyDecisionMapBeforeCacheLookup)
{
  auto core = make_snapshot_core();

  int call_count = 0;
  core->callbacks.frontier_search = [&call_count](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      call_count += 1;
      FrontierSearchResult result;
      result.frontiers = {dummy_frontier()};
      result.robot_map_cell = {1, 1};
      return result;
    };

  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);
  core->ingestRawMapUpdate(OccupancyGrid2d(build_grid(20, 20, 0)));
  ASSERT_TRUE(core->decision_map_dirty);
  core->get_frontier_snapshot(make_pose(1.0, 1.0), 0.3);

  EXPECT_EQ(call_count, 1);
  EXPECT_FALSE(core->decision_map_dirty);
}

TEST(FrontierSnapshotTests, PostGoalSettleReadyDependsOnlyOnElapsedTime)
{
  auto core = make_snapshot_core();
  core->params.post_goal_settle_enabled = true;
  core->params.post_goal_min_settle = 0.5;
  core->awaiting_map_refresh = true;
  core->post_goal_settle_active = true;
  core->post_goal_settle_started_at_ns = core->callbacks.now_ns();

  EXPECT_FALSE(core->post_goal_settle_ready());

  core->post_goal_settle_started_at_ns = core->callbacks.now_ns() - 600'000'000;
  EXPECT_TRUE(core->post_goal_settle_ready());
}

TEST(FrontierSelectionTests, SelectFrontierSequenceReturnsFullMrtspOrdering)
{
  auto core = make_snapshot_core();
  const FrontierSequence frontiers{
    make_frontier(1.0, 0.0),
    make_frontier(2.0, 0.0),
    make_frontier(5.0, 0.0),
  };

  const auto frontier_sequence = core->select_frontier_sequence(
    frontiers,
    make_pose(0.0, 0.0),
    frontiers.front());

  const auto selection = core->select_frontier(frontiers, make_pose(0.0, 0.0));

  ASSERT_EQ(frontier_sequence.size(), frontiers.size());
  ASSERT_TRUE(selection.frontier.has_value());
  EXPECT_TRUE(core->are_frontiers_equivalent(frontier_sequence.front(), *selection.frontier));
}

TEST(FrontierSelectionTests, BuildGoalPoseFacesTargetWhenLookAheadUnavailable)
{
  auto core = make_snapshot_core();
  const auto goal_pose = core->build_goal_pose(
    make_frontier(1.0, 1.0),
    make_pose(0.0, 0.0, 1.2));

  EXPECT_NEAR(goal_pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.w, 0.9238795325, 1e-9);
  EXPECT_NEAR(goal_pose.pose.orientation.z, 0.3826834324, 1e-9);
}

// Marker publish deduplication behavior.
TEST(FrontierMarkerTests, MarkerPublishDeduplicatedWhenSignatureUnchanged)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  FrontierSequence frontiers = FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)});
  core->publish_frontier_markers(frontiers);
  core->publish_frontier_markers(frontiers);

  EXPECT_EQ(publish_calls, 1);
}

TEST(FrontierMarkerTests, MarkerPublishRunsWhenSignatureChanges)
{
  auto core = make_snapshot_core();
  int publish_calls = 0;
  core->callbacks.publish_frontier_markers = [&publish_calls](const FrontierSequence &) {
      publish_calls += 1;
    };

  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(2.0, 2.0)}));
  core->publish_frontier_markers(
    FrontierExplorerCore::to_frontier_sequence({dummy_frontier(3.0, 3.0)}));

  EXPECT_EQ(publish_calls, 2);
}

// 2026-11-06: goal 可达性增强（回退 / clearance / 可达性 BFS）。
TEST(FrontierSearchTests, GoalBackoffMovesDispatchPointAwayFromBoundary)
{
  // 一条沿 x 的已知自由走廊 (x=0..8, y=5)，只在前方留一个未知格作 frontier。
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 0; x <= 8; ++x) {
    free_cells.emplace_back(x, 5);
  }
  auto [occupancy_map, costmap] = simple_frontier_world(free_cells);
  const auto robot_pose = make_pose(1.5, 5.5);

  FrontierSearchOptions base_options;
  base_options.min_frontier_size_cells = 1;
  const auto baseline = get_frontier(
    robot_pose, occupancy_map, costmap, std::nullopt, 0.0, false, base_options);
  ASSERT_EQ(baseline.frontiers.size(), 1u);
  const auto baseline_goal = baseline.frontiers.front().goal_point;
  ASSERT_TRUE(baseline_goal.has_value());

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.goal_backoff_from_frontier_m = 2.0;
  options.goal_backoff_search_radius_cells = 2;
  const auto backed_off = get_frontier(
    robot_pose, occupancy_map, costmap, std::nullopt, 0.0, false, options);
  ASSERT_EQ(backed_off.frontiers.size(), 1u);
  const auto backed_off_goal = backed_off.frontiers.front().goal_point;
  ASSERT_TRUE(backed_off_goal.has_value());

  const double baseline_distance = std::hypot(
    baseline_goal->first - robot_pose.position.x,
    baseline_goal->second - robot_pose.position.y);
  const double backed_off_distance = std::hypot(
    backed_off_goal->first - robot_pose.position.x,
    backed_off_goal->second - robot_pose.position.y);

  // 回退方向指向机器人，因此离机器人的距离应当明显变小（1 m 格量化）。
  EXPECT_LT(backed_off_distance, baseline_distance - 0.5);
  // 回退点仍然必须落在已知自由格上。
  int map_x = 0;
  int map_y = 0;
  ASSERT_TRUE(occupancy_map.worldToMapNoThrow(
      backed_off_goal->first, backed_off_goal->second, map_x, map_y));
  EXPECT_EQ(
    occupancy_map.getCost(map_x, map_y),
    static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace));
}

TEST(FrontierSearchTests, ReachabilityCheckDropsFrontierBehindBlockedDoorway)
{
  // 1 格宽的走廊 x=1..8, y=5；costmap 在 (5,5) 上封死，等于门洞被膨胀挡住。
  std::vector<std::pair<int, int>> free_cells;
  for (int x = 1; x <= 8; ++x) {
    free_cells.emplace_back(x, 5);
  }
  const std::vector<std::pair<int, int>> blocked_cells{{5, 5}};
  auto [occupancy_map, costmap] = simple_frontier_world(free_cells, blocked_cells);
  const auto robot_pose = make_pose(2.5, 5.5);

  FrontierSearchOptions base_options;
  base_options.min_frontier_size_cells = 1;
  const auto without_check = get_frontier(
    robot_pose, occupancy_map, costmap, std::nullopt, 0.0, false, base_options);
  bool far_frontier_seen = false;
  for (const auto & frontier : without_check.frontiers) {
    if (frontier.goal_point.has_value() && frontier.goal_point->first > 6.0) {
      far_frontier_seen = true;
    }
  }
  EXPECT_TRUE(far_frontier_seen);

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.goal_reachability_check_enabled = true;
  const auto with_check = get_frontier(
    robot_pose, occupancy_map, costmap, std::nullopt, 0.0, false, options);
  // 近侧 frontier 必须仍然保留: 可达性检查不能把整张图都否掉
  // (曾经把 frontier 簇首格当作 BFS 种子, 结果可达集为空、所有候选被丢弃)。
  ASSERT_FALSE(with_check.frontiers.empty());
  for (const auto & frontier : with_check.frontiers) {
    ASSERT_TRUE(frontier.goal_point.has_value());
    EXPECT_LT(frontier.goal_point->first, 6.0);
  }
  // 2026-11-19: 分阶段计数必须能指出是哪条规则吃掉了远侧候选。
  EXPECT_GE(with_check.stats.rejected_reachability, 1u);
  EXPECT_EQ(without_check.stats.rejected_reachability, 0u);
  EXPECT_EQ(with_check.stats.candidates, with_check.frontiers.size());
}

TEST(FrontierSearchTests, StatsCountClearanceRejections)
{
  // 单个 free 格 (5,5) 是唯一的 goal 落点；把 lethal 成本格放在 (5,7)
  // (正好在 clearance 半径 2 以内)，goal 本身 free 且不 blocked，只有
  // ±2 格净空检查会拒绝它。
  auto [occupancy_map, costmap] = simple_frontier_world(
    {{5, 5}}, {{5, 7}});
  const auto robot_pose = make_pose(5.5, 5.5);

  FrontierSearchOptions options;
  options.min_frontier_size_cells = 1;
  options.goal_clearance_check_enabled = true;
  options.goal_clearance_radius_cells = 2;
  const auto result = get_frontier(
    robot_pose, occupancy_map, costmap, std::nullopt, 0.0, false, options);

  EXPECT_GE(result.stats.rejected_clearance, 1u);
  EXPECT_EQ(result.stats.rejected_reachability, 0u);
  EXPECT_EQ(result.stats.candidates, 0u);
  EXPECT_EQ(result.stats.candidates, result.frontiers.size());
  EXPECT_GE(result.stats.clusters, 1u);
}

}  // namespace
}  // namespace frontier_exploration_ros2
