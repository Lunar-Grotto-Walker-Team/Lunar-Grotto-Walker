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

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "frontier_exploration_ros2/frontier_suppression.hpp"
#include "frontier_exploration_ros2/frontier_policy.hpp"
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "frontier_exploration_ros2/decision_map.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"
#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/cave_geometry.hpp"
#include "frontier_exploration_ros2/direction_policy.hpp"
#include "frontier_exploration_ros2/trajectory_buffer.hpp"
#include "frontier_exploration_ros2/junction_stack.hpp"

namespace frontier_exploration_ros2
{

// Minimal adapter interface that allows core tests to mock action goal handles.
class GoalHandleInterface
{
public:
  virtual ~GoalHandleInterface() = default;
  virtual void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) = 0;
};

// Immutable payload sent from core to node when dispatching an action goal.
struct GoalDispatchRequest
{
  int dispatch_id{0};
  std::string action_name;
  std::string goal_kind;
  geometry_msgs::msg::PoseStamped goal_pose;
  std::optional<FrontierLike> frontier;
  FrontierSequence frontier_sequence;
  std::string description;
};

// Runtime behavior knobs for frontier selection, goal updates, and settle policy.
struct FrontierExplorerCoreParams
{
  std::string map_topic{"map"};
  std::string costmap_topic{"global_costmap/costmap"};
  std::string local_costmap_topic{"local_costmap/costmap"};
  std::string navigate_to_pose_action_name{"navigate_to_pose"};
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  std::string frontier_marker_topic{"explore/frontiers"};
  std::string selected_frontier_topic{"explore/selected_frontier"};
  std::string optimized_map_topic{"explore/optimized_map"};
  double frontier_marker_scale{0.15};
  bool frontier_map_optimization_enabled{true};
  double sigma_s{2.0};
  double sigma_r{30.0};
  int dilation_kernel_radius_cells{1};
  double sensor_effective_range_m{1.5};
  double weight_distance_wd{1.0};
  double weight_gain_ws{1.0};
  double max_linear_speed_vmax{0.5};
  double max_angular_speed_wmax{1.0};
  // MRTSP can traverse its cost matrix greedily or with bounded dynamic programming.
  std::string mrtsp_solver{"dp"};
  // DP limits are algorithm-level controls for the MRTSP solver.
  std::size_t dp_solver_candidate_limit{15};
  std::size_t dp_planning_horizon{10};
  int occ_threshold{OCC_THRESHOLD};
  int min_frontier_size_cells{MIN_FRONTIER_SIZE};
  double frontier_candidate_min_goal_distance_m{0.0};
  // 2026-11-06: goal 可达性增强，见 FrontierSearchOptions 同名参数。
  double goal_backoff_from_frontier_m{0.0};
  int goal_backoff_search_radius_cells{3};
  bool goal_clearance_check_enabled{false};
  int goal_clearance_radius_cells{2};
  bool goal_reachability_check_enabled{false};
  // 2026-11-11: 探索完成去抖。原来一次"没有 frontier"就永久判定完成,
  // 地图瞬时为空/断裂(SLAM 重定位、costmap 清空、启动瞬间)会让探索直接停死。
  // 连续 confirm_scans 次无 frontier, 且地图已有足够已知格, 才判定完成。
  int exploration_complete_confirm_scans{5};
  int exploration_complete_min_known_cells{500};
  // 2026-11-19: 候选为空不等于探索完成。raw map 上仍存在"未知且 8 邻接可通行"
  // 的边界时, 说明 frontier 只是到不了/被压制, 禁止累加完成计数。
  bool exploration_complete_raw_boundary_guard{true};
  // 2026-11-11: 离线干跑。只发布/记录选中的目标, 不向 Nav2 发 action,
  // 用于在 bag 回放时检查选点质量。见 scripts/large/run_large_test.sh。
  bool dry_run{false};
  double frontier_selection_min_distance{0.5};
  bool escape_enabled{false};
  double frontier_visit_tolerance{0.30};
  bool goal_preemption_enabled{false};
  bool goal_skip_on_blocked_goal{false};
  double goal_preemption_min_interval_s{2.0};
  // LiDAR-model geometry used only by the visible-gain helper at the target pose.
  // Maximum ray length for the target-pose visibility estimate.
  double goal_preemption_lidar_range_m{12.0};
  // Angular coverage for the target-pose visibility estimate.
  double goal_preemption_lidar_fov_deg{360.0};
  // Angular sampling density for the target-pose ray-cast estimate.
  double goal_preemption_lidar_ray_step_deg{1.0};
  // Independent near-goal completion shortcut for revealed-preemption paths.
  double goal_preemption_complete_if_within_m{0.0};
  // Minimum occlusion-aware reveal length required to keep the current goal.
  double goal_preemption_lidar_min_reveal_length_m{0.5};
  // Optional heading correction for the target-pose sensor model.
  double goal_preemption_lidar_yaw_offset_deg{0.0};
  bool post_goal_settle_enabled{true};
  double post_goal_min_settle{0.80};
  double map_processing_rate_hz{1.0};
  bool return_to_start_on_complete{true};
  std::string all_frontiers_suppressed_behavior{"stay"};
  bool frontier_suppression_enabled{false};
  int frontier_suppression_attempt_threshold{3};
  double frontier_suppression_base_size_m{1.0};
  double frontier_suppression_expansion_size_m{0.5};
  double frontier_suppression_timeout_s{90.0};
  double frontier_suppression_no_progress_timeout_s{20.0};
  double frontier_suppression_progress_epsilon_m{0.05};
  double frontier_suppression_startup_grace_period_s{15.0};
  int frontier_suppression_max_attempt_records{256};
  int frontier_suppression_max_regions{64};
  // Cave mission mode. All cave-specific behavior (entrance geometry, direction
  // policy, junction stack, segmented return, target preemption) is gated on this
  // flag so default configurations keep the legacy exploration behavior.
  bool cave_mode_enabled{false};

  // Entrance boundary and direction policy.
  double entrance_radius_m{3.0};
  bool entrance_gate_enabled{true};
  double entrance_gate_margin_m{0.5};
  bool direction_policy_enabled{true};
  double front_cone_half_deg{60.0};
  double right_arc_max_deg{150.0};
  double tangent_window_m{1.0};

  // Trajectory recording (used for the direction tangent, path-center resume and
  // segmented return). traj_record_rate_hz <= 0 disables rate throttling.
  double traj_record_rate_hz{2.0};
  double traj_min_spacing_m{0.2};
  int traj_max_points{5000};

  // Depth-first junction memory.
  double junction_dedup_m{1.0};
  double junction_angle_sep_deg{45.0};
  int junction_stack_max{32};
  double suppression_clear_radius_m{2.0};

  // Segmented return to the entrance (must stay well inside the rolling costmap).
  double return_waypoint_spacing_m{5.0};
  int return_max_consecutive_failures{3};

  // Target preemption.
  double target_standoff_radius_m{0.7};
  double target_standoff_step_deg{15.0};
  double clearance_cap_m{1.0};
  double path_center_weight{0.5};
  double path_center_sigma_m{0.5};
  double heading_penalty{0.2};
  double standoff_reach_tolerance_m{0.25};
  double standoff_yaw_tolerance_deg{20.0};
  double target_dedup_m{0.5};
  double target_cooldown_s{5.0};
  double grab_done_timeout_s{60.0};
  double spin_time_allowance_s{10.0};
  double spin_yaw_tolerance_deg{15.0};
  double resume_clearance_m{0.5};

  // A replacement goal must be better than the active one by this fraction before
  // preemption is allowed (anti-oscillation second layer).
  double goal_hysteresis_threshold{0.15};

  // How many consecutive all-suppressed ticks at one pose count as "branch
  // exhausted" in cave mode before treating it as no frontier.
  int suppressed_branch_retries{3};

  // Post-exploration patrol: keep walking the recorded route to look for targets.
  bool patrol_enabled{true};
  double patrol_waypoint_spacing_m{3.0};
  // 0 disables the fallback; the decision tree owns the real timing.
  double patrol_timeout_s{0.0};
};

// Decision for the all-candidates-suppressed case in cave mode.
enum class SuppressedBranchDecision
{
  RETRY,
  EXHAUSTED,
};

// Host callbacks injected by the node wrapper (time, TF pose, action transport, logging).
struct FrontierExplorerCoreCallbacks
{
  std::function<int64_t()> now_ns;
  std::function<std::optional<geometry_msgs::msg::Pose>()> get_current_pose;
  std::function<bool(double timeout_sec)> wait_for_action_server;
  std::function<void(const GoalDispatchRequest &)> dispatch_goal_request;
  std::function<void(const FrontierSequence &)> publish_frontier_markers;
  std::function<void(const geometry_msgs::msg::PoseStamped &)> publish_selected_frontier_pose;
  std::function<void(const nav_msgs::msg::OccupancyGrid &)> publish_optimized_map;
  // Completion hook lets the ROS-facing node trigger optional post-completion side effects.
  std::function<void()> on_exploration_complete;
  // Fired on every mission state transition so the node can publish a heartbeat.
  std::function<void()> on_mission_state_changed;
  // Rotate in place to an absolute map yaw. Returns true when the heading was
  // reached; false when the action was unavailable or timed out.
  std::function<bool(double target_yaw, double time_allowance_s)> spin_to_yaw;
  std::function<bool()> debug_outputs_enabled;
  std::function<void(const std::string &)> log_debug;
  std::function<void(const std::string &)> log_info;
  std::function<void(const std::string &)> log_warn;
  std::function<void(const std::string &)> log_error;
  std::function<FrontierSearchResult(
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)> frontier_search;
};

// Mission-level state for the cave workflow. RuntimeState (session on/off) and
// GoalLifecycleState (single goal) remain independent of this.
enum class MissionState
{
  IDLE,
  EXPLORING,
  BACKTRACKING,
  // Terrain is fully explored but the mission keeps patrolling the recorded
  // route to look for targets until an external return command arrives.
  PATROLLING,
  PAUSED,
  HOLDING,
  RESUMING,
  RETURNING,
  COMPLETE,
};

// Stateful exploration controller independent from ROS transport details.
class FrontierExplorerCore
{
public:
  explicit FrontierExplorerCore(
    FrontierExplorerCoreParams params,
    FrontierExplorerCoreCallbacks callbacks);

  void occupancyGridCallback(const OccupancyGrid2d & map_msg);
  void costmapCallback(const OccupancyGrid2d & map_msg);
  void localCostmapCallback(const OccupancyGrid2d & map_msg);
  void ingestRawMapUpdate(const OccupancyGrid2d & map_msg);
  void handleUrgentRawMapUpdateForActiveGoal();
  void processPendingMapUpdate();

  [[nodiscard]] bool frontier_map_optimization_enabled() const;
  [[nodiscard]] FrontierSearchOptions frontier_search_options() const;
  [[nodiscard]] DecisionMapConfig decision_map_config() const;
  void refresh_decision_map();
  FrontierSequence build_mrtsp_frontier_sequence(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  void try_send_next_goal();

  // 2026-11-19: 分阶段计数日志（throttled）。现场调试时唯一能从日志看出
  // "前沿在哪一层被吃光"的手段。
  void log_frontier_stages(
    std::size_t snapshot_count,
    std::size_t cave_count,
    std::size_t suppressed_count,
    std::size_t policy_count,
    bool selected,
    const FrontierSearchStats & stats);

  std::pair<double, double> frontier_position(const FrontierLike & frontier) const;
  std::pair<double, double> frontier_reference_point(const FrontierLike & frontier) const;
  int frontier_size(const FrontierLike & frontier) const;
  std::string describe_frontier(const FrontierLike & frontier) const;
  FrontierSignature frontier_signature(const FrontierSequence & frontiers) const;

  bool frontier_snapshot_matches(
    const std::optional<FrontierSnapshot> & snapshot,
    const std::pair<int, int> & robot_map_cell,
    double min_goal_distance) const;

  FrontierSnapshot get_frontier_snapshot(
    const geometry_msgs::msg::Pose & current_pose,
    double min_goal_distance);

  void start_post_goal_settle();
  void wait_for_next_map_refresh();
  void clear_post_goal_wait_state();
  bool post_goal_settle_ready() const;
  [[nodiscard]] bool active_frontier_goal_in_progress() const;

  FrontierSelectionResult select_frontier(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  void record_start_pose(const geometry_msgs::msg::Pose & current_pose);

  bool are_frontiers_equivalent(
    const std::optional<FrontierLike> & first_frontier,
    const std::optional<FrontierLike> & second_frontier) const;

  bool frontier_exists_in_set(
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontiers) const;

  std::optional<std::string> frontier_cost_status(
    const std::optional<FrontierLike> & frontier) const;

  geometry_msgs::msg::PoseStamped build_goal_pose(
    const FrontierLike & target_frontier,
    const geometry_msgs::msg::Pose & current_pose) const;

  geometry_msgs::msg::PoseStamped build_dispatch_goal_pose(
      const FrontierLike & target_frontier,
    const geometry_msgs::msg::Pose & current_pose,
    bool bypass_min_distance_dispatch = false) const;

  std::vector<geometry_msgs::msg::PoseStamped> build_goal_pose_sequence(
    const FrontierSequence & target_frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;

  FrontierSequence select_frontier_sequence(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose,
    const std::optional<FrontierLike> & initial_frontier) const;

  bool are_frontier_sequences_equivalent(
    const FrontierSequence & first_frontier_sequence,
    const FrontierSequence & second_frontier_sequence) const;

  void set_goal_state(GoalLifecycleState state);
  void mark_dispatch_state(int dispatch_id, GoalLifecycleState state);

  void reset_replacement_candidate_tracking();
  bool has_stable_replacement_candidate(const FrontierSequence & frontier_sequence);
  [[nodiscard]] std::optional<std::pair<double, double>> active_goal_target_point() const;
  FrontierLike frontier_with_goal_point(
    const FrontierLike & frontier,
    const std::pair<double, double> & goal_point) const;
  // Reprojects the active goal into a hypothetical sensor pose and estimates the
  // still-visible frontier length from there; nullopt means "fall back to snapshot logic".
  std::optional<double> active_goal_visible_reveal_length() const;

  void consider_preempt_active_goal(const std::string & trigger_source = "map");

  void request_frontier_reselection(
    const FrontierSequence & frontier_sequence,
    const geometry_msgs::msg::Pose & current_pose,
    const std::string & selection_mode,
    const std::string & reselection_reason,
    const std::string & goal_update_log_prefix = "Preempting active frontier goal");

  void request_active_goal_cancel(const std::string & reason);
  void issue_active_goal_cancel();
  void cancel_response_callback(
    int dispatch_id,
    bool cancel_accepted,
    const std::string & error_message = "");

  bool dispatch_pending_frontier_goal(
    const std::optional<geometry_msgs::msg::Pose> & current_pose = std::nullopt);

  void start_exploration_session();
  void stop_exploration_session(const std::string & reason = "Stopping exploration session");
  [[nodiscard]] bool ready_for_shutdown() const;

  // Mission state machine (cave mode).
  [[nodiscard]] MissionState mission_state() const;
  [[nodiscard]] std::string mission_state_name() const;
  [[nodiscard]] const std::string & mission_reason() const;
  void set_mission_state(MissionState state, const std::string & reason);
  // Light pause: keeps maps, caches, junction stack and trajectory; stops goal
  // dispatch and cancels the in-flight goal.
  void pause_exploration(const std::string & reason);
  void resume_exploration();
  [[nodiscard]] bool mission_paused() const;
  [[nodiscard]] std::size_t junction_depth() const;

  // Entrance geometry and trajectory recording.
  void record_entrance_pose(const geometry_msgs::msg::Pose & current_pose);
  [[nodiscard]] std::optional<EntranceGeometry> entrance() const;
  void record_robot_trajectory(const geometry_msgs::msg::Pose & current_pose);
  [[nodiscard]] const TrajectoryBuffer & trajectory() const;

  // Candidate policy stages (public for tests and for reuse by the debug observer).
  [[nodiscard]] FrontierSequence filter_by_cave_geometry(
    const FrontierSequence & frontiers) const;
  [[nodiscard]] FrontierSequence apply_direction_policy(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;
  [[nodiscard]] double base_yaw_for_policy(
    const geometry_msgs::msg::Pose & current_pose) const;

  // Depth-first junction tracking and backtracking.
  [[nodiscard]] bool detect_junction(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose) const;
  bool try_backtrack();
  void clear_suppression_around(double x, double y, double radius_m);
  // Junction memory must be updated on every map tick, including while a goal is
  // in flight; the robot drives through junctions instead of stopping at them.
  void update_junction_memory(
    const FrontierSequence & frontiers,
    const geometry_msgs::msg::Pose & current_pose);

  // All-suppressed escalation.
  SuppressedBranchDecision note_suppressed_branch();
  void reset_suppressed_branch();

  // Terminal rejection of a dispatched goal, routed by goal kind.
  void on_goal_rejected(
    const std::string & goal_kind,
    const geometry_msgs::msg::Pose & goal_pose);

  [[nodiscard]] bool has_line_of_sight(
    double from_x, double from_y, double to_x, double to_y) const;

  // Segmented return to the entrance.
  bool start_returning();
  bool dispatch_next_return_waypoint();

  // Post-exploration patrol along the recorded route (ping-pong).
  bool start_patrolling();
  bool dispatch_next_patrol_waypoint();
  void on_patrol_waypoint_result(bool success);
  // External return command (decision tree timer).
  void request_return();

  // Target preemption (map-frame target coordinates).
  void ingest_target(double x, double y, int64_t stamp_ns);
  void on_target_standoff_result(bool success);
  void on_grab_done();
  void tick_mission();
  bool dispatch_resume_goal();
  void on_resume_center_result(bool success);
  [[nodiscard]] std::optional<std::pair<double, double>> active_target() const;
  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> compute_standoff_pose(
    double target_x,
    double target_y) const;
  [[nodiscard]] double clearance_score_at(double x, double y) const;

  // Goal hysteresis: distance per unit of information gain, mirroring the MRTSP
  // start-row cost shape.
  [[nodiscard]] double frontier_selection_cost(
    const FrontierCandidate & frontier,
    const geometry_msgs::msg::Pose & current_pose) const;
  [[nodiscard]] bool should_switch_to_frontier(
    const FrontierCandidate & candidate,
    const geometry_msgs::msg::Pose & current_pose) const;

  void handle_exploration_complete(const geometry_msgs::msg::Pose & current_pose);
  // 当前地图里已知格(非未知)数量是否达到阈值; 用于"完成"判定的最小地图门限。
  bool map_known_cells_reach(int threshold) const;
  // raw map 上"未知且 8 邻接可通行"的格数。可通行 = 0 <= cost < occ_threshold
  // (SOFT=50 在 Go2 上可走); cave 模式忽略入口门线/半径之外的格。按 map_generation
  // 缓存, 达到 stop_after 提前退出。
  std::size_t raw_unknown_boundary_cell_count(std::size_t stop_after = 1000);
  void handle_all_frontiers_suppressed(const geometry_msgs::msg::Pose & current_pose);
  void consider_cancel_suppressed_return_to_start();

  bool is_pose_within_xy_tolerance(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    double tolerance = 0.25) const;

  bool send_pose_goal(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & goal_kind,
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontier_sequence,
    const std::string & description);

  bool send_frontier_goal(
    const FrontierSequence & frontier_sequence,
    const geometry_msgs::msg::Pose & current_pose,
    const std::string & description,
    bool bypass_min_distance_dispatch = false);

  void dispatch_goal_request(
    const std::string & action_name,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & goal_kind,
    const std::optional<FrontierLike> & frontier,
    const FrontierSequence & frontier_sequence,
    const std::string & description);

  void clear_active_goal_state();

  void goal_response_callback(
    int dispatch_id,
    const std::shared_ptr<GoalHandleInterface> & received_goal_handle,
    bool accepted,
    const std::string & error_message = "");

  void get_result_callback(
    int dispatch_id,
    int status,
    int error_code,
    const std::string & error_message,
    const std::string & exception_text = "");

  void feedback_callback(double distance_remaining, int dispatch_id);
  bool evaluate_active_goal_progress_timeout();

  void publish_frontier_markers(const FrontierSequence & frontiers);

  void request_shutdown();

  static FrontierSequence to_frontier_sequence(
    const std::vector<FrontierCandidate> & frontiers);

  bool suppression_state_allocated() const
  {
    return static_cast<bool>(frontier_suppression_);
  }

  std::size_t suppression_attempt_count() const
  {
    return frontier_suppression_ ? frontier_suppression_->attempt_count() : 0U;
  }

  std::size_t suppressed_region_count() const
  {
    return frontier_suppression_ ? frontier_suppression_->region_count() : 0U;
  }

  // Configuration and host integration hooks.
  FrontierExplorerCoreParams params;
  FrontierExplorerCoreCallbacks callbacks;

  // Latest map/costmap snapshots and monotonic generation counters.
  std::optional<OccupancyGrid2d> map;
  std::optional<OccupancyGrid2d> decision_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr decision_map_msg;
  DecisionMapWorkspace decision_map_workspace;
  std::optional<OccupancyGrid2d> costmap;
  std::optional<OccupancyGrid2d> local_costmap;
  int map_generation{0};
  int decision_map_generation{0};
  int costmap_generation{0};
  int local_costmap_generation{0};
  int decision_map_cache_hits{0};
  int decision_map_cache_misses{0};

  // Frontier snapshot cache keyed by decision/costmap generations + robot cell + min distance.
  std::optional<FrontierSnapshot> frontier_snapshot;
  int frontier_snapshot_cache_hits{0};
  int frontier_snapshot_cache_misses{0};
  double frontier_stats_log_throttle_seconds{2.0};
  std::optional<int64_t> last_frontier_stats_log_time_ns;

  struct RawFrontierDebugCacheEntry
  {
    int map_generation{0};
    int costmap_generation{0};
    int local_costmap_generation{0};
    std::pair<int, int> robot_map_cell{0, 0};
    double min_goal_distance{0.0};
    FrontierSearchOptions search_options;
    std::size_t frontier_count{0};
  };
  std::optional<RawFrontierDebugCacheEntry> raw_frontier_debug_cache;

  struct MrtspOrderCacheEntry
  {
    FrontierSignature frontier_signature;
    int pose_x_bucket{0};
    int pose_y_bucket{0};
    int yaw_bucket{0};
    double sensor_effective_range_m{0.0};
    double weight_distance_wd{0.0};
    double weight_gain_ws{0.0};
    double max_linear_speed_vmax{0.0};
    double max_angular_speed_wmax{0.0};
    // Solver settings are part of the order cache key because they can change the
    // selected sequence without any map, frontier, or robot-pose change.
    std::string mrtsp_solver;
    std::size_t dp_solver_candidate_limit{0};
    std::size_t dp_planning_horizon{0};
    FrontierSequence frontier_sequence;
  };
  std::optional<MrtspOrderCacheEntry> mrtsp_order_cache;
  int mrtsp_order_cache_hits{0};
  int mrtsp_order_cache_misses{0};

  // Active action/goal lifecycle state.
  std::shared_ptr<GoalHandleInterface> goal_handle;
  std::optional<geometry_msgs::msg::PoseStamped> start_pose;
  std::optional<geometry_msgs::msg::PoseStamped> active_goal_pose;
  std::optional<FrontierLike> active_goal_frontier;
  FrontierSequence active_goal_frontiers;
  std::string active_goal_kind;
  std::string active_action_name;
  std::optional<int64_t> active_goal_sent_time_ns;
  bool goal_in_progress{false};
  GoalLifecycleState goal_state{GoalLifecycleState::IDLE};
  bool exploration_enabled{true};

  // Mission-level state machine (cave workflow). See MissionState.
  MissionState mission_state_{MissionState::IDLE};
  std::string mission_reason_;
  bool paused_by_user_{false};

  // Cave candidate policy and trajectory record.
  CaveGeometry cave_geometry_;
  DirectionPolicy direction_policy_;
  TrajectoryBuffer trajectory_;
  std::optional<int64_t> last_traj_record_ns_;
  JunctionStack junction_stack_;
  std::vector<TrajectoryPoint> return_waypoints_;
  std::size_t return_index_{0};
  int return_consecutive_failures_{0};

  std::optional<std::pair<double, double>> active_target_;
  int64_t holding_started_ns_{0};
  bool has_last_target_{false};
  std::pair<double, double> last_target_{0.0, 0.0};
  int64_t last_target_ns_{0};
  std::optional<double> active_goal_cost_;
  int suppressed_branch_streak_{0};
  int return_dispatch_failures_{0};
  // 分阶段计数日志的 throttle 状态（内容变化或 5 s 才再打一次）。
  int64_t last_stage_log_ns_{0};
  std::string last_stage_log_key_;

  std::vector<TrajectoryPoint> patrol_waypoints_;
  int patrol_index_{-1};
  int patrol_direction_{1};
  int64_t patrol_started_ns_{0};
  bool return_requested_{false};
  bool pause_was_patrol_{false};
  int patrol_dispatch_failures_{0};
  int current_dispatch_id{0};
  std::unordered_map<int, GoalLifecycleState> dispatch_states;

  // Exploration completion/reporting flags.
  // 连续"没有 frontier"的扫描次数, 用于完成判定去抖。
  int no_frontier_streak{0};
  // 上一次计入 streak 时 map_generation 的值。try_send_next_goal() 由
  // /exploration_map、global costmap、local costmap 三个回调共同触发,
  // 如果不按"新的一帧地图"去重, 同一帧会被计入 2~7 次, 去抖形同虚设。
  int no_frontier_streak_map_generation{-1};
  bool no_frontiers_reported{false};
  // "地图还太小"提示每次 streak 只打一次, 避免空地图期间每帧刷屏。
  bool no_frontier_small_map_reported{false};
  bool no_reachable_frontier_reported{false};
  bool all_frontiers_suppressed_reported{false};
  // raw 未知边界护栏: 计数缓存 + 日志去重。
  int raw_boundary_map_generation{-1};
  std::size_t raw_boundary_cell_count_cache{0};
  bool raw_boundary_blocks_completion_reported{false};

  // Post-goal settle and map-refresh gating state.
  bool awaiting_map_refresh{false};
  bool post_goal_settle_active{false};
  std::optional<int64_t> post_goal_settle_started_at_ns;
  bool decision_map_dirty{false};
  bool pending_costmap_search_input_update{false};
  bool pending_local_costmap_search_input_update{false}; 

  // Escape and return-to-start behavior flags.
  bool return_to_start_started{false};
  bool return_to_start_completed{false};
  bool suppressed_return_to_start_started{false};

  // Preemption/cancel pipeline and pending replacement goal state.
  bool cancel_request_in_progress{false};
  std::optional<std::string> pending_cancel_reason;
  FrontierSequence pending_frontier_sequence;
  std::string pending_frontier_selection_mode;
  std::string pending_frontier_dispatch_context;
  std::optional<std::string> active_goal_blocked_reason;
  std::optional<FrontierLike> distance_completed_frontier;
  std::optional<int64_t> last_low_gain_reselection_time_ns;
  // dry_run 时记录上一次已发布/记录的目标，避免同一目标每帧刷日志。
  std::optional<std::pair<double, double>> last_dry_run_goal_;

  // Replacement frontier debouncing and marker deduplication state.
  std::optional<FrontierLike> replacement_candidate_frontier;
  int replacement_candidate_hits{0};
  int replacement_required_hits{2};
  std::optional<FrontierSignature> last_published_frontier_signature;
  std::unique_ptr<FrontierSuppression> frontier_suppression_;
  std::optional<int64_t> frontier_suppression_activation_ns_;

  // Shutdown guard for callbacks racing during teardown.
  bool shutdown_requested{false};

private:
  struct DispatchContext
  {
    std::string goal_kind;
    std::optional<FrontierLike> frontier;
    FrontierSequence frontier_sequence;
    std::string action_name;
  };

  std::unordered_map<int, DispatchContext> dispatch_contexts;

  bool debug_outputs_enabled() const;
  void throttled_debug(const std::string & message);
  void log_frontier_snapshot_stats(
    const FrontierSequence & frontiers,
    double duration_ms,
    bool cache_hit);
  FrontierSuppression * ensure_frontier_suppression();
  bool suppression_enabled() const;
  bool suppression_runtime_active(int64_t now_ns) const;
  bool should_return_to_start_when_all_frontiers_suppressed() const;
  FrontierSequence filter_frontiers_for_suppression(const FrontierSequence & frontiers);
  double frontier_snapshot_min_goal_distance_for_pose(
    const geometry_msgs::msg::Pose & current_pose);
  void record_failed_frontier_attempt(const std::optional<FrontierLike> & frontier);
  void clear_active_goal_progress_state();
  void start_active_goal_progress_tracking();
  void commit_deferred_costmap_search_input_updates();
  void note_active_goal_progress(double distance_remaining);
  void reset_exploration_runtime_state(bool clear_maps);

  std::optional<DispatchContext> dispatch_context_for(int dispatch_id) const;
};

}  // namespace frontier_exploration_ros2
