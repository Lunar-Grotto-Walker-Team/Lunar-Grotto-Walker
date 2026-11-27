// include/lgw_bt_planner/servo/servo_node.hpp
//
// The lgw servo node class, moved out of servo_node.cpp so tests can link it.
// The body is UNCHANGED from the pre-split file (in-class definitions are
// implicitly inline); later tasks peel the members out into their own units.
#pragma once

//
// lgw_servo_node — ROS interface layer of the visual-servo takeover stack
// (spec docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md §7,
// plan Task 7).
//
// This node owns NO geometry and NO control math. It is the thin ROS shell
// that wires the five pure modules together:
//
//   tracker selection  — selected_track_id from UnifiedTrackArray (2026-11-17)
//   geometry_source   — head/arm/odom tier for this tick    (spec §7.4)
//   pinned posture    — target -> desired tcp_link pose     (spec 2026-10-30)
//   servo_control_law — pose error -> TwistStamped          (spec §7.6)
//   servo_state_machine — takeover/hold/release + watchdog  (spec §7.1)
//
// What lives here is only ROS plumbing: TF lookups, message conversion,
// Servo lifecycle (start/pause/unpause), the 100 Hz timer and the four
// /lgw/servo/* services. If you find yourself writing geometry or control
// math below, stop and call the module instead.
//
// Contracts inherited from earlier tasks (do not re-derive):
//   * Current tcp pose = TF <frame_prefix>base_link <- <frame_prefix>tcp_link,
//     probed with a zero-timeout can_transform() first. No /feedback/tcp_pose,
//     no link6 <-> tcp conversion (spec §7.6, D18).
//   * geometry_source::select_source wants T_base_odom =
//     lookupTransform("odom", "<prefix>base_link") AS RETURNED (it inverts
//     internally).
//   * The posture solver throws on a degenerate target: every call is wrapped
//     in try/catch and a throw is treated as HOLD.
//   * ControlLaw's in_deadband is ALSO set for non-finite inputs, so it is
//     never used alone as "converged".
//   * lgw_bt_planner::ServoState (enum) and lgw_bt_planner::msg::ServoState
//     (message) are distinct types: never `using namespace ...::msg`.
//
// Threading (spec §7.7): control timer on its own MutuallyExclusive group,
// subscriptions Reentrant, everything else on dedicated groups, spun by a
// MultiThreadedExecutor(2). Servo pause/unpause clients live on a group that
// is NOT added to the main executor; each blocking call spins a local
// SingleThreadedExecutor for that group only (same pattern as
// select_arm_observation_pose_service_node.cpp, see MEMORY
// feedback_rclcpp_ik_sync_subcall — never spin_until_future_complete(node)).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "lgw_bt_planner/servo/recovery_orchestrator.hpp"
#include "lgw_bt_planner/servo/servo_params.hpp"
// The generated listener (config/servo_node_parameters.yaml) is only held
// through a shared_ptr and the destructor is out-of-line, so a forward
// declaration is enough: including the generated header here would make every
// includer parse it for nothing.
namespace servo_params
{
class ParamListener;
}  // namespace servo_params
#include "lgw_bt_planner/servo/servo_session.hpp"
#include "lgw_bt_planner/servo/servo_state_reporter.hpp"
#include "lgw_bt_planner/servo/tracking_loop.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yoloe_interfaces/msg/unified_track_array.hpp>


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lgw_bt_planner/lookat_posture_family.hpp"
#include "lgw_bt_planner/lookat_qp_solver.hpp"
#include "lgw_bt_planner/lookat_solver.hpp"
#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/move_group_prepositioner.hpp"
#include "lgw_bt_planner/preposition_decision.hpp"
#include "lgw_bt_planner/msg/servo_state.hpp"
#include "lgw_bt_planner/servo_control_law.hpp"
#include "lgw_bt_planner/servo_state_machine.hpp"
#include "lgw_bt_planner/servo_types.hpp"
#include "lgw_bt_planner/srv/takeover.hpp"
#include "lgw_bt_planner/servo/servo_node_detail.hpp"

namespace lgw_bt_planner
{
// The node-scope constants and helpers live in servo_detail (a named
// namespace in a private header). A using-directive at THIS scope keeps
// the ~200 existing call sites unchanged; it cannot leak past the package
// because only this package includes servo_node.hpp.
using namespace servo_detail;  // NOLINT(build/namespaces)

class ServoNode : public rclcpp::Node, public SessionHost
{
public:
  explicit ServoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ServoNode() override;

private:
  // /servo_node/start_servo lifecycle. The service returns success even when
  // Servo's own waitForCompleteState() timed out (spec §7.9), so its return
  // value is only a trigger for the status-based confirmation below.
  enum class StartState
  {
    NOT_STARTED,   // waiting for the pose stream / the service
    REQUESTED,     // request in flight, no response yet
    CONFIRMING,    // response said success, waiting for Servo's output
    PAUSING,       // Servo confirmed; park it so IDLE is truly passive
    STARTED,       // confirmed running and paused
    FAILED,        // start failed -> FAULT
  };

  // ===== start-up ==========================================================

  void declare_and_load_parameters();

  // Loads arm_calibration.json (the _tcp variant, camera -> tcp_link). A
  // failure is fatal for this node: FAULT, no Servo start, takeover refused
  // (spec §7.8 "标定文件缺失/解析失败 → 启动即 FAULT").
  void load_calibration();

  // ===== subscriptions =====================================================

  void on_track_array(const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg);

  void on_pose_watch(const sensor_msgs::msg::JointState::SharedPtr msg);

  // True once a message has named all six arm joints at least once. A driver
  // that publishes only some joints would otherwise leave the rest at whatever
  // the array happened to hold — zero on the first message — and the following
  // monitor would compare against a pose the arm never had.
  bool measured_complete_locked() const;

  void on_follow_reference(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Max |commanded - measured| over the six arm joints, in radians.
  // NaN when either side has not produced a sample yet.
  double follow_gap_locked() const;

  // Has the COMMANDED joint vector advanced by at least kSatCmdProgressRad
  // since the last time this returned true? Used by the unreachable guard:
  // /control still moving means Servo is driving, so a stubbornly high
  // measured error is a slow follower rather than an unreachable target.
  // Updates its own reference, so call it exactly once per tick.
  bool commanded_advancing_locked();

  // Servo status carries only the *current iteration's* warning
  // (servo_calcs.cpp resets it right after publishing), so we accumulate a
  // halt window here rather than reading a single message.
  void on_servo_status(const std_msgs::msg::Int8::SharedPtr msg);

  void on_servo_output(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  void on_move_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  void on_ft_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  void on_start_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  // ===== control timer =====================================================

  void on_timer();

public:
  // State name for tests and diagnostics ("IDLE"/"TRACKING"/...).
  std::string state_for_test();

  // Test hooks for the scan gate (drive the SM into HOLD without hardware).
  void force_hold_for_test(HoldReason reason);
  bool scan_in_progress_for_test() const {return scan_in_progress_.load();}
  // Skip the MoveGroup ingress move so the scan worker can be exercised
  // without a planning pipeline (tests only).
  void skip_ingress_for_test() {skip_ingress_for_test_.store(true);}
  // Ask the scan worker to stop the way a release does.
  void cancel_scan_for_test() {scan_cancel_.store(true);}
  // Re-run the selection refresh against the injected array (no start needed).
  void reselect_for_test()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reselect_locked();
  }
  uint64_t selected_track_id_for_test() const
  {
    return has_selected_snapshot_ ? selected_snapshot_.track_id : 0u;
  }
  uint32_t reselect_loop_reset_count_for_test() const
  {
    return reselect_loop_reset_count_;
  }
  void set_latest_array_for_test(
    const yoloe_interfaces::msg::UnifiedTrackArray & arr);
  bool takeover_for_test(
    uint64_t track_id, const std::string & class_filter, bool & scan_only);
  // Full service path (including the scan worker hand-off), for tests that
  // need the SCANNING thread to actually run.
  bool takeover_service_for_test(
    uint64_t track_id, const std::string & class_filter,
    std::string & message);

  // Test-only conversion of a published track into the servo snapshot.
  TrackSnapshot make_snapshot_for_test(
    const yoloe_interfaces::msg::UnifiedTrack & track) const
  {
    return make_snapshot(track);
  }

  // SessionHost: cancel the active session (public for tests; the release
  // handler drives the same transition).
  void clear_session() override;

  // One control tick. `now_ns` is the ONLY time source for everything below:
  // production passes the wall clock, the replay harness passes stepped
  // timestamps so the same input sequence yields the same output sequence.
  void tick(int64_t now_ns);

private:
  // Single "failure -> HOLD" path (2026-11-07 node-split plan, D3). Before this
  // the same statements were repeated at seven sites, so any change to the
  // failure semantics needed seven edits. `clear_geometry=false` keeps the last
  // known geometry source in the published state -- the saturated path has
  // always done that and is the only one.
  bool hold_and_return(HoldReason reason, int64_t now_ns, bool clear_geometry = true);


  // Records why this session entered HOLD (spec 2026-11-03 §4). The state
  // machine only knows TARGET_LOST, so the reason must be captured at the
  // trigger site. Every TRACKING->HOLD path calls this exactly once.
  void enter_hold_locked(HoldReason reason, int64_t now_ns);

  // ===== HOLD recovery (spec 2026-11-03 §5-§9) =============================

  struct RecoveryTarget
  {
    bool available{false};
    tf2::Vector3 p_arm_base;
    int64_t age_ms{0};
  };

  // Geometry for the recovery solve: the live source when the target is
  // visible, otherwise the freshest contributor of the last selected
  // snapshot (still expressed in the arm base frame). Age is reported so the
  // pure decision can reject stale geometry.
  RecoveryTarget resolve_recovery_target_locked(int64_t now_ns, bool target_ok, const tf2::Vector3 & current_target);

  // Caller holds mutex_.
  void finish_recovery_locked(bool fatal, const std::string & detail);

  // A posture-family failure means "the target is unreachable RIGHT NOW",
  // which is transient (the dog walked over it). It must not consume the
  // attempt: undo the increment, stay in HOLD, and try again as soon as the
  // target is reachable. The pre-check can flap with the solver's warm start,
  // so this path is where the consistency is enforced.
  void finish_recovery_wait(const std::string & detail);

  void finish_recovery_failed(const std::string & detail, bool fatal);

  // Caller holds mutex_. True when the worker must stop: a release moved the
  // state away from RECOVERING, or the deadline requested a cancel.
  bool recovery_abort_locked() const;

  bool worker_aborted();

  void finish_recovery_cancelled();

  // Publishes a zero twist and compares Servo's replayed cache with the pose
  // measured at probe time. Caller must NOT hold mutex_, and nothing else may
  // be publishing twists (PREEMPT/HOLD/RECOVERING, never TRACKING).
  bool verify_servo_seed(const std::array<double, kArmJointCount> & expected, double tolerance_rad, std::string & error);

  // Caller holds mutex_. follower_lag: the command is already frozen by HOLD;
  // wait for the real arm to close the gap and then ask for a resume. Never
  // commands a MoveGroup move: the bottleneck is the driver/firmware, not the
  // pose (spec 2026-11-03 §3).
  void update_follower_recovery_locked(int64_t now_ns);

  // Caller holds mutex_. Gathers what only the node can see — the resolved
  // target, the solver's reachability verdict, the TF probe — and lets the
  // orchestrator decide. The worker thread below is the node's; the decision
  // is not.
  void maybe_start_recovery_locked(int64_t now_ns, bool target_ok, const tf2::Vector3 & current_target);

  // SCANNING (2026-11-13). Only HOLD + kTargetLost may enter; the worker
  // pauses Servo and drives the JTC through MoveGroup.
  void maybe_start_scan_locked(int64_t now_ns, bool target_ok);
  void scan_worker();
  // Constant-pose trajectory at the current command: preempts the scan
  // trajectory in the JTC so the arm stops where it is.
  void publish_scan_stop();
  bool scan_worker_aborted();
  void cancel_scan();
  void finish_scan_found();
  void finish_scan_timeout();
  bool base_is_static_locked(int64_t now_ns);

  // Runs on its own thread: solve the posture family, move with MoveGroup,
  // re-seed Servo, prove the seed with a zero-twist probe, then unpause.
  void recovery_worker();

  // One 100 Hz tick while TRACKING/HOLD. Returns true when Servo must be
  // paused (the caller performs the blocking call outside mutex_).
  bool update_tracking_locked(const rclcpp::Time & stamp, int64_t now_ns);

  // Publishes a TwistStamped in the BARE `base_link` frame Servo pins in
  // servo_params.yaml (planning_frame / robot_link_command_frame).
  void publish_twist_locked(double vx, double vy, double vz, double wx, double wy, double wz, int64_t stamp_ns);

  // ===== Servo lifecycle (spec §7.9) ======================================

  void maybe_start_servo_locked(int64_t now_ns);
  // One-shot startup check: the hand-eye chain the control law uses (FK +
  // arm_calibration.json) must agree with the TF chain the tracker uses
  // (RSP tcp_link -> camera mount -> optical frame).  A mismatch means the
  // two paths are looking at different robots/calibrations.
  void maybe_check_camera_tf_locked();

  struct TriggerOutcome
  {
    bool ok{false};
    std::string detail;
  };

  // Blocking Trigger call on an independent callback group. The local
  // executor only owns servo_sync_group_ (auto_add=false), so it can spin on
  // the calling thread without touching the main executor's node (see MEMORY
  // feedback_rclcpp_ik_sync_subcall).
  TriggerOutcome call_trigger_sync(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const char * name, double timeout_sec);

  // ===== /lgw/servo/* services (spec §7.2) ================================

  // Bring the arm into the posture family before Servo takes over.
  // Returns false (with `error`) when that could not be achieved — the caller
  // then refuses the takeover. MUST NOT be called with mutex_ held: the move
  // blocks for as long as MoveGroup needs, and the control timer needs the
  // mutex every 10 ms.
  bool maybe_preposition(std::string & error);

  // A takeover that already passed try_takeover_locked (state PREEMPT) but
  // failed later must go back to IDLE, not stop at RELEASING: only
  // PAUSE_OK leaves RELEASING, and a refused takeover never sends one
  // (review #12). Pause first, then complete the transition.
  void refuse_takeover_to_idle(const std::shared_ptr<lgw_bt_planner::srv::Takeover::Response> & response, const std::string & message);

  void handle_takeover(const std::shared_ptr<lgw_bt_planner::srv::Takeover::Request> request, std::shared_ptr<lgw_bt_planner::srv::Takeover::Response> response);

  // Runs under mutex_. On success the state machine is in PREEMPT (or
  // TRACKING if UNPAUSE_OK was already driven by the caller).
  bool try_takeover_locked(
    const lgw_bt_planner::srv::Takeover::Request & request,
    std::string & reason, bool & scan_only);

  void handle_release(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_pause(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_resume(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ===== SessionHost (Task 8) =============================================
  // The node is the session's world: it owns the state machine, the control
  // mutex and the Servo clients, and these trampolines are the whole of what
  // ServoSession is allowed to touch.
  void under_lock(const std::function<void()> & critical) override;

  ServoStateMachine & sm() override;

  void enter_hold(HoldReason reason) override;

  void arm_saturation_window() override;
  void mark_dirty() override;

  void cancel_preposition_unless_recovery() override;

  void arm_recovery_cancel() override;

  std::array<double, 6> measured_positions() override;

  bool verify_seed(const std::array<double, 6> & expected, double tolerance, std::string & error) override;

  RpcOutcome rpc(ServoRpc which, const char * tag) override;

  void log_info(const std::string & message) override;
  void log_warn(const std::string & message) override;
  void log_error(const std::string & message) override;

  // ===== helpers (all * _locked callers hold mutex_) ======================

  bool session_active_locked() const;

  static TrackSnapshot empty_snapshot();

  void clear_session_locked();

  void arm_saturation_window_locked();

  bool move_group_busy_locked() const;

  bool servo_halt_persisted_locked(int64_t now_ns) const;

  // Refreshes the selection from the latest array: servo follows
  // `selected_track_id` and never re-selects. A missing id clears the
  // snapshot, which the control tick reads as TARGET_LOST.
  void reselect_locked();

  // UnifiedTrack -> TrackSnapshot. The tracker publishes one current
  // arm-base point plus its validity; the servo no longer re-selects a
  // camera tier here.
  static TrackSnapshot make_snapshot(const yoloe_interfaces::msg::UnifiedTrack & track);

  // Contract 2: T_base_odom is lookupTransform("odom", "<prefix>base_link")
  // exactly as returned; geometry_source inverts it internally.
  std::optional<tf2::Transform> lookup_odom_base_locked();

  // TCP pose of the COMMAND domain: forward kinematics on the joints Servo
  // integrates on (/control/joint_states), i.e. where the JTC says the arm is.
  // Under the topic hardware backend that is NOT the real arm (which trails by
  // ~0.3 s), and anchoring the control law on the trailing pose while Servo
  // integrates the command makes the loop ring (measured 2026-11-06). The
  // real pose stays in use for monitoring (follow gap, halts, servo state).
  bool fk_tcp_locked(Pose6 & pose);

  // Contract 1: current tcp pose, from the command domain when available
  // (tcp_pose_domain=command, the default) and from TF
  // <prefix>base_link <- <prefix>tcp_link otherwise. TF lookups are guarded by
  // a zero-timeout can_transform probe first (a timed lookup blocks the timer).
  bool lookup_tcp_locked(Pose6 & pose);

  void publish_state_locked(const rclcpp::Time & stamp);

  // ===== parameters ========================================================
  // All node parameters live here; the members below are thin
  // references into it (2026-11-07 node-split plan, Task 3b).
  // The listener owns the declare + validation (Task 4); it must outlive every
  // use of `params_`, so it is constructed first thing in the constructor.
  std::shared_ptr<servo_params::ParamListener> param_listener_;
  ServoParams params_;

  std::string & frame_prefix_{params_.frame_prefix};
  std::string base_frame_;
  std::string tcp_frame_;
  std::string & pose_watch_topic_{params_.pose_watch_topic};
  std::string & calibration_path_{params_.calibration_path};

  PostureFamily posture_;
  // Selected at startup; never switches at runtime (2026-11-05).
  LookatSolver * solver_{&posture_};
  std::unique_ptr<QpPostureSolver> qp_solver_;
  QpSolverParams & qp_params_{params_.qp};
  PostureParams & posture_params_{params_.posture};
  // Solve-time telemetry, logged throttled: mean and worst case per window.
  std::string posture_last_status_;
  bool posture_trace_{false};
  PrepositionParams & preposition_params_{params_.preposition};
  double & preposition_velocity_scaling_{params_.preposition_velocity_scaling};
  double & preposition_accel_scaling_{params_.preposition_accel_scaling};
  double & preposition_tolerance_rad_{params_.preposition_tolerance_rad};
  double & preposition_timeout_s_{params_.preposition_timeout_s};
  std::unique_ptr<MoveGroupPrepositioner> prepositioner_;
  std::atomic_bool preposition_in_progress_{false};
  // HOLD recovery (spec 2026-11-03 §5-§9). One worker at a time; the flags
  // are atomic because the worker and the 100 Hz timer both read them.
  HoldRecoveryParams & recovery_params_{params_.recovery};
  double & recovery_velocity_scaling_{params_.recovery_velocity_scaling};
  double & recovery_accel_scaling_{params_.recovery_accel_scaling};
  double & recovery_timeout_s_{params_.recovery_timeout_s};
  double & recovery_tolerance_rad_{params_.recovery_tolerance_rad};
  int & recovery_settle_ms_{params_.recovery_settle_ms};
  double & recovery_presettle_gap_rad_{params_.recovery_presettle_gap_rad};
  int64_t & recovery_presettle_timeout_ms_{params_.recovery_presettle_timeout_ms};
  int64_t & recovery_presettle_stable_ms_{params_.recovery_presettle_stable_ms};
  std::atomic_bool recovery_in_progress_{false};
  std::atomic_bool recovery_cancel_{false};
  // follower_lag wait (spec §3.4): no MoveGroup move, only resume once the
  // real arm has closed the gap for a stable window.
  double & follow_resume_gap_rad_{params_.follow_resume_gap_rad};
  int64_t & follow_resume_stable_ms_{params_.follow_resume_stable_ms};
  int64_t follow_resume_since_ns_{0};
  bool follow_resume_requested_{false};
  std::thread recovery_thread_;
  // Scan worker state (atomic: the 100 Hz tick, on_track_array and the
  // worker all touch it).
  std::atomic_bool scan_in_progress_{false};
  std::atomic_bool scan_target_found_{false};
  std::atomic_bool scan_cancel_{false};
  std::atomic_bool skip_ingress_for_test_{false};
  std::atomic_bool auto_resume_requested_{false};
  std::thread scan_thread_;
  // SCANNING is a state: the worker keeps sweeping (alternating direction
  // each cycle) until a target appears or the session ends.  There is no
  // attempt counter and no cooldown.
  // True for a BT-started scan-only session (no target): retries must not
  // require a live tracker stream.
  bool scan_session_active_{false};
  int64_t scan_started_ns_{0};
  int64_t last_base_sample_ns_{0};
  tf2::Vector3 last_base_pos_{0, 0, 0};
  tf2::Vector3 recovery_target_;
  // Zero-twist probe used to prove the re-seed took (spec §7 Step 3).
  int64_t recovery_probe_armed_ns_{0};
  bool recovery_probe_got_{false};
  std::array<double, kArmJointCount> recovery_probe_q_{};
  bool needs_preposition_{false};
  double max_joint_delta_{0.0};
  double posture_solve_ms_sum_{0.0};
  double posture_solve_ms_max_{0.0};
  int posture_solve_count_{0};
  ControlLawParams & control_params_{params_.control};
  double & min_observe_distance_m_{params_.min_observe_distance_m};
  double & arm_source_scale_{params_.arm_source_scale};
  int64_t & tracker_stream_timeout_ms_{params_.tracker_stream_timeout_ms};
  // SCANNING configuration.
  bool & auto_scan_{params_.auto_scan};
  int & scan_start_delay_ms_{params_.scan_start_delay_ms};
  bool & auto_resume_on_target_{params_.auto_resume_on_target};
  bool & scan_require_base_static_{params_.scan_require_base_static};
  // 扫描入位前的"指令/实测对齐"门限, 独立于 recovery 的 0.005 rad
  // (后者物理上不可达: 正常跟随差 0.1-0.5 rad, 于是每次扫描白等 3 s 超时)。
  double & scan_presettle_gap_rad_{params_.scan_presettle_gap_rad};
  double & scan_velocity_scaling_{params_.scan_velocity_scaling};
  double & scan_accel_scaling_{params_.scan_accel_scaling};
  double & scan_tolerance_rad_{params_.scan_tolerance_rad};
  double & scan_hold_s_{params_.scan_hold_s};
  bool & roll_lock_enable_{params_.scan_roll_lock_enable};
  bool & camera_tf_check_enable_{params_.camera_tf_check_enable};
  std::string & camera_tf_frame_{params_.camera_tf_frame};
  double & camera_tf_pos_tol_m_{params_.camera_tf_pos_tol_m};
  double & camera_tf_ang_tol_rad_{params_.camera_tf_ang_tol_rad};
  bool camera_tf_checked_{false};
  ScanCircleParams & scan_circle_params_{params_.scan};

  // ===== calibration =======================================================
  bool calibration_ok_{false};
  tf2::Matrix3x3 R_cam_in_tcp_;
  tf2::Vector3 t_cam_in_tcp_;

  // ===== ROS handles =======================================================
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr start_client_group_;
  rclcpp::CallbackGroup::SharedPtr servo_sync_group_;

  rclcpp::Subscription<yoloe_interfaces::msg::UnifiedTrackArray>::SharedPtr track_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_watch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follow_ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    servo_output_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr move_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr ft_status_sub_;

  rclcpp::Publisher<lgw_bt_planner::msg::ServoState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  // SCANNING writes its trajectory straight to the JTC (the topic Servo
  // itself writes; Servo is paused for the whole scan).
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    scan_traj_pub_;

  rclcpp::Service<lgw_bt_planner::srv::Takeover>::SharedPtr takeover_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unpause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reseed_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ===== mutable state (guarded by mutex_) =================================
  std::mutex mutex_;
  std::mutex servo_call_mutex_;  // serialises the blocking Servo calls

  ServoStateMachine sm_;
  // Owns the release / pause / resume sequencing (Task 8). Constructed with
  // *this as its SessionHost. Named servo_session_ because `session_` is the
  // SessionState the control tick carries.
  std::unique_ptr<ServoSession> servo_session_;
  StreamWatchdog watchdog_;
  // The per-tick control pipeline (Task 6). It owns the control-law filter and
  // the feed-forward memory, so `reset_session` only has to say "new session".
  TrackingLoop loop_;
  SessionState session_;

  yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr latest_array_;
  TrackSnapshot selected_snapshot_;
  bool has_selected_snapshot_{false};
  // Diagnostic/test counter: reselects that had to drop the previous target's
  // control-law memory.
  uint32_t reselect_loop_reset_count_{0};

  int64_t array_recv_ns_{0};
  int64_t pose_recv_ns_{0};
  bool pose_seen_{false};

  // --- following monitor (spec §5.5) ---------------------------------------
  bool & follow_check_enabled_{params_.follow_check_enabled};
  std::string & follow_reference_topic_{params_.follow_reference_topic};
  double & follow_tolerance_rad_{params_.follow_tolerance_rad};
  int & follow_timeout_ms_{params_.follow_timeout_ms};
  std::array<double, kArmJointCount> measured_positions_{};
  // Which joints a /feedback message has ever named (see measured_complete_locked).
  std::array<bool, kArmJointCount> measured_joint_seen_{};
  // Joint velocities off the same message. Only meaningful when pose_watch is
  // the REAL arm: with a mock, pose_watch echoes the command and damping
  // against it would slow the open loop for no benefit. `damping_active_`
  // gates that at startup.
  std::array<double, kArmJointCount> measured_velocities_{};
  bool measured_vel_valid_{false};
  bool damping_active_{false};
  // true: tcp_now = FK(commanded joints) (decision A'); false: TF (real arm).
  bool tcp_pose_domain_command_{true};
  std::array<double, kArmJointCount> commanded_positions_{};
  bool measured_valid_{false};
  bool commanded_valid_{false};
  int64_t commanded_recv_ns_{0};
  int64_t follow_bad_since_ns_{0};
  int64_t follow_warn_last_ns_{0};

  action_msgs::msg::GoalStatusArray::SharedPtr move_status_;
  action_msgs::msg::GoalStatusArray::SharedPtr ft_status_;

  int64_t servo_output_recv_ns_{0};
  int64_t servo_halt_first_ns_{0};
  int64_t servo_halt_last_ns_{0};

  StartState start_state_{StartState::NOT_STARTED};
  int64_t start_request_ns_{0};
  int64_t start_response_ns_{0};
  int64_t last_start_probe_ns_{0};
  bool pause_on_confirm_requested_{false};

  // Last values reported on /lgw/servo/state (spec §7.2 / §7.6).
  double angular_error_rad_{0.0};
  double position_error_m_{0.0};
  double target_distance_m_{0.0};
  bool converged_{false};
  bool saturated_{false};
  bool too_close_{false};
  // HOLD recovery bookkeeping (spec 2026-11-03). hold_reason_ is captured at
  // the TRACKING->HOLD trigger site because the state machine only sees
  // TARGET_LOST.
  HoldReason hold_reason_{HoldReason::kNone};
  int64_t hold_since_ns_{0};
  std::array<double, kArmJointCount> last_sent_joints_{};
  bool last_sent_joints_valid_{false};
  std::string geometry_source_{"none"};
  uint64_t tracking_track_id_{0};

  // "angular error not decreasing for 3 s" window.
  bool sat_has_best_{false};
  double sat_best_rad_{0.0};
  int64_t sat_best_ns_{0};
  // Command-side progress reference for the unreachable guard: the commanded
  // joint vector and the time it last stopped advancing.
  std::array<double, kArmJointCount> sat_cmd_ref_{};
  int64_t sat_cmd_still_ns_{0};

  uint64_t tick_count_{0};
  bool state_dirty_{true};
  ServoState last_published_state_{ServoState::IDLE};
  std::string last_published_source_{"none"};
};

}  // namespace lgw_bt_planner
