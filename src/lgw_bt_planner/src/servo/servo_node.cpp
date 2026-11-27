// src/servo/servo_node.cpp
//
// Construction, parameter loading and calibration for ServoNode.
//
// The bodies used to live in servo_node.hpp (2026-11-12 header split): every
// translation unit that included the header re-parsed ~2300 lines of
// implementation. Only whitespace and the ServoNode:: qualifier changed when
// they moved here.

#include "lgw_bt_planner/servo/servo_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // parse_calibration_json
#include "lgw_bt_planner/servo_node_params.hpp"         // ParamListener

namespace lgw_bt_planner
{

ServoNode::ServoNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lgw_servo_node", options)
{
  declare_and_load_parameters();

  base_frame_ = frame_prefix_ + "base_link";
  tcp_frame_ = frame_prefix_ + "tcp_link";

  load_calibration();

  // tf2: dedicated listener thread, so the 100 Hz timer never spins tf and
  // every lookup below can be a zero-timeout probe (spec §7.7).
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Async start_servo client: auto-added so its response is handled by the
  // main executor while the timer keeps ticking.
  start_client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Blocking pause/unpause clients: NOT auto-added. Each call spins a local
  // executor over this group only (see file header).
  servo_sync_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  state_pub_ = create_publisher<lgw_bt_planner::msg::ServoState>(
    kStateTopic, rclcpp::QoS(10));
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    kTwistTopic, rclcpp::SystemDefaultsQoS());
  scan_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    kServoOutputTopic, rclcpp::SystemDefaultsQoS());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_group_;

  track_sub_ = create_subscription<yoloe_interfaces::msg::UnifiedTrackArray>(
    kTrackTopic, rclcpp::QoS(10).reliable(),
    std::bind(&ServoNode::on_track_array, this, std::placeholders::_1),
    sub_options);
  // Only message arrival is used (liveness, spec §7.8). Best effort keeps
  // us compatible with either a reliable or a best-effort publisher.
  pose_watch_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    pose_watch_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ServoNode::on_pose_watch, this, std::placeholders::_1),
    sub_options);
  // Commanded (JTC/mock) joints, for the following monitor (spec §5.5).
  follow_ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    follow_reference_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ServoNode::on_follow_reference, this, std::placeholders::_1),
    sub_options);
  servo_status_sub_ = create_subscription<std_msgs::msg::Int8>(
    kServoStatusTopic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ServoNode::on_servo_status, this, std::placeholders::_1),
    sub_options);
  // Servo's output topic: arrival proves its calc thread is really running
  // (the only trustworthy start_servo confirmation, spec §7.9).
  servo_output_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    kServoOutputTopic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ServoNode::on_servo_output, this, std::placeholders::_1),
    sub_options);
  move_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    kMoveActionStatusTopic, rclcpp::QoS(10),
    std::bind(&ServoNode::on_move_status, this, std::placeholders::_1),
    sub_options);
  ft_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    kFtActionStatusTopic, rclcpp::QoS(10),
    std::bind(&ServoNode::on_ft_status, this, std::placeholders::_1),
    sub_options);

  // Task 8: the lifecycle services live in ServoSession; it takes `this` as
  // its SessionHost, so it must be built after the state machine and the
  // clients exist but before the first service call can arrive.
  servo_session_ = std::make_unique<ServoSession>(*this);

  takeover_srv_ = create_service<lgw_bt_planner::srv::Takeover>(
    kTakeoverService,
    std::bind(
      &ServoNode::handle_takeover, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);
  release_srv_ = create_service<std_srvs::srv::Trigger>(
    kReleaseService,
    std::bind(
      &ServoNode::handle_release, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);
  pause_srv_ = create_service<std_srvs::srv::Trigger>(
    kPauseService,
    std::bind(&ServoNode::handle_pause, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);
  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    kResumeService,
    std::bind(&ServoNode::handle_resume, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);

  start_client_ = create_client<std_srvs::srv::Trigger>(
    kStartServoService, rmw_qos_profile_services_default, start_client_group_);
  pause_client_ = create_client<std_srvs::srv::Trigger>(
    kPauseServoService, rmw_qos_profile_services_default, servo_sync_group_);
  unpause_client_ = create_client<std_srvs::srv::Trigger>(
    kUnpauseServoService, rmw_qos_profile_services_default, servo_sync_group_);
  // Blocking start_servo, used only to re-seed Servo's cached command at
  // takeover (see handle_takeover). Deliberately separate from the async
  // boot-path start_client_ on start_client_group_: this one must NOT be
  // auto-added to the main executor, otherwise the local executor inside
  // call_trigger_sync would add the same group twice.
  reseed_client_ = create_client<std_srvs::srv::Trigger>(
    kStartServoService, rmw_qos_profile_services_default, servo_sync_group_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ServoNode::on_timer, this), timer_group_);

  RCLCPP_INFO(
    get_logger(),
    "lgw_servo_node up: frames %s <- %s, odom=%s, pose_watch=%s, "
    "calibration=%s, params: solver=%s min_observe=%.2f",
    base_frame_.c_str(), tcp_frame_.c_str(), kOdomFrame,
    pose_watch_topic_.c_str(), calibration_ok_ ? "ok" : "FAILED",
    solver_ == &posture_ ? "dls" : "qp", min_observe_distance_m_);
}

ServoNode::~ServoNode()
{
  // Never leave a worker touching members after destruction. Cancel the
  // MoveGroup goal first so move_to() returns promptly, then join.
  recovery_cancel_.store(true);
  if (prepositioner_) {
    prepositioner_->cancel();
  }
  if (recovery_thread_.joinable()) {
    recovery_thread_.join();
  }
  scan_cancel_.store(true);
  if (prepositioner_) {
    prepositioner_->cancel();
  }
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }
}

void ServoNode::declare_and_load_parameters()
{
  // Every name, type, default and range lives in
  // config/servo_node_parameters.yaml (Task 4). ParamListener declares them
  // all up front and validates them: an out-of-range value throws HERE, at
  // startup, instead of being clamped or defaulted silently. The shipped
  // robot tuning is still config/servo_node.yaml, layered on top of the
  // schema defaults; the two files can no longer disagree about a name.
  param_listener_ = std::make_shared<servo_params::ParamListener>(this);
  const servo_params::Params & p = param_listener_->get_params();

  frame_prefix_ = p.frame_prefix;
  pose_watch_topic_ = p.pose_watch_topic;
  // Which domain `tcp_now` (the control law's "current pose") comes from.
  // command (default): FK of follow_reference_topic's joints, the SAME domain
  //   moveit_servo integrates its deltas in. This is decision A'
  //   (2026-10-28) applied to the look-at law: the command may lead the real
  //   arm. The mock backend behaved this way implicitly (its TF WAS the
  //   command) and never rang; the topic backend exposed the mismatch.
  // measured: TF, i.e. the real arm. Use it to A/B the domains.
  const std::string & tcp_domain = p.tcp_pose_domain;
  tcp_pose_domain_command_ = tcp_domain != "measured";
  if (tcp_domain != "command" && tcp_domain != "measured") {
    RCLCPP_WARN(
      get_logger(), "unknown tcp_pose_domain '%s'; using command",
      tcp_domain.c_str());
  }
  calibration_path_ = p.arm_calibration_path;

  // Following monitor (spec §5.5). `follow_reference_topic` is the JTC/mock
  // domain that Servo integrates on; `pose_watch_topic` is the real arm that
  // TF (and therefore the lookat error) is derived from.
  follow_check_enabled_ = p.follow_check_enabled;
  follow_reference_topic_ = p.follow_reference_topic;
  follow_tolerance_rad_ = p.follow_tolerance_rad;
  follow_timeout_ms_ = static_cast<int>(p.follow_timeout_ms);

  // The look-at setpoint is the pinned-posture family -- the only rule left
  // (2026-11-10). The legacy base-relative orbit and the target-relative
  // standoff were removed: neither used IK, and the family has been the only
  // rule in production since spec 2026-10-30 rev2.
  // Pinned-posture family (spec 2026-10-30 rev2): j3/j5 pinned, j1/j4/j6
  // solved, j2 searched. Only initialised when the mode selects it.
  posture_params_.posture_j3_rad = p.posture_j3_rad;
  posture_params_.posture_j5_rad = p.posture_j5_rad;
  posture_params_.posture_j3_alt_rad = p.posture_j3_alt_rad;
  posture_params_.posture_j5_alt_rad = p.posture_j5_alt_rad;
  posture_params_.posture_j2_step_rad = p.posture_j2_step_rad;
  posture_params_.posture_j2_nominal_rad = p.posture_j2_nominal_rad;
  posture_params_.posture_j2_min_rad = p.posture_j2_min_rad;
  posture_params_.posture_j2_max_rad = p.posture_j2_max_rad;
  posture_params_.cam_min_height_m = p.posture_cam_min_height_m;
  posture_params_.cam_max_radius_m = p.posture_cam_max_radius_m;
  posture_params_.cam_min_radius_m = p.posture_cam_min_radius_m;
  posture_params_.cam_frontal_sector_rad = p.posture_cam_frontal_sector_rad;
  posture_params_.cam_frontal_taper_rad = p.posture_cam_frontal_taper_rad;
  posture_params_.cam_floor_distance_near_m =
    p.posture_cam_floor_distance_near_m;
  posture_params_.cam_floor_distance_far_m =
    p.posture_cam_floor_distance_far_m;
  posture_params_.joint_margin = p.posture_joint_margin;
  posture_params_.cond_max = p.posture_cond_max;
  // Diagnostics only (2026-11-07): trace why each tick's wrist branch was
  // chosen. Default off; a 5 Hz line is noise in normal operation.
  posture_trace_ = p.posture_trace;
  posture_params_.aim_tol_rad = p.posture_aim_tol_rad;
  posture_params_.max_solve_us = p.posture_max_overrun_us;
  preposition_params_.enable = p.preposition_enable;
  preposition_params_.joint_delta_rad = p.preposition_joint_delta_rad;
  preposition_params_.branch_delta_rad = p.preposition_branch_delta_rad;
  preposition_params_.limit_margin_rad = p.preposition_limit_margin_rad;
  preposition_params_.tcp_distance_m = p.preposition_tcp_distance_m;
  preposition_params_.tcp_angle_rad = p.preposition_tcp_angle_rad;
  preposition_velocity_scaling_ = p.preposition_velocity_scaling;
  preposition_accel_scaling_ = p.preposition_accel_scaling;
  preposition_tolerance_rad_ = p.preposition_tolerance_rad;
  preposition_timeout_s_ = p.preposition_timeout_s;

  const std::string & robot_description = p.robot_description;
  std::string posture_error;
  // URDF link names carry no TF prefix (robot_state_publisher adds
  // "arm1/" to the published frames, not to the description).
  if (!posture_.initialize(
      robot_description, /*is_file=*/false, "base_link", "tcp_link",
      posture_error))
  {
    RCLCPP_ERROR(
      get_logger(), "pinned_posture init failed (%s); node enters FAULT",
      posture_error.c_str());
    calibration_ok_ = false;
  }

  // Solver selection is a STARTUP decision (2026-11-05): no runtime
  // switching and no per-tick fallback. An unknown value keeps DLS so a
  // typo cannot take the arm offline.
  const std::string & solver_name = p.lookat_solver;
  const SolverKind kind = parse_solver_kind(solver_name, SolverKind::kDls);
  if (kind == SolverKind::kDls && solver_name != "dls") {
    RCLCPP_WARN(
      get_logger(), "unknown lookat_solver '%s'; using dls",
      solver_name.c_str());
  }
  solver_ = &posture_;
  if (kind == SolverKind::kQp) {
    qp_params_.w_aim = p.qp_w_aim;
    qp_params_.w_smooth = p.qp_w_smooth;
    qp_params_.w_posture = p.qp_w_posture;
    qp_params_.j5_branch_min_rad = p.qp_j5_branch_min_rad;
    qp_params_.cond_max_relaxed = p.qp_cond_max_relaxed;
    qp_params_.relaxed_min_azimuth_rad = p.qp_relaxed_min_azimuth_rad;
    qp_params_.joint_margin_rad = p.qp_joint_margin_rad;
    qp_params_.j5_branch_max_rad = p.qp_j5_branch_max_rad;
    qp_params_.prefer_positive_branch = p.qp_prefer_positive_branch;
    // SQP outer iterations. Measured 2026-11-09: 4 -> 77 % coverage,
    // 8 -> 95 %, 12 -> 97.6 %, saturated by 16. Mean solve 0.23 -> 0.47 ms,
    // max 0.58 -> 3.05 ms (budget 4 ms).
    qp_params_.max_linearizations =
      static_cast<int>(p.qp_max_linearizations);
    // Radius-vs-depression coupling (2026-11-09). Off at 0; the offline
    // sweep (tests/test_qp_j5_sweep.cpp) shows 0.40 raises the camera
    // radius 35-45 % at close range with no coverage loss on the grid.
    qp_params_.w_radius = p.qp_w_radius;
    qp_params_.radius_near_m = p.qp_radius_near_m;
    qp_params_.radius_steep_m = p.qp_radius_steep_m;
    qp_params_.radius_side_m = p.qp_radius_side_m;
    qp_solver_ = std::make_unique<QpPostureSolver>();
    std::string qp_error;
    if (qp_solver_->initialize(
        robot_description, /*is_file=*/false, "base_link", "tcp_link",
        qp_error))
    {
      qp_solver_->set_params(qp_params_);
      solver_ = qp_solver_.get();
    } else {
      RCLCPP_ERROR(
        get_logger(), "qp lookat init failed (%s); using dls",
        qp_error.c_str());
    }
  }
  RCLCPP_INFO(
    get_logger(), "lookat solver: %s", solver_ == &posture_ ? "dls" : "qp");
  prepositioner_ = std::make_unique<MoveGroupPrepositioner>(
    *this, "/move_action", "arm");

  tracker_stream_timeout_ms_ = p.tracker_stream_timeout_ms;
  arm_source_scale_ = p.arm_source_scale;

  auto_scan_ = p.auto_scan;
  scan_start_delay_ms_ = static_cast<int>(p.scan_start_delay_ms);
  auto_resume_on_target_ = p.auto_resume_on_target;
  scan_require_base_static_ = p.scan_require_base_static;
  scan_presettle_gap_rad_ = p.scan_presettle_gap_rad;
  scan_velocity_scaling_ = p.scan_velocity_scaling;
  scan_accel_scaling_ = p.scan_accel_scaling;
  scan_tolerance_rad_ = p.scan_tolerance_rad;
  scan_hold_s_ = p.scan_hold_s;
  roll_lock_enable_ = p.scan_roll_lock_enable;
  camera_tf_check_enable_ = p.camera_tf_check_enable;
  camera_tf_frame_ = p.camera_tf_frame;
  camera_tf_pos_tol_m_ = p.camera_tf_pos_tol_m;
  camera_tf_ang_tol_rad_ = p.camera_tf_ang_tol_rad;
  scan_circle_params_.j1_rad = p.scan_j1_rad;
  scan_circle_params_.j2_rad = p.scan_j2_rad;
  scan_circle_params_.j3_rad = p.scan_j3_rad;
  scan_circle_params_.j4_radius_rad = p.scan_j4_radius_rad;
  scan_circle_params_.j5_radius_rad = p.scan_j5_radius_rad;
  scan_circle_params_.period_s = p.scan_period_s;
  scan_circle_params_.duration_s = p.scan_traj_duration_s;
  scan_circle_params_.sample_rate_hz = p.scan_traj_sample_rate_hz;
  scan_circle_params_.max_velocity_rad_s = p.scan_traj_max_velocity_rad_s;

  control_params_.kp_pos = p.kp_pos;
  control_params_.kp_ang = p.kp_ang;
  control_params_.v_max = p.v_max;
  control_params_.w_max = p.w_max;
  control_params_.deadband_pos_m = p.deadband_pos_m;
  control_params_.deadband_ang_rad = p.deadband_ang_rad;
  control_params_.lowpass_cutoff_hz = p.lowpass_cutoff_hz;
  // Reference-velocity feed-forward (2026-11-05). The setpoint derivative
  // is computed here (consecutive desired tcp poses), filtered with its own
  // cutoff, then added inside compute_twist.
  params_.ff_enable = p.ff_enable;
  params_.ff_lowpass_hz = p.ff_lowpass_hz;
  params_.ff_deadband_mps = p.ff_deadband_mps;
  params_.ff_deadband_radps = p.ff_deadband_radps;
  params_.ff_max_mps = p.ff_max_mps;
  params_.ff_max_radps = p.ff_max_radps;
  control_params_.v_max_total = p.v_max_total;
  control_params_.w_max_total = p.w_max_total;
  // Measured-velocity damping (2026-11-06). The topic hardware backend
  // closes the loop on the REAL arm pose, which lags the command by ~0.3 s;
  // kp 2.0/3.0 then overshoots ~13 % and rings (measured on the robot).
  // kd has units of seconds and is only applied when pose_watch is the real
  // arm -- damping against a mock echo would slow the open loop for nothing.
  control_params_.kd_pos = p.kd_pos;
  control_params_.kd_ang = p.kd_ang;
  const bool real_pose_watch =
    pose_watch_topic_ == "/feedback/joint_states" ||
    pose_watch_topic_ == "feedback/joint_states";
  // Damping only makes sense in the MEASURED domain: it exists to buy phase
  // margin against the real arm's lag. With tcp_pose_domain=command the
  // loop is open again (tcp_now is the command itself), there is no lag to
  // compensate, and damping against the real arm would inject a foreign
  // signal into an otherwise open loop.
  damping_active_ = real_pose_watch && !tcp_pose_domain_command_ &&
    (control_params_.kd_pos > 0.0 || control_params_.kd_ang > 0.0);
  RCLCPP_INFO(
    get_logger(),
    "control law: tcp_pose_domain=%s, damping=%s (kd_pos %.2f, kd_ang %.2f, pose_watch %s)",
    tcp_pose_domain_command_ ? "command" : "measured",
    damping_active_ ? "on" : "off", control_params_.kd_pos,
    control_params_.kd_ang, pose_watch_topic_.c_str());
  control_params_.loop_rate_hz = 100.0;  // matches the 10 ms timer below

  min_observe_distance_m_ = p.min_observe_distance_m;


  // HOLD recovery (spec 2026-11-03 §8). Enabled by default; false restores
  // the pre-2026-11-03 behaviour exactly.
  recovery_params_.enable = p.hold_recovery_enable;
  recovery_params_.allow_target_lost = p.recovery_allow_target_lost;
  recovery_params_.max_attempts = static_cast<int>(p.recovery_max_attempts);
  recovery_params_.deadline_ms = p.recovery_deadline_ms;
  recovery_params_.geometry_max_age_ms = p.recovery_geometry_max_age_ms;
  recovery_params_.follower_timeout_ms = p.follow_recovery_timeout_ms;
  recovery_params_.retry_cooldown_ms = p.recovery_retry_cooldown_ms;
  recovery_velocity_scaling_ = p.recovery_velocity_scaling;
  recovery_accel_scaling_ = p.recovery_accel_scaling;
  recovery_timeout_s_ = p.recovery_timeout_s;
  recovery_tolerance_rad_ = p.recovery_tolerance_rad;
  recovery_settle_ms_ = static_cast<int>(p.recovery_settle_ms);
  // follow:=true: MoveGroup plans from /feedback while the JTC holds
  // /control. HOLD freezes the JTC, so wait for the real arm to catch up
  // (gap < this) before sending the goal; otherwise the JTC rejects the
  // trajectory on allowed_start_tolerance.
  recovery_presettle_gap_rad_ = p.recovery_presettle_gap_rad;
  recovery_presettle_timeout_ms_ = p.recovery_presettle_timeout_ms;
  // The gap must stay below the threshold for this long before the goal is
  // sent. A single-instant check raced the still-converging arm: the JTC
  // then rejected the trajectory on allowed_start_tolerance (measured
  // 2026-11-05: joint1 drifted 0.016 rad in the ~150 ms between the check
  // and the validation).
  recovery_presettle_stable_ms_ = p.recovery_presettle_stable_ms;
  // follower_lag recovery (spec 2026-11-03 §3.4): freeze and wait for the
  // real arm to close the gap; never move.
  follow_resume_gap_rad_ = p.follow_resume_gap_rad;
  follow_resume_stable_ms_ = p.follow_resume_stable_ms;
}

void ServoNode::load_calibration()
{
  std::string path = calibration_path_;
  try {
    if (path.empty()) {
      path = ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
        "/calibration/arm_calibration.json";
    }
    std::tie(R_cam_in_tcp_, t_cam_in_tcp_) = parse_calibration_json(path);
    calibration_ok_ = true;
    RCLCPP_INFO(
      get_logger(), "loaded arm hand-eye calibration: %s  t_cam_in_tcp=(%.4f,%.4f,%.4f)",
      path.c_str(), t_cam_in_tcp_.x(), t_cam_in_tcp_.y(), t_cam_in_tcp_.z());
  } catch (const std::exception & e) {
    calibration_ok_ = false;
    R_cam_in_tcp_ = tf2::Matrix3x3();
    t_cam_in_tcp_ = tf2::Vector3(0.0, 0.0, 0.0);
    sm_.handle(ServoEvent::FAULT);
    RCLCPP_ERROR(
      get_logger(),
      "failed to load arm calibration (%s): %s — node enters FAULT, "
      "takeover refused", path.c_str(), e.what());
  }
}


void ServoNode::on_track_array(const yoloe_interfaces::msg::UnifiedTrackArray::SharedPtr msg)
{
  const int64_t now_ns = now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);

  latest_array_ = msg;
  array_recv_ns_ = now_ns;
  // Liveness is MESSAGE ARRIVAL time: the tracker's own time_since_last_obs
  // freezes when the pipeline dies (spec §7.4 / StreamWatchdog docs).
  watchdog_.on_track_array(now_ns);

  for (const auto & track : msg->tracks) {
    if (!track.geometry_frame.empty() && track.geometry_frame != base_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "tracker geometry_frame='%s' differs from our base frame '%s'; "
        "refusing to drive. Check arm_frame_prefix on both sides.",
        track.geometry_frame.c_str(), base_frame_.c_str());
      has_selected_snapshot_ = false;
      // Do NOT classify this as target_lost: that would allow SCANNING on a
      // configuration error.
      hold_and_return(HoldReason::kPoseStale, now_ns);
      return;
    }
  }

  if (!session_active_locked()) {
    has_selected_snapshot_ = false;
    return;
  }
  reselect_locked();
  if (sm_.state() == ServoState::SCANNING &&
    has_selected_snapshot_ && selected_snapshot_.geometry_valid)
  {
    scan_target_found_.store(true);
  }
}

void ServoNode::on_pose_watch(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_recv_ns_ = now().nanoseconds();
  pose_seen_ = true;
  // TF carries the pose for the control law; these samples exist only so the
  // following monitor can compare against the commanded (JTC/mock) joints.
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    for (size_t j = 0; j < kArmJointCount; ++j) {
      if (msg->name[i] == kArmJoints[j]) {
        measured_positions_[j] = msg->position[i];
        measured_valid_ = true;
        measured_joint_seen_[j] = true;
        if (i < msg->velocity.size()) {
          measured_velocities_[j] = msg->velocity[i];
          measured_vel_valid_ = true;
        }
      }
    }
  }
}

bool ServoNode::measured_complete_locked() const
{
  for (size_t j = 0; j < kArmJointCount; ++j) {
    if (!measured_joint_seen_[j]) {
      return false;
    }
  }
  return true;
}

void ServoNode::on_follow_reference(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    for (size_t j = 0; j < kArmJointCount; ++j) {
      if (msg->name[i] == kArmJoints[j]) {
        commanded_positions_[j] = msg->position[i];
        commanded_valid_ = true;
      }
    }
  }
  commanded_recv_ns_ = now().nanoseconds();
}

double ServoNode::follow_gap_locked() const
{
  if (!measured_valid_ || !commanded_valid_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double worst = 0.0;
  for (size_t j = 0; j < kArmJointCount; ++j) {
    worst = std::max(worst, std::fabs(commanded_positions_[j] - measured_positions_[j]));
  }
  return worst;
}

bool ServoNode::commanded_advancing_locked()
{
  if (!commanded_valid_) {
    return false;
  }
  double worst = 0.0;
  for (size_t j = 0; j < kArmJointCount; ++j) {
    worst = std::max(
      worst, std::fabs(commanded_positions_[j] - sat_cmd_ref_[j]));
  }
  if (worst >= kSatCmdProgressRad) {
    for (size_t j = 0; j < kArmJointCount; ++j) {
      sat_cmd_ref_[j] = commanded_positions_[j];
    }
    return true;
  }
  return false;
}

void ServoNode::on_servo_status(const std_msgs::msg::Int8::SharedPtr msg)
{
  const int64_t now_ns = now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);

  const bool halt = msg->data == kServoStatusHaltSingularity ||
    msg->data == kServoStatusHaltCollision ||
    msg->data == kServoStatusJointBound;
  if (halt) {
    if (servo_halt_first_ns_ == 0) {
      servo_halt_first_ns_ = now_ns;
    }
    servo_halt_last_ns_ = now_ns;
  } else if (servo_halt_first_ns_ != 0 &&
    now_ns - servo_halt_last_ns_ > kServoHaltResetNs)
  {
    // Servo clears its status after every publish (servo_calcs.cpp), so
    // tolerate interleaved NO_WARNING messages; only a quiet second ends
    // the halt window.
    servo_halt_first_ns_ = 0;
  }
}

void ServoNode::on_servo_output(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Arrival proves Servo's calc loop runs; during a recovery probe it also
  // carries the re-seeded cache we must compare against the measured pose.
  if (recovery_probe_armed_ns_ != 0 && !msg->points.empty()) {
    const auto & pos = msg->points[0].positions;
    if (pos.size() >= kArmJointCount) {
      bool mapped = false;
      if (msg->joint_names.size() >= kArmJointCount) {
        for (std::size_t j = 0; j < kArmJointCount; ++j) {
          for (std::size_t k = 0; k < msg->joint_names.size(); ++k) {
            if (msg->joint_names[k] == kArmJoints[j] && k < pos.size()) {
              recovery_probe_q_[j] = pos[k];
              mapped = true;
              break;
            }
          }
        }
      }
      if (!mapped) {
        for (std::size_t j = 0; j < kArmJointCount; ++j) {
          recovery_probe_q_[j] = pos[j];
        }
      }
      recovery_probe_got_ = true;
    }
  }
  servo_output_recv_ns_ = now().nanoseconds();
}

void ServoNode::on_move_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  move_status_ = msg;
}

void ServoNode::on_ft_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ft_status_ = msg;
}

void ServoNode::on_start_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  const auto response = future.get();
  const int64_t now_ns = now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);

  if (start_state_ == StartState::FAILED) {
    RCLCPP_WARN(
      get_logger(), "start_servo response arrived after FAULT (success=%s); ignored",
      response->success ? "true" : "false");
    return;
  }
  if (!response->success) {
    start_state_ = StartState::FAILED;
    sm_.handle(ServoEvent::FAULT);
    RCLCPP_ERROR(
      get_logger(), "start_servo returned success=false — state -> FAULT");
    return;
  }
  if (start_state_ == StartState::REQUESTED) {
    // Do NOT trust this: the response is always success (spec §7.9). Publish
    // a zero twist and require Servo's own output within 300 ms.
    start_state_ = StartState::CONFIRMING;
    start_response_ns_ = now_ns;
    RCLCPP_INFO(
      get_logger(),
      "start_servo returned success; verifying Servo runs via %s",
      kServoOutputTopic);
  }
}

void ServoNode::on_timer()
{
  tick(now().nanoseconds());
}

std::string ServoNode::state_for_test()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_name(sm_.state());
}

void ServoNode::log_info(const std::string & message)
{
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void ServoNode::log_warn(const std::string & message)
{
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

void ServoNode::log_error(const std::string & message)
{
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

bool ServoNode::session_active_locked() const
{
  const ServoState state = sm_.state();
  return state == ServoState::PREEMPT || state == ServoState::TRACKING ||
         state == ServoState::HOLD || state == ServoState::RECOVERING ||
         state == ServoState::SCANNING || state == ServoState::RELEASING;
}

TrackSnapshot ServoNode::empty_snapshot()
{
  TrackSnapshot snapshot;
  snapshot.track_id = 0;
  snapshot.task_state.clear();
  snapshot.source.clear();
  snapshot.p_odom = tf2::Vector3(0.0, 0.0, 0.0);
  snapshot.p_arm_base = tf2::Vector3(0.0, 0.0, 0.0);
  snapshot.geometry_valid = false;
  snapshot.last_seen_ns = 0;
  return snapshot;
}

void ServoNode::clear_session_locked()
{
  loop_.reset();
  cancel_scan();
  scan_in_progress_.store(false);
  scan_target_found_.store(false);
  auto_resume_requested_.store(false);
  scan_session_active_ = false;
  selected_snapshot_ = empty_snapshot();
  has_selected_snapshot_ = false;
  tracking_track_id_ = 0;
  geometry_source_ = "none";
  angular_error_rad_ = 0.0;
  position_error_m_ = 0.0;
  target_distance_m_ = 0.0;
  converged_ = false;
  saturated_ = false;
  too_close_ = false;
  hold_reason_ = HoldReason::kNone;
  hold_since_ns_ = 0;
  session_.recovery_attempts = 0;
  session_.recovery_exhausted = false;
  arm_saturation_window_locked();
  state_dirty_ = true;
}

void ServoNode::arm_saturation_window_locked()
{
  sat_has_best_ = false;
  sat_best_rad_ = 0.0;
  sat_best_ns_ = 0;
  sat_cmd_still_ns_ = 0;
  sat_cmd_ref_ = commanded_positions_;
  saturated_ = false;
}

bool ServoNode::move_group_busy_locked() const
{
  return status_array_busy(move_status_) || status_array_busy(ft_status_);
}

bool ServoNode::servo_halt_persisted_locked(int64_t now_ns) const
{
  if (servo_halt_first_ns_ == 0) {
    return false;
  }
  if (now_ns - servo_halt_last_ns_ > kServoHaltResetNs) {
    return false;  // the halt window already lapsed
  }
  return now_ns - servo_halt_first_ns_ >= kServoHaltWindowNs;
}

void ServoNode::reselect_locked()
{
  if (!latest_array_ || latest_array_->selected_track_id == 0) {
    has_selected_snapshot_ = false;
    tracking_track_id_ = 0;
    return;
  }
  const uint64_t selected_id = latest_array_->selected_track_id;
  const yoloe_interfaces::msg::UnifiedTrack * selected = nullptr;
  for (const auto & track : latest_array_->tracks) {
    if (track.track_id == selected_id) {
      selected = &track;
      break;
    }
  }
  if (selected == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "selected_track_id=%lu is not in the latest track array",
      static_cast<unsigned long>(selected_id));
    has_selected_snapshot_ = false;
    tracking_track_id_ = 0;
    return;
  }
  const uint64_t previous_track_id = tracking_track_id_;
  selected_snapshot_ = make_snapshot(*selected);
  if (previous_track_id != selected_id) {
    // A new target must not inherit the previous target's feed-forward /
    // control-law memory.  This also covers A -> 0 (HOLD) -> B, where
    // tracking_track_id_ was cleared to 0 while the target was gone.
    loop_.reset();
    ++reselect_loop_reset_count_;
  }
  has_selected_snapshot_ = true;
  tracking_track_id_ = selected_id;
}

TrackSnapshot ServoNode::make_snapshot(const yoloe_interfaces::msg::UnifiedTrack & track)
{
  TrackSnapshot snapshot;
  snapshot.track_id = track.track_id;
  snapshot.task_state = track.task_state;
  snapshot.source = track.source;
  snapshot.p_odom.setValue(track.position.x, track.position.y, track.position.z);
  snapshot.p_arm_base.setValue(
    track.position_arm_base.x, track.position_arm_base.y,
    track.position_arm_base.z);
  snapshot.geometry_valid = track.geometry_valid;
  snapshot.last_seen_ns = time_to_ns(track.last_observation_stamp);
  return snapshot;
}
}  // namespace lgw_bt_planner
