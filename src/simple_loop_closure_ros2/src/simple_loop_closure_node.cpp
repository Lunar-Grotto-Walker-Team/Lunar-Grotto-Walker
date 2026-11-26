#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <simple_loop_closure/simple_loop_closure_node.hpp>
#include <simple_loop_closure/loop_noise.hpp>
#include <simple_loop_closure/factors.hpp>
#include <simple_loop_closure/ground_diagnostics.hpp>
#include <simple_loop_closure/gravity_math.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include <boost/make_shared.hpp>

#include <pcl/io/pcd_io.h>

SimpleLoopClosureNode::SimpleLoopClosureNode() : Node("simple_loop_closure")
{
  initialize();
}

void SimpleLoopClosureNode::initialize()
{
  pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pgo_map_cloud", 1);
  pub_vis_pose_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vis_pose_graph", 1);
  pub_pgo_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/pgo_odom", 1);

  this->declare_parameter<bool>("mapped_cloud", true);
  this->declare_parameter<double>("time_stamp_tolerance", 0.01);
  this->declare_parameter<double>("keyframe_dist_th", 0.3);
  this->declare_parameter<double>("keyframe_angular_dist_th", 0.2);
  this->declare_parameter<double>("loop_search_time_diff_th", 30.0);
  this->declare_parameter<double>("loop_search_dist_diff_th", 30.0);
  this->declare_parameter<double>("loop_search_angular_dist_th", 3.14);
  this->declare_parameter<int>("loop_search_frame_interval", 1);
  this->declare_parameter<double>("search_radius", 15.0);
  this->declare_parameter<int>("target_frame_num", 50);
  this->declare_parameter<double>("target_voxel_leaf_size", 0.4);
  this->declare_parameter<double>("source_voxel_leaf_size", 0.4);
  this->declare_parameter<double>("vis_map_voxel_leaf_size", 0.8);
  this->declare_parameter<double>("fitness_score_th", 0.3);
  this->declare_parameter<int>("vis_map_cloud_frame_interval", 3);
  this->declare_parameter<bool>("vis_incremental", true);
  this->declare_parameter<double>("vis_full_rebuild_interval_s", 60.0);
  // 2026-11-12: 线程频率. 1 Hz 对快跑和下游避障都太慢; 先参数化, 用
  // 5 Hz A/B, CPU 不够再降 2 Hz.
  this->declare_parameter<double>("loop_rate_hz", 5.0);
  this->declare_parameter<double>("vis_rate_hz", 5.0);
  // 噪声模型 (方差单位): 里程计按每关键帧 ~1 cm / ~0.6 deg 建模;
  // 回环方差取 ICP 的 MSE (PCL getFitnessScore 就是均方距离), 并设 1e-4 下限.
  this->declare_parameter<double>("odom_trans_var", 1e-4);
  this->declare_parameter<double>("odom_rot_var", 1e-4);
  // 2026-11-13: 世界 z 连续性 (m^2). 回环修正 yaw/姿态后, 机体系里程计边
  // 会把平移重新投影到世界 z; 这个软因子把每步世界 Δz 拉回 LIO 的观测,
  // 保证"没有可靠 z 观测时, 后端不把探索区 z 改得比 LIO 更差".
  this->declare_parameter<double>("odom_z_diff_var", 0.0);
  this->declare_parameter<double>("loop_var_scale", 1.0);
  this->declare_parameter<double>("loop_var_min", 1e-4);
  // ICP 对应点最大距离(米). 不能沿用 search_radius_: 那个值在构造函数末尾被
  // 平方给 nanoflann 用, 再 x2 传给 ICP 就成了 200 m, 等于不限。
  this->declare_parameter<double>("icp_max_corr_dist", 1.5);
  // 回环边与里程计预测的平移差上限(米). 超过就认为 ICP 匹配到了别处的
  // 自相似结构, 而不是真正的回访。
  this->declare_parameter<double>("loop_max_trans_delta", 0.3);
  // 距离比例门限 (0 = 关闭, 只用上面的固定门限).
  this->declare_parameter<double>("loop_max_trans_delta_ratio", 0.10);
  // 2026-11-17: 平移一致性门限的**硬上限**(米). <=0 = 不设上限(旧行为).
  // 走廊/洞穴的墙面沿轴向自相似, ICP 的滑移量随基线长度增长; 只靠
  // ratio 时 45 m 基线可放行 2.7 m、148 m 基线可放行 8.9 m 的假边,
  // 图被拽歪后回环还会继续"验证"这个歪. 硬上限把这类边直接拒掉.
  this->declare_parameter<double>("loop_max_trans_delta_cap", 0.0);
  // 2026-11-09: 回环边 3-DOF 开关. 默认 false = 只约束 x/y/yaw.
  this->declare_parameter<bool>("loop_constrain_z", false);
  this->declare_parameter<bool>("loop_constrain_attitude", false);
  // 被放开 DOF 的方差 (m^2 / rad^2). 1e4 相对里程计 1e-4 弱 8 个数量级.
  this->declare_parameter<double>("loop_var_free", 1.0e4);
  // 2026-11-12 P1: 真 3-DOF 回环 (world xy + yaw) 与 P0 对角方差回退.
  this->declare_parameter<std::string>("loop_factor_mode", "projected_3dof");
  // 2026-11-13: 地面平面诊断（只写日志，不进图）。每 N 个关键帧一条；
  // 0 = 关闭。用于定位"过门后平地建坡"这类姿态偏置事件。
  this->declare_parameter<int>("ground_diag_interval", 0);
  // 2026-11-13 P2: 地面法向连续性因子（roll/pitch 相对约束）。跨无效地面段
  // 连接当前有效帧与上一个有效帧，门后第一帧把倾角拉回来。
  this->declare_parameter<bool>("enable_floor_normal_factor", false);
  this->declare_parameter<double>("floor_normal_var_deg", 2.0);
  this->declare_parameter<double>("floor_normal_max_step_deg", 10.0);
  this->declare_parameter<int>("floor_normal_max_gap_keyframes", 50);
  // 2026-11-11 P0: 回环候选的几何质量门控 (场景无关).
  this->declare_parameter<double>("overlap_min", 0.30);
  this->declare_parameter<double>("overlap_dist", 0.20);
  this->declare_parameter<double>("loop_max_yaw_delta", 0.52);
  this->declare_parameter<double>("loop_max_candidate_dz", 1.0);
  this->declare_parameter<int>("loop_dedup_window", 20);
  // 2026-11-15: world-frame odometry translation (decouples gravity-prior
  // attitude corrections from world xy).  Default off = previous behavior.
  this->declare_parameter<bool>("use_world_odom_trans", false);
  // 2026-11-09: Go2 相对 z 软因子 (默认关闭, 洞穴数据到位后再开).
  this->declare_parameter<bool>("enable_go2_z", false);
  this->declare_parameter<std::string>("go2_pose_topic", "/utlidar/robot_pose");
  this->declare_parameter<double>("go2_pose_timeout", 0.30);
  this->declare_parameter<double>("go2_z_var", 1.0e-4);
  // Cauchy scale in whitened-residual units.  1.0 was too aggressive for
  // fast gait; 5.0 keeps inlier residuals active.
  this->declare_parameter<double>("go2_z_robust_scale", 5.0);
  // 2026-11-09: Go2 roll/pitch 软先验. 2026-11-11 P0 起默认开启:
  // 3-DOF 回环让 z/roll/pitch 自由后, 没有姿态基准会与 z 互相拉扯.
  this->declare_parameter<bool>("enable_go2_attitude", true);
  this->declare_parameter<double>("go2_attitude_var_deg", 3.0);
  // 2026-11-13 P2: IMU 重力软先验 (roll/pitch absolute; yaw/translation free).
  // 每个源都先归约成"机体系比力方向", 不再做 rpy 分量相减.  与
  // enable_go2_attitude 语义重叠, 两者互斥 (见 useGo2Attitude).
  this->declare_parameter<bool>("enable_gravity_prior", false);
  this->declare_parameter<std::string>("gravity_source", "lowstate");
  this->declare_parameter<std::string>("gravity_topic", "");
  this->declare_parameter<double>("gravity_var_deg", 3.0);
  this->declare_parameter<double>("gravity_robust_scale", 3.0);
  this->declare_parameter<double>("gravity_timeout", 0.30);
  // Full averaging window (s).  A single accel sample under gait swings by
  // tens of degrees; 0.30 s cuts the pitch scatter from 8.3 to 2.9 deg.
  this->declare_parameter<double>("gravity_lowpass_s", 0.30);
  // Dynamic gate: reject a window whose mean |accel| deviates more than this
  // fraction from the running norm (stairs/jumps).  0 disables.
  this->declare_parameter<double>("gravity_norm_tol", 0.20);
  this->declare_parameter<std::string>("gravity_extrinsic_rpy", "");
  this->declare_parameter<double>("gravity_roll_offset", 0.0);
  this->declare_parameter<double>("gravity_pitch_offset", 0.0);
  this->declare_parameter<bool>("allow_gravity_z_diff_coexist", false);
  // 2026-11-15: leg-kinematic world-z factor (cave-proxy A/B profiles only).
  this->declare_parameter<bool>("enable_leg_z", false);
  this->declare_parameter<std::string>("leg_lowstate_topic", "/lf/lowstate");
  this->declare_parameter<std::string>("leg_sport_topic",
                                       "/lf/sportmodestate");
  // device: lowstate time from the sportmodestate device stamp, attitude from
  // the robot_pose header.  arrival: Jetson callback time (jitter-prone).
  this->declare_parameter<std::string>("leg_time_source", "device");
  this->declare_parameter<double>("leg_z_var", 4.0e-4);
  this->declare_parameter<double>("leg_z_robust_scale", 3.0);
  this->declare_parameter<double>("leg_z_timeout", 0.10);
  this->declare_parameter<double>("leg_attitude_jump_deg", 10.0);
  this->declare_parameter<double>("leg_contact_force_on", 25.0);
  this->declare_parameter<double>("leg_contact_force_off", 12.0);
  this->declare_parameter<double>("leg_single_force", 35.0);
  this->declare_parameter<double>("leg_settle_s", 0.10);
  this->declare_parameter<double>("leg_consensus_spread", 0.04);
  this->declare_parameter<double>("leg_max_step_dz", 0.06);
  this->declare_parameter<double>("leg_pitch_offset_deg", 1.3);
  this->declare_parameter<double>("leg_roll_offset_deg", 0.0);
  this->declare_parameter<double>("leg_offset_x", 0.1934);
  this->declare_parameter<double>("leg_offset_y", 0.0465);
  this->declare_parameter<double>("leg_thigh_offset", 0.0955);
  this->declare_parameter<double>("leg_thigh_len", 0.213);
  this->declare_parameter<double>("leg_calf_len", 0.213);

  // 2026-11-19: prior-map localization (default off; behaviour unchanged).
  this->declare_parameter<bool>("enable_prior_map", false);
  this->declare_parameter<std::string>("prior_map_dir", "");
  this->declare_parameter<double>("prior_search_radius", 1.5);
  this->declare_parameter<double>("prior_max_trans_delta", 1.0);
  this->declare_parameter<double>("prior_icp_max_corr_dist", 1.0);
  this->declare_parameter<double>("prior_overlap_min", 0.35);
  this->declare_parameter<int>("prior_dedup_window", 20);
  this->declare_parameter<double>("prior_var_scale", 1.0);

  this->get_parameter("mapped_cloud", mapped_cloud_);
  this->get_parameter("time_stamp_tolerance", time_stamp_tolerance_);
  this->get_parameter("keyframe_dist_th", keyframe_dist_th_);
  this->get_parameter("keyframe_angular_dist_th", keyframe_angular_dist_th_);
  this->get_parameter("loop_search_time_diff_th", loop_search_time_diff_th_);
  this->get_parameter("loop_search_dist_diff_th", loop_search_dist_diff_th_);
  this->get_parameter("loop_search_angular_dist_th", loop_search_angular_dist_th_);
  this->get_parameter("loop_search_frame_interval", loop_search_frame_interval_);
  this->get_parameter("search_radius", search_radius_);
  this->get_parameter("target_frame_num", target_frame_num_);
  this->get_parameter("target_voxel_leaf_size", target_voxel_leaf_size_);
  this->get_parameter("source_voxel_leaf_size", source_voxel_leaf_size_);
  this->get_parameter("vis_map_voxel_leaf_size", vis_map_voxel_leaf_size_);
  this->get_parameter("fitness_score_th", fitness_score_th_);
  this->get_parameter("vis_map_cloud_frame_interval", vis_map_cloud_frame_interval_);
  this->get_parameter("vis_incremental", vis_incremental_);
  this->get_parameter("vis_full_rebuild_interval_s", vis_full_rebuild_interval_s_);
  this->get_parameter("loop_rate_hz", loop_rate_hz_);
  this->get_parameter("vis_rate_hz", vis_rate_hz_);
  if (loop_rate_hz_ <= 0.0) {
    loop_rate_hz_ = 1.0;
  }
  if (vis_rate_hz_ <= 0.0) {
    vis_rate_hz_ = 1.0;
  }
  this->get_parameter("odom_trans_var", odom_trans_var_);
  this->get_parameter("odom_rot_var", odom_rot_var_);
  this->get_parameter("odom_z_diff_var", odom_z_diff_var_);
  this->get_parameter("loop_var_scale", loop_var_scale_);
  this->get_parameter("loop_var_min", loop_var_min_);
  this->get_parameter("icp_max_corr_dist", icp_max_corr_dist_);
  this->get_parameter("loop_max_trans_delta", loop_max_trans_delta_);
  this->get_parameter("loop_max_trans_delta_ratio", loop_max_trans_delta_ratio_);
  this->get_parameter("loop_max_trans_delta_cap", loop_max_trans_delta_cap_);
  this->get_parameter("loop_constrain_z", loop_constrain_z_);
  this->get_parameter("loop_constrain_attitude", loop_constrain_attitude_);
  this->get_parameter("loop_var_free", loop_var_free_);
  this->get_parameter("loop_factor_mode", loop_factor_mode_);
  this->get_parameter("ground_diag_interval", ground_diag_interval_);
  if (ground_diag_interval_ < 0) {
    ground_diag_interval_ = 0;
  }
  this->get_parameter("enable_floor_normal_factor",
                      enable_floor_normal_factor_);
  this->get_parameter("floor_normal_var_deg", floor_normal_var_deg_);
  this->get_parameter("floor_normal_max_step_deg",
                      floor_normal_max_step_deg_);
  int floor_gap = 50;
  this->get_parameter("floor_normal_max_gap_keyframes", floor_gap);
  floor_normal_max_gap_keyframes_ = static_cast<double>(std::max(1, floor_gap));
  floor_normal_factor_count_ = 0;
  floor_normal_skip_gap_count_ = 0;
  floor_normal_skip_step_count_ = 0;
  if (loop_factor_mode_ != "projected_3dof" && loop_factor_mode_ != "diagonal") {
    RCLCPP_WARN(this->get_logger(),
                "Unknown loop_factor_mode '%s'; falling back to diagonal",
                loop_factor_mode_.c_str());
    loop_factor_mode_ = "diagonal";
  }
  if (loop_factor_mode_ == "projected_3dof" &&
      (loop_constrain_z_ || loop_constrain_attitude_)) {
    RCLCPP_WARN(this->get_logger(),
                "projected_3dof requires loop_constrain_z=false and "
                "loop_constrain_attitude=false; falling back to diagonal");
    loop_factor_mode_ = "diagonal";
  }
  this->get_parameter("overlap_min", overlap_min_);
  this->get_parameter("overlap_dist", overlap_dist_);
  this->get_parameter("loop_max_yaw_delta", loop_max_yaw_delta_);
  this->get_parameter("loop_max_candidate_dz", loop_max_candidate_dz_);
  this->get_parameter("loop_dedup_window", loop_dedup_window_);
  this->get_parameter("use_world_odom_trans", use_world_odom_trans_);
  this->get_parameter("enable_go2_z", enable_go2_z_);
  this->get_parameter("go2_pose_topic", go2_pose_topic_);
  this->get_parameter("go2_pose_timeout", go2_pose_timeout_);
  this->get_parameter("go2_z_var", go2_z_var_);
  this->get_parameter("go2_z_robust_scale", go2_z_robust_scale_);
  if (go2_z_robust_scale_ <= 0.0) {
    go2_z_robust_scale_ = 1.0;
  }
  this->get_parameter("enable_go2_attitude", enable_go2_attitude_);
  this->get_parameter("go2_attitude_var_deg", go2_attitude_var_deg_);
  this->get_parameter("enable_gravity_prior", enable_gravity_prior_);
  this->get_parameter("gravity_source", gravity_source_);
  this->get_parameter("gravity_topic", gravity_topic_);
  this->get_parameter("gravity_var_deg", gravity_var_deg_);
  this->get_parameter("gravity_robust_scale", gravity_robust_scale_);
  this->get_parameter("gravity_timeout", gravity_timeout_);
  this->get_parameter("gravity_lowpass_s", gravity_lowpass_s_);
  this->get_parameter("gravity_norm_tol", gravity_norm_tol_);
  if (gravity_norm_tol_ < 0.0) {
    gravity_norm_tol_ = 0.0;
  }
  for (double& ema : gravity_norm_ema_) {
    ema = 0.0;
  }
  if (gravity_lowpass_s_ < 0.0) {
    gravity_lowpass_s_ = 0.0;
  }
  this->get_parameter("gravity_extrinsic_rpy", gravity_extrinsic_rpy_);
  this->get_parameter("gravity_roll_offset", gravity_roll_offset_);
  this->get_parameter("gravity_pitch_offset", gravity_pitch_offset_);
  this->get_parameter("allow_gravity_z_diff_coexist",
                      allow_gravity_z_diff_coexist_);
  if (gravity_source_ != "lowstate" && gravity_source_ != "utlidar" &&
      gravity_source_ != "livox" && gravity_source_ != "auto") {
    RCLCPP_WARN(this->get_logger(),
                "Unknown gravity_source '%s'; gravity prior disabled",
                gravity_source_.c_str());
    enable_gravity_prior_ = false;
    gravity_source_ = "lowstate";
  }
  if (gravity_source_ == "auto") {
    // Prefer the robot body IMU; fall back to the L2 IMU when the window has
    // no lowstate sample (bag replay).  Both topics are subscribed.
    gravity_topic_ = "";
    gravity_source_id_ = -1;
  } else if (gravity_source_ == "lowstate") {
    gravity_source_id_ = simple_loop_closure::kGravityLowstate;
  } else if (gravity_source_ == "utlidar") {
    gravity_source_id_ = simple_loop_closure::kGravityUtlidar;
  } else {
    gravity_source_id_ = simple_loop_closure::kGravityLivox;
  }
  if (gravity_topic_.empty() && gravity_source_ != "auto") {
    gravity_topic_ = gravity_source_ == "lowstate"
                         ? "/lf/lowstate"
                         : (gravity_source_ == "utlidar" ? "/utlidar/imu"
                                                         : "/livox/imu");
  }
  gravity_extrinsic_.setIdentity();
  if (!gravity_extrinsic_rpy_.empty()) {
    double r = 0.0, p = 0.0, y = 0.0;
    if (std::sscanf(gravity_extrinsic_rpy_.c_str(), "%lf,%lf,%lf", &r, &p,
                    &y) == 3) {
      gravity_extrinsic_ =
          (gtsam::Rot3::Ypr(y * M_PI / 180.0, p * M_PI / 180.0,
                            r * M_PI / 180.0))
              .matrix();
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "gravity_extrinsic_rpy='%s' is not 'roll,pitch,yaw'; using "
                  "identity",
                  gravity_extrinsic_rpy_.c_str());
    }
  }
  go2_attitude_active_ =
      simple_loop_closure::useGo2Attitude(enable_go2_attitude_,
                                          enable_gravity_prior_);
  gravity_factor_count_ = 0;
  gravity_fail_count_ = 0;
  gravity_dynamic_skip_count_ = 0;
  gravity_invalid_count_ = 0;
  gravity_zero_warned_ = false;
  floor_frame_warned_ = false;
  go2_z_factor_count_ = 0;
  go2_roll_offset_ = 0.0;
  go2_pitch_offset_ = 0.0;
  go2_rp_offset_set_ = false;
  go2_rp_factor_count_ = 0;
  go2_zero_warned_ = false;

  // ---- Leg kinematic world-z factor (2026-11-15) ----------------------
  this->get_parameter("enable_leg_z", enable_leg_z_);
  this->get_parameter("leg_lowstate_topic", leg_lowstate_topic_);
  this->get_parameter("leg_sport_topic", leg_sport_topic_);
  this->get_parameter("leg_time_source", leg_time_source_);
  if (leg_time_source_ != "device" && leg_time_source_ != "arrival") {
    RCLCPP_WARN(this->get_logger(),
                "Unknown leg_time_source '%s'; falling back to device",
                leg_time_source_.c_str());
    leg_time_source_ = "device";
  }
  this->get_parameter("leg_z_var", leg_z_var_);
  this->get_parameter("leg_z_robust_scale", leg_z_robust_scale_);
  this->get_parameter("leg_z_timeout", leg_z_timeout_);
  this->get_parameter("leg_attitude_jump_deg", leg_attitude_jump_deg_);
  this->get_parameter("leg_contact_force_on", leg_contact_force_on_);
  this->get_parameter("leg_contact_force_off", leg_contact_force_off_);
  this->get_parameter("leg_single_force", leg_single_force_);
  this->get_parameter("leg_settle_s", leg_settle_s_);
  this->get_parameter("leg_consensus_spread", leg_consensus_spread_);
  this->get_parameter("leg_max_step_dz", leg_max_step_dz_);
  this->get_parameter("leg_pitch_offset_deg", leg_pitch_offset_deg_);
  this->get_parameter("leg_roll_offset_deg", leg_roll_offset_deg_);
  this->get_parameter("leg_offset_x", leg_offset_x_);
  this->get_parameter("leg_offset_y", leg_offset_y_);
  this->get_parameter("leg_thigh_offset", leg_thigh_offset_);
  this->get_parameter("leg_thigh_len", leg_thigh_len_);
  this->get_parameter("leg_calf_len", leg_calf_len_);
  this->get_parameter("enable_prior_map", enable_prior_map_);
  this->get_parameter("prior_map_dir", prior_map_dir_);
  this->get_parameter("prior_search_radius", prior_search_radius_);
  this->get_parameter("prior_max_trans_delta", prior_max_trans_delta_);
  this->get_parameter("prior_icp_max_corr_dist", prior_icp_max_corr_dist_);
  this->get_parameter("prior_overlap_min", prior_overlap_min_);
  this->get_parameter("prior_dedup_window", prior_dedup_window_);
  this->get_parameter("prior_var_scale", prior_var_scale_);
  prior_searched_id_ = 0;
  prev_prior_query_ = -1;
  prev_prior_target_ = -1;
  leg_geometry_ = {leg_offset_x_, leg_offset_y_, leg_thigh_offset_,
                   leg_thigh_len_, leg_calf_len_};
  simple_loop_closure::LegZConfig leg_cfg;
  leg_cfg.force_on = leg_contact_force_on_;
  leg_cfg.force_off = leg_contact_force_off_;
  leg_cfg.single_force = leg_single_force_;
  leg_cfg.settle_s = leg_settle_s_;
  leg_cfg.consensus_spread = leg_consensus_spread_;
  leg_cfg.max_step_dz = leg_max_step_dz_;
  leg_cfg.pitch_offset_deg = leg_pitch_offset_deg_;
  leg_cfg.roll_offset_deg = leg_roll_offset_deg_;
  leg_cfg.attitude_timeout = leg_z_timeout_;
  leg_cfg.attitude_jump_deg = leg_attitude_jump_deg_;
  leg_z_integrator_ =
      std::make_unique<simple_loop_closure::LegZIntegrator>(leg_cfg);
  leg_z_factor_count_ = 0;
  leg_z_zero_warned_ = false;
  leg_z_no_att_warned_ = false;
  leg_z_latest_valid_ = false;
  leg_z_kf_steps_used_ = 0;
  leg_lowstate_count_ = 0;
  leg_sport_missing_count_ = 0;
  leg_time_buf_.clear();

  if (enable_leg_z_ && enable_go2_z_) {
    RCLCPP_WARN(this->get_logger(),
                "enable_leg_z and enable_go2_z are both on: two independent "
                "dz measurements; use this only on flat sites");
  }
  if (enable_leg_z_ && odom_z_diff_var_ > 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "odom_z_diff_var=%.1e skipped: enable_leg_z provides the "
                "world-z constraint", odom_z_diff_var_);
  }
  if (odom_z_diff_var_ > 0.0 &&
      (enable_go2_attitude_ || enable_go2_z_)) {
    RCLCPP_WARN(this->get_logger(),
                "odom_z_diff_var=%.1e ignored: it must not run with "
                "enable_go2_attitude=%d or enable_go2_z=%d (attitude caused "
                ">30 m xy divergence; Go2 z is 2.5D body height and flattens "
                "slopes). Use it only in the cave/slope profile.",
                odom_z_diff_var_, enable_go2_attitude_, enable_go2_z_);
  }
  RCLCPP_INFO(this->get_logger(),
              "Odom translation frame: %s",
              use_world_odom_trans_ ? "world (attitude-decoupled)"
                                    : "body");
  RCLCPP_INFO(this->get_logger(),
              "Odom z continuity: %s (var=%.1e)",
              simple_loop_closure::useOdomZDiffWithLegZ(
                  enable_leg_z_, odom_z_diff_var_, go2_attitude_active_,
                  enable_go2_z_, enable_gravity_prior_,
                  allow_gravity_z_diff_coexist_)
                  ? "enabled" : "disabled",
              odom_z_diff_var_);
  if (enable_gravity_prior_ && odom_z_diff_var_ > 0.0 &&
      !allow_gravity_z_diff_coexist_) {
    RCLCPP_WARN(this->get_logger(),
                "odom_z_diff_var=%.1e skipped: enable_gravity_prior=true and "
                "allow_gravity_z_diff_coexist=false (run the slope A/B with the "
                "coexist flag before combining them)",
                odom_z_diff_var_);
  }
  if (enable_gravity_prior_ && enable_go2_attitude_) {
    RCLCPP_WARN(this->get_logger(),
                "enable_go2_attitude=true ignored: the IMU gravity prior "
                "replaces it (useGo2Attitude)");
  }
  RCLCPP_INFO(this->get_logger(),
              "Gravity prior: %s (source=%s topic=%s var=%.2f deg "
              "robust=%.1f timeout=%.2f lowpass=%.2f s norm_tol=%.2f)",
              enable_gravity_prior_ ? "enabled" : "disabled",
              gravity_source_.c_str(),
              gravity_source_ == "auto" ? "lowstate>livox(fallback)"
                                        : gravity_topic_.c_str(),
              gravity_var_deg_, gravity_robust_scale_, gravity_timeout_,
              gravity_lowpass_s_, gravity_norm_tol_);
  RCLCPP_INFO(this->get_logger(),
              "Floor normal factor: %s (var=%.2f deg step<=%.1f deg gap<=%.0f)",
              enable_floor_normal_factor_ ? "enabled" : "disabled",
              floor_normal_var_deg_, floor_normal_max_step_deg_,
              floor_normal_max_gap_keyframes_);
  RCLCPP_INFO(this->get_logger(), "Loop factor mode: %s",
              loop_factor_mode_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "Loop constraint: z=%s attitude=%s (free_var=%.0e)",
              loop_constrain_z_ ? "constrained" : "free",
              loop_constrain_attitude_ ? "constrained" : "free",
              loop_var_free_);
  RCLCPP_INFO(this->get_logger(),
              "Go2 z factor: %s (topic=%s var=%.4g robust_scale=%.1f)",
              enable_go2_z_ ? "enabled" : "disabled",
              go2_pose_topic_.c_str(), go2_z_var_, go2_z_robust_scale_);
  RCLCPP_INFO(this->get_logger(),
              "Leg z factor: %s (topic=%s time=%s var=%.4g robust=%.1f "
              "timeout=%.2f pitch=%.2f roll=%.2f)",
              enable_leg_z_ ? "enabled" : "disabled",
              leg_lowstate_topic_.c_str(), leg_time_source_.c_str(),
              leg_z_var_, leg_z_robust_scale_, leg_z_timeout_,
              leg_pitch_offset_deg_, leg_roll_offset_deg_);
  RCLCPP_INFO(this->get_logger(), "Thread rates: loop=%.1f Hz vis=%.1f Hz",
              loop_rate_hz_, vis_rate_hz_);
  RCLCPP_INFO(this->get_logger(), "Go2 attitude prior: %s (%.2f deg)",
              go2_attitude_active_ ? "enabled" : "disabled",
              go2_attitude_var_deg_);
  RCLCPP_INFO(this->get_logger(),
              "Loop gates: overlap>=%.2f (%.2f m) |yaw|<=%.2f rad |dz|<=%.2f m dedup=%d "
              "|d_xy|<=max(%.2f, %.3f*path)%s",
              overlap_min_, overlap_dist_, loop_max_yaw_delta_,
              loop_max_candidate_dz_, loop_dedup_window_,
              loop_max_trans_delta_, loop_max_trans_delta_ratio_,
              loop_max_trans_delta_cap_ > 0.0 ? ", capped" : "");
  if (loop_max_trans_delta_cap_ > 0.0) {
    RCLCPP_INFO(this->get_logger(), "Loop gate cap: |d_xy| <= %.2f m",
                loop_max_trans_delta_cap_);
  }
  vis_cache_added_id_ = 0;
  vis_last_full_rebuild_ns_ = 0;
  vis_cache_rebuild_requested_ = false;

  search_radius_ *= search_radius_;
  stop_loop_closure_thread_ = false;
  stop_visualize_thread_ = false;

  sub_cloud_.subscribe(this, "/cloud");
  sub_odom_.subscribe(this, "/odometry");

  synchronizer_ = std::make_shared<Sync>(SyncPolicy(50), sub_cloud_, sub_odom_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(time_stamp_tolerance_));
  synchronizer_->registerCallback(std::bind(&SimpleLoopClosureNode::pointCloudAndOdometryCallback, this,
                                            std::placeholders::_1, std::placeholders::_2));

  sub_save_req_ = this->create_subscription<std_msgs::msg::String>(
      "/save_req", 10, std::bind(&SimpleLoopClosureNode::saveRequestCallback, this, std::placeholders::_1));

  if (enable_leg_z_) {
    // The lowstate callback must not queue behind the point-cloud callback
    // on the single-threaded executor: that produced ~130 dt skips and a
    // +0.29 m z bias on a 356 s replay.  A dedicated mutually-exclusive
    // group runs concurrently with the default group, while the go2 pose
    // callback (same group) stays serialized with it.
    leg_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  if (enable_go2_z_ || go2_attitude_active_ || enable_leg_z_) {
    rclcpp::SubscriptionOptions go2_opts;
    if (enable_leg_z_) {
      go2_opts.callback_group = leg_cb_group_;
    }
    sub_go2_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        go2_pose_topic_, 10,
        std::bind(&SimpleLoopClosureNode::go2PoseCallback, this,
                  std::placeholders::_1),
        go2_opts);
  }

  if (enable_leg_z_) {
    // Dedicated lowstate subscription for the joint angles.  With
    // gravity_source=livox the gravity prior does not subscribe this topic,
    // so there is no duplicate callback.
    rclcpp::SubscriptionOptions leg_opts;
    leg_opts.callback_group = leg_cb_group_;
    sub_leg_lowstate_ =
        this->create_subscription<unitree_go::msg::LowState>(
            leg_lowstate_topic_, 50,
            std::bind(&SimpleLoopClosureNode::legLowStateCallback, this,
                      std::placeholders::_1),
            leg_opts);
    if (leg_time_source_ == "device") {
      sub_leg_sport_ =
          this->create_subscription<unitree_go::msg::SportModeState>(
              leg_sport_topic_, 50,
              std::bind(&SimpleLoopClosureNode::sportModeCallback, this,
                        std::placeholders::_1),
              leg_opts);
    }
  }

  if (enable_gravity_prior_) {
    const bool want_lowstate =
        gravity_source_ == "lowstate" || gravity_source_ == "auto";
    const bool want_imu = gravity_source_ != "lowstate";
    if (want_lowstate) {
      sub_gravity_lowstate_ =
          this->create_subscription<unitree_go::msg::LowState>(
              gravity_source_ == "auto" ? "/lf/lowstate" : gravity_topic_, 50,
              std::bind(&SimpleLoopClosureNode::gravityLowStateCallback, this,
                        std::placeholders::_1));
    }
    if (want_imu) {
      sub_gravity_imu_ =
          this->create_subscription<sensor_msgs::msg::Imu>(
              gravity_source_ == "auto"
                  ? "/livox/imu"
                  : (gravity_source_ == "utlidar" ? gravity_topic_
                                                  : gravity_topic_),
              200,
              std::bind(&SimpleLoopClosureNode::gravityImuCallback, this,
                        std::placeholders::_1));
    }
  }

  icp_.setMaximumIterations(50);
  icp_.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  icp_.setTransformationEpsilon(0.0001);
  icp_.setEuclideanFitnessEpsilon(0.0001);
  icp_.setRANSACIterations(0);

  // Prior ICP is deliberately tighter than the loop ICP: a prior match should
  // only ever move the pose by tens of centimetres, and a 5 m correspondence
  // distance would let it slide to a parallel branch on the other side of a
  // thin wall.
  prior_icp_.setMaximumIterations(50);
  prior_icp_.setMaxCorrespondenceDistance(prior_icp_max_corr_dist_);
  prior_icp_.setTransformationEpsilon(0.0001);
  prior_icp_.setEuclideanFitnessEpsilon(0.0001);
  prior_icp_.setRANSACIterations(0);

  vg_target_.setLeafSize(target_voxel_leaf_size_, target_voxel_leaf_size_, target_voxel_leaf_size_);
  vg_source_.setLeafSize(source_voxel_leaf_size_, source_voxel_leaf_size_, source_voxel_leaf_size_);
  vg_map_.setLeafSize(vis_map_voxel_leaf_size_, vis_map_voxel_leaf_size_, vis_map_voxel_leaf_size_);

  added_odom_id_ = 0;
  searched_loop_id_ = 0;
  keyframes_cloud_.clear();
  keyframes_odom_.clear();
  keyframes_leg_z_.clear();
  keyframes_leg_z_copied_.clear();

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(parameters);

  Eigen::VectorXd prior_noise_vector(6);
  prior_noise_vector << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_noise_vector);
  // GTSAM Pose3 的切空间顺序是 [Rx,Ry,Rz,Tx,Ty,Tz] —— 旋转在前
  // (见 gtsam/geometry/Pose3.h: "canonical coordinates [R_x,R_y,R_z,T_x,T_y,T_z]")。
  // 旧代码按 [T,T,T,R,R,R] 填, 因为两个方差当时都等于 1e-4 才没暴露问题;
  // 做各向异性调参前必须先按正确顺序写。
  Eigen::VectorXd odom_noise_vector(6);
  odom_noise_vector << odom_rot_var_, odom_rot_var_, odom_rot_var_,
                       odom_trans_var_, odom_trans_var_, odom_trans_var_;
  odom_noise_ = gtsam::noiseModel::Diagonal::Variances(odom_noise_vector);

  odom_edge_color_.r = 0.0;
  odom_edge_color_.g = 0.75;
  odom_edge_color_.b = 1.0;
  odom_edge_color_.a = 1.0;

  loop_edge_color_.r = 1.0;
  loop_edge_color_.g = 0.75;
  loop_edge_color_.b = 0.0;
  loop_edge_color_.a = 1.0;

  node_color_.r = 0.5;
  node_color_.g = 1.0;
  node_color_.b = 0.0;
  node_color_.a = 1.0;

  edge_scale_ = 0.1;
  node_scale_ = 0.15;

  saving_ = false;

  // Load the prior map before the loop thread can touch it.  A bad asset
  // disables the feature instead of aborting the node.
  loadPriorMap();

  // Start Threads
  loop_close_thread_ = std::thread(&SimpleLoopClosureNode::loopCloseThread, this);
  visualize_thread_ = std::thread(&SimpleLoopClosureNode::visualizeThread, this);

  RCLCPP_INFO(this->get_logger(), "Loop Closure Node Initialized.");
}

void makeDirectory(const std::string& directory)
{
  if (!std::filesystem::is_directory(directory) || !std::filesystem::exists(directory))
  {                                                // Check if directory exists
    std::filesystem::create_directory(directory);  // create directory
  }
}

bool SimpleLoopClosureNode::saveEachFrames()
{
  std::string frames_directory = save_directory_ + "frames/";
  makeDirectory(frames_directory);

  int optimization_result_size = 0;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
  }

  if (optimization_result_size <= 0)
    return false;

  int digits = std::to_string(optimization_result_size).length();
  if (digits < 6)
    digits = 6;

  std::string poses_csv_filename = save_directory_ + "poses.csv";
  std::ofstream poses_csv_file(poses_csv_filename);
  if (!poses_csv_file)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to open poses CSV file: %s", poses_csv_filename.c_str());
    return false;
  }
  // Frame convention marker: the prior-map loader refuses assets that do not
  // say which frame the per-keyframe clouds are in (mapped_cloud decides).
  poses_csv_file << "# frame: " << (mapped_cloud_ ? "body" : "world") << std::endl;
  poses_csv_file << "index, timestamp, x, y, z, qx, qy, qz, qw" << std::endl;
  for (int i = 0; i < optimization_result_size; i++)
  {
    Eigen::Affine3d pose;
    {
      MtxLockGuard guard(mtx_res_);
      pose = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(i).matrix());
    }

    PointCloudType copied_cloud;
    {
      MtxLockGuard guard(mtx_buf_);
      copied_cloud = *keyframes_cloud_[i];
    }

    std::stringstream frame_filename_str;
    frame_filename_str << std::setfill('0') << std::right << std::setw(digits) << i << ".pcd";
    Eigen::Quaterniond quat(pose.rotation());
    Eigen::Vector3d trans = pose.translation();
    pcl::io::savePCDFileBinary(frames_directory + frame_filename_str.str(), copied_cloud);
    poses_csv_file << std::fixed << i << ", " << pcl_conversions::fromPCL(copied_cloud.header.stamp).seconds() << ", "
                   << trans.x() << ", " << trans.y() << ", " << trans.z() << ", " << quat.x() << ", " << quat.y()
                   << ", " << quat.z() << ", " << quat.w() << std::endl;
  }

  poses_csv_file.close();

  return true;
}

void SimpleLoopClosureNode::saveThread()
{
  PointCloudType::Ptr map_cloud = constructPointCloudMap();

  if (map_cloud == nullptr || map_cloud->empty())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Point cloud map is empty.");
  }
  else
  {
    try
    {
      if (!saveEachFrames())
      {
        throw std::runtime_error("Save failed");
      }
      std::cout<<"SAVE DIRECTORY: "<<save_directory_<<std::endl;
      pcl::io::savePCDFileBinary(save_directory_ + "map.pcd", *map_cloud);
      RCLCPP_INFO_STREAM(this->get_logger(), "Save completed.");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Save failed: %s", e.what());
    }
  }
  saving_ = false;
}

void SimpleLoopClosureNode::saveRequestCallback(const std_msgs::msg::String::ConstSharedPtr& directory)
{
  if (saving_ == true)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Already in process. Request is denied.");
    return;
  }

  save_directory_ = directory->data;
  if (!save_directory_.empty() && save_directory_.back() != '/')
    save_directory_ += '/';

  RCLCPP_INFO_STREAM(this->get_logger(), "Start Saving.");
  if (save_thread_.joinable())
    save_thread_.join();

  saving_ = true;
  save_thread_ = std::thread(&SimpleLoopClosureNode::saveThread, this);
}

void SimpleLoopClosureNode::publishPoseGraphOptimizedOdometry(const Eigen::Affine3d& affine_curr,
                                                              const nav_msgs::msg::Odometry& odom_msg)
{
  Eigen::Affine3d pgo_affine;

  Eigen::Affine3d optimized_pose_last;
  int optimized_pose_id_last;
  getLastOptimizedPose(optimized_pose_last, optimized_pose_id_last);

  if (optimized_pose_id_last != 0)
  {
    MtxLockGuard guard(mtx_buf_);
    pgo_affine = optimized_pose_last * (keyframes_odom_[optimized_pose_id_last]->inverse() * affine_curr);
  }
  else
  {
    pgo_affine = affine_curr;
  }

  nav_msgs::msg::Odometry pgo_odom_msg = odom_msg;
  pgo_odom_msg.pose.pose = tf2::toMsg(pgo_affine);
  pub_pgo_odometry_->publish(pgo_odom_msg);
}

void SimpleLoopClosureNode::pointCloudAndOdometryCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
  PointCloudType::Ptr cloud_curr(new PointCloudType);
  pcl::fromROSMsg(*cloud_msg, *cloud_curr);

  Affine3dPtr affine_curr(new Eigen::Affine3d);
  tf2::fromMsg(odom_msg->pose.pose, *affine_curr);

  publishPoseGraphOptimizedOdometry(*affine_curr, *odom_msg);

  if (!keyframes_odom_.empty())
  {
    MtxLockGuard guard(mtx_buf_);
    double distance = (keyframes_odom_.back()->translation() - affine_curr->translation()).norm();
    Eigen::Quaterniond quat_prev(keyframes_odom_.back()->rotation());
    Eigen::Quaterniond quat_curr(affine_curr->rotation());
    double angle = quat_prev.angularDistance(quat_curr);

    if (distance < keyframe_dist_th_ && angle < keyframe_angular_dist_th_)
      return;
  }
  else
  {
    // first call
    odom_frame_id_ = odom_msg->header.frame_id;
  }

  if (mapped_cloud_)
  {
    PointCloudType::Ptr cloud_base(new PointCloudType);
    Eigen::Affine3d affine_inv = affine_curr->inverse();
    pcl::transformPointCloud(*cloud_curr, *cloud_base, affine_inv);
    cloud_curr = cloud_base;
  }

  // Floor fit for the continuity factor / diagnostics.  Computed once per
  // keyframe; the factor path passes world_gate=false (a tilted pose is
  // exactly what it must be able to correct).
  simple_loop_closure::GroundPlane floor;
  bool need_floor = enable_floor_normal_factor_ || ground_diag_interval_ > 0;
  if (need_floor && !mapped_cloud_) {
    // The fit assumes the keyframe cloud is in the body frame (mapped_cloud
    // transforms it).  With world-frame clouds the height/step/rotation
    // assumptions are all wrong, so disable rather than produce a wrong edge.
    if (!floor_frame_warned_) {
      RCLCPP_WARN(this->get_logger(),
                  "Floor fit needs mapped_cloud=true (body-frame clouds); "
                  "disabled");
      floor_frame_warned_ = true;
    }
    need_floor = false;
  }
  if (need_floor) {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(cloud_curr->size() / 2 + 1);
    for (size_t k = 0; k < cloud_curr->size(); k += 2) {
      const auto& p = cloud_curr->points[k];
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        pts.emplace_back(p.x, p.y, p.z);
      }
    }
    floor = simple_loop_closure::fitGroundPlane(
        pts, affine_curr->rotation(), 0.30, 80, false);
  }

  size_t diag_id = 0;
  bool run_ground_diag = false;
  {
    MtxLockGuard guard(mtx_buf_);
    if (trajectory_dist_.empty())
      trajectory_dist_.push_back(0);
    else
      trajectory_dist_.push_back(trajectory_dist_.back() +
                                 (keyframes_odom_.back()->translation() - affine_curr->translation()).norm());

    keyframes_cloud_.push_back(cloud_curr);
    keyframes_odom_.push_back(affine_curr);
    // Arrival time, for matching the gravity buffer (same clock as its
    // samples).  The LIO header clock differs from wall time in replay.
    const double kf_stamp = this->now().seconds();
    keyframes_stamp_.push_back(kf_stamp);

    // Leg z snapshot: NaN when tracking is invalid or no step was accepted
    // since the previous keyframe.  A frozen z must not become a zero-dz
    // factor, which would pin the graph's z.
    double leg_z = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> lock(mtx_leg_z_);
      const long steps_now = leg_z_integrator_->diagnostics().steps_used;
      if (simple_loop_closure::legZKeyframeUsable(
              leg_z_latest_valid_, steps_now, leg_z_kf_steps_used_)) {
        leg_z = simple_loop_closure::legZAtKeyframe(
            leg_z_latest_stamp_, leg_z_latest_valid_, leg_z_latest_, kf_stamp,
            leg_z_timeout_);
      }
      leg_z_kf_steps_used_ = steps_now;
    }
    keyframes_leg_z_.push_back(leg_z);

    // Match by arrival time, not header stamps: in replay the Go2 pose
    // header clock can be offset by hundreds of seconds from the LIO
    // odometry header (same reason analyze_drift.py uses bag receive time).
    Go2Sample go2;
    bool have_go2 = false;
    if (enable_go2_z_ || go2_attitude_active_) {
      std::lock_guard<std::mutex> lock(mtx_go2_);
      have_go2 = simple_loop_closure::nearestGo2Sample(
          go2_buf_, kf_stamp, go2_pose_timeout_, go2);
    }
    if (have_go2) {
      keyframes_go2_z_.push_back(go2.z);
      keyframes_go2_q_.push_back(go2.q);
    } else {
      keyframes_go2_z_.push_back(
          std::numeric_limits<double>::quiet_NaN());
      keyframes_go2_q_.push_back(
          Eigen::Quaterniond(std::nan(""), 0.0, 0.0, 0.0));
    }

    keyframes_floor_normal_.push_back(
        floor.valid ? floor.normal_body
                    : Eigen::Vector3d(std::nan(""), std::nan(""), std::nan("")));
    diag_id = keyframes_cloud_.size() - 1;
    run_ground_diag =
        ground_diag_interval_ > 0 &&
        (diag_id % static_cast<size_t>(ground_diag_interval_) == 0);
  }
  if (run_ground_diag) {
    logGroundDiagnostics(floor, *affine_curr, diag_id);
  }
}

void SimpleLoopClosureNode::logGroundDiagnostics(
    const simple_loop_closure::GroundPlane& g, const Eigen::Affine3d& lio_pose,
    size_t id) {
  if (!g.valid) {
    RCLCPP_INFO(this->get_logger(),
                "Ground diag: id=%zu invalid (no floor in band)", id);
    return;
  }
  // Diagnostic-only world gate: a keyframe whose current pose is tilted more
  // than 25 deg is not a trustworthy attitude sample.  The factor path does
  // not apply this gate (see fitGroundPlane's world_gate argument).
  if (g.normal_world.z() < 0.90) {
    RCLCPP_INFO(this->get_logger(),
                "Ground diag: id=%zu invalid (world normal %.2f deg off)", id,
                std::acos(std::max(-1.0, std::min(1.0, g.normal_world.z()))) *
                180.0 / M_PI);
    return;
  }

  double pgo_roll = std::numeric_limits<double>::quiet_NaN();
  double pgo_pitch = pgo_roll;
  double pgo_pitch_err = pgo_roll;
  double pgo_roll_err = pgo_roll;
  {
    MtxLockGuard guard(mtx_res_);
    if (!optimization_result_.empty()) {
      const gtsam::Key kid = optimization_result_.exists(
                                 static_cast<gtsam::Key>(id))
                                 ? static_cast<gtsam::Key>(id)
                                 : static_cast<gtsam::Key>(
                                       optimization_result_.size() - 1);
      const gtsam::Rot3 R = optimization_result_.at<gtsam::Pose3>(kid).rotation();
      const auto rpy = R.rpy();
      pgo_roll = rpy(0) * 180.0 / M_PI;
      pgo_pitch = rpy(1) * 180.0 / M_PI;
      const Eigen::Vector3d nw = (R.matrix() * g.normal_body).normalized();
      if (nw.z() > 0.1) {
        pgo_pitch_err = std::atan2(-nw.x(), nw.z()) * 180.0 / M_PI;
        pgo_roll_err = std::atan2(nw.y(), nw.z()) * 180.0 / M_PI;
      }
    }
  }
  const auto lio_rpy = gtsam::Rot3(lio_pose.rotation()).rpy();

  RCLCPP_INFO(this->get_logger(),
              "Ground diag: id=%zu in=%zu h=%.3f pitch_err=%+.2f roll_err=%+.2f "
              "lio_roll=%+.2f lio_pitch=%+.2f pgo_roll=%+.2f pgo_pitch=%+.2f "
              "pgo_pitch_err=%+.2f pgo_roll_err=%+.2f",
              id, g.inliers, g.height, g.pitch_err_deg, g.roll_err_deg,
              lio_rpy(0) * 180.0 / M_PI, lio_rpy(1) * 180.0 / M_PI, pgo_roll,
              pgo_pitch, pgo_pitch_err, pgo_roll_err);
}

void SimpleLoopClosureNode::go2PoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  Go2Sample s;
  s.stamp = this->now().seconds();
  s.z = std::isfinite(msg->pose.position.z)
            ? msg->pose.position.z
            : std::numeric_limits<double>::quiet_NaN();
  // Header time on the robot clock.  The leg-z attitude pairing uses it so
  // it does not depend on Jetson callback jitter; the Go2 z factor keeps
  // using the arrival time above.
  s.device_stamp =
      static_cast<double>(msg->header.stamp.sec) +
      msg->header.stamp.nanosec * 1e-9;
  if (!std::isfinite(s.device_stamp) || s.device_stamp <= 0.0) {
    s.device_stamp = std::numeric_limits<double>::quiet_NaN();
  }

  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  if (!simple_loop_closure::validQuaternion(q)) {
    // Sentinel: the attitude factor checks q.w() for finiteness.
    s.q = Eigen::Quaterniond(std::nan(""), 0.0, 0.0, 0.0);
  } else {
    s.q = q.normalized();
  }

  {
    std::lock_guard<std::mutex> lock(mtx_go2_);
    go2_buf_.push_back(s);
    while (!go2_buf_.empty() && s.stamp - go2_buf_.front().stamp > 5.0) {
      go2_buf_.pop_front();
    }
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Go2 pose buffer: %zu samples", go2_buf_.size());
}

void SimpleLoopClosureNode::legLowStateCallback(
    const unitree_go::msg::LowState::ConstSharedPtr& msg) {
  if (!leg_z_integrator_) {
    return;
  }
  const double arrival = this->now().seconds();
  simple_loop_closure::LegSample s;
  s.stamp = arrival;
  ++leg_lowstate_count_;

  // LowState has no header.  In device mode, give it the timestamp of the
  // 20 Hz sportmodestate message from the same control cycle; this removes
  // Jetson callback jitter from the integrator's dt and attitude pairing.
  if (leg_time_source_ == "device") {
    simple_loop_closure::LegTimeSample pair;
    if (!simple_loop_closure::nearestLegTimeSample(
            leg_time_buf_, arrival, leg_z_timeout_, pair)) {
      ++leg_sport_missing_count_;
      return;  // never mix the Jetson and device clocks
    }
    // LowState arrives ~15 ms before the sportmodestate of the same cycle;
    // shift the sport device stamp by the arrival delta.
    s.stamp = simple_loop_closure::legLowStateDeviceStamp(pair, arrival);
  }

  for (int i = 0; i < 4; ++i) {
    s.foot[i] = simple_loop_closure::legFootPosition(
        leg_geometry_, i, msg->motor_state[3 * i].q,
        msg->motor_state[3 * i + 1].q, msg->motor_state[3 * i + 2].q);
    s.force[i] = static_cast<double>(msg->foot_force[i]);
  }

  simple_loop_closure::LegAttitude att;
  {
    simple_loop_closure::Go2Sample go2;
    std::lock_guard<std::mutex> lock(mtx_go2_);
    const bool found =
        leg_time_source_ == "device"
            ? simple_loop_closure::nearestGo2SampleByDevice(
                  go2_buf_, s.stamp, leg_z_timeout_, go2)
            : simple_loop_closure::nearestGo2Sample(
                  go2_buf_, s.stamp, leg_z_timeout_, go2);
    if (found && simple_loop_closure::validQuaternion(go2.q)) {
      att.stamp = leg_time_source_ == "device" ? go2.device_stamp : go2.stamp;
      att.R = go2.q.toRotationMatrix();
      att.valid = true;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_leg_z_);
    leg_z_integrator_->update(s, att);
    // Keyframes are stamped with the Jetson arrival clock; keep the latest
    // arrival separate from the device stamp used inside the integrator.
    leg_z_latest_stamp_ = arrival;
    leg_z_latest_ = leg_z_integrator_->z();
    // Without a valid attitude the integrator skips every step and z() is
    // frozen.  Marking this true unconditionally would emit a chain of
    // zero-dz factors and silently pin the graph's z.
    leg_z_latest_valid_ = att.valid;
  }
}

void SimpleLoopClosureNode::sportModeCallback(
    const unitree_go::msg::SportModeState::ConstSharedPtr& msg) {
  simple_loop_closure::LegTimeSample s;
  s.arrival = this->now().seconds();
  s.device_stamp =
      static_cast<double>(msg->stamp.sec) + msg->stamp.nanosec * 1e-9;
  if (!std::isfinite(s.device_stamp) || s.device_stamp <= 0.0) {
    return;
  }
  leg_time_buf_.push_back(s);
  while (!leg_time_buf_.empty() &&
         s.arrival - leg_time_buf_.front().arrival > 5.0) {
    leg_time_buf_.pop_front();
  }
}

void SimpleLoopClosureNode::pushGravitySample(
    const simple_loop_closure::GravitySample& sample) {
  std::lock_guard<std::mutex> lock(mtx_gravity_);
  gravity_buf_.push_back(sample);
  while (!gravity_buf_.empty() &&
         sample.stamp - gravity_buf_.front().stamp > 5.0) {
    gravity_buf_.pop_front();
  }
}

void SimpleLoopClosureNode::gravityLowStateCallback(
    const unitree_go::msg::LowState::ConstSharedPtr& msg) {
  simple_loop_closure::GravitySample s;
  s.stamp = this->now().seconds();
  s.source = simple_loop_closure::kGravityLowstate;
  const auto& a = msg->imu_state.accelerometer;
  s.specific_force = Eigen::Vector3d(a[0], a[1], a[2]);
  // unitree_go IMUState.quaternion is [w, x, y, z] (verified on the standing
  // bag: q[0] ~ 1 when level).
  const auto& q = msg->imu_state.quaternion;
  const Eigen::Quaterniond qe(q[0], q[1], q[2], q[3]);
  if (simple_loop_closure::validQuaternion(qe)) {
    s.q = qe.normalized();
    s.has_q = true;
  }
  pushGravitySample(s);
}

void SimpleLoopClosureNode::gravityImuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  simple_loop_closure::GravitySample s;
  s.stamp = this->now().seconds();
  const Eigen::Vector3d a(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
  if (gravity_source_ == "utlidar") {
    // calibrate_imu.cpp transform: raw utlidar axes -> Go2 base (z up).
    s.specific_force = simple_loop_closure::utlidarSpecificForce(a);
    s.source = simple_loop_closure::kGravityUtlidar;
  } else {
    // livox: the L2 IMU is the Point-LIO IMU, already in the body frame
    // (z up, units of g; only the direction is used).
    s.specific_force = a;
    s.source = simple_loop_closure::kGravityLivox;
  }
  pushGravitySample(s);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructPointCloudMap(const int interval)
{
  int optimization_result_size = 0;
  int first_cloud_size = 0;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
    if (!optimization_result_.empty())
      first_cloud_size = keyframes_cloud_.front()->size();
  }

  if (optimization_result_size <= 0)
    return nullptr;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new PointCloudType);
  map_cloud->reserve((size_t)((double)(first_cloud_size * optimization_result_size) * 1.5));
  for (int i = 0; i < optimization_result_size; i += (interval + 1))
  {
    Eigen::Affine3d pose;
    {
      MtxLockGuard guard(mtx_res_);
      pose = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(i).matrix());
    }

    PointCloudType transformed_cloud;
    {
      MtxLockGuard guard(mtx_buf_);
      pcl::transformPointCloud(*keyframes_cloud_[i], transformed_cloud, pose);
    }

    *map_cloud += transformed_cloud;
  }

  return map_cloud;
}

void SimpleLoopClosureNode::publishMapCloud(const PointCloudType::Ptr& map_cloud)
{
  if (map_cloud == NULL)
    return;

  PointCloudType map_cloud_ds_;

  vg_map_.setInputCloud(map_cloud);
  vg_map_.filter(map_cloud_ds_);

  // 与增量路径保持一致: 只发 XYZ (PointXYZI 32 B/点 -> PointXYZ 16 B/点).
  VisCloudType map_cloud_xyz;
  map_cloud_xyz.reserve(map_cloud_ds_.size());
  for (const auto& point : map_cloud_ds_.points)
  {
    VisPointType vis_point;
    vis_point.x = point.x;
    vis_point.y = point.y;
    vis_point.z = point.z;
    map_cloud_xyz.push_back(vis_point);
  }

  sensor_msgs::msg::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(map_cloud_xyz, map_cloud_msg);
  map_cloud_msg.header.stamp = this->now();
  map_cloud_msg.header.frame_id = odom_frame_id_;

  pub_map_cloud_->publish(map_cloud_msg);
}

// ---------------------------------------------------------------------------
// Incremental visualization map cache
//
// The old path rebuilt the map from scratch every second (transform every
// keyframe, concatenate, filter the whole map at 0.05 m).  This keeps the
// transformed, voxel-deduplicated map and only folds in keyframes it has not
// seen yet.  A loop closure moves old poses, so it requests a full rebuild;
// a periodic rebuild also absorbs the small refinements iSAM2 applies to
// recent poses without a closure.
// ---------------------------------------------------------------------------

uint64_t SimpleLoopClosureNode::voxelKey(double x, double y, double z) const
{
  // 21 bits per axis at vis_map_voxel_leaf_size: +-52 km, far beyond any
  // indoor map.  Same leaf size the old whole-map VoxelGrid used.
  const double leaf = vis_map_voxel_leaf_size_ > 0.0 ? vis_map_voxel_leaf_size_ : 0.05;
  auto quantize = [leaf](double value) -> int64_t { return static_cast<int64_t>(std::floor(value / leaf)); };
  auto pack = [](int64_t cell) -> uint64_t {
    return static_cast<uint64_t>((cell + (1 << 20)) & ((1 << 21) - 1));
  };
  return pack(quantize(x)) | (pack(quantize(y)) << 21) | (pack(quantize(z)) << 42);
}

void SimpleLoopClosureNode::clearVisualizationMapCache()
{
  vis_voxel_keys_.clear();
  vis_map_cache_.clear();
  vis_cache_added_id_ = 0;
}

void SimpleLoopClosureNode::appendNewKeyframesToCache()
{
  std::vector<PointCloudType::Ptr> new_clouds;
  {
    MtxLockGuard guard(mtx_buf_);
    for (size_t i = vis_cache_added_id_; i < keyframes_cloud_.size(); ++i)
      new_clouds.push_back(keyframes_cloud_[i]);
  }
  if (new_clouds.empty())
    return;

  std::vector<Eigen::Affine3d> poses;
  {
    MtxLockGuard guard(mtx_res_);
    const size_t available = optimization_result_.size() > vis_cache_added_id_
                                 ? optimization_result_.size() - vis_cache_added_id_
                                 : 0;
    const size_t count = std::min(new_clouds.size(), available);
    if (count == 0)
      return;
    poses.reserve(count);
    for (size_t i = 0; i < count; ++i)
      poses.push_back(Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(vis_cache_added_id_ + i).matrix()));
    new_clouds.resize(count);
  }

  PointCloudType transformed;
  for (size_t i = 0; i < new_clouds.size(); ++i)
  {
    pcl::transformPointCloud(*new_clouds[i], transformed, poses[i]);
    for (const auto& point : transformed.points)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        continue;
      if (vis_voxel_keys_.insert(voxelKey(point.x, point.y, point.z)).second)
      {
        // Publish XYZ only (see VisPointType); intensity is unused downstream.
        VisPointType vis_point;
        vis_point.x = point.x;
        vis_point.y = point.y;
        vis_point.z = point.z;
        vis_map_cache_.push_back(vis_point);
      }
    }
  }
  vis_cache_added_id_ += new_clouds.size();
}

void SimpleLoopClosureNode::updateVisualizationMapCache()
{
  const int64_t now_ns = this->now().nanoseconds();
  bool periodic = false;
  if (vis_full_rebuild_interval_s_ > 0.0 && vis_last_full_rebuild_ns_ > 0)
    periodic = (now_ns - vis_last_full_rebuild_ns_) >
               static_cast<int64_t>(vis_full_rebuild_interval_s_ * 1e9);
  const bool requested = vis_cache_rebuild_requested_.exchange(false);
  if (requested || periodic || vis_last_full_rebuild_ns_ == 0)
  {
    if (requested)
      RCLCPP_INFO(this->get_logger(), "vis map: full rebuild requested (loop closure)");
    clearVisualizationMapCache();
    vis_last_full_rebuild_ns_ = now_ns;
  }
  appendNewKeyframesToCache();
}

void SimpleLoopClosureNode::publishCachedMapCloud()
{
  // Before the first optimized pose exists the cache is empty; publishing an
  // empty cloud only makes every downstream consumer log a warning.  The old
  // whole-map path returned early in the same situation.
  if (vis_map_cache_.empty())
    return;

  sensor_msgs::msg::PointCloud2 map_cloud_msg;
  pcl::toROSMsg(vis_map_cache_, map_cloud_msg);
  map_cloud_msg.header.stamp = this->now();
  map_cloud_msg.header.frame_id = odom_frame_id_;
  pub_map_cloud_->publish(map_cloud_msg);
}

void SimpleLoopClosureNode::constructVisualizationOdometryEdges(const int& res_size,
                                                                const std_msgs::msg::Header& header,
                                                                visualization_msgs::msg::Marker& marker_msg)
{
  marker_msg.header = header;
  marker_msg.ns = "odom_edges";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = edge_scale_;
  marker_msg.color = odom_edge_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(res_size * 2);
  for (int i = 0; i < res_size - 1; i++)
  {
    MtxLockGuard guard(mtx_res_);
    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(i);
    gtsam::Pose3 pose2 = optimization_result_.at<gtsam::Pose3>(i + 1);

    geometry_msgs::msg::Point p1, p2;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    p2.x = pose2.x();
    p2.y = pose2.y();
    p2.z = pose2.z();
    marker_msg.points.emplace_back(p1);
    marker_msg.points.emplace_back(p2);
  }
}

void SimpleLoopClosureNode::constructVisualizationLoopEdges(const int& res_size, const std_msgs::msg::Header& header,
                                                            visualization_msgs::msg::Marker& marker_msg)
{
  int edge_num;
  {
    MtxLockGuard guard(mtx_res_);
    edge_num = loop_edges_.size();
  }

  marker_msg.header = header;
  marker_msg.ns = "loop_edges";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = edge_scale_;
  marker_msg.color = loop_edge_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(edge_num * 2);
  for (int i = 0; i < edge_num; i++)
  {
    MtxLockGuard guard(mtx_res_);

    int id1 = loop_edges_[i].first;
    int id2 = loop_edges_[i].second;

    if (id1 >= res_size || id2 >= res_size)
      continue;

    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(id1);
    gtsam::Pose3 pose2 = optimization_result_.at<gtsam::Pose3>(id2);

    geometry_msgs::msg::Point p1, p2;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    p2.x = pose2.x();
    p2.y = pose2.y();
    p2.z = pose2.z();
    marker_msg.points.emplace_back(p1);
    marker_msg.points.emplace_back(p2);
  }
}

void SimpleLoopClosureNode::constructVisualizationNodes(const int& res_size, const std_msgs::msg::Header& header,
                                                        visualization_msgs::msg::Marker& marker_msg)
{
  marker_msg.header = header;
  marker_msg.ns = "nodes";
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_msg.scale.x = node_scale_;
  marker_msg.scale.y = node_scale_;
  marker_msg.scale.z = node_scale_;
  marker_msg.color = node_color_;

  marker_msg.points.clear();
  marker_msg.points.reserve(res_size);
  for (int i = 0; i < res_size; i++)
  {
    MtxLockGuard guard(mtx_res_);
    gtsam::Pose3 pose1 = optimization_result_.at<gtsam::Pose3>(i);

    geometry_msgs::msg::Point p1;
    p1.x = pose1.x();
    p1.y = pose1.y();
    p1.z = pose1.z();
    marker_msg.points.emplace_back(p1);
  }
}

void SimpleLoopClosureNode::publishVisualizationGraph()
{
  int optimization_result_size;
  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_size = optimization_result_.size();
  }

  if (optimization_result_size <= 0)
    return;

  std_msgs::msg::Header header;
  rclcpp::Time t = this->now();
  header.stamp = t;
  header.frame_id = odom_frame_id_;

  visualization_msgs::msg::MarkerArray marker_array_msg;
  marker_array_msg.markers.resize(3);
  constructVisualizationOdometryEdges(optimization_result_size, header, marker_array_msg.markers[0]);
  constructVisualizationLoopEdges(optimization_result_size, header, marker_array_msg.markers[1]);
  constructVisualizationNodes(optimization_result_size, header, marker_array_msg.markers[2]);

  pub_vis_pose_graph_->publish(marker_array_msg);
}

void SimpleLoopClosureNode::copyKeyFrames()
{
  MtxLockGuard guard(mtx_buf_);
  for (size_t i = keyframes_cloud_copied_.size(); i < keyframes_cloud_.size(); i++)
  {
    keyframes_cloud_copied_.push_back(keyframes_cloud_[i]);
    keyframes_odom_copied_.push_back(keyframes_odom_[i]);
    trajectory_dist_copied_.push_back(trajectory_dist_[i]);
    keyframes_go2_z_copied_.push_back(keyframes_go2_z_[i]);
    keyframes_go2_q_copied_.push_back(keyframes_go2_q_[i]);
    keyframes_stamp_copied_.push_back(keyframes_stamp_[i]);
    keyframes_leg_z_copied_.push_back(keyframes_leg_z_[i]);
    keyframes_floor_normal_copied_.push_back(keyframes_floor_normal_[i]);
  }
}

void SimpleLoopClosureNode::getLastOptimizedPose(Eigen::Affine3d& pose, int& id)
{
  MtxLockGuard guard(mtx_res_);
  if (!optimization_result_.empty())
  {
    pose = optimization_result_.at<gtsam::Pose3>(optimization_result_.size() - 1).matrix();
    id = optimization_result_.size() - 1;
  }
  else
  {
    MtxLockGuard guard(mtx_buf_);
    if (!keyframes_odom_.empty())
    {
      pose = *keyframes_odom_[0];
      id = 0;
    }
    else
    {
      pose = Eigen::Affine3d();
      id = 0;
    }
  }
}

bool SimpleLoopClosureNode::constructOdometryGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& init_estimate)
{
  if (added_odom_id_ >= keyframes_odom_copied_.size())
    return false;

  if (added_odom_id_ == 0)
  {
    gtsam::Pose3 pose = gtsam::Pose3(keyframes_odom_copied_[0]->matrix());
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, prior_noise_));
    init_estimate.insert(0, pose);
    added_odom_id_ = 1;
  }

  Eigen::Affine3d optimized_pose_last;
  int optimized_pose_id_last;
  getLastOptimizedPose(optimized_pose_last, optimized_pose_id_last);
  for (size_t i = added_odom_id_; i < keyframes_odom_copied_.size(); i++)
  {
    Eigen::Affine3d pose_diff = keyframes_odom_copied_[i - 1]->inverse() * (*keyframes_odom_copied_[i]);
    // 2026-11-11 P0: 按关键帧实际运动量缩放里程计噪声. 关键帧不是等间距的
    // (快走/慢走、转弯都会改变间隔), 固定方差会让走得多的一步和走得少的
    // 一步被同等信任. motionScale 只放大不缩小, 名义间距以下保持原方差.
    const double odom_dist = pose_diff.translation().norm();
    const double odom_ang =
        gtsam::Rot3::Logmap(gtsam::Rot3(pose_diff.rotation())).norm();
    const double dist_scale = simple_loop_closure::motionScale(
        odom_dist, keyframe_dist_th_);
    const double ang_scale = simple_loop_closure::motionScale(
        odom_ang, keyframe_angular_dist_th_);
    Eigen::VectorXd odom_var(6);
    odom_var << odom_rot_var_ * ang_scale * ang_scale,
        odom_rot_var_ * ang_scale * ang_scale,
        odom_rot_var_ * ang_scale * ang_scale,
        odom_trans_var_ * dist_scale * dist_scale,
        odom_trans_var_ * dist_scale * dist_scale,
        odom_trans_var_ * dist_scale * dist_scale;
    const gtsam::SharedNoiseModel odom_noise =
        gtsam::noiseModel::Diagonal::Variances(odom_var);
    if (use_world_odom_trans_) {
      // 2026-11-15: body-frame odometry translation couples attitude
      // corrections into world xy.  The gravity prior changes roll/pitch by
      // up to 7.5 deg on the corridor, and R_i * dp_body then rotates the
      // whole remaining path (measured 5.9 m xy bend / 137 s).  Split the
      // edge: rotation stays body-frame, translation is constrained in the
      // world frame, so attitude corrections cannot move positions.
      Eigen::VectorXd rot_var = odom_var;
      rot_var(3) = rot_var(4) = rot_var(5) = 1e4;  // translation free here
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          i - 1, i,
          gtsam::Pose3(gtsam::Rot3(pose_diff.rotation()),
                       gtsam::Point3(0.0, 0.0, 0.0)),
          gtsam::noiseModel::Diagonal::Variances(rot_var)));

      const double trans_var =
          odom_trans_var_ * dist_scale * dist_scale;
      // z stays owned by leg z when both keyframes have a finite sample;
      // otherwise fall back to the odometry variance so z is never free.
      double z_var = trans_var;
      if (enable_leg_z_ && i < keyframes_leg_z_copied_.size() &&
          std::isfinite(keyframes_leg_z_copied_[i]) &&
          std::isfinite(keyframes_leg_z_copied_[i - 1])) {
        z_var = 1e4;
      }
      const Eigen::Vector3d dp_world =
          keyframes_odom_copied_[i]->translation() -
          keyframes_odom_copied_[i - 1]->translation();
      gtsam::Vector3 trans_var_vec(trans_var, trans_var, z_var);
      graph.add(boost::make_shared<simple_loop_closure::WorldTransDiffFactor>(
          static_cast<gtsam::Key>(i - 1), static_cast<gtsam::Key>(i),
          gtsam::Vector3(dp_world.x(), dp_world.y(), dp_world.z()),
          gtsam::noiseModel::Diagonal::Variances(trans_var_vec)));
    } else {
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          i - 1, i, gtsam::Pose3(pose_diff.matrix()), odom_noise));
    }
    if (simple_loop_closure::useOdomZDiffWithLegZ(
            enable_leg_z_, odom_z_diff_var_, go2_attitude_active_,
            enable_go2_z_, enable_gravity_prior_,
            allow_gravity_z_diff_coexist_)) {
      // World-z continuity, decoupled from the body-frame odometry edge.
      // The measurement is the LIO world Δz, so at the LIO solution the
      // residual is zero; loop corrections that rotate the pose cannot use
      // the translation null space to move z.
      const double dz_world = keyframes_odom_copied_[i]->translation().z() -
                              keyframes_odom_copied_[i - 1]->translation().z();
      graph.add(boost::make_shared<simple_loop_closure::ZDiffFactor>(
          static_cast<gtsam::Key>(i - 1), static_cast<gtsam::Key>(i), dz_world,
          gtsam::noiseModel::Diagonal::Variances(
              (gtsam::Vector(1) << odom_z_diff_var_).finished())));
    }

    Eigen::Affine3d pose_init = optimized_pose_last * (keyframes_odom_copied_[optimized_pose_id_last]->inverse() *
                                                       (*keyframes_odom_copied_[i]));
    init_estimate.insert(i, gtsam::Pose3(pose_init.matrix()));

    if (enable_go2_z_ && i > 0 &&
        std::isfinite(keyframes_go2_z_copied_[i]) &&
        std::isfinite(keyframes_go2_z_copied_[i - 1])) {
      // Relative z only: the Go2 z origin is arbitrary, and an absolute
      // prior was measured to distort xy (loop closure 5.5 m) by fighting
      // the trajectory's z datum.  The difference cancels the offset.
      const double dz = keyframes_go2_z_copied_[i] -
                        keyframes_go2_z_copied_[i - 1];
      graph.add(boost::make_shared<simple_loop_closure::ZDiffFactor>(
          static_cast<gtsam::Key>(i - 1), static_cast<gtsam::Key>(i), dz,
          simple_loop_closure::robustNoise1(
              go2_z_var_, go2_z_robust_scale_)));
      ++go2_z_factor_count_;
    }

    if (enable_leg_z_ && i > 0 &&
        std::isfinite(keyframes_leg_z_copied_[i]) &&
        std::isfinite(keyframes_leg_z_copied_[i - 1])) {
      // Independent world Δz from the foot anchors.  The cumulative value
      // is sampled by arrival time at keyframe creation; a stale/missing
      // sample is stored as NaN and adds no factor.
      const double dz = keyframes_leg_z_copied_[i] -
                        keyframes_leg_z_copied_[i - 1];
      graph.add(boost::make_shared<simple_loop_closure::ZDiffFactor>(
          static_cast<gtsam::Key>(i - 1), static_cast<gtsam::Key>(i), dz,
          simple_loop_closure::robustNoise1(leg_z_var_,
                                            leg_z_robust_scale_)));
      ++leg_z_factor_count_;
    }

    // IMU gravity soft prior.  The residual is roll/pitch only: the target
    // keeps the LIO yaw/translation and takes roll/pitch from the measured
    // specific-force direction, with yaw left free in the noise model.
    if (enable_gravity_prior_ && i < keyframes_stamp_copied_.size()) {
      simple_loop_closure::GravitySample gs;
      bool have_gravity = false;
      int used_source = gravity_source_id_ >= 0 ? gravity_source_id_ : 0;
      {
        std::lock_guard<std::mutex> lock(mtx_gravity_);
        const double half = 0.5 * gravity_lowpass_s_;
        const double stamp = keyframes_stamp_copied_[i];
        if (gravity_source_ == "auto") {
          // Prefer the robot body IMU; fall back to the L2 IMU for keyframes
          // whose window has no lowstate sample (bag replay).
          have_gravity = simple_loop_closure::averageGravitySample(
              gravity_buf_, stamp, half, gs,
              simple_loop_closure::kGravityLowstate);
          used_source = simple_loop_closure::kGravityLowstate;
          if (!have_gravity) {
            have_gravity = simple_loop_closure::averageGravitySample(
                gravity_buf_, stamp, half, gs,
                simple_loop_closure::kGravityLivox);
            used_source = simple_loop_closure::kGravityLivox;
          }
        } else {
          have_gravity = simple_loop_closure::averageGravitySample(
              gravity_buf_, stamp, half, gs, gravity_source_id_);
        }
        if (!have_gravity) {
          // Fall back to the nearest sample only if the window is empty (a
          // dropped stream), keeping the old timeout semantics.
          if (gravity_source_ == "auto") {
            have_gravity = simple_loop_closure::nearestGravitySample(
                gravity_buf_, stamp, gravity_timeout_, gs,
                simple_loop_closure::kGravityLowstate);
            used_source = simple_loop_closure::kGravityLowstate;
            if (!have_gravity) {
              have_gravity = simple_loop_closure::nearestGravitySample(
                  gravity_buf_, stamp, gravity_timeout_, gs,
                  simple_loop_closure::kGravityLivox);
              used_source = simple_loop_closure::kGravityLivox;
            }
          } else {
            have_gravity = simple_loop_closure::nearestGravitySample(
                gravity_buf_, stamp, gravity_timeout_, gs, gravity_source_id_);
          }
        }
      }
      const double g_norm = gs.specific_force.norm();
      double& norm_ema =
          gravity_norm_ema_[static_cast<size_t>(used_source) % 3];
      const bool dynamic_ok =
          !have_gravity ||
          simple_loop_closure::gravityNormAccepted(g_norm, norm_ema,
                                                   gravity_norm_tol_);
      if (have_gravity && !dynamic_ok) {
        ++gravity_dynamic_skip_count_;
      }
      if (!have_gravity) {
        ++gravity_fail_count_;
      } else if (dynamic_ok) {
        if (norm_ema <= 0.0) {
          norm_ema = g_norm;
        } else {
          norm_ema = 0.95 * norm_ema + 0.05 * g_norm;
        }
        const Eigen::Vector3d g_body =
            (gravity_extrinsic_ * gs.specific_force).normalized();
        const gtsam::Pose3 lio_pose(keyframes_odom_copied_[i]->matrix());
        if (g_body.z() > 0.3) {
          const gtsam::Pose3 target =
              simple_loop_closure::targetPoseFromGravity(
                  lio_pose, g_body, gravity_roll_offset_,
                  gravity_pitch_offset_);
          const double rp_var =
              std::pow(gravity_var_deg_ * M_PI / 180.0, 2);
          graph.add(gtsam::PriorFactor<gtsam::Pose3>(
              static_cast<gtsam::Key>(i), target,
              simple_loop_closure::makeRollPitchPriorNoise(
                  rp_var, loop_var_free_, gravity_robust_scale_)));
          ++gravity_factor_count_;
        } else {
          ++gravity_invalid_count_;
        }
      }
      if (i % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "Gravity prior @%zu: added=%d no_sample=%d dynamic_skip=%d "
                    "invalid=%d",
                    i, gravity_factor_count_, gravity_fail_count_,
                    gravity_dynamic_skip_count_, gravity_invalid_count_);
        if (!gravity_zero_warned_ && gravity_factor_count_ == 0 &&
            added_odom_id_ > 50) {
          RCLCPP_WARN(this->get_logger(),
                      "Gravity prior has added 0 factors after %zu keyframes: "
                      "check gravity_source=%s, gravity_topic=%s and the accel "
                      "sign convention",
                      added_odom_id_, gravity_source_.c_str(),
                      gravity_topic_.c_str());
          gravity_zero_warned_ = true;
        }
      }
    }

    // Floor-normal continuity: connect this valid floor observation to the
    // previous valid one, bridging any run of invalid frames (the door).
    if (enable_floor_normal_factor_ &&
        i < keyframes_floor_normal_copied_.size() &&
        keyframes_floor_normal_copied_[i].allFinite()) {
      int j = -1;
      for (int k = static_cast<int>(i) - 1; k >= 0; --k) {
        if (keyframes_floor_normal_copied_[static_cast<size_t>(k)]
                .allFinite()) {
          j = k;
          break;
        }
      }
      if (j >= 0) {
        const int gap = static_cast<int>(i) - j;
        if (gap > static_cast<int>(floor_normal_max_gap_keyframes_)) {
          ++floor_normal_skip_gap_count_;
        } else {
          const Eigen::Vector3d& n_j =
              keyframes_floor_normal_copied_[static_cast<size_t>(j)];
          const Eigen::Matrix3d R_i = keyframes_odom_copied_[i]->rotation();
          const Eigen::Matrix3d R_j =
              keyframes_odom_copied_[static_cast<size_t>(j)]->rotation();
          const Eigen::Vector3d nw_i =
              (R_i * keyframes_floor_normal_copied_[i]).normalized();
          const Eigen::Vector3d nw_j = (R_j * n_j).normalized();
          const double step_deg =
              std::acos(std::max(-1.0, std::min(1.0, nw_i.dot(nw_j)))) *
              180.0 / M_PI;
          if (step_deg > floor_normal_max_step_deg_) {
            ++floor_normal_skip_step_count_;
          } else {
            const double var =
                std::pow(floor_normal_var_deg_ * M_PI / 180.0, 2) *
                (1.0 + static_cast<double>(gap) / 20.0);
            graph.add(boost::make_shared<simple_loop_closure::PoseNormalFactor>(
                static_cast<gtsam::Key>(j), static_cast<gtsam::Key>(i), n_j,
                keyframes_floor_normal_copied_[i],
                simple_loop_closure::robustNoise3(var, 1.0)));
            ++floor_normal_factor_count_;
          }
        }
      }
      if (i % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "Floor normal factor @%zu: added=%d gap_skip=%d "
                    "step_skip=%d",
                    i, floor_normal_factor_count_,
                    floor_normal_skip_gap_count_, floor_normal_skip_step_count_);
      }
    }

    if (go2_attitude_active_ &&
        std::isfinite(keyframes_go2_q_copied_[i].w())) {
      const gtsam::Pose3 lio_pose(keyframes_odom_copied_[i]->matrix());
      const Eigen::Quaterniond& go2_q = keyframes_go2_q_copied_[i];
      if (!go2_rp_offset_set_) {
        const gtsam::Vector3 lio_rpy = lio_pose.rotation().rpy();
        const gtsam::Vector3 go2_rpy =
            gtsam::Rot3(go2_q.toRotationMatrix()).rpy();
        // Anchor the Go2 attitude to the LIO initial attitude.  The two
        // are gravity-aligned, so this offset should be small; it only
        // absorbs a constant mounting/calibration bias.
        go2_roll_offset_ = lio_rpy(0) - go2_rpy(0);
        go2_pitch_offset_ = lio_rpy(1) - go2_rpy(1);
        go2_rp_offset_set_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Go2 attitude offset at keyframe %zu: roll=%+.2f pitch=%+.2f deg",
                    i, go2_roll_offset_ * 180.0 / M_PI,
                    go2_pitch_offset_ * 180.0 / M_PI);
      }
      const gtsam::Pose3 target =
          simple_loop_closure::targetPoseFromGo2Attitude(
              lio_pose, go2_q, go2_roll_offset_, go2_pitch_offset_);
      const double rp_var =
          std::pow(go2_attitude_var_deg_ * M_PI / 180.0, 2);
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          static_cast<gtsam::Key>(i), target,
          simple_loop_closure::makeRollPitchPriorNoise(rp_var,
                                                       loop_var_free_)));
      ++go2_rp_factor_count_;
    }

    added_odom_id_++;
  }

  return true;
}

bool SimpleLoopClosureNode::constructKDTreeMatrix(KDTreeMatrix& kdtree_mat)
{
  MtxLockGuard guard(mtx_res_);
  if (optimization_result_.empty())
  {
    return false;
  }

  kdtree_mat.resize(optimization_result_.size(), 3);
  for (size_t i = 0; i < optimization_result_.size(); i++)
  {
    kdtree_mat.row(i) = optimization_result_.at<gtsam::Pose3>(i).translation();
  }

  return true;
}

int SimpleLoopClosureNode::searchTarget(const KDTree& kdtree, const int& id_query, const Eigen::Affine3d& pose_query,
                                        const rclcpp::Time& stamp_query)
{
  NanoFlannSearchResult search_result;
  search_result.reserve(1000);
  nanoflann::SearchParameters search_params(0.0, true);
  kdtree.index_->radiusSearch(pose_query.translation().data(), search_radius_, search_result, search_params);

  Eigen::Quaterniond quat_query(pose_query.rotation());

  int target_id = -1;
  for (size_t i = 0; i < search_result.size(); i++)
  {
    int tmp_id = search_result[i].first;
    rclcpp::Time tmp_stamp = pcl_conversions::fromPCL(keyframes_cloud_copied_[tmp_id]->header.stamp);
    double time_diff = std::fabs(stamp_query.seconds() - tmp_stamp.seconds());

    Eigen::Affine3d affine_target;
    {
      MtxLockGuard guard(mtx_res_);
      affine_target = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(tmp_id).matrix());
    }

    Eigen::Quaterniond quat_target(affine_target.rotation());
    double angle_diff = quat_query.angularDistance(quat_target);
    double dist_diff = std::fabs(trajectory_dist_copied_[id_query] - trajectory_dist_copied_[tmp_id]);
    if (time_diff > loop_search_time_diff_th_ && angle_diff < loop_search_angular_dist_th_ &&
        dist_diff > loop_search_dist_diff_th_)
    {
      target_id = tmp_id;
      break;
    }
  }

  return target_id;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructTargetCloud(const int target_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new PointCloudType);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ds(new PointCloudType);

  // 用有符号索引:target_id < target_frame_num_ 时 target_id - target_frame_num_ 为负数,
  // 若直接赋给 size_t 会下溢成巨大值,循环体一次都不执行,target_cloud 为空,
  // 下游 PCL 报 "Invalid or empty point cloud dataset" / "No input target dataset"。
  // 这正好发生在"回环到地图最开始的 0..9 号关键帧"时,即起点附近的回环永远闭不上。
  const int first_frame = (target_id - target_frame_num_ < 0) ? 0 : target_id - target_frame_num_;
  const int last_frame = target_id + target_frame_num_;
  for (int i = first_frame; i <= last_frame; ++i)
  {
    Eigen::Affine3d tmp_affine;
    {
      MtxLockGuard guard(mtx_res_);
      if (i < 0 || i >= static_cast<int>(optimization_result_.size()))
        continue;

      tmp_affine = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(i).matrix());
    }

    PointCloudType tmp_cloud;
    pcl::transformPointCloud(*keyframes_cloud_copied_[i], tmp_cloud, tmp_affine);

    *target_cloud += tmp_cloud;
  }

  if (target_voxel_leaf_size_ <= 0.0)
  {
    target_cloud_ds = target_cloud;
  }
  else
  {
    vg_target_.setInputCloud(target_cloud);
    vg_target_.filter(*target_cloud_ds);
  }

  return target_cloud_ds;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructSourceCloud(const int source_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ds(new PointCloudType);

  if (source_voxel_leaf_size_ <= 0.0)
  {
    *source_cloud_ds = *keyframes_cloud_copied_[source_id];
  }
  else
  {
    vg_source_.setInputCloud(keyframes_cloud_copied_[source_id]);
    vg_source_.filter(*source_cloud_ds);
  }

  return source_cloud_ds;
}

bool SimpleLoopClosureNode::tryRegistration(const Eigen::Affine3d& init_pose, const PointCloudType::Ptr& source_cloud,
                                            const PointCloudType::Ptr& target_cloud, Eigen::Affine3d& result,
                                            double& score, double& overlap, double& icp_ms)
{
  return tryRegistrationWith(icp_, init_pose, source_cloud, target_cloud,
                             result, score, overlap, icp_ms);
}

bool SimpleLoopClosureNode::tryRegistrationWith(
    pcl::IterativeClosestPoint<PointType, PointType>& icp,
    const Eigen::Affine3d& init_pose, const PointCloudType::Ptr& source_cloud,
    const PointCloudType::Ptr& target_cloud, Eigen::Affine3d& result,
    double& score, double& overlap, double& icp_ms)
{
  const auto t_start = std::chrono::steady_clock::now();
  PointCloudType::Ptr unused_cloud(new PointCloudType);
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.align(*unused_cloud, init_pose.matrix().cast<float>());

  Eigen::Affine3f result_f(icp.getFinalTransformation());
  result = result_f.cast<double>();
  score = icp.getFitnessScore();

  // 2026-11-11 P0: overlap 是"源点云经 ICP 后有多少落在目标子图里", 与
  // 场景尺度/形状无关. 自相似墙面的错误配对常常 fitness 很低但 overlap
  // 也低 (ICP 把源点云滑到另一面墙上, 只对上一小部分).
  std::vector<Eigen::Vector3d> src_pts;
  std::vector<Eigen::Vector3d> dst_pts;
  src_pts.reserve(source_cloud->size());
  dst_pts.reserve(target_cloud->size());
  for (const auto& p : source_cloud->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      src_pts.emplace_back(p.x, p.y, p.z);
  }
  for (const auto& p : target_cloud->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      dst_pts.emplace_back(p.x, p.y, p.z);
  }
  overlap = simple_loop_closure::overlapRatio(
      src_pts, dst_pts, result, overlap_dist_);

  icp_ms = std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t_start).count();

  if (icp.hasConverged() && score < fitness_score_th_)
    return true;

  return false;
}

bool SimpleLoopClosureNode::loadPriorMap()
{
  prior_poses_.clear();
  prior_clouds_.clear();

  if (!enable_prior_map_)
  {
    RCLCPP_INFO(this->get_logger(), "prior map: disabled");
    return false;
  }

  const std::string csv_path = prior_map_dir_ + "/poses.csv";
  std::ifstream csv(csv_path);
  if (!csv)
  {
    RCLCPP_WARN(this->get_logger(), "prior map: cannot open %s; disabled",
                csv_path.c_str());
    enable_prior_map_ = false;
    return false;
  }

  std::string first_line;
  std::getline(csv, first_line);
  if (!simple_loop_closure::priorHeaderIsBodyFrame(first_line))
  {
    RCLCPP_WARN(this->get_logger(),
                "prior map: %s must start with '# frame: body' (asset saved "
                "with mapped_cloud=false?); disabled",
                csv_path.c_str());
    enable_prior_map_ = false;
    return false;
  }

  if (!simple_loop_closure::parsePriorPoses(csv, prior_poses_) ||
      prior_poses_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "prior map: no parsable poses in %s; disabled",
                csv_path.c_str());
    enable_prior_map_ = false;
    return false;
  }

  prior_clouds_.resize(prior_poses_.size());
  std::vector<size_t> kept;
  kept.reserve(prior_poses_.size());
  size_t missing = 0;
  for (size_t i = 0; i < prior_poses_.size(); ++i)
  {
    std::ostringstream name;
    name << prior_map_dir_ << "/frames/" << std::setfill('0')
         << std::setw(6) << prior_poses_[i].index << ".pcd";
    PointCloudType::Ptr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(name.str(), *cloud) != 0 ||
        cloud->empty())
    {
      ++missing;
      continue;
    }
    // Downsample once at load: the raw keyframe clouds are ~10^4 points each
    // and concatenating them per query pushed the loop thread to 360 ms/cycle.
    if (target_voxel_leaf_size_ > 0.0)
    {
      PointCloudType::Ptr ds(new PointCloudType);
      vg_target_.setInputCloud(cloud);
      vg_target_.filter(*ds);
      cloud = ds;
    }
    prior_clouds_[i] = cloud;
    kept.push_back(i);
  }

  if (kept.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "prior map: no readable frames under %s/frames; disabled",
                prior_map_dir_.c_str());
    enable_prior_map_ = false;
    return false;
  }

  prior_index_.build(prior_poses_, kept);
  RCLCPP_INFO(this->get_logger(),
              "Prior map loaded: %zu keyframes from %s (%zu missing)",
              kept.size(), prior_map_dir_.c_str(), missing);
  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SimpleLoopClosureNode::constructPriorTargetCloud(
    const int prior_id)
{
  PointCloudType::Ptr target_cloud(new PointCloudType);
  if (prior_id < 0 || prior_id >= static_cast<int>(prior_poses_.size()))
    return target_cloud;

  // Nearest few frames only: with 0.5 m spacing a 3 m radius pulls in ~35
  // frames, and the ICP cost grows with the target size for no gain.
  const Eigen::Vector3d centre = prior_poses_[prior_id].t;
  std::vector<std::pair<double, size_t>> near;
  near.reserve(prior_poses_.size());
  for (size_t i = 0; i < prior_poses_.size(); ++i)
  {
    if (!prior_clouds_[i]) continue;
    const Eigen::Vector3d d = prior_poses_[i].t - centre;
    const double dist = std::hypot(d.x(), d.y());
    if (dist > 3.0) continue;
    near.emplace_back(dist, i);
  }
  std::sort(near.begin(), near.end());
  const size_t kMaxFrames = 5;
  if (near.size() > kMaxFrames) near.resize(kMaxFrames);

  for (const auto& entry : near)
  {
    const size_t i = entry.second;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = prior_poses_[i].q.toRotationMatrix();
    pose.translation() = prior_poses_[i].t;
    PointCloudType tmp;
    pcl::transformPointCloud(*prior_clouds_[i], tmp, pose);
    *target_cloud += tmp;
  }

  if (target_cloud->empty() || target_voxel_leaf_size_ <= 0.0)
    return target_cloud;

  PointCloudType::Ptr target_cloud_ds(new PointCloudType);
  vg_target_.setInputCloud(target_cloud);
  vg_target_.filter(*target_cloud_ds);
  return target_cloud_ds;
}

bool SimpleLoopClosureNode::constructPriorEdges(gtsam::NonlinearFactorGraph& graph)
{
  if (!enable_prior_map_ || prior_index_.size() == 0)
    return false;
  if (prior_searched_id_ >= keyframes_cloud_copied_.size())
    return false;

  Eigen::Affine3d optimized_pose_last;
  int optimized_pose_id_last;
  getLastOptimizedPose(optimized_pose_last, optimized_pose_id_last);

  const int stride = loop_search_frame_interval_ + 1;
  for (size_t i = prior_searched_id_; i < keyframes_cloud_copied_.size();
       i += stride)
  {
    const Eigen::Affine3d pose_query =
        optimized_pose_last *
        (keyframes_odom_copied_[optimized_pose_id_last]->inverse() *
         (*keyframes_odom_copied_[i]));

    const int prior_id = prior_index_.nearest(
        pose_query.translation().x(), pose_query.translation().y(),
        prior_search_radius_);
    if (prior_id < 0)
      continue;

    PointCloudType::Ptr target_cloud = constructPriorTargetCloud(prior_id);
    if (!target_cloud || target_cloud->empty())
    {
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (empty_target): q=%zu p=%d", i, prior_id);
      continue;
    }
    PointCloudType::Ptr source_cloud = constructSourceCloud(static_cast<int>(i));

    Eigen::Affine3d registration_result;
    double fitness_score = 0.0;
    double overlap = 0.0;
    double icp_ms = 0.0;
    if (!tryRegistrationWith(prior_icp_, pose_query, source_cloud, target_cloud,
                             registration_result, fitness_score, overlap,
                             icp_ms))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (fit): q=%zu p=%d fit=%.4f (thr %.3f) "
                  "overlap=%.2f icp=%.1fms",
                  i, prior_id, fitness_score, fitness_score_th_, overlap,
                  icp_ms);
      continue;
    }

    if (overlap < prior_overlap_min_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (overlap): q=%zu p=%d overlap=%.2f (thr %.2f)",
                  i, prior_id, overlap, prior_overlap_min_);
      continue;
    }

    // The correction magnitude is what decides whether this is real drift or
    // an ICP slide; it is an absolute gate because a prior edge has no
    // meaningful "path length" to scale against.
    const double correction =
        (registration_result.translation() - pose_query.translation()).norm();
    if (correction > prior_max_trans_delta_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (inconsistent): q=%zu p=%d |d|=%.2f m "
                  "(thr %.2f)",
                  i, prior_id, correction, prior_max_trans_delta_);
      continue;
    }

    if (!simple_loop_closure::priorSequenceAgrees(
            prev_prior_query_, prev_prior_target_, static_cast<int>(i),
            prior_id, 2 * stride + 2, 3))
    {
      // Hold this candidate so the next query keyframe can confirm it.
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (sequence): q=%zu p=%d (pending)", i,
                  prior_id);
      prev_prior_query_ = static_cast<int>(i);
      prev_prior_target_ = prior_id;
      continue;
    }

    if (simple_loop_closure::isDuplicateLoop(
            static_cast<int>(i), prior_id, last_prior_query_for_target_,
            prior_dedup_window_))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Prior Rejected (duplicate): q=%zu p=%d", i, prior_id);
      continue;
    }
    last_prior_query_for_target_[prior_id] = static_cast<int>(i);

    const Eigen::VectorXd noise_vector = simple_loop_closure::makeLoopVariances(
        fitness_score, prior_var_scale_, loop_var_min_, loop_constrain_z_,
        loop_constrain_attitude_, loop_var_free_);
    Eigen::VectorXd v(3);
    v << noise_vector(2), noise_vector(3), noise_vector(4);  // yaw, x, y
    const gtsam::SharedNoiseModel projected_noise =
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(v));

    // ICP returns the corrected pose in the world frame, so the measurement is
    // absolute (not relative to the prediction).
    const Eigen::Vector2d measured_xy(registration_result.translation().x(),
                                      registration_result.translation().y());
    const double measured_yaw = simple_loop_closure::yawDelta(
        registration_result.rotation(), Eigen::Matrix3d::Identity());
    if (!measured_xy.allFinite() || !std::isfinite(measured_yaw))
    {
      RCLCPP_WARN(this->get_logger(),
                  "Prior Rejected (non-finite): q=%zu p=%d", i, prior_id);
      continue;
    }

    graph.add(boost::make_shared<simple_loop_closure::WorldXYPhiPriorFactor>(
        i, measured_xy, measured_yaw, projected_noise));

    RCLCPP_INFO(this->get_logger(),
                "Prior Accepted: q=%zu p=%d |d_xy|=%.2f overlap=%.2f yaw=%+.2f "
                "fit=%.4f icp=%.1fms",
                i, prior_id, correction, overlap, measured_yaw, fitness_score,
                icp_ms);

    prev_prior_query_ = static_cast<int>(i);
    prev_prior_target_ = prior_id;
    vis_cache_rebuild_requested_ = true;
  }

  prior_searched_id_ = keyframes_cloud_copied_.size();
  return true;
}

bool SimpleLoopClosureNode::constructLoopEdge(gtsam::NonlinearFactorGraph& graph)
{
  if (searched_loop_id_ >= keyframes_cloud_copied_.size())
    return false;

  KDTreeMatrix kdtree_mat;
  if (!constructKDTreeMatrix(kdtree_mat))
    return false;

  KDTree kdtree(3, std::cref(kdtree_mat), 10);
  kdtree.index_->buildIndex();

  Eigen::Affine3d optimized_pose_last;
  int optimized_pose_id_last;
  getLastOptimizedPose(optimized_pose_last, optimized_pose_id_last);

  for (size_t i = searched_loop_id_; i < keyframes_cloud_copied_.size(); i += (loop_search_frame_interval_ + 1))
  {
    Eigen::Affine3d pose_query = optimized_pose_last * (keyframes_odom_copied_[optimized_pose_id_last]->inverse() *
                                                        (*keyframes_odom_copied_[i]));
    rclcpp::Time stamp_query = pcl_conversions::fromPCL(keyframes_cloud_copied_[i]->header.stamp);

    int target_id = searchTarget(kdtree, i, pose_query, stamp_query);

    if (target_id == -1)
      continue;

    PointCloudType::Ptr target_cloud = constructTargetCloud(target_id);
    PointCloudType::Ptr source_cloud = constructSourceCloud(i);
    Eigen::Affine3d registration_result;
    double fitness_score;
    double overlap;
    double icp_ms;
    if (!tryRegistration(pose_query, source_cloud, target_cloud, registration_result, fitness_score, overlap, icp_ms))
    {
      // DIAG 2026-11-07: tell "no candidate" apart from "candidate rejected
      // by the fitness gate" -- the two have very different fixes.
      RCLCPP_INFO(this->get_logger(),
                  "Loop Rejected (fit): %zu -> %d  fit=%.4f (thr %.3f) overlap=%.2f icp=%.1fms",
                  i, target_id, fitness_score, fitness_score_th_, overlap, icp_ms);
      continue;
    }

    // 2026-11-11 P0: overlap gate.  ICP 收敛且 MSE 很低, 并不代表源点云
    // 真的和目标子图是同一处: 长走廊/洞穴里 ICP 可以沿墙滑到另一个位置,
    // 只对上一小段. 真实回访的源点云大部分都应该落在目标子图里.
    if (overlap < overlap_min_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Loop Rejected (overlap): %zu -> %d  overlap=%.2f (thr %.2f) fit=%.4f icp=%.1fms",
                  i, target_id, overlap, overlap_min_, fitness_score, icp_ms);
      continue;
    }

    // 2026-11-11 P0: yaw gate. 比较 ICP 的 yaw 修正和里程计预测,
    // 不是比较两个关键帧的朝向 (回访时接近方向可以不同).
    const double icp_yaw = simple_loop_closure::yawDelta(
        registration_result.rotation(), pose_query.rotation());
    if (std::abs(icp_yaw) > loop_max_yaw_delta_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Loop Rejected (yaw): %zu -> %d  icp_yaw=%+.2f rad (thr %.2f) overlap=%.2f fit=%.4f icp=%.1fms",
                  i, target_id, icp_yaw, loop_max_yaw_delta_, overlap,
                  fitness_score, icp_ms);
      continue;
    }

    Eigen::Affine3d target_pose;
    {
      MtxLockGuard guard(mtx_res_);
      target_pose = Eigen::Affine3d(optimization_result_.at<gtsam::Pose3>(target_id).matrix());
    }

    // 2026-11-11 P0: 候选高度差 gate. 只有可信的世界 z 参考才启用.
    // LIO/PGO 的 z 在 3-DOF 模式下是自由 DOF, 正是要检测的漂移本身; 拿它
    // 当门限会把快跑走廊里真实回访 (PGO z 已漂 1.8 m) 当成跨层拒掉.
    // Go2 的 robot_pose.z 在坡面是机身/局部地面高度 (2.5D), 只在
    // enable_go2_z (平地/局部水平) 时才可当高度参考.
    const size_t qi = i;
    const size_t ti = static_cast<size_t>(target_id);
    const bool go2_pair =
        enable_go2_z_ &&
        qi < keyframes_go2_z_copied_.size() &&
        ti < keyframes_go2_z_copied_.size() &&
        std::isfinite(keyframes_go2_z_copied_[qi]) &&
        std::isfinite(keyframes_go2_z_copied_[ti]);
    double candidate_dz = std::numeric_limits<double>::quiet_NaN();
    if (go2_pair) {
      candidate_dz = std::abs(keyframes_go2_z_copied_[qi] -
                              keyframes_go2_z_copied_[ti]);
      if (candidate_dz > loop_max_candidate_dz_) {
        RCLCPP_INFO(this->get_logger(),
                    "Loop Rejected (dz): %zu -> %d  |dz|=%.2f m (thr %.2f, go2) overlap=%.2f fit=%.4f icp=%.1fms",
                    i, target_id, candidate_dz, loop_max_candidate_dz_,
                    overlap, fitness_score, icp_ms);
        continue;
      }
    }

    // 2026-11-09: 3-DOF 回环。PCL 的 getFitnessScore() 是均方距离 (m^2),
    // 直接当方差用并设 loop_var_min_ 下限; 默认只让 z/roll/pitch 之外
    // (x, y, yaw) 参与约束。走廊实测：6-DOF 回环把 z std 从原始 LIO 的
    // 0.311 拉到 0.383、末帧从 -0.017 拉到 +0.626; 回环对 xy 是正收益,
    // 对 z 是净负收益。需要旧行为时把两个 loop_constrain_* 都设 true。
    const Eigen::VectorXd noise_vector = simple_loop_closure::makeLoopVariances(
        fitness_score, loop_var_scale_, loop_var_min_,
        loop_constrain_z_, loop_constrain_attitude_, loop_var_free_);
    gtsam::SharedNoiseModel constraint_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(noise_vector));

    Eigen::Affine3d pose_diff = registration_result.inverse() * (target_pose);

    // 2026-11-07 回环边一致性门限:
    // 里程计预测的相对位姿 = pose_query^-1 * target_pose; ICP 修正后的相对
    // 位姿 = pose_diff. 二者的平移差就是这条边要把图拽多远. 走廊/洞穴的墙
    // 面高度自相似, ICP 能在完全不同的位置找到 MSE 很低(0.07 量级)的对齐,
    // fitness 门限拦不住; 这种边一旦进图, iSAM2 只能在互相矛盾的 z 约束
    // 之间折中. 实测 corridor: 95 条接受边里 33 条 |d|>0.3 m, 最大 4.6 m,
    // 把 PGO 的 z 跨度从 LIO 的 1.28 m 摊到 2.05 m.
    const Eigen::Affine3d odom_rel = pose_query.inverse() * target_pose;
    const Eigen::Vector3d trans_delta = pose_diff.translation() - odom_rel.translation();
    // trans_delta is expressed in the query/body frame; rotate it to the
    // world frame before taking xy.  Otherwise roll/pitch leaks world z into
    // the "xy" gate on cave slopes.
    const Eigen::Vector3d trans_delta_world =
        pose_query.rotation() * trans_delta;
    // z 不参与约束时，门限也必须只看 xy；否则一个正常的 z 差会把
    // 长基线回环整条拒掉（旧 0.5 m 门限实测会拒 59/95 条）。
    const double d_xy =
        std::hypot(trans_delta_world.x(), trans_delta_world.y());
    const double gate_delta = loop_constrain_z_ ? trans_delta.norm() : d_xy;
    const double path_len = std::abs(
        trajectory_dist_copied_[i] - trajectory_dist_copied_[target_id]);
    const double gate_limit = simple_loop_closure::makeLoopGate(
        path_len, loop_max_trans_delta_, loop_max_trans_delta_ratio_,
        loop_max_trans_delta_cap_);
    if (gate_delta > gate_limit)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Loop Rejected (inconsistent): %zu -> %d  |d_xy|=%.2f |d_xyz|=%.2f m "
                  "d_world=(%+.2f,%+.2f,%+.2f) path=%.1f gate=%.2f  fit=%.4f",
                  i, target_id, d_xy, trans_delta.norm(),
                  trans_delta_world.x(), trans_delta_world.y(),
                  trans_delta_world.z(), path_len, gate_limit, fitness_score);
      continue;
    }
    // 2026-11-11 P0: 去重. 同一 target 在 dedup 窗口内的重复边不增加信息,
    // 只会把同一处错误/正确测量重复计权; 窗口按查询关键帧计.
    if (simple_loop_closure::isDuplicateLoop(
            static_cast<int>(i), target_id, last_loop_query_for_target_,
            loop_dedup_window_))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Loop Rejected (duplicate): %zu -> %d  last=%d window=%d overlap=%.2f fit=%.4f",
                  i, target_id, last_loop_query_for_target_[target_id],
                  loop_dedup_window_, overlap, fitness_score);
      continue;
    }
    last_loop_query_for_target_[target_id] = static_cast<int>(i);

    RCLCPP_INFO(this->get_logger(),
                "Loop Accepted: %zu -> %d  |d_xy|=%.2f |d_xyz|=%.2f path=%.1f gate=%.2f "
                "overlap=%.2f yaw=%+.2f dz=%.2f fit=%.4f icp=%.1fms",
                i, target_id, d_xy, trans_delta.norm(), path_len, gate_limit,
                overlap, icp_yaw, candidate_dz, fitness_score, icp_ms);

    if (simple_loop_closure::useProjected3Dof(
            loop_factor_mode_, loop_constrain_z_, loop_constrain_attitude_)) {
      // 2026-11-12 P1-A: 真 3-DOF. 残差只含 world xy + yaw, z/roll/pitch
      // 不进入因子, 所以 yaw/姿态修正无法通过机体系平移残差耦合进世界 z.
      const Eigen::Vector2d measured_xy(
          target_pose.translation().x() - registration_result.translation().x(),
          target_pose.translation().y() - registration_result.translation().y());
      const double measured_yaw = simple_loop_closure::yawDelta(
          target_pose.rotation(), registration_result.rotation());
      if (!measured_xy.allFinite() || !std::isfinite(measured_yaw)) {
        RCLCPP_WARN(this->get_logger(),
                    "Loop Rejected (non-finite): %zu -> %d  xy=(%.3f,%.3f) yaw=%.3f",
                    i, target_id, measured_xy.x(), measured_xy.y(),
                    measured_yaw);
        continue;  // do not poison iSAM2
      }
      Eigen::VectorXd v(3);
      v << noise_vector(2), noise_vector(3), noise_vector(4);  // yaw, x, y
      const gtsam::SharedNoiseModel projected_noise =
          gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Cauchy::Create(1),
              gtsam::noiseModel::Diagonal::Variances(v));
      graph.add(boost::make_shared<simple_loop_closure::WorldXYPhiFactor>(
          i, target_id, measured_xy, measured_yaw, projected_noise));
    } else {
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          i, target_id, gtsam::Pose3(pose_diff.matrix()), constraint_noise));
    }
    // DIAG 2026-11-07: is the loop edge's z trustworthy?
    //   rel_z       z the constraint asserts between the two keyframes
    //   odom_rel_z  z the odometry chain predicted for the same pair
    //   rel_z - odom_rel_z  what the edge actually pulls the graph by
    //   shift       how far ICP moved the source from its prediction
    //               (world axes, so shift_z is the vertical correction)
    // A single-floor site must give rel_z ~= odom_rel_z ~= 0.  Repeated
    // |rel_z - odom_rel_z| of 0.3..1 m means ICP is fabricating a vertical
    // offset (corridor slip / false pair) instead of closing the loop.
    {
      const Eigen::Vector3d shift =
          registration_result.translation() - pose_query.translation();
      const double rel_z = pose_diff.translation().z();
      const double odom_rel_z = (pose_query.inverse() * target_pose).translation().z();
      RCLCPP_INFO(this->get_logger(),
                  "Loop Detected: %zu -> %d  rel_z=%+.2f (odom %+.2f, d=%+.2f)  "
                  "shift=(%+.2f,%+.2f,%+.2f)  fit=%.4f",
                  i, target_id, rel_z, odom_rel_z, rel_z - odom_rel_z,
                  shift.x(), shift.y(), shift.z(), fitness_score);
    }

    {
      MtxLockGuard guard(mtx_res_);
      loop_edges_.push_back(LoopEdgeID(i, target_id));
    }
    // A loop closure corrects the poses of old keyframes; the cached
    // visualization map must be rebuilt from the new estimate.
    vis_cache_rebuild_requested_ = true;
  }
  searched_loop_id_ = keyframes_cloud_copied_.size();

  return true;
}

bool SimpleLoopClosureNode::updateISAM2(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& init_estimate)
{
  if (graph.empty())
    return false;

  if (init_estimate.empty())
    isam2_.update(graph);
  else
    isam2_.update(graph, init_estimate);
  isam2_.update();

  {
    MtxLockGuard guard(mtx_res_);
    optimization_result_ = isam2_.calculateEstimate();
  }

  return true;
}

void SimpleLoopClosureNode::visualizeThread()
{
  rclcpp::Rate rate(vis_rate_hz_);
  while (rclcpp::ok() && !stop_visualize_thread_)
  {
    const auto vis_start = std::chrono::steady_clock::now();
    if (vis_incremental_)
    {
      updateVisualizationMapCache();
      publishCachedMapCloud();
    }
    else
    {
      PointCloudType::Ptr map_cloud = constructPointCloudMap(vis_map_cloud_frame_interval_);
      publishMapCloud(map_cloud);
    }
    publishVisualizationGraph();
    const double vis_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - vis_start).count();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Vis thread: %.1f ms/cycle (target %.1f Hz)",
                         vis_ms, vis_rate_hz_);
    rate.sleep();
  }
}

void SimpleLoopClosureNode::loopCloseThread()
{
  rclcpp::Rate rate(loop_rate_hz_);
  while (rclcpp::ok() && !stop_loop_closure_thread_)
  {
    const auto loop_start = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values init_estimate;

    copyKeyFrames();
    constructOdometryGraph(graph, init_estimate);
    constructLoopEdge(graph);
    constructPriorEdges(graph);
    updateISAM2(graph, init_estimate);

    if (enable_go2_z_ || enable_go2_attitude_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Go2 factors: z=%d rp=%d (keyframes=%zu)",
                           go2_z_factor_count_, go2_rp_factor_count_,
                           keyframes_odom_copied_.size());
      if (enable_go2_z_) {
        // DIAG 2026-11-12: are the z factors actually satisfied?  If rms/max
        // stay small while the PGO z curve drifts, the Go2 *measurements*
        // themselves are drifting; if residual grows with the z error, the
        // robust kernel is down-weighting them.
        double sum_sq = 0.0;
        double max_abs = 0.0;
        int n = 0;
        {
          MtxLockGuard guard(mtx_res_);
          const size_t nk = std::min(
              optimization_result_.size(), keyframes_go2_z_copied_.size());
          for (size_t i = 1; i < nk; ++i) {
            if (!std::isfinite(keyframes_go2_z_copied_[i]) ||
                !std::isfinite(keyframes_go2_z_copied_[i - 1])) {
              continue;
            }
            const double pred =
                optimization_result_.at<gtsam::Pose3>(i).z() -
                optimization_result_.at<gtsam::Pose3>(i - 1).z();
            const double meas = keyframes_go2_z_copied_[i] -
                                keyframes_go2_z_copied_[i - 1];
            const double e = pred - meas;
            sum_sq += e * e;
            max_abs = std::max(max_abs, std::abs(e));
            ++n;
          }
        }
        if (n > 0) {
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "Go2 z residual: rms=%.4f m max=%.4f m n=%d", std::sqrt(sum_sq / n),
              max_abs, n);
        }
      }
      if (!go2_zero_warned_ && keyframes_odom_copied_.size() > 50 &&
          ((enable_go2_z_ && go2_z_factor_count_ == 0) ||
           (enable_go2_attitude_ && go2_rp_factor_count_ == 0))) {
        RCLCPP_WARN(this->get_logger(),
                    "Go2 factor enabled but zero factors after %zu keyframes; "
                    "check %s / QoS / arrival time",
                    keyframes_odom_copied_.size(), go2_pose_topic_.c_str());
        go2_zero_warned_ = true;
      }
    }

    if (enable_leg_z_) {
      simple_loop_closure::LegZDiagnostics d;
      double leg_z_cum = 0.0;
      {
        std::lock_guard<std::mutex> lock(mtx_leg_z_);
        d = leg_z_integrator_->diagnostics();
        leg_z_cum = leg_z_integrator_->z();
      }
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Leg z: n=%d used=%ld skipped=%ld (no_att=%ld att_jump=%ld "
          "no_contact=%ld consensus=%ld big_dz=%ld non_finite=%ld dt=%ld "
          "sport_miss=%ld) cumulative=%.3f m",
          leg_z_factor_count_, d.steps_used, d.steps_skipped,
          d.skip_no_attitude, d.skip_attitude_jump, d.skip_no_contact,
          d.skip_consensus, d.skip_big_dz, d.skip_non_finite, d.skip_dt,
          leg_sport_missing_count_, leg_z_cum);
      if (!leg_z_zero_warned_ && keyframes_odom_copied_.size() > 50 &&
          leg_z_factor_count_ == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "enable_leg_z but zero factors after %zu keyframes; "
                    "check %s / %s / arrival time",
                    keyframes_odom_copied_.size(),
                    leg_lowstate_topic_.c_str(), go2_pose_topic_.c_str());
        leg_z_zero_warned_ = true;
      }
      if (!leg_z_no_att_warned_ && keyframes_odom_copied_.size() > 50 &&
          d.steps_used == 0 && d.skip_no_attitude > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "leg z enabled but no valid %s attitude; z factors "
                    "suppressed (no_att=%ld)",
                    go2_pose_topic_.c_str(), d.skip_no_attitude);
        leg_z_no_att_warned_ = true;
      }
      if (leg_lowstate_count_ == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "leg z enabled but no %s samples",
                             leg_lowstate_topic_.c_str());
      }
      if (leg_time_source_ == "device" && leg_sport_missing_count_ > 0 &&
          leg_lowstate_count_ > 0 &&
          leg_sport_missing_count_ * 2 > leg_lowstate_count_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "leg z device time: no %s pair for %ld/%ld lowstate samples; "
            "falling back is disabled to avoid mixing clocks",
            leg_sport_topic_.c_str(), leg_sport_missing_count_,
            leg_lowstate_count_.load());
      }
    }

    // DIAG 2026-11-07: /aft_pgo_odom only shows the *current* pose, so a
    // vertical spread of the accumulated poses is invisible there.  The map
    // is built from every pose, so log the z range across all of them.
    {
      MtxLockGuard guard(mtx_res_);
      if (!optimization_result_.empty())
      {
        double zmin = 1e9, zmax = -1e9;
        size_t imin = 0, imax = 0;
        for (size_t i = 0; i < optimization_result_.size(); ++i)
        {
          const double z = optimization_result_.at<gtsam::Pose3>(i).translation().z();
          if (z < zmin) { zmin = z; imin = i; }
          if (z > zmax) { zmax = z; imax = i; }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "pose z range: %.2f (i=%zu) .. %.2f (i=%zu), n=%zu",
          zmin, imin, zmax, imax, optimization_result_.size());
      }
    }

    const double loop_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - loop_start).count();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Loop thread: %.1f ms/cycle (target %.1f Hz)",
                         loop_ms, loop_rate_hz_);
    rate.sleep();
  }
}

void SimpleLoopClosureNode::spin()
{
  RCLCPP_INFO(this->get_logger(), "Loop Closure Started");
  // Multi-threaded so the leg lowstate callback (dedicated callback group)
  // is not blocked by the point-cloud callback.  All other subscriptions
  // stay in the default mutually-exclusive group and remain serialized.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shared_from_this());
  executor.spin();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto loop_closure_node = std::make_shared<SimpleLoopClosureNode>();
  loop_closure_node->spin();
  rclcpp::shutdown();
  return 0;
}
