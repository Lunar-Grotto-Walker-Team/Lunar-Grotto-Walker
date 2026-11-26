#pragma once

#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

#include "simple_loop_closure/prior_map.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include "simple_loop_closure/go2_buffer.hpp"
#include "simple_loop_closure/gravity_buffer.hpp"
#include "simple_loop_closure/ground_diagnostics.hpp"
#include "simple_loop_closure/leg_kinematics.hpp"
#include "simple_loop_closure/leg_time_buffer.hpp"
#include "simple_loop_closure/leg_z_integrator.hpp"
#include "simple_loop_closure/loop_quality.hpp"

#include <nanoflann.hpp>

using Go2Sample = simple_loop_closure::Go2Sample;

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>
#include <deque>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>
#include <unordered_set>
#include <unordered_map>

class SimpleLoopClosureNode : public rclcpp::Node
{
private: 
    typedef pcl::PointXYZI PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    // Visualization map is published as XYZ only.  Nothing downstream uses
    // intensity (pgo_gridmap_projector reads x/y/z by field offset), and
    // PointXYZ is 16 B/point vs 32 B for PointXYZI (PCL pads to 16-byte
    // alignment), i.e. /aft_pgo_map halves in size at the same density.
    typedef pcl::PointXYZ VisPointType;
    typedef pcl::PointCloud<VisPointType> VisCloudType;
    typedef std::lock_guard<std::mutex> MtxLockGuard;
    typedef std::shared_ptr<Eigen::Affine3d> Affine3dPtr;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 3> KDTreeMatrix;
    typedef nanoflann::KDTreeEigenMatrixAdaptor<KDTreeMatrix, 3, nanoflann::metric_L2_Simple> KDTree;
    typedef std::vector<nanoflann::ResultItem<long int, double>> NanoFlannSearchResult;
    typedef std::pair<int, int> LoopEdgeID;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_pose_graph_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pgo_odometry_;


    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
    std::shared_ptr<Sync> synchronizer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_req_;

    // ---- Go2 soft factors (2026-11-09) ---------------------------------
    // /utlidar/robot_pose is the Go2 internal lidar SLAM pose: same physical
    // lidar frame as Point-LIO, but an independent algorithm.  Its z is
    // stable to ~5 mm over the corridor bag while Point-LIO drifts ~0.3 m.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_go2_pose_;
    std::deque<Go2Sample> go2_buf_;
    std::mutex mtx_go2_;
    std::deque<double> keyframes_go2_z_;
    std::deque<Eigen::Quaterniond> keyframes_go2_q_;
    std::deque<double> keyframes_go2_z_copied_;
    std::deque<Eigen::Quaterniond> keyframes_go2_q_copied_;

    // ---- Leg kinematic world-z factor (2026-11-15) ----------------------
    // /lf/lowstate joints + foot forces -> URDF FK -> foot-anchor consensus
    // -> cumulative world z, sampled at keyframes and added as a ZDiffFactor.
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
        sub_leg_lowstate_;
    std::unique_ptr<simple_loop_closure::LegZIntegrator> leg_z_integrator_;
    simple_loop_closure::LegGeometry leg_geometry_;
    std::mutex mtx_leg_z_;
    double leg_z_latest_stamp_{0.0};
    double leg_z_latest_{0.0};
    bool leg_z_latest_valid_{false};
    std::deque<double> keyframes_leg_z_;
    std::deque<double> keyframes_leg_z_copied_;
    std::atomic<long> leg_lowstate_count_{0};  // callback -> loop thread
    int leg_z_factor_count_{0};
    bool leg_z_zero_warned_{false};
    bool leg_z_no_att_warned_{false};
    long leg_z_kf_steps_used_{0};
    long leg_sport_missing_count_{0};
    std::string leg_sport_topic_;
    std::string leg_time_source_;
    std::deque<simple_loop_closure::LegTimeSample> leg_time_buf_;
    rclcpp::CallbackGroup::SharedPtr leg_cb_group_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
        sub_leg_sport_;
    bool enable_leg_z_;
    std::string leg_lowstate_topic_;
    double leg_z_var_;
    double leg_z_robust_scale_;
    double leg_z_timeout_;
    double leg_attitude_jump_deg_;
    double leg_contact_force_on_;
    double leg_contact_force_off_;
    double leg_single_force_;
    double leg_settle_s_;
    double leg_consensus_spread_;
    double leg_max_step_dz_;
    double leg_pitch_offset_deg_;
    double leg_roll_offset_deg_;
    double leg_offset_x_;
    double leg_offset_y_;
    double leg_thigh_offset_;
    double leg_thigh_len_;
    double leg_calf_len_;

    // ---- IMU gravity soft prior (2026-11-13 P2) -------------------------
    // Every source (lowstate / utlidar / livox) is reduced to a specific
    // force in its own frame; `gravity_extrinsic_` maps that to the LIO body
    // frame (both are z-up; the default identity covers the standard
    // mounting, gravity_extrinsic_rpy absorbs a measured tilt).
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
        sub_gravity_lowstate_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gravity_imu_;
    std::deque<simple_loop_closure::GravitySample> gravity_buf_;
    std::mutex mtx_gravity_;
    std::deque<double> keyframes_stamp_;
    std::deque<double> keyframes_stamp_copied_;
    bool enable_gravity_prior_;
    std::string gravity_source_;
    std::string gravity_topic_;
    double gravity_var_deg_;
    double gravity_robust_scale_;
    double gravity_timeout_;
    double gravity_lowpass_s_;
    std::string gravity_extrinsic_rpy_;
    double gravity_roll_offset_;
    double gravity_pitch_offset_;
    bool allow_gravity_z_diff_coexist_;
    Eigen::Matrix3d gravity_extrinsic_;
    bool go2_attitude_active_;
    double gravity_norm_tol_;
    double gravity_norm_ema_[3];  // per source: lowstate / utlidar / livox
    int gravity_source_id_;
    int gravity_factor_count_;
    int gravity_fail_count_;
    int gravity_dynamic_skip_count_;
    int gravity_invalid_count_;
    bool gravity_zero_warned_;
    bool floor_frame_warned_;
    bool enable_go2_z_;
    double go2_z_var_;
    double go2_z_robust_scale_;
    double go2_pose_timeout_;
    std::string go2_pose_topic_;
    int go2_z_factor_count_;
    bool enable_go2_attitude_;
    double go2_attitude_var_deg_;
    double go2_roll_offset_;
    double go2_pitch_offset_;
    bool go2_rp_offset_set_;
    int go2_rp_factor_count_;
    bool go2_zero_warned_;

    std_msgs::msg::ColorRGBA odom_edge_color_;
    std_msgs::msg::ColorRGBA loop_edge_color_;
    std_msgs::msg::ColorRGBA node_color_;
    double edge_scale_;
    double node_scale_;

    std::mutex mtx_buf_;
    std::deque<PointCloudType::Ptr> keyframes_cloud_;
    std::deque<Affine3dPtr> keyframes_odom_;
    std::deque<PointCloudType::Ptr> keyframes_cloud_copied_;
    std::deque<Affine3dPtr> keyframes_odom_copied_;
    std::deque<double> trajectory_dist_;
    std::deque<double> trajectory_dist_copied_;
    std::string odom_frame_id_;

    size_t added_odom_id_;
    size_t searched_loop_id_;

    // Prior map (2026-11-19): constraints against a previous session's cave
    // map.  The prior poses are constants, so an accepted registration
    // becomes a unary absolute factor on the query keyframe instead of a
    // sparse node in optimization_result_.
    bool enable_prior_map_;
    std::string prior_map_dir_;
    double prior_search_radius_;
    double prior_max_trans_delta_;
    double prior_icp_max_corr_dist_;
    double prior_overlap_min_;
    int prior_dedup_window_;
    double prior_var_scale_;
    std::vector<simple_loop_closure::PriorPose> prior_poses_;
    std::vector<PointCloudType::Ptr> prior_clouds_;
    simple_loop_closure::PriorIndex prior_index_;
    std::unordered_map<int, int> last_prior_query_for_target_;
    size_t prior_searched_id_;
    int prev_prior_query_;
    int prev_prior_target_;
    pcl::IterativeClosestPoint<PointType, PointType> prior_icp_;

    gtsam::ISAM2 isam2_;
    std::mutex mtx_res_;
    gtsam::Values optimization_result_;
    std::deque<LoopEdgeID> loop_edges_;

    gtsam::SharedNoiseModel prior_noise_;
    gtsam::SharedNoiseModel odom_noise_;
    gtsam::SharedNoiseModel const_loop_edge_noise_;

    // 2026-11-07: 噪声模型参数化. 旧值 odom 方差 1e-6 (标准差 1 mm) 把图钉死
    // 在 LIO 相对位姿上, 回环边用 fitness(0.01 量级) 当方差, 比里程计弱 4-6 个
    // 数量级, 实测 PGO 输出几乎不修正 LIO 的 z 漂移. 现在里程计按每关键帧
    // ~1 cm 建模, 回环按 ICP 的 MSE 建模并设下限, 让回环真正能把长走廊的
    // 纵向/高度误差拉回来.
    double odom_trans_var_;   // m^2
    // 2026-11-15: constrain odometry translation in the world frame.
    bool use_world_odom_trans_;
    double odom_rot_var_;     // rad^2
    // 2026-11-13: world-z continuity per odometry edge (m^2).  Keeps the
    // graph's vertical shape on the LIO odometry even when loop edges change
    // yaw/attitude (the body-frame odometry edge then re-projects translation
    // and can move world z).  0 disables.
    double odom_z_diff_var_;
    double loop_var_scale_;   // 回环方差 = fitness * scale
    double loop_var_min_;     // 回环方差下限 (m^2)

    // 2026-11-07: 回环边质量控制.
    //   icp_max_corr_dist_: ICP 对应点最大距离 (米). 旧代码把 search_radius_
    //     平方后的值再 x2 传给 setMaxCorrespondenceDistance (=200 m, 等于
    //     不设限), 走廊里 ICP 可以整面墙滑过去而不被拒.
    //   loop_max_trans_delta_: 回环边与里程计预测的平移差**下限** (米).
    //     真实门限是 max(this, ratio * 两关键帧间路径长度). ICP 得到的
    //     相对位姿和里程计预测差得太多, 说明它匹配到的是别处自相似的墙,
    //     而不是真正的回访.
    double icp_max_corr_dist_;
    double loop_max_trans_delta_;
    // 2026-11-17: hard cap for the path-proportional gate (0 = no cap).
    double loop_max_trans_delta_cap_;
    // 2026-11-09: 距离比例门限。false loop 的 ICP 平移差可以到 7 m，
    // 固定门限要么全放行要么拒掉长基线真回环；用
    // max(abs_floor, ratio * path_len) 区分两者。
    double loop_max_trans_delta_ratio_;
    // 2026-11-09: 回环边默认只约束 x/y/yaw (3-DOF). z/roll/pitch 的方差
    // 设 loop_var_free_, 等价于让里程计链 (和可选的 Go2 软因子) 决定.
    bool loop_constrain_z_;
    bool loop_constrain_attitude_;
    double loop_var_free_;
    // 2026-11-12 P1: "projected_3dof" (default) or "diagonal" (P0 fallback).
    std::string loop_factor_mode_;
    // 2026-11-13: log ground-plane attitude/height every N keyframes (0 = off).
    int ground_diag_interval_;

    // ---- Floor-normal continuity (2026-11-13 P2) ------------------------
    // Per-keyframe up-normal in the body frame (NaN when no valid floor).
    // The factor bridges the current valid frame to the last valid one,
    // which is what pulls a tilted pose back after the floor reappears.
    bool enable_floor_normal_factor_;
    double floor_normal_var_deg_;
    double floor_normal_max_step_deg_;
    double floor_normal_max_gap_keyframes_;
    int floor_normal_factor_count_;
    int floor_normal_skip_gap_count_;
    int floor_normal_skip_step_count_;
    std::deque<Eigen::Vector3d> keyframes_floor_normal_;
    std::deque<Eigen::Vector3d> keyframes_floor_normal_copied_;

    // 2026-11-11 P0: 进图前的几何质量门控. 这些都是"回环候选有多可信"的
    // 场景无关统计量, 用来在洞穴/窄道这类自相似环境里挡掉误检.
    //   overlap_min_: ICP 后源点云落在目标子图 overlap_dist 内的比例下限.
    //   overlap_dist_: 判定重叠的欧氏距离 (米).
    //   loop_max_yaw_delta_: ICP yaw 相对里程计预测的最大修正 (rad).
    //   loop_max_candidate_dz_: 候选关键帧的高度差上限 (米); 跨层回环在这里
    //     被挡掉, 同一层/坡面回环不受影响.
    //   loop_dedup_window_: 同一 target 在这么多查询帧内只保留第一条边.
    double overlap_min_;
    double overlap_dist_;
    double loop_max_yaw_delta_;
    double loop_max_candidate_dz_;
    int loop_dedup_window_;
    std::unordered_map<int, int> last_loop_query_for_target_;

    pcl::IterativeClosestPoint<PointType, PointType> icp_;
    pcl::VoxelGrid<PointType> vg_target_;
    pcl::VoxelGrid<PointType> vg_source_;
    pcl::VoxelGrid<PointType> vg_map_;

    bool mapped_cloud_;
    double time_stamp_tolerance_;
    double keyframe_dist_th_;
    double keyframe_angular_dist_th_;
    double loop_search_time_diff_th_;
    double loop_search_dist_diff_th_;
    double loop_search_angular_dist_th_;
    int loop_search_frame_interval_;
    double search_radius_;
    int target_frame_num_;
    double target_voxel_leaf_size_;
    double source_voxel_leaf_size_;
    double vis_map_voxel_leaf_size_;
    double fitness_score_th_;
    int vis_map_cloud_frame_interval_;

    // ---- Incremental visualization map cache ----------------------------
    // The old path re-transformed every keyframe and re-ran the 0.05 m voxel
    // filter over the whole map every second: O(total points), which is what
    // dragged /aft_pgo_map below 1 Hz as the map grew.  Instead, transform
    // each keyframe once and insert its points into a 0.05 m voxel set
    // (first-write-wins); publish the accumulated cloud as-is.  A loop
    // closure changes old poses, so it requests a full rebuild; a periodic
    // rebuild also absorbs the small pose refinements iSAM2 makes without a
    // closure.  Set vis_incremental: false to get the old behaviour.
    bool vis_incremental_;
    double vis_full_rebuild_interval_s_;
    std::unordered_set<uint64_t> vis_voxel_keys_;
    VisCloudType vis_map_cache_;
    size_t vis_cache_added_id_;
    int64_t vis_last_full_rebuild_ns_;
    std::atomic<bool> vis_cache_rebuild_requested_;

    // 2026-11-12: thread rates.  The old hard-coded 1 Hz is too slow for a
    // fast robot: between two optimisations it travels ~1 m and ~8 keyframes,
    // and the map/elevation chain sees a stale pose for up to a second.
    // loop_rate_hz drives loop search + iSAM2; vis_rate_hz drives
    // /aft_pgo_map and /vis_pose_graph.  Keep them separate because map
    // serialisation is much more expensive than the graph update.
    double loop_rate_hz_;
    double vis_rate_hz_;
    std::atomic<bool> stop_loop_closure_thread_;
    std::atomic<bool> stop_visualize_thread_;

    std::thread loop_close_thread_;
    std::thread visualize_thread_;
    std::thread save_thread_;
    bool saving_;
    std::string save_directory_;

    void initialize();

    bool saveEachFrames();

    void saveThread();

    void saveRequestCallback(const std_msgs::msg::String::ConstSharedPtr &directory);

    void publishPoseGraphOptimizedOdometry(const Eigen::Affine3d &affine_curr, const nav_msgs::msg::Odometry &odom_msg);

    // 2026-11-13: per-keyframe ground-plane diagnostics (log only).
    void logGroundDiagnostics(
        const simple_loop_closure::GroundPlane &floor,
        const Eigen::Affine3d &lio_pose, size_t id);

    void pointCloudAndOdometryCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);

    void go2PoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);

    void sportModeCallback(
        const unitree_go::msg::SportModeState::ConstSharedPtr &msg);

    void legLowStateCallback(
        const unitree_go::msg::LowState::ConstSharedPtr &msg);

    void gravityLowStateCallback(
        const unitree_go::msg::LowState::ConstSharedPtr &msg);
    void gravityImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    void pushGravitySample(const simple_loop_closure::GravitySample &sample);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructPointCloudMap(const int interval = 0);
    // PointCloudType::Ptr constructPointCloudMap(const int interval = 0);

    void publishMapCloud(const PointCloudType::Ptr &map_cloud);

    // Incremental vis-map helpers (visualizeThread owns the cache).
    uint64_t voxelKey(double x, double y, double z) const;
    void clearVisualizationMapCache();
    void appendNewKeyframesToCache();
    void updateVisualizationMapCache();
    void publishCachedMapCloud();

    void constructVisualizationOdometryEdges(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void constructVisualizationLoopEdges(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void constructVisualizationNodes(const int &res_size, const std_msgs::msg::Header &header, visualization_msgs::msg::Marker &marker_msg);

    void publishVisualizationGraph();

    void visualizeThread();

    void copyKeyFrames();

    void getLastOptimizedPose(Eigen::Affine3d &pose, int &id);

    bool constructOdometryGraph(gtsam::NonlinearFactorGraph &graph, gtsam::Values &init_estimate);

    bool constructKDTreeMatrix(KDTreeMatrix &kdtree_mat);

    int searchTarget(const KDTree &kdtree, const int &id_query, const Eigen::Affine3d &pose_query, const rclcpp::Time &stamp_query);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructTargetCloud(const int target_id);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructSourceCloud(const int source_id);

    bool tryRegistration(const Eigen::Affine3d &init_pose, const PointCloudType::Ptr &source_cloud, const PointCloudType::Ptr &target_cloud, Eigen::Affine3d &result, double &score, double &overlap, double &icp_ms);

    bool tryRegistrationWith(pcl::IterativeClosestPoint<PointType, PointType> &icp, const Eigen::Affine3d &init_pose, const PointCloudType::Ptr &source_cloud, const PointCloudType::Ptr &target_cloud, Eigen::Affine3d &result, double &score, double &overlap, double &icp_ms);

    bool loadPriorMap();

    bool constructPriorEdges(gtsam::NonlinearFactorGraph &graph);

    pcl::PointCloud<pcl::PointXYZI>::Ptr constructPriorTargetCloud(const int prior_id);

    bool constructLoopEdge(gtsam::NonlinearFactorGraph &graph);

    bool updateISAM2(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &init_estimate);

    void loopCloseThread();

public:
    typedef std::shared_ptr<SimpleLoopClosureNode> Ptr;

    SimpleLoopClosureNode();

    // The class owns three std::thread members, so the defaulted destructor
    // called std::terminate on every shutdown: destroying a joinable thread
    // aborts the process ("terminate called without an active exception",
    // exit code -6). Signal the loops and join them instead. Each cycle is
    // bounded (measured: loop thread <= 200 ms, vis thread <= 55 ms), so the
    // extra wait is sub-second.
    ~SimpleLoopClosureNode()
    {
      stop_loop_closure_thread_ = true;
      stop_visualize_thread_ = true;
      if (loop_close_thread_.joinable()) loop_close_thread_.join();
      if (visualize_thread_.joinable()) visualize_thread_.join();
      if (save_thread_.joinable()) save_thread_.join();
    }

    void spin();
};