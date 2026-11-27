// transit_node.hpp — ROS 接线层。
//
// 只做"翻译"：把状态机的下一条指令变成 nav2 的 FollowPath / Spin 动作调用，
// 再把结果喂回状态机。不直接发布 cmd_vel，不做全局规划，任何回调都不阻塞。

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "go2_transit/action/rotate_to_yaw.hpp"
#include "go2_transit/action/transit_to_pose.hpp"
#include "go2_transit/transit_core.hpp"

namespace go2_transit
{

class TransitNode : public rclcpp::Node
{
public:
  explicit TransitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 进程退出前调用：尽力取消在飞的 nav2 动作。
  // rclcpp_action 客户端析构不会替服务端取消 goal，不取消的话节点退出后
  // controller_server / behavior_server 会继续把这条动作执行完 —— 狗会继续走。
  void cancel_all_on_shutdown(rclcpp::executors::SingleThreadedExecutor & executor);

private:
  using TransitToPose = go2_transit::action::TransitToPose;
  using RotateToYaw = go2_transit::action::RotateToYaw;
  using FollowPath = nav2_msgs::action::FollowPath;
  using Spin = nav2_msgs::action::Spin;

  using TransitGoalHandle = rclcpp_action::ServerGoalHandle<TransitToPose>;
  using RotateGoalHandle = rclcpp_action::ServerGoalHandle<RotateToYaw>;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<Spin>;

  enum class ActiveKind { NONE, TRANSIT, ROTATE };

  // ---- 动作服务器 ----
  rclcpp_action::GoalResponse handle_transit_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TransitToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_transit_cancel(
    const std::shared_ptr<TransitGoalHandle> goal_handle);
  void handle_transit_accepted(const std::shared_ptr<TransitGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handle_rotate_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RotateToYaw::Goal> goal);
  rclcpp_action::CancelResponse handle_rotate_cancel(
    const std::shared_ptr<RotateGoalHandle> goal_handle);
  void handle_rotate_accepted(const std::shared_ptr<RotateGoalHandle> goal_handle);

  // ---- 主循环 ----
  void tick();
  void publish_status();
  void on_stop(const std_msgs::msg::Empty::ConstSharedPtr msg);

  void start_transit(const std::shared_ptr<TransitGoalHandle> & goal_handle);
  void start_rotate(const std::shared_ptr<RotateGoalHandle> & goal_handle);
  void preempt_active();

  TransitParams machine_params() const;
  bool resolve_transit_goal();
  std::optional<Pose2D> lookup_pose();
  bool nav2_ready() const;

  void dispatch(const MachineCommand & cmd);
  void dispatch_follow_path(const MachineCommand & cmd);
  void dispatch_spin(const MachineCommand & cmd);
  void check_follow_path_timeout();
  void simulate_dispatch(const MachineCommand & cmd);
  void cancel_in_flight();
  void finish_active_goal();
  void fill_feedback();

  double elapsed_since_goal_start() const;
  Pose2D current_target_pose() const;

  // ---- 参数 ----
  std::string planning_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  double default_xy_tolerance_{0.20};
  double default_yaw_tolerance_{0.17};
  double path_spacing_m_{0.25};
  double pre_align_threshold_rad_{0.5};
  bool pre_align_fallback_{true};
  double follow_path_timeout_s_{300.0};
  double spin_timeout_s_{30.0};
  double total_timeout_s_{360.0};
  double ready_wait_s_{20.0};
  int max_retries_{2};
  std::string controller_id_{"FollowPath"};
  std::string goal_checker_id_{"general_goal_checker"};
  bool dry_run_{false};
  double status_rate_hz_{2.0};
  std::string action_namespace_{"go2_transit"};

  // ---- ROS 接口 ----
  rclcpp_action::Server<TransitToPose>::SharedPtr transit_server_;
  rclcpp_action::Server<RotateToYaw>::SharedPtr rotate_server_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- 活动任务状态 ----
  ActiveKind active_{ActiveKind::NONE};
  std::shared_ptr<TransitGoalHandle> transit_handle_;
  std::shared_ptr<RotateGoalHandle> rotate_handle_;
  std::unique_ptr<TransitStateMachine> machine_;

  geometry_msgs::msg::PoseStamped raw_target_;
  Pose2D target_{};
  double goal_xy_tolerance_{0.0};
  bool goal_align_yaw_{false};
  double goal_yaw_tolerance_{0.0};
  bool goal_resolved_{false};
  bool rotate_goal_{false};

  std::chrono::steady_clock::time_point goal_started_at_{};
  std::uint64_t last_dispatched_id_{0};
  std::uint64_t dispatch_epoch_{0};
  FollowPathGoalHandle::SharedPtr fp_handle_;
  SpinGoalHandle::SharedPtr spin_handle_;

  bool cancel_requested_{false};
  std::optional<MachineCommand> sim_pending_cmd_;
  bool ready_reported_{false};
  bool fp_in_flight_{false};
  std::chrono::steady_clock::time_point fp_dispatched_at_{};
  // dry_run 只在第一次拿到真实位姿时播种，之后位姿由模拟推进。
  bool dry_run_pose_seeded_{false};
  Pose2D last_pose_{};
  bool have_pose_{false};
  FailReason last_fail_reason_{FailReason::OK};
};

}  // namespace go2_transit
