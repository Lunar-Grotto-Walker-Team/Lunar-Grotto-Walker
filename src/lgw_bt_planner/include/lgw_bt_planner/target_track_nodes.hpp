// include/lgw_bt_planner/target_track_nodes.hpp
//
// BT leaves for the track-collect validation chain (2026-11-17).  They all
// consume the tracker's /perception/tracks/unified; navigation is manual.
//
//   WaitForTargetSelected  RUNNING until selected_track_id != 0
//   WaitUntilTargetNear    RUNNING until the selected target is within the
//                          XoY band around the arm base (0.6 m default)
//   WaitForArmHits         RUNNING until arm_hits_in_window >= min_hits
//   IsArmCameraAlive       arm camera stream arrived within max_age_ms
//
// The decision math lives in target_track_query.hpp (pure, unit-tested);
// these classes are the ROS subscription shells.
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <rclcpp/subscription.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>
#include <yoloe_interfaces/msg/unified_track_array.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace lgw_bt_planner
{

// Shared "latest UnifiedTrackArray" cache.  Copying a handful of tracks per
// callback is cheap and keeps the callbacks lock-scoped.
struct TrackArrayCache
{
  std::shared_ptr<rclcpp::Subscription<
      yoloe_interfaces::msg::UnifiedTrackArray>> sub;
  mutable std::mutex mutex;
  yoloe_interfaces::msg::UnifiedTrackArray latest;
  bool seen{false};

  void init(rclcpp::Node * node);
  bool snapshot(yoloe_interfaces::msg::UnifiedTrackArray * out) const;
};

// Reactive condition: SUCCESS iff the tracker currently has a fresh selected
// target.  This is the preemption trigger for a ReactiveFallback — unlike
// WaitForTargetSelected it never returns RUNNING, so the fallback can still
// tick the exploration branch.
class IsTargetAvailable
  : public BT::RosTopicSubNode<yoloe_interfaces::msg::UnifiedTrackArray>
{
public:
  IsTargetAvailable(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<yoloe_interfaces::msg::UnifiedTrackArray>(
      name, config, params) {}

  IsTargetAvailable() = delete;
  IsTargetAvailable(const IsTargetAvailable &) = delete;
  IsTargetAvailable & operator=(const IsTargetAvailable &) = delete;

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(
    const std::shared_ptr<
      yoloe_interfaces::msg::UnifiedTrackArray> & last_msg) override;
};

// Reactive condition (2026-11-20): SUCCESS iff the tracker has a fresh
// selected target that is inside the XoY band (and, optionally, the frontal
// sector).  Unlike WaitUntilTargetNear it NEVER returns RUNNING, so it can be
// the left child of a ReactiveFallback to preempt a patrol leg: the patrol
// keeps driving while the condition is FAILURE, and is halted the tick the
// target comes into grab range.
class IsTargetWithin
  : public BT::RosTopicSubNode<yoloe_interfaces::msg::UnifiedTrackArray>
{
public:
  IsTargetWithin(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<yoloe_interfaces::msg::UnifiedTrackArray>(
      name, config, params) {}

  IsTargetWithin() = delete;
  IsTargetWithin(const IsTargetWithin &) = delete;
  IsTargetWithin & operator=(const IsTargetWithin &) = delete;

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(
    const std::shared_ptr<
      yoloe_interfaces::msg::UnifiedTrackArray> & last_msg) override;

private:
  // One INFO line per entering the band, so the log shows WHY the patrol was
  // interrupted without spamming at tick rate.
  bool inside_{false};
};

class WaitForTargetSelected : public BT::StatefulActionNode
{
public:
  WaitForTargetSelected(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  TrackArrayCache cache_;
  int timeout_ms_{60000};
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.wait_for_target_selected")};
};

class WaitUntilTargetNear : public BT::StatefulActionNode
{
public:
  WaitUntilTargetNear(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  TrackArrayCache cache_;
  double min_xy_m_{0.0};
  double max_xy_m_{0.6};
  // 2026-11-19: 目标必须在臂基座 +-max_azimuth_rad 扇区内。
  // 2026-11-20: 扇区中心改由 sector_center_rad 指定(默认 0 = +X 轴;
  // -pi/2 = -Y 轴)。<=0 关闭该判据(保持旧行为: 只看距离)。
  double max_azimuth_rad_{0.0};
  double sector_center_rad_{0.0};
  int timeout_ms_{120000};
  std::chrono::steady_clock::time_point deadline_;
  // 1 Hz heartbeat while waiting (2026-11-18).  Without it a stalled tick
  // thread and "the target is simply not inside the band" look identical in
  // the log: both produce silence until the 120 s timeout warn — which never
  // fires at all if the tick loop is dead.
  rclcpp::Clock heartbeat_clock_{RCL_STEADY_TIME};
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.wait_until_target_near")};
};

class WaitForArmHits : public BT::StatefulActionNode
{
public:
  WaitForArmHits(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  TrackArrayCache cache_;
  int min_hits_{3};
  int timeout_ms_{5000};
  std::chrono::steady_clock::time_point deadline_;
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.wait_for_arm_hits")};
};

class IsArmCameraAlive : public BT::SyncActionNode
{
public:
  IsArmCameraAlive(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void on_result(
    const yoloe_interfaces::msg::ObservationResult::SharedPtr msg);

  std::shared_ptr<rclcpp::Subscription<
      yoloe_interfaces::msg::ObservationResult>> sub_;
  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point last_arrival_{};
  bool seen_{false};
  int max_age_ms_{500};
};

}  // namespace lgw_bt_planner
