// include/lgw_bt_planner/base_pose_nodes.hpp
//
// 基地圆心的两个薄壳(2026-11-18):
//
//   RecordBasePose      启动时把"基地圆心"记进 blackboard(odom 系 Point)。
//                       与 MarkTrack(action=6) 配合: 后者让 tracker 用它划
//                       建轨屏蔽圈, 这里让行为树自己也能判断"回没回到基地"。
//   IsWithinBaseRadius  用当前 arm base 的 odom 位置与圆心比, XoY 距离
//                       <= radius_m 就 SUCCESS(条件式, 永不返回 RUNNING)。
//
// 两个节点都只做 TF 查询 + 黑板读写, 不含任何控制逻辑。
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace lgw_bt_planner
{

class RecordBasePose : public BT::SyncActionNode
{
public:
  RecordBasePose(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.record_base_pose")};
};

class IsWithinBaseRadius : public BT::SyncActionNode
{
public:
  IsWithinBaseRadius(
    const std::string & name, const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  rclcpp::Logger logger_{
    rclcpp::get_logger("lgw_bt_planner.is_within_base_radius")};
};

}  // namespace lgw_bt_planner
