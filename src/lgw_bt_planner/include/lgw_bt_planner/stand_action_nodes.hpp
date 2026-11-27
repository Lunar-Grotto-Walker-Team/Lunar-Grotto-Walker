// stand_action_nodes.hpp — Go2 起身/趴下/BalanceStand BT 叶节点。
//
// 模式:BT::StatefulActionNode 子类(BT.ROS2 v0.3.0),rclcpp::Node 由
// BT::RosNodeParams 注入(与 move_arm_to_*_action_node.hpp 一致)。
//
// 参考实现:go2_bt_ws/src/go2_bt_planner/src/bt_runner_node.cpp L55-275
// (StandUp / BalanceStand / StandDown / StopMove TopicNode)。
// 我们按 lgw_bt_planner 的分文件风格重写,不跨 ws include go2_bt_planner。
//
// 完成判据(沿用 go2_bt_ws 真机实测值):
//   StandUp     : body_height >= 0.30 连续 5 帧 → SUCCESS;超时 5.0s → FAILURE
//   StandDown   : body_height <  0.20 连续 5 帧 → SUCCESS;超时 5.0s → FAILURE
//   BalanceStand: state.mode == 1               → SUCCESS;超时 2.0s → FAILURE
//   response.code==0 是 ack,不代表动作完成 — 详见 docs/go2_sport_api.md。

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>

namespace lgw_bt_planner
{

// Unitree Go2 sport API ID,数值与官方 SDK2 ros2_sport_client.h 一致。
// 与 go2_bt_planner/sport_api_ids.hpp 数值相同;lgw 不依赖该头文件(跨 ws)，
// 在此独立定义一份。
inline constexpr int32_t kApiIdBalanceStand = 1002;
inline constexpr int32_t kApiIdStopMove     = 1003;
inline constexpr int32_t kApiIdStandUp      = 1004;
inline constexpr int32_t kApiIdStandDown    = 1005;

// body_height 完成阈值(米)。与 go2_bt_ws 真机标定一致:趴 ≈ 0.10，站 ≈ 0.315。
inline constexpr double kBodyHeightStanding = 0.30;
inline constexpr double kBodyHeightLying    = 0.20;
inline constexpr int    kStableFrames       = 5;
inline constexpr std::chrono::seconds kPostureTimeoutSec{5};
inline constexpr std::chrono::seconds kBalanceStandTimeoutSec{2};

// 基类:持有 publisher + subscriber + 最新 state 缓存。
// 派生类实现 onStart/onRunning/onHalted 的语义。
class SportTopicNodeBase : public BT::StatefulActionNode
{
public:
  SportTopicNodeBase(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

protected:
  // 发布一个 api_id 命令(只设 header.identity.api_id,parameter 留空,
  // 与 go2_bt_ws make_request 一致)。
  void publish_api(int32_t api_id);

  // 把缓存的最新 state 拷到 out。返回 false 表示还没收到任何 state。
  bool copy_latest_state(unitree_go::msg::SportModeState & out) const;

  // 共享的 rclcpp::Node(由 bt_runner 通过 RosNodeParams 注入)。
  rclcpp::Node::SharedPtr node_;

private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;

  mutable std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  bool has_state_ = false;
};

// 起身:发 api_id=1004,等 body_height>=0.30 稳定 5 帧,超时 5s。
class StandUpNode : public SportTopicNodeBase
{
public:
  using SportTopicNodeBase::SportTopicNodeBase;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // BT.CPP v4 static_assert:带 RosNodeParams 参数构造的节点必须实现
  // providedPorts()。本节点无端口(阈值/超时用 inline constexpr 写死),
  // 返回空列表。
  static BT::PortsList providedPorts() { return {}; }

private:
  rclcpp::Time start_time_;
  int ge_count_ = 0;
};

// BalanceStand:发 api_id=1002,等 state.mode==1,超时 2s。
// 起身后必调:等价于按遥控 Start 键,否则狗不响应 Move。
class BalanceStandNode : public SportTopicNodeBase
{
public:
  using SportTopicNodeBase::SportTopicNodeBase;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  rclcpp::Time start_time_;
};

// 趴下:发 api_id=1005,等 body_height<0.20 稳定 5 帧,超时 5s。
class StandDownNode : public SportTopicNodeBase
{
public:
  using SportTopicNodeBase::SportTopicNodeBase;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  rclcpp::Time start_time_;
  int lt_count_ = 0;
};

}  // namespace lgw_bt_planner
