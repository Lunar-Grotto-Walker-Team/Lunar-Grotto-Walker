// stand_action_nodes.cpp — 见头文件说明。
//
// QoS 选择(BEST_EFFORT)沿用 go2_bt_ws 真机跑通的 rclcpp::SensorDataQoS()。
// 真机 /api/sport/request publisher 是 RELIABLE,但 BARE DDS 的订阅方
// 不暴露 QoS,go2_bt_ws 用 BEST_EFFORT 真机闭环已验证,沿用最稳。
// 真机 /lf/sportmodestate 7 个订阅者全是 BEST_EFFORT,我们的订阅必须
// 用 BEST_EFFORT 才能加入它们的 DDS 网络。

#include "lgw_bt_planner/stand_action_nodes.hpp"

#include <stdexcept>

namespace lgw_bt_planner
{

namespace
{

// 只设 header.identity.api_id,parameter 留空(官方 SDK2 的用法:
// StandUp / StandDown / BalanceStand / StopMove 都不要参数)。
unitree_api::msg::Request make_request(int32_t api_id)
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = api_id;
  return req;
}

}  // namespace

SportTopicNodeBase::SportTopicNodeBase(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  // RosNodeParams.nh 是 weak_ptr;BT.ROS2 v0.3.0 保证 bt_runner 在调节点
  // 构造时 node 还活着(参考 bt_plugin_registration.cpp 的 lock 注释)。
  auto nh_shared = params.nh.lock();
  if (!nh_shared) {
    throw std::runtime_error(
      "SportTopicNodeBase: RosNodeParams.nh weak_ptr expired; "
      "the rclcpp::Node passed to BT::RegisterRosNode must outlive the call");
  }
  node_ = nh_shared;

  // 与 go2_bt_ws SportTopicNodeBase 一致:eager init 两个端点,避免
  // lazy-init 的 check-then-act race。3 个实例 × 2 = 6 个端点,代价可忽略。
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>(
    "/api/sport/request", rclcpp::SensorDataQoS());

  state_sub_ = node_->create_subscription<unitree_go::msg::SportModeState>(
    "/lf/sportmodestate", rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      // 订阅回调在 executor 线程,tick 在 BT 线程,必须加锁。
      std::lock_guard<std::mutex> lk(state_mutex_);
      latest_state_ = *msg;
      has_state_ = true;
    });
}

void SportTopicNodeBase::publish_api(int32_t api_id)
{
  // request_pub_ 在构造时已就绪,这里不再 null check(基类不变量)。
  request_pub_->publish(make_request(api_id));
}

bool SportTopicNodeBase::copy_latest_state(
  unitree_go::msg::SportModeState & out) const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  if (!has_state_) {
    return false;
  }
  out = latest_state_;
  return true;
}

// ----------------------- StandUpNode -----------------------

BT::NodeStatus StandUpNode::onStart()
{
  publish_api(kApiIdStandUp);
  start_time_ = node_->now();
  ge_count_ = 0;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StandUpNode::onRunning()
{
  unitree_go::msg::SportModeState s;
  if (copy_latest_state(s)) {
    if (s.body_height >= kBodyHeightStanding) {
      if (++ge_count_ >= kStableFrames) {
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      ge_count_ = 0;
    }
  }
  if ((node_->now() - start_time_) > kPostureTimeoutSec) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void StandUpNode::onHalted()
{
  // 与 go2_bt_ws 一致:被打断就发 StopMove 防狗继续起身动作。
  publish_api(kApiIdStopMove);
}

// ----------------------- BalanceStandNode -----------------------

BT::NodeStatus BalanceStandNode::onStart()
{
  publish_api(kApiIdBalanceStand);
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BalanceStandNode::onRunning()
{
  unitree_go::msg::SportModeState s;
  if (copy_latest_state(s) && s.mode == 1) {
    return BT::NodeStatus::SUCCESS;
  }
  if ((node_->now() - start_time_) > kBalanceStandTimeoutSec) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void BalanceStandNode::onHalted()
{
  // BalanceStand 没有"撤销"动作;留空以免触发 StopMove 把 mode 打回 0
  // (go2_bt_ws 同位置)。
}

// ----------------------- StandDownNode -----------------------

BT::NodeStatus StandDownNode::onStart()
{
  publish_api(kApiIdStandDown);
  start_time_ = node_->now();
  lt_count_ = 0;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StandDownNode::onRunning()
{
  unitree_go::msg::SportModeState s;
  if (copy_latest_state(s)) {
    if (s.body_height < kBodyHeightLying) {
      if (++lt_count_ >= kStableFrames) {
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      lt_count_ = 0;
    }
  }
  if ((node_->now() - start_time_) > kPostureTimeoutSec) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void StandDownNode::onHalted()
{
  publish_api(kApiIdStopMove);
}

}  // namespace lgw_bt_planner
