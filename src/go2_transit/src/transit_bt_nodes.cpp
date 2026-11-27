// transit_bt_nodes.cpp — TransitToLocalPoint 实现 + 插件注册。

#include "go2_transit/transit_bt_nodes.hpp"

#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/plugins.hpp>

#include "go2_transit/transit_core.hpp"

namespace go2_transit
{

TransitToLocalPoint::TransitToLocalPoint(
  const std::string & name, const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  // params.nh 是 weak_ptr；bt_runner_node 活得比插件久，lock 一次即可。
  node_ = params.nh.lock();
  if (!node_) {
    throw BT::RuntimeError(
            "TransitToLocalPoint: RosNodeParams.nh 已失效，"
            "注册时必须传 bt_runner 的 Node");
  }

  // 自建 TF：bt_runner 虽然往全局黑板塞了 tf_buffer，但现有 BT 节点没人用过那个端口，
  // 类型匹配没有先例。自己开一个 TransformListener 便宜且确定。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  client_ = rclcpp_action::create_client<TransitToPose>(node_, "/go2_transit/transit_to_pose");
}

BT::PortsList TransitToLocalPoint::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "局部目标位姿，frame_id 必须是 'local'"),
    BT::InputPort<std::string>("planning_frame", "odom", "规划坐标系"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "机器人基座坐标系"),
    BT::InputPort<double>("xy_tolerance", 0.0, "0 = 用 transit 节点默认值"),
    BT::InputPort<double>("yaw_tolerance", 0.0, "0 = 用 transit 节点默认值"),
    BT::InputPort<double>("ready_wait_s", 10.0, "等 TF / 动作服务器的上限"),
    BT::InputPort<double>(
      "result_timeout_s", 420.0,
      "等 transit 出结果的上限（应大于 transit 自身的 total_timeout_s=360）"),
    // 2026-11-20: 巡逻腿用。halt(BT 打断)之后保留局部原点, 重新进入时
    // 仍然打同一个终点 —— 被采集打断的腿会"接着走完剩下的路程", 而不是
    // 从当前位置再走一遍全长。默认 false 保持一次性摆渡的旧语义。
    BT::InputPort<bool>(
      "keep_origin", false,
      "halt 后保留局部原点(巡逻腿恢复用); false = 每次重新定位"),
    BT::OutputPort<std::string>("result_reason", "结果字符串，便于日志排查"),
  };
}

double TransitToLocalPoint::elapsed_s() const
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - started_at_).count();
}

BT::NodeStatus TransitToLocalPoint::fail_with(const std::string & reason)
{
  setOutput("result_reason", reason);
  return BT::NodeStatus::FAILURE;
}

bool TransitToLocalPoint::ready()
{
  if (!client_ || !client_->action_server_is_ready()) {
    return false;
  }
  return tf_buffer_->canTransform(planning_frame_, base_frame_, tf2::TimePointZero);
}

bool TransitToLocalPoint::capture_origin()
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      planning_frame_, base_frame_, tf2::TimePointZero);
    origin_.x = tf.transform.translation.x;
    origin_.y = tf.transform.translation.y;
    origin_.yaw = tf2::getYaw(tf.transform.rotation);
    origin_captured_ = true;
    RCLCPP_INFO(
      logger_, "捕获局部原点: x=%.3f y=%.3f yaw=%.1f°（%s 系），局部目标 (%.2f, %.2f)",
      origin_.x, origin_.y, origin_.yaw * 180.0 / M_PI, planning_frame_.c_str(),
      goal_local_.pose.position.x, goal_local_.pose.position.y);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "捕获局部原点失败: %s", ex.what());
    return false;
  }
}

void TransitToLocalPoint::send_goal()
{
  const double lx = goal_local_.pose.position.x;
  const double ly = goal_local_.pose.position.y;
  const Pose2D target = local_to_planning(origin_, lx, ly);

  TransitToPose::Goal goal;
  goal.target.header.stamp = node_->now();
  goal.target.header.frame_id = planning_frame_;
  goal.target.pose.position.x = target.x;
  goal.target.pose.position.y = target.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, origin_.yaw);
  goal.target.pose.orientation = tf2::toMsg(q);
  goal.align_yaw = true;
  goal.xy_tolerance = static_cast<float>(xy_tolerance_);
  goal.yaw_tolerance = static_cast<float>(yaw_tolerance_);

  RCLCPP_INFO(
    logger_, "下发 TransitToPose: 局部(%.2f, %.2f) → %s 系 (%.3f, %.3f)，对齐 yaw=%.1f°",
    lx, ly, planning_frame_.c_str(), target.x, target.y, origin_.yaw * 180.0 / M_PI);

  goal_response_ = client_->async_send_goal(goal);
  goal_sent_at_ = std::chrono::steady_clock::now();
  phase_ = Phase::WaitResult;
}

BT::NodeStatus TransitToLocalPoint::onStart()
{
  if (!getInput("goal", goal_local_)) {
    RCLCPP_ERROR(logger_, "缺少必需端口 [goal]");
    return BT::NodeStatus::FAILURE;
  }
  if (!is_local_frame(goal_local_.header.frame_id)) {
    RCLCPP_ERROR(
      logger_, "goal 的 frame_id 必须是 'local'，实际是 '%s'（不会当 odom 绝对坐标用）",
      goal_local_.header.frame_id.c_str());
    return BT::NodeStatus::FAILURE;
  }
  getInput("planning_frame", planning_frame_);
  getInput("base_frame", base_frame_);
  getInput("xy_tolerance", xy_tolerance_);
  getInput("yaw_tolerance", yaw_tolerance_);
  getInput("ready_wait_s", ready_wait_s_);
  getInput("result_timeout_s", result_timeout_s_);
  getInput("keep_origin", keep_origin_);

  phase_ = Phase::WaitReady;
  goal_handle_.reset();
  started_at_ = std::chrono::steady_clock::now();

  return onRunning();
}

BT::NodeStatus TransitToLocalPoint::onRunning()
{
  if (phase_ == Phase::WaitReady) {
    // 就绪门与原点捕获共用同一个 deadline：任何一条分支都不允许无限 RUNNING。
    const bool timed_out = elapsed_s() > ready_wait_s_;

    if (!ready()) {
      // 就绪前绝不下发 goal，也绝不拿 {0,0,0} 兜底。
      if (timed_out) {
        RCLCPP_ERROR(
          logger_, "%.1f s 内 TF/动作服务器未就绪，放弃本次尝试", ready_wait_s_);
        return fail_with("not_ready");
      }
      return BT::NodeStatus::RUNNING;
    }

    // 原点只捕获一次：三次重试打的是同一个点。
    if (!origin_captured_ && !capture_origin()) {
      if (timed_out) {
        RCLCPP_ERROR(logger_, "%.1f s 内拿不到位姿，放弃本次尝试", ready_wait_s_);
        return fail_with("not_ready");
      }
      return BT::NodeStatus::RUNNING;
    }

    send_goal();
    return BT::NodeStatus::RUNNING;
  }

  // WaitResult：goal 响应和最终结果都要有上限，否则 transit 进程退出后整棵树卡死。
  {
    const double since_sent = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - goal_sent_at_).count();
    if (since_sent > result_timeout_s_) {
      RCLCPP_ERROR(
        logger_, "%.1f s 内没有收到结果，放弃本次尝试（transit 可能已退出）",
        result_timeout_s_);
      if (goal_handle_) {
        client_->async_cancel_goal(goal_handle_);
      } else {
        client_->async_cancel_all_goals();
      }
      return fail_with("timeout");
    }
  }

  if (!goal_handle_) {
    if (goal_response_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_response_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(logger_, "TransitToPose 目标被拒绝");
      setOutput("result_reason", std::string("rejected"));
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = client_->async_get_result(goal_handle_);
    return BT::NodeStatus::RUNNING;
  }

  if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto wrapped = result_future_.get();
  const bool ok = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
  std::string reason = ok ? "ok" : "aborted";
  if (wrapped.result) {
    reason = wrapped.result->reason;
  }
  setOutput("result_reason", reason);

  RCLCPP_INFO(
    logger_, "TransitToPose 结束: code=%d reason=%s",
    static_cast<int>(wrapped.code), reason.c_str());
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void TransitToLocalPoint::onHalted()
{
  // 无条件取消所有 goal：goal_handle_ 要等响应回来才有值，若 halt 恰好落在
  // "请求已发出、响应未到"的窗口里，只按句柄取消会漏掉，狗会继续走。
  // 这个 client 只服务本节点，cancel_all 是安全的。
  if (client_) {
    client_->async_cancel_all_goals();
  }
  goal_handle_.reset();
  result_future_ = {};
  goal_response_ = {};
  // 树被取消后重新执行时要重新定位：原点作废。
  // 例外: keep_origin=true 的巡逻腿 —— 打断它的是"途中采集", 采集完要
  // 接着走到同一个终点, 所以原点必须保留; 重新进入时 goal 还是同一个
  // local 向量, Nav2 会从当前位置直接开到那个绝对点(剩余路程)。
  if (!keep_origin_) {
    origin_captured_ = false;
  }
  phase_ = Phase::WaitReady;
  RCLCPP_WARN(
    logger_, "被 halt：已取消在飞动作；局部原点 %s",
    keep_origin_ ? "保留(keep_origin=true)" : "作废");
}

}  // namespace go2_transit

BT_REGISTER_ROS_NODES(factory, params)
{
  auto nh_shared = params.nh.lock();
  if (!nh_shared) {
    throw std::runtime_error(
            "go2_transit BT plugin: RosNodeParams.nh 已失效");
  }
  factory.registerNodeType<go2_transit::TransitToLocalPoint>(
    "TransitToLocalPoint", BT::RosNodeParams(nh_shared));
}
