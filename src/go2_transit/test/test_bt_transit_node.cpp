// test_bt_transit_node.cpp — TransitToLocalPoint 的节点级测试。
//
// 直接构造 BT 节点并手动 tick，不需要整棵行为树；用进程内的假 transit 动作服务器
// 控制"接受但不回结果"，专门钉住"等结果必须有超时"这条。
//
// 注意：不调用 onStart 之外的 BT 机制，所以这里校验的是节点自身的状态机。

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "go2_transit/action/transit_to_pose.hpp"
#include "go2_transit/transit_bt_nodes.hpp"

using namespace std::chrono_literals;

namespace
{

void ensure_ros_initialized()
{
  static bool initialized = false;
  if (!initialized) {
    rclcpp::init(0, nullptr);
    initialized = true;
  }
}

struct RosShutdownGuard
{
  ~RosShutdownGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};
RosShutdownGuard g_shutdown_guard;

// 接受 goal 但永不返回结果，模拟 transit_node 在"已接受、未出结果"之间卡住。
class HungTransitServer : public rclcpp::Node
{
public:
  using TransitToPose = go2_transit::action::TransitToPose;

  HungTransitServer()
  : Node("hung_transit_server")
  {
    server_ = rclcpp_action::create_server<TransitToPose>(
      this, "/go2_transit/transit_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const TransitToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TransitToPose>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TransitToPose>> handle) {
        // 必须存住句柄，否则 goal 会被当成已结束。
        std::lock_guard<std::mutex> lock(mutex_);
        handles_.push_back(handle);
      });
  }

private:
  rclcpp_action::Server<TransitToPose>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<TransitToPose>>> handles_;
};

class TfPublisher : public rclcpp::Node
{
public:
  TfPublisher()
  : Node("bt_test_tf"), broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    timer_ = create_wall_timer(20ms, [this]() {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = "odom";
      t.child_frame_id = "base_footprint";
      t.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(t);
    });
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// 构造一个带 goal 端口的 BT 节点配置。
BT::NodeConfig make_config(const geometry_msgs::msg::PoseStamped & goal)
{
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("goal_local", goal);
  config.blackboard->set("reason_out", std::string());
  // 端口映射必须写成 {key} 才是黑板引用；写裸 key 会被当成字面量去 convertFromString。
  config.input_ports["goal"] = "{goal_local}";
  config.output_ports["result_reason"] = "{reason_out}";
  // 标量端口直接给字面量即可（会被 convertFromString 解析成 double）。
  config.input_ports["ready_wait_s"] = "2.0";
  config.input_ports["result_timeout_s"] = "0.5";
  return config;
}

geometry_msgs::msg::PoseStamped make_local_goal(double x, double y)
{
  geometry_msgs::msg::PoseStamped g;
  g.header.frame_id = "local";
  g.pose.position.x = x;
  g.pose.position.y = y;
  g.pose.orientation.w = 1.0;
  return g;
}

}  // namespace

// transit 动作服务器接受后一直不回结果：节点必须在 result_timeout_s 内放弃，
// 不能永远 RUNNING（那样整棵树卡死，操作员只能手动取消）。
TEST(BtTransitNode, ResultWaitHasTimeout)
{
  ensure_ros_initialized();

  auto server_node = std::make_shared<HungTransitServer>();
  auto tf_node = std::make_shared<TfPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(tf_node);

  auto config = make_config(make_local_goal(1.0, 0.0));
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  std::string reason;
  {
    BT::RosNodeParams params;
    params.nh = server_node;
    go2_transit::TransitToLocalPoint node("go_there", config, params);

    // 上限：ready_wait_s=2 + result_timeout_s=0.5，再留余量。
    // 修好之后应当在 ~1 s 内返回 FAILURE；没修则一直 RUNNING，判定失败。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    status = node.onStart();
    while (status == BT::NodeStatus::RUNNING &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some(20ms);
      status = node.onRunning();
    }
    node.onHalted();
    (void)config.blackboard->get("reason_out", reason);
  }

  EXPECT_EQ(status, BT::NodeStatus::FAILURE)
    << "结果等待没有超时：节点在 transit 服务器挂掉时会永远 RUNNING";
  EXPECT_EQ(reason, "timeout");
}

// frame_id 不是 local：必须立刻 FAILURE，绝不当作 odom 绝对坐标。
TEST(BtTransitNode, RejectsNonLocalFrame)
{
  ensure_ros_initialized();

  auto node_handle = rclcpp::Node::make_shared("bt_test_frame");
  auto goal = make_local_goal(1.0, 0.0);
  goal.header.frame_id = "odom";
  auto config = make_config(goal);

  BT::RosNodeParams params;
  params.nh = node_handle;
  go2_transit::TransitToLocalPoint node("go_there", config, params);

  EXPECT_EQ(node.onStart(), BT::NodeStatus::FAILURE);
}
