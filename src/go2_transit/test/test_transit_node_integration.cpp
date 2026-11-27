// test_transit_node_integration.cpp — 节点层回归测试。
//
// 用一个进程内的假 nav2（FollowPath / Spin 动作服务器 + TF 广播）驱动
// TransitNode，钉住三类只在接线层才会出现的问题：
//   1. 被抢占任务的旧 result 不能回流到新任务（epoch 跨 goal 复用）；
//   2. follow_path_timeout_s 必须真的生效，而不是静默挂到总超时；
//   3. planning_frame != behavior_server global_frame 时，Spin 的目标 yaw
//      必须换算到 Spin 自己的坐标系。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "go2_transit/action/transit_to_pose.hpp"
#include "go2_transit/transit_node.hpp"

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

// 假 nav2：FollowPath 默认永远不返回（模拟 DWB 卡死），可记录收到的目标。
class FakeNav2 : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using Spin = nav2_msgs::action::Spin;

  FakeNav2()
  : Node("fake_nav2")
  {
    follow_path_server_ = rclcpp_action::create_server<FollowPath>(
      this, "/follow_path",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> handle) {
        // 不能在 handle_cancel 里直接调 canceled()：rclcpp 要等本回调返回后
        // 才把 goal 置为 CANCELING，提前调用会抛 "invalid transition"。
        std::lock_guard<std::mutex> lock(mutex_);
        follow_path_cancel_count_++;
        fp_to_cancel_.push_back(handle);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        follow_path_goal_count_++;
        // 必须把句柄存住：句柄一析构，这个 goal 就不再是"在跑"，对端会立刻
        // 拿到结果，假服务器就挂不住了。
        fp_handles_.push_back(handle);
      });

    spin_server_ = rclcpp_action::create_server<Spin>(
      this, "/spin",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Spin::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Spin>> handle) {
        auto result = std::make_shared<Spin::Result>();
        handle->canceled(result);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Spin>> handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        spin_target_yaw_ = handle->get_goal()->target_yaw;
        spin_goal_count_++;
        auto result = std::make_shared<Spin::Result>();
        handle->succeed(result);
      });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(20ms, std::bind(&FakeNav2::publish_tf, this));
  }

  void set_pose(double x, double y, double yaw)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_x_ = x;
    robot_y_ = y;
    robot_yaw_ = yaw;
  }

  // map -> odom 的 yaw。非 0 时用来验证 Spin 的坐标换算。
  void set_map_to_odom_yaw(double yaw)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_yaw_ = yaw;
    publish_map_odom_ = true;
  }

  int follow_path_goal_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return follow_path_goal_count_;
  }

  int follow_path_cancel_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return follow_path_cancel_count_;
  }

  int spin_goal_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return spin_goal_count_;
  }

  double spin_target_yaw()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return spin_target_yaw_;
  }

private:
  void publish_tf()
  {
    // 真的把取消结果发出去：只有进入 CANCELING 之后调 canceled() 才合法。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto it = fp_to_cancel_.begin(); it != fp_to_cancel_.end(); ) {
        if ((*it)->is_canceling()) {
          (*it)->canceled(std::make_shared<FollowPath::Result>());
          it = fp_to_cancel_.erase(it);
        } else {
          ++it;
        }
      }
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double map_yaw = 0.0;
    bool publish_map = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      x = robot_x_;
      y = robot_y_;
      yaw = robot_yaw_;
      map_yaw = map_to_odom_yaw_;
      publish_map = publish_map_odom_;
    }

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_footprint";
    odom_to_base.transform.translation.x = x;
    odom_to_base.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom_to_base.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(odom_to_base);

    if (publish_map) {
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = now();
      map_to_odom.header.frame_id = "map";
      map_to_odom.child_frame_id = "odom";
      tf2::Quaternion q2;
      q2.setRPY(0.0, 0.0, map_yaw);
      map_to_odom.transform.rotation = tf2::toMsg(q2);
      tf_broadcaster_->sendTransform(map_to_odom);
    }
  }

  rclcpp_action::Server<FollowPath>::SharedPtr follow_path_server_;
  rclcpp_action::Server<Spin>::SharedPtr spin_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::mutex mutex_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double map_to_odom_yaw_{0.0};
  bool publish_map_odom_{false};
  int follow_path_goal_count_{0};
  int follow_path_cancel_count_{0};
  int spin_goal_count_{0};
  double spin_target_yaw_{0.0};
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>>> fp_handles_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>>> fp_to_cancel_;
};

class TransitNodeIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ensure_ros_initialized();
    fake_ = std::make_shared<FakeNav2>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(fake_);
  }

  void TearDown() override
  {
    if (transit_) {
      executor_->remove_node(transit_);
      transit_.reset();
    }
    executor_->remove_node(fake_);
    fake_.reset();
    executor_.reset();
  }

  void make_transit_node(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    transit_ = std::make_shared<go2_transit::TransitNode>(options);
    executor_->add_node(transit_);
    client_node_ = rclcpp::Node::make_shared("transit_test_client");
    executor_->add_node(client_node_);
    client_ = rclcpp_action::create_client<go2_transit::action::TransitToPose>(
      client_node_, "/go2_transit/transit_to_pose");
  }

  template<typename Predicate>
  bool spin_until(Predicate predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(10ms);
      if (predicate()) {
        return true;
      }
    }
    return predicate();
  }

  go2_transit::action::TransitToPose::Goal make_goal(double x, double y)
  {
    go2_transit::action::TransitToPose::Goal goal;
    goal.target.header.frame_id = "odom";
    goal.target.pose.position.x = x;
    goal.target.pose.position.y = y;
    goal.target.pose.orientation.w = 1.0;
    return goal;
  }

  std::shared_ptr<FakeNav2> fake_;
  std::shared_ptr<go2_transit::TransitNode> transit_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<go2_transit::action::TransitToPose>::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

}  // namespace

// 被抢占任务的 CANCELED 结果不能回流到新任务的状态机。
//
// 旧实现在 start_*/finish_active_goal 里把 dispatch_epoch_ 归零，于是
// "旧任务第 1 次派发" 与 "新任务第 1 次派发" 拿到同一个 epoch，旧结果被
// 当成新任务的失败输入。这里 max_retries=0，一旦误吃就直接 unreachable。
TEST_F(TransitNodeIntegration, PreemptedGoalDoesNotFeedOldResultIntoNewGoal)
{
  make_transit_node({
    {"ready_wait_s", 5.0},
    {"pre_align_threshold_rad", 0.0},
    {"max_retries", 0},
    {"total_timeout_s", 30.0},
    {"follow_path_timeout_s", 30.0},
    {"path_spacing_m", 1.0},
  });
  fake_->set_pose(0.0, 0.0, 0.0);

  ASSERT_TRUE(spin_until([this] { return client_->action_server_is_ready(); }, 5000ms));

  auto goal_a = make_goal(5.0, 0.0);
  auto future_a_handle = client_->async_send_goal(goal_a);
  ASSERT_TRUE(
    spin_until(
      [&] {return future_a_handle.wait_for(0s) == std::future_status::ready;}, 5000ms));
  auto handle_a = future_a_handle.get();
  ASSERT_NE(handle_a, nullptr);
  auto future_a_result = client_->async_get_result(handle_a);

  // 等 A 真正发出 FollowPath
  ASSERT_TRUE(spin_until([this] { return fake_->follow_path_goal_count() >= 1; }, 5000ms));

  // B 抢占 A
  auto goal_b = make_goal(6.0, 0.0);
  auto future_b_handle = client_->async_send_goal(goal_b);
  ASSERT_TRUE(
    spin_until(
      [&] {return future_b_handle.wait_for(0s) == std::future_status::ready;}, 5000ms));
  auto handle_b = future_b_handle.get();
  ASSERT_NE(handle_b, nullptr);
  auto future_b_result = client_->async_get_result(handle_b);

  // A 的取消要真的送到假服务器
  ASSERT_TRUE(spin_until([this] { return fake_->follow_path_cancel_count() >= 1; }, 5000ms));

  // 给旧的 CANCELED 结果足够的回流时间：若 epoch 复用，B 会在这段窗口里被判失败
  const bool b_finished_early = spin_until(
    [&] {return future_b_result.wait_for(0s) == std::future_status::ready;}, 1500ms);

  EXPECT_FALSE(b_finished_early)
    << "新任务在被抢占任务的取消结果回流后被误判结束（epoch 复用）";

  // 被抢占的 A 自身应当以大原因为取消的 ABORTED 收场
  ASSERT_TRUE(
    spin_until(
      [&] {return future_a_result.wait_for(0s) == std::future_status::ready;}, 3000ms));
  EXPECT_EQ(future_a_result.get().code, rclcpp_action::ResultCode::ABORTED);
}

// follow_path_timeout_s 必须真的生效：DWB 卡住时要有界重试，而不是挂到总超时。
TEST_F(TransitNodeIntegration, FollowPathTimeoutIsEnforced)
{
  make_transit_node({
    {"ready_wait_s", 5.0},
    {"pre_align_threshold_rad", 0.0},
    {"max_retries", 1},
    {"total_timeout_s", 60.0},
    {"follow_path_timeout_s", 0.3},
    {"path_spacing_m", 1.0},
  });
  fake_->set_pose(0.0, 0.0, 0.0);

  ASSERT_TRUE(spin_until([this] { return client_->action_server_is_ready(); }, 5000ms));

  auto goal = make_goal(5.0, 0.0);
  auto future_handle = client_->async_send_goal(goal);
  ASSERT_TRUE(
    spin_until(
      [&] {return future_handle.wait_for(0s) == std::future_status::ready;}, 5000ms));
  auto handle = future_handle.get();
  ASSERT_NE(handle, nullptr);
  auto future_result = client_->async_get_result(handle);

  const bool finished = spin_until(
    [&] {return future_result.wait_for(0s) == std::future_status::ready;}, 8000ms);

  ASSERT_TRUE(finished) << "FollowPath 超时从未生效，任务一直挂着";
  const auto wrapped = future_result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  if (wrapped.result) {
    EXPECT_EQ(wrapped.result->reason, "unreachable");
  }
  // max_retries=1 → 第一次 + 一次重试 = 2 次派发
  EXPECT_GE(fake_->follow_path_goal_count(), 2);
}

// 退出时取消在飞动作，绝不能再派发新的。
//
// 真机踩过：cancel_all_on_shutdown 里那 600 ms 的 spin 把 CANCELED 结果喂回
// 状态机，状态机按重试逻辑又发了一条 FollowPath —— 进程已经要退了，狗却收到
// 一条新指令继续走。
TEST_F(TransitNodeIntegration, ShutdownCancelNeverRedispatches)
{
  make_transit_node({
    {"ready_wait_s", 5.0},
    {"pre_align_threshold_rad", 0.0},
    {"max_retries", 2},
    {"total_timeout_s", 60.0},
    {"follow_path_timeout_s", 30.0},
    {"path_spacing_m", 1.0},
  });
  fake_->set_pose(0.0, 0.0, 0.0);

  ASSERT_TRUE(spin_until([this] { return client_->action_server_is_ready(); }, 5000ms));

  auto future_handle = client_->async_send_goal(make_goal(5.0, 0.0));
  ASSERT_TRUE(
    spin_until(
      [&] {return future_handle.wait_for(0s) == std::future_status::ready;}, 5000ms));
  ASSERT_NE(future_handle.get(), nullptr);
  ASSERT_TRUE(spin_until([this] { return fake_->follow_path_goal_count() >= 1; }, 5000ms));

  const int before = fake_->follow_path_goal_count();
  transit_->cancel_all_on_shutdown(*executor_);

  // 取消之后再多转一会儿，给任何迟到的"重试"足够时间冒出来
  spin_until([] { return false; }, 500ms);

  EXPECT_EQ(fake_->follow_path_goal_count(), before)
    << "退出取消之后状态机又派发了一条新的 FollowPath";
}

// nav2 的 Spin 把 target_yaw 当**相对转角**（spin.cpp: cmd_yaw_ = target_yaw,
// remaining_yaw = |cmd_yaw_| - |relative_yaw_|）。所以下发前必须把
// "绝对目标朝向"换算成"相对当前朝向的增量"，否则会多转一大截。
TEST_F(TransitNodeIntegration, SpinTargetYawIsSentAsRelativeDelta)
{
  make_transit_node({
    {"ready_wait_s", 5.0},
    {"pre_align_threshold_rad", 0.0},
    {"max_retries", 0},
    {"total_timeout_s", 30.0},
    {"follow_path_timeout_s", 30.0},
  });
  fake_->set_pose(0.0, 0.0, 0.5);  // 当前朝向 0.5 rad

  ASSERT_TRUE(spin_until([this] { return client_->action_server_is_ready(); }, 5000ms));

  auto rotate_client = rclcpp_action::create_client<go2_transit::action::RotateToYaw>(
    client_node_, "/go2_transit/rotate_to_yaw");
  ASSERT_TRUE(spin_until([&] { return rotate_client->action_server_is_ready(); }, 5000ms));

  go2_transit::action::RotateToYaw::Goal goal;
  goal.target_yaw = 1.0F;  // 绝对目标 1.0 rad → 应当只转 +0.5
  auto future_handle = rotate_client->async_send_goal(goal);
  ASSERT_TRUE(
    spin_until(
      [&] {return future_handle.wait_for(0s) == std::future_status::ready;}, 5000ms));
  auto handle = future_handle.get();
  ASSERT_NE(handle, nullptr);
  auto future_result = rotate_client->async_get_result(handle);

  ASSERT_TRUE(spin_until([this] { return fake_->spin_goal_count() >= 1; }, 5000ms));
  EXPECT_NEAR(fake_->spin_target_yaw(), 0.5, 0.05)
    << "Spin 下发的是绝对角而不是相对增量，nav2 会多转一大截";

  spin_until([&] {return future_result.wait_for(0s) == std::future_status::ready;}, 5000ms);
}
