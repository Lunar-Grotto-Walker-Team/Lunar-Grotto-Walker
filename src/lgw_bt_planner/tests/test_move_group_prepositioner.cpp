// tests/test_move_group_prepositioner.cpp
//
// The fake /move_action server runs IN THIS PROCESS: rules out external
// orchestration, and lets each failure mode (reject / abort / never responds /
// cancel) be produced deterministically — none of which a real MoveIt instance
// can do on demand.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "lgw_bt_planner/move_group_prepositioner.hpp"

using lgw_bt_planner::make_joint_goal;
using lgw_bt_planner::MoveGroupPrepositioner;
using MoveGroup = moveit_msgs::action::MoveGroup;

namespace
{

const std::array<std::string, 6> kNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const std::array<double, 6> kTarget{0.1, 0.9, -0.48, -1.1, 0.6, 1.3};

enum class Mode { kOk, kAbort, kReject, kHang };

class FakeMoveGroupServer
{
public:
  explicit FakeMoveGroupServer(Mode mode)
  : mode_(mode)
  {
    static std::atomic<int> seq{0};
    const std::string tag = std::to_string(seq++);
    action_name_ = "/move_action_" + tag;
    node_ = std::make_shared<rclcpp::Node>("fake_move_group_server_" + tag);
    using std::placeholders::_1;
    using std::placeholders::_2;
    server_ = rclcpp_action::create_server<MoveGroup>(
      node_, action_name_,
      std::bind(&FakeMoveGroupServer::handle_goal, this, _1, _2),
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&FakeMoveGroupServer::handle_accepted, this, _1));
    thread_ = std::thread([this]() {
        executor_.add_node(node_);
        executor_.spin();
      });
  }

  const std::string & action_name() const {return action_name_;}
  double last_velocity_scaling() const {return last_velocity_.load();}
  double last_accel_scaling() const {return last_accel_.load();}

  ~FakeMoveGroupServer()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveGroup::Goal>)
  {
    return mode_ == Mode::kReject ?
           rclcpp_action::GoalResponse::REJECT :
           rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> goal)
  {
    if (const auto received = goal->get_goal()) {
      last_velocity_.store(received->request.max_velocity_scaling_factor);
      last_accel_.store(received->request.max_acceleration_scaling_factor);
    }
    if (mode_ == Mode::kHang) {
      // Keep the handle alive. If it were dropped without being resolved,
      // rclcpp_action would terminate the goal itself and the client would see
      // CANCELED instead of a timeout — which would hide the very behaviour
      // this test is here to pin down.
      hanging_ = goal;
      return;
    }
    if (mode_ == Mode::kAbort) {
      goal->abort(std::make_shared<MoveGroup::Result>());
      return;
    }
    auto result = std::make_shared<MoveGroup::Result>();
    result->error_code.val = 1;
    goal->succeed(result);
  }

private:
  Mode mode_;
  std::atomic<double> last_velocity_{-1.0};
  std::atomic<double> last_accel_{-1.0};
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> hanging_;
  std::string action_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MoveGroup>::SharedPtr server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

}  // namespace

TEST(PrepositionGoal, JointNamesAndPositionsMatch)
{
  const auto goal = make_joint_goal(kTarget, kNames, 0.3, 0.3, 0.02);
  ASSERT_EQ(goal.request.goal_constraints.size(), 1U);
  const auto & jc = goal.request.goal_constraints.front().joint_constraints;
  ASSERT_EQ(jc.size(), 6U);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(jc[i].joint_name, kNames[i]);
    EXPECT_DOUBLE_EQ(jc[i].position, kTarget[i]);
  }
}

TEST(PrepositionGoal, ZeroScalingIsWrittenAsZero)
{
  // 0 must reach MoveIt unchanged: it means "use joint_limits.yaml's default".
  // Substituting anything here silently changes how fast the arm travels.
  const auto goal = make_joint_goal(kTarget, kNames, 0.0, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(goal.request.max_velocity_scaling_factor, 0.0);
  EXPECT_DOUBLE_EQ(goal.request.max_acceleration_scaling_factor, 0.0);
}

TEST(PrepositionGoal, AccelerationScalingIsIndependentFromVelocity)
{
  // Time-optimal parameterisation is acceleration-limited for short moves, so
  // the two scalings must be settable separately. Tying them together was the
  // reason a 0.46 rad recovery move took 1.9 s.
  const auto goal = make_joint_goal(kTarget, kNames, 0.3, 0.5, 0.02);
  EXPECT_DOUBLE_EQ(goal.request.max_velocity_scaling_factor, 0.3);
  EXPECT_DOUBLE_EQ(goal.request.max_acceleration_scaling_factor, 0.5);
}

TEST(PrepositionGoal, ZeroAccelerationScalingIsWrittenAsZero)
{
  const auto goal = make_joint_goal(kTarget, kNames, 0.3, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(goal.request.max_acceleration_scaling_factor, 0.0);
}

TEST(PrepositionGoal, ToleranceAppliedToEveryJoint)
{
  const auto goal = make_joint_goal(kTarget, kNames, 0.3, 0.3, 0.07);
  for (const auto & jc : goal.request.goal_constraints.front().joint_constraints) {
    EXPECT_DOUBLE_EQ(jc.tolerance_above, 0.07);
    EXPECT_DOUBLE_EQ(jc.tolerance_below, 0.07);
  }
}

TEST(PrepositionGoal, GroupIsArm)
{
  EXPECT_EQ(
    make_joint_goal(kTarget, kNames, 0.3, 0.3, 0.02, "arm").request.group_name, "arm");
}

class PrepositionerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(PrepositionerTest, SucceedsWhenServerAccepts)
{
  FakeMoveGroupServer server(Mode::kOk);
  auto node = std::make_shared<rclcpp::Node>("prepositioner_client_ok");
  MoveGroupPrepositioner preparer(*node, server.action_name(), "arm");

  std::string error;
  EXPECT_TRUE(preparer.move_to(
      kTarget, kNames, 0.3, 0.5, 0.02, std::chrono::milliseconds(3000), error)) << error;
}

TEST_F(PrepositionerTest, ForwardsAccelerationScalingToTheGoal)
{
  FakeMoveGroupServer server(Mode::kOk);
  auto node = std::make_shared<rclcpp::Node>("prepositioner_client_accel");
  MoveGroupPrepositioner preparer(*node, server.action_name(), "arm");

  std::string error;
  ASSERT_TRUE(preparer.move_to(
      kTarget, kNames, 0.3, 0.5, 0.02, std::chrono::milliseconds(3000), error)) << error;
  EXPECT_DOUBLE_EQ(server.last_velocity_scaling(), 0.3);
  EXPECT_DOUBLE_EQ(server.last_accel_scaling(), 0.5);
}

TEST_F(PrepositionerTest, ReturnsFalseWhenServerRejects)
{
  FakeMoveGroupServer server(Mode::kReject);
  auto node = std::make_shared<rclcpp::Node>("prepositioner_client_reject");
  MoveGroupPrepositioner preparer(*node, server.action_name(), "arm");

  std::string error;
  EXPECT_FALSE(preparer.move_to(
      kTarget, kNames, 0.3, 0.5, 0.02, std::chrono::milliseconds(3000), error));
  EXPECT_NE(error.find("rejected"), std::string::npos) << error;
}

TEST_F(PrepositionerTest, ReturnsFalseWhenServerAborts)
{
  FakeMoveGroupServer server(Mode::kAbort);
  auto node = std::make_shared<rclcpp::Node>("prepositioner_client_abort");
  MoveGroupPrepositioner preparer(*node, server.action_name(), "arm");

  std::string error;
  EXPECT_FALSE(preparer.move_to(
      kTarget, kNames, 0.3, 0.5, 0.02, std::chrono::milliseconds(3000), error));
  EXPECT_NE(error.find("aborted"), std::string::npos) << error;
}

TEST_F(PrepositionerTest, TimesOutWhenServerNeverAnswers)
{
  FakeMoveGroupServer server(Mode::kHang);
  auto node = std::make_shared<rclcpp::Node>("prepositioner_client_hang");
  MoveGroupPrepositioner preparer(*node, server.action_name(), "arm");

  std::string error;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(preparer.move_to(
      kTarget, kNames, 0.3, 0.5, 0.02, std::chrono::milliseconds(400), error));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  // It must give up on its own budget, not hang.
  EXPECT_LT(elapsed, std::chrono::milliseconds(2000));
  EXPECT_NE(error.find("timed out"), std::string::npos) << error;
}
