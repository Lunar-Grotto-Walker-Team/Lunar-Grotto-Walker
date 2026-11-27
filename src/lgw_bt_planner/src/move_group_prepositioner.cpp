// src/move_group_prepositioner.cpp — see the header for the contract.

#include "lgw_bt_planner/move_group_prepositioner.hpp"

#include <atomic>
#include <mutex>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

namespace lgw_bt_planner
{

moveit_msgs::action::MoveGroup::Goal make_joint_goal(
  const std::array<double, 6> & joints,
  const std::array<std::string, 6> & names,
  double velocity_scaling,
  double accel_scaling,
  double tolerance_rad,
  const std::string & group_name)
{
  moveit_msgs::action::MoveGroup::Goal goal;
  goal.request.group_name = group_name;
  goal.request.num_planning_attempts = 5;
  goal.request.allowed_planning_time = 2.0;
  // Written verbatim. 0 = "unspecified", which makes MoveIt fall back to
  // joint_limits.yaml's default_*_scaling_factor (0.1). Do NOT replace
  // 0 with a "sensible" number here — the contract is that 0 means "use the
  // configured default".
  goal.request.max_velocity_scaling_factor = velocity_scaling;
  goal.request.max_acceleration_scaling_factor = accel_scaling;

  moveit_msgs::msg::Constraints constraints;
  for (std::size_t i = 0; i < 6; ++i) {
    moveit_msgs::msg::JointConstraint joint;
    joint.joint_name = names[i];
    joint.position = joints[i];
    joint.tolerance_above = tolerance_rad;
    joint.tolerance_below = tolerance_rad;
    joint.weight = 1.0;
    constraints.joint_constraints.push_back(joint);
  }
  goal.request.goal_constraints.push_back(constraints);
  return goal;
}

struct MoveGroupPrepositioner::Impl
{
  std::string action_name;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp_action::Client<MoveGroup>::SharedPtr client;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::atomic_bool cancel_requested{false};
  rclcpp::Logger logger{rclcpp::get_logger("move_group_prepositioner")};
};

MoveGroupPrepositioner::MoveGroupPrepositioner(
  rclcpp::Node & node,
  const std::string & action_name,
  const std::string & group_name)
: impl_(std::make_unique<Impl>())
{
  (void)group_name;   // every goal carries it through make_joint_goal()
  impl_->action_name = action_name;
  // Dedicated group, NOT added to the node's main executor: the caller blocks
  // on move_to(), and the control timer must keep ticking on the other thread.
  impl_->callback_group = node.create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  impl_->client = rclcpp_action::create_client<MoveGroup>(
    &node, action_name, impl_->callback_group);
  impl_->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor->add_callback_group(
    impl_->callback_group, node.get_node_base_interface());
}

MoveGroupPrepositioner::~MoveGroupPrepositioner()
{
  if (impl_ && impl_->executor && impl_->callback_group) {
    impl_->executor->remove_callback_group(impl_->callback_group);
  }
}

void MoveGroupPrepositioner::cancel()
{
  RCLCPP_WARN(
    impl_->logger, "cancel requested for action '%s'",
    impl_->action_name.c_str());
  impl_->cancel_requested.store(true);
}

bool MoveGroupPrepositioner::move_to(
  const std::array<double, 6> & joints,
  const std::array<std::string, 6> & names,
  double velocity_scaling,
  double accel_scaling,
  double tolerance_rad,
  std::chrono::milliseconds timeout,
  std::string & error)
{
  if (!impl_->client->wait_for_action_server(timeout)) {
    error = "move_group action server '" + impl_->action_name + "' unavailable";
    return false;
  }

  // Do NOT clear cancel_requested here: a release that lands between the
  // caller's check and this call must still abort the goal. The flag is
  // cleared on exit by the guard below.
  struct CancelReset
  {
    std::atomic_bool & flag;
    ~CancelReset() {flag.store(false);}
  } cancel_reset{impl_->cancel_requested};
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto goal = make_joint_goal(
    joints, names, velocity_scaling, accel_scaling, tolerance_rad);

  auto goal_future = impl_->client->async_send_goal(goal);
  while (goal_future.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() > deadline) {
      error = "timed out waiting for move_group to accept the goal";
      return false;
    }
    impl_->executor->spin_some();
  }

  auto handle = goal_future.get();
  if (!handle) {
    error = "move_group rejected the goal";
    return false;
  }
  RCLCPP_INFO(impl_->logger, "goal accepted (action '%s')", impl_->action_name.c_str());

  auto result_future = impl_->client->async_get_result(handle);
  while (result_future.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready)
  {
    if (impl_->cancel_requested.exchange(false)) {
      RCLCPP_WARN(
        impl_->logger, "sending cancel_goal (action '%s')",
        impl_->action_name.c_str());
      impl_->client->async_cancel_goal(handle);
    }
    if (std::chrono::steady_clock::now() > deadline) {
      impl_->client->async_cancel_goal(handle);
      error = "timed out waiting for move_group to finish";
      return false;
    }
    impl_->executor->spin_some();
  }

  const auto wrapped = result_future.get();
  RCLCPP_INFO(
    impl_->logger, "goal finished with code %d (action '%s')",
    static_cast<int>(wrapped.code), impl_->action_name.c_str());
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error = (wrapped.code == rclcpp_action::ResultCode::CANCELED) ?
      "move_group goal canceled" : "move_group goal aborted";
    return false;
  }
  if (wrapped.result && wrapped.result->error_code.val != 1) {
    error = "move_group returned error code " +
      std::to_string(wrapped.result->error_code.val);
    return false;
  }
  return true;
}

}  // namespace lgw_bt_planner
