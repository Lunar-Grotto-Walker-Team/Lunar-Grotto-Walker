// bt_runner_node.cpp — minimal TreeExecutionServer subclass.
//
// The base class (vendored at go2_bt_ws/BehaviorTree.ROS2/
// behaviortree_ros2) handles:
//   - plugin .so discovery from the `plugins` ROS param
//   - behavior tree XML discovery from the `behavior_trees` ROS param
//   - action server hosting (`<action_name>` action, default /bt_execution)
//   - tree ticking at `tick_frequency` Hz on its own thread
//   - Groot2 publisher auto-attached
//   - parameter hot-reload (re-runs executeRegistration() when params change)
//
// This subclass only does what the base class can't: load the SRDF
// named-targets table at startup (one-shot, before any goal is served) and
// inject two shared_ptrs into the GLOBAL blackboard each time a tree is
// created (so plugin-loaded BT nodes can read them via
// getInput<T>(...) using the @{key} port-remap idiom from BT.CPP t16).
//
// Constructor uses the second TreeExecutionServer ctor that takes an
// already-created rclcpp::Node::SharedPtr so we can pin the node name to
// "bt_runner" — the cleanup greps in scripts/start_*.sh match on that.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_ros2/tree_execution_server.hpp>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "lgw_bt_planner/named_target_loader.hpp"

#include <chrono>
#include <memory>

namespace
{
// Blackboard keys that plugin-loaded BT nodes read from. Mirrors the
// constants in bt_plugin_registration.cpp and the .cpp files of nodes
// that need them (move_arm_to_named_action_node, detach_object_service_node).
constexpr const char* kBlackboardKeyNamedTargets = "named_targets";
constexpr const char* kBlackboardKeyTfBuffer = "tf_buffer";
}  // namespace

class BtRunner : public BT::TreeExecutionServer
{
public:
  explicit BtRunner(const rclcpp::NodeOptions& options)
    : BT::TreeExecutionServer(std::make_shared<rclcpp::Node>("bt_runner", options)),
      // Loaded in the body: the xacro branch arguments are ROS parameters, and
      // parameters can only be declared once the node exists.
      named_targets_(std::make_shared<const lgw_bt_planner::TargetTable>()),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->node()->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this->node()))
  {
    // 2026-11-17: the SRDF is a per-arm-type xacro template. These three
    // arguments select the branch, mirroring the arguments move_group gets at
    // launch. Without them every branch's group_states are merged and same-name
    // states clobber each other (nero's 7-joint `home` beat piper's 6-joint one).
    const auto arm_type =
      this->node()->declare_parameter<std::string>("srdf_arm_type", "piper");
    const auto effector_type =
      this->node()->declare_parameter<std::string>("srdf_effector_type", "agx_gripper");
    const auto revo2_type =
      this->node()->declare_parameter<std::string>("srdf_revo2_type", "left");

    named_targets_ = std::make_shared<const lgw_bt_planner::TargetTable>(
      lgw_bt_planner::NamedTargetLoader::load_from_share(
        arm_type, effector_type, revo2_type));

    RCLCPP_INFO(
      this->node()->get_logger(),
      "Loaded %zu named targets from SRDF (arm_type=%s, effector_type=%s, revo2_type=%s)",
      named_targets_->size(), arm_type.c_str(), effector_type.c_str(), revo2_type.c_str());
  }

  // Plugin + tree discovery are done by the base class via executeRegistration().
  // We don't need to override registerNodesIntoFactory() because every BT
  // leaf node in this package is registered by the .so plugin symbol.
  //
  // We DO need to override onTreeCreated() to push shared_ptrs into the
  // GLOBAL blackboard each time a tree is instantiated.
  //
  // Idiomatic BT.CPP t16 "global blackboard" pattern: globalBlackboard() is
  // the parent of every tree's root blackboard (TreeExecutionServer wires
  // that up in tree_execution_server.cpp:193). BT nodes access these keys
  // via the @{key} port-remap syntax in XML, which causes
  // Blackboard::getAnyPort() to traverse to the parent and find the value.
  // Example: <MoveArmToNamed named_targets="@{named_targets}" .../>
  //
  // Type note: BT.CPP's Blackboard stores the registered port type as
  // typeid(T) and cast<T>() requires an exact match. InputPort<T> in v4
  // normalizes the type via std::decay, which strips the inner const from
  // std::shared_ptr<const Foo>. We therefore set the global entry as
  // std::shared_ptr<TargetTable> (no const), matching the port declaration
  // in MoveArmToNamedActionNode::providedPorts(). The shared_ptr's
  // ownership — not the BB type — keeps the SRDF table from being mutated
  // by nodes.
  void onTreeCreated(BT::Tree& tree) override
  {
    auto bb = globalBlackboard();
    bb->set<std::shared_ptr<lgw_bt_planner::TargetTable>>(
      kBlackboardKeyNamedTargets,
      std::const_pointer_cast<lgw_bt_planner::TargetTable>(named_targets_));
    bb->set<std::shared_ptr<tf2_ros::Buffer>>(
      kBlackboardKeyTfBuffer, tf_buffer_);

    // Drop any logger attached to the previous tree (each goal creates a
    // fresh tree; the logger holds a const Tree& reference). Mirrors the
    // pattern used for Groot2Publisher in
    // tree_execution_server.cpp around line 203.
    stdout_logger_.reset();
    stdout_logger_ = std::make_shared<BT::StdCoutLogger>(tree);
  }

private:
  std::shared_ptr<const lgw_bt_planner::TargetTable> named_targets_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<BT::StdCoutLogger> stdout_logger_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<BtRunner>(rclcpp::NodeOptions());

  // MultiThreadedExecutor with the documented 0-worker-thread + 250ms
  // spin-some workaround for the rclcpp MultiThreadedExecutor deadlock
  // bug (see btcpp_ros2_samples/src/sample_bt_executor.cpp:68-71 in the
  // vendored BT.ROS2). The tree ticking itself runs on TreeExecutionServer's
  // own thread, so the executor only needs to service ROS callbacks.
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(server->node());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}