// Diagnostic probe — see docs/servo/2026-11-02-servo-state-monitor-stuck.md §6.
//
// Builds an independent moveit::core::RobotModel + CurrentStateMonitor (the
// exact class moveit_servo uses to seed its state) and points it at the same
// joint_states topic Servo is configured with. It also keeps its own plain
// rclcpp subscription to that topic and counts callbacks.
//
// The counters separate the two candidate failure paths from the doc:
//   our_msgs == 0                        -> nothing reaches the node at all
//   our_msgs > 0, monitor_cb == 0        -> monitor's own subscription is
//                                           not being fed (topic/QoS/start)
//   monitor_cb > 0, monitor q == zeros   -> samples are applied but the group
//                                           lookup yields nothing (path B)
//   monitor q tracks the mock            -> problem is inside Servo's instance
//
// Throwaway tool: no unit tests, no safety logic, never runs the arm.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

namespace
{
constexpr int kDefaultPrintPeriodMs = 1000;
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("servo_state_probe");

  const std::string joint_states_topic =
      node->declare_parameter<std::string>("joint_states_topic", "/control/joint_states");
  const std::string group_name =
      node->declare_parameter<std::string>("joint_group", "arm");
  // rclcpp auto-declares use_sim_time when the parameter is supplied, so it
  // cannot be declared here (ParameterAlreadyDeclaredException otherwise).
  bool use_sim_time = false;
  node->get_parameter_or("use_sim_time", use_sim_time, false);

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // 1. Robot model — same parameter name (and therefore same URDF/SRDF) the
  //    servo_node gets from the launch file. Kinematics plugins are not
  //    needed: we only resolve the joint group.
  auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      node, "robot_description", /*load_kinematics_solvers=*/false);
  const auto model = loader->getModel();
  if (!model)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not load robot model from parameter 'robot_description'.");
    rclcpp::shutdown();
    return 1;
  }

  const auto* group = model->getJointModelGroup(group_name);
  if (!group)
  {
    RCLCPP_ERROR(node->get_logger(), "Robot model has no joint group '%s'. Groups:",
                 group_name.c_str());
    for (const auto& name : model->getJointModelGroupNames())
      RCLCPP_ERROR(node->get_logger(), "  - %s", name.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "model='%s' group='%s' joints=%zu",
              model->getName().c_str(), group_name.c_str(),
              group->getVariableNames().size());

  // 2. TF: CurrentStateMonitor wants a Buffer. The mock/real stack publishes
  //    static + dynamic TF; a listener keeps the buffer fresh if anything
  //    else asks for it.
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  // 3. The object under test.
  auto monitor = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(
      node, model, tf_buffer, use_sim_time);
  monitor->startStateMonitor(joint_states_topic);

  auto monitor_updates = std::make_shared<std::size_t>(0);
  monitor->addUpdateCallback(
      [monitor_updates, node](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        ++(*monitor_updates);
        if (*monitor_updates == 1)
        {
          RCLCPP_INFO(node->get_logger(),
                      "monitor update callback fired for the first time "
                      "(stamp=%.3f, %zu names)",
                      rclcpp::Time(msg->header.stamp).seconds(), msg->name.size());
        }
      });

  // 4. Independent plain subscription on the same topic: proves that samples
  //    reach this node and gives us the ground-truth values to compare the
  //    monitor's copy against.
  auto our_msgs = std::make_shared<std::size_t>(0);
  auto raw_q = std::make_shared<std::vector<double>>(group->getVariableCount(), 0.0);
  auto raw_names = std::make_shared<std::vector<std::string>>();
  auto raw_stamp = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::QoS(10),
      [our_msgs, raw_q, raw_names, raw_stamp, group](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        ++(*our_msgs);
        *raw_stamp = rclcpp::Time(msg->header.stamp);
        *raw_names = msg->name;
        for (std::size_t i = 0; i < group->getVariableCount(); ++i)
        {
          const auto& joint_name = group->getVariableNames()[i];
          for (std::size_t j = 0; j < msg->name.size(); ++j)
          {
            if (msg->name[j] == joint_name && j < msg->position.size())
            {
              (*raw_q)[i] = msg->position[j];
              break;
            }
          }
        }
      });

  // Print the resolved topic/activity once, so a name mismatch is obvious.
  RCLCPP_INFO(node->get_logger(),
              "requested topic='%s'  monitor topic='%s'  active=%d  complete=%d",
              joint_states_topic.c_str(), monitor->getMonitoredTopic().c_str(),
              static_cast<int>(monitor->isActive()),
              static_cast<int>(monitor->haveCompleteState()));

  const auto start = clock->now();
  auto range_min = std::make_shared<std::vector<double>>(group->getVariableCount(), 1e9);
  auto range_max = std::make_shared<std::vector<double>>(group->getVariableCount(), -1e9);

  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(
          node->declare_parameter<int>("print_period_ms", kDefaultPrintPeriodMs)),
      [node, monitor, group, our_msgs, monitor_updates, raw_q, raw_names, raw_stamp, start,
       range_min, range_max]() {
        auto state = monitor->getCurrentState();
        std::vector<double> q(group->getVariableCount(), 0.0);
        if (state)
          state->copyJointGroupPositions(group, q);

        for (std::size_t i = 0; i < q.size(); ++i)
        {
          range_min->at(i) = std::min(range_min->at(i), q[i]);
          range_max->at(i) = std::max(range_max->at(i), q[i]);
        }

        const double age = (raw_stamp->nanoseconds() == 0)
                               ? -1.0
                               : (node->now() - *raw_stamp).seconds();
        const double monitor_age = (monitor->getCurrentStateTime().nanoseconds() == 0)
                                       ? -1.0
                                       : (node->now() - monitor->getCurrentStateTime()).seconds();

        std::string raw, mon, diff;
        for (std::size_t i = 0; i < q.size(); ++i)
        {
          char buf[96];
          snprintf(buf, sizeof(buf), " %.3f", (*raw_q)[i]);
          raw += buf;
          snprintf(buf, sizeof(buf), " %.3f", q[i]);
          mon += buf;
          snprintf(buf, sizeof(buf), " %+.3f", q[i] - (*raw_q)[i]);
          diff += buf;
        }

        RCLCPP_INFO(node->get_logger(),
                    "t=%5.1fs our_msgs=%zu monitor_cb=%zu complete=%d "
                    "raw_age=%.2fs monitor_age=%.2fs",
                    (node->now() - start).seconds(), *our_msgs, *monitor_updates,
                    static_cast<int>(monitor->haveCompleteState()), age, monitor_age);
        RCLCPP_INFO(node->get_logger(), "  raw    =[%s ]", raw.c_str());
        RCLCPP_INFO(node->get_logger(), "  monitor=[%s ]", mon.c_str());
        RCLCPP_INFO(node->get_logger(), "  diff   =[%s ]", diff.c_str());

        std::string span;
        for (std::size_t i = 0; i < q.size(); ++i)
        {
          char buf[96];
          snprintf(buf, sizeof(buf), " %.3f", range_max->at(i) - range_min->at(i));
          span += buf;
        }
        RCLCPP_INFO(node->get_logger(), "  monitor_range_since_start=[%s ]", span.c_str());
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
