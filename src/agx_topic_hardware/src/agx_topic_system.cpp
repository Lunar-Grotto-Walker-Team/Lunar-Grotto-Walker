// src/agx_topic_system.cpp

#include "agx_topic_hardware/agx_topic_system.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace agx_topic_hardware
{

AgxTopicSystem::~AgxTopicSystem()
{
  stop_spinner();
}

void AgxTopicSystem::stop_spinner()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();
  state_sub_.reset();
  command_pub_.reset();
  node_.reset();
}

hardware_interface::CallbackReturn AgxTopicSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto param = [&info](const std::string & key,
      const std::string & fallback) {
      const auto it = info.hardware_parameters.find(key);
      return it == info.hardware_parameters.end() ? fallback : it->second;
    };
  state_topic_ = param("state_topic", state_topic_);
  command_topic_ = param("command_topic", command_topic_);

  joint_names_.clear();
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
  }
  if (joint_names_.empty()) {
    return CallbackReturn::ERROR;
  }

  const std::size_t n = joint_names_.size();
  hw_state_pos_.assign(n, 0.0);
  hw_state_vel_.assign(n, 0.0);
  hw_cmd_pos_.assign(n, 0.0);
  rx_pos_.assign(n, 0.0);
  rx_vel_.assign(n, 0.0);

  node_ = std::make_shared<rclcpp::Node>("agx_topic_system");
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, rclcpp::SensorDataQoS().reliable(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      state_callback(msg);
    });
  command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, rclcpp::QoS(10));

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  RCLCPP_INFO(
    node_->get_logger(),
    "agx topic hardware: state <- %s, command -> %s, %zu joints",
    state_topic_.c_str(), command_topic_.c_str(), n);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AgxTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2);
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_state_pos_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_state_vel_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AgxTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_cmd_pos_[i]);
  }
  return interfaces;
}

void AgxTopicSystem::state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    const auto it = std::find(
      joint_names_.begin(), joint_names_.end(), msg->name[i]);
    if (it == joint_names_.end()) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(
      std::distance(joint_names_.begin(), it));
    if (i < msg->position.size()) {
      rx_pos_[idx] = msg->position[i];
    }
    if (i < msg->velocity.size()) {
      rx_vel_[idx] = msg->velocity[i];
    }
  }
  rx_valid_ = true;
}

hardware_interface::return_type AgxTopicSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (lock.owns_lock() && rx_valid_) {
    hw_state_pos_ = rx_pos_;
    hw_state_vel_ = rx_vel_;
    if (!cmd_seeded_) {
      // First feedback: make the very first command we emit "hold the real
      // pose". Without this the command buffer is still the zero-initialised
      // default and the driver would move the arm to its home pose.
      hw_cmd_pos_ = rx_pos_;
      last_sent_pos_ = rx_pos_;
      cmd_seeded_ = true;
      RCLCPP_INFO(node_->get_logger(), "agx topic hardware: command seeded from feedback");
    }
  }
  // No lock / no message yet: keep the previous state instead of blocking the
  // controller loop.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgxTopicSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // No feedback yet -> the command buffer is still the zero default; do not
  // publish it (the driver would execute a move to home).
  if (!cmd_seeded_) {
    return hardware_interface::return_type::OK;
  }
  for (const double q : hw_cmd_pos_) {
    if (!std::isfinite(q)) {
      // A controller has not written finite commands yet; never forward NaN.
      return hardware_interface::return_type::OK;
    }
  }

  // Publish EVERY cycle (200 Hz), like the joint_state_broadcaster did in the
  // mock setup -- no change detection, no heartbeat. Two measured reasons:
  //   * the driver turns each message into a point-to-point move_j(); a 50 Hz
  //     cadence was too sparse and the arm stuttered (2026-11-06: feedback
  //     stalled ~38 % of the time, runs up to 540 ms);
  //   * the servo node treats a /control/joint_states gap > 100 ms as a dead
  //     command stream and drops to HOLD (not_following), so a 200 ms idle
  //     heartbeat flaps the state machine while the arm is converged.
  const auto now = std::chrono::steady_clock::now();

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;
  msg.name = joint_names_;
  msg.position = hw_cmd_pos_;
  msg.velocity.assign(joint_names_.size(), 0.0);
  command_pub_->publish(msg);
  last_sent_pos_ = hw_cmd_pos_;
  last_publish_ = now;
  return hardware_interface::return_type::OK;
}

}  // namespace agx_topic_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  agx_topic_hardware::AgxTopicSystem, hardware_interface::SystemInterface)
