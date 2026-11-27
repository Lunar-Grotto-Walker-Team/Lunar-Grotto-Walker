// include/agx_topic_hardware/agx_topic_system.hpp
//
// ros2_control SystemInterface for the agx arm: the STATE comes from the real
// driver's feedback topic, the COMMAND is published to the topic the driver
// consumes. Replacing mock_components/GenericSystem (whose "state" is just the
// last command) with this plugin is what makes MoveIt/Servo see reality:
//
//   /feedback/joint_states --(read)--> state interfaces
//   command interfaces     --(write)--> /control/joint_states --> agx driver
//
// read()/write() run in the controller_manager's 200 Hz loop and only copy
// buffers; the DDS receive/send lives in a separate executor thread.

#ifndef AGX_TOPIC_HARDWARE__AGX_TOPIC_SYSTEM_HPP_
#define AGX_TOPIC_HARDWARE__AGX_TOPIC_SYSTEM_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace agx_topic_hardware
{

class AgxTopicSystem : public hardware_interface::SystemInterface
{
public:
  ~AgxTopicSystem() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
  override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
  override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void stop_spinner();

  std::vector<std::string> joint_names_;
  std::vector<double> hw_state_pos_;
  std::vector<double> hw_state_vel_;
  std::vector<double> hw_cmd_pos_;

  // Latest message, mapped by joint name. Written by the subscription thread,
  // read by read() under try_lock.
  std::mutex state_mutex_;
  std::vector<double> rx_pos_;
  std::vector<double> rx_vel_;
  bool rx_valid_{false};

  // Command bookkeeping. Until the first real feedback arrives the command
  // buffer is meaningless (zeros), and publishing it would make the driver
  // execute move_j([0 ...]) -- i.e. drive the arm to the home pose at startup.
  // So: seed the command from the first feedback, and only publish afterwards.
  bool cmd_seeded_{false};
  std::vector<double> last_sent_pos_;
  std::chrono::steady_clock::time_point last_publish_{};

  std::string state_topic_{"/feedback/joint_states"};
  std::string command_topic_{"/control/joint_states"};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

}  // namespace agx_topic_hardware

#endif  // AGX_TOPIC_HARDWARE__AGX_TOPIC_SYSTEM_HPP_
