// src/no_op_smoothing.cpp
//
// Pass-through MoveIt Servo smoothing plugin.
//
// WHY THIS EXISTS (diagnostic, 2026-10-30)
// MoveIt Servo runs every outgoing command through a smoothing plugin
// (servo_calcs.cpp: applyJointUpdate() -> smoother_->doSmoothing(position)).
// It smooths the ABSOLUTE joint position command, which is re-seeded from the
// joint_topic every cycle, so the filter's own history feeds back into its
// input. Measured with the default Butterworth (coeff 1.5, then 1.01): every
// joint advanced only ~0.2x of the commanded twist increment, uniformly and
// independently of the twist magnitude or the motion profile (step and
// sinusoidal sweep both), and reproducing even with a twist injected directly
// into Servo (our own node idle). Singularity scaling, collision scaling,
// velocity limits, the IK-vs-Jacobian choice and the calc rate (100 Hz) were
// all ruled out first.
//
// MoveIt ships only one smoothing implementation, so there is no parameter to
// turn it off. This plugin is the "off" switch: doSmoothing() returns the
// command untouched. It is intended for diagnosing the attenuation above and,
// if confirmed, as the production setting — a servo loop fed by a 100 Hz
// velocity stream does not want an extra low-pass inside the position
// integrator.
//
// Select it with:
//   servo_params.yaml: moveit_servo.smoothing_filter_plugin_name:
//                        "lgw_servo_smoothing::NoOpSmoothing"
// and switch back to "online_signal_smoothing::ButterworthFilterPlugin" to
// restore MoveIt's default behaviour.

#include <moveit/online_signal_smoothing/smoothing_base_class.h>
// NOTE: do not include moveit/robot_model/robot_model.h here. It drags in
// srdfdom/model.h, which is not on the include path this package gets from
// moveit_core. The base class only needs the forward-declared
// RobotModelConstPtr, and a shared_ptr copy does not require the complete type.
#include <pluginlib/class_list_macros.hpp>

#include <cstddef>
#include <vector>

namespace lgw_servo_smoothing
{

class NoOpSmoothing : public online_signal_smoothing::SmoothingBaseClass
{
public:
  NoOpSmoothing() = default;
  ~NoOpSmoothing() override = default;

  bool initialize(
    rclcpp::Node::SharedPtr /*node*/,
    moveit::core::RobotModelConstPtr /*robot_model*/,
    size_t /*num_joints*/) override
  {
    return true;
  }

  // The whole point: leave the position command exactly as Servo computed it.
  bool doSmoothing(std::vector<double>& /*position_vector*/) override
  {
    return true;
  }

  // Nothing to reset: there is no filter state.
  bool reset(const std::vector<double>& /*joint_positions*/) override
  {
    return true;
  }
};

}  // namespace lgw_servo_smoothing

PLUGINLIB_EXPORT_CLASS(
  lgw_servo_smoothing::NoOpSmoothing,
  online_signal_smoothing::SmoothingBaseClass)
