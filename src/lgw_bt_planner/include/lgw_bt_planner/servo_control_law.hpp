// include/lgw_bt_planner/servo_control_law.hpp
//
// Pure 100 Hz proportional control law for the MoveIt Servo takeover stack
// (spec 2026-10-26 §7.6). Turns the desired-vs-current `tcp_link` pose error
// into the TwistStamped velocity command that the Servo node (Task 7)
// publishes to /servo_node/delta_twist_cmds.
//
// REFERENCE POINT: `tcp_link`. Both poses are tcp_link poses expressed in
// the arm base_link frame. The Servo Jacobian is evaluated at the SRDF `arm`
// group tip (which is tcp_link here) and compute_tcp_pose() already emits
// the tcp pose, so there is NO link6 conversion and NO 12 cm offset in this
// module (spec §7.6, decision D18). Do not add one.
//
//   e_pos = desired.p - current.p
//   R_err = desired.R * current.R^T      (rotation that takes current to desired)
//   e_ang = axis_angle(R_err)            (rotation vector, radians)
//
//   v = clamp(kp_pos * e_pos, v_max) * source_scale
//   w = clamp(kp_ang * e_ang, w_max) * source_scale
//
// `clamp` scales the whole vector down to the given maximum magnitude.
// `source_scale` is the per-tier gain from the geometry-source module
// (head 1.0 / arm 0.7 / odom 0.5) and multiplies the CLAMPED output, per the
// formula above and spec §7.4/§7.6.
//
// The scaled command then goes through a first-order low-pass whose state
// lives in ControlLawState and persists across ticks:
//
//   alpha = 1 - exp(-2*pi*lowpass_cutoff_hz / loop_rate_hz)
//   filtered += alpha * (raw - filtered)
//
// (defaults: 8 Hz at 100 Hz -> alpha ~= 0.395). The first call starts from
// zero. The low-pass suppresses the high-frequency jitter that Go2 gait
// motion injects around zero error (spec §7.6, risk R4).
//
// DEADBAND (spec §7.6, decision D7): when BOTH ||e_ang|| <
// deadband_ang_rad (2 deg) and ||e_pos|| < deadband_pos_m (1 cm), the tick
// returns exactly zero twist with `in_deadband = true`. The filter state is
// deliberately left untouched — the output is zeroed, not the memory — so a
// target drifting back out of the deadband resumes from the previous
// filtered value instead of snapping from zero.
//
// NaN safety: a non-finite rotation (or translation) is treated as zero
// error and can never reach the output; the identity / near-identity
// rotation yields a zero angular error, and the near-pi case is handled
// without dividing by a vanishing sine.
//
// Pure C++17 + tf2 math only: no rclcpp, no ROS runtime, no BT, no Eigen.

#ifndef LGW_BT_PLANNER__SERVO_CONTROL_LAW_HPP_
#define LGW_BT_PLANNER__SERVO_CONTROL_LAW_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Tuning for compute_twist. Defaults are the spec §7.6 values.
struct ControlLawParams
{
  double kp_pos{2.0};
  double kp_ang{3.0};
  double v_max{0.15};                 // m/s
  double w_max{0.6};                  // rad/s
  double deadband_pos_m{0.01};        // 1 cm
  double deadband_ang_rad{0.034906585};  // 2 deg
  double lowpass_cutoff_hz{8.0};
  double loop_rate_hz{100.0};
  // Hard ceiling for the SUM of the proportional term and the feed-forward.
  // v_max/w_max still bound the proportional term exactly as before; this
  // only stops a runaway reference velocity from reaching the actuators.
  double v_max_total{1.2};            // m/s
  double w_max_total{1.6};            // rad/s
  // Measured-velocity damping (2026-11-06): the loop closes on the REAL arm
  // pose, which lags the command by ~0.3 s. The P-only law then overshoots
  // (~13 % measured) and rings. kd has units of seconds: the command loses
  // kd * v_measured, so a term matched to the loop delay (~0.3 s) supplies
  // the missing phase margin. 0 disables it (legacy behaviour).
  double kd_pos{0.30};                // s
  double kd_ang{0.30};                // s
};

// Low-pass memory carried across ticks. Zero on first use; never reset by
// the deadband path.
struct ControlLawState
{
  tf2::Vector3 v_filtered{0, 0, 0};
  tf2::Vector3 w_filtered{0, 0, 0};
};

// One tick's command. Linear/angular velocities in the arm base_link frame;
// `in_deadband` is true when the output was zeroed by the deadband rule.
struct TwistCommand
{
  // tf2::Vector3's default constructor leaves the components uninitialised,
  // so a default-built TwistCommand (e.g. the `twist` field of a refused
  // TrackingOutput) used to hold garbage. Zero is the honest default: "no
  // command", and it makes reading it on a non-publish path well defined.
  tf2::Vector3 v{0, 0, 0};
  tf2::Vector3 w{0, 0, 0};
  bool in_deadband{false};
};

// Compute the velocity command for one 100 Hz tick.
//
// `desired` and `current` are tcp_link poses in the arm base_link frame.
// `source_scale` is the geometry-source tier gain (1.0 / 0.7 / 0.5).
// `state` carries and is updated with the low-pass memory.
// `v_ff` / `w_ff` are the REFERENCE velocity: the derivative of `desired`
// (tcp velocity that keeps the setpoint satisfied). They are added AFTER the
// proportional clamp and the low-pass, then the sum is capped by
// v_max_total/w_max_total. Two behaviours are deliberate:
//   - inside the deadband the output is the feed-forward, NOT zero (a
//     world-fixed target while the platform walks keeps the setpoint moving
//     even when the error is below the deadband; zeroing it would stop the
//     arm and re-open the error);
//   - the feed-forward bypasses the proportional low-pass: the caller filters
//     its own derivative (a differentiator needs its own cutoff).
TwistCommand compute_twist(
  const Pose6& desired,
  const Pose6& current,
  const ControlLawParams& params,
  ControlLawState& state,
  double source_scale,
  const tf2::Vector3& v_ff = tf2::Vector3(0, 0, 0),
  const tf2::Vector3& w_ff = tf2::Vector3(0, 0, 0),
  const tf2::Vector3& v_meas = tf2::Vector3(0, 0, 0),
  const tf2::Vector3& w_meas = tf2::Vector3(0, 0, 0));

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO_CONTROL_LAW_HPP_
