// src/servo_control_law.cpp
// Pure 100 Hz proportional control law for the MoveIt Servo takeover stack
// (spec 2026-10-26 §7.6). No rclcpp / ROS runtime / BT / Eigen — see
// include/lgw_bt_planner/servo_control_law.hpp for the full rule set.

#include "lgw_bt_planner/servo_control_law.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace lgw_bt_planner
{
namespace
{

constexpr double kTwoPi = 6.283185307179586476925286766559;
// Below this the quaternion vector part carries no usable direction
// (identity rotation, or accumulated round-off), so the rotation vector is
// exactly zero rather than a normalized noise direction.
constexpr double kMinSinHalfAngle = 1e-9;
// Generic "essentially zero" length: quaternion norms and vector norms
// below this are treated as degenerate.
constexpr double kMinNorm = 1e-12;

bool is_finite(const tf2::Vector3& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

// Rotation vector (axis * angle, radians) of the rotation `R_err`. Handles
// the identity / near-identity case (returns zero) and the near-pi case
// safely: the quaternion vector part is never divided by a vanishing sine,
// because |v| = sin(angle/2) ~= 1 at angle ~= pi. Non-finite input maps to
// zero so a NaN can never propagate into the command.
tf2::Vector3 rotation_vector(const tf2::Matrix3x3& R_err)
{
  tf2::Quaternion q;
  R_err.getRotation(q);

  const double norm =
    std::sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  if (!std::isfinite(norm) || norm < kMinNorm) {
    return tf2::Vector3(0, 0, 0);
  }

  double w = q.w() / norm;
  double x = q.x() / norm;
  double y = q.y() / norm;
  double z = q.z() / norm;
  // q and -q encode the same rotation; pick the hemisphere whose angle is
  // in [0, pi] so the output is the shortest rotation.
  if (w < 0.0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }

  const double sin_half = std::sqrt(x * x + y * y + z * z);
  if (sin_half < kMinSinHalfAngle) {
    return tf2::Vector3(0, 0, 0);
  }

  const double angle = 2.0 * std::atan2(sin_half, w);
  tf2::Vector3 out(x, y, z);
  out *= angle / sin_half;
  return is_finite(out) ? out : tf2::Vector3(0, 0, 0);
}

// Scale `v` down to at most `max_norm` in magnitude, preserving direction.
// A non-positive / non-finite `max_norm` yields zero; a non-finite `v`
// yields zero.
tf2::Vector3 clamp_norm(const tf2::Vector3& v, double max_norm)
{
  const double norm = v.length();
  if (!is_finite(v) || !std::isfinite(norm) || !std::isfinite(max_norm)) {
    return tf2::Vector3(0, 0, 0);
  }
  if (max_norm <= 0.0) {
    return tf2::Vector3(0, 0, 0);
  }
  if (norm <= max_norm || norm < kMinNorm) {
    return v;
  }
  return v * (max_norm / norm);
}

// First-order low-pass coefficient for one tick. A non-positive loop rate
// would divide by zero, so it degrades to "no filtering" (alpha = 1); the
// result is clamped to [0, 1] so a negative cutoff cannot invert the sign.
double lowpass_alpha(const ControlLawParams& params)
{
  if (!std::isfinite(params.loop_rate_hz) || params.loop_rate_hz <= 0.0) {
    return 1.0;
  }
  const double alpha =
    1.0 - std::exp(-kTwoPi * params.lowpass_cutoff_hz / params.loop_rate_hz);
  if (!std::isfinite(alpha)) {
    return 1.0;
  }
  return std::clamp(alpha, 0.0, 1.0);
}

}  // namespace

TwistCommand compute_twist(
  const Pose6& desired,
  const Pose6& current,
  const ControlLawParams& params,
  ControlLawState& state,
  double source_scale,
  const tf2::Vector3& v_ff,
  const tf2::Vector3& w_ff,
  const tf2::Vector3& v_meas,
  const tf2::Vector3& w_meas)
{
  // Position error is a plain frame-to-frame difference: both poses are
  // tcp_link poses in the same (arm base) frame.
  tf2::Vector3 e_pos = desired.p - current.p;
  if (!is_finite(e_pos)) {
    e_pos = tf2::Vector3(0, 0, 0);
  }

  // Angular error: the rotation that takes `current` to `desired`, expressed
  // in the common base frame as a rotation vector.
  tf2::Vector3 e_ang = rotation_vector(desired.R * current.R.transpose());
  if (!is_finite(e_ang)) {
    e_ang = tf2::Vector3(0, 0, 0);
  }

  // Strict-alignment deadband (spec §7.6): BOTH axes must be inside their
  // threshold for the tick to be deadbanded.
  const bool in_deadband =
    e_ang.length() < params.deadband_ang_rad &&
    e_pos.length() < params.deadband_pos_m;

  const tf2::Vector3 v_ff_safe = is_finite(v_ff) ? v_ff : tf2::Vector3(0, 0, 0);
  const tf2::Vector3 w_ff_safe = is_finite(w_ff) ? w_ff : tf2::Vector3(0, 0, 0);

  // Measured-velocity damping (2026-11-06). Subtracted AFTER the proportional
  // low-pass: the term exists to add phase margin against the loop delay, so
  // filtering it with the same 8 Hz pole would remove what it is there for.
  // Its share is clamped to v_max/w_max so a velocity spike cannot command the
  // arm by itself, and the sum is still bounded by v_max_total/w_max_total.
  // Not multiplied by source_scale: the measured motion is physical, not a
  // geometry-source estimate.
  const tf2::Vector3 v_damp = is_finite(v_meas) ?
    clamp_norm(v_meas * params.kd_pos, params.v_max) * -1.0 :
    tf2::Vector3(0, 0, 0);
  const tf2::Vector3 w_damp = is_finite(w_meas) ?
    clamp_norm(w_meas * params.kd_ang, params.w_max) * -1.0 :
    tf2::Vector3(0, 0, 0);

  if (in_deadband) {
    TwistCommand cmd;
    // Keep the reference velocity: it is what holds the setpoint while the
    // position error is (correctly) inside the deadband. The damping stays:
    // an arm still coasting inside the deadband would otherwise leave it and
    // re-open the error every cycle.
    cmd.v = clamp_norm(v_ff_safe + v_damp, params.v_max_total);
    cmd.w = clamp_norm(w_ff_safe + w_damp, params.w_max_total);
    cmd.in_deadband = true;
    // Deliberately do NOT touch state: the proportional path is zeroed, the
    // filter memory is preserved.
    return cmd;
  }

  // Proportional term, clamped as a vector, then scaled by the geometry
  // source gain (formula order: scale applies to the CLAMPED value). A
  // non-finite scale (bad upstream tier) is degraded to zero rather than
  // being allowed to poison the persistent filter state with NaNs.
  tf2::Vector3 v_raw = clamp_norm(e_pos * params.kp_pos, params.v_max) *
    source_scale;
  tf2::Vector3 w_raw = clamp_norm(e_ang * params.kp_ang, params.w_max) *
    source_scale;
  if (!is_finite(v_raw)) {
    v_raw = tf2::Vector3(0, 0, 0);
  }
  if (!is_finite(w_raw)) {
    w_raw = tf2::Vector3(0, 0, 0);
  }

  const double alpha = lowpass_alpha(params);
  state.v_filtered += (v_raw - state.v_filtered) * alpha;
  state.w_filtered += (w_raw - state.w_filtered) * alpha;

  TwistCommand cmd;
  cmd.v = clamp_norm(state.v_filtered + v_ff_safe + v_damp, params.v_max_total);
  cmd.w = clamp_norm(state.w_filtered + w_ff_safe + w_damp, params.w_max_total);
  cmd.in_deadband = false;
  return cmd;
}

}  // namespace lgw_bt_planner
