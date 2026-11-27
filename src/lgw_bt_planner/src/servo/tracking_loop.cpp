// src/servo/tracking_loop.cpp
//
// Task 6 of the 2026-11-07 node-split plan. See the header for the split of
// duties. Pure C++17 + tf2 math: no rclcpp, no TF, no clock.

#include "lgw_bt_planner/servo/tracking_loop.hpp"

#include <algorithm>
#include <cmath>

namespace lgw_bt_planner
{

TrackingOutput TrackingLoop::step(
  const TrackingInput & in, const ServoParams & params, SessionState & st)
{
  TrackingOutput out;
  out.source_name = in.source_name;

  // Refusals are ordered from "no target at all" to "target fine but the
  // geometry is unusable", because the reason names drive the HOLD recovery
  // policy (target_lost and tracker_stale are recoverable, pose_stale is not).

  // No tier won this tick: nothing to aim at. Drop the feed-forward memory —
  // after a dropout the next setpoint is not the continuation of this one,
  // and differentiating across the gap would command a jump.
  if (in.source_name == kNoSource || !(in.source_scale > 0.0)) {
    ff_have_prev_ = false;
    st.hold_reason = HoldReason::kTargetLost;
    st.geometry_source = kNoSource;
    out.hold_reason = st.hold_reason;
    return out;
  }

  // The tracker says this point is not drivable (TF missing / no geometry).
  if (!in.geometry_valid) {
    ff_have_prev_ = false;
    st.hold_reason = HoldReason::kTrackerStale;
    st.geometry_source = kNoSource;
    out.hold_reason = st.hold_reason;
    return out;
  }

  // The follower's joint message did not name every joint. Zero-filling the
  // missing ones would command a pose nobody measured.
  if (!in.joints_complete) {
    ff_have_prev_ = false;
    st.hold_reason = HoldReason::kJointStateIncomplete;
    out.hold_reason = st.hold_reason;
    return out;
  }

  // A NaN must never reach Servo, and the deadband path inside compute_twist
  // silently zeroes non-finite input — which would look like convergence.
  if (!pose_finite(in.tcp_now) || !pose_finite(in.desired) ||
    !vector_finite(in.p_arm_base))
  {
    ff_have_prev_ = false;
    st.hold_reason = HoldReason::kPoseStale;
    out.hold_reason = st.hold_reason;
    return out;
  }

  tf2::Vector3 v_ff(0, 0, 0);
  tf2::Vector3 w_ff(0, 0, 0);
  feed_forward(in, params, v_ff, w_ff);

  out.twist = compute_twist(
    in.desired, in.tcp_now, params.control, law_state_, in.source_scale,
    v_ff, w_ff, in.v_meas, in.w_meas);
  out.publish = true;
  out.hold_reason = HoldReason::kNone;
  st.hold_reason = HoldReason::kNone;
  st.geometry_source = in.source_name;
  return out;
}

void TrackingLoop::reset()
{
  law_state_ = ControlLawState{};
  ff_prev_desired_ = Pose6{};
  ff_prev_ns_ = 0;
  ff_have_prev_ = false;
  ff_v_.setValue(0, 0, 0);
  ff_w_.setValue(0, 0, 0);
}

void TrackingLoop::feed_forward(
  const TrackingInput & in, const ServoParams & params,
  tf2::Vector3 & v_ff, tf2::Vector3 & w_ff)
{
  v_ff.setValue(0, 0, 0);
  w_ff.setValue(0, 0, 0);
  if (!params.ff_enable) {
    return;
  }

  const double dt = ff_have_prev_ ?
    static_cast<double>(in.now_ns - ff_prev_ns_) * 1e-9 : 0.0;
  if (ff_have_prev_ && dt > 1e-3) {
    const tf2::Vector3 v_ff_raw = (in.desired.p - ff_prev_desired_.p) / dt;
    // Small-angle angular velocity from the skew part of the rotation error;
    // per tick the rotation is ~1e-2 rad, well inside the range where the
    // linearisation is exact to the nanometre level.
    const tf2::Matrix3x3 R_err = ff_prev_desired_.R.transpose() * in.desired.R;
    const tf2::Vector3 w_ff_raw(
      (R_err[2][1] - R_err[1][2]) / (2.0 * dt),
      (R_err[0][2] - R_err[2][0]) / (2.0 * dt),
      (R_err[1][0] - R_err[0][1]) / (2.0 * dt));
    const double alpha = std::clamp(
      1.0 - std::exp(-2.0 * 3.14159265358979323846 * params.ff_lowpass_hz * dt),
      0.0, 1.0);
    ff_v_ += (v_ff_raw - ff_v_) * alpha;
    ff_w_ += (w_ff_raw - ff_w_) * alpha;
    if (ff_v_.length() < params.ff_deadband_mps) {
      ff_v_.setValue(0, 0, 0);
    }
    if (ff_w_.length() < params.ff_deadband_radps) {
      ff_w_.setValue(0, 0, 0);
    }
    const double v_len = ff_v_.length();
    if (v_len > params.ff_max_mps) {
      ff_v_ *= params.ff_max_mps / v_len;
    }
    const double w_len = ff_w_.length();
    if (w_len > params.ff_max_radps) {
      ff_w_ *= params.ff_max_radps / w_len;
    }
    v_ff = ff_v_;
    w_ff = ff_w_;
  }

  ff_prev_desired_ = in.desired;
  ff_prev_ns_ = in.now_ns;
  ff_have_prev_ = true;
}

}  // namespace lgw_bt_planner
