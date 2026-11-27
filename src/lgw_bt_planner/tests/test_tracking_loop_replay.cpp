// tests/test_tracking_loop_replay.cpp
//
// Task 6 of the 2026-11-07 node-split plan, Step 4 ("回放 diff 为空").
//
// TrackingLoop replaced ~70 inline lines of update_tracking_locked. A unit
// test proves the new code obeys its own rules; it does NOT prove the rules
// are the old ones. This file supplies the reference: the pre-refactor block,
// transcribed from 3001f71 (servo_node.hpp, update_tracking_locked), driven
// over the same synthetic trajectory as the class. Any behavioural drift shows
// up as a per-tick mismatch, not as a smooth-looking log.
//
// The trajectory is deliberately mundane — a target orbiting a walking
// platform — because the equivalence claim only needs the arithmetic, not the
// scenario semantics.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lgw_bt_planner/servo/servo_params.hpp"
#include "lgw_bt_planner/servo/tracking_loop.hpp"

namespace
{

using lgw_bt_planner::ControlLawParams;
using lgw_bt_planner::ControlLawState;
using lgw_bt_planner::Pose6;
using lgw_bt_planner::ServoParams;
using lgw_bt_planner::SessionState;
using lgw_bt_planner::TrackingInput;
using lgw_bt_planner::TrackingLoop;
using lgw_bt_planner::TwistCommand;

// The baseline pipeline, verbatim from 3001f71: feed-forward, then
// compute_twist. Nothing here is allowed to "improve" — its only job is to be
// what the node used to do.
class BaselinePipeline
{
public:
  explicit BaselinePipeline(double scale = 1.0)
  : scale_(scale) {}

  TwistCommand step(
    const Pose6 & desired, const Pose6 & tcp_now, int64_t now_ns,
    const ServoParams & params)
  {
    tf2::Vector3 v_ff(0, 0, 0);
    tf2::Vector3 w_ff(0, 0, 0);
    if (params.ff_enable) {
      const double dt = ff_have_prev_ ?
        static_cast<double>(now_ns - ff_prev_ns_) * 1e-9 : 0.0;
      if (ff_have_prev_ && dt > 1e-3) {
        const tf2::Vector3 v_ff_raw = (desired.p - ff_prev_desired_.p) / dt;
        const tf2::Matrix3x3 R_err =
          ff_prev_desired_.R.transpose() * desired.R;
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
      ff_prev_desired_ = desired;
      ff_prev_ns_ = now_ns;
      ff_have_prev_ = true;
    }

    return lgw_bt_planner::compute_twist(
      desired, tcp_now, params.control, control_state_, scale_,
      v_ff, w_ff, tf2::Vector3(0, 0, 0), tf2::Vector3(0, 0, 0));
  }

private:
  double scale_{1.0};
  ControlLawState control_state_;
  Pose6 ff_prev_desired_;
  int64_t ff_prev_ns_{0};
  bool ff_have_prev_{false};
  tf2::Vector3 ff_v_{0, 0, 0};
  tf2::Vector3 ff_w_{0, 0, 0};
};

Pose6 trajectory_point(double t_s)
{
  Pose6 p;
  // Target orbits at 0.5 rad/s while the platform sways: the same shape of
  // input a walking dog produces.
  p.p.setValue(
    0.45 + 0.05 * std::sin(2.5 * t_s),
    0.04 * std::cos(0.9 * t_s),
    0.30 + 0.03 * std::sin(1.7 * t_s));
  const double yaw = 0.15 * std::sin(0.6 * t_s);
  p.R.setEulerYPR(yaw, 0.02 * std::cos(1.1 * t_s), 0.0);
  return p;
}

struct Tick
{
  int64_t now_ns;
  Pose6 desired;
  Pose6 tcp_now;
};

std::vector<Tick> make_ticks(int count)
{
  std::vector<Tick> ticks;
  ticks.reserve(count);
  for (int i = 0; i < count; ++i) {
    const int64_t now_ns = 1'000'000'000 + static_cast<int64_t>(i) * 10'000'000;
    const double t_s = static_cast<double>(i) * 0.01;
    Tick tick;
    tick.now_ns = now_ns;
    tick.desired = trajectory_point(t_s);
    // The arm trails the setpoint by ~300 ms, as the real driver does.
    tick.tcp_now = trajectory_point(t_s - 0.3);
    ticks.push_back(tick);
  }
  return ticks;
}

}  // namespace

TEST(TrackingLoopReplay, MatchesThePreRefactorTickForEveryTick)
{
  const auto ticks = make_ticks(400);
  ServoParams params;                // shipped defaults
  TrackingLoop loop;
  SessionState st;
  BaselinePipeline baseline;
  double peak = 0.0;

  for (size_t i = 0; i < ticks.size(); ++i) {
    TrackingInput in;
    in.now_ns = ticks[i].now_ns;
    in.source_name = "head";
    in.source_scale = 1.0;
    in.tcp_now = ticks[i].tcp_now;
    in.desired = ticks[i].desired;
    in.p_arm_base = ticks[i].desired.p;

    const auto out = loop.step(in, params, st);
    const auto ref = baseline.step(ticks[i].desired, ticks[i].tcp_now,
        ticks[i].now_ns, params);

    ASSERT_TRUE(out.publish) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.x(), ref.v.x()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.y(), ref.v.y()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.z(), ref.v.z()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.x(), ref.w.x()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.y(), ref.w.y()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.z(), ref.w.z()) << "tick " << i;
    EXPECT_EQ(out.twist.in_deadband, ref.in_deadband) << "tick " << i;
    peak = std::max(peak, ref.v.length());
  }

  // A trajectory that never commands anything would make every comparison
  // above trivially true (0 == 0). This is the guard that it did not.
  EXPECT_GT(peak, 0.01) << "the scenario never left the deadband";
}

TEST(TrackingLoopReplay, MatchesTheBaselineUnderTheShippedYamlGains)
{
  // The replay above runs on code defaults. The shipped yaml moves kp to
  // 3.0/5.0; the arithmetic must be equivalent there too, because that is what
  // the robot actually runs.
  const auto ticks = make_ticks(200);
  ServoParams params;
  params.control.kp_pos = 3.0;
  params.control.kp_ang = 5.0;
  params.control.kd_pos = 0.30;
  params.control.kd_ang = 0.30;
  params.ff_lowpass_hz = 6.0;

  TrackingLoop loop;
  SessionState st;
  BaselinePipeline baseline(0.7);    // the ARM tier's gain

  for (size_t i = 0; i < ticks.size(); ++i) {
    TrackingInput in;
    in.now_ns = ticks[i].now_ns;
    in.source_name = "arm";
    in.source_scale = 0.7;
    in.tcp_now = ticks[i].tcp_now;
    in.desired = ticks[i].desired;
    in.p_arm_base = ticks[i].desired.p;

    const auto out = loop.step(in, params, st);
    const auto ref = baseline.step(ticks[i].desired, ticks[i].tcp_now,
        ticks[i].now_ns, params);

    ASSERT_TRUE(out.publish) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.x(), ref.v.x()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.y(), ref.v.y()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.v.z(), ref.v.z()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.x(), ref.w.x()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.y(), ref.w.y()) << "tick " << i;
    EXPECT_DOUBLE_EQ(out.twist.w.z(), ref.w.z()) << "tick " << i;
    EXPECT_EQ(out.twist.in_deadband, ref.in_deadband) << "tick " << i;
  }
}
