// include/lgw_bt_planner/servo/tracking_loop.hpp
//
// The per-tick control pipeline as a class (Task 6 of the 2026-11-07
// node-split plan, spec step S3). Everything here is pure: no rclcpp, no TF
// lookup, no clock. The node gathers the tick's inputs — it is the only thing
// that owns a clock and a buffer — and this class decides what to command.
//
// Split of duties, deliberately narrow:
//   node        pick geometry (select_source), solve the setpoint (posture /
//               standoff / legacy), look up the TCP pose, measure the
//               follower gap, track recoveries.
//   TrackingLoop  refuse unacceptable inputs, remember the feed-forward
//               derivative, run compute_twist, say whether to publish.
//
// The refusal rules are the point of the class: each one used to be a line
// buried in a 350-line tick where only a robot run could prove it wrong.

#ifndef LGW_BT_PLANNER__SERVO__TRACKING_LOOP_HPP_
#define LGW_BT_PLANNER__SERVO__TRACKING_LOOP_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/servo/servo_params.hpp"
#include "lgw_bt_planner/servo_control_law.hpp"
#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Cross-tick memory owned by the SESSION, not by the node (spec §9 "状态归属").
// The loop fills the fields it computes; the rest belongs to the recovery
// orchestrator and the servo session, which take the same object by reference
// so the call order inside the control tick never changes.
struct SessionState
{
  HoldReason hold_reason{HoldReason::kNone};
  int recovery_attempts{0};
  std::string geometry_source{"none"};
  bool session_active{false};
  std::vector<double> last_sent_joints;
  bool last_sent_joints_valid{false};
  // HOLD recovery bookkeeping (Task 7). The orchestrator owns the decision;
  // these two are the only things it has to remember between ticks.
  bool recovery_exhausted{false};
  int64_t last_recovery_end_ns{0};
};

// Everything one tick needs that is NOT the node's persistent state.
struct TrackingInput
{
  int64_t now_ns{0};
  // The tracker's published validity flag. False -> HOLD; the servo no
  // longer applies its own freshness/tier timers.
  bool geometry_valid{true};
  // Winning source ("head"/"arm"/"odom"/"none") and its gain. "none" or a
  // non-positive scale means the target is lost.
  std::string source_name{"none"};
  double source_scale{0.0};
  Pose6 tcp_now;                  // measured TCP in arm base_link
  Pose6 desired;                  // setpoint solved by the node this tick
  tf2::Vector3 p_arm_base{0, 0, 0};
  // Measured TCP twist (J(q) dq), already gated by the node's damping rule.
  tf2::Vector3 v_meas{0, 0, 0};
  tf2::Vector3 w_meas{0, 0, 0};
  // False when the follower's joint message did not name every joint.
  bool joints_complete{true};
};

struct TrackingOutput
{
  TwistCommand twist;
  std::string source_name{"none"};
  // kNone when publish is true; otherwise why the tick was refused.
  HoldReason hold_reason{HoldReason::kNone};
  // True also for a deadband tick: a silent publisher looks dead to Servo, so
  // "hold still" is a zero twist, not an absent one.
  bool publish{false};
};

class TrackingLoop
{
public:
  static constexpr const char * kNoSource = "none";

  // One tick. `st` is the session's memory, updated in place.
  TrackingOutput step(
    const TrackingInput & in, const ServoParams & params, SessionState & st);

  // Forget the feed-forward derivative. Called when a session starts: the
  // first tick of a new session has no previous setpoint to differentiate,
  // and pretending otherwise would command last session's velocity.
  void reset();

private:
  // Derivative of the setpoint, low-passed, deadbanded and clamped. Pulled out
  // of the tick so the memory lives with the class that owns it.
  void feed_forward(
    const TrackingInput & in, const ServoParams & params,
    tf2::Vector3 & v_ff, tf2::Vector3 & w_ff);

  ControlLawState law_state_;
  Pose6 ff_prev_desired_;
  int64_t ff_prev_ns_{0};
  bool ff_have_prev_{false};
  tf2::Vector3 ff_v_{0, 0, 0};
  tf2::Vector3 ff_w_{0, 0, 0};
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__TRACKING_LOOP_HPP_
