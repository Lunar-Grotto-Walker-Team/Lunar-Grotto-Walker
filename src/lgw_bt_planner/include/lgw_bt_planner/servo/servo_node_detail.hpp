// include/lgw_bt_planner/servo/servo_node_detail.hpp
//
// File-scope constants and helpers for servo_node.
//
// They used to live in an anonymous namespace inside servo_node.hpp, which
// gave every translation unit its own copy and made `ServoNode`'s member
// `start_state_` a type with internal linkage (GCC -Wsubobject-linkage).
// A named namespace in a private header fixes that and keeps the constants
// out of the node's public surface.

#ifndef LGW_BT_PLANNER__SERVO__SERVO_NODE_DETAIL_HPP_
#define LGW_BT_PLANNER__SERVO__SERVO_NODE_DETAIL_HPP_

#include <cmath>
#include <cstdint>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace lgw_bt_planner
{
namespace servo_detail
{

// --- topics / services (spec §7.2) -----------------------------------------
constexpr char kTrackTopic[] = "/perception/tracks/unified";
constexpr char kTwistTopic[] = "/servo_node/delta_twist_cmds";
constexpr char kStateTopic[] = "/lgw/servo/state";
constexpr char kServoStatusTopic[] = "/servo_node/status";
// Servo's own output topic (servo_params.yaml command_out_topic). Its first
// message is the only hard proof that start_servo really started the calc
// thread (servo_node.cpp startCB returns success regardless — spec §7.9).
constexpr char kServoOutputTopic[] = "/arm_controller/joint_trajectory";
constexpr char kMoveActionStatusTopic[] = "/move_action/_action/status";
constexpr char kFtActionStatusTopic[] =
  "/arm_controller/follow_joint_trajectory/_action/status";

constexpr char kTakeoverService[] = "/lgw/servo/takeover";

// A change of wrist branch moves j5 by at least 2 * j5_branch_min (1.2 rad);
// within one branch the solver's j5 swings at most ~0.35 rad. 0.6 separates
// the two cleanly.
constexpr double kBranchSwitchDeltaRad = 0.6;
constexpr char kReleaseService[] = "/lgw/servo/release";
constexpr char kPauseService[] = "/lgw/servo/pause";
constexpr char kResumeService[] = "/lgw/servo/resume";

constexpr char kStartServoService[] = "/servo_node/start_servo";
constexpr char kPauseServoService[] = "/servo_node/pause_servo";
constexpr char kUnpauseServoService[] = "/servo_node/unpause_servo";

// Servo expects the twist in its planning frame / robot_link_command_frame,
// which servo_params.yaml pins to the BARE `base_link` (no arm1/ prefix).
constexpr char kTwistFrame[] = "base_link";
// odom is never prefixed (only rsp's robot frames are).
constexpr char kOdomFrame[] = "odom";

// Arm joints, in the order the MoveIt group uses.
constexpr size_t kArmJointCount = 6;
constexpr char kArmJoints[kArmJointCount][8] = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

// --- timings ---------------------------------------------------------------
// /pose_watch_topic({/control,/feedback}/joint_states) liveness (spec §7.8).
constexpr int64_t kPoseWatchTimeoutNs = 100'000'000;  // 100 ms

// Following monitor (spec §5.5). Servo integrates its commands on the JTC/mock
// state, so the arm is a *follower* of that state; if the real arm falls
// behind, the commanded pose stops meaning anything. Compare the two joint
// vectors and drop to HOLD when they diverge for longer than the window.
constexpr int64_t kFollowWarnPeriodNs = 1'000'000'000;  // 1 s log throttle
// Servo start_servo blocks internally on waitForCompleteState(10 s) and its
// response ALWAYS says success (spec §7.9), so the response is only a trigger:
// after it we publish a zero twist and require Servo's own output on
// /arm_controller/joint_trajectory within 300 ms, else FAULT.
// 2026-11-18: 12 s -> 30 s。真机上 Servo 的初始化(robot model + planning
// scene monitor + waitForCompleteState)在 SoC 负载高时要 14 s 以上, 12 s 的
// 截止线会把"迟到但 success=true"的响应判成 FAULT, 节点从此只能重启
// (takeover 全部 rejected: fault)。放宽到 30 s 只是少一次误判 —— Servo 真
// 起不来时, 后面的 output 验证(kServoStartVerifyNs)仍然会判 FAULT。
constexpr int64_t kServoStartResponseTimeoutNs = 30'000'000'000;  // 30 s
constexpr int64_t kServoStartVerifyNs = 300'000'000;              // 300 ms
// How often the zero twist is re-sent while waiting for Servo's output.
// Servo drops a command older than incoming_command_timeout (0.3 s), and a
// zero twist is exactly the safe no-motion probe.
constexpr int64_t kServoStartProbePeriodNs = 100'000'000;  // 100 ms
constexpr double kPauseTimeoutSec = 1.0;
constexpr double kUnpauseTimeoutSec = 1.0;
// Re-running start_servo at takeover can block inside Servo's
// waitForCompleteState(10 s); give the response the same 12 s the boot path
// allows.
constexpr double kReseedTimeoutSec = 12.0;
// Angular error that has not improved for this long -> saturated -> HOLD
// (spec §7.6 "3 s"). A 0.5 deg improvement re-arms the window so sensor
// noise cannot keep the timer running forever.
constexpr int64_t kSaturationWindowNs = 3'000'000'000;  // 3 s
constexpr double kSaturationImproveRad = 0.008726646;   // 0.5 deg
// Command-side progress threshold for the unreachable guard: if the commanded
// joints move this much, the loop is advancing and saturation is not declared.
constexpr double kSatCmdProgressRad = 0.02;
// Servo halt statuses that persist this long while TRACKING -> HOLD (§7.8).
constexpr int64_t kServoHaltWindowNs = 3'000'000'000;  // 3 s
constexpr int64_t kServoHaltResetNs = 1'000'000'000;   // 1 s without a halt
// The control timer runs at 100 Hz; publish /lgw/servo/state at 10 Hz (plus
// immediately on any state / geometry-source change).
constexpr uint64_t kStatePublishEveryTicks = 10;

// moveit_servo StatusCode values we treat as "Servo halted" (status_codes.h).
constexpr int8_t kServoStatusHaltSingularity = 2;
constexpr int8_t kServoStatusHaltCollision = 4;
constexpr int8_t kServoStatusJointBound = 5;

inline int64_t time_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

// Magnitude of the shortest rotation taking `current` to `desired`
// (radians, [0, pi]). Reporting only — the control law computes its own
// rotation vector in servo_control_law.cpp.
inline double rotation_angle(
  const tf2::Matrix3x3 & desired, const tf2::Matrix3x3 & current)
{
  tf2::Quaternion q;
  (desired * current.transpose()).getRotation(q);
  q.normalize();
  const double sin_half =
    std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  return 2.0 * std::atan2(sin_half, std::fabs(q.w()));
}

inline bool status_array_busy(
  const action_msgs::msg::GoalStatusArray::SharedPtr & array)
{
  if (!array) {
    return false;
  }
  for (const auto & goal : array->status_list) {
    if (goal.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      goal.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      return true;
    }
  }
  return false;
}

}  // namespace servo_detail
}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SERVO_NODE_DETAIL_HPP_
