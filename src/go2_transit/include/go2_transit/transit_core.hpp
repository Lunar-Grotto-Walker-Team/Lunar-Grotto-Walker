// transit_core.hpp — 纯逻辑：角度数学 / 直线路径生成 / 状态机。
//
// 本头文件不包含任何 ROS 依赖，gtest 直接链接 go2_transit_core 即可测试。

#pragma once

#include <string>
#include <vector>

#include "go2_transit/transit_types.hpp"

namespace go2_transit
{

// 归一化到 (-pi, pi]。
double normalize_angle(double angle_rad);

// from 转到 to 的最短有向角差，范围 (-pi, pi]。
double shortest_angular_distance(double from_rad, double to_rad);

double distance_2d(const Pose2D & a, const Pose2D & b);

bool within_xy_tolerance(
  const Pose2D & current, const Pose2D & goal, double tolerance_m);

bool within_yaw_tolerance(
  double current_yaw, double target_yaw, double tolerance_rad);

// 从 start 到 goal 的等距直线路径，至少两点；每点 yaw 都指向 goal。
// spacing_m <= 0 或距离为 0 时退化成两点路径（不会死循环）。
std::vector<Pose2D> build_straight_path(
  const Pose2D & start, const Pose2D & goal, double spacing_m);

// 把"以 origin 为原点、origin.yaw 为 x 轴正向"的局部坐标换算到 origin 所在的坐标系。
//   x = origin.x + local_x·cos yaw − local_y·sin yaw
//   y = origin.y + local_x·sin yaw + local_y·cos yaw
Pose2D local_to_planning(const Pose2D & origin, double local_x, double local_y);

// BT 树里用 SetPoseStamped 的 frame_id 约定：只有字面量 "local" 表示局部坐标。
bool is_local_frame(const std::string & frame_id);

// 一次摆渡任务的状态机。
//
// 纯事件驱动，不持有时间：驱动层在定时器里调 next_command() 拿"下一条指令"，
// 把指令发给 nav2，再通过 report_action_done / report_timeout / fail 把结果喂回来。
// next_command() 是幂等的：状态没有推进时返回同一条（同 id），驱动层据此避免重复派发。
class TransitStateMachine
{
public:
  TransitStateMachine(const TransitParams & params, TaskMode mode);

  // 容差传 0 表示用 params 里的默认值。
  void set_transit_goal(
    const Pose2D & target, double xy_tolerance, bool align_yaw, double yaw_tolerance);
  void set_rotate_goal(double target_yaw, double yaw_tolerance);

  void set_ready(bool ready);
  void set_pose(const Pose2D & pose);
  void report_action_done(bool success);
  void report_timeout();
  void fail(FailReason reason);
  void cancel();

  MachineCommand next_command();
  TransitPhase phase() const { return phase_; }
  bool finished() const { return finished_; }
  FailReason result_reason() const { return result_reason_; }

private:
  enum class Pending { NONE, SPIN, FOLLOW_PATH };

  void advance_entry();
  void enter_pre_align(double bearing);
  void enter_traverse();
  void enter_final_align();
  void verify();
  void finish_ok();
  void issue_spin(double yaw);
  void issue_follow_path();
  void issue_terminal(MachineCommand::Kind kind, FailReason reason);
  bool consume_retry_or_fail();
  bool at_target_xy() const;
  bool at_target_yaw() const;

  TransitParams params_;
  TaskMode mode_;

  Pose2D target_{};
  double target_yaw_{0.0};
  double xy_tolerance_{0.20};
  double yaw_tolerance_{0.17};
  bool align_yaw_{false};

  Pose2D pose_{};
  bool ready_{false};
  TransitPhase phase_{TransitPhase::WAIT_READY};
  Pending pending_{Pending::NONE};

  bool pre_align_done_{false};
  bool fallback_used_{false};
  int retries_{0};

  bool finished_{false};
  FailReason result_reason_{FailReason::OK};
  std::uint64_t next_id_{0};
  MachineCommand last_{};
};

}  // namespace go2_transit
