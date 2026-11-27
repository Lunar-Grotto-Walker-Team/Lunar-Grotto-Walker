// transit_types.hpp — 与 ROS 无关的纯数据类型与枚举。
//
// 放在独立头里是为了让状态机、路径生成这些纯逻辑可以被 gtest 直接链接，
// 不需要任何 rclcpp / tf2 依赖。

#pragma once

#include <cstdint>
#include <vector>

namespace go2_transit
{

// 平面位姿（规划坐标系下的 x, y, yaw）。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class TaskMode { TRANSIT, ROTATE };

// 从 ROS 参数读进来、喂给状态机的一份配置快照。
struct TransitParams
{
  double default_xy_tolerance{0.20};
  double default_yaw_tolerance{0.17};
  double path_spacing_m{0.25};
  double pre_align_threshold_rad{0.5};
  bool pre_align_fallback{true};
  int max_retries{2};
};

enum class TransitPhase
{
  WAIT_READY,
  PRE_ALIGN,
  TRAVERSE,
  FINAL_ALIGN,
  VERIFY,
  DONE,
};

enum class FailReason
{
  OK,
  TIMEOUT,
  UNREACHABLE,
  NAV2_UNAVAILABLE,
  TF_UNAVAILABLE,
  INVALID_GOAL,
  CANCELED,
};

// 这两个字符串直接出现在 /go2_transit/status 与动作 result.reason 里，
// 是对外契约，不可随手改名。
const char * to_string(TransitPhase phase);
const char * to_string(FailReason reason);

// 状态机对驱动层下达的"下一条指令"。
//
// id 单调递增：状态不变时 next_command() 返回同一条（同 id），
// 驱动层据此判断"这条是不是已经派发过了"。
struct MachineCommand
{
  enum class Kind { WAIT_READY, SPIN, FOLLOW_PATH, SUCCEED, FAIL };

  Kind kind{Kind::WAIT_READY};
  std::uint64_t id{0};
  double spin_target_yaw{0.0};
  std::vector<Pose2D> path;
  FailReason fail_reason{FailReason::OK};
};

}  // namespace go2_transit
