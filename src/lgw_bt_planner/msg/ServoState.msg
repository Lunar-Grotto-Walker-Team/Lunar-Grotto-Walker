# ServoState.msg
# servo 节点状态与误差上报(/lgw/servo/state)。
#
# state            IDLE / PREEMPT / TRACKING / HOLD / RECOVERING / RELEASING / FAULT
#                  (RECOVERING = HOLD 的主动子状态:Servo paused,本节点用 MoveGroup
#                   移动机械臂;见 spec 2026-11-03 §6)
#                  (对应 C++ enum class lgw_bt_planner::ServoState;
#                   两者同名但类型不同,节点内禁止 using namespace
#                   lgw_bt_planner::msg;)
# geometry_source  head / arm / odom / none(TRACKING 子状态)
# track_id         当前跟踪的 track(无则 0)
# angular_error_rad    期望 tcp 姿态与当前姿态的角误差
# position_error_m     期望 tcp 位置与当前位置的距离误差
# target_distance_m    tcp 到目标点的距离
# converged / saturated / too_close   收敛 / 指令饱和 / 目标过近
# stamp            本状态采样时刻
#
# 见 docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md §7.2。

string state
string geometry_source
uint64 track_id
float64 angular_error_rad
float64 position_error_m
float64 target_distance_m
bool converged
bool saturated
bool too_close
# Pre-position diagnostics (spec 2026-10-30 §3.8): whether the takeover had to
# move the arm into the posture family, and how far outside it was.
bool needs_preposition
float64 max_joint_delta
# --- HOLD recovery (spec 2026-11-03) -------------------------------------
# hold_reason        none / target_lost / tracker_stale / pose_stale /
#                    move_group_busy / servo_halt / saturated /
#                    follower_lag / operator_pause
# recovery_attempts  MoveGroup recovery moves started in this HOLD session
# recovery_exhausted no further automatic recovery until release/takeover
# resumable          TARGET_OK latch armed (a resume would be accepted)
string hold_reason
uint32 recovery_attempts
bool recovery_exhausted
bool resumable
builtin_interfaces/Time hold_since
builtin_interfaces/Time stamp
