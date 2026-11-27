// body_pose_bridge_node.hpp
//
// BodyPoseBridgeNode — BT→Sport API 适配器,把 BT SetBodyPose 的一次性 RPC
// 翻译成 Go2 firmware 需要的 (Euler → BalanceStand) 2-call sequence +
// 持续 50Hz Euler refresh 让 firmware 保持 latched rpy。
//
// 设计历史(2026-10-10~10-13,真机 Jetson 调试):
//   v1 50Hz full 3-call —— rpy 在 [target, 中立] 震荡,verify 凑不齐
//   v2 单发 3-call —— 一次成功,但 ~1s 内 firmware balance 拉回中立
//   v3 100Hz BalanceStand-only —— verify 仍凑不齐
//   v4 5Hz full 3-call —— 同 v1 震荡
//   v5 单发 3-call + 50Hz verify-timer with cv_.wait_until ——
//      真机验证发现 SingleThreadedExecutor 在 cv_.wait_until 阻塞期间
//      repeat_timer_/verify_timer_ 全部不 fire,refresh 实际从未跑过;
//      verify 永远 0 stable ticks timeout,BT 永远 FAILURE,hold 行为
//      从未发生。
//   v6 — 单发 3-call + 50Hz Euler+BH refresh,handler 立即返回 SUCCESS,
//      把 lifecycle 完全交给 BT。删 verify(cv/stability/tolerance/
//      timer 全删),handler 不再阻塞 executor,refresh timer 立即可以
//      fire。
//   v7 (2026-10-13) — 移除 body_h 控制。Go2 固件对 BodyHeight api(1013)
//      bh_off 参数不响应(真机 act body_h 在 bh_off=0 vs bh_off=-0.18
//      几乎无差,差 ≤ 2mm),因此:
//        * .srv 去除 body_h 字段;
//        * bridge 3-call (Euler→BodyHeight→BalStand) 退化为 2-call
//          (Euler→BalStand);
//        * 50Hz refresh (Euler+BH) 退化为 Euler-only;
//        * target_body_h_offset_ 成员 / kBodyHeightOffset* / kApiIdBodyHeight
//          / publish_body_height() 全部删除。
//      SetBodyPose 现在只负责 rpy 倾斜;body 升降由 StandUp 的身高档位
//      覆盖。
//
// state machine (v7):
//   IDLE     —— 默认,无 refresh,无 timer
//   HOLDING  —— refresh loop 跑着,每 20ms 重发 Euler(不重发 BalanceStand
//              也不发 BodyHeight)。handler 在刚启动 refresh 时进入 HOLDING,
//              clear 路径取消 timer 时回 IDLE。
//
// 设计参考:docs/superpowers/specs/2026-10-10-body-pose-bridge-design.md。

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "body_pose_bridge/srv/set_body_pose.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace body_pose_bridge
{

class BodyPoseBridgeNode : public rclcpp::Node
{
public:
  explicit BodyPoseBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- Service handler (v6+: fires 2-call, starts refresh loop, returns
  //      SUCCESS immediately; lifecycle owned by BT, not the bridge) ----
  void handle_set_body_pose(
    const std::shared_ptr<srv::SetBodyPose::Request> req,
    std::shared_ptr<srv::SetBodyPose::Response> res);

  // ---- 2026-10-10 BCD fix: graceful-shutdown fail-safe ----
  // Registered as rclcpp::on_shutdown() callback at construction.
  // If the bridge is still HOLDING when shutdown begins (e.g. the BT
  // tree FAILURE'd mid-tilt and the operator Ctrl-C'd launch), cancel
  // the refresh timer and publish one final (0,0,0) + BalanceStand
  // sequence to release the dog back to neutral with a known good
  // posture, instead of relying on the implicit "Go2 firmware balance
  // controller drifts back within ~1s of last BalanceStand" behavior.
  // Does NOT cover SIGKILL / OOM paths.
  void fail_safe_neutral_publish();

  // ---- 50Hz refresh timer callback (v6+): re-send Euler only to
  //      refresh target rpy while keeping the firmware balance session
  //      alive from the one-shot 2-call entry at handle_set_body_pose().
  //      v7 dropped BodyHeight refresh (firmware dead control). ----
  void publish_euler_repeat();

  // ---- Clear收尾:1 次 (0,0,0) Euler + BalStand ----
  void send_final_balanced_pose();

  // ---- lf/sportmodestate 订阅回调,缓存 latest_state_ ----
  void state_cb(const unitree_go::msg::SportModeState::SharedPtr msg);

  // ---- 内部辅助:发布单个 sport API request ----
  void publish_euler(float r, float p, float y);
  void publish_balance_stand();

  // ---- members ----
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
  rclcpp::Service<srv::SetBodyPose>::SharedPtr service_server_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  // Single refresh timer (v6+): 50Hz (20ms), calls publish_euler_repeat()
  // — re-sends Euler only (no BalanceStand, no BodyHeight). v1/v4
  // oscillated because BalanceStand re-enters balance session too fast;
  // keeping it out of the refresh loop lets the latched balance session
  // (set up by the one-shot 2-call) persist while rpy targets get
  // refreshed at 50Hz. v5 also ran a verify_timer_ in parallel but its
  // cv_.wait_until handler blocked the SingleThreadedExecutor and starved
  // both timers — verify never got a single tick. v6 drops verify
  // entirely; BT owns lifecycle.
  // Use TimerBase (not the templated WallTimer<...>) so the SharedPtr
  // doesn't need the callback functor type in the header.
  rclcpp::TimerBase::SharedPtr repeat_timer_;

  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_{};
  bool state_received_{false};

  std::mutex target_mutex_;            // 保护 target_rpy / timer / state
  float target_roll_{0.0f};
  float target_pitch_{0.0f};
  float target_yaw_{0.0f};

  enum class State { IDLE, HOLDING };
  State state_{State::IDLE};

  // constants (mirrored from lgw_bt_planner sport_api_constants.hpp —
  // duplicated here so this package has no cross-package dependency on
  // lgw_bt_planner).
  static constexpr float kEulerRollMin = -0.75f;
  static constexpr float kEulerRollMax =  0.75f;
  static constexpr float kEulerPitchMin = -0.75f;
  static constexpr float kEulerPitchMax =  0.75f;
  static constexpr float kEulerYawMin = -3.14159f;
  static constexpr float kEulerYawMax =  3.14159f;
  static constexpr int32_t kApiIdEuler = 1007;
  static constexpr int32_t kApiIdBalanceStand = 1002;
};

}  // namespace body_pose_bridge
