// body_pose_bridge_node.cpp
//
// BodyPoseBridgeNode implementation. See hpp for design rationale and
// docs/superpowers/specs/2026-10-10-body-pose-bridge-design.md for the
// full design.
//
// Hard constraints (must NEVER be relaxed):
//   * Call sequence is Euler(1007) → BalanceStand(1002). The Go2 state
//     machine latches rpy at BalanceStand time (see
//     unitree_sdk2/example/go2/go2_sport_client.cpp:63 "controlled by
//     dBodyHeight + rpy" — the SDK comment still mentions dBodyHeight
//     but the firmware only honors rpy; BodyHeight 1013 is a dead
//     control on Go2 per real-hardware 2026-10-10 verification, see
//     feedback memory feedback_go2_bodyheight_1013_unresponsive).
//     Sending Euler AFTER BalanceStand is silently dropped.
//   * Drop-BalanceStand-from-refresh model (2026-10-10 v5, still in
//     force as of v7): bridge sends the 2-call sequence ONCE at
//     handle_set_body_pose() to latch entry into balance mode + initial
//     rpy. Then runs a 50Hz repeat_timer_ that re-sends ONLY Euler
//     (no BalanceStand, no BodyHeight). Hypothesis (validated across
//     v1-v4): re-sending BalanceStand at > 1Hz restarts the balance
//     session, forcing the controller to re-settle every cycle.
//   * v6 (current refresh design): drop-verify. v5's verify_timer_ +
//     cv_.wait_until in handle_set_body_pose() blocked the
//     SingleThreadedExecutor for the full 5s window, so repeat_timer_
//     never got a single tick. v6 drops verify entirely: the handler
//     fires the 2-call + starts repeat_timer_ + returns SUCCESS
//     immediately. BT owns lifecycle; refresh runs in the background
//     while BT does Sleep 2s; clear=true cancels the timer.
//   * v7 (2026-10-13): drop-BodyHeight. Go2 firmware does not honor
//     api 1013 bh_off (real-hardware verified 2026-10-10), so the
//     3-call (Euler→BodyHeight→BalStand) collapses to 2-call
//     (Euler→BalStand) and the 50Hz refresh collapses to Euler-only.
//     body_h 升降通过 StandUp 的身高档位覆盖。
//   * In HOLDING, the bridge owns /api/sport/request. repeat_timer_
//     keeps refreshing Euler; BalanceStand stays latched from the
//     one-shot 2-call. clear=true cancels the timer and sends a final
//     (0,0,0) + BalanceStand pair, ending the firmware balance
//     session.
//   * XML callers must still issue `<SetBodyPose clear="true"/>` before
//     StandDown to release /api/sport/request so StandDown's 1005
//     publish is not stomped by the bridge's 50Hz refresh.

#include "body_pose_bridge/body_pose_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/qos.hpp>

namespace body_pose_bridge
{

BodyPoseBridgeNode::BodyPoseBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("body_pose_bridge_node", options)
{
  // RELIABLE on /api/sport/request (mirrors sport_api_constants.hpp and
  // the other Go2 BT nodes — Go2 firmware processes in order, but we
  // want every republish delivered).
  request_pub_ = this->create_publisher<unitree_api::msg::Request>(
    "/api/sport/request", rclcpp::QoS(1));

  // BEST_EFFORT on lf/sportmodestate — same as other BT nodes.
  state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
    "lf/sportmodestate",
    rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      this->state_cb(msg);
    });

  // Timer callback group, isolated from default group so the refresh
  // timer can run on its own (v6 handler returns immediately, so this
  // is mostly defensive — keeps the timer callable independent of any
  // future blocking handler that might re-introduce one).
  timer_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<srv::SetBodyPose>(
    "/set_body_pose",
    [this](
      const std::shared_ptr<srv::SetBodyPose::Request> req,
      std::shared_ptr<srv::SetBodyPose::Response> res)
    {
      this->handle_set_body_pose(req, res);
    });

  RCLCPP_INFO(this->get_logger(),
    "body_pose_bridge_node initialized: pub=/api/sport/request "
    "sub=lf/sportmodestate service=/set_body_pose");

  // Fail-safe on graceful shutdown (2026-10-10 BCD fix, 2026-10-13 v7
  // 简化): when this bridge is killed via Ctrl-C / launch kill-tree /
  // SIGTERM, the rclcpp shutdown sequence fires rclcpp::on_shutdown
  // callbacks BEFORE publishers are torn down. If we're still HOLDING
  // a non-neutral pose, publish one last (0,0,0) + BalanceStand pair +
  // cancel the 50Hz timer so the dog is explicitly released back to
  // neutral with a known good posture, instead of relying on the
  // implicit "balance controller pulls body back within ~1s of last
  // BalanceStand" behavior described in spec §5 / §1.1.
  //
  // SIGKILL / OOM paths are NOT covered — there is no user-space
  // hook. Those paths still rely on the spec §5 fail-safe (firmware
  // balance controller drift). This is belt-and-suspenders for the
  // gracefully-killed case only.
  rclcpp::on_shutdown(
    [this]() { this->fail_safe_neutral_publish(); });
}

void BodyPoseBridgeNode::fail_safe_neutral_publish()
{
  // Locked so we don't race publish_euler_repeat() during the brief
  // window between cancel() and our final publish (cancel schedules
  // stop but a tick may already be in-flight).
  std::lock_guard<std::mutex> lock(target_mutex_);

  if (state_ != State::HOLDING) {
    RCLCPP_INFO(this->get_logger(),
      "[shutdown] bridge not HOLDING, no fail-safe neutral publish");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "[shutdown] bridge HOLDING (last rpy=[%.3f, %.3f, %.3f]) → "
    "cancelling refresh timer + publishing fail-safe neutral pose",
    target_roll_, target_pitch_, target_yaw_);

  // Cancel the refresh timer FIRST so it doesn't double-publish during
  // the shutdown sequence. cancel() is async; reset() also drops the
  // shared_ptr. Either alone leaves the timer alive briefly; both
  // gives the strongest "no more ticks" guarantee short of waiting
  // on a future.
  if (repeat_timer_) {
    repeat_timer_->cancel();
    repeat_timer_.reset();
  }

  // Reset targets to neutral and push one final 2-call sequence
  // through the publisher that's still alive at this point in
  // shutdown.
  target_roll_ = 0.0f;
  target_pitch_ = 0.0f;
  target_yaw_ = 0.0f;
  state_ = State::IDLE;

  publish_euler(0.0f, 0.0f, 0.0f);
  publish_balance_stand();
}

void BodyPoseBridgeNode::state_cb(
  const unitree_go::msg::SportModeState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool first = !state_received_;
  latest_state_ = *msg;
  state_received_ = true;
  if (first) {
    RCLCPP_INFO(this->get_logger(),
      "first state cb: rpy=[%.3f, %.3f, %.3f] body_h=%.3f mode=%u",
      msg->imu_state.rpy[0], msg->imu_state.rpy[1],
      msg->imu_state.rpy[2], msg->body_height, msg->mode);
  }
}

void BodyPoseBridgeNode::publish_euler(float r, float p, float y)
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = kApiIdEuler;
  // Match SportClient::Euler parameter — built manually to avoid pulling
  // nlohmann/json into this node. See
  // unitree_ros2/example/src/.../ros2_sport_client.cpp.
  req.parameter = std::string("{\"x\": ") + std::to_string(r) +
                  ", \"y\": " + std::to_string(p) +
                  ", \"z\": " + std::to_string(y) + "}";
  request_pub_->publish(req);
}

void BodyPoseBridgeNode::publish_balance_stand()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = kApiIdBalanceStand;
  request_pub_->publish(req);
}

void BodyPoseBridgeNode::send_final_balanced_pose()
{
  // Clear收尾:按 call sequence 发一次 rpy=0 + BalStand
  // (must hold target_mutex_ while calling this; caller does that).
  publish_euler(0.0f, 0.0f, 0.0f);
  publish_balance_stand();
}

void BodyPoseBridgeNode::publish_euler_repeat()
{
  // Snapshot under lock; do publish outside the lock so a slow DDS
  // subscriber doesn't stall other callbacks holding target_mutex_.
  float r, p, y;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (state_ != State::HOLDING) {
      return;  // timer fired during shutdown / after clear
    }
    r = target_roll_;
    p = target_pitch_;
    y = target_yaw_;
  }

  // v6+: drop-BalanceStand-from-refresh. Re-send Euler at 50Hz to
  // refresh target rpy while the firmware balance session (latched by
  // the one-shot 2-call in handle_set_body_pose) stays alive. We
  // deliberately DO NOT call publish_balance_stand() here — hypothesis
  // is that repeatedly re-entering the balance session is what caused
  // v1 (50Hz full 3-call) and v4 (5Hz full 3-call) to oscillate.
  // v7 (2026-10-13) dropped BodyHeight from the refresh too — Go2
  // firmware is a dead control for body_h.
  publish_euler(r, p, y);

  // [DIAGNOSTIC 2026-10-10 v6, kept across v7] 1Hz 节流打印 HOLDING
  // 期间 dog 实际 rpy + body_h 状态,验证 refresh 真的在跑 + firmware
  // 真的 latch 目标 pitch。body_h 保留打印是为了看 firmware 真实身高
  // (虽然 bridge 不再控它);target bh_off 字段在 v7 已删。
  unitree_go::msg::SportModeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[hold-dump] act rpy=[%.3f, %.3f, %.3f] body_h=%.3f mode=%u "
    "| tgt rpy=[%.3f, %.3f, %.3f]",
    state.imu_state.rpy[0], state.imu_state.rpy[1], state.imu_state.rpy[2],
    state.body_height, state.mode,
    r, p, y);
}

void BodyPoseBridgeNode::handle_set_body_pose(
  const std::shared_ptr<srv::SetBodyPose::Request> req,
  std::shared_ptr<srv::SetBodyPose::Response> res)
{
  std::unique_lock<std::mutex> lock(target_mutex_);

  if (req->clear) {
    // ---- clear 路径 ----
    const bool was_idle = (state_ == State::IDLE);
    if (state_ == State::HOLDING) {
      // 取消 refresh timer,发一次 2-call (0,0,0) 让 dog 回到中立
      repeat_timer_.reset();
      send_final_balanced_pose();
    }
    state_ = State::IDLE;
    res->success = true;
    res->message = was_idle ? "clear_already_idle" : "ok";
    RCLCPP_INFO(this->get_logger(),
      "[set_body_pose] clear=true (was %s) → IDLE",
      was_idle ? "IDLE" : "HOLDING");
    return;
  }

  // ---- set 路径 ----
  // 1. clamp rpy 到 Go2 spec 范围
  const float r = static_cast<float>(std::clamp(
    req->roll,
    static_cast<double>(kEulerRollMin),
    static_cast<double>(kEulerRollMax)));
  const float p = static_cast<float>(std::clamp(
    req->pitch,
    static_cast<double>(kEulerPitchMin),
    static_cast<double>(kEulerPitchMax)));
  const float y = static_cast<float>(std::clamp(
    req->yaw,
    static_cast<double>(kEulerYawMin),
    static_cast<double>(kEulerYawMax)));
  if (r != req->roll || p != req->pitch || y != req->yaw)
  {
    RCLCPP_WARN(this->get_logger(),
      "[set_body_pose] input (r=%.3f, p=%.3f, y=%.3f) clamped to "
      "(r=%.3f, p=%.3f, y=%.3f)",
      req->roll, req->pitch, req->yaw,
      r, p, y);
  }

  // 2. 更新目标
  target_roll_ = r;
  target_pitch_ = p;
  target_yaw_ = y;

  // 3. (v6+) 单次 2-call + 启动 50Hz refresh timer + 立即返回 SUCCESS。
  //    v5 的 verify_timer_ + cv_.wait_until 在 SingleThreadedExecutor 里
  //    把 executor 阻塞 5s,repeat_timer_ 一次都没 fire,refresh 实际从未跑。
  //    v6+ 删 verify:handler 发完 2-call + 启动 repeat_timer_ 后立即返回
  //    SUCCESS,BT 走下一步(Sleep 2s),executor 切到 refresh timer,50Hz
  //    Euler 持续刷新 firmware latched 的 rpy。
  //    下一次 SetBodyPose(recenter)直接 update target_;clear=true 路径
  //    取消 timer + 发最终 2-call 回中立。

  publish_euler(r, p, y);
  publish_balance_stand();   // 触发 latch(rpy 写入 firmware)

  // 50Hz repeat_timer_:每 20ms 重发 Euler(不发 BalanceStand,
  // 假设 BalanceStand 重启 balance session 是 v1-v4 震荡的根因)。
  // BalanceStand 只在 set path 开头发一次作为 mode entry,latch 之后
  // session 由 Euler 持续刷新 target 维持。
  repeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() { this->publish_euler_repeat(); },
    timer_cb_group_);

  state_ = State::HOLDING;
  RCLCPP_INFO(this->get_logger(),
    "[set_body_pose] v7 fired-and-returned SUCCESS, refresh loop running "
    "(50Hz Euler-only), rpy=[%.3f, %.3f, %.3f]",
    r, p, y);

  // v6+: handler 立即返回 SUCCESS。BT 拿 success 后进入 Sleep 2s,
  // executor 切到 refresh timer。clear=true 时 BT 会发回,这里 cancel。
  res->success = true;
  res->message = "ok";
}

}  // namespace body_pose_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<body_pose_bridge::BodyPoseBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
