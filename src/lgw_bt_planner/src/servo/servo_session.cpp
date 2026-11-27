// src/servo/servo_session.cpp
//
// Task 8 of the 2026-11-07 node-split plan. The three lifecycle services,
// lifted out of the node with their sequencing intact. Each was transcribed
// from servo_node.hpp's handle_release / handle_pause / handle_resume; the
// comments that explain WHY a step exists came with them.

#include "lgw_bt_planner/servo/servo_session.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace lgw_bt_planner
{

ServoSession::ServoSession(SessionHost & host)
: host_(host) {}

SessionResult ServoSession::release()
{
  SessionResult result;

  // A TAKEOVER pre-position may be in flight (PREEMPT blocks inside the
  // takeover handler); cancel that one. The recovery worker reuses the same
  // prepositioner but sets recovery_in_progress_, so it is excluded here and
  // its bounded move is allowed to finish (see below).
  host_.cancel_preposition_unless_recovery();

  // A HOLD recovery is a bounded, slow move that ends at a known posture
  // family. Do NOT cancel it: interrupting mid-trajectory leaves the arm in an
  // arbitrary pose, while finishing it is deterministic. What must stop is the
  // servo session: arm the cancel flag so the worker never re-seeds or
  // unpauses once the state has left RECOVERING.
  host_.arm_recovery_cancel();

  bool need_pause = false;
  bool refused = false;
  ServoState state_before{};
  ServoState state_after{};
  host_.under_lock([&] {
      state_before = host_.sm().state();
      if (!host_.sm().handle(ServoEvent::RELEASE)) {
        // Only FAULT refuses RELEASE (IDLE / RELEASING are accepted no-ops).
        refused = true;
        state_after = host_.sm().state();
        return;
      }
      need_pause = host_.sm().state() == ServoState::RELEASING;
      state_after = host_.sm().state();
      host_.mark_dirty();
    });

  if (refused) {
    result.message = std::string("release refused in state ") +
      state_name(state_before);
    host_.log_warn(result.message);
    return result;
  }
  if (!need_pause) {
    result.success = true;
    result.message = "already released";
    host_.log_info("release: already idle/releasing");
    return result;
  }

  const RpcOutcome outcome = host_.rpc(ServoRpc::kPause, "pause_servo");
  host_.under_lock([&] {
      if (outcome.ok) {
        host_.sm().handle(ServoEvent::PAUSE_OK);   // RELEASING -> IDLE
        host_.clear_session();
        result.success = true;
        result.message = "released";
      } else {
        host_.sm().handle(ServoEvent::FAULT);
        result.success = false;
        result.message = outcome.detail;
      }
    });

  if (result.success) {
    host_.log_info("release: Servo paused, control returned");
  } else {
    host_.log_error("release failed: " + outcome.detail + " -- state -> FAULT");
  }
  return result;
}

SessionResult ServoSession::pause()
{
  SessionResult result;

  bool refused = false;
  ServoState state_before{};
  host_.under_lock([&] {
      state_before = host_.sm().state();
      if (state_before != ServoState::TRACKING ||
        !host_.sm().handle(ServoEvent::PAUSE))
      {
        refused = true;
        return;
      }
      // A human/action pause must never trigger an automatic move.
      host_.enter_hold(HoldReason::kOperatorPause);
      host_.mark_dirty();
    });

  if (refused) {
    result.message = std::string("pause refused in state ") +
      state_name(state_before);
    host_.log_warn(result.message);
    return result;
  }

  // R10: keep control (HOLD), but stop Servo so MoveGroup can run again.
  const RpcOutcome outcome = host_.rpc(ServoRpc::kPause, "pause_servo");
  host_.under_lock([&] {
      if (!outcome.ok) {
        host_.sm().handle(ServoEvent::FAULT);
        result.success = false;
        result.message = outcome.detail;
        return;
      }
      result.success = true;
      result.message = "hold";
    });

  if (result.success) {
    resume_seed_attempts_ = 0;   // 新一次 HOLD: 播种重试预算重新计
    host_.log_info("pause: TRACKING -> HOLD");
  } else {
    host_.log_error("pause failed: " + outcome.detail + " -- state -> FAULT");
  }
  return result;
}

SessionResult ServoSession::resume()
{
  SessionResult result;

  bool refused = false;
  ServoState state_before{};
  bool target_ok = false;
  host_.under_lock([&] {
      state_before = host_.sm().state();
      target_ok = host_.sm().target_ok();
      refused = state_before != ServoState::HOLD || !target_ok;
    });

  if (refused) {
    if (state_before != ServoState::HOLD) {
      result.message = std::string("resume refused in state ") +
        state_name(state_before);
    } else {
      // HOLD + RESUME needs a fresh TARGET_OK latch (armed by the control
      // tick while a usable target exists).
      result.message = "no usable target (TARGET_OK latch not armed)";
    }
    host_.log_warn("resume refused: " + result.message);
    return result;
  }

  // HOLD may have involved a MoveGroup move (operator pause + reposition, or
  // the recovery move). Servo's last_sent_command_ is still the pre-move pose,
  // and the first all-zero twist after unpause replays it verbatim -- the arm
  // would jump back. Re-run start_servo first so the cache is seeded from the
  // CURRENT pose. ServoCalcs::start() is safe to call again (spec §7.9), and
  // it leaves Servo unpaused.
  // 臂还在明显移动时先不播种。verify_seed 拿的 `expected` 是调用前的快照,
  // 而零指令探针要等最多 300 ms 才回来 —— HOLD 时 follow:=true 的实测臂还在
  // 收敛(约 1 s),这 300 ms 里臂真的在动,"臂在动"就被记成"播种没生效",
  // 一次就打到 FAULT(2026-11-16 真机开车跟踪时踩到)。这里只做一次廉价判断,
  // 不阻塞;下一个 tick 再试,不消耗尝试次数。
  {
    constexpr double kQuasiStaticDeltaRad = 0.003;   // ~0.03 rad/s @100 Hz
    std::array<double, 6> q_now{};
    host_.under_lock([&] {q_now = host_.measured_positions();});
    bool quasi_static = true;
    if (resume_prev_valid_) {
      double dq = 0.0;
      for (std::size_t i = 0; i < q_now.size(); ++i) {
        dq = std::max(dq, std::fabs(q_now[i] - resume_prev_q_[i]));
      }
      quasi_static = dq < kQuasiStaticDeltaRad;
    }
    resume_prev_q_ = q_now;
    resume_prev_valid_ = true;
    if (!quasi_static) {
      result.message = "arm still settling; resume deferred";
      return result;
    }
  }

  const RpcOutcome reseed = host_.rpc(ServoRpc::kReseed, "start_servo(resume)");
  if (!reseed.ok) {
    host_.rpc(ServoRpc::kPause, "pause_servo");
    host_.under_lock([&] {host_.sm().handle(ServoEvent::FAULT);});
    result.message = reseed.detail;
    host_.log_error(
      "resume reseed failed: " + reseed.detail + " -- state -> FAULT");
    return result;
  }

  // start_servo's response is always success (spec §7.9), so prove the seed
  // while the node is still HOLD: the control law is silent and the
  // zero-twist probe is deterministic.
  // 容差 0.10 -> 0.15 且允许有限次重试: 瞬态抖动不该一次就打死任务。
  std::array<double, 6> expected{};
  host_.under_lock([&] {expected = host_.measured_positions();});
  std::string probe_error;
  if (!host_.verify_seed(expected, 0.15, probe_error)) {
    // start_servo 已经把 Servo unpause 了, 推迟时必须先把它停回去。
    host_.rpc(ServoRpc::kPause, "pause_servo");
    constexpr int kMaxSeedAttempts = 10;
    if (++resume_seed_attempts_ < kMaxSeedAttempts) {
      result.message = probe_error + " (retry " +
        std::to_string(resume_seed_attempts_) + "/" +
        std::to_string(kMaxSeedAttempts) + ")";
      host_.log_warn("resume reseed deferred: " + result.message);
      return result;   // 保持 HOLD, 下一个 tick 再试
    }
    host_.under_lock([&] {host_.sm().handle(ServoEvent::FAULT);});
    result.message = probe_error;
    host_.log_error(
      "resume reseed check failed: " + probe_error + " -- state -> FAULT");
    return result;
  }
  resume_seed_attempts_ = 0;

  // Commit the transition only now: a release may have arrived during the
  // blocking calls.
  bool transitioned = false;
  host_.under_lock([&] {
      transitioned = host_.sm().state() == ServoState::HOLD &&
        host_.sm().target_ok() && host_.sm().handle(ServoEvent::RESUME);
      if (transitioned) {
        host_.arm_saturation_window();
        host_.mark_dirty();
      }
    });

  if (!transitioned) {
    host_.rpc(ServoRpc::kPause, "pause_servo");
    ServoState state_now{};
    host_.under_lock([&] {state_now = host_.sm().state();});
    result.message = "resume cancelled (session state changed)";
    host_.log_warn(
      "resume cancelled: " + std::string(state_name(state_now)));
    return result;
  }

  const RpcOutcome outcome = host_.rpc(ServoRpc::kUnpause, "unpause_servo");
  host_.under_lock([&] {
      if (!outcome.ok) {
        host_.sm().handle(ServoEvent::FAULT);
        result.success = false;
        result.message = outcome.detail;
        return;
      }
      result.success = true;
      result.message = "tracking";
    });

  if (result.success) {
    host_.log_info("resume: HOLD -> TRACKING (cache re-seeded)");
  } else {
    host_.log_error(
      "resume failed: " + outcome.detail + " -- state -> FAULT");
  }
  return result;
}

}  // namespace lgw_bt_planner
