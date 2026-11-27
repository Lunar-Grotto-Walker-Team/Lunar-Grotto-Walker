// include/lgw_bt_planner/servo/servo_session.hpp
//
// Task 8 of the 2026-11-07 node-split plan (spec step S5). The release / pause
// / resume services were ~200 lines of sequencing inside the node: lock, take
// a state-machine event, UNLOCK, make a blocking Servo call, lock again,
// re-check that nothing happened while the lock was open, commit. That
// sequence is the behaviour, and it could only be tested by running a robot.
//
// ServoSession owns the sequence. It cannot own the things the sequence acts
// on -- the state machine, the control mutex and the Servo clients all belong
// to the node and are shared with the 100 Hz tick -- so it takes a SessionHost
// and calls back through it. The node's service handlers shrink to a
// delegation.
//
// What deliberately did NOT move: takeover. It drives the pre-position move
// and the target selection, which are the node's own job (spec §9 keeps
// posture_/qp_solver_/prepositioner_ in the node), and its sequencing is
// different in kind: it blocks on a MoveGroup action, not on a Servo call.

#ifndef LGW_BT_PLANNER__SERVO__SERVO_SESSION_HPP_
#define LGW_BT_PLANNER__SERVO__SERVO_SESSION_HPP_

#include <array>
#include <functional>
#include <string>

#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/servo_state_machine.hpp"

namespace lgw_bt_planner
{

// The Servo services a session drives. Each one BLOCKS and must therefore be
// called with the control mutex released.
enum class ServoRpc
{
  kPause,
  kUnpause,
  kReseed,
};

struct RpcOutcome
{
  bool ok{false};
  std::string detail;
};

// The node's side of the contract. Everything here is already owned by the
// node and shared with the control tick; the session never holds a copy.
class SessionHost
{
public:
  virtual ~SessionHost() = default;

  // Run `critical` under the control mutex. The session must never hold that
  // mutex across a ServoRpc call, which is why this takes a closure instead
  // of a lock/unlock pair.
  virtual void under_lock(const std::function<void()> & critical) = 0;

  virtual ServoStateMachine & sm() = 0;

  // Node-side actions that conceptually belong to the session.
  virtual void enter_hold(HoldReason reason) = 0;
  virtual void clear_session() = 0;
  virtual void arm_saturation_window() = 0;
  virtual void mark_dirty() = 0;

  // Stop a takeover pre-position that is still in flight, but leave a HOLD
  // recovery move alone (it is bounded and finishes at a known posture).
  virtual void cancel_preposition_unless_recovery() = 0;
  // Tell the recovery worker not to re-seed or unpause once the session has
  // left RECOVERING.
  virtual void arm_recovery_cancel() = 0;

  virtual std::array<double, 6> measured_positions() = 0;
  virtual bool verify_seed(
    const std::array<double, 6> & expected, double tolerance,
    std::string & error) = 0;

  virtual RpcOutcome rpc(ServoRpc which, const char * tag) = 0;

  virtual void log_info(const std::string & message) = 0;
  virtual void log_warn(const std::string & message) = 0;
  virtual void log_error(const std::string & message) = 0;
};

struct SessionResult
{
  bool success{false};
  std::string message;
};

class ServoSession
{
public:
  explicit ServoSession(SessionHost & host);

  // Idempotent: a second release in IDLE / RELEASING answers success without
  // pausing Servo again.
  SessionResult release();
  // Only from TRACKING. Enters HOLD with kOperatorPause, so an automated
  // recovery can never be triggered by a human pause.
  SessionResult pause();
  // Only from HOLD with the TARGET_OK latch armed. Re-seeds Servo's command
  // cache and proves the seed before committing the transition.
  SessionResult resume();

private:
  SessionHost & host_;
  // HOLD -> TRACKING 播种校验的重试状态(见 resume())。
  // HOLD 时 follow:=true 的实测臂还在收敛, verify_seed 的快照会过期,
  // 于是"臂在动"被误判成"播种失败"直接 FAULT(2026-11-16 真机开车跟踪)。
  // 臂还明显在动就先跳过, 下一个 tick 再试。
  std::array<double, 6> resume_prev_q_{};
  bool resume_prev_valid_{false};
  int resume_seed_attempts_{0};
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SERVO_SESSION_HPP_
