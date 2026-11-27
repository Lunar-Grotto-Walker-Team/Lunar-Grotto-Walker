// include/lgw_bt_planner/servo/recovery_orchestrator.hpp
//
// Task 7 of the 2026-11-07 node-split plan (spec step S4). "Should we start a
// HOLD recovery right now, and against what?" was spread over
// maybe_start_recovery_locked plus three node members that remembered the
// answer (attempts, exhausted, last-attempt time). It is now a pure decision
// plus the session's own memory.
//
// What deliberately did NOT move: the recovery WORKER. Its thread, its
// blocking MoveGroup call, the Servo re-seed and the zero-twist probe stay in
// the node (spec §9, D8 — "保留现有线程与阻塞结构"). The orchestrator only
// decides; the node executes.

#ifndef LGW_BT_PLANNER__SERVO__RECOVERY_ORCHESTRATOR_HPP_
#define LGW_BT_PLANNER__SERVO__RECOVERY_ORCHESTRATOR_HPP_

#include <cstdint>
#include <string>

#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/servo/servo_params.hpp"
#include "lgw_bt_planner/servo/tracking_loop.hpp"
#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// Everything the decision needs, gathered by the node (this is where the TF
// lookups and the solver call happen).
struct RecoveryInput
{
  // The reason the HOLD was entered. A snapshot: if the target has since
  // vanished the decision routes through the target-lost gate instead.
  HoldReason reason{HoldReason::kNone};
  int64_t now_ns{0};
  int64_t hold_since_ns{0};
  bool target_ok{true};
  bool move_group_busy{false};
  bool measured_valid{false};
  bool tcp_valid{false};
  bool geometry_available{false};
  int64_t geometry_age_ms{0};
  int64_t since_last_attempt_ms{-1};
  // Whether this configuration HAS a posture family to ask. It is initialised
  // only in pinned_posture mode (servo_node.hpp startup block); in legacy /
  // target_relative the family does not participate in the setpoint at all,
  // so reachability is not a meaningful gate there.
  bool family_applicable{true};
  // Can the posture family serve the current target? Asked BEFORE the budget
  // so an unreachable target never consumes the single attempt.
  bool family_ok{true};
  // Target in arm base_link, in case the plan asks for a move.
  tf2::Vector3 target{0, 0, 0};
};

struct RecoveryPlan
{
  bool needed{false};
  // Where the worker should aim. The joints are solved by the worker against
  // the live target (the plan's `joints` field would be stale by the time the
  // thread ran).
  tf2::Vector3 target{0, 0, 0};
  // Why, for the caller's log. Empty when a move was requested.
  std::string detail;
};

class RecoveryOrchestrator
{
public:
  // Static because the decision has no state of its own: everything it
  // remembers lives in SessionState and everything it reads comes in.
  static RecoveryPlan decide(
    const RecoveryInput & in, const ServoParams & params, SessionState & st);
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__RECOVERY_ORCHESTRATOR_HPP_
