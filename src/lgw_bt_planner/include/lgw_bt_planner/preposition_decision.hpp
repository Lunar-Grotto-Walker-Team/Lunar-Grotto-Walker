// include/lgw_bt_planner/preposition_decision.hpp
//
// "Does the arm need to be moved before Servo takes over?" (spec 2026-10-30
// §3.8 rev3).
//
// Pure function, no ROS: the node feeds it the pose the posture family wants
// for the current target and the pose the arm is actually in, and gets back a
// verdict plus the measurements behind it.
//
// Why it matters: at takeover the arm can be anywhere. If it sits in the other
// wrist branch (q5 < 0 while the family pins q5 = +0.6), the straight path
// Servo integrates runs through the wrist singularity — the exact failure this
// whole redesign exists to remove.

#ifndef LGW_BT_PLANNER__PREPOSITION_DECISION_HPP_
#define LGW_BT_PLANNER__PREPOSITION_DECISION_HPP_

#include <array>
#include <string>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

struct PrepositionParams
{
  bool enable{true};
  double joint_delta_rad{0.15};      // criterion 1: any joint further than this
  double branch_delta_rad{1.571};    // criterion 2: |dq5| beyond this = other branch
  double limit_margin_rad{0.10};     // criterion 3: closer than this to a limit
  double tcp_distance_m{0.05};       // criterion 4a
  double tcp_angle_rad{0.15};        // criterion 4b
};

struct PrepositionDecision
{
  bool needed{false};
  // "ok" / "joint_delta" / "branch" / "limit" / "tcp" / "disabled"
  std::string reason{"ok"};
  double max_joint_delta{0.0};
  double q5_delta{0.0};
  double min_limit_margin{0.0};
  double tcp_distance{0.0};
  double tcp_angle{0.0};
};

// Evaluate the four criteria. Always fills every measurement, so the node can
// log them even when the answer is "no move needed".
PrepositionDecision decide_preposition(
  const std::array<double, 6> & q_family,
  const std::array<double, 6> & q_now,
  const Pose6 & tcp_family,
  const Pose6 & tcp_now,
  const PrepositionParams & params);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__PREPOSITION_DECISION_HPP_
