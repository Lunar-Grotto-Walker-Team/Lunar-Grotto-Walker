// include/lgw_bt_planner/servo/posture_guard.hpp
//
// The posture family answers with two different kinds of pose, and only one of
// them is safe to command unconditionally:
//
//   ok == true   a legal pose that passed every guard. Its jumps are the
//                deliberate wrist-branch switches, which the design feeds
//                live (servo_node.hpp explains why).
//   ok == false  a BEST-EFFORT pose: "no candidate centred the target, here
//                is the iterate that came closest". Across the wrist band that
//                iterate can sit on the opposite branch, ~1.4 rad from the
//                pose being commanded.
//
// Publishing the second kind whenever it appears makes the arm swing between
// two candidates. Measured on the real machine 2026-11-07 with a STATIONARY
// target near the arm base: j5 alternated +0.851 <-> -0.600 as the solver
// alternated legal / best-effort, 24 branch crossings while the target's own
// speed was under 0.10 m/s.
//
// So: a best-effort pose is published only while it is continuous with what
// was last commanded. Anything further is a HOLD — the recovery path and the
// operator decide what to do next, rather than the control law chasing a
// pose nobody believes.

#ifndef LGW_BT_PLANNER__SERVO__POSTURE_GUARD_HPP_
#define LGW_BT_PLANNER__SERVO__POSTURE_GUARD_HPP_

#include <array>
#include <cmath>
#include <cstddef>

namespace lgw_bt_planner
{

// Largest per-tick joint step a best-effort pose may ask for, radians.
// A normal tracking tick moves well under 0.1 rad; a branch jump is 1.2-1.5.
// 0.25 sits above honest motion and far below the jump it exists to stop.
inline constexpr double kBestEffortMaxDeltaRad = 0.25;

// True when every joint of `candidate` is within `max_delta_rad` of
// `last_sent`. A never-commanded arm has no reference and is rejected by the
// caller (last_sent_joints_valid == false), not here.
inline bool posture_continuous(
  const std::array<double, 6> & candidate,
  const std::array<double, 6> & last_sent,
  double max_delta_rad)
{
  for (std::size_t i = 0; i < candidate.size(); ++i) {
    if (!std::isfinite(candidate[i]) ||
      std::fabs(candidate[i] - last_sent[i]) > max_delta_rad)
    {
      return false;
    }
  }
  return true;
}

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__POSTURE_GUARD_HPP_
