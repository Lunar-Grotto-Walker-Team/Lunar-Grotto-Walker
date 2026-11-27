// include/lgw_bt_planner/servo/lookat_mode.hpp
//
// The three shipped look-at modes, as a type. Before this they were two bools
// (lookat_target_relative_ / lookat_pinned_posture_), and adding the third one
// silently left the start-up banner wrong (2026-10-31).

#ifndef LGW_BT_PLANNER__SERVO__LOOKAT_MODE_HPP_
#define LGW_BT_PLANNER__SERVO__LOOKAT_MODE_HPP_

#include <string>

namespace lgw_bt_planner
{

enum class LookatMode
{
  kLegacy,          // arm-relative stand-off (pre-2026-10-30 behaviour)
  kTargetRelative,  // stand-off along the target bearing
  kPinnedPosture,   // pinned-posture family (QP / DLS)
};

// Unknown values fall back to kLegacy, matching the parameter handler: a typo
// must not take the arm offline.
LookatMode parse_lookat_mode(const std::string & name);

// The parameter value that produces this mode (for the start-up banner and
// diagnostics). Round-trips with parse_lookat_mode for all three.
const char * lookat_mode_name(LookatMode mode);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__LOOKAT_MODE_HPP_
