// src/servo/lookat_mode.cpp

#include "lgw_bt_planner/servo/lookat_mode.hpp"

namespace lgw_bt_planner
{

LookatMode parse_lookat_mode(const std::string & name)
{
  if (name == "target_relative") {
    return LookatMode::kTargetRelative;
  }
  if (name == "pinned_posture") {
    return LookatMode::kPinnedPosture;
  }
  return LookatMode::kLegacy;
}

const char * lookat_mode_name(LookatMode mode)
{
  switch (mode) {
    case LookatMode::kTargetRelative:
      return "target_relative";
    case LookatMode::kPinnedPosture:
      return "pinned_posture";
    case LookatMode::kLegacy:
    default:
      return "legacy";
  }
}

}  // namespace lgw_bt_planner
