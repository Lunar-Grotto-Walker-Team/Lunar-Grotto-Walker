#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace lgw_bt_planner
{

/// One SRDF <group_state>:
///   <group_state name="X" group="Y"> <joint name="J" value="V"/> ... </group_state>
/// Captures name → group + (joint → value) map so the caller can route the
/// MoveGroup goal to the correct planning group ("arm" vs "gripper" vs "hand").
struct NamedTarget
{
  std::string group;                       // e.g. "arm", "gripper", "hand"
  std::map<std::string, double> joints;    // joint_name → value
};

/// name → NamedTarget. Pure std::regex, no XML lib.
using TargetTable = std::map<std::string, NamedTarget>;

class NamedTargetLoader
{
public:
  /// Read agx_arm_moveit/config/agx_arm.srdf.xacro from the share directory
  /// (NOT the stale .srdf artifact — see named_target_loader.cpp for why).
  ///
  /// The xacro is a template with one branch per arm/effector type; the branch
  /// conditions are evaluated against the three arguments (same names and
  /// defaults as the xacro itself: arm_type=piper, effector_type=none,
  /// revo2_type=left) so that only the active branch's group_states land in the
  /// table. Without this, branches clobber each other by name — `home` from the
  /// nero branch (7 joints) used to overwrite the piper one (6 joints).
  ///
  /// Throws std::runtime_error if file cannot be read.
  static TargetTable load_from_share(
    const std::string& arm_type = "piper",
    const std::string& effector_type = "none",
    const std::string& revo2_type = "left");

  /// Look up joint values for `target` in the given order of `joint_names`.
  /// Missing joint → 0.0 (matches "home" all-zero fallback).
  /// Missing target → throws std::runtime_error.
  static std::vector<double> get_joint_values(
    const TargetTable& table,
    const std::string& target,
    const std::vector<std::string>& joint_names);
};

}  // namespace lgw_bt_planner