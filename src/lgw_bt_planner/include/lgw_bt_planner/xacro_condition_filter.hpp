#pragma once

// xacro_condition_filter.hpp — drop inactive <xacro:if>/<xacro:unless> blocks
// from the raw xacro text before the named-target loader regex-parses it.
//
// Why (2026-11-17): NamedTargetLoader reads agx_arm.srdf.xacro as plain text,
// so every arm-type branch contributed its group_states. `home` exists once per
// branch and the nero branch (which has joint7) overwrote the piper one, so the
// BT's "home" carried a joint the real arm does not have. Different branches
// defining the same state name silently clobber each other.

#include <string>

namespace lgw_bt_planner
{

/// Evaluate the small subset of xacro conditions used by agx_arm.srdf.xacro:
///   $(eval arm_type == 'nero')
///   $(eval effector_type == 'agx_gripper')
///   $(eval effector_type == 'revo2' and revo2_type == 'left')
/// Supports `and` chains of `ident == 'value'` / `ident != 'value'`.
/// Anything else (unknown identifier, `or`, function calls, ...) evaluates to
/// false — fail closed, so an unparsed branch never leaks extra states in.
bool eval_xacro_condition(
  const std::string& expr,
  const std::string& arm_type,
  const std::string& effector_type,
  const std::string& revo2_type);

/// Return `content` with every inactive conditional block removed and the
/// xacro conditional tags themselves stripped. Text outside any conditional is
/// kept verbatim; everything inside an active block is kept recursively.
std::string strip_inactive_xacro_blocks(
  const std::string& content,
  const std::string& arm_type,
  const std::string& effector_type,
  const std::string& revo2_type);

}  // namespace lgw_bt_planner
