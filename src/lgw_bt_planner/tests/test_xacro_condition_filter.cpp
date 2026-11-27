// test_xacro_condition_filter.cpp — branch selection for the SRDF template.
//
// Regression (2026-11-17): NamedTargetLoader regex-parsed agx_arm.srdf.xacro as
// plain text, so every arm-type branch contributed its group_states and the
// nero `home` (7 joints, incl. joint7) overwrote the piper `home` (6 joints).
// The BT then commanded a joint the real arm does not have.

#include <gtest/gtest.h>

#include <string>

#include "lgw_bt_planner/xacro_condition_filter.hpp"

using lgw_bt_planner::eval_xacro_condition;
using lgw_bt_planner::strip_inactive_xacro_blocks;

namespace
{
// Raw-string delimiter is XACRO, not the default, because the xacro attribute
// value ends with `)"` — which would otherwise close a default-delimited
// raw string literal.
const char* kPiperBlock = R"XACRO(
  <xacro:unless value="$(eval arm_type == 'nero')">
    <group_state name="home" group="arm">
      <joint name="joint1" value="0" />
      <joint name="joint6" value="0" />
    </group_state>
  </xacro:unless>
  <xacro:if value="$(eval arm_type == 'nero')">
    <group_state name="home" group="arm">
      <joint name="joint1" value="0" />
      <joint name="joint7" value="0" />
    </group_state>
  </xacro:if>
)XACRO";
}  // namespace

TEST(XacroCondition, EqualityAndInequality)
{
  EXPECT_TRUE(eval_xacro_condition("$(eval arm_type == 'piper')", "piper", "none", "left"));
  EXPECT_FALSE(eval_xacro_condition("$(eval arm_type == 'nero')", "piper", "none", "left"));
  EXPECT_TRUE(eval_xacro_condition("$(eval arm_type != 'nero')", "piper", "none", "left"));
}

TEST(XacroCondition, AndChain)
{
  EXPECT_TRUE(eval_xacro_condition(
    "$(eval effector_type == 'revo2' and revo2_type == 'left')", "piper", "revo2", "left"));
  EXPECT_FALSE(eval_xacro_condition(
    "$(eval effector_type == 'revo2' and revo2_type == 'right')", "piper", "revo2", "left"));
}

TEST(XacroCondition, UnknownOrUnsupportedFailsClosed)
{
  EXPECT_FALSE(eval_xacro_condition("$(eval unknown_thing == 'x')", "piper", "none", "left"));
  EXPECT_FALSE(eval_xacro_condition("$(eval arm_type == 'piper' or 1)", "piper", "none", "left"));
  EXPECT_FALSE(eval_xacro_condition("$(eval 1 == 1)", "piper", "none", "left"));
  EXPECT_FALSE(eval_xacro_condition("arm_type == 'piper'", "piper", "none", "left"));
}

TEST(XacroCondition, PiperBranchKeepsOnlyNonNeroStates)
{
  const std::string out =
    strip_inactive_xacro_blocks(kPiperBlock, "piper", "agx_gripper", "left");
  EXPECT_NE(out.find("joint6"), std::string::npos) << out;
  EXPECT_EQ(out.find("joint7"), std::string::npos) << out;
  // The conditional tags themselves are stripped.
  EXPECT_EQ(out.find("xacro:"), std::string::npos) << out;
  EXPECT_EQ(out.find("home"), out.rfind("home")) << out;  // exactly one state
}

TEST(XacroCondition, NeroBranchKeepsOnlyNeroStates)
{
  const std::string out =
    strip_inactive_xacro_blocks(kPiperBlock, "nero", "agx_gripper", "left");
  EXPECT_NE(out.find("joint7"), std::string::npos) << out;
  EXPECT_EQ(out.find("joint6"), std::string::npos) << out;
}

TEST(XacroCondition, NestedBlocksRequireAllEnclosingConditions)
{
  const std::string nested =
    "<xacro:if value=\"$(eval effector_type == 'agx_gripper')\">\n"
    "  <group_state name=\"gripper_open\" group=\"gripper\">\n"
    "    <joint name=\"gripper\" value=\"0.1\" />\n"
    "  </group_state>\n"
    "  <xacro:if value=\"$(eval arm_type == 'nero')\">\n"
    "    <group_state name=\"bad\" group=\"arm\"><joint name=\"joint7\" value=\"0\" /></group_state>\n"
    "  </xacro:if>\n"
    "</xacro:if>\n";

  const std::string out =
    strip_inactive_xacro_blocks(nested, "piper", "agx_gripper", "left");
  EXPECT_NE(out.find("gripper_open"), std::string::npos) << out;
  EXPECT_EQ(out.find("bad"), std::string::npos) << out;

  const std::string none_out =
    strip_inactive_xacro_blocks(nested, "piper", "none", "left");
  EXPECT_EQ(none_out.find("gripper_open"), std::string::npos) << none_out;
}

TEST(XacroCondition, TextOutsideConditionalsIsPreserved)
{
  const std::string src =
    "<?xml version=\"1.0\"?>\n<robot name=\"x\">\n"
    "  <group name=\"arm\"><chain base_link=\"base_link\" tip_link=\"tcp_link\"/></group>\n"
    "  <xacro:if value=\"$(eval arm_type == 'nero')\"><group_state name=\"x\"/></xacro:if>\n"
    "</robot>\n";
  const std::string out = strip_inactive_xacro_blocks(src, "piper", "none", "left");
  EXPECT_NE(out.find("<chain base_link=\"base_link\" tip_link=\"tcp_link\"/>"), std::string::npos)
    << out;
  EXPECT_EQ(out.find("group_state"), std::string::npos) << out;
}
