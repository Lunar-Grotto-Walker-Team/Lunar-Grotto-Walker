// tests/test_lookat_mode.cpp
//
// Task 3a of the 2026-11-07 node-split plan: the look-at mode becomes a type.
// The old code expressed it as two bools, which is why adding the third mode
// left the start-up banner silently wrong.

#include <gtest/gtest.h>

#include "lgw_bt_planner/servo/lookat_mode.hpp"

using lgw_bt_planner::LookatMode;
using lgw_bt_planner::lookat_mode_name;
using lgw_bt_planner::parse_lookat_mode;

TEST(LookatMode, ParsesTheThreeShippedValues)
{
  EXPECT_EQ(parse_lookat_mode("legacy"), LookatMode::kLegacy);
  EXPECT_EQ(parse_lookat_mode("target_relative"), LookatMode::kTargetRelative);
  EXPECT_EQ(parse_lookat_mode("pinned_posture"), LookatMode::kPinnedPosture);
}

TEST(LookatMode, UnknownValueFallsBackToLegacy)
{
  // A typo in the parameter must not take the arm offline.
  EXPECT_EQ(parse_lookat_mode("nonsense"), LookatMode::kLegacy);
  EXPECT_EQ(parse_lookat_mode(""), LookatMode::kLegacy);
}

TEST(LookatMode, NameRoundTripsWithParse)
{
  for (const auto mode : {LookatMode::kLegacy, LookatMode::kTargetRelative,
      LookatMode::kPinnedPosture})
  {
    EXPECT_EQ(parse_lookat_mode(lookat_mode_name(mode)), mode);
  }
}

TEST(LookatMode, NamesMatchTheParameterValuesTheLaunchFileUses)
{
  EXPECT_STREQ(lookat_mode_name(LookatMode::kLegacy), "legacy");
  EXPECT_STREQ(lookat_mode_name(LookatMode::kTargetRelative), "target_relative");
  EXPECT_STREQ(lookat_mode_name(LookatMode::kPinnedPosture), "pinned_posture");
}
