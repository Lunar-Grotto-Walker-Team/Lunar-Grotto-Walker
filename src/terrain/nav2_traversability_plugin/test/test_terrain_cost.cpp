// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "nav2_traversability_plugin/terrain_cost.hpp"

using nav2_traversability_plugin::terrainByteToCost;
using nav2_traversability_plugin::UnknownPolicy;
using nav2_traversability_plugin::unknownPolicyFromString;

namespace
{

TEST(TerrainCost, FreeByteStaysFree)
{
  const auto result = terrainByteToCost(0u);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, nav2_costmap_2d::FREE_SPACE);
}

TEST(TerrainCost, LethalByteBecomesLethalObstacle)
{
  // 254 read as int8 would be -2 and silently degrade to FREE.
  const auto result = terrainByteToCost(254u);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TerrainCost, SoftByteKeepsItsCost)
{
  // 128 read as int8 would be -128 and silently degrade to FREE.
  const auto result = terrainByteToCost(128u);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, 128u);
  EXPECT_LT(result.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TerrainCost, UnknownByteIsNotLethalByDefault)
{
  // 85 must NOT become a lethal obstacle (that was the inverted behaviour).
  // Default policy: write NO_INFORMATION -> the costmap treats the cell as
  // unknown space instead of silently calling it traversable.
  const auto result = terrainByteToCost(85u);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_NE(result.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TerrainCost, UnknownByteCanBeSkipped)
{
  const auto result = terrainByteToCost(85u, UnknownPolicy::kSkip);
  EXPECT_FALSE(result.has_cost);
}

TEST(TerrainCost, UnknownByteCanBeLethal)
{
  const auto result = terrainByteToCost(85u, UnknownPolicy::kLethal);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TerrainCost, NoInformationByteFollowsThePolicy)
{
  EXPECT_EQ(
    terrainByteToCost(255u).cost, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(terrainByteToCost(255u, UnknownPolicy::kSkip).has_cost);
  EXPECT_EQ(
    terrainByteToCost(255u, UnknownPolicy::kLethal).cost,
    nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(TerrainCost, UnknownPolicyFromString)
{
  EXPECT_EQ(
    unknownPolicyFromString("no_information"), UnknownPolicy::kNoInformation);
  EXPECT_EQ(unknownPolicyFromString("skip"), UnknownPolicy::kSkip);
  EXPECT_EQ(unknownPolicyFromString("lethal"), UnknownPolicy::kLethal);
  // Unknown values fall back to the safe default.
  EXPECT_EQ(unknownPolicyFromString("bogus"), UnknownPolicy::kNoInformation);
}

TEST(TerrainCost, SoftBandValueIsPassedThrough)
{
  const auto result = terrainByteToCost(200u);
  EXPECT_TRUE(result.has_cost);
  EXPECT_EQ(result.cost, 200u);
}

}  // namespace
