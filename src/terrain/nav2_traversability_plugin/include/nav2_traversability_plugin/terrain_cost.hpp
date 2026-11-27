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

#ifndef NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_COST_HPP_
#define NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_COST_HPP_

#include <cstdint>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_traversability_plugin
{

/// Raw terrain bytes produced by `elevation_traversability_bridge`.
/// They follow the Nav2 Costmap2D-internal convention (see the bridge
/// README), NOT the nav_msgs/OccupancyGrid occupancy convention.
constexpr uint8_t TERRAIN_COST_FREE = 0u;
/// No terrain information for that cell (never observed by the lidar).
constexpr uint8_t TERRAIN_COST_UNKNOWN = 85u;
/// Passable, but penalized (warning band).
constexpr uint8_t TERRAIN_COST_SOFT = 128u;
/// Impassable.
constexpr uint8_t TERRAIN_COST_LETHAL = 254u;

/// Outcome of mapping a raw terrain byte onto a Nav2 Costmap2D cost.
struct TerrainCost
{
  /// false -> the byte carries no terrain information; the cell must be
  /// left untouched so the other costmap layers decide its cost.
  bool has_cost;
  /// Nav2 Costmap2D cost. Only meaningful when has_cost is true.
  unsigned char cost;
};

/// What to do with a cell that has no terrain information (the bridge marks
/// never-observed cells as UNKNOWN).
enum class UnknownPolicy
{
  /// Write NO_INFORMATION (255): the costmap treats the cell as unknown
  /// space, so a planner will not route through it. This is the default:
  /// "not observed" must not silently become "traversable".
  kNoInformation,
  /// Leave the cell untouched (the other costmap layers decide). Useful when
  /// the unobserved area is huge and blocking it would starve the planner.
  kSkip,
  /// Write LETHAL_OBSTACLE (254): treat unobserved terrain as an obstacle.
  kLethal,
};

/// Parse the `unknown_policy` parameter value.
inline UnknownPolicy unknownPolicyFromString(const std::string & value)
{
  if (value == "skip") {
    return UnknownPolicy::kSkip;
  }
  if (value == "lethal") {
    return UnknownPolicy::kLethal;
  }
  return UnknownPolicy::kNoInformation;
}

/// Map a raw terrain byte onto the cost to write into the master grid.
///
/// The byte has to be read as UNSIGNED. `nav_msgs/OccupancyGrid.data` is
/// declared `int8[]`, but the bridge writes Costmap2D-internal bytes
/// (0 / 85 / 128 / 254). Reading them as signed int8 turns 254 into -2 and
/// 128 into -128, so every lethal / soft cell silently degrades to FREE,
/// while the UNKNOWN marker 85 becomes ">= 65" and turns into a lethal
/// obstacle - exactly inverted.
inline TerrainCost terrainByteToCost(
  uint8_t raw, UnknownPolicy unknown_policy = UnknownPolicy::kNoInformation)
{
  if (raw == TERRAIN_COST_FREE) {
    return {true, nav2_costmap_2d::FREE_SPACE};
  }
  if (raw == TERRAIN_COST_UNKNOWN || raw == nav2_costmap_2d::NO_INFORMATION) {
    // No terrain information for this cell.
    switch (unknown_policy) {
      case UnknownPolicy::kSkip:
        return {false, 0u};
      case UnknownPolicy::kLethal:
        return {true, nav2_costmap_2d::LETHAL_OBSTACLE};
      case UnknownPolicy::kNoInformation:
      default:
        return {true, nav2_costmap_2d::NO_INFORMATION};
    }
  }
  if (raw >= TERRAIN_COST_LETHAL) {
    return {true, nav2_costmap_2d::LETHAL_OBSTACLE};
  }
  // Soft band (e.g. 128): keep the value so the planner sees the cell as
  // passable but costly instead of silently dropping it to FREE.
  return {true, static_cast<unsigned char>(raw)};
}

}  // namespace nav2_traversability_plugin

#endif  // NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_COST_HPP_
