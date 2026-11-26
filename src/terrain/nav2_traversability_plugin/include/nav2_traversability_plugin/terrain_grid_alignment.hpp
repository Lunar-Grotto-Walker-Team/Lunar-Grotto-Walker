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

#ifndef NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_GRID_ALIGNMENT_HPP_
#define NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_GRID_ALIGNMENT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_traversability_plugin
{

namespace
{
/// Floor() a cell coordinate, snapping values that sit (numerically) exactly
/// on a cell boundary to that boundary.
///
/// Without the snap, a point that is exactly on a cell edge can fall into
/// either of the two neighbouring cells depending on floating point noise:
/// 0.3 / 0.1 == 2.9999999999999996 -> floor -> 2 instead of 3, and with a
/// rotated grid even cos(pi/2) == 6.1e-17 leaks into the result.
inline int cellIndex(double v)
{
  const double nearest = std::round(v);
  if (std::fabs(v - nearest) < 1e-6) {
    return static_cast<int>(nearest);
  }
  return static_cast<int>(std::floor(v));
}
}  // namespace

/// 2D alignment between an incoming `nav_msgs/OccupancyGrid` and the costmap's
/// global frame.
///
/// The OccupancyGrid carries its own frame (usually "map", but it may be
/// anything). The costmap works in its `global_frame`. This helper stores the
/// 2D rigid transform between the two so that
///
///   world -> grid cell index
///
/// is a single, well defined operation instead of the naive "subtract the
/// origin and divide by the resolution", which is only correct when both
/// frames are axis aligned with each other.
struct GridAlignment
{
  /// Grid origin expressed in the costmap global frame.
  double origin_x{0.0};
  double origin_y{0.0};
  /// cos / sin of the grid yaw relative to the global frame.
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  /// Cell size in meters.
  double resolution{0.1};
  /// False until setFromTransform() succeeded.
  bool valid{false};

  /// Set the alignment from a rigid 2D transform (translation + yaw).
  /// Returns false (and leaves the object invalid) for a non-positive
  /// resolution, which would make the world->grid mapping undefined.
  bool setFromTransform(double tx, double ty, double yaw, double res)
  {
    if (!(res > 0.0)) {
      valid = false;
      return false;
    }
    origin_x = tx;
    origin_y = ty;
    cos_yaw = std::cos(yaw);
    sin_yaw = std::sin(yaw);
    resolution = res;
    valid = true;
    return true;
  }

  /// Convert a point in the global frame into grid cell indices.
  /// Returns false when the alignment has not been initialized.
  bool worldToGrid(double wx, double wy, int & mx, int & my) const
  {
    if (!valid) {
      return false;
    }
    const double dx = wx - origin_x;
    const double dy = wy - origin_y;
    // grid = R(-yaw) * (world - origin)
    const double gx = (cos_yaw * dx + sin_yaw * dy) / resolution;
    const double gy = (-sin_yaw * dx + cos_yaw * dy) / resolution;
    mx = cellIndex(gx);
    my = cellIndex(gy);
    return true;
  }

  /// Axis aligned bounding box (in the global frame) of the whole grid.
  void gridBounds(
    int width, int height, double & min_x, double & min_y,
    double & max_x, double & max_y) const
  {
    const double w = width * resolution;
    const double h = height * resolution;
    // Grid corners in grid coordinates -> global frame.
    const double corners_x[4] = {0.0, w, 0.0, w};
    const double corners_y[4] = {0.0, 0.0, h, h};
    min_x = min_y = std::numeric_limits<double>::max();
    max_x = max_y = -std::numeric_limits<double>::max();
    for (int i = 0; i < 4; ++i) {
      const double x = origin_x + cos_yaw * corners_x[i] - sin_yaw * corners_y[i];
      const double y = origin_y + sin_yaw * corners_x[i] + cos_yaw * corners_y[i];
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  }
};

}  // namespace nav2_traversability_plugin

#endif  // NAV2_TRAVERSABILITY_PLUGIN__TERRAIN_GRID_ALIGNMENT_HPP_
