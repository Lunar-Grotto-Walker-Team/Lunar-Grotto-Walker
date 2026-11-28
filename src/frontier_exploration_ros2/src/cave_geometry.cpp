/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2/cave_geometry.hpp"

#include <cmath>

namespace frontier_exploration_ros2
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

CaveGeometry::CaveGeometry(EntranceConstraint constraint)
: constraint_(constraint)
{
}

void CaveGeometry::set_entrance(double x, double y, double heading)
{
  entrance_.x = x;
  entrance_.y = y;
  entrance_.heading = heading;
  entrance_.valid = true;
}

void CaveGeometry::clear()
{
  entrance_ = EntranceGeometry{};
}

bool CaveGeometry::has_entrance() const
{
  return entrance_.valid;
}

const EntranceGeometry & CaveGeometry::entrance() const
{
  return entrance_;
}

bool CaveGeometry::is_within_entrance_radius(double x, double y) const
{
  if (!entrance_.valid) {
    return false;
  }
  const double dx = x - entrance_.x;
  const double dy = y - entrance_.y;
  return std::hypot(dx, dy) < constraint_.radius_m;
}

bool CaveGeometry::is_outside_cave(double x, double y) const
{
  if (!entrance_.valid) {
    return false;
  }
  if (is_within_entrance_radius(x, y)) {
    return true;
  }
  if (!constraint_.gate_enabled) {
    return false;
  }
  const double dot =
    (x - entrance_.x) * std::cos(entrance_.heading) +
    (y - entrance_.y) * std::sin(entrance_.heading);
  return dot < -constraint_.gate_margin_m;
}

}  // namespace frontier_exploration_ros2
