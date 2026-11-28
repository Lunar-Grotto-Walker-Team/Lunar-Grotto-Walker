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

#include "frontier_exploration_ros2/trajectory_buffer.hpp"

#include <cmath>
#include <limits>

namespace frontier_exploration_ros2
{
namespace
{

double distance_between(const TrajectoryPoint & a, const TrajectoryPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

TrajectoryBuffer::TrajectoryBuffer(TrajectoryBufferConfig config)
: config_(config)
{
}

void TrajectoryBuffer::add(double x, double y, double yaw, int64_t stamp_ns)
{
  TrajectoryPoint point{x, y, yaw, stamp_ns};
  if (!points_.empty() && distance_between(points_.back(), point) < config_.min_spacing_m) {
    return;
  }
  points_.push_back(point);
  if (config_.max_points > 0U) {
    while (points_.size() > config_.max_points) {
      points_.erase(points_.begin());
    }
  }
}

void TrajectoryBuffer::clear()
{
  points_.clear();
}

std::size_t TrajectoryBuffer::size() const
{
  return points_.size();
}

const std::vector<TrajectoryPoint> & TrajectoryBuffer::points() const
{
  return points_;
}

std::optional<double> TrajectoryBuffer::tangent_yaw() const
{
  if (points_.empty()) {
    return std::nullopt;
  }
  const TrajectoryPoint & last = points_.back();
  if (points_.size() == 1U) {
    return last.yaw;
  }

  std::size_t anchor = points_.size() - 2U;
  double accumulated = 0.0;
  for (std::size_t i = points_.size() - 1U; i > 0U; --i) {
    accumulated += distance_between(points_[i - 1U], points_[i]);
    anchor = i - 1U;
    if (accumulated >= config_.tangent_window_m) {
      break;
    }
  }

  const TrajectoryPoint & from = points_[anchor];
  const double dx = last.x - from.x;
  const double dy = last.y - from.y;
  if (std::hypot(dx, dy) <= std::numeric_limits<double>::epsilon()) {
    return last.yaw;
  }
  return std::atan2(dy, dx);
}

std::optional<TrajectoryPoint> TrajectoryBuffer::nearest_accepted(
  double x,
  double y,
  const std::function<bool(const TrajectoryPoint &)> & accept) const
{
  std::optional<TrajectoryPoint> best;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto & point : points_) {
    if (accept && !accept(point)) {
      continue;
    }
    const double distance = std::hypot(point.x - x, point.y - y);
    if (distance < best_distance) {
      best_distance = distance;
      best = point;
    }
  }
  return best;
}

std::vector<TrajectoryPoint> TrajectoryBuffer::reverse_waypoints(double spacing_m) const
{
  std::vector<TrajectoryPoint> waypoints;
  if (points_.empty()) {
    return waypoints;
  }

  std::size_t last_index = points_.size() - 1U;
  waypoints.push_back(points_[last_index]);

  while (last_index > 0U) {
    std::size_t found = last_index;
    for (std::size_t i = last_index; i > 0U; --i) {
      if (distance_between(points_[i - 1U], waypoints.back()) >= spacing_m) {
        found = i - 1U;
        break;
      }
    }
    if (found == last_index) {
      break;
    }
    last_index = found;
    waypoints.push_back(points_[last_index]);
  }

  if (last_index != 0U) {
    waypoints.push_back(points_.front());
  }
  return waypoints;
}

}  // namespace frontier_exploration_ros2
