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

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace frontier_exploration_ros2
{

struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  int64_t stamp_ns{0};
};

struct TrajectoryBufferConfig
{
  double min_spacing_m{0.2};
  std::size_t max_points{5000};
  double tangent_window_m{1.0};
};

// Bounded record of where the robot has been. Used to derive the travel
// direction for the direction policy, to pick a point back on the path after
// grasping, and to build coarse return waypoints.
class TrajectoryBuffer
{
public:
  explicit TrajectoryBuffer(TrajectoryBufferConfig config = TrajectoryBufferConfig{});

  void add(double x, double y, double yaw, int64_t stamp_ns);
  void clear();

  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] const std::vector<TrajectoryPoint> & points() const;

  // Direction of travel over the last tangent_window_m; falls back to the most
  // recent yaw when fewer than two samples exist.
  [[nodiscard]] std::optional<double> tangent_yaw() const;

  [[nodiscard]] std::optional<TrajectoryPoint> nearest_accepted(
    double x,
    double y,
    const std::function<bool(const TrajectoryPoint &)> & accept) const;

  // Waypoints from the newest sample back to the oldest, spaced at least
  // spacing_m apart, always including the oldest sample.
  [[nodiscard]] std::vector<TrajectoryPoint> reverse_waypoints(double spacing_m) const;

private:
  TrajectoryBufferConfig config_;
  std::vector<TrajectoryPoint> points_;
};

}  // namespace frontier_exploration_ros2
