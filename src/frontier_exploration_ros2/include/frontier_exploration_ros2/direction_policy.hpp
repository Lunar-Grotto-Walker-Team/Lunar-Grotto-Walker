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
#include <vector>

namespace frontier_exploration_ros2
{

// Relative-angle sectors evaluated against the robot's intended travel direction.
// "Right" is negative yaw because ROS yaw grows counter-clockwise.
enum class DirectionBucket
{
  FRONT,
  RIGHT,
  LEFT,
  BACK,
};

struct DirectionPolicyConfig
{
  double front_cone_half_deg{60.0};
  double right_arc_max_deg{150.0};
};

class DirectionPolicy
{
public:
  explicit DirectionPolicy(DirectionPolicyConfig config = DirectionPolicyConfig{});

  [[nodiscard]] DirectionBucket bucket_for(double relative_angle_rad) const;

  // Indices of the highest-priority non-empty bucket (FRONT > RIGHT > LEFT > BACK).
  [[nodiscard]] std::vector<std::size_t> select_bucket(
    const std::vector<double> & relative_angles) const;

  [[nodiscard]] std::vector<DirectionBucket> non_empty_buckets(
    const std::vector<double> & relative_angles) const;

  // Number of circular clusters formed by `angles` when successive angles closer
  // than `gap_rad` are merged. Always at least 1 for a non-empty input.
  [[nodiscard]] std::size_t cluster_count(
    const std::vector<double> & angles,
    double gap_rad) const;

  static double normalize_angle(double angle);
  static double relative_angle(double dx, double dy, double base_yaw);

private:
  DirectionPolicyConfig config_;
};

}  // namespace frontier_exploration_ros2
