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

#include "frontier_exploration_ros2/direction_policy.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

DirectionPolicy::DirectionPolicy(DirectionPolicyConfig config)
: config_(config)
{
}

double DirectionPolicy::normalize_angle(double angle)
{
  double normalized = std::fmod(angle + kPi, 2.0 * kPi);
  if (normalized <= 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

double DirectionPolicy::relative_angle(double dx, double dy, double base_yaw)
{
  return normalize_angle(std::atan2(dy, dx) - base_yaw);
}

DirectionBucket DirectionPolicy::bucket_for(double relative_angle_rad) const
{
  const double d = normalize_angle(relative_angle_rad);
  const double front = degrees_to_radians(config_.front_cone_half_deg);
  const double right_max = degrees_to_radians(config_.right_arc_max_deg);
  if (std::abs(d) <= front) {
    return DirectionBucket::FRONT;
  }
  if (d < 0.0 && d >= -right_max) {
    return DirectionBucket::RIGHT;
  }
  if (d > 0.0 && d <= right_max) {
    return DirectionBucket::LEFT;
  }
  return DirectionBucket::BACK;
}

std::vector<std::size_t> DirectionPolicy::select_bucket(
  const std::vector<double> & relative_angles) const
{
  std::vector<std::size_t> selected;
  for (const auto bucket :
    {DirectionBucket::FRONT, DirectionBucket::RIGHT, DirectionBucket::LEFT, DirectionBucket::BACK})
  {
    selected.clear();
    for (std::size_t i = 0; i < relative_angles.size(); ++i) {
      if (bucket_for(relative_angles[i]) == bucket) {
        selected.push_back(i);
      }
    }
    if (!selected.empty()) {
      return selected;
    }
  }
  return {};
}

std::vector<DirectionBucket> DirectionPolicy::non_empty_buckets(
  const std::vector<double> & relative_angles) const
{
  std::vector<DirectionBucket> result;
  for (const auto bucket :
    {DirectionBucket::FRONT, DirectionBucket::RIGHT, DirectionBucket::LEFT, DirectionBucket::BACK})
  {
    const bool present = std::any_of(
      relative_angles.begin(), relative_angles.end(),
      [this, bucket](double angle) {return bucket_for(angle) == bucket;});
    if (present) {
      result.push_back(bucket);
    }
  }
  return result;
}

std::size_t DirectionPolicy::cluster_count(
  const std::vector<double> & angles,
  double gap_rad) const
{
  if (angles.empty()) {
    return 0U;
  }
  if (angles.size() == 1U) {
    return 1U;
  }
  std::vector<double> sorted = angles;
  std::sort(sorted.begin(), sorted.end());

  std::size_t large_gaps = 0U;
  for (std::size_t i = 1; i < sorted.size(); ++i) {
    if (sorted[i] - sorted[i - 1] > gap_rad) {
      ++large_gaps;
    }
  }
  const double wrap_gap = (sorted.front() + 2.0 * kPi) - sorted.back();
  if (wrap_gap > gap_rad) {
    ++large_gaps;
  }
  return std::max<std::size_t>(1U, large_gaps);
}

}  // namespace frontier_exploration_ros2
