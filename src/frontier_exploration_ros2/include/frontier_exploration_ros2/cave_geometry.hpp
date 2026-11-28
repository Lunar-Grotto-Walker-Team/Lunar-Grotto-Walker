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

#include <geometry_msgs/msg/quaternion.hpp>

namespace frontier_exploration_ros2
{

// Entrance-based cave boundary: a radius around the recorded entrance pose and
// an optional half-plane gate oriented along the heading the robot had when the
// entrance was recorded.
struct EntranceConstraint
{
  double radius_m{3.0};
  bool gate_enabled{true};
  double gate_margin_m{0.5};
};

struct EntranceGeometry
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  bool valid{false};
};

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);

class CaveGeometry
{
public:
  explicit CaveGeometry(EntranceConstraint constraint = EntranceConstraint{});

  void set_entrance(double x, double y, double heading);
  void clear();

  [[nodiscard]] bool has_entrance() const;
  [[nodiscard]] const EntranceGeometry & entrance() const;

  // True when the point should be treated as outside the cave: inside the
  // entrance radius, or behind the entrance gate.
  [[nodiscard]] bool is_outside_cave(double x, double y) const;
  [[nodiscard]] bool is_within_entrance_radius(double x, double y) const;

private:
  EntranceConstraint constraint_;
  EntranceGeometry entrance_;
};

}  // namespace frontier_exploration_ros2
