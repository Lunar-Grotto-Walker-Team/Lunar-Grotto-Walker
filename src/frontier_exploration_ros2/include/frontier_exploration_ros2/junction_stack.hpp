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
#include <optional>
#include <vector>

namespace frontier_exploration_ros2
{

struct JunctionNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  int64_t stamp_ns{0};
  bool failed{false};
};

struct JunctionStackConfig
{
  double dedup_m{1.0};
  std::size_t max_size{32};
};

// Depth-first branch memory: junction poses pushed as the robot passes them,
// popped when a branch dead-ends.
class JunctionStack
{
public:
  explicit JunctionStack(JunctionStackConfig config = JunctionStackConfig{});

  // Returns true when a new node was pushed, false when an existing nearby node
  // was merged into.
  bool push_or_merge(double x, double y, double yaw, int64_t stamp_ns);

  // Most recent non-failed node; failed nodes encountered are discarded.
  std::optional<JunctionNode> pop_next();

  void mark_failed(double x, double y, double tolerance_m);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] const std::vector<JunctionNode> & nodes() const;
  void clear();

private:
  JunctionStackConfig config_;
  std::vector<JunctionNode> nodes_;
};

}  // namespace frontier_exploration_ros2
