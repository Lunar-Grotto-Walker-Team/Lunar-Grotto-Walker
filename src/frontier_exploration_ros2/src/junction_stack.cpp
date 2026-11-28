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

#include "frontier_exploration_ros2/junction_stack.hpp"

#include <cmath>
#include <limits>

namespace frontier_exploration_ros2
{
namespace
{

double distance_to(const JunctionNode & node, double x, double y)
{
  return std::hypot(node.x - x, node.y - y);
}

}  // namespace

JunctionStack::JunctionStack(JunctionStackConfig config)
: config_(config)
{
}

bool JunctionStack::push_or_merge(double x, double y, double yaw, int64_t stamp_ns)
{
  for (auto & node : nodes_) {
    if (distance_to(node, x, y) < config_.dedup_m) {
      node.x = x;
      node.y = y;
      node.yaw = yaw;
      node.stamp_ns = stamp_ns;
      return false;
    }
  }

  nodes_.push_back(JunctionNode{x, y, yaw, stamp_ns, false});
  if (config_.max_size > 0U) {
    while (nodes_.size() > config_.max_size) {
      nodes_.erase(nodes_.begin());
    }
  }
  return true;
}

std::optional<JunctionNode> JunctionStack::pop_next()
{
  while (!nodes_.empty()) {
    JunctionNode node = nodes_.back();
    nodes_.pop_back();
    if (!node.failed) {
      return node;
    }
  }
  return std::nullopt;
}

void JunctionStack::mark_failed(double x, double y, double tolerance_m)
{
  JunctionNode * nearest = nullptr;
  double nearest_distance = std::numeric_limits<double>::max();
  for (auto & node : nodes_) {
    const double distance = distance_to(node, x, y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = &node;
    }
  }
  if (nearest != nullptr && nearest_distance <= tolerance_m) {
    nearest->failed = true;
  }
}

bool JunctionStack::empty() const
{
  return nodes_.empty();
}

std::size_t JunctionStack::size() const
{
  return nodes_.size();
}

const std::vector<JunctionNode> & JunctionStack::nodes() const
{
  return nodes_;
}

void JunctionStack::clear()
{
  nodes_.clear();
}

}  // namespace frontier_exploration_ros2
