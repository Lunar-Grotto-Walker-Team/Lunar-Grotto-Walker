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

#include <gtest/gtest.h>

#include "frontier_exploration_ros2/junction_stack.hpp"

using namespace frontier_exploration_ros2;

TEST(JunctionStackTest, PushAndMergeWithinDedupRadius)
{
  JunctionStack stack(JunctionStackConfig{1.0, 32});
  EXPECT_TRUE(stack.push_or_merge(0.0, 0.0, 0.0, 1));
  EXPECT_FALSE(stack.push_or_merge(0.5, 0.0, 0.0, 2));
  EXPECT_EQ(stack.size(), 1U);
  EXPECT_EQ(stack.nodes().back().stamp_ns, 2);
  EXPECT_TRUE(stack.push_or_merge(2.0, 0.0, 0.0, 3));
  EXPECT_EQ(stack.size(), 2U);
}

TEST(JunctionStackTest, MaxSizeDropsOldest)
{
  JunctionStack stack(JunctionStackConfig{0.1, 2});
  stack.push_or_merge(0.0, 0.0, 0.0, 1);
  stack.push_or_merge(5.0, 0.0, 0.0, 2);
  stack.push_or_merge(10.0, 0.0, 0.0, 3);
  ASSERT_EQ(stack.size(), 2U);
  EXPECT_DOUBLE_EQ(stack.nodes().front().x, 5.0);
}

TEST(JunctionStackTest, PopNextReturnsMostRecent)
{
  JunctionStack stack;
  stack.push_or_merge(0.0, 0.0, 0.0, 1);
  stack.push_or_merge(5.0, 0.0, 0.0, 2);
  const auto node = stack.pop_next();
  ASSERT_TRUE(node.has_value());
  EXPECT_DOUBLE_EQ(node->x, 5.0);
  EXPECT_EQ(stack.size(), 1U);
}

TEST(JunctionStackTest, PopNextSkipsFailedNodes)
{
  JunctionStack stack;
  stack.push_or_merge(0.0, 0.0, 0.0, 1);
  stack.push_or_merge(5.0, 0.0, 0.0, 2);
  stack.mark_failed(5.0, 0.0, 1.0);
  const auto node = stack.pop_next();
  ASSERT_TRUE(node.has_value());
  EXPECT_DOUBLE_EQ(node->x, 0.0);
  EXPECT_TRUE(stack.empty());
}

TEST(JunctionStackTest, PopNextOnEmptyReturnsNullopt)
{
  JunctionStack stack;
  EXPECT_FALSE(stack.pop_next().has_value());
}
