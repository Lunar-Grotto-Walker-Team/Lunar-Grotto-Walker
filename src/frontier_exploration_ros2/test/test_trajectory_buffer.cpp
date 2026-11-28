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

#include <cmath>

#include "frontier_exploration_ros2/trajectory_buffer.hpp"

using namespace frontier_exploration_ros2;

TEST(TrajectoryBufferTest, MinSpacingDropsNearDuplicate)
{
  TrajectoryBuffer buffer(TrajectoryBufferConfig{0.2, 5000, 1.0});
  buffer.add(0.0, 0.0, 0.0, 0);
  buffer.add(0.1, 0.0, 0.0, 1);
  buffer.add(0.5, 0.0, 0.0, 2);
  EXPECT_EQ(buffer.size(), 2U);
}

TEST(TrajectoryBufferTest, RingDropsOldest)
{
  TrajectoryBuffer buffer(TrajectoryBufferConfig{0.0, 3, 1.0});
  for (int i = 0; i < 5; ++i) {
    buffer.add(static_cast<double>(i), 0.0, 0.0, i);
  }
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_DOUBLE_EQ(buffer.points().front().x, 2.0);
}

TEST(TrajectoryBufferTest, TangentUsesRecentWindow)
{
  TrajectoryBuffer buffer(TrajectoryBufferConfig{0.0, 5000, 1.0});
  buffer.add(0.0, 0.0, 0.0, 0);
  buffer.add(1.0, 0.0, 0.0, 1);
  buffer.add(2.0, 1.0, 0.0, 2);   // recent 1 m displacement is (1,1)
  const auto yaw = buffer.tangent_yaw();
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(*yaw, M_PI / 4.0, 1e-9);
}

TEST(TrajectoryBufferTest, TangentFallsBackToLastYaw)
{
  TrajectoryBuffer buffer;
  buffer.add(0.0, 0.0, 0.3, 0);
  const auto yaw = buffer.tangent_yaw();
  ASSERT_TRUE(yaw.has_value());
  EXPECT_DOUBLE_EQ(*yaw, 0.3);
  EXPECT_FALSE(TrajectoryBuffer{}.tangent_yaw().has_value());
}

TEST(TrajectoryBufferTest, NearestAcceptedSkipsRejected)
{
  TrajectoryBuffer buffer(TrajectoryBufferConfig{0.0, 5000, 1.0});
  buffer.add(0.0, 0.0, 0.0, 0);
  buffer.add(1.0, 0.0, 0.0, 1);
  buffer.add(2.0, 0.0, 0.0, 2);
  const auto point = buffer.nearest_accepted(
    1.9, 0.0, [](const TrajectoryPoint & p) {return p.x < 1.5;});
  ASSERT_TRUE(point.has_value());
  EXPECT_DOUBLE_EQ(point->x, 1.0);
}

TEST(TrajectoryBufferTest, ReverseWaypointsSpacing)
{
  TrajectoryBuffer buffer(TrajectoryBufferConfig{0.0, 5000, 1.0});
  for (int i = 0; i <= 10; ++i) {
    buffer.add(static_cast<double>(i), 0.0, 0.0, i);
  }
  const auto waypoints = buffer.reverse_waypoints(5.0);
  ASSERT_FALSE(waypoints.empty());
  EXPECT_DOUBLE_EQ(waypoints.front().x, 10.0);
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    EXPECT_GE(std::abs(waypoints[i - 1].x - waypoints[i].x), 5.0);
  }
  EXPECT_DOUBLE_EQ(waypoints.back().x, 0.0);
}
