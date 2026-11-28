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
#include <vector>

#include "frontier_exploration_ros2/direction_policy.hpp"

using namespace frontier_exploration_ros2;

TEST(DirectionPolicyTest, BucketBoundaries)
{
  DirectionPolicy policy(DirectionPolicyConfig{60.0, 150.0});
  EXPECT_EQ(policy.bucket_for(0.0), DirectionBucket::FRONT);
  EXPECT_EQ(policy.bucket_for(M_PI / 3.0), DirectionBucket::FRONT);
  EXPECT_EQ(policy.bucket_for(-M_PI / 3.0 - 0.01), DirectionBucket::RIGHT);
  EXPECT_EQ(policy.bucket_for(-2.0), DirectionBucket::RIGHT);
  EXPECT_EQ(policy.bucket_for(2.0), DirectionBucket::LEFT);
  EXPECT_EQ(policy.bucket_for(M_PI), DirectionBucket::BACK);
  EXPECT_EQ(policy.bucket_for(-M_PI), DirectionBucket::BACK);
}

TEST(DirectionPolicyTest, FrontWinsOverCloserRight)
{
  DirectionPolicy policy;
  const std::vector<double> angles{0.7, -1.4};
  const auto selected = policy.select_bucket(angles);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front(), 0U);
}

TEST(DirectionPolicyTest, RightWinsWhenFrontEmpty)
{
  DirectionPolicy policy;
  const std::vector<double> angles{-1.2, 1.4};
  const auto selected = policy.select_bucket(angles);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front(), 0U);
}

TEST(DirectionPolicyTest, LeftWinsWhenFrontAndRightEmpty)
{
  DirectionPolicy policy;
  const std::vector<double> angles{1.4, 3.0};
  const auto selected = policy.select_bucket(angles);
  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front(), 0U);
}

TEST(DirectionPolicyTest, EmptyWhenNoCandidates)
{
  DirectionPolicy policy;
  EXPECT_TRUE(policy.select_bucket({}).empty());
  EXPECT_TRUE(policy.non_empty_buckets({}).empty());
}

TEST(DirectionPolicyTest, ClusterCountSplitsBranches)
{
  DirectionPolicy policy;
  EXPECT_EQ(policy.cluster_count({1.47, 1.57, 1.67}, M_PI / 4.0), 1U);
  EXPECT_EQ(policy.cluster_count({-1.2, 0.0, 0.05}, M_PI / 4.0), 2U);
}

TEST(DirectionPolicyTest, ClusterCountHandlesWrapAround)
{
  DirectionPolicy policy;
  EXPECT_EQ(policy.cluster_count({3.10, -3.12}, M_PI / 4.0), 1U);
}

TEST(DirectionPolicyTest, RelativeAngleUsesBaseYaw)
{
  EXPECT_NEAR(DirectionPolicy::relative_angle(1.0, 0.0, M_PI / 2.0), -M_PI / 2.0, 1e-9);
}
