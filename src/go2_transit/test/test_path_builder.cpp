#include <gtest/gtest.h>

#include <cmath>

#include "go2_transit/transit_core.hpp"

using go2_transit::Pose2D;
using go2_transit::build_straight_path;

TEST(PathBuilder, SpacingIsExact)
{
  auto path = build_straight_path(Pose2D{0, 0, 0}, Pose2D{10, 0, 0}, 0.25);
  ASSERT_GE(path.size(), 2u);
  for (size_t i = 1; i < path.size(); ++i) {
    const double d = go2_transit::distance_2d(path[i - 1], path[i]);
    EXPECT_LE(d, 0.25 + 1e-6);
    EXPECT_GE(d, 1e-6);  // 不允许重复点
  }
  EXPECT_NEAR(path.back().x, 10.0, 1e-9);
}

TEST(PathBuilder, HeadingsPointAlongPath)
{
  auto path = build_straight_path(Pose2D{0, 0, 0}, Pose2D{0, 10, 0}, 0.5);
  for (const auto & p : path) {
    EXPECT_NEAR(p.yaw, M_PI_2, 1e-9);  // 朝 +y
  }
}

TEST(PathBuilder, DegenerateWhenAlreadyAtGoal)
{
  auto path = build_straight_path(Pose2D{1, 2, 0.3}, Pose2D{1, 2, 0.3}, 0.25);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_NEAR(path[0].x, 1.0, 1e-9);
  EXPECT_NEAR(path[1].x, 1.0, 1e-9);
}

TEST(PathBuilder, ShortDistanceStillHasTwoPoints)
{
  auto path = build_straight_path(Pose2D{0, 0, 0}, Pose2D{0.1, 0, 0}, 0.25);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_NEAR(path.back().x, 0.1, 1e-9);
}

TEST(PathBuilder, RejectsNonPositiveSpacing)
{
  auto path = build_straight_path(Pose2D{0, 0, 0}, Pose2D{1, 0, 0}, 0.0);
  EXPECT_EQ(path.size(), 2u);  // 退化为起终点，不能死循环
}
