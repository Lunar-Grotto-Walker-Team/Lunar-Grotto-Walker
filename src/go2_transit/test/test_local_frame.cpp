#include <gtest/gtest.h>

#include <cmath>

#include "go2_transit/transit_core.hpp"

using go2_transit::Pose2D;

TEST(LocalFrame, IdentityOriginIsPassThrough)
{
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, 0}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 3.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
}

TEST(LocalFrame, OriginYawRotatesTheLocalXAxis)
{
  // 原点朝 +y（90°）：往前走 3 m 应当落在 (0, 3)
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, M_PI_2}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 3.0, 1e-9);
}

TEST(LocalFrame, OriginOffsetIsAdded)
{
  // 原点 (10,20) 朝 -x（π）：往前走 3 m 应当落在 (7, 20)
  const auto p = go2_transit::local_to_planning(Pose2D{10.0, 20.0, M_PI}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 7.0, 1e-9);
  EXPECT_NEAR(p.y, 20.0, 1e-9);
}

TEST(LocalFrame, LocalYIsLeftPositive)
{
  // 朝 +x，往左 2 m → (0, 2)
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, 0}, 0.0, 2.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 2.0, 1e-9);
}

TEST(LocalFrame, ZeroGoalIsTheOriginItself)
{
  const auto p = go2_transit::local_to_planning(Pose2D{5.0, -3.0, 0.7}, 0.0, 0.0);
  EXPECT_NEAR(p.x, 5.0, 1e-9);
  EXPECT_NEAR(p.y, -3.0, 1e-9);
}

TEST(LocalFrame, FrameIdMustBeExactlyLocal)
{
  EXPECT_TRUE(go2_transit::is_local_frame("local"));
  EXPECT_FALSE(go2_transit::is_local_frame(""));
  EXPECT_FALSE(go2_transit::is_local_frame("odom"));
  EXPECT_FALSE(go2_transit::is_local_frame("Local"));
}
