#include <gtest/gtest.h>

#include <cmath>

#include "go2_transit/transit_core.hpp"

using go2_transit::Pose2D;

TEST(AngleMath, NormalizeWrapsIntoPiRange)
{
  // ±pi 是同一个朝向，边界上不约束符号，只要求幅度落在 pi
  EXPECT_NEAR(std::fabs(go2_transit::normalize_angle(3.0 * M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(std::fabs(go2_transit::normalize_angle(-3.0 * M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(go2_transit::normalize_angle(0.5), 0.5, 1e-12);
}

TEST(AngleMath, ShortestDistanceCrossesWrap)
{
  // 3.0 -> -3.0 应该走 +0.283 的短边，而不是 -6.0 的长边
  EXPECT_NEAR(go2_transit::shortest_angular_distance(3.0, -3.0), 0.2831853, 1e-6);
}

TEST(AngleMath, XyToleranceBoundary)
{
  Pose2D goal{1.0, 1.0, 0.0};
  EXPECT_TRUE(
    go2_transit::within_xy_tolerance(Pose2D{1.0 + 0.249, 1.0, 0.0}, goal, 0.25));
  EXPECT_FALSE(
    go2_transit::within_xy_tolerance(Pose2D{1.0 + 0.251, 1.0, 0.0}, goal, 0.25));
}

TEST(AngleMath, YawToleranceBoundary)
{
  EXPECT_TRUE(go2_transit::within_yaw_tolerance(0.169, 0.0, 0.17));
  EXPECT_FALSE(go2_transit::within_yaw_tolerance(0.171, 0.0, 0.17));
  // 跨 ±pi 的短边判定
  EXPECT_TRUE(go2_transit::within_yaw_tolerance(3.10, -3.10, 0.17));
}

TEST(AngleMath, DistanceIsEuclidean)
{
  EXPECT_NEAR(go2_transit::distance_2d(Pose2D{0, 0, 0}, Pose2D{3, 4, 0}), 5.0, 1e-9);
}

// /go2_transit/status 和动作 result.reason 都直接用这些字符串，改动即破坏下游解析
TEST(TransitTypes, PhaseAndReasonHaveStableNames)
{
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::WAIT_READY), "wait_ready");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::PRE_ALIGN), "pre_align");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::TRAVERSE), "traverse");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::FINAL_ALIGN), "final_align");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::VERIFY), "verify");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::TransitPhase::DONE), "done");

  EXPECT_STREQ(go2_transit::to_string(go2_transit::FailReason::OK), "ok");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::FailReason::TIMEOUT), "timeout");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::FailReason::UNREACHABLE), "unreachable");
  EXPECT_STREQ(
    go2_transit::to_string(go2_transit::FailReason::NAV2_UNAVAILABLE),
    "nav2_unavailable");
  EXPECT_STREQ(
    go2_transit::to_string(go2_transit::FailReason::TF_UNAVAILABLE), "tf_unavailable");
  EXPECT_STREQ(
    go2_transit::to_string(go2_transit::FailReason::INVALID_GOAL), "invalid_goal");
  EXPECT_STREQ(go2_transit::to_string(go2_transit::FailReason::CANCELED), "canceled");
}
