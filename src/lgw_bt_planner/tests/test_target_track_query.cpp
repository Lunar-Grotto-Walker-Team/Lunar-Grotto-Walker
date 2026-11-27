// tests/test_target_track_query.cpp
// Pure tests for the UnifiedTrackArray queries used by the new BT leaves.
// No ROS runtime needed.

#include "lgw_bt_planner/target_track_query.hpp"

#include <gtest/gtest.h>

namespace
{

using yoloe_interfaces::msg::UnifiedTrack;
using yoloe_interfaces::msg::UnifiedTrackArray;

UnifiedTrack make_track(
  uint64_t id, double x, double y, bool geometry_valid = true)
{
  UnifiedTrack track;
  track.track_id = id;
  track.geometry_valid = geometry_valid;
  track.position_arm_base.x = x;
  track.position_arm_base.y = y;
  track.position_arm_base.z = 0.0;
  return track;
}

}  // namespace

TEST(TargetTrackQuery, FindsSelectedTrack)
{
  UnifiedTrackArray arr;
  arr.selected_track_id = 7;
  arr.tracks.push_back(make_track(3, 1.0, 0.0));
  arr.tracks.push_back(make_track(7, 0.4, 0.1));
  const auto * track = lgw_bt_planner::track_query::selected_track(arr);
  ASSERT_NE(track, nullptr);
  EXPECT_EQ(track->track_id, 7u);
}

TEST(TargetTrackQuery, ZeroAndMissingReturnNull)
{
  UnifiedTrackArray arr;
  arr.tracks.push_back(make_track(3, 1.0, 0.0));
  EXPECT_EQ(lgw_bt_planner::track_query::selected_track(arr), nullptr);
  arr.selected_track_id = 99;
  EXPECT_EQ(lgw_bt_planner::track_query::selected_track(arr), nullptr);
  EXPECT_EQ(lgw_bt_planner::track_query::find_track(arr, 0), nullptr);
}

TEST(TargetTrackQuery, XyDistanceIgnoresZ)
{
  auto track = make_track(1, 0.3, 0.4);
  track.position_arm_base.z = 9.0;
  EXPECT_DOUBLE_EQ(lgw_bt_planner::track_query::arm_base_xy_m(track), 0.5);
}

TEST(TargetTrackQuery, BandRequiresGeometryValid)
{
  const auto track = make_track(1, 0.3, 0.0, false);
  EXPECT_FALSE(lgw_bt_planner::track_query::xy_in_band(track, 0.0, 0.6));
}

// 2026-11-19: 正前方扇区。0.2618 rad = 15 deg。
TEST(TargetTrackQuery, FrontSectorAcceptsStraightAhead)
{
  const auto track = make_track(1, 0.5, 0.0);
  EXPECT_TRUE(lgw_bt_planner::track_query::in_front_sector(track, 0.2618));
}

// 2026-11-20: 带符号方位角。扇区中心不在 +X 时, 左右必须可区分。
TEST(TargetTrackQuery, SignedAzimuthAndCustomSectorCentre)
{
  constexpr double kMinusY = -1.5707963;   // -pi/2
  constexpr double kPi = 3.14159265358979;
  constexpr double kZero = 0.0;

  // 目标在 -Y 侧(y<0): 带符号方位 -90°; 旧的 abs 版本会给出 +90°。
  const auto right = make_track(1, 0.0, -0.5);
  EXPECT_NEAR(
    lgw_bt_planner::track_query::arm_base_azimuth_signed_rad(right), -kPi / 2,
    1e-6);
  EXPECT_NEAR(
    lgw_bt_planner::track_query::arm_base_azimuth_rad(right), kPi / 2, 1e-6);

  // 以 -Y 为中心 ±15°: 正右侧通过
  EXPECT_TRUE(
    lgw_bt_planner::track_query::in_front_sector(right, 0.2618, kMinusY));
  // 同样位置的 +Y 目标(左侧)必须被拒
  const auto left = make_track(2, 0.0, 0.5);
  EXPECT_FALSE(
    lgw_bt_planner::track_query::in_front_sector(left, 0.2618, kMinusY));

  // 默认中心 0 = +X: -Y 侧的右目标要被拒(旧的 abs 版本会误放行)
  EXPECT_FALSE(
    lgw_bt_planner::track_query::in_front_sector(right, 0.2618, kZero));
}

TEST(TargetTrackQuery, FrontSectorRejectsSideAndRear)
{
  constexpr double k15 = 0.2618;
  // 45 度偏侧: 拒绝
  EXPECT_FALSE(
    lgw_bt_planner::track_query::in_front_sector(make_track(1, 0.4, 0.4), k15));
  // 正后方: 拒绝
  EXPECT_FALSE(
    lgw_bt_planner::track_query::in_front_sector(make_track(1, -0.5, 0.0), k15));
}

TEST(TargetTrackQuery, FrontSectorBoundaryIsInclusive)
{
  // 恰好 15 度: x=0.5, y=0.5*tan(15 deg)
  const double y = 0.5 * std::tan(0.2618);
  EXPECT_TRUE(lgw_bt_planner::track_query::in_front_sector(
    make_track(1, 0.5, y), 0.2618));
}

TEST(TargetTrackQuery, FrontSectorDisabledWhenAngleNonPositive)
{
  // <=0 关闭判据: 侧后方的目标也放行(保持旧行为)
  EXPECT_TRUE(
    lgw_bt_planner::track_query::in_front_sector(make_track(1, -0.5, 0.0), 0.0));
}

TEST(TargetTrackQuery, FrontSectorRequiresGeometryValid)
{
  EXPECT_FALSE(lgw_bt_planner::track_query::in_front_sector(
    make_track(1, 0.5, 0.0, false), 0.2618));
}

TEST(TargetTrackQuery, FreshnessBoundIsInclusive)
{
  auto track = make_track(1, 0.3, 0.0);
  track.time_since_last_observation_ms = 1000;
  EXPECT_TRUE(lgw_bt_planner::track_query::is_fresh(track, 1000));
  track.time_since_last_observation_ms = 1001;
  EXPECT_FALSE(lgw_bt_planner::track_query::is_fresh(track, 1000));
}

TEST(TargetTrackQuery, BandBoundsAreInclusive)
{
  const auto track = make_track(1, 0.6, 0.0);
  EXPECT_TRUE(lgw_bt_planner::track_query::xy_in_band(track, 0.0, 0.6));
  EXPECT_TRUE(lgw_bt_planner::track_query::xy_in_band(track, 0.6, 0.6));
  EXPECT_FALSE(lgw_bt_planner::track_query::xy_in_band(track, 0.0, 0.59));
}
