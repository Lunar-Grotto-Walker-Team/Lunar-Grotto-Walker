// tests/test_servo_scan.cpp
// Pure scan waypoint generator (2026-11-13, SCANNING state).

#include "lgw_bt_planner/servo/servo_scan.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

TEST(ScanWaypoints, PreservesJ4J6AndRaisesJ2J3)
{
  const std::array<double, 6> q{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  lgw_bt_planner::ScanParams p;
  p.j2_raise_rad = 0.8;
  p.j3_raise_rad = -0.4;
  const auto wps = lgw_bt_planner::make_scan_waypoints(q, p);
  ASSERT_FALSE(wps.empty());
  for (const auto & w : wps) {
    EXPECT_NEAR(w[1], 0.8, 1e-9);
    EXPECT_NEAR(w[2], -0.4, 1e-9);
    EXPECT_NEAR(w[3], 0.4, 1e-9);
    EXPECT_NEAR(w[5], 0.6, 1e-9);
  }
}

TEST(ScanWaypoints, CoversBothJ5BranchesAndAvoidsGap)
{
  const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
  lgw_bt_planner::ScanParams p;
  const auto wps = lgw_bt_planner::make_scan_waypoints(q, p);
  bool neg = false, pos = false;
  for (const auto & w : wps) {
    EXPECT_GE(std::abs(w[4]), p.j5_gap_rad - 1e-9)
      << "j5 inside singularity gap";
    if (w[4] <= p.j5_max_rad + 1e-9) {
      neg = true;
    }
    if (w[4] >= p.j5_min_rad_pos - 1e-9) {
      pos = true;
    }
  }
  EXPECT_TRUE(neg);
  EXPECT_TRUE(pos);
}

TEST(ScanWaypoints, CoversJ1RangeAndRejectsBadStep)
{
  const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
  lgw_bt_planner::ScanParams p;
  p.j1_min_rad = -0.5;
  p.j1_max_rad = 0.5;
  p.j1_step_rad = 0.25;
  const auto wps = lgw_bt_planner::make_scan_waypoints(q, p);
  ASSERT_FALSE(wps.empty());
  EXPECT_NEAR(wps.front()[0], -0.5, 1e-9);
  EXPECT_NEAR(wps.back()[0], 0.5, 1e-9);

  lgw_bt_planner::ScanParams bad = p;
  bad.j1_step_rad = 0.0;
  EXPECT_TRUE(lgw_bt_planner::make_scan_waypoints(q, bad).empty());
}
