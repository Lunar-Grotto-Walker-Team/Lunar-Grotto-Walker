// tests/test_geometry_source.cpp
// Pure-function unit tests for select_source (Task 3 of the 2026-10-26
// servo-takeover plan, spec §7.4).
//
// No ROS runtime / executor / tf2_ros Buffer needed: plain structs + tf2
// LinearMath. Run the binary directly from build/lgw_bt_planner/
// (test_geometry_source) — colcon test is not available in this container
// (see CLAUDE.md).
//
// Transform convention (spec §7.4, matches input_adapter.py):
//   `T_base_odom` is lookupTransform("odom", "base_link") — it maps BASE
//   points into odom (p_odom = T_base_odom * p_base). The module inverts it
//   to go odom -> base: p_arm_base = T_base_odom.inverse() * p_odom.
//   BothStaleFallsToOdom pins this: a pure +1/+2/+3 translation maps
//   (4,5,6) to (3,3,3).

#include "lgw_bt_planner/geometry_source.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace
{

constexpr int64_t kNsPerMs = 1000000;

lgw_bt_planner::Contributor make_contributor(
  lgw_bt_planner::SourceKind kind, int64_t stamp_ns, const tf2::Vector3& p_arm_base)
{
  lgw_bt_planner::Contributor c;
  c.kind = kind;
  c.p_arm_base = p_arm_base;
  c.stamp_ns = stamp_ns;
  c.confidence = 0.9f;
  return c;
}

lgw_bt_planner::TrackSnapshot make_snapshot(
  const std::vector<lgw_bt_planner::Contributor>& contributors,
  int64_t time_since_last_obs_ms)
{
  lgw_bt_planner::TrackSnapshot t;
  t.track_id = 7;
  t.status = "confirmed";
  t.p_odom = tf2::Vector3(4.0, 5.0, 6.0);
  t.contributors = contributors;
  t.time_since_last_obs_ms = time_since_last_obs_ms;
  return t;
}

// Pure translation (1,2,3): T_base_odom = lookup("odom", "base_link"),
// i.e. maps BASE points into odom.
tf2::Transform translation_base_odom()
{
  tf2::Transform T;
  T.setIdentity();
  T.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));
  return T;
}

}  // namespace

// Priority 1: a fresh head contributor wins even when a fresh arm
// contributor is also present, at full gain.
TEST(GeometrySource, HeadWinsWhenFresh) {
  const int64_t now = 1000000000LL;  // arbitrary epoch ns
  const auto track = make_snapshot(
    {
      make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 100 * kNsPerMs, {1.0, 0.0, 0.0}),
      make_contributor(lgw_bt_planner::SourceKind::ARM, now - 100 * kNsPerMs, {0.0, 1.0, 0.0}),
    },
    /*time_since_last_obs_ms=*/1000);
  lgw_bt_planner::GeometrySourceParams params;
  lgw_bt_planner::SelectionState state;

  const auto sel = lgw_bt_planner::select_source(
    track, now, translation_base_odom(), params, state);

  EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::HEAD);
  EXPECT_EQ(sel.reason, "head");
  EXPECT_DOUBLE_EQ(sel.scale, 1.0);
  EXPECT_NEAR(sel.p_arm_base.x(), 1.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.y(), 0.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.z(), 0.0, 1e-9);
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::HEAD);
}

// Priority 2: a stale head (age 500ms > 400ms max) downgrades to the arm
// contributor IMMEDIATELY (no hysteresis on downgrade) at 0.7 gain.
TEST(GeometrySource, HeadStaleFallsToArmImmediately) {
  const int64_t now = 1000000000LL;
  const auto track = make_snapshot(
    {
      make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 500 * kNsPerMs, {9.0, 9.0, 9.0}),
      make_contributor(lgw_bt_planner::SourceKind::ARM, now - 100 * kNsPerMs, {0.0, 1.0, 0.0}),
    },
    /*time_since_last_obs_ms=*/200);
  lgw_bt_planner::GeometrySourceParams params;
  // Previous tick was on HEAD, mid-hysteresis counter left behind.
  lgw_bt_planner::SelectionState state;
  state.last = lgw_bt_planner::SourceKind::HEAD;
  state.upgrade_ticks = 7;

  const auto sel = lgw_bt_planner::select_source(
    track, now, translation_base_odom(), params, state);

  EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::ARM);
  EXPECT_EQ(sel.reason, "arm");
  EXPECT_DOUBLE_EQ(sel.scale, 0.7);
  EXPECT_NEAR(sel.p_arm_base.x(), 0.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.y(), 1.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.z(), 0.0, 1e-9);
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::ARM);
  EXPECT_EQ(state.upgrade_ticks, 0);
}

// Priority 3: both cameras stale but the track was seen recently enough
// -> KF back-projection through T_base_odom, applied DIRECTLY, at 0.5 gain.
TEST(GeometrySource, BothStaleFallsToOdom) {
  const int64_t now = 1000000000LL;
  const auto track = make_snapshot(
    {
      make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 500 * kNsPerMs, {9.0, 9.0, 9.0}),
      make_contributor(lgw_bt_planner::SourceKind::ARM, now - 600 * kNsPerMs, {8.0, 8.0, 8.0}),
    },
    /*time_since_last_obs_ms=*/800);
  lgw_bt_planner::GeometrySourceParams params;
  lgw_bt_planner::SelectionState state;
  state.last = lgw_bt_planner::SourceKind::ARM;

  const auto sel = lgw_bt_planner::select_source(
    track, now, translation_base_odom(), params, state);

  EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::ODOM);
  EXPECT_EQ(sel.reason, "odom");
  EXPECT_DOUBLE_EQ(sel.scale, 0.5);
  // p_odom = (4,5,6); T_base_odom adds (1,2,3) going base -> odom, so its
  // inverse subtracts it going odom -> base => (3,3,3).
  EXPECT_NEAR(sel.p_arm_base.x(), 3.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.y(), 3.0, 1e-9);
  EXPECT_NEAR(sel.p_arm_base.z(), 3.0, 1e-9);
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::ODOM);
}

// Priority 4: the odom fallback expires at > 1500ms (the spec uses <=, so
// exactly 1500ms is still usable) -> nothing usable, reason "stale".
TEST(GeometrySource, OdomExpiredReturnsNone) {
  const int64_t now = 1000000000LL;
  lgw_bt_planner::GeometrySourceParams params;
  lgw_bt_planner::SelectionState state;
  state.last = lgw_bt_planner::SourceKind::ODOM;

  const auto expired = lgw_bt_planner::select_source(
    make_snapshot({}, /*time_since_last_obs_ms=*/1600),
    now, translation_base_odom(), params, state);

  EXPECT_EQ(expired.kind, lgw_bt_planner::SourceKind::NONE);
  EXPECT_EQ(expired.reason, "stale");
  EXPECT_DOUBLE_EQ(expired.scale, 0.0);
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::NONE);
  EXPECT_EQ(state.upgrade_ticks, 0);

  // Boundary: 1500ms is still within odom_fallback_max_age_ms (<=).
  lgw_bt_planner::SelectionState boundary_state;
  const auto boundary = lgw_bt_planner::select_source(
    make_snapshot({}, /*time_since_last_obs_ms=*/1500),
    now, translation_base_odom(), params, boundary_state);
  EXPECT_EQ(boundary.kind, lgw_bt_planner::SourceKind::ODOM);
  EXPECT_EQ(boundary.reason, "odom");
}

// Upgrading back to a higher-priority source needs `upgrade_hysteresis_ticks`
// consecutive valid ticks (default 30). Starting on ARM with a fresh head:
// ticks 1..29 stay on ARM, tick 30 switches to HEAD.
TEST(GeometrySource, UpgradeNeedsHysteresis) {
  const int64_t t0 = 1000000000LL;
  lgw_bt_planner::GeometrySourceParams params;  // upgrade_hysteresis_ticks = 30
  lgw_bt_planner::SelectionState state;
  state.last = lgw_bt_planner::SourceKind::ARM;

  for (int tick = 1; tick <= 29; ++tick) {
    const int64_t now = t0 + tick * 10 * kNsPerMs;
    // Both sources stay fresh every tick, otherwise ARM would vanish and
    // the switch would be an immediate replacement, not a hysteresis upgrade.
    const auto track = make_snapshot(
      {
        make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 100 * kNsPerMs, {1.0, 0.0, 0.0}),
        make_contributor(lgw_bt_planner::SourceKind::ARM, now - 100 * kNsPerMs, {0.0, 1.0, 0.0}),
      },
      /*time_since_last_obs_ms=*/100);

    const auto sel = lgw_bt_planner::select_source(
      track, now, translation_base_odom(), params, state);

    EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::ARM) << "tick " << tick;
    EXPECT_EQ(sel.reason, "arm") << "tick " << tick;
    EXPECT_EQ(state.upgrade_ticks, tick) << "tick " << tick;
  }

  const int64_t now = t0 + 30 * 10 * kNsPerMs;
  const auto track = make_snapshot(
    {
      make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 100 * kNsPerMs, {1.0, 0.0, 0.0}),
      make_contributor(lgw_bt_planner::SourceKind::ARM, now - 100 * kNsPerMs, {0.0, 1.0, 0.0}),
    },
    /*time_since_last_obs_ms=*/100);

  const auto sel = lgw_bt_planner::select_source(
    track, now, translation_base_odom(), params, state);

  EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::HEAD);
  EXPECT_EQ(sel.reason, "head");
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::HEAD);
  EXPECT_EQ(state.upgrade_ticks, 0);
}

// Without T_base_odom the odom tier is unusable; both cameras stale ->
// NONE, even though the track itself is recent.
TEST(GeometrySource, MissingTransformDisablesOdom) {
  const int64_t now = 1000000000LL;
  const auto track = make_snapshot(
    {
      make_contributor(lgw_bt_planner::SourceKind::HEAD, now - 900 * kNsPerMs, {9.0, 9.0, 9.0}),
      make_contributor(lgw_bt_planner::SourceKind::ARM, now - 900 * kNsPerMs, {8.0, 8.0, 8.0}),
    },
    /*time_since_last_obs_ms=*/100);
  lgw_bt_planner::GeometrySourceParams params;
  lgw_bt_planner::SelectionState state;
  state.last = lgw_bt_planner::SourceKind::HEAD;

  const auto sel = lgw_bt_planner::select_source(
    track, now, std::nullopt, params, state);

  EXPECT_EQ(sel.kind, lgw_bt_planner::SourceKind::NONE);
  EXPECT_EQ(sel.reason, "stale");
  EXPECT_DOUBLE_EQ(sel.scale, 0.0);
  EXPECT_EQ(state.last, lgw_bt_planner::SourceKind::NONE);
}
