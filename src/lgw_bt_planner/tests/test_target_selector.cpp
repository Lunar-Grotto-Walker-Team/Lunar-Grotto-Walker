// tests/test_target_selector.cpp
// Pure-function unit tests for select_target (2026-11-13 target-registry
// contract).  No ROS runtime needed.

#include "lgw_bt_planner/target_selector.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Vector3.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace
{

lgw_bt_planner::TrackCandidate mk(
  uint64_t track_id, const std::string& task_state,
  const std::string& class_name, const tf2::Vector3& p_odom)
{
  lgw_bt_planner::TrackCandidate c;
  c.track_id = track_id;
  c.task_state = task_state;
  c.class_name = class_name;
  c.p_odom = p_odom;
  return c;
}

}  // namespace

// An explicitly requested id always wins over proximity; its task_state is
// not consulted (the wire only carries active tracks).
TEST(TargetSelector, RequestedTrackWins)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(3, "active", "person", {1.0, 0.0, 0.0}),
    mk(7, "lost", "cup", {3.0, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.requested_track_id = 7;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.reason, "requested");
  EXPECT_EQ(sel.track_id, 7u);
  EXPECT_EQ(sel.index, 1u);
  EXPECT_EQ(state.last_track_id, 7u);
}

// Auto mode: nearest active track; a non-active candidate is skipped even
// when it is closer.
TEST(TargetSelector, AutoPicksNearestActive)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(5, "lost", "person", {0.5, 0.0, 0.0}),
    mk(7, "active", "person", {3.0, 0.0, 0.0}),
    mk(3, "active", "person", {1.0, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.reason, "nearest");
  EXPECT_EQ(sel.track_id, 3u);
  EXPECT_EQ(sel.index, 2u);
  EXPECT_EQ(state.last_track_id, 3u);
}

// A non-empty class_filter drops every other class before ranking.
TEST(TargetSelector, ClassFilterApplies)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(1, "active", "person", {0.5, 0.0, 0.0}),
    mk(2, "active", "cup", {2.0, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.class_filter = "cup";
  params.max_distance_m = 0.0;   // 本用例只测类别过滤, 关掉远场门限
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.reason, "nearest");
  EXPECT_EQ(sel.track_id, 2u);
  EXPECT_EQ(sel.index, 1u);

  lgw_bt_planner::TargetSelectorParams bottle_only;
  bottle_only.class_filter = "bottle";
  lgw_bt_planner::TargetSelectorState state2;
  const auto none = lgw_bt_planner::select_target(
    candidates, base, bottle_only, state2);
  EXPECT_FALSE(none.ok);
  EXPECT_EQ(none.reason, "no_candidate");
  EXPECT_EQ(state2.last_track_id, 0u);
}

// Requested id absent -> requested_missing; nothing else is substituted.
TEST(TargetSelector, MissingRequestedHolds)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(3, "active", "person", {1.0, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.requested_track_id = 9;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_FALSE(sel.ok);
  EXPECT_EQ(sel.reason, "requested_missing");
  EXPECT_EQ(state.last_track_id, 0u);
}

// Auto mode cannot rank without the robot's position in odom.
TEST(TargetSelector, AutoWithoutBasePoseFails)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(3, "active", "person", {1.0, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  const std::optional<tf2::Vector3> base = std::nullopt;
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_FALSE(sel.ok);
  EXPECT_EQ(sel.reason, "no_base_pose");
  EXPECT_EQ(state.last_track_id, 0u);
}

// Sticky selection (2026-11-15): while the tracked target is still active the
// servo keeps it, even when a different target has become nearer.  Switching
// happens only when the current target disappears.
TEST(TargetSelector, KeepsTheCurrentTargetWhileItStaysActive)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(4, "active", "wood", {2.0, 0.0, 0.0}),
    mk(9, "active", "wood", {0.8, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.max_distance_m = 0.0;   // 本用例只测粘性, 关掉远场门限(目标在 2.0 m)
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;
  state.last_track_id = 4;   // currently following 4

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.reason, "current");
  EXPECT_EQ(sel.track_id, 4u) << "a nearer target must not steal the lock";
}

TEST(TargetSelector, SwitchesToNearestWhenTheCurrentTargetIsGone)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(9, "active", "wood", {0.8, 0.0, 0.0}),
    mk(11, "active", "wood", {1.6, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;
  state.last_track_id = 4;   // 4 is no longer published

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.reason, "nearest");
  EXPECT_EQ(sel.track_id, 9u);
}

// Targets closer than min_distance_m to the base origin are ignored: they are
// usually the robot's own body or near-field false positives.
TEST(TargetSelector, IgnoresTargetsInsideTheMinimumDistance)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(2, "active", "rocka", {0.18, -0.08, -0.02}),   // 0.20 m
    mk(3, "active", "wood", {0.85, 0.10, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.min_distance_m = 0.3;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.track_id, 3u);

  const std::vector<lgw_bt_planner::TrackCandidate> only_close{
    mk(2, "active", "rocka", {0.18, -0.08, -0.02})};
  lgw_bt_planner::TargetSelectorState state2;
  const auto none = lgw_bt_planner::select_target(
    only_close, base, params, state2);
  EXPECT_FALSE(none.ok);
  EXPECT_EQ(none.reason, "no_candidate");
}

// The gate also applies to the sticky path: a target that drifted into the
// near field is dropped like any other.
TEST(TargetSelector, DropsTheCurrentTargetOnceItEntersTheNearField)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(4, "active", "wood", {0.20, 0.0, 0.0}),
    mk(9, "active", "wood", {1.2, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.min_distance_m = 0.3;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;
  state.last_track_id = 4;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);

  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.track_id, 9u);
}

// Far gate (2026-11-16): the served band is [min, max]; anything beyond max is
// dropped, and losing the only target leaves the selector empty (which sends
// the servo back to HOLD/SCANNING).
TEST(TargetSelector, IgnoresTargetsBeyondTheMaximumDistance)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(5, "active", "wood", {2.40, 0.0, 0.0}),    // 2.4 m — too far
    mk(6, "active", "wood", {1.10, 0.0, 0.0}),    // 1.1 m — inside the band
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.min_distance_m = 0.3;
  params.max_distance_m = 1.5;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);
  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.track_id, 6u);

  // Only the far one left: nothing is served.
  const std::vector<lgw_bt_planner::TrackCandidate> only_far{
    mk(5, "active", "wood", {2.40, 0.0, 0.0})};
  lgw_bt_planner::TargetSelectorState state2;
  const auto none = lgw_bt_planner::select_target(only_far, base, params, state2);
  EXPECT_FALSE(none.ok);
  EXPECT_EQ(none.reason, "no_candidate");
}

// The sticky path respects the ceiling too: a target that drifted past max is
// dropped like any other, and the next in-band one takes over.
TEST(TargetSelector, DropsTheCurrentTargetOnceItGoesBeyondTheMaximumDistance)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(7, "active", "wood", {1.80, 0.0, 0.0}),    // drifted past 1.5 m
    mk(8, "active", "wood", {1.20, 0.0, 0.0}),
  };
  lgw_bt_planner::TargetSelectorParams params;
  params.min_distance_m = 0.3;
  params.max_distance_m = 1.5;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;
  state.last_track_id = 7;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);
  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.track_id, 8u);
}

// max_distance_m <= 0 disables the ceiling (backwards compatible).
TEST(TargetSelector, ZeroMaximumDistanceDisablesTheFarGate)
{
  const std::vector<lgw_bt_planner::TrackCandidate> candidates{
    mk(9, "active", "wood", {3.20, 0.0, 0.0})};
  lgw_bt_planner::TargetSelectorParams params;
  params.min_distance_m = 0.3;
  params.max_distance_m = 0.0;
  const std::optional<tf2::Vector3> base = tf2::Vector3(0.0, 0.0, 0.0);
  lgw_bt_planner::TargetSelectorState state;

  const auto sel = lgw_bt_planner::select_target(candidates, base, params, state);
  EXPECT_TRUE(sel.ok);
  EXPECT_EQ(sel.track_id, 9u);
}
