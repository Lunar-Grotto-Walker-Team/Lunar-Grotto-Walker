// include/lgw_bt_planner/geometry_source.hpp
//
// Pure three-tier geometry-source selection for the MoveIt Servo takeover
// stack (spec 2026-10-26 §7.4). Once per control tick (100Hz) the Servo
// node asks: which camera/tracker tier can we trust right now, and where is
// the target in the arm base_link frame?
//
// Priority (highest first):
//   1. HEAD  — fresh head-camera contributor  (scale head_scale, default 1.0)
//   2. ARM   — fresh arm-camera contributor   (scale arm_scale,  0.7)
//   3. ODOM  — tracker KF back-projection     (scale odom_scale, 0.5)
//   4. NONE  — nothing usable -> HOLD         (reason "stale")
//
// Rules (spec §7.4):
//   * A contributor's age is `now_ns - contributor.stamp_ns`, i.e. measured
//     from the detection CAPTURE stamp — never from message-receipt time.
//     YOLOE inference alone costs 170-190ms, so a receipt-based age would
//     systematically understate staleness. Fresh means `age < max_age_ms`
//     (strict). A contributor stamped in the future counts as fresh.
//   * DOWNGRADE IS IMMEDIATE (safety direction must not be delayed): if the
//     best available tier is lower priority than the last-used tier, switch
//     on this tick.
//   * UPGRADE NEEDS HYSTERESIS: a higher-priority tier must stay valid for
//     `upgrade_hysteresis_ticks` consecutive ticks (default 30 ~= 300ms)
//     before we switch back. While counting, the previous tier keeps
//     serving. If the previous tier becomes invalid mid-count, there is
//     nothing to hold onto and the higher tier is adopted immediately.
//   * ODOM tier: `time_since_last_obs_ms <= odom_fallback_max_age_ms`
//     (inclusive; 1500ms by default, matching the tracker's deletion
//     horizon) AND a transform is available. `p_arm_base` is
//     `T_base_odom * p_odom`.
//   * Starting from NONE (first tick, or after a dropout), the best
//     available tier is adopted immediately; hysteresis only guards
//     switching BACK from a still-usable lower tier.
//
// Transform convention (spec §7.4, matches input_adapter.py):
// `T_base_odom` is `lookupTransform("odom", "base_link")` — it maps
// BASE-frame points into odom (p_odom = T_base_odom * p_base). To get odom
// points into the base frame this module INVERTS it:
// `p_arm_base = T_base_odom.inverse() * p_odom`. The servo node (Task 7)
// therefore passes `lookupTransform("odom", "base_link")` UNCHANGED — do not
// pre-invert it there.
//
// Pure C++17 + tf2 math only: no rclcpp, no ROS runtime, no BT, no Eigen.
// The servo node (Task 7) is responsible for building TrackSnapshot from
// UnifiedTrackArray + the per-camera geometry and for performing the TF
// lookup (with can_transform(0) first, to avoid a blocking miss).

#ifndef LGW_BT_PLANNER__GEOMETRY_SOURCE_HPP_
#define LGW_BT_PLANNER__GEOMETRY_SOURCE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// One detection contributing to this tick. `p_arm_base` is already expressed
// in the arm base_link frame; `stamp_ns` is the capture stamp of the
// detection (same clock as `now_ns` passed to select_source).
struct Contributor
{
  SourceKind kind{SourceKind::NONE};
  tf2::Vector3 p_arm_base;
  int64_t stamp_ns{0};
  float confidence{0.0f};   // informational; selection is by freshness only
};

// The followed track as flattened from the latest UnifiedTrackArray, plus
// this tick's per-camera contributors. `contributors` may hold zero, one or
// two entries (at most one HEAD and one ARM). If a kind appears twice, the
// freshest entry wins (higher `stamp_ns`).
struct TrackSnapshot
{
  uint64_t track_id{0};
  std::string status;
  tf2::Vector3 p_odom;                    // KF position in odom
  std::vector<Contributor> contributors;
  int64_t last_obs_stamp_ns{0};
  int64_t time_since_last_obs_ms{0};      // tracker's own field (odom tier)
};

struct GeometrySourceParams
{
  int64_t head_max_age_ms{400};
  int64_t arm_max_age_ms{400};
  int64_t odom_fallback_max_age_ms{1500};
  int upgrade_hysteresis_ticks{30};
  double head_scale{1.0};
  double arm_scale{0.7};
  double odom_scale{0.5};
};

// Memory carried across ticks: the tier used last tick and how many
// consecutive ticks the current upgrade candidate has been valid.
struct SelectionState
{
  SourceKind last{SourceKind::NONE};
  int upgrade_ticks{0};
};

// Pick the geometry source for this tick.
//
// Returns a fully-populated SourceSelection:
//   kind       winning tier (NONE = HOLD)
//   p_arm_base target position in arm base_link (zero vector for NONE)
//   scale      per-tier gain (0.0 for NONE)
//   reason     "head" / "arm" / "odom" / "stale"
//
// `T_base_odom` is `lookupTransform("odom", "base_link")` (base -> odom), so
// the odom tier uses its inverse to go odom -> base
// (`T_base_odom->inverse() * track.p_odom`); std::nullopt disables the odom
// tier. `state` is updated in place for the next tick.
SourceSelection select_source(
  const TrackSnapshot& track,
  int64_t now_ns,
  const std::optional<tf2::Transform>& T_base_odom,
  const GeometrySourceParams& params,
  SelectionState& state);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__GEOMETRY_SOURCE_HPP_
