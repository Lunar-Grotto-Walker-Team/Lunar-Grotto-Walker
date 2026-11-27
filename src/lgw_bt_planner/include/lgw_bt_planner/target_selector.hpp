// include/lgw_bt_planner/target_selector.hpp
//
// Pure target-selection logic for the MoveIt Servo takeover stack.
// Given the tracks of the latest UnifiedTrackArray as plain structs, decide
// which one to follow: an explicitly requested track id, or the active track
// nearest to the robot in odom (with an optional class filter).
//
// Reacquisition is gone: the tracker owns lost-track re-ID.  Servo only
// follows the published active list.
//
// Pure C++17 + tf2 math only: no rclcpp, no ROS runtime, no BT, no Eigen.

#ifndef LGW_BT_PLANNER__TARGET_SELECTOR_HPP_
#define LGW_BT_PLANNER__TARGET_SELECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/servo_types.hpp"

namespace lgw_bt_planner
{

// One track of the latest UnifiedTrackArray, flattened.
struct TrackCandidate
{
  uint64_t track_id{0};
  std::string task_state;          // "active" on the wire
  std::string class_name;
  tf2::Vector3 p_odom;
  int64_t last_seen_ns{0};
};

struct TargetSelectorParams
{
  uint64_t requested_track_id{0};  // 0 = auto
  std::string class_filter;        // empty = no filter
  // Candidates closer than this to the base origin are dropped: at that
  // range the "detection" is usually the robot's own body or a near-field
  // false positive (2026-11-15, seen as tray-height rock tracks).
  double min_distance_m{0.3};
  // Targets farther than this are dropped too (2026-11-16): beyond ~1.5 m the
  // 3D estimate is too coarse to point the wrist at, and chasing them drags
  // the arm onto its joint limits. <= 0 disables the upper bound.
  // With min 0.3 the served band is 0.3-1.5 m; anything outside leaves the
  // selector empty, which sends the servo back to HOLD/SCANNING.
  double max_distance_m{1.5};
};

// Diagnostic memory only: the last id the selector locked onto.
struct TargetSelectorState
{
  uint64_t last_track_id{0};
};

struct TargetSelection
{
  bool ok{false};
  size_t index{0};
  uint64_t track_id{0};
  std::string reason;              // requested / nearest /
                                   // requested_missing / no_candidate / no_base_pose
};

// Choose the track to follow for this control tick.
//
// requested_track_id != 0:
//   1. that id in candidates -> ok, reason="requested" (task_state NOT
//      checked: the publisher only sends active tracks);
//   2. otherwise ok=false, reason="requested_missing" (no auto-switch).
//
// requested_track_id == 0 (auto):
//   1. class_filter non-empty -> drop candidates whose class_name differs;
//   2. keep only task_state == "active";
//   3. drop candidates closer than min_distance_m to the base origin;
//   4. p_base_in_odom == nullopt -> ok=false, reason="no_base_pose";
//   5. state.last_track_id still eligible -> keep it, reason="current"
//      (sticky: a nearer target does NOT steal the lock);
//   6. otherwise the smallest |p_odom - *p_base_in_odom| -> reason="nearest";
//      empty set -> ok=false, reason="no_candidate".
TargetSelection select_target(
  const std::vector<TrackCandidate>& candidates,
  const std::optional<tf2::Vector3>& p_base_in_odom,
  const TargetSelectorParams& params,
  TargetSelectorState& state);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__TARGET_SELECTOR_HPP_
