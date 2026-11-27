// src/target_selector.cpp
// Pure target selection; see target_selector.hpp for the rule set.

#include "lgw_bt_planner/target_selector.hpp"

namespace lgw_bt_planner
{

TargetSelection select_target(
  const std::vector<TrackCandidate>& candidates,
  const std::optional<tf2::Vector3>& p_base_in_odom,
  const TargetSelectorParams& params,
  TargetSelectorState& state)
{
  TargetSelection out;
  size_t index = candidates.size();

  if (params.requested_track_id != 0) {
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].track_id == params.requested_track_id) {
        index = i;
        out.reason = "requested";
        break;
      }
    }
    if (index == candidates.size()) {
      out.reason = "requested_missing";
      return out;
    }
  } else {
    if (!p_base_in_odom) {
      out.reason = "no_base_pose";
      return out;
    }
    const auto eligible = [&](const TrackCandidate & c) {
        if (!params.class_filter.empty() && c.class_name != params.class_filter) {
          return false;
        }
        if (c.task_state != "active") {
          return false;
        }
        // Distance band: near-field tracks are usually the robot's own body
        // or a depth false positive (chasing them swings the arm across the
        // base); far ones are too coarse to aim at and drag the wrist onto
        // its limits. Served band is [min_distance_m, max_distance_m]
        // (0.3-1.5 m by default). max_distance_m <= 0 disables the ceiling.
        const double distance = (c.p_odom - *p_base_in_odom).length();
        if (distance < params.min_distance_m) {
          return false;
        }
        return params.max_distance_m <= 0.0 || distance <= params.max_distance_m;
      };

    // Keep the target we are already following while it stays eligible; only
    // a lost target lets another one take over (2026-11-15).
    if (state.last_track_id != 0) {
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].track_id == state.last_track_id &&
          eligible(candidates[i]))
        {
          index = i;
          out.reason = "current";
          break;
        }
      }
    }

    if (index == candidates.size()) {
      double best_distance = 0.0;
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (!eligible(candidates[i])) {
          continue;
        }
        const double distance =
          (candidates[i].p_odom - *p_base_in_odom).length();
        if (index == candidates.size() || distance < best_distance) {
          index = i;
          best_distance = distance;
        }
      }
      if (index != candidates.size()) {
        out.reason = "nearest";
      }
    }
    if (index == candidates.size()) {
      out.reason = "no_candidate";
      return out;
    }
  }

  const TrackCandidate& hit = candidates[index];
  out.ok = true;
  out.index = index;
  out.track_id = hit.track_id;
  state.last_track_id = hit.track_id;
  return out;
}

}  // namespace lgw_bt_planner
