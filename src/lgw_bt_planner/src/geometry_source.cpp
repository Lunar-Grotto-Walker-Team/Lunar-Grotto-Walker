// src/geometry_source.cpp
// Pure three-tier geometry-source selection for the MoveIt Servo takeover
// stack (spec 2026-10-26 §7.4). No rclcpp / ROS runtime / BT / Eigen — see
// include/lgw_bt_planner/geometry_source.hpp for the full rule set.

#include "lgw_bt_planner/geometry_source.hpp"

namespace lgw_bt_planner
{
namespace
{

constexpr int64_t kNanosecondsPerMillisecond = 1000000;

// Lower rank = higher priority. NONE ranks last so it never wins a
// comparison against a real tier.
int priority_rank(SourceKind kind)
{
  switch (kind) {
    case SourceKind::HEAD:
      return 0;
    case SourceKind::ARM:
      return 1;
    case SourceKind::ODOM:
      return 2;
    case SourceKind::NONE:
    default:
      return 3;
  }
}

// Freshest contributor of `kind` whose capture age is strictly below
// `max_age_ms`; nullptr if none qualifies. Contributor stamps may be in the
// future under clock skew — a negative age is treated as fresh.
const Contributor* find_fresh(
  const std::vector<Contributor>& contributors, SourceKind kind,
  int64_t now_ns, int64_t max_age_ms)
{
  const int64_t max_age_ns = max_age_ms * kNanosecondsPerMillisecond;
  const Contributor* freshest = nullptr;
  for (const Contributor& c : contributors) {
    if (c.kind != kind) {
      continue;
    }
    if (now_ns - c.stamp_ns >= max_age_ns) {
      continue;
    }
    if (freshest == nullptr || c.stamp_ns > freshest->stamp_ns) {
      freshest = &c;
    }
  }
  return freshest;
}

}  // namespace

SourceSelection select_source(
  const TrackSnapshot& track,
  int64_t now_ns,
  const std::optional<tf2::Transform>& T_base_odom,
  const GeometrySourceParams& params,
  SelectionState& state)
{
  const Contributor* head =
    find_fresh(track.contributors, SourceKind::HEAD, now_ns, params.head_max_age_ms);
  const Contributor* arm =
    find_fresh(track.contributors, SourceKind::ARM, now_ns, params.arm_max_age_ms);
  // Spec §7.4: inclusive (<=) so a track at the tracker's 1500ms deletion
  // horizon is still usable on this tick.
  const bool odom_usable = T_base_odom.has_value() &&
    track.time_since_last_obs_ms <= params.odom_fallback_max_age_ms;

  SourceKind best = SourceKind::NONE;
  if (head != nullptr) {
    best = SourceKind::HEAD;
  } else if (arm != nullptr) {
    best = SourceKind::ARM;
  } else if (odom_usable) {
    best = SourceKind::ODOM;
  }

  auto build = [&](SourceKind kind) {
      SourceSelection sel;
      sel.kind = kind;
      switch (kind) {
        case SourceKind::HEAD:
          sel.p_arm_base = head->p_arm_base;
          sel.scale = params.head_scale;
          sel.reason = "head";
          break;
        case SourceKind::ARM:
          sel.p_arm_base = arm->p_arm_base;
          sel.scale = params.arm_scale;
          sel.reason = "arm";
          break;
        case SourceKind::ODOM:
          // T_base_odom is lookupTransform(odom <- base_link) (base -> odom),
          // so odom -> base needs its inverse.
          sel.p_arm_base = T_base_odom->inverse() * track.p_odom;
          sel.scale = params.odom_scale;
          sel.reason = "odom";
          break;
        case SourceKind::NONE:
        default:
          sel.p_arm_base = tf2::Vector3(0.0, 0.0, 0.0);
          sel.scale = 0.0;
          sel.reason = "stale";
          break;
      }
      return sel;
    };

  if (best == SourceKind::NONE) {
    state.last = SourceKind::NONE;
    state.upgrade_ticks = 0;
    return build(SourceKind::NONE);
  }

  // First tick, or resuming after a dropout: no tier to protect.
  if (state.last == SourceKind::NONE) {
    state.last = best;
    state.upgrade_ticks = 0;
    return build(best);
  }

  const int best_rank = priority_rank(best);
  const int last_rank = priority_rank(state.last);

  // Same tier, or a downgrade (higher rank number = lower priority):
  // adopt immediately — the safety direction must never be delayed.
  if (best_rank >= last_rank) {
    state.last = best;
    state.upgrade_ticks = 0;
    return build(best);
  }

  // Upgrade candidate. Hysteresis only makes sense while the tier we are
  // still serving is itself usable; if it just vanished, holding it would
  // mean emitting geometry we no longer trust, so switch right away.
  const bool last_usable =
    (state.last == SourceKind::HEAD && head != nullptr) ||
    (state.last == SourceKind::ARM && arm != nullptr) ||
    (state.last == SourceKind::ODOM && odom_usable);
  if (!last_usable) {
    state.last = best;
    state.upgrade_ticks = 0;
    return build(best);
  }

  ++state.upgrade_ticks;
  if (state.upgrade_ticks >= params.upgrade_hysteresis_ticks) {
    state.last = best;
    state.upgrade_ticks = 0;
    return build(best);
  }
  return build(state.last);
}

}  // namespace lgw_bt_planner
