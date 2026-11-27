// src/lookat_standoff_setpoint.cpp
// Target-relative lookat standoff (spec 2026-10-29). See the header for the
// distance-definition rationale and for why the legacy base-relative module is
// left untouched.

#include "lgw_bt_planner/lookat_standoff_setpoint.hpp"

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <algorithm>
#include <cmath>

namespace lgw_bt_planner
{
namespace
{

constexpr double kEps = 1e-9;
constexpr double kMinLookDistance = 1e-3;
// Angular resolution of the elevation search. The search only runs when the
// current bearing violates a constraint, and stops at the first feasible
// elevation, so a healthy tick costs a single evaluation.
constexpr double kElevationStep = 0.02;

tf2::Matrix3x3 roll_about_optical_axis(const tf2::Matrix3x3& R, double roll)
{
  if (roll == 0.0) {
    return R;
  }
  const double c = std::cos(roll);
  const double s = std::sin(roll);
  // Row-major rotation about +z. Applied on the right, so column 2 (the
  // optical axis) is untouched and the target stays exactly centred.
  const tf2::Matrix3x3 Rz(c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0);
  return R * Rz;
}

// Camera pose in base for a chosen camera origin, aiming at the target.
// Returns false when the look direction is undefined (target == camera).
bool build_tcp_pose(
  const tf2::Vector3& p_cam_base,
  const tf2::Vector3& p_target_base,
  const tf2::Matrix3x3& R_cam_in_tcp,
  const tf2::Vector3& t_cam_in_tcp,
  double roll,
  Pose6& out)
{
  const tf2::Vector3 delta = p_target_base - p_cam_base;
  if (delta.length() < kMinLookDistance) {
    return false;
  }
  const tf2::Vector3 direction = delta.normalized();
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  const tf2::Matrix3x3 R_cam_in_base = roll_about_optical_axis(
    apply_uprightness_flip(
      build_lookat_orientation(direction, world_up), world_up),
    roll);

  const geometry_msgs::msg::PoseStamped tcp_pose = compute_tcp_pose(
    p_cam_base, R_cam_in_base, R_cam_in_tcp, t_cam_in_tcp);

  out.p.setValue(
    tcp_pose.pose.position.x,
    tcp_pose.pose.position.y,
    tcp_pose.pose.position.z);
  tf2::Quaternion q(
    tcp_pose.pose.orientation.x,
    tcp_pose.pose.orientation.y,
    tcp_pose.pose.orientation.z,
    tcp_pose.pose.orientation.w);
  q.normalize();
  out.R = tf2::Matrix3x3(q);
  return true;
}

}  // namespace

const char* standoff_status_name(StandoffStatus status)
{
  switch (status) {
    case StandoffStatus::kOk:
      return "ok";
    case StandoffStatus::kReachClamped:
      return "reach_clamped";
    case StandoffStatus::kHeldLast:
      return "held_last";
    case StandoffStatus::kDegenerate:
      return "degenerate";
    default:
      return "unknown";
  }
}

StandoffResult compute_standoff_setpoint(
  const tf2::Vector3& p_target_base,
  const tf2::Vector3& p_cam_now_base,
  double dt_s,
  const tf2::Matrix3x3& R_cam_in_tcp,
  const tf2::Vector3& t_cam_in_tcp,
  const StandoffParams& params,
  StandoffState& state)
{
  StandoffResult out;

  auto emit_pose = [&](const tf2::Vector3& p_cam, StandoffStatus status) {
      Pose6 pose;
      if (build_tcp_pose(
          p_cam, p_target_base, R_cam_in_tcp, t_cam_in_tcp,
          params.camera_roll_rad, pose))
      {
        out.tcp_desired = pose;
        out.status = status;
        state.last_tcp_desired = pose;
        state.has_last_pose = true;
      } else if (state.has_last_pose) {
        // Look direction undefined this tick: repeat the previous pose rather
        // than emitting a NaN orientation to Servo.
        out.tcp_desired = state.last_tcp_desired;
        out.status = StandoffStatus::kDegenerate;
      } else {
        out.status = StandoffStatus::kDegenerate;
      }
      out.achieved_standoff_m = (p_cam - p_target_base).length();
      out.reach_m = p_cam.length();
      return out;
    };

  auto emit_last_feasible = [&](StandoffStatus fallback_status) {
      if (state.has_last_feasible) {
        return emit_pose(state.last_p_cam_base, StandoffStatus::kHeldLast);
      }
      return emit_pose(p_cam_now_base, fallback_status);
    };

  // --- 1. Bearing: unit vector FROM the target TO the camera, so the camera
  // origin is p_target + d * u. Keeping the current bearing is what stops the
  // camera from swinging around the target when the platform translates.
  tf2::Vector3 u = p_cam_now_base - p_target_base;
  if (u.length() < kMinLookDistance) {
    u = params.fallback_view_dir;
    out.used_fallback_view = true;
  }
  if (u.length() < kMinLookDistance) {
    return emit_last_feasible(StandoffStatus::kDegenerate);
  }
  u.normalize();

  const tf2::Vector3 delta = p_target_base;  // base origin is (0,0,0)
  const double azimuth = std::atan2(u.y(), u.x());
  const double sin_el_start = std::max(-1.0, std::min(1.0, u.z()));

  const auto dir_for = [azimuth](double sin_elevation) {
      const double c = std::sqrt(std::max(0.0, 1.0 - sin_elevation * sin_elevation));
      return tf2::Vector3(
        c * std::cos(azimuth), c * std::sin(azimuth), sin_elevation);
    };

  // Candidate evaluation at one elevation: solve the standoff that puts the
  // reach at its preferred value, then check every constraint. The standoff
  // band is applied before the checks so a clamped standoff that violates
  // reach/height is rejected rather than silently emitted.
  struct Candidate
  {
    bool feasible{false};
    bool clamped{false};
    double standoff_m{0.0};
    tf2::Vector3 p_cam;
  };

  const auto evaluate = [&](double sin_el) {
      Candidate cand;
      const tf2::Vector3 ui = dir_for(sin_el);
      const double along = delta.dot(ui);
      const tf2::Vector3 perp = delta - ui * along;
      const double r_perp = perp.length();  // closest this ray gets to the base
      if (r_perp > params.reach_max_m + kEps) {
        return cand;
      }
      // |delta + d*ui|^2 = r^2  =>  d = -along - sqrt(r^2 - r_perp^2)
      const double r_lo = std::max(params.reach_min_m, r_perp);
      const double r_goal = std::min(
        std::max(params.reach_pref_m, r_lo), params.reach_max_m);
      const bool reach_limited =
        std::fabs(r_goal - params.reach_pref_m) > kEps;

      // Both roots of the quadratic put the camera on the reach sphere; take
      // the SMALLEST POSITIVE one, i.e. the station on the base side of the
      // target rather than the one beyond it.
      const double root = std::sqrt(
        std::max(0.0, r_goal * r_goal - r_perp * r_perp));
      const double d_low = -along - root;
      const double d_high = -along + root;
      double d = (d_low > kEps) ? d_low : d_high;
      if (!std::isfinite(d) || d <= kEps) {
        return cand;
      }
      const double d_clamped = std::min(
        std::max(d, params.standoff_min_m), params.standoff_max_m);
      const bool standoff_clamped = std::fabs(d_clamped - d) > kEps;

      const tf2::Vector3 p_cam = p_target_base + ui * d_clamped;
      const double reach = p_cam.length();
      if (reach > params.reach_max_m + kEps ||
        reach < params.reach_min_m - kEps ||
        p_cam.z() < params.cam_min_height_m - kEps)
      {
        return cand;
      }

      cand.feasible = true;
      cand.clamped = reach_limited || standoff_clamped;
      cand.standoff_m = d_clamped;
      cand.p_cam = p_cam;
      return cand;
    };

  // --- 2. Prefer the current bearing; if it violates the floor or the shell,
  // raise the elevation until something fits. Raising the elevation is the
  // only legal way to clear the floor without breaking the standoff, which is
  // why this is a search rather than a z post-clamp.
  Candidate chosen = evaluate(sin_el_start);
  double sin_el = sin_el_start;
  if (!chosen.feasible) {
    for (double s = sin_el_start + kElevationStep; s <= 1.0 + kEps;
      s += kElevationStep)
    {
      chosen = evaluate(s);
      if (chosen.feasible) {
        sin_el = s;
        chosen.clamped = true;  // the bearing had to move
        break;
      }
    }
  }
  if (!chosen.feasible) {
    return emit_last_feasible(StandoffStatus::kHeldLast);
  }

  out.requested_standoff_m = chosen.standoff_m;

  // --- 3. Slew limit. Feasibility wins over smoothness: if limiting the
  // standoff would leave the shell or dip under the floor, keep the feasible
  // value for this tick instead.
  double standoff = chosen.standoff_m;
  tf2::Vector3 p_cam = chosen.p_cam;
  bool clamped = chosen.clamped;
  if (state.has_last_standoff && dt_s > 0.0 &&
    params.standoff_rate_limit_mps > 0.0)
  {
    const double max_step = params.standoff_rate_limit_mps * dt_s;
    const double limited = std::min(
      std::max(
        std::min(
          std::max(standoff, state.last_standoff_m - max_step),
          state.last_standoff_m + max_step),
        params.standoff_min_m),
      params.standoff_max_m);
    if (std::fabs(limited - standoff) > kEps) {
      const tf2::Vector3 p_limited =
        p_target_base + dir_for(sin_el) * limited;
      if (p_limited.length() <= params.reach_max_m + kEps &&
        p_limited.length() >= params.reach_min_m - kEps &&
        p_limited.z() >= params.cam_min_height_m - kEps)
      {
        standoff = limited;
        p_cam = p_limited;
        clamped = true;
      }
    }
  }

  state.last_p_cam_base = p_cam;
  state.has_last_feasible = true;
  state.last_standoff_m = standoff;
  state.has_last_standoff = true;

  return emit_pose(
    p_cam, clamped ? StandoffStatus::kReachClamped : StandoffStatus::kOk);
}

}  // namespace lgw_bt_planner
