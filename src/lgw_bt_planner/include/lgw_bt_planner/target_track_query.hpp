// include/lgw_bt_planner/target_track_query.hpp
//
// Pure queries over the tracker's UnifiedTrackArray (2026-11-17).
//
// The BT shells in target_track_nodes.cpp only subscribe and poll; every
// decision lives here so it can be unit-tested without ROS.
#pragma once

#include <yoloe_interfaces/msg/unified_track.hpp>
#include <yoloe_interfaces/msg/unified_track_array.hpp>

#include <cmath>
#include <cstdint>

namespace lgw_bt_planner
{
namespace track_query
{

inline const yoloe_interfaces::msg::UnifiedTrack * find_track(
  const yoloe_interfaces::msg::UnifiedTrackArray & array, uint64_t track_id)
{
  if (track_id == 0) {
    return nullptr;
  }
  for (const auto & track : array.tracks) {
    if (track.track_id == track_id) {
      return &track;
    }
  }
  return nullptr;
}

inline const yoloe_interfaces::msg::UnifiedTrack * selected_track(
  const yoloe_interfaces::msg::UnifiedTrackArray & array)
{
  return find_track(array, array.selected_track_id);
}

// `position_arm_base` is already expressed in arm1/base_link; the XoY norm is
// the planar distance to the arm base origin.
inline double arm_base_xy_m(const yoloe_interfaces::msg::UnifiedTrack & track)
{
  return std::hypot(
    static_cast<double>(track.position_arm_base.x),
    static_cast<double>(track.position_arm_base.y));
}

inline bool is_fresh(
  const yoloe_interfaces::msg::UnifiedTrack & track, int max_age_ms)
{
  return track.time_since_last_observation_ms <= max_age_ms;
}

inline bool xy_in_band(
  const yoloe_interfaces::msg::UnifiedTrack & track,
  double min_xy_m, double max_xy_m)
{
  if (!track.geometry_valid) {
    return false;
  }
  const double xy = arm_base_xy_m(track);
  return xy >= min_xy_m && xy <= max_xy_m;
}

// 目标相对臂基座 +X 轴的方位角(rad, 左右对称, 取值 [0, pi])。
// 角度来自 XoY 分量, 与 arm_base_xy_m 同一参照。
//
// 2026-11-20: 这个 abs() 版本**丢掉左右**。历史上只用于"正前方 ±15°"这种
// 对称判据, 所以看不出问题; 一旦扇区中心不在 0, 它就无法表达"左侧/右侧"
// (实测: 目标在右侧 y<0 时仍打印 +45°)。需要方向语义时一律用
// arm_base_azimuth_signed_rad()。
inline double arm_base_azimuth_rad(
  const yoloe_interfaces::msg::UnifiedTrack & track)
{
  return std::abs(std::atan2(
    static_cast<double>(track.position_arm_base.y),
    static_cast<double>(track.position_arm_base.x)));
}

// 带符号的方位角(rad, 取值 (-pi, pi]): +Y 为正, -Y 为负。
// in_front_sector() 用它, 因此扇区中心可以是任意方位(例如 -pi/2 = -Y)。
inline double arm_base_azimuth_signed_rad(
  const yoloe_interfaces::msg::UnifiedTrack & track)
{
  return std::atan2(
    static_cast<double>(track.position_arm_base.y),
    static_cast<double>(track.position_arm_base.x));
}

// 2026-11-19: 目标必须在基座正前方 +-half_angle_rad 的扇区内。
// 与 xy_in_band 一起构成接近判据: 只有"距离够近且在正前方"才进验证,
// 避免机械臂往侧后方伸(那一片姿态很容易撞限位/奇异)。
// Sector test around an arbitrary azimuth (2026-11-20). center_rad defaults
// to 0 = the arm base +X axis (historical behaviour); e.g. -pi/2 centres the
// sector on the -Y axis. The offset is wrapped into [-pi, pi] first, so a
// sector straddling the +-180 deg seam behaves correctly.
inline bool in_front_sector(
  const yoloe_interfaces::msg::UnifiedTrack & track, double half_angle_rad,
  double center_rad = 0.0)
{
  if (half_angle_rad <= 0.0) {
    return true;  // <=0: 关闭该判据
  }
  if (!track.geometry_valid) {
    return false;
  }
  double d = arm_base_azimuth_signed_rad(track) - center_rad;
  while (d > M_PI) {
    d -= 2.0 * M_PI;
  }
  while (d < -M_PI) {
    d += 2.0 * M_PI;
  }
  return std::fabs(d) <= half_angle_rad;
}

}  // namespace track_query
}  // namespace lgw_bt_planner
