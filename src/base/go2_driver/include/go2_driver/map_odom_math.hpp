// map_odom_math.hpp
//
// Pure geometry helper for map_odom_correction. Deliberately depends only on
// Eigen (no ROS / tf2) so it can be compiled and unit-tested with plain g++.
#pragma once

#include <Eigen/Geometry>

// Compute the dynamic map -> odom correction.
//
//   T_map_odom = T_map_body * (T_base_body)^-1 * (T_odom_base)^-1
//
// Parameter semantics (must match pointlio_cloud_relay's transform.py):
//   T_map_body  : map <- body          (PGO-corrected body pose, or raw LIO TF)
//   T_base_body : base <- body         (static installation extrinsic;
//                                       translation = body origin expressed in
//                                       the base frame)
//   T_odom_base : odom <- base_footprint (published by the Go2 driver)
//
// Chaining:
//   map <- odom = (map <- body)(body <- base)(base <- odom)
inline Eigen::Matrix4d computeMapOdom(
  const Eigen::Matrix4d & T_map_body,
  const Eigen::Matrix4d & T_base_body,
  const Eigen::Matrix4d & T_odom_base)
{
  return T_map_body * T_base_body.inverse() * T_odom_base.inverse();
}

// A PGO pose is usable only while it is recent. Once the publisher stops
// (or slows below the threshold), map_odom_correction must fall back to the
// raw LIO TF instead of broadcasting a frozen map->odom forever.
// Boundary: exactly at the timeout still counts as fresh.
inline bool isPgoFresh(double age_sec, double timeout_sec)
{
  return age_sec <= timeout_sec;
}
