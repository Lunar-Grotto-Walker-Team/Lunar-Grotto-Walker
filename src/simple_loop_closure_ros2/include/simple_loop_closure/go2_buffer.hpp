#pragma once

#include <cmath>
#include <deque>
#include <limits>

#include <Eigen/Geometry>

namespace simple_loop_closure {

struct Go2Sample {
  double stamp;  // arrival time (Jetson clock)
  double z;
  Eigen::Quaterniond q;
  // Message header time (robot clock).  Used by the leg-z attitude pairing,
  // which must not depend on Jetson callback jitter.  NaN when unavailable.
  double device_stamp{std::numeric_limits<double>::quiet_NaN()};
};

// A zero/NaN quaternion must never reach the attitude prior: invalid
// samples are stored with a NaN w and skipped by the factor.
inline bool validQuaternion(const Eigen::Quaterniond& q) {
  const double n = q.norm();
  return std::isfinite(n) && n > 1e-6 && q.coeffs().allFinite();
}

// Nearest sample within `timeout`.  The caller owns the buffer lock; this
// helper is pure so it can be unit-tested without a ROS node.
inline bool nearestGo2Sample(const std::deque<Go2Sample>& buf, double stamp,
                             double timeout, Go2Sample& out) {
  if (buf.empty()) {
    return false;
  }
  double best = 1e9;
  const Go2Sample* best_s = nullptr;
  for (const auto& s : buf) {
    const double d = std::abs(s.stamp - stamp);
    if (d < best) {
      best = d;
      best_s = &s;
    }
  }
  if (best_s == nullptr || best > timeout) {
    return false;
  }
  out = *best_s;
  return true;
}

// Nearest sample by the message header time (robot clock).  Used by the leg-z
// attitude pairing so it is independent of Jetson callback jitter.
inline bool nearestGo2SampleByDevice(const std::deque<Go2Sample>& buf,
                                     double device_stamp, double timeout,
                                     Go2Sample& out) {
  if (!std::isfinite(device_stamp)) {
    return false;
  }
  double best = 1e9;
  const Go2Sample* best_s = nullptr;
  for (const auto& s : buf) {
    if (!std::isfinite(s.device_stamp)) {
      continue;
    }
    const double d = std::abs(s.device_stamp - device_stamp);
    if (d < best) {
      best = d;
      best_s = &s;
    }
  }
  if (best_s == nullptr || best > timeout) {
    return false;
  }
  out = *best_s;
  return true;
}

}  // namespace simple_loop_closure
