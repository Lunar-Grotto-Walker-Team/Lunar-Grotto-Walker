#pragma once

#include <cmath>
#include <deque>

#include <Eigen/Geometry>

namespace simple_loop_closure {

// Source tags, used by gravity_source=auto to prefer lowstate and fall back
// to livox when the window has no lowstate sample (bag replay).
enum GravitySourceId {
  kGravityLowstate = 0,
  kGravityUtlidar = 1,
  kGravityLivox = 2,
};

// One IMU sample reduced to a specific-force vector (accelerometer reading,
// "up" at rest) in the frame of its source topic.  `q` is only filled for
// sources that publish an orientation (lowstate).
struct GravitySample {
  double stamp{0.0};
  Eigen::Vector3d specific_force{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  bool has_q{false};
  int source{kGravityLowstate};
};

inline bool validGravitySample(const GravitySample& s) {
  return s.specific_force.allFinite() && s.specific_force.norm() > 1e-3;
}

// Nearest sample within `timeout`.  Invalid (NaN / zero) samples are skipped
// so a single bad IMU frame cannot poison the prior.  The caller owns the
// buffer lock; pure so it can be unit-tested without a ROS node.
inline bool nearestGravitySample(const std::deque<GravitySample>& buf,
                                 double stamp, double timeout,
                                 GravitySample& out, int source = -1) {
  double best = 1e9;
  const GravitySample* best_s = nullptr;
  for (const auto& s : buf) {
    if (!validGravitySample(s)) {
      continue;
    }
    if (source >= 0 && s.source != source) {
      continue;
    }
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

// Mean specific force over [stamp - half_window, stamp + half_window].
// The accelerometer under locomotion swings by tens of degrees within one
// gait cycle (measured on the slope bag: instantaneous pitch std 8.3 deg,
// range -67..+39), so a single sample is not a gravity measurement.  A
// 0.30 s window brings the pitch std to 2.9 deg while still tracking a ramp.
inline bool averageGravitySample(const std::deque<GravitySample>& buf,
                                 double stamp, double half_window,
                                 GravitySample& out, int source = -1) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  for (const auto& s : buf) {
    if (!validGravitySample(s)) {
      continue;
    }
    if (source >= 0 && s.source != source) {
      continue;
    }
    if (std::abs(s.stamp - stamp) > half_window) {
      continue;
    }
    sum += s.specific_force;
    ++n;
    if (s.has_q && !out.has_q) {
      out.q = s.q;
      out.has_q = true;
    }
  }
  if (n == 0) {
    return false;
  }
  out.stamp = stamp;
  out.specific_force = sum / static_cast<double>(n);
  out.source = source >= 0 ? source : out.source;
  return true;
}

// Dynamic gate for the gravity prior.  A stair/jump impact lasts 0.1-0.3 s,
// so even the 0.30 s mean specific force deviates by 20-40% in magnitude;
// using it injects a bogus attitude correction and makes iSAM2 repeatedly
// re-linearise the whole graph (seen on the park stairs).  Reject windows
// whose magnitude is far from the running norm.  tol <= 0 disables the gate;
// ema <= 0 means it is not initialised yet.
inline bool gravityNormAccepted(double norm, double ema, double tol) {
  if (!(tol > 0.0) || !(ema > 0.0) || !std::isfinite(norm)) {
    return true;
  }
  return std::abs(norm / ema - 1.0) <= tol;
}

}  // namespace simple_loop_closure
