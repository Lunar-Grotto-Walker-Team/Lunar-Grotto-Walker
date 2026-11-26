#pragma once

#include <cmath>
#include <deque>

namespace simple_loop_closure {

// A timestamped sibling of /lf/lowstate.  LowState has no header, but
// /lf/sportmodestate is published in the same 20 Hz control cycle and does
// carry a device stamp.  Pairing the lowstate arrival with the nearest
// sportmodestate arrival gives the lowstate a device-clock timestamp that is
// immune to Jetson callback jitter.  The robot clock differs from Jetson's by
// a constant offset, which cancels because the integrator only uses
// differences.
struct LegTimeSample {
  double arrival{0.0};       // Jetson arrival time
  double device_stamp{0.0};  // robot-side header stamp
};

inline bool nearestLegTimeSample(const std::deque<LegTimeSample>& buf,
                                 double arrival, double timeout,
                                 LegTimeSample& out) {
  if (!std::isfinite(arrival)) {
    return false;
  }
  double best = 1e9;
  const LegTimeSample* best_s = nullptr;
  for (const auto& s : buf) {
    if (!std::isfinite(s.device_stamp)) {
      continue;
    }
    const double d = std::abs(s.arrival - arrival);
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

// LowState is published ~15 ms before the sportmodestate of the same cycle
// (measured: median 15.4 ms, p10/p90 +-0.15 ms on the corridor bag).  Shift
// the sport device stamp by the arrival delta to recover the lowstate's
// device time; this matches the reference's lowstate receive-time pairing.
inline double legLowStateDeviceStamp(const LegTimeSample& pair,
                                     double lowstate_arrival) {
  return pair.device_stamp + (lowstate_arrival - pair.arrival);
}

}  // namespace simple_loop_closure
