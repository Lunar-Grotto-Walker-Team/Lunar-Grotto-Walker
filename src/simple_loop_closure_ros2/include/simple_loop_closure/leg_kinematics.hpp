#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>

namespace simple_loop_closure {

// Go2 leg geometry, from go2_description.urdf:
//   hip origin (0.1934, 0.0465, 0), thigh lateral offset 0.0955,
//   thigh and calf links 0.213 each.
struct LegGeometry {
  double offset_x{0.1934};
  double offset_y{0.0465};
  double thigh_offset{0.0955};
  double thigh_len{0.213};
  double calf_len{0.213};
};

// leg: 0=FL, 1=FR, 2=RL, 3=RR. Joint order: hip roll, thigh, calf.
// Output is the foot position in the URDF body frame (x forward, y left,
// z up).  A standing leg gives z ~= -0.29 m.
inline Eigen::Vector3d legFootPosition(const LegGeometry& g, int leg,
                                       double q1, double q2, double q3) {
  const double fx = (leg == 0 || leg == 1) ? 1.0 : -1.0;   // front/back
  const double fy = (leg == 0 || leg == 2) ? 1.0 : -1.0;   // left/right

  const double s1 = std::sin(q1), c1 = std::cos(q1);
  const double s2 = std::sin(q2), c2 = std::cos(q2);
  const double s23 = std::sin(q2 + q3), c23 = std::cos(q2 + q3);

  const double x = fx * g.offset_x - g.thigh_len * s2 - g.calf_len * s23;
  const double y = fy * g.offset_y + fy * g.thigh_offset * c1 +
                   (g.thigh_len * c2 + g.calf_len * c23) * s1;
  const double z = fy * g.thigh_offset * s1 -
                   (g.thigh_len * c2 + g.calf_len * c23) * c1;
  return Eigen::Vector3d(x, y, z);
}

// Schmitt trigger on foot force: latch contact above `force_on`, clear
// below `force_off`.  `contactStart` is the latch time, used by the
// single-foot settle gate.
class LegContactDetector {
 public:
  LegContactDetector(double force_on, double force_off)
      : force_on_(force_on), force_off_(force_off) {}

  const std::array<bool, 4>& update(double stamp,
                                    const std::array<double, 4>& force) {
    for (int i = 0; i < 4; ++i) {
      if (!contact_[i] && force[i] > force_on_) {
        contact_[i] = true;
        contact_start_[i] = stamp;
      } else if (contact_[i] && force[i] < force_off_) {
        contact_[i] = false;
      }
    }
    return contact_;
  }

  const std::array<bool, 4>& contact() const { return contact_; }
  const std::array<double, 4>& contactStart() const { return contact_start_; }

 private:
  double force_on_;
  double force_off_;
  std::array<bool, 4> contact_{{false, false, false, false}};
  std::array<double, 4> contact_start_{{0.0, 0.0, 0.0, 0.0}};
};

}  // namespace simple_loop_closure
