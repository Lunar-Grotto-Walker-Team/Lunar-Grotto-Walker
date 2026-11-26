#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "simple_loop_closure/leg_kinematics.hpp"

namespace simple_loop_closure {

struct LegZConfig {
  double force_on{25.0};
  double force_off{12.0};
  double single_force{35.0};
  double settle_s{0.10};
  double consensus_spread{0.04};
  double max_step_dz{0.06};
  double pitch_offset_deg{1.3};
  double roll_offset_deg{0.0};
  double attitude_timeout{0.10};
  double attitude_jump_deg{10.0};
};

struct LegSample {
  double stamp{0.0};
  std::array<Eigen::Vector3d, 4> foot{};
  std::array<double, 4> force{};
};

struct LegAttitude {
  double stamp{0.0};
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  bool valid{false};
};

struct LegZDiagnostics {
  long steps_used{0};
  long steps_skipped{0};
  long skip_no_attitude{0};
  long skip_no_contact{0};
  long skip_consensus{0};
  long skip_big_dz{0};
  long skip_dt{0};
  long skip_non_finite{0};
  long skip_attitude_jump{0};
  double last_consensus_spread{0.0};
};

// A keyframe's leg-z snapshot is usable only when tracking is valid and at
// least one step was accepted since the previous keyframe.  A frozen z
// (missing attitude or a long no-contact/consensus gap) must never become
// a zero-dz factor, which would silently pin the graph's z.
inline bool legZKeyframeUsable(bool latest_valid, long steps_now,
                               long steps_prev) {
  return latest_valid && steps_now > steps_prev;
}

// Keyframe snapshot: NaN when the latest lowstate sample is invalid or stale.
inline double legZAtKeyframe(double latest_stamp, bool latest_valid,
                             double latest_z, double kf_stamp,
                             double timeout) {
  if (!latest_valid || !std::isfinite(latest_stamp) ||
      kf_stamp - latest_stamp > timeout) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return latest_z;
}

// Integrates world-z from foot anchors:
//   for a foot in contact at both endpoints, dp = R_prev*p_prev - R_cur*p_cur
//   and dz is the world-up component.  A low-pass/attitude error is absorbed
//   by the constant pitch/roll mount calibration.
class LegZIntegrator {
 public:
  explicit LegZIntegrator(const LegZConfig& cfg)
      : cfg_(cfg), contact_(cfg.force_on, cfg.force_off) {}

  // Returns true when this sample contributed a z step.
  bool update(const LegSample& s, const LegAttitude& att) {
    bool finite = std::isfinite(s.stamp);
    for (int i = 0; i < 4; ++i) {
      finite = finite && s.foot[i].allFinite() &&
               std::isfinite(s.force[i]);
    }
    if (!finite) {
      ++diag_.steps_skipped;
      ++diag_.skip_non_finite;
      // Drop the sample: storing NaN as prev would poison the next anchor.
      return false;
    }

    const auto& cur_contact = contact_.update(s.stamp, s.force);

    // Both endpoints need a valid attitude: an invalid previous R would
    // silently act as identity.
    if (!att.valid ||
        std::abs(att.stamp - s.stamp) > cfg_.attitude_timeout) {
      ++diag_.steps_skipped;
      ++diag_.skip_no_attitude;
      return finish(s, att, cur_contact, false);
    }
    if (have_prev_ && !prev_att_.valid) {
      ++diag_.steps_skipped;
      ++diag_.skip_no_attitude;
      return finish(s, att, cur_contact, false);
    }
    if (have_prev_ && prev_att_.valid) {
      const double jump_deg =
          Eigen::AngleAxisd(prev_att_.R.transpose() * att.R).angle() *
          180.0 / M_PI;
      if (jump_deg > cfg_.attitude_jump_deg) {
        ++diag_.steps_skipped;
        ++diag_.skip_attitude_jump;
        return finish(s, att, cur_contact, false);
      }
    }

    if (!have_prev_) {
      return finish(s, att, cur_contact, false);  // warm-up, not a skip
    }

    const double dt = s.stamp - prev_sample_.stamp;
    if (dt <= 0.005 || dt >= 0.12) {
      ++diag_.steps_skipped;
      ++diag_.skip_dt;
      return finish(s, att, cur_contact, false);
    }

    std::array<bool, 4> shared{};
    int n_shared = 0;
    for (int i = 0; i < 4; ++i) {
      shared[i] = cur_contact[i] && prev_contact_[i];
      if (shared[i]) ++n_shared;
    }
    if (n_shared == 0) {
      ++diag_.steps_skipped;
      ++diag_.skip_no_contact;
      return finish(s, att, cur_contact, false);
    }

    std::array<Eigen::Vector3d, 4> dp{};
    std::array<int, 4> used{};
    int n_used = 0;
    for (int i = 0; i < 4; ++i) {
      if (!shared[i]) continue;
      const Eigen::Vector3d p_prev = calibrate(prev_sample_.foot[i]);
      const Eigen::Vector3d p_cur = calibrate(s.foot[i]);
      dp[i] = prev_att_.R * p_prev - att.R * p_cur;
      used[n_used++] = i;
    }

    double dz = 0.0;
    bool ok = false;
    if (n_used >= 2) {
      double med[3];
      for (int a = 0; a < 3; ++a) {
        double vals[4];
        for (int k = 0; k < n_used; ++k) vals[k] = dp[used[k]][a];
        med[a] = median(vals, n_used);
      }
      double sum_z = 0.0;
      int kept = 0;
      double max_dist = 0.0;
      for (int k = 0; k < n_used; ++k) {
        const Eigen::Vector3d& d = dp[used[k]];
        const double dist =
            std::sqrt((d.x() - med[0]) * (d.x() - med[0]) +
                      (d.y() - med[1]) * (d.y() - med[1]) +
                      (d.z() - med[2]) * (d.z() - med[2]));
        max_dist = std::max(max_dist, dist);
        if (dist < cfg_.consensus_spread) {
          sum_z += d.z();
          ++kept;
        }
      }
      diag_.last_consensus_spread = max_dist;
      if (kept > 0) {
        dz = sum_z / kept;
        ok = true;
      }
    } else {
      const int li = used[0];
      const double age = s.stamp - contact_.contactStart()[li];
      if (age > cfg_.settle_s && s.force[li] > cfg_.single_force &&
          prev_sample_.force[li] > cfg_.single_force) {
        dz = dp[li].z();
        ok = true;
      }
    }

    if (!ok) {
      ++diag_.steps_skipped;
      ++diag_.skip_consensus;
      return finish(s, att, cur_contact, false);
    }
    if (std::abs(dz) > cfg_.max_step_dz) {
      ++diag_.steps_skipped;
      ++diag_.skip_big_dz;
      return finish(s, att, cur_contact, false);
    }

    z_ += dz;
    ++diag_.steps_used;
    return finish(s, att, cur_contact, true);
  }

  double z() const { return z_; }
  const LegZDiagnostics& diagnostics() const { return diag_; }

  void reset() {
    z_ = 0.0;
    diag_ = LegZDiagnostics{};
    have_prev_ = false;
    prev_contact_ = {{false, false, false, false}};
    contact_ = LegContactDetector(cfg_.force_on, cfg_.force_off);
  }

 private:
  static double median(double* v, int n) {
    std::sort(v, v + n);
    if (n % 2 == 1) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
  }

  Eigen::Vector3d calibrate(const Eigen::Vector3d& p) const {
    const double a = cfg_.roll_offset_deg * M_PI / 180.0;
    const double b = cfg_.pitch_offset_deg * M_PI / 180.0;
    const double ca = std::cos(a), sa = std::sin(a);
    const double cb = std::cos(b), sb = std::sin(b);
    // p' = Rx(a) * Ry(b) * p
    const Eigen::Vector3d pb(cb * p.x() + sb * p.z(), p.y(),
                             -sb * p.x() + cb * p.z());
    return Eigen::Vector3d(pb.x(), ca * pb.y() - sa * pb.z(),
                           sa * pb.y() + ca * pb.z());
  }

  bool finish(const LegSample& s, const LegAttitude& att,
              const std::array<bool, 4>& contact, bool ok) {
    prev_sample_ = s;
    prev_att_ = att;
    prev_contact_ = contact;
    have_prev_ = true;
    return ok;
  }

  LegZConfig cfg_;
  LegContactDetector contact_;
  double z_{0.0};
  LegZDiagnostics diag_{};
  bool have_prev_{false};
  LegSample prev_sample_{};
  LegAttitude prev_att_{};
  std::array<bool, 4> prev_contact_{{false, false, false, false}};
};

}  // namespace simple_loop_closure
