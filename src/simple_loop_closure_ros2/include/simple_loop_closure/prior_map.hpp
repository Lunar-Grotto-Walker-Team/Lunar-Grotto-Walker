#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace simple_loop_closure {

// One prior-map keyframe.  `t` / `q` are the keyframe pose, the matching
// cloud on disk is body-frame (the same convention as keyframes_cloud_ when
// mapped_cloud=true).
struct PriorPose {
  int index{0};
  double stamp{0.0};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};

// A world-frame asset loaded as if it were body-frame fails silently: every
// registration would still converge to *something*.  The asset therefore
// declares its convention on the first line and loading refuses anything
// else.
inline bool priorHeaderIsBodyFrame(const std::string& first_line) {
  return first_line.find("frame: body") != std::string::npos;
}

// Parses `index, timestamp, x, y, z, qx, qy, qz, qw` rows, skipping '#'
// comments and the header row.  Returns false when a row cannot be parsed.
inline bool parsePriorPoses(std::istream& in, std::vector<PriorPose>& out) {
  out.clear();
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (line.find("index") != std::string::npos) continue;  // header row

    std::stringstream ss(line);
    std::string field;
    std::vector<double> v;
    while (std::getline(ss, field, ',')) {
      if (field.find_first_not_of(" \t") == std::string::npos) return false;
      char* end = nullptr;
      const double d = std::strtod(field.c_str(), &end);
      if (end == field.c_str()) return false;
      v.push_back(d);
    }
    if (v.size() < 9) return false;

    PriorPose p;
    p.index = static_cast<int>(v[0]);
    p.stamp = v[1];
    p.t = Eigen::Vector3d(v[2], v[3], v[4]);
    p.q = Eigen::Quaterniond(v[8], v[5], v[6], v[7]);  // w, x, y, z
    p.q.normalize();
    out.push_back(p);
  }
  return !out.empty();
}

// True when a (already translated) pose lies outside the exclusion disc.
// Used to drop the open area around the base, which is where the previous
// session's map is least trustworthy and where the base may have moved.
inline bool outsidePriorCut(double x, double y, double cx, double cy,
                            double radius) {
  if (radius <= 0.0) return true;
  const double dx = x - cx;
  const double dy = y - cy;
  return dx * dx + dy * dy > radius * radius;
}

// Picks the keyframes that belong to the prior: translate into the current
// base frame first, then crop to a circle (crop_radius <= 0 disables the
// crop), then keep the first frame of every `min_spacing` metres along the
// original order.  Returns indices into `poses`.
inline std::vector<size_t> selectPriorPoses(const std::vector<PriorPose>& poses,
                                            double dx, double dy,
                                            double crop_cx, double crop_cy,
                                            double crop_radius,
                                            double min_spacing) {
  std::vector<size_t> kept;
  const Eigen::Vector2d centre(crop_cx, crop_cy);
  Eigen::Vector2d last(0.0, 0.0);
  bool have_last = false;

  for (size_t i = 0; i < poses.size(); ++i) {
    const Eigen::Vector2d xy(poses[i].t.x() + dx, poses[i].t.y() + dy);
    if (crop_radius > 0.0 && (xy - centre).norm() > crop_radius) continue;
    if (have_last && min_spacing > 0.0 && (xy - last).norm() < min_spacing) {
      continue;
    }
    kept.push_back(i);
    last = xy;
    have_last = true;
  }
  return kept;
}

// Linear xy nearest-neighbour over the selected prior keyframes.  The asset
// is ~10^2 frames, so a KD-tree would cost more than it saves.
class PriorIndex {
 public:
  void build(const std::vector<PriorPose>& poses,
             const std::vector<size_t>& kept) {
    ids_ = kept;
    xy_.clear();
    xy_.reserve(kept.size());
    for (size_t i : kept) {
      xy_.emplace_back(poses[i].t.x(), poses[i].t.y());
    }
  }

  // Index into the original `poses` vector, or -1 when nothing is within
  // `radius`.
  int nearest(double x, double y, double radius) const {
    int best = -1;
    double best_d2 = radius * radius;
    const Eigen::Vector2d q(x, y);
    for (size_t k = 0; k < xy_.size(); ++k) {
      const double d2 = (xy_[k] - q).squaredNorm();
      if (d2 <= best_d2) {
        best_d2 = d2;
        best = static_cast<int>(ids_[k]);
      }
    }
    return best;
  }

  size_t size() const { return ids_.size(); }

 private:
  std::vector<size_t> ids_;
  std::vector<Eigen::Vector2d> xy_;
};

// Sequential-consistency gate: a candidate is usable only when it continues
// the previous accepted pair.  The first candidate of a run has no
// predecessor (prev_prior < 0) and must wait for a second frame -- that is
// what keeps a single stray match on a parallel branch from entering the
// graph.
inline bool priorSequenceAgrees(int prev_query, int prev_prior, int query,
                                int prior, int max_query_step,
                                int max_prior_step) {
  if (prev_prior < 0) return false;
  if (query - prev_query > max_query_step) return false;
  return std::abs(prior - prev_prior) <= max_prior_step;
}

}  // namespace simple_loop_closure
