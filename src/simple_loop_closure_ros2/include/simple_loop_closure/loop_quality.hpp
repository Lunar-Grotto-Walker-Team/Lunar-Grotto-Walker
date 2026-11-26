#ifndef SIMPLE_LOOP_CLOSURE__LOOP_QUALITY_HPP_
#define SIMPLE_LOOP_CLOSURE__LOOP_QUALITY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <nanoflann.hpp>

namespace simple_loop_closure
{

struct PointCloudAdaptor
{
  const std::vector<Eigen::Vector3d> & pts;

  std::size_t kdtree_get_point_count() const {return pts.size();}

  double kdtree_get_pt(std::size_t idx, std::size_t dim) const
  {
    return pts[idx][static_cast<Eigen::Index>(dim)];
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX &) const {return false;}
};

// Fraction of `src` points (after applying T) that have a point in `dst`
// within `max_dist`.  Normalised by the source size, so a query scan that is
// only half-covered by the target submap scores 0.5.
//
// LIMITATION: on a perfectly self-similar surface (a uniform tunnel wall)
// an axial ICP slide keeps every source point on the target surface, so this
// ratio stays high.  This gate catches partial overlaps / gross slides, not
// the last few decimetres of an axial slide; the translation-consistency gate
// is the primary defence for that class.
inline double overlapRatio(
  const std::vector<Eigen::Vector3d> & src,
  const std::vector<Eigen::Vector3d> & dst,
  const Eigen::Affine3d & T, double max_dist,
  std::size_t * inliers_out = nullptr)
{
  if (src.empty() || dst.empty() || max_dist <= 0.0) {
    if (inliers_out != nullptr) {*inliers_out = 0;}
    return 0.0;
  }

  PointCloudAdaptor adaptor{dst};
  nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
    PointCloudAdaptor, 3> index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  const double max_sq = max_dist * max_dist;
  std::size_t inliers = 0;
  for (const auto & p : src) {
    const Eigen::Vector3d q = T * p;
    unsigned int idx = 0;
    double sq_dist = 0.0;
    if (index.knnSearch(q.data(), 1, &idx, &sq_dist) > 0 && sq_dist <= max_sq) {
      ++inliers;
    }
  }

  if (inliers_out != nullptr) {*inliers_out = inliers;}
  return static_cast<double>(inliers) / static_cast<double>(src.size());
}

// Yaw difference wrapped to [-pi, pi].
inline double yawDelta(const Eigen::Matrix3d & R1, const Eigen::Matrix3d & R2)
{
  const double y1 = std::atan2(R1(1, 0), R1(0, 0));
  const double y2 = std::atan2(R2(1, 0), R2(0, 0));
  double d = y1 - y2;
  while (d > M_PI) {d -= 2.0 * M_PI;}
  while (d < -M_PI) {d += 2.0 * M_PI;}
  return d;
}

// True when the most recent accepted loop from this target was within
// `window` query frames.  The caller updates the map on acceptance.
inline bool isDuplicateLoop(
  int query, int target,
  const std::unordered_map<int, int> & last_query_for_target,
  int window)
{
  const auto it = last_query_for_target.find(target);
  if (it == last_query_for_target.end()) {return false;}
  return (query - it->second) <= window;
}

// Scale odometry variance by how far the keyframe moved compared with the
// nominal keyframe spacing.  Never shrinks below the nominal noise.
inline double motionScale(double measured, double nominal)
{
  if (nominal <= 0.0) {return 1.0;}
  return std::max(1.0, measured / nominal);
}

}  // namespace simple_loop_closure

#endif  // SIMPLE_LOOP_CLOSURE__LOOP_QUALITY_HPP_
