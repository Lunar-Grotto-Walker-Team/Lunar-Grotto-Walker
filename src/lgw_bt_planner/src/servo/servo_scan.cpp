// src/servo/servo_scan.cpp
// Pure scan waypoint generator; see servo_scan.hpp.

#include "lgw_bt_planner/servo/servo_scan.hpp"

#include <algorithm>
#include <cmath>

namespace lgw_bt_planner
{

namespace
{

void append_j5_branch(
  std::vector<std::array<double, 6>> & out,
  double j1, double j2, double j3, double j4, double j6,
  double j5_from, double j5_to, double step, double gap)
{
  if (step <= 0.0) {
    return;
  }
  const double lo = std::min(j5_from, j5_to);
  const double hi = std::max(j5_from, j5_to);
  for (double j5 = lo; j5 <= hi + 1e-9; j5 += step) {
    if (std::abs(j5) < gap - 1e-9) {
      continue;  // never emit a waypoint inside the singularity gap
    }
    out.push_back({j1, j2, j3, j4, j5, j6});
  }
}

}  // namespace

std::vector<std::array<double, 6>> make_scan_waypoints(
  const std::array<double, 6> & measured, const ScanParams & params)
{
  std::vector<std::array<double, 6>> out;
  const double j1_span = params.j1_max_rad - params.j1_min_rad;
  if (j1_span < 0.0 || params.j1_step_rad <= 0.0) {
    return out;
  }
  const int j1_steps = std::max(
    1, static_cast<int>(std::ceil(j1_span / params.j1_step_rad)));

  const double j2 = params.j2_raise_rad;
  const double j3 = params.j3_raise_rad;
  const double j4 = measured[3];
  const double j6 = measured[5];

  for (int i = 0; i <= j1_steps; ++i) {
    const double j1 = params.j1_min_rad + j1_span * i / j1_steps;
    append_j5_branch(
      out, j1, j2, j3, j4, j6,
      params.j5_min_rad, params.j5_max_rad, params.j5_step_rad,
      params.j5_gap_rad);
    append_j5_branch(
      out, j1, j2, j3, j4, j6,
      params.j5_min_rad_pos, params.j5_max_rad_pos, params.j5_step_rad,
      params.j5_gap_rad);
  }
  return out;
}

}  // namespace lgw_bt_planner
