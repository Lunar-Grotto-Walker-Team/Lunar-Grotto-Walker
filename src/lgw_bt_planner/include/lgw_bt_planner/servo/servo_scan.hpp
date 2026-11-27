// include/lgw_bt_planner/servo/servo_scan.hpp
//
// Pure joint-space scan waypoint generator for the SCANNING state.
// The servo node owns the clock/TF/MoveGroup; this module only turns the
// measured joints and the scan parameters into a deterministic list of
// 6-joint goals.
//
// j1 sweeps horizontally; j5 visits BOTH wrist branches (negative and
// positive) while avoiding the exact singularity near 0; j2/j3 are raised to
// the observation posture; j4/j6 keep the measured value.

#ifndef LGW_BT_PLANNER__SERVO__SERVO_SCAN_HPP_
#define LGW_BT_PLANNER__SERVO__SERVO_SCAN_HPP_

#include <array>
#include <vector>

namespace lgw_bt_planner
{

struct ScanParams
{
  double j1_min_rad{-1.047};
  double j1_max_rad{1.047};
  double j1_step_rad{0.1745};
  double j5_min_rad{-1.0};        // negative branch
  double j5_max_rad{-0.3};
  double j5_min_rad_pos{0.3};     // positive branch
  double j5_max_rad_pos{1.0};
  double j5_step_rad{0.2};
  double j5_gap_rad{0.3};
  double j2_raise_rad{0.0};
  double j3_raise_rad{0.0};
};

// Returns the scan goals in execution order.  Every goal carries the
// raised j2/j3 and the measured j4/j6.  The caller seeds MoveGroup from the
// live measured pose before the first goal; the measured pose is not part
// of the returned list.
std::vector<std::array<double, 6>> make_scan_waypoints(
  const std::array<double, 6> & measured, const ScanParams & params);

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SERVO_SCAN_HPP_
