// include/lgw_bt_planner/piper_limits.hpp
//
// Piper joint limits, from agx_arm_description/piper/piper_description.urdf.
// Shared by the posture family (solver clamping) and the pre-position criteria
// (limit-distance measurement) so the two can never disagree.

#ifndef LGW_BT_PLANNER__PIPER_LIMITS_HPP_
#define LGW_BT_PLANNER__PIPER_LIMITS_HPP_

namespace lgw_bt_planner
{

// Index order: joint1..joint6.
constexpr double kPiperLower[6] = {
  -2.6179938, 0.0, -2.9670597, -1.7453292, -1.2217304, -2.0943951};
constexpr double kPiperUpper[6] = {
  2.6179938, 3.1415926, 0.0, 1.7453292, 1.2217304, 2.0943951};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__PIPER_LIMITS_HPP_
