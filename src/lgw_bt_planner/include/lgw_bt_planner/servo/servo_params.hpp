// include/lgw_bt_planner/servo/servo_params.hpp
//
// All node parameters in one value object (Task 3b of the 2026-11-07
// node-split plan). The node keeps thin reference members bound to these
// fields, so every existing use keeps compiling while the storage moves here.
// A later task moves declare_and_load_parameters() itself into
// servo_params.cpp; that move is intentionally NOT in this step.

#ifndef LGW_BT_PLANNER__SERVO__SERVO_PARAMS_HPP_
#define LGW_BT_PLANNER__SERVO__SERVO_PARAMS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "lgw_bt_planner/hold_recovery_decision.hpp"
#include "lgw_bt_planner/lookat_posture_family.hpp"
#include "lgw_bt_planner/lookat_qp_solver.hpp"
#include "lgw_bt_planner/preposition_decision.hpp"
#include "lgw_bt_planner/servo/scan_trajectory.hpp"
#include "lgw_bt_planner/servo_control_law.hpp"

namespace lgw_bt_planner
{

struct ServoParams
{
  // scalars (defaults copied from the shipped declare_parameter calls)
  std::string frame_prefix{"arm1/"};
  std::string pose_watch_topic{"/control/joint_states"};
  std::string calibration_path{""};
  bool follow_check_enabled{true};
  std::string follow_reference_topic{"/control/joint_states"};
  double follow_tolerance_rad{0.15};
  int follow_timeout_ms{300};
  double preposition_velocity_scaling{0.3};
  double preposition_accel_scaling{0.5};
  double preposition_tolerance_rad{0.02};
  double preposition_timeout_s{10.0};
  int64_t tracker_stream_timeout_ms{500};
  bool ff_enable{true};
  double ff_lowpass_hz{8.0};
  double ff_deadband_mps{0.01};
  double ff_deadband_radps{0.01};
  double ff_max_mps{0.60};
  double ff_max_radps{0.80};
  double min_observe_distance_m{0.35};
  // Effective gain of the published arm-base target. 1.0 is the design
  // default; 0.7 preserves the pre-2026-11-13 arm tier until the bench
  // revalidation is recorded.
  double arm_source_scale{1.0};
  // Scanning (SCANNING state).
  bool auto_scan{true};
  int scan_start_delay_ms{500};
  bool auto_resume_on_target{true};
  bool scan_require_base_static{true};
  // 扫描入位前的指令/实测对齐门限; recovery 的 0.005 rad 对它太严。
  double scan_presettle_gap_rad{0.15};
  double scan_velocity_scaling{0.2};
  double scan_accel_scaling{0.3};
  double scan_tolerance_rad{0.02};
  double scan_hold_s{0.20};
  bool scan_roll_lock_enable{true};
  // Startup consistency check between the FK+JSON hand-eye chain (used by
  // the control law) and the TF chain (used by the tracker).
  bool camera_tf_check_enable{true};
  std::string camera_tf_frame{"camera_arm_color_optical_frame"};
  double camera_tf_pos_tol_m{0.01};
  double camera_tf_ang_tol_rad{0.0175};
  ScanCircleParams scan;
  double recovery_velocity_scaling{0.30};
  double recovery_accel_scaling{0.50};
  double recovery_timeout_s{8.0};
  double recovery_tolerance_rad{0.02};
  int recovery_settle_ms{200};
  double recovery_presettle_gap_rad{0.005};
  int64_t recovery_presettle_timeout_ms{3000};
  int64_t recovery_presettle_stable_ms{250};
  double follow_resume_gap_rad{0.10};
  int64_t follow_resume_stable_ms{300};
  // grouped parameter families (the structs already existed)
  ControlLawParams control;
  PostureParams posture;
  PrepositionParams preposition;
  QpSolverParams qp;
  HoldRecoveryParams recovery;
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__SERVO__SERVO_PARAMS_HPP_
