// sport_api_constants.hpp
//
// Shared constants for Go2 sport API BT nodes (StandUp / BalanceStand /
// StandDown). Single source of truth — three .cpp files #include this
// and reference constants directly, no inheritance involved (see spec §3
// "Why no shared base class").
//
// Topic names + API IDs are mirrored from go2_bt_ws/CLAUDE.md and
// unitree_ros2/example/src/include/common/ros2_sport_client.h.
// Thresholds are the verified success/failure gates from
// go2_bt_ws/CLAUDE.md node table.

#pragma once

#include <cstdint>

namespace lgw_bt_planner::sport_api
{

// ---- Topic names ----
// Note: /api/sport/request has a leading "/", lf/sportmodestate does NOT.
inline constexpr const char* kSportRequestTopic   = "/api/sport/request";
inline constexpr const char* kSportModeStateTopic = "lf/sportmodestate";
// VUI service (屏幕亮度) 用独立 topic,与 sport 平行;不在
// /api/sport/request 上发(SetBrightness 是 ROBOT_VUI_API_ID 1005,
// ROBOT_VUI_SERVICE_NAME = "vui",见 unitree_sdk2/.../vui/vui_api.hpp:19-24)。
//
// topic 命名:Go2 C++ SDK `channel_namer.hpp:10` 写明
//   ROBOT_SDK_CHANNEL_PREFIX = "rt/api/"
//   拼接 → "rt/api/vui/request";第三方 Go2 ROS2 SDK
// `webrtc_topics.py:34` 同名映射 RTC_TOPIC["VUI"]。
// sport 走 "/api/sport/request"(ros2_sport_client.cpp:73)是因为
// unitree_ros2 官方 ROS2 wrapper 用了不同命名(固件兼容),但 VUI
// 没有官方 ROS2 wrapper,只能跟 C++ SDK 走 "rt/api/vui/request"。
// 2026-10-14 真机验证:换 "/api/vui/request" → "rt/api/vui/request"
// 之前狗屏幕亮度不变(无可见响应),换成 "rt/api/vui/request" 后
// 才能调起固件 VUI handler。
inline constexpr const char* kVuiRequestTopic      = "rt/api/vui/request";

// ---- Sport API IDs (see unitree_ros2/example/.../ros2_sport_client.h) ----
inline constexpr int32_t kApiIdStandUp      = 1004;
inline constexpr int32_t kApiIdStandDown    = 1005;
inline constexpr int32_t kApiIdBalanceStand = 1002;
inline constexpr int32_t kApiIdStopMove     = 1003;
inline constexpr int32_t kApiIdBodyHeight   = 1013;
inline constexpr int32_t kApiIdEuler        = 1007;

// ---- Gait switching API IDs (Go2, sdk v0.2.0+ 2025-07-29) ----
// 旧的 ROBOT_SPORT_API_ID_SWITCHGAIT(1011)已在 v0.2.0 删除,新 API
// 拆成 4 个 trigger + 7 个 bool flag,见 unitree_ros2/CHANGELOG.md:60-67
// 与官方 gait API 文档。Trigger 命令参数 JSON = ""(空);Flag 命令
// 参数 JSON = `{"data": <bool>}`。
inline constexpr int32_t kApiIdStaticWalk      = 1061;  // 常规行走(优雅)
inline constexpr int32_t kApiIdTrotRun         = 1062;  // 跑步(最高 3.7m/s)
inline constexpr int32_t kApiIdEconomicGait    = 1063;  // 续航(身高较高)
inline constexpr int32_t kApiIdFreeWalk        = 2045;  // Ai 灵动模式(默认)
inline constexpr int32_t kApiIdFreeBound       = 2046;  // flag:并腿跑
inline constexpr int32_t kApiIdFreeJump        = 2047;  // flag:跳跃
inline constexpr int32_t kApiIdFreeAvoid       = 2048;  // flag:闪避
inline constexpr int32_t kApiIdClassicWalk     = 2049;  // flag:Ai 经典
inline constexpr int32_t kApiIdWalkUpright     = 2050;  // flag:后腿直立(电机易过热)
inline constexpr int32_t kApiIdCrossStep       = 2051;  // flag:交叉步(电机易过热)
inline constexpr int32_t kApiIdAutoRecoverSet  = 2054;  // flag:摔倒自恢复(有背载时关)

// ---- VUI API IDs (Go2 EDU 屏幕/UI 控制) ----
// ROBOT_VUI_API_ID_SETBRIGHTNESS = 1005,走 rt/api/vui/request。
// 等级 0-10(`go2_vui_client.cpp:32` `++level %= 11`);JSON = `{"data": level}`。
inline constexpr int32_t kApiIdVuiSetBrightness = 1005;

// ---- Completion thresholds (from go2_bt_ws/CLAUDE.md node table) ----
inline constexpr float  kStandHeight    = 0.30f;
inline constexpr float  kLyingHeight    = 0.20f;
inline constexpr int    kStableFrames   = 5;

// ---- BodyHeight tuning (per Unitree Go2 /body_height service spec) ----
// BodyHeight is a RELATIVE offset from the Go2's default standing
// height (0.33 m). Valid offset range per docs: [-0.18, +0.03].
inline constexpr float kDefaultGo2BodyHeight   = 0.33f;
inline constexpr float kBodyHeightOffsetMin   = -0.18f;
inline constexpr float kBodyHeightOffsetMax   = 0.03f;
inline constexpr float kDefaultBodyHeightTol  = 0.02f;

// ---- EulerPose tuning (per Unitree Go2 sport API Euler 1007) ----
// Sign convention: right-hand rule on body frame (X forward, Y left,
// Z up). pitch>0 = nose up, pitch<0 = nose down. Valid ranges below
// are the Go2 spec; out-of-range values are CLAMPED at the BT node
// (with a WARN log) — the node does not FAIL on them. The service
// only works in balanced-standing state; caller must have sent a
// BalanceStand (1002) before this node runs.
inline constexpr float kEulerRollMin         = -0.75f;
inline constexpr float kEulerRollMax         =  0.75f;
inline constexpr float kEulerPitchMin        = -0.75f;
inline constexpr float kEulerPitchMax        =  0.75f;
inline constexpr float kEulerYawMin          = -3.14159f;
inline constexpr float kEulerYawMax          =  3.14159f;
inline constexpr float kDefaultEulerTol      =  0.05f;

// ---- Per-node timeouts (seconds) ----
inline constexpr double kStandUpTimeoutSec      = 5.0;
inline constexpr double kStandDownTimeoutSec    = 5.0;
inline constexpr double kBodyHeightTimeoutSec   = 5.0;
inline constexpr double kEulerTimeoutSec        = 5.0;
inline constexpr double kBodyPoseTimeoutSec     = 5.0;
// kSwitchGaitTimeoutSec / kStableGaitFrames removed 2026-10-13: SwitchGait
// is now Pattern A (fire-and-forget) per feedback_go2_new_gait_state_dead
// — Go2 firmware processes v0.2.0+ new gait commands but does not update
// lf/sportmodestate gait_type / mode, so a polling completion criterion
// would always timeout. The official SportClient (ros2_sport_client.cpp)
// also only publishes without state verify.

// ---- VUI SetBrightness 推荐等级范围 ----
// `go2_vui_client.cpp:32` `++level %= 11` 暗示有效 0-10 闭环;
// 11 也可能被固件接受但行为未文档化。set_led_brightness_node 收到
// 范围外值会 WARN 一次(不强制 reject,交给 firmware)。
inline constexpr int kVuiBrightnessMin         = 0;
inline constexpr int kVuiBrightnessMax         = 10;
inline constexpr int kVuiBrightnessDefault     = 5;

}  // namespace lgw_bt_planner::sport_api
