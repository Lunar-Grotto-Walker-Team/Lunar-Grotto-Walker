# 行为树栈：端点契约

> 只列跨包边界、且行为树直接依赖的端点。类型以包内实际定义为准。

## 动作（Action）

| 端点 | 类型 | 提供方 | 使用方 |
|---|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` | `agx_arm_moveit`（vendor） | `MoveArmToPose`、`MoveArmToNamed`、`servo_node` |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | ros2_control | MoveIt |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory`（话题） | ros2_control | `ControlGripper` |
| `/go2_transit/transit_to_pose` | `go2_transit/action/TransitToPose` | `go2_transit` | `TransitToLocalPoint` |
| `/lgw_bt_execution` | `btcpp_ros2_interfaces/action/ExecuteTree` | `bt_runner` | `run_bt.sh`、外部调用方 |

## 服务（Service）

| 端点 | 类型 | 提供方 | 使用方 |
|---|---|---|---|
| `/set_body_pose` | `body_pose_bridge/srv/SetBodyPose` | `body_pose_bridge` | `SetBodyPose` 节点 |
| `/set_vui_brightness` | `lgw_vui_bridge/srv/SetVuiBrightness` | `lgw_vui_bridge` | `SetLedBrightness` 节点 |
| `/select_arm_observation_pose` | `lgw_bt_planner/srv/SelectArmObservationPose` | `select_arm_observation_pose_service_node` | `SelectArmObservationPose` 节点 |
| `/lgw/servo/takeover`、`/lgw/servo/release` | `std_srvs/srv/Trigger` | `servo_node` | `ServoTakeover`、`ServoRelease` |
| `/lgw/servo/pause`、`/lgw/servo/resume` | `std_srvs/srv/Trigger` | `servo_node` | 外部/恢复逻辑 |
| `/lgw/tracker/mark_track` | `yoloe_interfaces/srv/MarkTrack` | `lgw_3d_tracker` | `MarkTrack` 节点 |
| `/start_mapping` | `std_srvs/srv/Trigger` | 导航侧 | `StartMapping` 节点 |
| `/compute_ik`、`/apply_planning_scene` | `moveit_msgs/srv/*` | MoveIt | `servo_node` 内部 |

## 话题（Topic）

| 端点 | 类型 | 提供方 | 使用方 |
|---|---|---|---|
| `/perception/two_stage/camera_head/result` | `yoloe_interfaces/msg/ObservationResult` | 感知链路（独立仓库） | `IsHeadDetectionConfirmed`、tracker |
| `/perception/two_stage/camera_arm/result` | 同上 | 同上 | `IsArmDetectionConfirmed`、tracker |
| `/perception/tracks/unified` | `yoloe_interfaces/msg/UnifiedTrackArray` | `lgw_3d_tracker` | BT 跟踪节点 |
| `/feedback/joint_states` | `sensor_msgs/msg/JointState` | `agx_arm_ctrl` | `agx_topic_hardware`、MoveIt 状态 |
| `/feedback/tcp_pose` | `geometry_msgs/msg/PoseStamped` | `agx_arm_ctrl` | servo / 验证脚本 |
| `/feedback/gripper_status` | `agx_arm_msgs/msg/GripperStatus` | `agx_arm_ctrl` | 夹爪判据 |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | `agx_topic_hardware` | `agx_arm_ctrl` |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | MoveIt / RSP |
| `/lgw/servo/state` | `lgw_bt_planner/msg/ServoState` | `servo_node` | 监控/诊断 |
| `/bt_user_input` | `std_msgs/msg/Bool` | 操作者 | `WaitForUserInput` |
| `/api/sport/request` | `unitree_api/msg/Request` | BT 节点 / bridge | Go2 固件 |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | Go2 | 状态判断 |

## 关键约束

- `/api/sport/request` 的 QoS 必须 **RELIABLE**（和 Go2 固件一致）；
- `/feedback/joint_states` 与 `/control/joint_states` 属于**两个域**：
  servo 的 `tcp_pose_domain` 决定 FK 用哪一个，混用会导致换支时手腕来回摆；
- 感知链路缺失时，`/perception/two_stage/*` 无数据，依赖检测结果的 BT 子树会失败，
  但机械臂与 Go2 基础动作不受影响。
