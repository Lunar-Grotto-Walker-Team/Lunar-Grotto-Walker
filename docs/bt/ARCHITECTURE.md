# 行为树与机械臂栈：架构

> 目标平台：Go2（Jetson Orin NX）+ AgileX Piper 机械臂 + 头部/臂端双 RealSense。
> 本文描述节点与包的边界；端点契约见 [TOPICS.md](TOPICS.md)。

## 1. 总览

```
                 ┌──────────────────────────────┐
  感知（可选）   │ lgw-yoloe-perception（独立仓库）│
                 │  Foxy 推理 + Humble 双相机编排 │
                 └──────────────┬───────────────┘
                                │ /perception/two_stage/*/result
                 ┌──────────────▼───────────────┐
                 │ lgw_3d_tracker               │
                 │  统一 3D 跟踪 → /perception/tracks/unified
                 └──────────────┬───────────────┘
                                │
   ┌────────────────────────────▼─────────────────────────────┐
   │ lgw_bt_planner                                           │
   │  bt_runner（BT.CPP v4 + BT.ROS2 v0.3.0）                 │
   │   ├─ 35 个 BT 节点（机械臂 / 感知几何 / 跟踪 / Go2 运动）│
   │   └─ servo_node（MoveIt Servo + LookAt QP + 恢复）       │
   │  select_arm_observation_pose_service_node                │
   └───────┬──────────────────────────┬───────────────────────┘
           │ /move_action             │ /feedback/* /control/*
   ┌───────▼──────────────┐   ┌───────▼──────────────────────┐
   │ agx_arm_moveit       │   │ agx_arm_ctrl（Piper 驱动）    │
   │ （vendor agx_arm_ros）│   │ （vendor agx_arm_ros）        │
   │ MoveIt + ros2_control│   │ CAN → 机械臂 / 夹爪           │
   └──────────────────────┘   └──────────────────────────────┘
           ▲
           │ hardware_backend:=topic 时经 agx_topic_hardware 插件
   ┌───────┴──────────────┐   ┌──────────────────────────────┐
   │ body_pose_bridge     │   │ lgw_vui_bridge               │
   │ /set_body_pose       │   │ /set_vui_brightness          │
   └──────────────────────┘   └──────────────────────────────┘
```

## 2. 包

| 包 | 角色 | 可执行 / 产物 |
|---|---|---|
| `lgw_bt_planner` | 行为树编排 + 伺服控制 | `bt_runner`、`servo_node`、`select_arm_observation_pose_service_node`、BT 节点插件库 |
| `lgw_3d_tracker` | 头/臂双相机统一 3D 跟踪 | `unified_track_node` |
| `body_pose_bridge` | Go2 姿态服务（Euler + BalanceStand 顺序封装） | `body_pose_bridge_node` |
| `lgw_vui_bridge` | VUI 亮度（SDK 子进程隔离） | `vui_bridge_node`、`go2_vui_set_brightness_helper` |
| `agx_topic_hardware` | ros2_control 硬件插件（topic 后端） | `AgxTopicSystem` 插件 |
| `lgw_servo_smoothing` | MoveIt Servo 平滑插件 | 插件库 |
| `servo_state_probe` | 独立 MoveIt 状态探针（诊断） | `servo_state_probe` |
| `yoloe_interfaces` | 感知/跟踪 msg 与 srv | — |
| `go2_transit` | 两点直线行走 / 原地转向，注册 `TransitToLocalPoint` | `transit_node`、BT 插件库 |

外部依赖（不属本仓库）：`agx_arm_ros`、`BehaviorTree.ROS2`、`unitree_ros2`（vendor），
`point_lio`（独立仓库），`lgw-yoloe-perception`（AGPL 独立仓库）。见
[依赖说明](../dependencies.md)。

## 3. BT 节点（35 个）

注册位置：`src/lgw_bt_planner/src/bt_plugin_registration.cpp`。

| 类别 | 节点 |
|---|---|
| 机械臂 | `MoveArmToPose` `MoveArmToNamed` `SetPoseStamped` `ControlGripper` |
| 感知几何 | `ComputeLookatPose` `ComputeGraspPose` `VerifyHeadArmConsistency` `ComputeHeadTargetWorld` `ComputeArmTargetWorld` `MakePoseFromPointAndReference` `SelectArmObservationPose` `IsHeadDetectionConfirmed` `IsArmDetectionConfirmed` |
| 跟踪 / Servo | `IsTargetAvailable` `IsTargetWithin` `ComputeGraspPosesFromTrack` `ComputeGraspPosesSampled` `WaitForTargetSelected` `WaitUntilTargetNear` `WaitForArmHits` `IsArmCameraAlive` `ServoTakeover` `ServoRelease` `MarkTrack` `RecordBasePose` `IsWithinBaseRadius` |
| Go2 运动 | `StandUp` `BalanceStand` `StandDown` `SetBodyHeight` `SetBodyPose` `SwitchGait` `SetLedBrightness` `StartMapping` |
| 交互 | `WaitForUserInput` |
| 外部注册 | `TransitToLocalPoint`（`go2_transit`） |

BT 树共 22 棵，位于 `src/lgw_bt_planner/bt_trees/`。常用入口：

| 树 | 用途 |
|---|---|
| `pick_and_place_demo` | 单次抓放（MoveIt 规划） |
| `track_collect_reactive_v2` | 跟踪 + 抵近 + 抓取（需要跟踪与导航） |
| `head_arm_independent_lookat` | 头/臂独立 LookAt |
| `transit_mission` / `transit_patrol` / `transit_return` | 导航子树的入口 |

## 4. 启动拓扑

```
start_lgw_arm.sh      CAN 1 Mbps + agx_arm_ctrl（可 --driver-only）
        ↓
start_lgw_yoloe.sh    宿主机 Foxy 推理 + 容器内两路 RealSense（可选）
        ↓
start_lgw_chain.sh    MoveIt/Servo + lgw servo + tracker +（可选）bt_runner
        ↓
run_bt.sh <TreeID>    触发 /lgw_bt_execution
```

脚本路径与环境变量约定见 [`tools/lgw/README.md`](../../tools/lgw/README.md)。

## 5. 关键设计约束

1. **BT 节点不直接控制硬件**：机械臂动作走 `/move_action` 或 ros2_control action，
   Go2 动作走 Unitree Sport API 服务/话题。
2. **`SetBodyPose` 的调用顺序**：`Euler → BalanceStand`，顺序反了 Go2 会静默丢弃姿态。
3. **`hardware_backend:=topic`**：真实状态来自 `/feedback/joint_states`，命令写
   `/control/joint_states`；JSB 必须重映射到 `/joint_states`，否则初始状态会先于
   真实反馈灌给驱动。
4. **Unitree SDK 与 rclcpp 同进程的 cyclonedds 冲突**：VUI 亮度走一次性 helper 子进程。
5. **停链路/跑测试前先 `control_enable false`**：避免全零位把 j3 顶到限位。
