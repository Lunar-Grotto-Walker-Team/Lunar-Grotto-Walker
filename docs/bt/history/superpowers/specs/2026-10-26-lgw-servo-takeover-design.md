# lgw Servo 接管与视觉伺服 设计方案

- 日期:2026-10-26
- 状态:设计基线(待实施)
- 作者:Claude Code(与用户共同评审)
- 调研记录:`docs/servo/2026-10-26-servo-takeover-design.md`(含证据、代码行号、推导)
- 实施计划:`docs/superpowers/plans/2026-10-26-lgw-servo-takeover.md`

---

## 0. 摘要

为 lg_walker_ws 增加一条"MoveIt Servo 接管机械臂"的通路:BT(或未来的行为技能)通过服务请求接管,
Servo 以闭环方式驱动机械臂,使臂端相机持续对准 3D tracker 输出的目标轨迹;释放时把控制权交还 MoveGroup。

核心设计:

1. **接缝:JTC topic**。Servo 发 `trajectory_msgs/JointTrajectory` 到
   `/arm_controller/joint_trajectory`;`arm_controller`(JTC)始终是唯一写者。
   MoveIt 走 action,Servo 走 topic,两者都进同一个 JTC;不新增 controller,不做 controller 切换。
2. **控制律交给 MoveIt Servo**:IK、雅可比、碰撞检查、奇异点、关节限位、平滑全部复用官方实现。
3. **自研部分只有一层状态机节点**:目标选择、几何源选择、lookat 位姿计算、100Hz 连续 P 环、接管/释放时序。
4. **视觉伺服的几何不使用 odom 未知量**:两个相机到机械臂基座的链路都是已知的,
   当前 tcp 位姿由 TF(`arm1/base_link ← arm1/tcp_link`)给出,闭环内零未知变换。
5. **3D tracker 增加"臂基座系几何"输出**:odom KF 继续负责跨帧归属,新增的臂基座坐标负责伺服几何。

---

## 1. 目标与边界

### 1.1 任务场景

Go2 行走中发现静态目标,在靠近目标的整个过程中,**保证目标始终位于臂端相机的观测视角内**。
伺服不是"把 tcp 送到目标点",而是"持续调整臂的指向,让臂端相机盯着目标"。

### 1.2 目标

- 服务端节点接收服务请求后接管机械臂,跟随 3D tracker 的目标轨迹。
- 释放时把控制权归还 MoveGroup,不产生跳变。
- 伺服过程中保留碰撞检查与关节限位。
- 允许 Go2 行走中伺服(行走作为基座扰动处理,不联动转体)。
- 本期不实现与 BT 的交互,但接口需兼容将来两种形态:作为 BT 技能节点、或被 BT 技能调用的独立节点。
- **最终形态是单栈**:一次 bringup 启动驱动 + MoveIt + Servo + BT,运行时通过服务在 MoveIt 与 Servo 之间切换。
  "两个独立驱动进程栈"只用于开发期隔离验证,不是最终架构。

### 1.3 非目标

- 不做抓取阶段的力控 / 阻抗控制。
- 不改 `agx_arm_ctrl` 的命令协议,不 fork 上游 `agx_arm_ros`。
- 不做 Go2 转体与机械臂伺服的联动。
- 不标定 `arm_mount_offset`。
- 不跟踪运动目标(见 1.4)。
- **不做多目标候选池与自动切换**(v2 预留,见 §8.5)。
- **不包含 Go2 机身碰撞检查**(v1 只接 head 相机碰撞 + 关节限位/工作空间钳制)。
- **不包含 Go2 机身碰撞检查**(v1 只接 head 相机碰撞 + 关节限位/工作空间钳制)。
- **不做多目标候选池与自动切换**(v2 预留,见 §8.5)。

### 1.4 关键假设

| 假设 | 影响 |
|---|---|
| **目标在物理空间近似静止**(三维位置不大幅改变) | 3D tracker 的 3 维位置 KF(无速度分量)是合理建模;odom 兜底可用到 track 过期 |
| Go2 `base_link` 与机械臂基座假设重合(`arm_mount_offset` 为单位阵) | 伺服闭环不使用该变换;只影响 odom 派生输出 |
| Go2 行走带来的基座运动是低频扰动 | 100Hz 控制环 + 2° 死区可吸收;需真机实测 |
| MoveIt 的模型帧名不带 `arm1/` 前缀 | Servo 参数必须用裸名 `base_link` / `tcp_link` |

> 若将来目标会移动,必须重新评估:KF 需加速度分量,odom 兜底窗口缩短到 ≤500ms。

### 1.5 验收标准

| # | 场景 | 通过标准 |
|---|---|---|
| A1 | 无真机(mock)启动 | 节点启动;`takeover` 后 `/arm_controller/joint_trajectory` 有 Servo 输出,`/arm_controller/follow_joint_trajectory/_action/status` 无活动 goal;`release` 后输出停止 |
| A2 | 接管往返 | 连续 20 次 takeover/release,无节点崩溃,JTC 不进入错误态 |
| A3 | 静止目标、head 源 | 10s 内 `angular_error_rad` 稳定 < 2°;`geometry_source == "head"` |
| A4 | 遮挡 head | 1 个控制周期内 `geometry_source` 切到 `arm`;恢复后经滞回切回 `head` |
| A5 | 双相机失效 | track 未过期时 `geometry_source == "odom"`;过期后进入 `HOLD` |
| A6 | 释放 | `release` 后 MoveGroup 能执行一次 `MoveArmToPose`;关节命令无不连续跳变。JTC 最后指令 = Servo 最后一点,理论上无跳变;阈值:相邻 5ms 内单关节 > 0.02 rad 判失败 |
| A7 | 单元测试 | lookat 位姿、几何源切换、odom 反算、状态机、控制律均有 gtest 覆盖 |

---

## 2. 背景与约束

### 2.1 现状控制链路

```
move_group → FollowJointTrajectory → ros2_control_node / arm_controller
           → mock_components/GenericSystem → joint_state_broadcaster
           → control/joint_states (200Hz) → agx_arm_ctrl_single_node → CAN
```

`agx_arm_ctrl` 一次性消费 `control/joint_states`,同时驱动臂、夹爪、手。
因此该话题的任何写入都必须经过单一写者。

### 2.2 单写者约束

`joint_state_broadcaster` 无条件以 200Hz 发布,内容是 mock 硬件 state interface 的镜像
(= 最近一次 command)。任何节点直接 publish `control/joint_states` 都会在 ≤5ms 内被覆写。
该约束对 arm joint 与 gripper 同样成立。

### 2.3 control gate 现状

`start_lgw_arm.sh` 未传 `auto_control_gate`,因此 `agx_arm_control_gate` 不启动,
`control_enabled` 恒为 `true`,**没有门卫**。互斥由 JTC 单写者 + 接管策略(§5.3)提供。

### 2.4 坐标系现状

`start_lgw_arm.sh` 已传 `frame_prefix:='arm1/'`,TF 每帧唯一 parent:

```
odom → base_link → arm1/world → arm1/base_link → … → arm1/tcp_link
```

命名规则:

| 场景 | 名字 |
|---|---|
| TF / tracker / 桥 | `arm1/base_link`、`arm1/tcp_link` |
| MoveIt 模型 / Servo 参数 | `base_link`、`tcp_link` |

`arm1/base_link` 与模型里的 `base_link` 指同一个物理帧(机械臂基座),
与挂载偏移是否标定无关。

---

## 3. 总体架构

```
┌──────────────────────────────────────────────────────────────┐
│ L3  lgw_servo_node(新增)                                     │
│     状态机 + 目标选择 + 几何源 + lookat + 100Hz P 环          │
│     服务:takeover / release / pause / resume                  │
│     话题:/lgw/servo/state                                     │
└───────────────┬──────────────────────────────────────────────┘
                │ TwistStamped(frame_id=base_link)
                ▼
┌──────────────────────────────────────────────────────────────┐
│ L2  moveit_servo / servo_node_main(官方)                     │
│     IK / 雅可比 / 碰撞 / 奇异点 / 限位 / 平滑                  │
│     输出 trajectory_msgs/JointTrajectory(单点)                │
└───────────────┬──────────────────────────────────────────────┘
                │ /arm_controller/joint_trajectory
                ▼
┌──────────────────────────────────────────────────────────────┐
│ L1  arm_controller (JointTrajectoryController)               │
│     两个输入:MoveGroup action / Servo topic                   │
│     JTC 是唯一写 command interface 的人                       │
└───────────────┬──────────────────────────────────────────────┘
                │ mock → joint_state_broadcaster → control/joint_states
                ▼
           agx_arm_ctrl → CAN
```

模块边界要求:

- **纯逻辑**(目标选择、几何源选择、odom 反算、lookat 位姿、控制律、状态机迁移)必须与 ROS 接口分离,
  放在可单测的函数/类里。
- ROS 接口层只做订阅、发布、服务、定时器、Servo 暂停/恢复与 Goal 状态监控。
- 这样将来搬进 BT 技能节点时,纯逻辑可原样复用。

---

## 4. L0:坐标系与命名规则

- Servo 参数统一使用模型名:`planning_frame: base_link`、`robot_link_command_frame: base_link`、
  `ee_frame_name: tcp_link`、`move_group_name: arm`。
  `tcp_link` 是 SRDF `arm` 组的 tip(`agx_arm.srdf.xacro:12`,chain `base_link → tcp_link`),
  也是 Servo 雅可比与 IK 的求值点(`servo_calcs.cpp:571`、`ik_solver_->getTipFrame()`);
  它是 `link6` 沿 z 偏移 `tcp_offset`(当前 `[0,0,0.12]`)的固定子帧。
  **控制律的误差必须在 `tcp_link` 上计算**——雅可比在此点求值,误差也在此点定义,两者一致。
  `ee_frame_name` 只在输入 twist 的 `frame_id` 等于它时参与坐标变换;本项目固定发 `base_link`,
  因此它不影响指令解释,填 `tcp_link` 只为语义清晰。
- 状态机发出的 `TwistStamped.frame_id` 固定为 `base_link`,与 `robot_link_command_frame` 相同,
  Servo 走单位变换,不跨 TF 解析。
- tracker 输出的 `position_arm_base` 所在帧为 `arm1/base_link`,与模型 `base_link` 是同一物理帧,
  数值可直接使用。
- odom 反算只在兜底档使用:`p_armbase = T_armbase_base · T_base_odom(now)⁻¹ · p_odom`,
  其中 `T_armbase_base` 视为单位阵。

---

## 5. L1:接缝与接管

### 5.1 接缝:JTC topic

`arm_controller`(JointTrajectoryController)是**唯一写 command interface 的人**。
它有两个输入:

| 输入 | 来源 | 用途 |
|---|---|---|
| action `/arm_controller/follow_joint_trajectory` | `move_group` | MoveIt 轨迹执行 |
| topic `/arm_controller/joint_trajectory` | `servo_node` | Servo 单点位置流 |

JTC 在 `on_activate` 时置 `subscriber_is_active_=true`;`topic_callback` 每收到一条消息
就 `add_new_trajectory_msg` 替换当前外部轨迹。Servo 以 `publish_period` 持续发单点轨迹,
因此 JTC 的外部队列始终是"最新一帧"。

**不新增 controller,不调用 `switch_controller`。** MoveIt 和 Servo 共用同一个 JTC。

> 两个输入不能同时活动:action 和 topic 都调用 `add_new_trajectory_msg`,后到的覆盖先到的。
> 这条由 §5.3 的策略保证,不是 controller 级硬隔离。

### 5.2 启动结构:单栈(最终)与两栈(开发)

**最终生产:单栈**

新增 `lgw_bt_planner/launch/lgw_full.launch.py`,一次启动:

1. include `agx_arm_moveit/launch/demo.launch.py`(rsp + move_group + JTC + broadcaster)
2. `moveit_servo/servo_node_main`(参数见 §6)
3. `lgw_servo_node`(参数见 §7)
4. `apply_head_camera_collision_node`
5. include `lgw_bt_planner/launch/lgw_bt_planner.launch.py`(bt_runner)

台架/真机只差一个 `follow` 参数:它同时决定 rsp、Servo `joint_topic`、节点 `pose_watch_topic`
三处取 `control/joint_states` 还是 `feedback/joint_states`(由 launch 统一派生,避免手工配错)。

CAN 与 `agx_arm_ctrl` 由 `scripts/start_lgw_full.sh` 在 launch 之前启动。
只有一个驱动进程,不存在 CAN 冲突。

**开发验证:两栈(仅用于隔离问题)**

| 栈 | 内容 | 驱动模式 |
|---|---|---|
| 正常栈 | driver + `demo.launch.py` + BT | `move_j` |
| 伺服栈 | driver + `demo.launch.py` + `servo_node` + `lgw_servo_node` | 按 §5.4 评估 |

两栈不能同时运行(两个驱动进程会抢 `can0`);切换 = 停一个、起另一个。
开发完成后合并为单栈,两栈互斥不再适用。

> **栈所有权(已定)**:v1 采用"并行但互斥"。`start_lgw_arm_servo.sh` 独立,
> preflight 必须确认没有另一个 `controller_manager` / `move_group` 在跑,否则拒绝启动。
> `start_lgw_arm.sh` 与 `bt_with_lgw_moveit.launch.py` 本期不改;BT 阶段再统一。

### 5.3 接管 / 释放时序

| 阶段 | 动作 |
|---|---|
| bringup | 调用一次 `/servo_node/start_servo`;用 `/servo_node/status` 或命令输出确认真的启动 |
| takeover | 检查 `/move_action` 与 FJT status 都无活动 goal → `unpause_servo`;Servo 开始向 JTC topic 发单点轨迹 |
| 伺服中 | 监控两者 status;发现活动 goal(MoveGroup 规划/执行开始)→ `pause_servo` + HOLD |
| release | `pause_servo`;Servo 停发;JTC 保持最后一点;MoveGroup 可执行新 action |

- "忙"的判据有两条,**缺一不可**:
  - `/move_action/_action/status` 存在 `ACCEPTED`/`EXECUTING` goal —— 覆盖 MoveGroup 的**规划阶段**
    (此时 JTC 上还没有 FJT goal,FJT 判据是空的);
  - `/arm_controller/follow_joint_trajectory/_action/status` 存在活动 goal。
- `takeover` 在忙时默认拒绝(`success=false, "move_group busy"`)。
- `HOLD` 用 `pause_servo`,不释放 Servo 进程。
- 释放后不需要单点轨迹补救:JTC 的最后指令就是 Servo 的最后一点,等于当前位姿。

### 5.4 驱动模式(进程级)

`fast_mode` / `control_mode` 是驱动进程级参数,启动后不能通过 `ros2 param set` 改变
(驱动在 `__init__` 读一次,无动态参数回调)。

单栈里只能选一个模式,对 MoveIt 和 Servo **同时生效**:

| 模式 | MoveIt | Servo | 说明 |
|---|---|---|---|
| `move_j` | 正常 | 可能滞后 | 最安全 |
| `move_js` | 较生硬 | 响应快 | SDK 标注高风险 |
| `cpv`(若采用) | 位置流 + 限速 | 位置流 + 限速 | 需驱动补丁 |

先用 Task 7A 评估 `move_j` / `move_js`;若都不满足,再评估 CPV。
正常栈固定 `move_j`;伺服栈/单栈按评估结果选择。

---

## 6. L2:moveit_servo

### 6.1 依赖

- `apt install ros-humble-moveit-servo`(容器内;验证通过后再固化 Dockerfile)
- 版本参考:host 源码 `/home/unitree/ros2_ws/src/moveit2/moveit_ros/moveit_servo`,2.5.9

### 6.2 参数

```yaml
moveit_servo:
  ros__parameters:
    move_group_name: arm
    planning_frame: base_link
    robot_link_command_frame: base_link
    ee_frame_name: tcp_link

    command_in_type: "speed_units"
    command_out_type: trajectory_msgs/JointTrajectory
    publish_joint_positions: true
    publish_joint_velocities: true
    publish_joint_accelerations: false
    command_out_topic: /arm_controller/joint_trajectory

    joint_topic: /feedback/joint_states   # 台架改为 /control/joint_states
    is_primary_planning_scene_monitor: false
    monitored_planning_scene_topic: /monitored_planning_scene

    incoming_command_timeout: 0.3
    publish_period: 0.01
    low_latency_mode: false
    num_outgoing_halt_msgs_to_publish: 4

    check_collisions: true
    collision_check_rate: 10.0
    self_collision_proximity_threshold: 0.02
    scene_collision_proximity_threshold: 0.03
    joint_limit_margin: 0.1
    lower_singularity_threshold: 17.0
    hard_stop_singularity_threshold: 30.0
    leaving_singularity_threshold_multiplier: 2.0
    smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"

    cartesian_command_in_topic: /servo_node/delta_twist_cmds
    joint_command_in_topic: /servo_node/delta_joint_cmds
    status_topic: /servo_node/status
```

约束(源码已核实):

- `command_out_type` 只接受 `trajectory_msgs/JointTrajectory` 与 `std_msgs/Float64MultiArray`。
- 本方案用 `JointTrajectory` → JTC;`publish_joint_positions` / `publish_joint_velocities` 可同时为 true。
- 用 `Float64MultiArray` 时两者只能二选一(本方案不使用)。
- `joint_topic` 必须与 `rsp.launch.py` 的 joint states 源一致:
  真机(`follow:=true`)为 `/feedback/joint_states`,台架(`follow:=false`)为 `/control/joint_states`。
  否则 Servo 的 `waitForCompleteState` 拿不到数据,闭环退化为开环。
- 本节点读当前 tcp 位姿同样走 TF,其上游就是同一个 joint states 源,两者天然一致。
- JTC 的 `allow_nonzero_velocity_at_trajectory_end` 保持默认 `true`;
  若设为 `false`,Servo 的非零末端速度会被 JTC 拒绝。

### 6.3 频率语义

- `controller_manager.update_rate` = 200Hz(控制/状态环)
- `/feedback/joint_states` = 200Hz(真机);`/arm_controller/joint_trajectory` 话题 = 100Hz(Servo 输出)
- Servo 输出频率 = `1/publish_period`(本方案 100Hz)
- 目标/setpoint 更新 = tracker 的 10-15Hz
- **控制环频率 ≠ 目标更新频率**:误差每 10ms 重算,目标每 70-100ms 更新。

---

## 7. L3:伺服服务端节点

### 7.1 状态机

```
          takeover                unpause_servo 成功
IDLE ───────────────▶ PREEMPT ──────────────────▶ TRACKING
 ▲                       │                          │  ▲
 │                       │ unpause_servo 失败        │  │ TARGET_OK + RESUME
 │                       ▼                          │  │(latch 必须仍然有效)
 │                     FAULT                       HOLD
 │                                                    ▲
 │                                    TARGET_LOST / PAUSE
 │                                                    │
 │                   release(PREEMPT/TRACKING/HOLD)   │ release
 └────────────────── RELEASING ◀──────────────────────┘
                        │ PAUSE_OK
                        ▼
                       IDLE
```

- 超时/目标丢失只会 `TRACKING → HOLD`(由 `TARGET_LOST` 驱动),**不会**进 `FAULT`。
- `TARGET_OK` 是 latch:+ `TARGET_LOST` / `PAUSE` / 离开 `HOLD` 都清零;`HOLD + RESUME` 必须有新的 `TARGET_OK`。
- `RECOVERING`(2026-11-03 扩展,见
  `2026-11-03-hold-recovery-design.md` §6)是 `HOLD` 的主动子状态:
  - `HOLD + RECOVER_START` → `RECOVERING`(Servo 保持 paused,本节点用 MoveGroup 驱动 JTC);
  - `RECOVERING + RECOVER_OK + TARGET_OK` → `TRACKING`;无 `TARGET_OK` → `HOLD`;
  - `RECOVERING + RECOVER_FAIL` → `HOLD`;
  - `RECOVERING` 中 `TARGET_OK/TARGET_LOST` 只改 latch;`PAUSE` 接受且不变;
  - `RECOVERING + RESUME` 拒绝(Servo 不应成为第二个 JTC 写者),打断用 `RELEASE`。
- `RELEASE` 幂等:`IDLE + RELEASE`、`RELEASING + RELEASE` 返回接受、状态不变;只有 `FAULT + RELEASE` 拒绝(必须 `RESET`)。

- `PREEMPT` / `RELEASING` 同步等待 Servo `unpause/pause` 响应,不发布 twist。
- `TRACKING → HOLD` 只停指令,不释放控制权。
- 任意异常路径必须保证:Servo 已 `pause`(MoveGroup 可执行),或明确停在 `FAULT` 并上报。
- `TRACKING` 内部按几何源分三种子状态(head/arm/odom),但对外仍报 `TRACKING` +
  `geometry_source` 字段。

### 7.2 接口

| 接口 | 类型 | 语义 |
|---|---|---|
| `/lgw/servo/takeover` | `lgw_bt_planner/srv/Takeover` | 接管并开始跟踪 |
| `/lgw/servo/release` | `std_srvs/Trigger` | 停止并交还控制权 |
| `/lgw/servo/pause` | `std_srvs/Trigger` | 保持控制权,转 HOLD |
| `/lgw/servo/resume` | `std_srvs/Trigger` | HOLD → TRACKING |
| `/lgw/servo/state` | `lgw_bt_planner/msg/ServoState` | 状态与误差 |

```srv
# srv/Takeover.srv
uint64 track_id        # 0 = auto(odom 系距 base_link 最近的 confirmed track)
string class_filter    # 空 = 不限类别
---
bool success
string message
string state
```

```msg
# msg/ServoState.msg
string state             # IDLE / PREEMPT / TRACKING / HOLD / RELEASING / FAULT
string geometry_source   # head / arm / odom / none
uint64 track_id
float64 angular_error_rad
float64 position_error_m
float64 target_distance_m
bool converged
bool saturated
bool too_close
builtin_interfaces/Time stamp
```

### 7.3 目标选择

实现为**纯模块** `select_target()`(`target_selector.hpp/.cpp`),不依赖 ROS,
输入是最新数组中候选的拷贝 + `base_link` 原点在 odom 系的位置,输出选中的下标与理由。
节点只负责把 `UnifiedTrackArray` 转成 `TrackCandidate` 并调用它。

规则:

- `track_id != 0`:使用指定 track(不看 status;过期与否由 §7.4 几何源判定)。
- `track_id == 0`:选 odom 系下距 `base_link` 原点最近的 `confirmed` track。
  需要 `T_base_odom` 的平移;拿不到 → `no_base_pose`,节点进 `HOLD`(不自作主张按别的指标排)。
- `class_filter` 非空时先按类别过滤。
- 指定 track 被删除(非 coasting):
  - 默认 → `HOLD`;
  - 可选 `reacquire_on_loss`(默认 false):同类别 + 距上次已知位置 < `reacquire_radius_m`(0.5m)
    + `confirmed`,满足则改追。
- **MoveGroup 忙时**:默认拒绝 takeover(返回 `success=false`,
  `message="move_group busy"`),避免把正在执行的轨迹截断在半路。
  判据见 §5.3(`/move_action` + FJT 两个 status)。
  可选参数 `force_cancel_moves`(默认 false):**v1 不实现取消**——置 true 时 takeover 直接返回
  `success=false, message="force_cancel_moves not implemented in v1"`,而不是假装支持。

`reason` 取值:`requested` / `nearest` / `reacquired` / `requested_missing` /
`no_candidate` / `no_base_pose`,直接进 `ServoState.message` 便于现场定位。
- **MoveGroup 正在执行轨迹时**:默认拒绝 takeover(返回 `success=false`,
  `message="move_group busy"`),避免把正在执行的轨迹截断在半路。
  可选 `force_cancel_moves`(默认 false):先取消活动 goal,再接管。

> **v1 不做候选池与自动切换。** `track_id=0` 只表示"选最近的 confirmed track",
> 不做排序打分、不做多目标驻留、不做自动切换。v2 方案与前置改造见 §8.5。

> **v1 不做候选池与自动切换。** `track_id=0` 只表示"选最近的 confirmed track",
> 不做排序打分、不做多目标驻留、不做自动切换。v2 方案与前置改造见 §8.5。

### 7.4 几何源三档

每个控制周期(100Hz)选一次:

| 优先级 | 条件 | 使用 |
|---|---|---|
| 1 | head 贡献者 `age < head_max_age_ms`(400ms) | head 的 `position_arm_base` |
| 2 | arm 贡献者 `age < arm_max_age_ms`(400ms) | arm 的 `position_arm_base` |
| 3 | track 存在且 `time_since_last_observation_ms ≤ odom_fallback_max_age_ms`(1500ms) | odom KF 反算 |
| 4 | 以上都不满足 | HOLD(不发运动) |

- **age 的基准是 detection 的 capture stamp**(`now - contributor_stamp`)。
  本仓库实测 YOLOE 推理耗时 **170–190ms**(`docs/2026-10-23-tracker-clock-sync-and-lookup-fastfail.md:146`),
  因此 200ms 门限会让稳态 age 紧贴阈值、几何源频繁降级。**默认取 400ms**,留出一个丢帧周期的余量。
- **age 的基准是 detection 的 capture stamp**(`now - contributor_stamp`)。
  本仓库实测 YOLOE 推理耗时 **170–190ms**(`docs/2026-10-23-tracker-clock-sync-and-lookup-fastfail.md:146`),
  因此 200ms 门限会让稳态 age 紧贴阈值、几何源频繁降级。**默认取 400ms**,留出一个丢帧周期的余量。
- **降级立即生效**(安全方向不延迟)。
- **升级需滞回**:高优先级源连续 `upgrade_hysteresis_ticks`(默认 30 tick ≈ 300ms)有效才切回。
- **低优先级降增益**:arm 档 0.7×,odom 档 0.5×。

**tracker 轨迹删除的精确语义(已核实源码)**

| 事件 | 判据 | 默认值 |
|---|---|---|
| `confirmed → coasting` | `time_since_last_obs_ms ≥ coast_enter_ms` 且 `> dt_ms` | 300ms |
| `→ deleted`(停止发布) | `time_since_last_obs_ms ≥ coast_max_age_ms` | 1500ms |

- `dt_ms` 是**两次 pipeline tick 的间隔**,不是观测间隔;因此删除实际发生在
  越过 1500ms 后的第一个 tick,约 **1500–1600ms**。
- **重要陷阱**:pipeline 由 `ObservationResult` 到达触发,没有周期定时器。
  若两台相机/YOLOE 完全断流,`tick_lifecycle` 不再运行,`time_since_last_obs_ms`
  会**冻结**,轨迹永远不会被删除。
- 因此伺服侧不能只信 tracker 的字段,必须自己维护看门狗:
  - `UnifiedTrackArray` 超过 `tracker_stream_timeout_ms`(默认 500ms)未更新 → HOLD;
  - 指定 track 从最新数组中消失 → 按"删除"处理。

**tracker 轨迹删除的精确语义(已核实源码)**

| 事件 | 判据 | 默认值 |
|---|---|---|
| `confirmed → coasting` | `time_since_last_obs_ms ≥ coast_enter_ms` 且 `> dt_ms` | 300ms |
| `→ deleted`(停止发布) | `time_since_last_obs_ms ≥ coast_max_age_ms` | 1500ms |

- `dt_ms` 是**两次 pipeline tick 的间隔**,不是观测间隔;因此删除实际发生在
  越过 1500ms 后的第一个 tick,约 **1500–1600ms**。
- **重要陷阱**:pipeline 由 `ObservationResult` 到达触发,没有周期定时器。
  若两台相机/YOLOE 完全断流,`tick_lifecycle` 不再运行,`time_since_last_obs_ms`
  会**冻结**,轨迹永远不会被删除。
- 因此伺服侧不能只信 tracker 的字段,必须自己维护看门狗:
  - `UnifiedTrackArray` 超过 `tracker_stream_timeout_ms`(默认 500ms)未更新 → HOLD;
  - 指定 track 从最新数组中消失 → 按"删除"处理。

odom 反算:

```
p_armbase = T_base_odom(now)⁻¹ · p_odom        # 挂载偏移视为单位阵
```

用 `tf2_ros::Buffer` 查询 `odom ← base_link`;先 `can_transform(timeout=0)` 探测,
再 `lookup_transform`,避免缺帧时空等阻塞执行器。

### 7.5 lookat 位姿

输入:目标在臂基座系的位置 `P_target`。

1. 选择 tcp 位置(距离自由度):
   - `r_xy = ‖(P_target.x, P_target.y)‖`。
   - 若 `r_xy < 1e-3`:回落到 `ee_anchor`(默认 `0.4;0.0;0.2`)。
   - 否则:`P_ee = (d_xoy·P_target.x/r_xy, d_xoy·P_target.y/r_xy, z_fixed)`,
     `d_xoy = 0.4`,`z_fixed = 0.3`。
2. `direction = normalize(P_target - P_ee)`。
3. `R_arm_in_base = apply_uprightness_flip(build_lookat_orientation(direction, (0,0,1)), (0,0,1))`。
   前者处理 `direction ≈ ±world_up` 的退化情形(失败抛异常),后者保证 `cam_y·base_z < 0`(相机不倒置)。
   翻转逻辑抽成 `apply_uprightness_flip` 共享函数,与两个 BT 节点
   (`ComputeLookatPoseNode` / `ComputeGraspPose`)调用同一份实现,不再三处各写一遍。
4. `tcp_pose = compute_tcp_pose(P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp)`。
   `R_arm_in_tcp` / `t_arm_in_tcp` 来自 `calibration/arm_calibration.json`。
5. **`LookatResult` 不含距离。** 相机到目标的距离必须用**当前**相机位姿算
   (`cam_now = tcp_now.p + tcp_now.R · t_cam_in_tcp`),不是期望位姿——期望的 standoff
   恒为 `d_xoy=0.4`,拿它算距离永远 ≥0.4,近距保护会变成死代码。该值由节点在 §7.6 计算。

以上函数直接复用 `lgw_bt_planner` 已导出的 `parse_calibration_json`、
`build_lookat_orientation`、`compute_tcp_pose`,不重写。

### 7.6 控制律

**参考点:`tcp_link`。** Servo 的雅可比在 SRDF `arm` 组的 tip 上求值
(`servo_calcs.cpp:571`),本仓库该 tip 就是 `tcp_link`;IK 分支用
`ik_solver_->getTipFrame()`,同样是 `tcp_link`。`compute_tcp_pose()` 输出的期望位姿
也正好是 `tcp_link` 的位姿(标定文件 `arm_calibration.json` 已含 tcp 偏置)。

因此**不需要任何 link6 ↔ tcp 转换**:期望位姿与当前位姿都在 `tcp_link` 上,
误差与雅可比同点,不会引入 `ω × 0.12` 的耦合项。

- **角误差**:`e_ang = angle(tcp_desired.orientation · tcp_current.orientation⁻¹)`,按 2° 判据。
- **位置误差**:`e_pos = tcp_desired.position - tcp_current.position`。

```
e_pos = tcp_desired.position - tcp_current.position
e_ang = angle(tcp_desired.orientation · tcp_current.orientation⁻¹)

v = clamp(Kp_pos · e_pos, v_max) * source_scale
ω = clamp(Kp_ang · e_ang, ω_max) * source_scale
```

- 默认:`v_max = 0.15 m/s`、`ω_max = 0.6 rad/s`、`Kp_pos = 2.0`、`Kp_ang = 3.0`。
- 死区(严格对准):`e_ang < 2°` 且 `‖e_pos‖ < 1cm` → 输出零 twist,但仍持续发布,
  避免 Servo 因 `incoming_command_timeout` 反复 halt。
- 误差先过一阶低通(截止 5-10Hz),再做速率限幅,抑制行走时围绕零误差的高频抖动。
- 不可达检测:`e_ang` 连续 3s 不减反增 → `UNREACHABLE` → HOLD + `saturated=true`。
- 近距保护:**当前**相机位置到目标距离 < `min_observe_distance_m`(0.35m)→ `too_close=true`(不自动退出)。
  相机位置 = `tcp_now.p + tcp_now.R · t_cam_in_tcp`,用 TF 的当前 tcp 位姿 + 标定偏置,不用期望位姿。

输出:`geometry_msgs/TwistStamped`,`frame_id = base_link`,发布到 `/servo_node/delta_twist_cmds`。

### 7.7 线程与频率

- 控制定时器 100Hz,挂在独立的 `MutuallyExclusive` callback group。
- 订阅(`/perception/tracks/unified`、`joint_states`、`/move_action` 与 FJT 的 status)
  挂在 `Reentrant` group。
- `tf2_ros::Buffer` 使用 `setUsingDedicatedThread(true)`;查 TF 前先零超时 `can_transform`。
- Servo `start/pause/unpause` 客户端挂独立 callback group,避免与定时器互锁。
- `MultiThreadedExecutor` 至少 2 线程。

### 7.8 错误处理

| 条件 | 行为 |
|---|---|
| Servo start/pause/unpause 调用失败或超时 | `FAULT`,不发 twist |
| 位姿流:`pose_watch_topic` 超过 100ms 未收到 joint_states | `HOLD`(TF 随之冻结,不能继续闭环保留旧位姿) |
| TF `arm1/base_link ← arm1/tcp_link` 不可用 | `HOLD`(启动阶段则拒绝 takeover) |
| tracker 无有效 track | `HOLD`(已接管时) |
| `UnifiedTrackArray` 超过 500ms 未更新 | `HOLD`(tracker 生命周期可能已冻结,见 §7.4) |
| `UnifiedTrackArray` 超过 500ms 未更新 | `HOLD`(tracker 生命周期可能已冻结,见 §7.4) |
| TF `odom←base_link` 不可用 | odom 档不可用;若此时无相机源 → `HOLD` |
| 标定文件缺失/解析失败 | 启动即 `FAULT`,拒绝 takeover |
| Servo 状态 `HALT_FOR_SINGULARITY` / 碰撞降速 | 记录到 state 日志;持续 > 3s → `HOLD` |

### 7.9 Servo 进程生命周期

已核实(`servo_node.cpp`、`servo.cpp`、`servo_calcs.cpp`):

| 行为 | 事实 |
|---|---|
| `/servo_node/start_servo` | 调 `Servo::start()` → `waitForCompleteState("arm", 10s)`;超时只打 ERROR 并 **return**,但服务仍返回 `success=true` |
| 重复调用 `start_servo` | `ServoCalcs::start()` 先 `stop()` 并 join 线程,因此**不会崩溃**;但会重建线程并把 `last_sent_command_` 重置为当前 state |
| `/servo_node/stop_servo` | 与 `pause_servo` 等价,都是 `setPaused(true)` |
| `/servo_node/pause_servo` / `unpause_servo` | 暂停 / 恢复计算线程 |

因此:

- `start_servo` 的调用时机:bringup 一次(`ros2_control` 起来且 joint_states 已有数据
  之后),**并且每次 takeover 摆位完成后再调用一次**(2026-11-02 修订,原因见
  `docs/servo/2026-11-02-servo-state-monitor-stuck.md` §10–11)。
  原规则"整个进程生命周期内只调用一次"已废弃:`last_sent_command_` 只在
  `ServoCalcs::start()` 播种,而控制律在死区内会发逐位为零的 twist,Servo 对零 twist
  原样复播该缓存;若缓存仍是 bringup 时的 home 零位,首拍零指令就会把臂拽回零位。
  重调一次即把缓存重置为摆位后的当前 state(见本表第 2 行),Servo 对"重复调用"是安全的。
- `HOLD → TRACKING` 用 `unpause_servo`;`TRACKING → HOLD` 用 `pause_servo`。
- **不能信任 `start_servo` 的返回值**:必须用 `/servo_node/status` 或
  "是否收到 `/arm_controller/joint_trajectory` 输出"确认真的启动;超时则 `FAULT`。
- `ServoCalcs::start()` 会把 `last_sent_command_` 初始化为当前 state,启动瞬间不会跳变。

### 7.9 Servo 进程生命周期

已核实(`servo_node.cpp`、`servo.cpp`、`servo_calcs.cpp`):

| 行为 | 事实 |
|---|---|
| `/servo_node/start_servo` | 调 `Servo::start()` → `waitForCompleteState("arm", 10s)`;超时只打 ERROR 并 **return**,但服务仍返回 `success=true` |
| 重复调用 `start_servo` | `ServoCalcs::start()` 先 `stop()` 并 join 线程,因此**不会崩溃**;但会重建线程并把 `last_sent_command_` 重置为当前 state |
| `/servo_node/stop_servo` | 与 `pause_servo` 等价,都是 `setPaused(true)` |
| `/servo_node/pause_servo` / `unpause_servo` | 暂停 / 恢复计算线程 |

因此:

- 整个进程生命周期内**只调用一次 `start_servo`**,时机 = `ros2_control` 起来且
  `/feedback/joint_states` 已有数据之后。
- `HOLD → TRACKING` 用 `unpause_servo`;`TRACKING → HOLD` 用 `pause_servo`。
- **不能信任 `start_servo` 的返回值**:必须用 `/servo_node/status` 或
  "是否收到 `/servo_position_controller/commands`"确认真的启动;超时则 `FAULT`。
- `ServoCalcs::start()` 会把 `last_sent_command_` 初始化为当前 state,启动瞬间不会跳变。

---

## 8. 3D tracker 改造

### 8.1 消息扩展

`yoloe_interfaces/msg/UnifiedTrack.msg` 增加:

```msg
# === 伺服几何(臂基座系,纯已知链) ===
geometry_msgs/Point position_arm_base
string geometry_frame                              # "arm1/base_link"
builtin_interfaces/Time last_observation_stamp     # 真实观测时刻

# === 每相机贡献(供几何源选择) ===
string[] contributor_cameras
geometry_msgs/Point[] contributor_positions_arm_base
builtin_interfaces/Time[] contributor_stamps
float32[] contributor_confidences
```

`position`(odom KF)保留不变,继续负责跨帧归属与去重。

### 8.2 适配器改造

`input_adapter.py` 每条 detection 额外计算臂基座坐标,两条相机链**统一走 TF**,
不再在 TF 之外手算标定矩阵:

| 相机 | 链 |
|---|---|
| head | `lookup(arm1/base_link ← camera_head_color_optical_frame, 观测 stamp) · p_cam` |
| arm | `lookup(arm1/base_link ← camera_arm_color_optical_frame, 观测 stamp) · p_cam` |

两条链都由 tracker 自己发布的安装桥接通:`arm1/base_link → camera_head_link`、
`arm1/tcp_link → camera_arm_link`(`broadcast_camera_mount_tf`,数值由同一份标定 JSON 合成),
再由 realsense 的静态链到 `..._color_optical_frame`。
`docs/tf-frame-prefix.md` §5 已实测"分段矩阵合成 ≡ 整链 TF",最大偏差 5.6e-16,
两种写法数值等价。

为什么不再走矩阵:

- 同一变换只保留一处来源(TF)。标定改了不会出现"桥更新了、adapter 没更新"的双实现漂移;
- head / arm 两条链同构,不再有"head 不用 TF、arm 用 TF"的不对称;
- lookup 的起点是 `arm1/base_link`(带前缀),不经过机器人本体的 `base_link`,
  不会触发当年 `base_link` 双 parent 的歧义。

代价:安装桥要等 realsense 的内部链出现后才发布(0.2s 轮询,首次成功即发一次)。
在那之前**整条相机链都不可用**——不只 `position_arm_base`,`position`(odom)也依赖同一条桥,
因此 `adapt()` 对整个批次返回空,表现为"该相机此刻没有观测"。启动后约 0.2s 内如此,之后恢复正常。

同时把 `position_arm_base`、stamp、置信度写入 `TrackObservation`,再由 `output_formatter`
按相机分别汇总为 `contributor_*` 数组。

### 8.3 时间戳

- `UnifiedTrack.stamp` 保持现状(pipeline now)。
- 新增 `last_observation_stamp` = 实际触发该 track 最近一次更新的 detection stamp。
- `contributor_stamps` 每项为该相机贡献的 detection stamp。

### 8.4 兼容性

- `UnifiedTrackArray` 当前无下游消费者,字段只增不改,不破坏现有代码。
- odom KF 与 `position` 语义不变,`arm_mount_offset` 继续只影响 odom 派生。

### 8.5 v2 预留:长保留与候选池(不在本期实现)

目标:让伺服在目标暂时丢失时能从历史轨迹池中重捕获/切换。

**已核实的现状约束**

| 事实 | 位置 | 影响 |
|---|---|---|
| 删除判据是 `time_since_last_obs_ms >= coast_max_age_ms` | `track.py:111` | 设 `-1` 恒为真 → **会立刻删光**;必须改成"负值 = 禁用" |
| 删除判据**不区分状态** | 同上 | tentative 误检也会驻留;拉长阈值会污染候选池 |
| 没有 `max_tracks` 上限 | `tracker.py` | 长保留会导致轨迹数无界增长,`predict_all` 与匈牙利关联成本上升 |
| 关联门控是**欧氏距离** 0.15m | `cost_matrix.py` | 旧轨迹不会抢远处观测,但跨时间重检出新位置会新建重复轨迹 |
| 每条未关联观测都新建 tentative | `tracker.py:189-200` | 误检持续产生新轨迹 |

**v2 设计要点(建议)**

1. **分状态保留**:tentative 1–2s;confirmed/coasting 可配(10–30s);另加 `max_tracks`(如 200)按最后观测时间 LRU。
2. **负值语义**:`coast_max_age_ms < 0` 表示不按时间删除,但**必须**有 `max_tracks` 兜底。
3. **跨时间去重**:同一类别 + 位置差 < `duplicate_track_distance_m` 的旧 coasting 轨迹,应被新轨迹合并或淘汰。
4. **排序不用"状态 + 距离"**:状态在 300ms 后饱和,无法区分"刚丢失"与"丢了 30s"。改用含新鲜度的打分:
   `score = w_status + w_recency·exp(-age/tau) + w_dist·(1-d/dmax) + w_conf·confidence`
5. **切换滞回**:当前目标 coasting 超过 `switch_after_ms` 且候选分数高出 `switch_margin` 才切换。
6. **语义隔离**:长保留只用于**身份/选择**;几何仍只来自新鲜观测或 ≤1.5s 的 odom 兜底。

> 结论:长保留 + 自动选择的思路可行,但必须按上述 6 点改造;
> 直接用"-1 + 状态/距离排序"不可靠。

### 8.5 v2 预留:长保留与候选池(不在本期实现)

目标:让伺服在目标暂时丢失时能从历史轨迹池中重捕获/切换。

**已核实的现状约束**

| 事实 | 位置 | 影响 |
|---|---|---|
| 删除判据是 `time_since_last_obs_ms >= coast_max_age_ms` | `track.py:111` | 设 `-1` 恒为真 → **会立刻删光**;必须改成"负值 = 禁用" |
| 删除判据**不区分状态** | 同上 | tentative 误检也会驻留;拉长阈值会污染候选池 |
| 没有 `max_tracks` 上限 | `tracker.py` | 长保留会导致轨迹数无界增长,`predict_all` 与匈牙利关联成本上升 |
| 关联门控是**欧氏距离** 0.15m | `cost_matrix.py` | 旧轨迹不会抢远处观测,但跨时间重检出新位置会新建重复轨迹 |
| 每条未关联观测都新建 tentative | `tracker.py:189-200` | 误检持续产生新轨迹 |

**v2 设计要点(建议)**

1. **分状态保留**:tentative 1–2s;confirmed/coasting 可配(10–30s);另加 `max_tracks`(如 200)按最后观测时间 LRU。
2. **负值语义**:`coast_max_age_ms < 0` 表示不按时间删除,但**必须**有 `max_tracks` 兜底。
3. **跨时间去重**:同一类别 + 位置差 < `duplicate_track_distance_m` 的旧 coasting 轨迹,应被新轨迹合并或淘汰。
4. **排序不用"状态 + 距离"**:状态在 300ms 后饱和,无法区分"刚丢失"与"丢了 30s"。改用含新鲜度的打分:
   `score = w_status + w_recency·exp(-age/tau) + w_dist·(1-d/dmax) + w_conf·confidence`
5. **切换滞回**:当前目标 coasting 超过 `switch_after_ms` 且候选分数高出 `switch_margin` 才切换。
6. **语义隔离**:长保留只用于**身份/选择**;几何仍只来自新鲜观测或 ≤1.5s 的 odom 兜底。

> 结论:长保留 + 自动选择的思路可行,但必须按上述 6 点改造;
> 直接用"-1 + 状态/距离排序"不可靠。

---

## 9. 数据流

```
YOLOE (head/arm) ──ObservationResult──▶ lgw_3d_tracker
                                          │ 每条 detection:
                                          │   ├─ p_odom     (KF / 归属)
                                          │   └─ p_armbase  (新增 / 伺服几何)
                                          ▼
                       /perception/tracks/unified (UnifiedTrackArray)
                                          │
                                          ▼
                                   lgw_servo_node
                     ├─ 目标选择(track_id / class / nearest)
                     ├─ 几何源三档(head → arm → odom)
                     ├─ lookat 位姿(arm base)
                     ├─ 100Hz P 环(TF 取当前 tcp_link 位姿)
                     └─ TwistStamped(frame_id=base_link)
                                          ▼
                          /servo_node/delta_twist_cmds → moveit_servo
                                          ▼
                          /arm_controller/joint_trajectory
                                          ▼
                          arm_controller (JTC) → mock → broadcaster
                                          ▼
                              control/joint_states → agx_arm_ctrl → CAN
```

---

## 10. 测试策略

### 10.1 单元测试(gtest,链接 `lgw_bt_planner_nodes`)

| 测试文件 | 覆盖 |
|---|---|
| `tests/test_servo_control_law.cpp` | 饱和、死区、低通、增益缩放、不可达判定 |
| `tests/test_geometry_source.cpp` | 三档优先级、降级立即生效、升级滞回、odom 反算 |
| `tests/test_servo_state_machine.cpp` | 状态迁移、失败路径、HOLD 不释放控制权 |
| `tests/test_lookat_setpoint.cpp` | `P_ee` 选位、退化回落、光轴与方向一致 |

> 本容器不能跑 `colcon test`(缺 `ament_cmake_test`),直接运行 build 下的 gtest 二进制。

### 10.2 台架集成(无真机)

台架不启动 `agx_arm_ctrl`,以 `follow:=false` 启动 `demo.launch.py`:
此时 `rsp.launch.py` 订阅 `control/joint_states`(mock broadcaster 的输出),
`arm1/base_link → arm1/tcp_link` 的 TF 仍然有效,且跟随 JTC 的指令。
因此**台架可以直接跑完整闭环**:Servo → JTC → mock → broadcaster → TF → 控制律,
只是被控对象没有动力学(指令即状态),误差应快速收敛到 0。

- Servo 的 `joint_topic` 设为 `/control/joint_states`;节点的 `pose_watch_topic` 同样设为它。
- 不需要 fake `/feedback/tcp_pose`,也不需要内置 FK 回退。
- `start_servo` 仍需在 joint states 有数据之后调用,并校验确实启动(见 §7.9)。
- 真机阶段把 `follow` 切到 `true`,上述两条话题自动换成 `/feedback/*`。

| 用例 | 方法 | 通过 |
|---|---|---|
| Servo topic 输出 | `ros2 topic hz /arm_controller/joint_trajectory` | takeover 后有稳定输出 |
| 接管往返 | 连续 20 次 takeover/release | 全部成功,无崩溃 |
| Servo 参数加载 | 启动 `servo_node_main`,看 `/servo_node/status` | 无参数校验失败 |
| Goal 冲突保护 | 伺服中发一个 MoveGroup goal(含规划阶段) | `pause_servo` + HOLD,或 goal 被拒 |
| 闭环收敛 | 静态假 track,mock 被控对象 | `angular_error_rad` 收敛 < 2° |
| 释放连续性 | **真机阶段**:release 后采样 `/feedback/joint_states` | 满足 A6 阈值 |

### 10.3 真机验收

**前置条件**:YOLOE(双相机)与 `lgw_3d_tracker` 必须先运行。
`start_lgw_full.sh` 不负责拉起它们;脚本启动时 precheck
`/perception/tracks/unified` 是否有消息,没有则报错退出。

按 §1.5 的 A3–A6 执行,先静止、后行走、逐步加速。

---

## 11. 风险与待验证

| # | 风险 | 处置 | 状态 |
|---|---|---|---|
| R1 | MoveGroup action 与 Servo topic 同时喂 JTC(后到覆盖先到) | §5.3:takeover 检查 `/move_action` + FJT 两个 status、伺服中同时监控、BT 串行;台架 Goal 冲突用例 | 设计已定 |
| R2 | Servo 的 `JointTrajectory` 被 JTC 拒绝(末端速度/关节名) | 已核实 Humble JTC `allow_nonzero_velocity_at_trajectory_end` 默认 `true`,非零末端速度可通过;关节名须与 `arm_controller.joints` 完全一致;台架看 JTC 日志 | 已核实 |
| R3 | Servo PSM 在带前缀 TF 下异常 | 台架启动检查;必要时调整 frame 参数 | 待实测 |
| R4 | 行走基座扰动 | 低通 + 速率限幅 + 2° 死区;逐步加速验证 | 待实测 |
| R5 | 目标进入可达边界 | `saturated` 上报,本期不转体 | 设计已定 |
| R6 | 行走时 Go2 身体不在 planning scene | **已决:v1 不纳入**。只接入已存在的 `apply_head_camera_collision_node`,身体碰撞靠关节限位 + 工作空间钳制兜底 | 已决(v2 再评估) |
| R7 | 10-15Hz 目标 + 100Hz 输出造成滞后 | 低通/增益调参;静态目标下可接受 | 待实测 |
| R8 | Orin NX CPU(YOLOE + Servo 碰撞 + JTC) | `collision_check_rate` 可下调;真机监控 | 待实测 |
| R9 | 驱动侧 `control_mode` 未实现;CPV 需改上游驱动 | **本期不做**(见 D21):先用 `move_j` / `move_js`。若 7A 判定必须 CPV,再在 `servo-cpv` 分支追加补丁 | 已决 |
| R10 | 驱动模式在单栈里只能选一个,对 MoveIt 与 Servo 同时生效 | Task 7A 评估 `move_j` / `move_js`;必要时再评估 CPV | 待实测 |
| R11 | 位姿源与 Servo `joint_topic` 不一致(一个 `feedback/` 一个 `control/`) | 闭环变成开环且不报错。启动自检:两话题在同一时刻都有数据;`lgw_full.launch.py` 里由 `follow` 统一派生 | 设计已定 |
| R12 | **启动时 JTC 保持 `initial_positions.yaml`(全零),把真机拉回 home** | JTC 是位置控制器,每周期都写末次目标;启动即保持 0 → `control/joint_states = 0` → 驱动跟随。**操作约定:启动栈前臂必须在 home**(`start_lgw_full.sh` 启动前检查)。运行中不受影响 | 已决(A)/待实现检查 |
| R13 | JTC 保持的是 mock 域位置,真机经 `control/joint_states` 跟随 | 臂被遮挡跟不上时 JTC 仍继续命令它去 mock 位置,不构成闭环保护;靠关节限位 + 工作空间钳制兜底 | 已知,不处理 |
| R9 | 驱动侧 `control_mode` 尚未实现;伺服若要使用 CPV 需额外补丁 | CPV 已在 `servo-cpv` 分支实测可用(§12.5);`control_mode` 路由与启动配置待实现 | 待决策 |

---

## 12. 决策记录

| # | 决策 | 理由 |
|---|---|---|
| D1 | 接缝用 JTC topic(Servo → `/arm_controller/joint_trajectory`) | JTC 是唯一写者;不新增 controller,无释放回跳 |
| D2 | 用 MoveIt Servo 而非自写闭环 | 复用 IK/雅可比/碰撞/限位/平滑 |
| D3 | 不用 `PoseTracking::moveToPose` | 阻塞式 waypoint,不适合连续跟踪 |
| D4 | 自研 100Hz 连续 P 环 | 目标 10-15Hz,控制环需独立高频 |
| D5 | 几何源 head → arm → odom | head 稳(纯扰动抑制);arm 覆盖强但有自激;odom 常量偏置兜底 |
| D6 | odom 兜底窗口 1500ms | 静态目标假设;动目标需缩到 ≤500ms |
| D7 | 死区 2° / 1cm,严格对准 | 用户选择持续微调 |
| D8 | 完成判据 = 显式 release | 自动还权有失控风险 |
| D9 | 接口用服务 + state 话题 | 接管是持续状态,不是可完成 goal |
| D10 | `xoy_distance_m=0.4`、`z_fixed_m=0.3` 固定 | 与现有 BT 树一致;standoff 模式留后续 |
| D11 | 新代码放 `lgw_bt_planner` | 复用 lookat 数学/标定/测试;避免跨包导出 |
| D12 | tracker 只增字段、不改 `position` 语义 | 零破坏;odom 归属与臂基座几何分离 |
| D13 | 多目标候选池 / 自动切换推迟到 v2 | v1 先跑通主链路;方案与前置改造见 §8.5 |
| D14 | Go2 机身碰撞不纳入 v1 | 源脚本不在活动源码树;靠关节限位 + 工作空间钳制兜底 |
| D15 | 最终单栈 + 开发期两栈 | 最终一次 bringup;开发期两栈仅用于隔离验证 |
| D13 | 多目标候选池 / 自动切换推迟到 v2 | v1 先跑通主链路;方案与前置改造见 §8.5 |
| D14 | Go2 机身碰撞不纳入 v1 | 源脚本不在活动源码树;靠关节限位 + 工作空间钳制兜底 |
| D15 | 新 launch 与旧栈"并行但互斥" | 回归面最小;BT 阶段再统一 ownership |
| D16 | 2026-10-27 将 `agx_arm_ros` 升级到 `b9ad14d`,保留 `frame_prefix` 本地补丁 | 获取 CPV 支持;不升到 HEAD,避免 diagnostics breaking change |
| D17 | 驱动模式进程级固定;单栈选一个,对 MoveIt/Servo 同时生效 | `fast_mode` 无动态回调;先用 Task 7A 评估 `move_j` / `move_js` |
| D18 | 控制律参考点 = `tcp_link`(SRDF `arm` 组 tip) | 雅可比与 IK 都在组 tip 求值;`compute_tcp_pose` 输出即 tcp 位姿。原稿按 `link6` 计算会凭空引入 12cm 偏置,已更正 |
| D19 | 当前 tcp 位姿走 TF(`arm1/base_link ← arm1/tcp_link`),不用 `/feedback/tcp_pose` | 台架(`follow:=false`)与真机(`follow:=true`)同源;驱动该话题无 `frame_id`、时间戳来自 SDK,不适合做闭环与时序判据 |
| D20 | "MoveGroup 忙" = `/move_action` status 有 `ACCEPTED`/`EXECUTING` **或** FJT status 有活动 goal | 只看 FJT 会漏掉 MoveGroup 的规划阶段(数秒、JTC 上尚无 goal) |
| D21 | `control_mode` / CPV 不进 v1 | 用户已定"先不用 CPV";Task 7A 用数据判定,需要时在 `servo-cpv` 分支追加,不在 `src/` 里 fork |
| D22 | 目标选择抽成纯模块 `select_target()` | 它有分支逻辑(自动/指定/类别过滤/reacquire),和其它纯逻辑同一标准;不留在 ROS 壳里只靠集成测 |
| D23 | tracker 的臂基座几何**只走 TF**,不再用标定矩阵手算 | 安装桥已把同一份标定发布成 TF,实测等价(5.6e-16);避免双实现漂移,且 head/arm 同构。代价:桥发布前几何不可用(与"相机无数据"同表现) |
| D24 | 接受"启动栈时臂必须在 home",由 `start_lgw_full.sh` 启动前检查 | JTC 启动即保持 `initial_positions.yaml`(全零),臂不在 home 会被拉回 home 且速度不受 Servo 限速约束。改为运行时生成 initial_positions 需改上游 demo.launch,收益不抵风险 |
| D25 | 限速只用 `v_max`/`w_max` 或 `override_velocity_scaling_factor` | `scale.linear`/`scale.rotational` 在 `command_in_type:=speed_units` 下**完全不生效**(servo_calcs.cpp:1006-1016),曾导致真机以 0.6 rad/s 意外运动 |

---

## 12.5 2026-10-27 实测记录:仓库升级与 CPV 验证

### 12.5.1 升级结果

| 项 | 值 |
|---|---|
| 仓库 | `agx_arm_ws/src/agx_arm_ros` |
| 新分支 | `servo-cpv` |
| 基点 | `b9ad14d`(CPV 支持 + JointState 末端联动 + 反馈恢复) |
| 本地补丁 | `4d7b2b4`(原未提交的 `frame_prefix` 改动) |
| 回滚点 | `cfd71ecd`(原 `ros2`)/ `local-patches` 分支 |
| 构建 | `agx_arm_msgs` / `agx_arm_ctrl` / `agx_arm_moveit` 全部成功 |

**未改变**、且 `lg_walker_ws` 依赖的接口:

- `agx_arm_moveit` 的 `demo.launch.py`、`_moveit_config_builder.py`、`move_group.launch.py`、
  `spawn_controllers.launch.py`;
- `agx_arm_moveit/config` 的 `agx_arm.srdf.xacro`、`agx_arm.urdf.xacro`、`joint_limits.yaml`;
- `start_single_agx_arm_moveit.launch.py`;
- `agx_arm_msgs/msg` 的 6 个消息(含 `AgxArmStatus` / `GripperStatus` / `HandStatus`)。

`start_single_agx_arm.launch.py` 仅新增一行 `control/move_cpv` remap。
因此 BT 的 SRDF named target、`MoveArmToPose`、`/feedback/joint_states`、`/feedback/tcp_pose`
不受影响。唯一需要回归的是 `/control/joint_states` 的末端联动(夹爪/手)与 `fw_version` 解析。

### 12.5.2 CPV 只读探针

```
firmware = S-V1.8-8  (H-V1.2-1, node_type ARM_MC)
joint1..6 get_cpv_cv = 0.0   (6/6 有数值响应)
max_joint_spd = 3.0 rad/s    (全部关节)
```

结论:Piper v188 **支持 CPV**;轮廓速度初始未设置(`0.0`)。

探针脚本:`agx_arm_ws/scripts/cpv_probe.py`(只读,无运动指令)。

### 12.5.3 CPV 运动测试

参数:关节 6、`+0.05 rad`、`cv=0.2 rad/s`、`acc=dcc=0.5 rad/s²`。

| 项 | 结果 |
|---|---|
| `set_cpv_cv/acc/dcc` | 全部 `True`(ACK + 回读校验) |
| 去程 | 0.5010 → 0.5516 rad,约 0.45s,**单调、无过冲** |
| 回程 | 0.5516 → 0.5014 rad,同样平滑 |
| 终点误差 | 0.00037 rad(≈0.02°,编码器分辨率内) |
| 其他关节 | 未动 |

结论:CPV 的位置流 + 轮廓速度/加减速限制在真机上可用,运动平滑,适合伺服。

探针脚本:`agx_arm_ws/scripts/cpv_motion_probe.py`(运动测试,仅限工作区清空时运行)。

### 12.5.4 对设计的影响(已定:本期不用 CPV)

- CPV 已从"未验证的能力"变为"已验证可用"(`§12.5.3`);
- **本期仍不引入 CPV**(D21):先用 Task 7A 在 `move_j` / `move_js` 之间选;
  只有两者都不满足伺服需求时,才把 CPV 作为后续任务加进来;
- 若将来采用 CPV,启动时必须一次性为 6 个关节配置 `set_cpv_acc/dcc/cv`;
  **不要依赖上游默认**(`_ensure_cpv_joint_velocity` 会把未设置的轮廓速度设为
  `max_joint_spd = 3.0 rad/s`);
- 建议安全值:`cv = 0.5 rad/s`、`acc = dcc = 2.0 rad/s²`(待台架调参);
- 将来需要驱动侧 `control_mode` 参数(`j` / `js` / `cpv`),把 `/control/joint_states`
  路由到对应接口;该参数尚未实现,补丁落在 `servo-cpv` 分支(不在 `src/` 里 fork)。

**副作用**:运动测试把关节 6 的 `cpv_cv` 写成了 0.2 rad/s(存 Flash);其余关节仍为 `0.0`。
不影响 `move_j` / `move_js`,因为这两个接口不使用轮廓速度。

### 12.5.5 驱动模式评估计划(待执行)

在引入 CPV 之前,先用数据判断 `move_j` / `move_js` 是否已经够用。

**约束**:`fast_mode` 在驱动 `__init__` 里只读一次,没有动态参数回调;
运行时 `ros2 param set` 不生效,**必须重启驱动进程**才能切换。`speed_percent` 同理。

**测试矩阵**(同一套保守 Servo 参数):

| 运行 | `fast_mode` | 驱动接口 |
|---|---|---|
| A | `false` | `move_j` |
| B | `true` | `move_js` |

保守参数:`servo_velocity_scaling=0.2`(→ `override_velocity_scaling_factor`,
按 SRDF 5.0 rad/s 封顶)、`joint_limit_margin=0.1`、
`check_collisions=true`、`collision_check_rate=10.0`、`incoming_command_timeout=0.3`。

> **不要用 `scale.linear` / `scale.rotational` 限速** —— `servo_calcs.cpp:999-1016` 里
> 它们只在 `command_in_type: unitless` 时参与运算;本项目用 `speed_units`,twist 被原样执行。
> 真正可用的限速是:节点自身的 `v_max` / `w_max`,或 `override_velocity_scaling_factor`。
> (2026-10-28 真机实测确认,见 §12.6)

**输入**:恒定 twist 1s(如 `linear.x=0.02 m/s` 或 `angular.z=0.1 rad/s`),然后零 twist。

**指标**:上升时间(到 90%)、过冲、稳态 RMS、最大关节速度、符号变化次数。

**判定**:

| 结果 | 决策 |
|---|---|
| A 收敛 ≤2s、过冲 <10%、稳态 RMS <0.5° | 用 `move_j`,**不引入 CPV** |
| A 不满足、B 满足且无机械冲击 | 用 `move_js`,不引入 CPV |
| 两者都不满足 | 再评估 CPV(需要 `control_mode` 补丁) |

执行步骤见 plan 的 Task 7A。

---

## 12.6 2026-10-28 真机验证记录(部分)

### 已通过

| 项 | 结果 |
|---|---|
| 驱动启动 | `S-V1.8-8 → v188`,夹爪初始化 OK,控制使能 |
| 单栈 bringup | rsp / move_group / JTC / Servo / `lgw_servo_node` 全部起来 |
| `start_servo` | 6.7 s 返回;收到 Servo 真实输出后主动 pause |
| takeover(指定 track_id) | `success=true, state=TRACKING` |
| release | `success=true`,臂位姿前后一致 |
| Servo → JTC 通路 | `/arm_controller/joint_trajectory` 100 Hz,positions 与实测一致 |

### 发现的问题

**P1(已修)`:scale.linear` / `scale.rotational` 在 `speed_units` 下不生效。**
`servo_calcs.cpp:999-1016` 只在 `unitless` 分支乘 scale。本项目用 `speed_units`,
twist 被原样执行。曾误以为 `servo_linear_scale:=0.0` 能让臂不动,实测臂以 0.6 rad/s 运动 0.35 rad
并触发奇异急停。改用 `override_velocity_scaling_factor`(两模式均生效)或节点侧 `v_max/w_max`。

**P2:home(全零)是奇异构型。** 条件数 29 341(阈值 halt>30 / decel>17),Servo 正确拒动。
纯 +z 上抬 25 cm 只能降到 149;脱离需 q2/q3 弯曲 + q5 偏离(q2=1.2, q3=−1.2 → q5≥0.56)。
因此**接管前必须先把臂摆离 home**;BT 流程里本就先 `MoveArmToNamed`/`MoveArmToPose` 再接管。

**P3:auto 模式需要 `odom` 帧。** 该帧由 tracker 的 `OdomTFBroadcaster` 发布;
未起 tracker 时 `takeover(track_id=0)` 返回 `no_base_pose`(按设计不自作主张降级排序)。

### 尚未验证

- 闭环收敛(需在非奇异位姿、用真实限速重跑)
- `/move_action` 与 FJT 双 status 的 busy 拒绝
- 行走场景

---

## 13. 文件清单

### 新增

| 文件 | 职责 |
|---|---|
| `src/lgw_bt_planner/srv/Takeover.srv` | 接管服务定义 |
| `src/lgw_bt_planner/msg/ServoState.msg` | 状态消息 |
| `src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp` | 目标选择 + reacquire(纯逻辑) |
| `src/lgw_bt_planner/src/target_selector.cpp` | 同上实现 |
| `src/lgw_bt_planner/include/lgw_bt_planner/geometry_source.hpp` | 三档选择 + odom 反算(纯逻辑) |
| `src/lgw_bt_planner/src/geometry_source.cpp` | 同上实现 |
| `src/lgw_bt_planner/include/lgw_bt_planner/servo_control_law.hpp` | P 环/死区/低通(纯逻辑) |
| `src/lgw_bt_planner/src/servo_control_law.cpp` | 同上实现 |
| `src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp` | 状态迁移(纯逻辑) |
| `src/lgw_bt_planner/src/servo_state_machine.cpp` | 同上实现 |
| `src/lgw_bt_planner/src/servo_node.cpp` | ROS 接口层 |
| `src/lgw_bt_planner/launch/lgw_full.launch.py` | 单栈 bringup:`demo.launch.py` + servo + `lgw_servo_node` + bt_runner |
| `src/lgw_bt_planner/config/servo_params.yaml` | moveit_servo 参数 |
| `src/lgw_bt_planner/config/servo_node.yaml` | lgw_servo_node 参数 |
| `src/lgw_bt_planner/tests/test_target_selector.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_geometry_source.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_servo_control_law.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_servo_state_machine.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_lookat_setpoint.cpp` | 单测 |
| `scripts/start_lgw_full.sh` | 单栈 bringup(CAN + driver + `lgw_full.launch.py`) |
| `scripts/driver_mode_eval.py` | `move_j` vs `move_js` 评估脚本 |
| `scripts/fake_track_publisher.py` | 台架静态 track(15Hz),用于 A1/A2 与闭环收敛用例 |
| `src/lgw_3d_tracker/...`(改) | 见下 |

### 修改

| 文件 | 改动 |
|---|---|
| `src/lgw_bt_planner/CMakeLists.txt` | 加 srv/msg、新源文件、新 gtest、install config |
| `src/lgw_bt_planner/package.xml` | 加 `moveit_servo`(exec)、`std_srvs`、`action_msgs`、`tf2` |
| `src/yoloe_interfaces/msg/UnifiedTrack.msg` | 新增臂基座几何与贡献者字段 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py` | 每条 detection 算 `position_arm_base` |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py` | `TrackObservation` 增字段 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py` | 输出新字段与贡献者数组 |

---

## 14. 参考

| 主题 | 位置 |
|---|---|
| 调研记录(证据/行号/推导) | `docs/servo/2026-10-26-servo-takeover-design.md` |
| 控制链路与 mock 硬件 | `agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.ros2_control.xacro` |
| controller YAML 生成 | `agx_arm_moveit/launch/demo.launch.py` |
| control gate | `agx_arm_moveit/scripts/agx_arm_control_gate` |
| 驱动订阅与转发 | `agx_arm_ctrl/agx_arm_ctrl_single_node.py` |
| TF 前缀与桥 | `docs/tf-frame-prefix.md` |
| lookat 数学 | `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp` |
| 标定 | `src/lgw_bt_planner/calibration/{arm,head}_calibration.json` |
| servo 参数校验 | `/home/unitree/ros2_ws/src/moveit2/moveit_ros/moveit_servo/src/servo_parameters.cpp` |
| servo 示例参数 | `.../moveit_servo/config/panda_simulated_config.yaml` |
| tracker 输入/输出 | `src/lgw_3d_tracker/.../input_adapter.py`、`output_formatter.py` |
