# Servo 接管 MoveIt 规划器 + 视觉伺服:调研与设计

- 日期:2026-10-26
- 状态:调研完成,设计待评审(未写代码)
- 涉及工作区:`l4t/agx_arm_ws`(上游 MoveIt/驱动)、`l4t/lg_walker_ws`(自写包)
- 目标读者:后续实施者

---

## 0. TL;DR

1. 当前机械臂的唯一驱动通路是 **`move_group → ros2_control(mock 硬件) → JointStateBroadcaster → control/joint_states → agx_arm_ctrl → CAN`**。
   `JointStateBroadcaster` 以 200Hz 持续发布,**任何绕过它的直接 publish 都会在 5ms 内被覆写**。
2. 因此 Servo 接入不能靠"也往 `control/joint_states` 发",必须**换掉写者**。
3. 采用 MoveIt 官方路径:**`arm_controller`(JointTrajectoryController)↔ `servo_position_controller`(JointGroupPositionController)由 `controller_manager` 互斥切换**。
   controller_manager 本身就是单写者仲裁器,不需要自写 arbiter。
4. `moveit_servo` 输出 `std_msgs/Float64MultiArray`(位置),直接写进 `servo_position_controller`;
   它自己负责 IK、雅可比、碰撞检查、奇异点降速、平滑、关节限位。
5. 自写部分收敛为一个**状态机节点** `lgw_servo_controller`:BT 服务接口 + lookat 数学 + `TwistStamped` 输出。
6. 视觉伺服的几何**可以做到零未知量**:剥掉 `T_base_odom` 与 `T_armbase_base` 两个外因子后,
   两个相机到臂基座都是纯已知链;`/feedback/tcp_pose` 本身就在臂基座系。
7. 但当前 3D tracker **只输出 odom 系融合位置**,把上游几何、时间戳、每相机贡献都折叠掉了。
   需要按 §7 改造后才能喂给伺服。

---

## 1. 结论与范围

### 1.1 结论

- **可行**。Servo 可以与 MoveGroup 共存于同一套 ros2_control,但同一时刻只有一个 controller 在写关节命令。
- **不需要**修改上游 `agx_arm_ros` / `agx_arm_ctrl`。
- **需要**新增:一个 controller 配置、一个 servo 参数文件、一个组合 launch、一个状态机节点。
- **需要**改造 3D tracker 的输出契约(见 §7)。
- **不需要**精确的 Go2↔机械臂挂载偏移(`arm_mount_offset`)来跑通伺服;它只影响 tracker 的 odom 派生输出。

### 1.2 目标

**任务场景:** Go2 行走中发现目标,在靠近目标的整个过程中,**保证目标始终位于臂端相机的观测视角内**。
伺服不是"把 tcp 送到目标点",而是"持续调整臂的指向,让臂端相机盯着目标"。

- 服务端节点接收服务请求后接管机械臂,跟随 3D tracker 的目标轨迹。
- 退出时把控制权归还 MoveGroup,不产生跳变。
- 伺服过程中保留碰撞检查与关节限位。
- 允许 Go2 行走中伺服(行走作为基座扰动处理,本期不联动转体)。
- 本期**不实现与 BT 的交互**,只实现服务端节点本身。

### 1.3 非目标(本期)

- 不做抓取阶段的力控 / 阻抗控制。
- 不改 `agx_arm_ctrl` 的命令协议。
- **不做 Go2 转体与机械臂伺服的联动** —— 行走本身允许(作为基座扰动),但不自动转身。
- 不标定 `arm_mount_offset`(遗留项,见 §10)。

### 1.4 关键假设

| 假设 | 影响 |
|---|---|
| **目标在物理空间近似静止**(三维位置不大幅改变) | 3D tracker 的 3 维位置 KF(无速度分量)是合理建模;odom 兜底可用到 track 过期 |
| Go2 与机械臂基座的挂载偏移未标定 | 伺服闭环不使用它;只影响 odom 派生输出 |
| 本期不与 BT 对接 | 只实现服务端节点;接口设计需兼容将来"作为 BT 技能"与"被 BT 技能调用"两种形态 |

> 静态目标假设是 §6.3 三档策略与 §6.4 odom 兜底的前提。
> 若将来目标会移动(例如跟踪行人),必须重新评估:KF 需加速度分量,odom 兜底窗口需缩短到 ≤500ms。

---

## 2. 现状:机械臂控制链路

### 2.1 完整链路(已核实)

```
move_group
  │  FollowJointTrajectory action
  ▼
ros2_control_node / arm_controller (JointTrajectoryController)
  │  写 position command interface
  ▼
mock_components/GenericSystem        ← 只回显,不驱动硬件
  │  state interface = 最近一次 command
  ▼
joint_state_broadcaster
  │  /joint_states  (200Hz, update_rate=200)
  │  demo.launch.py:205  remap → control_topic
  ▼
/control/joint_states
  │
  ▼
agx_arm_ctrl_single_node._joint_states_callback()   ← python 节点
  ├─ _control_arm_joints()      → agx_arm.move_j()  / move_js() (fast_mode)
  ├─ _control_gripper_joint()   → gripper.move()
  └─ _control_hand_joints()     → hand.position_ctrl()
  ▼
piper CAN
```

关键代码位置:

| 事实 | 位置 |
|---|---|
| mock 硬件插件 | `agx_arm_moveit/config/agx_arm.ros2_control.xacro:8` |
| ros2_control_node 的 `joint_states → control_topic` remap | `agx_arm_moveit/launch/demo.launch.py:205` |
| controller 类型与 `command_interfaces: [position]` | `demo.launch.py:_build_ros2_controllers_file()`(约 29-99 行) |
| `_joint_states_callback` 一次驱动臂+夹爪+手 | `agx_arm_ctrl/agx_arm_ctrl_single_node.py:737-751` |
| `_control_gripper_joint` 每帧无条件调用 `gripper.move()` | 同文件 `697-710` |
| `/feedback/joint_states` 与 `/feedback/tcp_pose` 200Hz | 同文件 `pub_rate` 默认 200,`start_single_agx_arm.launch.py:77-81` |

### 2.2 为什么这是关键约束

`JointStateBroadcaster` 是**无条件、200Hz** 发布的。它发布的值是 mock 硬件 state interface 的镜像,而 mock 的 state 来自最近一次命令。因此在没有轨迹执行时,它仍然以 200Hz 发**上一个轨迹的末点**。

后果:

- 任何节点直接 publish `control/joint_states` 想控制关节,都会在 ≤5ms 内被下一个 echo 帧覆写。
- 该约束**不只针对夹爪**,对 arm joint 同样成立。

### 2.3 夹爪的历史问题(已验证现象)

曾尝试由 BT 节点发布 `control/joint_states` 控制夹爪,MoveIt 关闭时生效,MoveIt 开启时无响应。
原因是 `_control_gripper_joint()` 每帧无条件执行 `gripper.move()`,MoveIt 侧的夹爪轨迹经 echo 链路持续占据该关节。后改为通过 MoveIt 的 `gripper` planning group 控制(`gripper_open/half/close` named states),这是正确的做法。

参考:`agx_arm_ros/test/gripper/test_gripper_joint_states.yaml` 就是 `_joint_states_callback` 的入参格式
(`name: [gripper]`, `position: [0.0]`, `effort: [2.0]`)。

### 2.4 control gate 的实际状态(已核实)

`agx_arm_control_gate` 只在 `auto_control_gate:=true` 时启动(`demo.launch.py:156-171`)。

`start_lgw_arm.sh` 的 launch 调用**没有传** `auto_control_gate`,因此:

| 项 | 结果 |
|---|---|
| `auto_control_gate` | `false`(默认) |
| `agx_arm_control_gate` 节点 | **不启动** |
| `control_enabled` 传给驱动 | `true`(`start_single_agx_arm_moveit.launch.py:122-124` 的 `IfElseSubstitution`) |
| 历史日志 | 反复出现 `control_enabled: True` |

即:**gate 常开,没有门卫**。这既意味着 Servo 不会被 gate 挡住,也意味着没有任何互斥保护——所以互斥必须由 controller 层提供。

### 2.5 坐标系现状(已核实,修正早期判断)

`start_lgw_arm.sh:166` **已传** `frame_prefix:='arm1/'`。

实际 TF 链路(见 `docs/tf-frame-prefix.md`):

```
odom ──▶ base_link ──[挂载桥]──▶ arm1/world ──▶ arm1/base_link ──▶ … ──▶ arm1/tcp_link
```

- 每个帧恰好一个 parent,`base_link` 不再被抢。
- `camera_head_link` 的 parent 是 **`arm1/base_link`**(head 标定以机械臂基座为基准)。
- `camera_arm_link` 挂在 `arm1/tcp_link`。

**两条命名规则**(`docs/tf-frame-prefix.md` §7):

| 场景 | 用哪个名字 |
|---|---|
| 和 TF 说话(tracker / 桥 / rviz) | `arm1/base_link`、`arm1/tcp_link` |
| 和 MoveIt 说话(model / SRDF / Servo) | 裸名 `base_link`、`tcp_link` |

`start_lgw_arm.sh:164` 的 `tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'` 是**夹爪 TCP 偏置**,
driver 与 MoveIt 两侧都传了,与 `arm_calibration.json` 中"已叠加 12cm"的说法一致。

`lgw_3d_tracker/config/runtime.yaml:55` 的 `arm_mount_offset` 仍是单位阵(Go2 `base_link` 与臂基座假设重合),
这是**独立的一项**,与 `tcp_offset` 无关。

---

## 3. 接入方式的取舍

### 3.1 为什么不用"自写闭环"

曾评估两条自写路线:

| 路线 | 问题 |
|---|---|
| 自写 PBVS → `control/move_c` | `move_c` 是 SDK 的三段笛卡尔运动,语义与实时伺服不匹配 |
| 自写 PBVS → `control/move_js`(MIT 模式) | 与 `_joint_states_callback` 的 `move_j` 同时写同一 SDK,`is_mit_mode` 来回翻转,必然互相打架 |

自写还必须自己实现:IK、雅可比、碰撞检查、关节限位、奇异点处理、平滑。
MoveIt Servo 已经提供这些,且是官方维护路径。

### 3.2 为什么用 controller 切换而不是自写 arbiter

早期方案是写一个 `command_arbiter` 单写者节点,接在 `moveit echo` 与 `servo` 之间。
改用 controller 切换的理由:

| 维度 | 自写 arbiter | controller 切换 |
|---|---|---|
| 单写者保证 | 自己实现 | controller_manager 强制 |
| 释放跳变 | 需自写 FROZEN 态与重入逻辑 | 由 JTC/controller 生命周期处理(待实测) |
| 新增代码量 | arbiter + 桥 | 0(仅 launch + 一个状态机节点) |
| 夹爪 | 需处理非 arm 关节透传 | `gripper_controller` 全程 active,天然不受影响 |
| 上游依赖 | 仅 remap | 需要能加载新 controller(见 §10) |

**结论:采用 controller 切换。** `controller_manager` 本身就是本项目缺失的那个"门卫"。

---

## 4. 设计:四层切分

```
┌──────────────────────────────────────────────────────────────┐
│ L3  lgw_servo_controller   (新增,状态机 + lookat + twist)     │
│     服务:Takeover / Release / SetTracking                    │
│     话题:/lgw/servo/state                                    │
└───────────────┬──────────────────────────────────────────────┘
                │ TwistStamped (frame_id = base_link)
                ▼
┌──────────────────────────────────────────────────────────────┐
│ L2  moveit_servo (servo_node)                                │
│     IK / 雅可比 / 碰撞 / 奇异点 / 限位 / 平滑                  │
│     输出 Float64MultiArray(位置)                              │
└───────────────┬──────────────────────────────────────────────┘
                │ /servo_position_controller/commands
                ▼
┌──────────────────────────────────────────────────────────────┐
│ L1  controller_manager                                       │
│     arm_controller ↔ servo_position_controller 互斥          │
│     同一时刻只有一个写同一批 position command interface       │
└───────────────┬──────────────────────────────────────────────┘
                │ mock 硬件 → JointStateBroadcaster → control/joint_states
                ▼
           agx_arm_ctrl → CAN

L0  TF / 坐标系:arm1/ 前缀已启用;arm_mount_offset 仅影响 odom 派生
```

### L0 —— 前置条件:坐标系

**已满足:**

- `arm1/` 前缀已启用,TF 每帧唯一 parent。
- 头相机、臂相机到臂基座的变换链完全已知(静态标定 + 臂内 FK)。

**仍遗留:**

- `arm_mount_offset` 未实测。**它不参与伺服闭环**,只影响 tracker 的 odom 派生输出。

**Servo 侧的命名约定:**

```yaml
move_group_name: arm
planning_frame: base_link            # 模型名,不能写 arm1/base_link
robot_link_command_frame: base_link  # 模型名
ee_frame_name: tcp_link              # 模型名
```

状态机发 `TwistStamped` 时 `frame_id` 写 `base_link`,与 `robot_link_command_frame` 相同,
让 Servo 走单位变换、不跨 TF 解析。**否则 Servo 会去 TF 里找 `base_link`(那是 Go2 躯干),
今天数值巧合相等,一旦 `arm_mount_offset` 实测填值就会整段偏。**

### L1 —— 控制权层:controller 切换

```
arm_controller             joint_trajectory_controller/JointTrajectoryController   active at startup
servo_position_controller  position_controllers/JointGroupPositionController        inactive at startup
```

- 两者都声明 `command_interfaces: [position]`,操作同一批 joint。
- `controller_manager` 不允许两个 active controller 同时占用同一 command interface。
- 接管:`/controller_manager/switch_controller`
  - `activate: [servo_position_controller]`
  - `deactivate: [arm_controller]`
  - `strict: true`
- 归还:反向。

**为什么需要自写组合 launch(或等价注入):**

- Servo 控制器的类型必须在 `controller_manager` 参数里声明。
- Humble 的 `LoadController.srv` 只有 `string name`,**没有 `type` 字段**。
- `demo.launch.py` 内部用 `_build_ros2_controllers_file()` **硬编码生成** controller YAML,没有扩展点。

可选做法(实施时择一,见 §10 待验证):

1. 自写组合 launch:不复用 `demo.launch.py`,自己 include `rsp.launch.py` + `move_group.launch.py`,
   并用自带的 controller YAML 启动 `ros2_control_node`。
2. 保留 `demo.launch.py`,启动后用 `spawner -t position_controllers/JointGroupPositionController --inactive`
   注入(容器内 `spawner --help` 显示支持 `-t`;但 LoadController.srv 无 type 字段,是否真生效需实测)。

### L2 —— moveit_servo

**版本与来源**

- 容器内未安装;`apt-cache search` 有 `ros-humble-moveit-servo`。
- host 上有源码副本:`/home/unitree/ros2_ws/src/moveit2/moveit_ros/moveit_servo`,版本 **2.5.9**。

**已验证的输出能力**(`servo_parameters.cpp`)

- `command_out_type` 只接受:`trajectory_msgs/JointTrajectory`、`std_msgs/Float64MultiArray`(第 405 行)。
- 用 `Float64MultiArray` 时,**`publish_joint_positions` 与 `publish_joint_velocities` 只能二选一**(第 422 行)。

**初版参数**

```yaml
move_group_name: arm
planning_frame: base_link
robot_link_command_frame: base_link
ee_frame_name: tcp_link

command_in_type: "speed_units"
command_out_type: std_msgs/Float64MultiArray
publish_joint_positions: true
publish_joint_velocities: false
publish_joint_accelerations: false
command_out_topic: /servo_position_controller/commands

joint_topic: /feedback/joint_states        # ★ 必须是真机反馈,不能用 mock echo
is_primary_planning_scene_monitor: false   # move_group 是 primary
monitored_planning_scene_topic: /monitored_planning_scene

incoming_command_timeout: 0.3              # 默认 0.1 对 ~10-15Hz 输入太紧
publish_period: 0.01                       # 100Hz 输出,输入仍是 10-15Hz
low_latency_mode: false
num_outgoing_halt_msgs_to_publish: 4

check_collisions: true
collision_check_rate: 10.0                 # Orin NX 上不要调高
self_collision_proximity_threshold: 0.02
scene_collision_proximity_threshold: 0.03
joint_limit_margin: 0.1
lower_singularity_threshold: 17.0
hard_stop_singularity_threshold: 30.0
leaving_singularity_threshold_multiplier: 2.0
smoothing_filter_plugin_name: "online_signal_smoothing::ButterworthFilterPlugin"
```

**★ `joint_topic` 是最容易埋雷的一条。**
若指到 `control/joint_states`(mock echo),Servo 以为机械臂已经到达它命令的位置,闭环退化成开环。
必须指 `/feedback/joint_states`(200Hz 真机反馈)。

### L3 —— 状态机节点 `lgw_servo_controller`

**职责边界(只做三件,不做 IK)**

1. 目标选择 + setpoint 计算:订阅 tracker,选 track,用 lookat 数学算目标 tcp 位姿。
2. 误差转换:与 `/feedback/tcp_pose` 作差 → 限幅 → `TwistStamped`。
3. 状态机 + 控制权切换。

**目标选择策略(本期)**

- 模式 A:调用方显式指定 `track_id`。
- 模式 B:服务端自主选择 —— 在 odom 系下取**距 `base_link` 原点最近**的 confirmed track。
- 选择只需要 odom 系,不需要挂载偏移(距离比较对刚体变换不敏感)。
- 若显式指定的 `track_id` 消失,默认转 HOLD 并上报(不自动改追别的目标)。

**lookat 位姿的取法**

- 姿态:相机 z 轴指向目标,由 `build_lookat_orientation` 决定(含 uprightness 翻转)。
- 位置:沿用现有 BT 树的 xoy-orbit —— `xoy_distance_m=0.4`、`z_fixed_m=0.3`
  (`head_arm_independent_lookat.xml:169-175`),两个值都做成参数。
- 退化保护:目标贴近基座 z 轴时回落到 `ee_position_world` anchor(已有实现)。
- 输出帧:`base_link`(模型名),与 Servo 的 `planning_frame` 一致。
  现有 BT 树写的是 `world_frame="world"`,在模型里 `world→base_link` 是 identity,
  数值等价;伺服侧统一用 `base_link` 更明确。

**lookat 位姿的取法**

- 姿态:相机 z 轴指向目标,由 `build_lookat_orientation` 决定(含 uprightness 翻转)。
- 位置:沿用现有 BT 树的 xoy-orbit —— `xoy_distance_m=0.4`、`z_fixed_m=0.3`
  (`head_arm_independent_lookat.xml:169-175`),两个值都做成参数。
- 退化保护:目标贴近基座 z 轴时回落到 `ee_position_world` anchor(已有实现)。
- 输出帧:`base_link`(模型名),与 Servo 的 `planning_frame` 一致。
  现有 BT 树写的是 `world_frame="world"`,在模型里 `world→base_link` 是 identity,
  数值等价;伺服侧统一用 `base_link` 更明确。

**目标选择策略(本期)**

- 模式 A:调用方显式指定 `track_id`。
- 模式 B:服务端自主选择 —— 在 odom 系下取**距 `base_link` 原点最近**的 confirmed track。
- 选择只需要 odom 系,不需要挂载偏移(距离比较对刚体变换不敏感)。
- 若显式指定的 `track_id` 消失,默认转 HOLD 并上报(不自动改追别的目标)。

**接口(本期定为服务 + state 话题)**

| 接口 | 类型 | 语义 |
|---|---|---|
| `/lgw/servo/takeover` | 自定义 srv:`uint64 track_id`(0=auto)+ `string class_filter` | IDLE → 切 controller → TRACKING;返回 success / message / state |
| `/lgw/servo/release` | `std_srvs/Trigger` | TRACKING/HOLD → 切回 controller → IDLE |
| `/lgw/servo/pause` | `std_srvs/Trigger` | 保持控制权,停止跟随(转 HOLD) |
| `/lgw/servo/resume` | `std_srvs/Trigger` | HOLD → TRACKING |
| `/lgw/servo/state` | 自定义 msg | 状态 + 误差 + 距离 + 标志位(见下) |
| 订阅 tracker | `yoloe_interfaces/UnifiedTrackArray` | 目标轨迹 |

`state` 消息字段(供外部/未来 BT 决策):

```msg
string state                 # IDLE / PREEMPT / TRACKING / HOLD / RELEASING / FAULT
string geometry_source       # head / arm / odom
uint64 track_id
float64 angular_error_rad
float64 position_error_m
float64 target_distance_m
bool converged               # e_ang < 2° 且 ‖e_pos‖ < 1cm
bool saturated               # 顶到关节限位 / 可达边界
bool too_close               # 相机距目标 < min_observe_distance_m
builtin_interfaces/Time stamp
```

> **为什么先不做 action**:接管是一段持续状态,不是会完成的目标;action 的 result 语义在这里别扭,
> 唯一优势(cancel)已被 `release` 服务覆盖。
>
> **将来接 BT 的两种形态都兼容**:
> ① 本项目保持独立服务端,BT 用 `BT::RosServiceNode` 调用;
> ② 把状态机搬进 BT 技能节点(作为 tick 驱动的动作节点),内部复用同一套数学与 controller 切换逻辑。
> 因此状态机逻辑应与 ROS 接口层分离,便于后者复用。

**内部数学**

复用 `lgw_bt_planner/src/compute_lookat_pose_node.cpp` 中的**自由函数**:

- `build_lookat_orientation(direction, world_up)` — 含 direction∥world_up 退化处理、uprightness 翻转
- `compute_tcp_pose(P_ee_in_base, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp)`

不要重写:这两个坑(退化、相机朝上)已经处理过。

**误差与控制律**

```
e_pos  = p_target_armbase - p_tcp_armbase
e_rot  = R_target_armbase · R_tcp_armbase^T  → 轴角
v      = clamp(Kp_pos · e_pos, v_max)
ω      = clamp(Kp_rot · e_rot, ω_max)
```

输入频率 ~10-15Hz,输出给 Servo 也是这个频率;Servo 以 `publish_period` 保持并积分。

**HOLD 判据(与 tracker 语义对齐)**

| 条件 | 动作 |
|---|---|
| `track.status == coasting` | 发零 twist(保持),不跟随外推 |
| `time_since_last_observation_ms > 300` | HOLD |
| `> 1500ms`(tracker `coast_max_age_ms`) | HOLD 并上报 FAULT |
| 目标丢失 / track 删除 | HOLD |

注意:当前 KF 是纯 3 维位置、无速度,`coasting` 时的预测值实际上停在最后位置,
所以"HOLD"与"跟随 coasting 值"数值上等价;但仍应显式发零 twist,避免未来 KF 升级后行为突变。

---

## 5. 状态机

```
          Takeover                controller 切换完成
IDLE ───────────────▶ PREEMPT ──────────────────────▶ TRACKING
 ▲                       │                              │  ▲
 │                       │ 切换失败                      │  │ 目标恢复
 │                       ▼                              │  │
 │                     FAULT ◀────────── 超时/丢失 ──── HOLD
 │                       │                              │
 │                       │ Release                      │ Release
 └───────────────────────┴──────────────────────────────┘
                           │
                           ▼
                      RELEASING ──▶ IDLE
```

要点:

- `PREEMPT` 与 `RELEASING` 都涉及 controller 切换,必须是**同步等待成功**再迁移状态。
- 任何异常路径都必须保证:要么 controller 已经切回 `arm_controller`,要么明确停在 FAULT 并报警。
- `TRACKING → HOLD` 只停指令,不释放控制权(避免频繁切换 controller)。

---

## 6. 视觉伺服的几何

### 6.1 精确性推导

当前 tracker 的 `input_adapter._cam_to_odom()`:

```
head:  p_odom = T_base_odom · T_armbase_base · T_cam_armbase · p_cam
arm :  p_odom = T_base_odom · T_tcp_base    · T_cam_tcp    · p_cam
```

其中:

- `T_base_odom`:SLAM `odom → base_link`
- `T_armbase_base`:机械臂挂载偏移(`arm_mount_offset`,**当前假设单位阵,未实测**)
- `T_cam_armbase`:head 标定(camera → **机械臂基座**)
- `T_cam_tcp`:arm 标定(camera → tcp_link,已叠 12cm `tcp_offset`)

`T_base_odom` 与 `T_armbase_base` **只出现在最左边两个因子**。刚体变换左乘,因此直接剥掉即可得到臂基座坐标:

```
head → 臂基座:  p_armbase = T_cam_armbase · p_cam
                （纯静态标定,不需要任何 TF lookup）

arm  → 臂基座:  p_armbase = T_tcp_armbase · T_cam_tcp · p_cam
                T_tcp_armbase = lookup("arm1/base_link", "arm1/tcp_link")
                （臂内链,不经过 Go2 base_link）
```

**伺服闭环里一个未知量都不需要。**

同时 `/feedback/tcp_pose` 是 SDK FK 输出,基准就是臂基座,
所以目标点与当前位姿在同一个系里,**连 TF 都不用查**。

### 6.2 "几何源"的定义(已确认)

**几何源 = 把目标位置换算到机械臂基座系(`arm1/base_link`)时,采用哪一路观测作为输入。**

伺服的控制量是"臂端相机指向目标的位姿误差",误差必须在统一坐标系里计算,
所以每个控制周期都要先决定用哪一路源。候选三路:

| 优先级 | 源 | 转换链 | 性质 |
|---|---|---|---|
| 1 | head 相机观测 | `T_cam_armbase`(纯静态标定) | 臂动时目标在臂基座系**不动**;无自激 |
| 2 | arm 相机观测 | `T_tcp_armbase(观测时刻) · T_cam_tcp` | eye-in-hand;臂动即观测变,有自激风险 |
| 3 | odom KF 状态 | `T_armbase_base · T_base_odom(t)⁻¹ · p_odom` | 只剩一个常量未知;语义是"最后已知位置" |

> **注意:"能力"与"稳定性"是两个维度。** arm 相机在"能指向哪里"上更强更灵活,
> 但它同时是执行器与传感器,回路里含"臂动 → 观测变"的正反馈;
> head 相机固定于臂基座,目标在臂基座系静止,回路是纯扰动抑制。
> 10-15Hz 输入下 head 的相位余量明显更大,所以 head 优先。

### 6.3 三档切换策略(已确认)

场景:Go2 行走中发现目标,靠近过程中保证目标始终在观测视角内。

```
每次控制 tick:
  head 贡献者有效且新鲜(age < 200ms)      → 用 head 的 position_arm_base
  else arm 贡献者有效且新鲜(age < 200ms)   → 用 arm 的 position_arm_base
  else track 存在且未过期(< 1500ms)        → 用 odom KF 反算到臂基座
  else                                    → HOLD
```

- **切换滞回**:高优先级源需连续 N 个 tick(约 200-300ms)有效才切回,避免抖动。
- **降级立即生效**(安全方向不延迟):head 一失效就切 arm,arm 一失效就切 odom。
- **低优先级降增益**:arm 档 0.7×,odom 档 0.5×。理由分别是自激风险与"最后已知位置"语义。
- **静态目标假设下**:odom 兜底可用到 track 过期(默认 `coast_max_age_ms = 1500ms`);
  若将来目标会移动,必须把窗口缩短到 ≤500ms(见 §1.4)。
- **静态目标假设下**:odom 兜底可用到 track 过期(默认 `coast_max_age_ms = 1500ms`);
  若将来目标会移动,必须把窗口缩短到 ≤500ms(见 §1.4)。

### 6.4 odom 兜底的理论验证(已确认可行)

由 §6.1 的链式关系:

```
p_odom = T_base_odom(t) · T_armbase_base · T_cam_armbase · p_cam
```

因此

```
p_armbase = T_armbase_base · T_base_odom(t)⁻¹ · p_odom
```

**结论:可以逆推。** 误差只来自一个**常量刚体变换** `T_armbase_base`(挂载偏移),
不是随时间变化的随机误差。当前假设为单位阵,即

```
p_armbase ≈ T_base_odom(t)⁻¹ · p_odom
```

成立需要满足三个条件:

1. **时间对齐**
   - 对原始观测:用**该观测的 stamp** 查 `T_base_odom`。
   - 对 KF 状态:用 **now** 查 —— 要问的是"目标此刻相对机器人身体在哪里",
     而 KF 状态已经是目标在世界系里的位置。
   - Go2 行走时用错时刻会引入 `速度 × 时间差` 的误差(1 m/s × 50ms = 5cm)。
2. **`T_base_odom` 可用**:tracker 已有 30s TF 缓存;servo 节点也可自建 listener。
3. **理解它的语义**:KF 是 3 维位置、无速度,`coasting` 时状态冻结。
   所以 odom 兜底给的是"目标**最后已知**的世界位置",不是当前位置。
   目标在移动时该值必然滞后。

**误差性质**

| `T_armbase_base` 误差 | 后果 |
|---|---|
| 纯平移 | 目标位置整体偏移;角度误差约 `Δ/d`,随距离衰减 |
| 旋转 | **指向角系统性旋转**,与距离无关 —— 更危险 |

因此:兜底适合"大致指向最后已知位置",不适合作为高精度伺服的几何源。
标定 `arm_mount_offset` 之后,这一档会变成精确源。

### 6.5 判据与控制律(已确认)

**判据 = 臂端相机光轴与目标方向的夹角。**

等价说法:用 `ComputeLookatPose` 的数学算出目标位姿,伺服把 tcp 驱到该位姿;
夹角就是"目标位姿"与"实际 tcp 位姿"之间的旋转误差。

**lookat 位姿不是唯一的 —— 有一个"距离自由度"**

`build_lookat_orientation` 只决定姿态(相机 z 轴指向目标),不决定 tcp 沿视线放在哪里。
现有 BT 树的取法(`head_arm_independent_lookat.xml:169-175`):

| 参数 | 值 | 含义 |
|---|---|---|
| `xoy_distance_m` | `0.4` | tcp 落在半径 0.4m 的圆周上,方位取目标在 xy 平面的方向 |
| `z_fixed_m` | `0.3` | tcp 高度固定 |
| `ee_position_world` | `0.4;0.0;0.2` | `r_xy≈0` 时的回落 anchor |

伺服沿用这组默认值,并做成参数(行走时可能需要调整,避免臂伸得过直)。

**控制律**

```
e_pos = p_des - p_tcp                       (均在臂基座系)
e_ang = angle(R_des · R_tcp^T)              (轴角,rad)

v = clamp(Kp_pos · e_pos, v_max)            # 默认 v_max ≈ 0.15 m/s
ω = clamp(Kp_ang · e_ang, ω_max)            # 默认 ω_max ≈ 0.6 rad/s
```

- **饱和**:远距离/大角度时直接顶到 `v_max / ω_max`,实现"快速运动到目标位姿"。
- **死区(已定:严格对准)**:`e_ang < 2°` 且 `‖e_pos‖ < 1cm` 时输出零 twist。
  零 twist 仍按控制频率持续发布,避免 Servo 的 `incoming_command_timeout` 触发 halt。
  为抑制行走时围绕零误差的高频抖动,误差先过一阶低通(截止约 5-10Hz)并做速率限幅。
- **不可达检测**:若 `e_ang` 连续约 3s 不减反增,判定 `UNREACHABLE` → HOLD 并上报。

**三个频率不要混**

| 环节 | 频率 | 来源 |
|---|---|---|
| ros2_control 控制/状态环 | **200Hz** | `controller_manager.update_rate: 200` |
| `/feedback/joint_states`、`/feedback/tcp_pose` | **200Hz** | `agx_arm_ctrl` 的 `pub_rate: 200` |
| **我们自己的 P 环** | **100Hz** | 用最新 setpoint + 200Hz tcp 位姿重算误差 |
| 目标 / setpoint 更新 | **10-15Hz** | tracker 输出 |
| Servo 输出命令 | **`publish_period`** | 默认 `0.034s ≈ 29Hz`;计划设 `0.01s = 100Hz` |

关键:**控制环频率 ≠ 目标更新频率**。目标 10-15Hz 变一次,但误差每 10ms 重算一次,
所以运动是连续的,不会"每 100ms 走一步"。

**Servo 本身不会"到达就停"**

- `servo_node` 是**连续速度接口**:持续接收 twist,按 `publish_period` 积分成关节位置输出。
  它没有"到达"概念,只有 `incoming_command_timeout` 超时后 halt。
- "运动到一个目标点就停"是 **`PoseTracking::moveToPose()`** 的 API 语义:
  它是阻塞函数,`satisfiesPoseTolerance()` 一满足就 `break` 返回。
  它是为 waypoint 搬运写的,不是跟踪节点。
- 因此本设计不使用 `moveToPose`,而是在自己的 100Hz 环里持续写 twist;**Servo 全程不停**。

**为什么仍不直接用 `PoseTracking`**

`moveit_servo` 2.5.9 内含 `PoseTracking` 类(`src/pose_tracking.cpp`),提供 PID + 容差。
但它的公开入口只有阻塞式的 `moveToPose()`,循环在容差满足时 `break`。
若要用它做连续跟踪,只能在外层反复调用,语义不干净、且批处理式地丢掉中间误差。

因此:自己写连续 P 环(约 30 行),借用它的容差/增益作为参考值。
若将来需要"停到某个静止位姿"的 waypoint 模式,再复用 `PoseTracking`。

### 6.5 判据与控制律(已确认)

**判据 = 臂端相机光轴与目标方向的夹角。**

等价说法:用 `ComputeLookatPose` 的数学算出目标位姿,伺服把 tcp 驱到该位姿;
夹角就是"目标位姿"与"实际 tcp 位姿"之间的旋转误差。

**lookat 位姿不是唯一的 —— 有一个"距离自由度"**

`build_lookat_orientation` 只决定姿态(相机 z 轴指向目标),不决定 tcp 沿视线放在哪里。
现有 BT 树的取法(`head_arm_independent_lookat.xml:169-175`):

| 参数 | 值 | 含义 |
|---|---|---|
| `xoy_distance_m` | `0.4` | tcp 落在半径 0.4m 的圆周上,方位取目标在 xy 平面的方向 |
| `z_fixed_m` | `0.3` | tcp 高度固定 |
| `ee_position_world` | `0.4;0.0;0.2` | `r_xy≈0` 时的回落 anchor |

伺服沿用这组默认值,并做成参数(行走时可能需要调整,避免臂伸得过直)。

**控制律**

```
e_pos = p_des - p_tcp                       (均在臂基座系)
e_ang = angle(R_des · R_tcp^T)              (轴角,rad)

v = clamp(Kp_pos · e_pos, v_max)            # 默认 v_max ≈ 0.15 m/s
ω = clamp(Kp_ang · e_ang, ω_max)            # 默认 ω_max ≈ 0.6 rad/s
```

- **饱和**:远距离/大角度时直接顶到 `v_max / ω_max`,实现"快速运动到目标位姿"。
- **死区(已定:严格对准)**:`e_ang < 2°` 且 `‖e_pos‖ < 1cm` 时输出零 twist。
  零 twist 仍按控制频率持续发布,避免 Servo 的 `incoming_command_timeout` 触发 halt。
  为抑制行走时围绕零误差的高频抖动,误差先过一阶低通(截止约 5-10Hz)并做速率限幅。
- **不可达检测**:若 `e_ang` 连续约 3s 不减反增,判定 `UNREACHABLE` → HOLD 并上报。

**三个频率不要混**

| 环节 | 频率 | 来源 |
|---|---|---|
| ros2_control 控制/状态环 | **200Hz** | `controller_manager.update_rate: 200` |
| `/feedback/joint_states`、`/feedback/tcp_pose` | **200Hz** | `agx_arm_ctrl` 的 `pub_rate: 200` |
| **我们自己的 P 环** | **100Hz** | 用最新 setpoint + 200Hz tcp 位姿重算误差 |
| 目标 / setpoint 更新 | **10-15Hz** | tracker 输出 |
| Servo 输出命令 | **`publish_period`** | 默认 `0.034s ≈ 29Hz`;计划设 `0.01s = 100Hz` |

关键:**控制环频率 ≠ 目标更新频率**。目标 10-15Hz 变一次,但误差每 10ms 重算一次,
所以运动是连续的,不会"每 100ms 走一步"。

**Servo 本身不会"到达就停"**

- `servo_node` 是**连续速度接口**:持续接收 twist,按 `publish_period` 积分成关节位置输出。
  它没有"到达"概念,只有 `incoming_command_timeout` 超时后 halt。
- "运动到一个目标点就停"是 **`PoseTracking::moveToPose()`** 的 API 语义:
  它是阻塞函数,`satisfiesPoseTolerance()` 一满足就 `break` 返回。
  它是为 waypoint 搬运写的,不是跟踪节点。
- 因此本设计不使用 `moveToPose`,而是在自己的 100Hz 环里持续写 twist;**Servo 全程不停**。

**为什么仍不直接用 `PoseTracking`**

`moveit_servo` 2.5.9 内含 `PoseTracking` 类(`src/pose_tracking.cpp`),提供 PID + 容差。
但它的公开入口只有阻塞式的 `moveToPose()`,循环在容差满足时 `break`。
若要用它做连续跟踪,只能在外层反复调用,语义不干净、且批处理式地丢掉中间误差。

因此:自己写连续 P 环(约 30 行),借用它的容差/增益作为参考值。
若将来需要"停到某个静止位姿"的 waypoint 模式,再复用 `PoseTracking`。

---

## 7. 3D tracker 改造清单

### 7.1 现状

- `TrackObservation` 只有 `position_odom`(`unified_track/track.py`),**相机系原始坐标被丢弃**。
- `Track` 只保留 `last_obs`;`output_formatter` 输出的是 **odom 系 KF 状态**。
- `UnifiedTrack.stamp` = `header.stamp` = pipeline 的 now,不是真实观测时刻。
- KF 是 **3 维位置、无速度**(`output_formatter.py` 注释:"已删除字段 velocity(3 维 KF 无)")。
- 实测输出频率约 **10-12Hz**。
- `UnifiedTrackArray` 目前**无下游消费者**(`docs/tf-frame-prefix.md` §6 遗留 4)。

### 7.2 P0 —— 不改就跑不通伺服

**P0-1 `TrackObservation` 增加 `position_arm_base`**

在 `input_adapter.adapt()` 里对每条 detection 多算一份,走 §6.1 的两条纯已知链。
head 完全不用 TF;arm 只查 `arm1/base_link ← arm1/tcp_link`。

**P0-2 `UnifiedTrack.msg` 增加字段**

```msg
# === 伺服几何(臂基座系,纯已知链,不含 odom) ===
geometry_msgs/Point position_arm_base
string geometry_frame                              # "arm1/base_link"
builtin_interfaces/Time last_observation_stamp     # 真实观测时刻,不是 pipeline now
```

**P0-3 透传每相机贡献**

```msg
string[] contributor_cameras
geometry_msgs/Point[] contributor_positions_arm_base
builtin_interfaces/Time[] contributor_stamps
float32[] contributor_confidences
```

理由:tracker 的价值是**归属**(哪些观测属于同一 track_id),不是替伺服选几何源。
融合后的值是 odom KF 输出,无法反推回臂基座;只有在观测阶段保留每相机的臂基座坐标,
下游才能做"head 优先 / arm 兜底"的切换(§6.3)。

三档策略已确认,因此**本项是必须项**,不能缩减为 head-only:
head 与 arm 两路都必须透传各自的 `position_arm_base`、stamp、置信度。

**P0-4 时间戳语义修正**

伺服要把目标点与 `/feedback/tcp_pose` 对齐,必须知道位置对应哪一帧。
至少补 `last_observation_stamp`;`time_since_last_observation_ms` 已存在,可直接做 HOLD 判据。

### 7.3 P1 —— 提升跟随质量

**P1-1 odom KF 保留,不要搬到臂基座**

- Go2 行走时,目标在 odom 中静止 → 利于跨帧归属。
- 在臂基座系中目标随机器人运动 → 会把自身运动混进关联距离。
- 所以分工:**odom KF 负责归属;臂基座通道只负责几何透传**。

**P1-2 可选:臂基座通道加轻量滤波 + 速度前馈**

3 维位置 KF 无速度,10Hz 下跟随**移动**目标会滞后。

但本期场景是**静态目标**(§1.4),因此:
- **不需要**速度前馈,KF 保持 3 维位置即可;
- 若将来目标会移动(跟踪行人等),再在臂基座通道上加 α-β 或 6 维 KF 出速度。

**第一版不做。**

### 7.4 P2 —— 可选

- **主目标标记**:BT 指定 `track_id` / `class_name`,servo 自行选择,不必改 tracker。
- **`position` 语义**:保持 odom 不变即可(当前无消费者,但没必要冒风险)。

### 7.5 改造后的数据流

```
YOLOE (head/arm) ──ObservationResult──▶ tracker
                                          │ 每条 detection:
                                          │   ├─ p_odom     (现状,KF/归属)
                                          │   └─ p_armbase  (新增,纯已知链)
                                          ▼
                    /perception/tracks/unified  (UnifiedTrackArray)
                     ├─ position (odom, KF)          ← 归属/去重
                     ├─ position_arm_base            ← ★ 伺服几何
                     ├─ contributor_*                ← ★ 几何源选择
                     └─ last_observation_stamp       ← ★ 时间对齐
                                          ▼
                    lgw_servo_controller
                     ├─ 选目标 track
                     ├─ lookat 数学(臂基座系)
                     ├─ 误差 ← /feedback/tcp_pose(同为臂基座系,零 TF)
                     └─ TwistStamped(frame_id=base_link) → moveit_servo
```

---

## 8. 冲突与风险登记

| # | 冲突/风险 | 处置 | 状态 |
|---|---|---|---|
| 1 | `control/joint_states` 200Hz echo 被单写者占用 | controller_manager 互斥切换 | 设计已解 |
| 2 | 释放时 JTC 重激活可能导致回跳(从伺服结束位姿跳回旧轨迹末点) | 依赖 JTC `on_activate` 行为,需实测;必要时释放前注入"起点=当前位姿"的轨迹 | **待实测** |
| 3 | Humble `LoadController.srv` 无 `type` 字段 | 自写组合 launch,或实测 `spawner -t` | **待实测** |
| 4 | Servo 的 `joint_topic` 若接 mock echo,闭环退化开环 | 强制 `joint_topic: /feedback/joint_states` | 设计已定 |
| 5 | Servo 用模型名,TF 用 `arm1/` 前缀;名字不同物 | `planning_frame`/`robot_link_command_frame` 用裸名;twist 的 `frame_id` 与命令帧一致 | 设计已定 |
| 6 | 目标输入 10-15Hz,低于典型 Servo 输入 | `incoming_command_timeout: 0.3`,publish 100Hz 保持 | 设计已定 |
| 7 | 自写闭环无碰撞检查 | Servo 自带,`check_collisions: true` | 设计已解 |
| 8 | tracker 只输出 odom 融合值 | §7 改造 | 待实施 |
| 9 | `arm_mount_offset` 未实测 | 不阻塞伺服;仅影响 odom 派生输出 | 遗留 |
| 10 | eye-in-hand 伺服自激 | 第一版 head 优先 | 建议 |
| 11 | CPU:YOLOE + Servo collision(10Hz) + JTC 在 Orin NX | 实测;必要时降 `collision_check_rate` | **待实测** |
| 12 | 夹爪在伺服期间的归属 | `gripper_controller` 保持 active,MoveIt 仍可规划夹爪 | 设计已解 |
| 13 | Go2 行走时基座运动作为扰动进入伺服环 | 降低增益;先在静止条件下验证,再逐步加速;`incoming_command_timeout` 保持短 | **待实测** |
| 14 | 目标接近机械臂可达角域边界 | Servo 饱和在关节限位;状态机上报 `SATURATED`,本期不自动转体 | 设计已定 |
| 15 | 行走伺服时 Go2 身体不在 planning scene,碰撞检查看不见自身 | 启用 `agx_arm_ctrl/piper_with_go2_collision.launch.py` 中的 `go2_collision_scene` 节点 | **待实施** |
| 16 | 目标在 head/arm/odom 三档间抖动 | 高优先级切回需滞回;低优先级立即降级;低档降增益 | 设计已定 |
| 17 | odom 兜底假定目标静止 | 仅在 track 未过期(≤1500ms)内启用;过期即 HOLD | 设计已定 |
| 13 | Go2 行走时基座运动作为扰动进入伺服环 | 降低增益;先在静止条件下验证,再逐步加速;`incoming_command_timeout` 保持短 | **待实测** |
| 14 | 目标接近机械臂可达角域边界 | Servo 饱和在关节限位;状态机上报 `SATURATED`,本期不自动转体 | 设计已定 |
| 15 | 行走伺服时 Go2 身体不在 planning scene,碰撞检查看不见自身 | 启用 `agx_arm_ctrl/piper_with_go2_collision.launch.py` 中的 `go2_collision_scene` 节点 | **待实施** |
| 16 | 目标在 head/arm/odom 三档间抖动 | 高优先级切回需滞回;低优先级立即降级;低档降增益 | 设计已定 |
| 17 | odom 兜底假定目标静止 | 仅在 track 未过期(≤1500ms)内启用;过期即 HOLD | 设计已定 |

---

## 9. 实施顺序(建议)

1. **验证 L1**:在不动真机的前提下,确认能给 `controller_manager` 注入一个 inactive 的
   `JointGroupPositionController`,并能 `switch_controller` 往返。
2. **验证 L2**:`moveit_servo` 空跑(不切 controller),检查参数加载、PSM、`/servo_node/status`。
3. **验证回跳**:在 mock 或低速真机上确认 JTC 重激活不会跳。
4. **L3 骨架**:状态机 + 服务接口,先只做 `Takeover/Release`,`TwistStamped` 固定为零。
5. **接入 lookat**:用一个硬编码的臂基座目标点验证 setpoint 与误差符号。
6. **接入 tracker**:按 head → arm → odom 三档依次接入,每档单独验证后再叠加滞回与降增益。
7. **真机联调**:限速、限工作空间、E-stop 随手可触。

---

## 10. 待验证项(实施前必须实测)

| # | 待验证 | 方法 | 通过标准 |
|---|---|---|---|
| 1 | `control_topic` remap 可覆盖 | `ros2 node info /ros2_control_node` 看 `joint_states` 的 remap | 输出到指定 topic |
| 2 | controller 类型可否运行时注入 | `spawner -t ... --inactive` 实测 | 控制器出现在 `ros2 control list_controllers` |
| 3 | JTC 重激活是否回跳 | 手动切 controller 并在切回后观察 `/feedback/joint_states` | 无不连续跳变 |
| 4 | Servo 参数是否全部加载 | `/servo_node/status` + 日志 | 无参数校验失败 |
| 5 | Servo PSM 在有 prefix 的 TF 下正常 | 加载后看 collision 场景 | 无 TF 报错 |
| 6 | `/feedback/joint_states` 频率与延迟 | `ros2 topic hz` / `delay` | ≥200Hz,延迟 <20ms |
| 7 | CPU 余量 | `top` / `docker stats` 在 servo + YOLOE 同时运行时 | 无持续满载 |
| 8 | `arm_mount_offset` | 卷尺 + 静态位姿测量 | 仅供 odom 派生使用 |

---

## 11. 已决 / 未决

### 已决(2026-10-26)

1. **几何源三档**:head 优先 → arm 兜底 → odom 最后兜底;高优先级切回需滞回,降级立即生效,低档降增益。
2. **本期范围**:只实现服务端节点,不做 BT 交互。将来可"作为 BT 技能"或"被 BT 技能调用的节点",两种形态都要兼容。
3. **目标选择**:显式 `track_id`,或服务端自主选 odom 系下距 `base_link` 最近的 confirmed track。
4. **Go2 行走中伺服**:允许;行走作为基座扰动处理,不联动转体。
5. **odom 兜底可行性**:理论上成立,误差是常量刚体变换(§6.4)。
6. **FOV 判据**:用臂端相机光轴与目标方向的夹角,不用图像像素偏移(§6.5)。
7. **lookat 位姿的取法**:沿用 `xoy_distance_m=0.4` + `z_fixed_m=0.3`,输出帧 `base_link`;v1 固定不动态调整。
8. **不使用 `PoseTracking::moveToPose`**:它是阻塞式 waypoint,不符合连续跟踪(§6.5)。
9. **死区:严格对准** —— `e_ang < 2°` 且 `‖e_pos‖ < 1cm`;持续微调,不加 FOV 边缘判据。
10. **控制环 100Hz,目标更新 10-15Hz**:两者解耦(§6.5)。
11. **完成判据**:只有显式 `release` 才归还控制权;自动退出条件(`auto_release_distance_m`、`max_tracking_duration_s`)可选且默认关闭。目标丢失 → **HOLD**,不自动还权。
12. **静态目标假设**:KF 无速度分量是合理建模;odom 兜底可用到 track 过期(≤1500ms),增益 0.5×,之后 HOLD。
13. **`track_id` 消失**:默认 HOLD;可选"有条件重捕获"(同类别 + 距上次已知位置 < 0.5m + status=confirmed)。
14. **接口形态**:服务 + state 话题,不做 action。
15. **近距离保护**:`xoy_distance_m` 固定 0.4;新增 `min_observe_distance_m`(默认 0.35m),低于此值上报 `TOO_CLOSE`。

### 仍未决

1. **具体阈值需真机调参**:2° / 1cm / `v_max` / `ω_max` / 0.5m 重捕获半径 / 0.35m 近距保护。
2. **行走时 `xoy_distance_m` 是否要随步态调整**:Go2 行走有俯仰与抬升,`z_fixed_m=0.3` 是机身系量,是否需要补偿。
3. **是否需要 standoff 模式**(相机到目标保持恒定距离):取决于后续抓取阶段的精度要求。
4. **与 BT 对接的具体形态**:作为 BT 技能节点 vs 被调用节点,留待 BT 阶段决定;本期只保证状态机逻辑与 ROS 接口层分离。

---

## 12. 参考与证据索引

| 主题 | 位置 |
|---|---|
| 控制链路 / mock 硬件 | `agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.ros2_control.xacro` |
| controller YAML 生成 | `agx_arm_moveit/launch/demo.launch.py`(`_build_ros2_controllers_file`) |
| `joint_states` remap | `demo.launch.py:205` |
| control gate | `agx_arm_moveit/scripts/agx_arm_control_gate`;`demo.launch.py:156-171` |
| 驱动订阅与转发 | `agx_arm_ctrl/agx_arm_ctrl_single_node.py:330-367, 697-710, 737-751` |
| 夹爪测试格式 | `agx_arm_ros/test/gripper/test_gripper_joint_states.yaml` |
| TF 前缀与桥 | `docs/tf-frame-prefix.md` |
| tracker 输入适配 | `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py` |
| tracker 输出格式 | `.../unified_track/output_formatter.py` |
| tracker 运行时配置 | `src/lgw_3d_tracker/config/runtime.yaml` |
| lookat 数学 | `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp` |
| 标定文件 | `src/lgw_bt_planner/calibration/{arm,head}_calibration.json` |
| moveit_servo 参数校验 | `/home/unitree/ros2_ws/src/moveit2/moveit_ros/moveit_servo/src/servo_parameters.cpp:405,422` |
| moveit_servo 示例参数 | `.../moveit_servo/config/panda_simulated_config.yaml` |
