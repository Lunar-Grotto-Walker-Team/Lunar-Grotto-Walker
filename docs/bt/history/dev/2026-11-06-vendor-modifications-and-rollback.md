# agx 机械臂 vendor 修改与回退记录（2026-11-06）

> 目的：**任何一条改动都能一键回到原状**，避免 topic-based 硬件插件失败时整条
> MoveIt/Servo pipeline 不可用。所有修改都在厂商包内，**不在 git 跟踪范围**
> （`agx_arm_ws` 的 git 排除了 embedded `agx_arm_ros`）。

## 已做的修改（真机事故后的安全修复）

### V1. `agx_arm_moveit/config/initial_positions.yaml`
FakeSystem 初始状态（起栈时经 JSB 下发给驱动的第一条指令）。

| 关节 | 原值 | 现值 |
|---|---|---|
| joint2 | 0.0 | **0.30** |
| joint3 | 0.0 | **-0.20** |

原因：joint3 的 0 就是它的上限（URDF `[-2.967, 0]`），起栈指令把 j3 压到限位，
固件进入保护/失能，腕部靠重力落下。

**回退**：两个值改回 `0.0`。

### V2. `agx_arm_moveit/config/agx_arm.srdf.xacro`（2 处 `group_state name="home"`）
### V3. `agx_arm_moveit/config/agx_arm.srdf`（1 处）
BT / MoveGroup 的 Named "home" 位姿。

| 关节 | 原值 | 现值 |
|---|---|---|
| joint2 | 0 | **0.30** |
| joint3 | 0 | **-0.20** |

**回退**：三处 `value="0.30"` → `value="0"`、`value="-0.20"` → `value="0"`。

### V4. `agx_arm_ctrl/.../agx_arm_ctrl_single_node.py` → `_move_home_callback`
驱动 `/move_home` 的目标。

原：`move_j([0] * self.arm_joint_count)` / `move_js([0] * ...)`
现：`home = [0]*n; home[1]=0.30; home[2]=-0.20; move_j(home)`

**回退**：恢复两行 `move_j([0] * self.arm_joint_count)` / `move_js(...)`。

> V2/V3/V4 改完需要 `colcon build --packages-select agx_arm_moveit agx_arm_ctrl`
> （已做过）。

## V5–V9. topic-based 硬件插件（已实施 2026-11-06）

| # | 文件 | 改动 | 回退方式 |
|---|---|---|---|
| V5 | `agx_arm_moveit/config/agx_arm.urdf.xacro` | 新增 `<xacro:arg name="hardware_backend" default="fake"/>`，透传给 ros2_control 宏 | 默认值即回退（不改文件） |
| V6 | `agx_arm_moveit/config/agx_arm.ros2_control.xacro` | 宏参数 `hardware_backend:=fake state_topic:=/feedback/joint_states command_topic:=/control/joint_states`；`topic` 走 `<plugin>agx_topic_hardware/AgxTopicSystem</plugin>`，否则 `mock_components/GenericSystem` | 同上 |
| V7 | `agx_arm_moveit/launch/_moveit_config_builder.py` | `urdf_mappings` 增加 `"hardware_backend": hardware_backend` | 同上 |
| V8 | `agx_arm_moveit/launch/demo.launch.py` | ① `DeclareLaunchArgument("hardware_backend", fake|topic, default fake)`；② `_build_moveit` 里 `jsb_topic = "/joint_states" if topic else control_topic`，`ros2_control_node` 的 `joint_states` remap 用它；③ `_build_ros2_controllers_file(..., hardware_backend)`，topic 时给 arm_controller 加 `open_loop_control: True` + 轨迹容差 | ①②`默认 fake` 时行为不变；③ 整块只在 topic 分支 |
| V9 | `lgw_bt_planner/launch/lgw_full.launch.py` | 声明并透传 `hardware_backend` 给 demo.launch | 默认 fake |

> **V8② 是最关键的一条**：FakeSystem 时代 JSB 必须重映射到 `control_topic`，因为
> “状态即指令”就是靠它下发。换成 topic 后端后 state 来自插件，若仍让 JSB 发到
> `/control/joint_states`，它会在插件拿到反馈之前把初始状态（全零/park）灌给驱动
> —— 实测把手臂从 park 拉到 j3=0 限位。topic 后端下 JSB 必须发到 `/joint_states`。

### 插件侧的启动安全（与 vendor 无关，但同样要记住）

`src/agx_topic_hardware/src/agx_topic_system.cpp` 的 `write()`：

1. 第一帧真实反馈到达前往 `/control/joint_states` **一个字节都不发**；
2. 第一帧反馈把命令缓冲 seed 成真实位姿（首条指令 = “保持当前位置”）；
3. 之后只在命令变化（>1e-4 rad，限 50 Hz）或 200 ms 心跳时发布；
4. 任何 NaN 命令直接丢弃。

去掉这三条中的任意一条，起栈就会把手臂拖向 home。

**回退开关**：`hardware_backend:=fake`（默认）→ 行为与今天完全一致；
`hardware_backend:=topic` → 新插件。切换只需 launch 参数，**不需要改文件**。
唯一的例外是 V8②：若要把 JSB remap 彻底恢复成常量，改回
`remappings=[("joint_states", LaunchConfiguration("control_topic"))]`。

## 新包（在 git 跟踪范围内）

`lg_walker_ws/src/agx_topic_hardware/` —— 插件本身，随主仓库版本管理；
vendor 修改只有上面的参数与宏选择，尽量薄。

## 真机验证结果（2026-11-06）

| 检查项 | fake 后端（旧） | topic 后端（新） |
|---|---|---|
| 起栈是否动臂 | 修 park 后不动 | **不动**（seed 修复后 15 s 采样恒定） |
| Servo 状态源 | JSB 回声（state=command） | `/feedback/joint_states`（真实） |
| MoveGroup 规划轨迹 | 瞬间“完成” | JTC 容差 `goal_time 2.0 / stopped_velocity 0.2 / 每关节 0.5,0.15` |
| 恢复时 probe 校验 | FAULT（mock 先完成、真机滞后） | **不再 FAULT**（状态是真的，probe 与真实位姿一致） |
| 120 s 假目标跟踪 | 之前轮次 | 跟踪正常，2 次 `servo_halt` 各 ~1.8 s 恢复成功，0 次 FAULT |

跟踪窗口结束后栈**保持运行**（用户要求）。

## 抖动问题的根因与修复（2026-11-06 下午，真机迭代）

topic 后端上来后手臂"一卡一卡、不如 mock 丝滑"，逐一排除后定位到**三处**，全部已修：

| # | 现象 | 根因 | 修法 |
|---|---|---|---|
| 1 | 走一下停一下（反馈 38% 时间完全静止，最长 540 ms） | 插件把 `/control/joint_states` 限流到 50 Hz；驱动把每条消息当一次点对点 `move_j` | 插件改为**每周期 200 Hz 发布**（等于旧 JSB） |
| 2 | 换支时手腕来回摆（静止期 j4 p2p 1.62 rad = 93°） | 控制律 `tcp_now` 用 TF（=真机滞后位姿），Servo 却积分在 command 域，两个域不一致 | `tcp_pose_domain: command`（`servo_node.yaml`）：`tcp_now` 改由 `/control/joint_states` 做 FK |
| 3 | 命令流带 29 Hz 锯齿（20–40 Hz 纹波 0.05–0.47°，mock 仅 0.04–0.08°） | JTC 每次收到 Servo 新点都从**真机滞后位置**重新起插，命令在两次更新之间被拉回 | arm_controller 加 **`open_loop_control: True`**（JTC 从自己上一条命令起插） |

配套：`measured-velocity damping`（`kd_pos/kd_ang`，单位秒）只在 `tcp_pose_domain: measured` 时启用，作该回退路径的相位补偿（实测过冲 13.2%→8.0%，穿零 23→6 次）。

**修复后真机实测**：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| 平台静止时手臂 p2p（ivot 6.4 s 窗口） | 1620 mrad（93°） | **0.0–2.8 mrad（0–0.16°）** |
| 手臂相对命令 | 最大 50°（换支时） | **1–2.4°** |
| 命令 20–40 Hz 纹波 | 0.05–0.47° rms | **0.001–0.009° rms** |
| 60 s 扫掠 HOLD 次数 | 11 | **0** |
| QP 姿态解 `max_joint_delta` | 随换支跳 ±1.2 rad | **p2p = 0.0000** |

> 另一处环境问题：多次栈重启会**泄漏 `odom_tf_bridge.py` 进程**（曾达 10 个并发），
> 多个 `odom→base_link` 广播者会让目标在基座系抖动。已补进 `scripts/bench_cleanup.sh`。
