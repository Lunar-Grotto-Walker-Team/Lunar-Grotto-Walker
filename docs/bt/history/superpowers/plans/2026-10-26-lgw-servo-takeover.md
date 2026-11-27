# lgw Servo 接管与视觉伺服 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增加一条 MoveIt Servo 接管机械臂的通路,使臂端相机持续对准 3D tracker 的静态目标;释放时把控制权交还 MoveGroup。

**Architecture:** 单栈:一次 bringup 启动 driver + MoveIt + Servo + BT。接缝用 JTC topic:Servo 发 `JointTrajectory` 到 `/arm_controller/joint_trajectory`,`arm_controller` 始终是唯一写者;MoveIt 走 action,Servo 走 topic,运行时通过服务切换。自研部分只有 `lgw_servo_node`(状态机 + 几何源 + lookat + 100Hz P 环)。纯逻辑与 ROS 接口分离。

**Tech Stack:** ROS 2 Humble, `moveit_servo` 2.5.x, `ros2_control`(`mock_components/GenericSystem`), `JointTrajectoryController`, tf2, gtest, BehaviorTree 无关(v1 不接 BT)。

**Spec:** `docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md`

## Global Constraints

- 容器:`self_l4t`,工作目录 `/my_ws/l4t/lg_walker_ws`;构建前 `source /my_ws/l4t/l4t_setup.sh`。容器未运行时先 `docker start self_l4t`。
- 不 fork 上游包;所有新代码放 `src/lgw_bt_planner`(spec D11)。
- 本容器不能 `colcon test`(缺 `ament_cmake_test`);gtest 直接运行 `build/lgw_bt_planner/<test_binary>`。
- 构建:`colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON`。
- C++17;数学类型沿用 tf2(`tf2::Vector3` / `tf2::Matrix3x3` / `tf2::Transform`),不引入 Eigen 到公开头文件。
- 模型名 vs TF 名:MoveIt/Servo 用 `base_link`、`link6`、`tcp_link`;TF 用 `arm1/base_link`、`arm1/tcp_link`。
- Servo frame:`planning_frame=base_link`、`robot_link_command_frame=base_link`、`ee_frame_name=tcp_link`、`move_group_name=arm`。
  **`tcp_link` 就是 SRDF `arm` 组的 tip(雅可比/IK 求值点),控制律在它上面算误差,不做 link6 换算。**
- controller 名:`arm_controller`、`gripper_controller`、`joint_state_broadcaster`;关节 `joint1…joint6`。
- 固定数值:`xoy_distance_m=0.4`、`z_fixed_m=0.3`、`ee_anchor=(0.4,0,0.2)`。
  `tcp_offset=[0,0,0.12,0,0,0]` 只用于 launch:必须同时传给 `agx_arm_ctrl` 和 `demo.launch.py`
  (URDF `tcp_link` 与驱动 tcp 约定一致),控制律里不再出现它。
- 几何源:`head_max_age_ms=400`、`arm_max_age_ms=400`、`odom_fallback_max_age_ms=1500`、`upgrade_hysteresis_ticks=30`;增益 `head=1.0`、`arm=0.7`、`odom=0.5`。
- 控制律:`Kp_pos=2.0`、`Kp_ang=3.0`、`v_max=0.15 m/s`、`ω_max=0.6 rad/s`、死区 `2° / 1cm`、低通 `8Hz`、控制环 `100Hz`。
- Servo 参数:`incoming_command_timeout=0.3`、`publish_period=0.01`、`collision_check_rate=10.0`、
  `joint_topic` 按模式取:真机 `follow:=true` → `/feedback/joint_states`,台架 `follow:=false` → `/control/joint_states`。
- 节点位姿源:`pose_watch_topic` 与上面同源;取 tcp 位姿用 TF(`<prefix>base_link ← <prefix>tcp_link`),零超时 `can_transform` 探测。
- MoveGroup 忙判据:`/move_action/_action/status` 有 `ACCEPTED`/`EXECUTING` **或**
  `/arm_controller/follow_joint_trajectory/_action/status` 有活动 goal。
- Servo 输出:`trajectory_msgs/JointTrajectory` → `/arm_controller/joint_trajectory`;`publish_joint_positions/velocities=true`。
- 最终单栈;开发期两栈仅用于隔离验证,不能同时运行(抢 can0)。
- 驱动模式(`move_j`/`move_js`/`cpv`)是进程级,单栈选一个,对 MoveIt/Servo 同时生效。
- v1 不做候选池/自动切换;不检查 Go2 机身碰撞;新 launch 与旧栈并行但互斥;不做 BT 交互。
- 禁止 `pkill -f ros`;清理按 PID。

## Verified Environment (2026-10-27)

以下为已实测事实,供执行者参考;**不改变本计划 Task 1–10 的设计**。

| 项 | 值 |
|---|---|
| `agx_arm_ros` 分支 | `servo-cpv` @ `4d7b2b4` |
| 基点 | `b9ad14d`(CPV + JointState 末端联动 + 反馈恢复) |
| 回滚点 | `cfd71ecd`(`ros2`)/ `local-patches` 分支 |
| 固件 | `S-V1.8-8`(v188,Piper) |
| CPV 只读 | `get_cpv_cv` 6/6 关节返回数值(初始 0.0) |
| CPV 运动 | 关节 6 +0.05 rad,`cv=0.2`、`acc=dcc=0.5`,平滑无过冲,回位误差 0.00037 rad |
| 副作用 | 关节 6 `cpv_cv` 已持久化为 0.2 rad/s |
| 未实现 | 驱动 `control_mode` 参数仍不存在;Task 6/7 已按单栈 + JTC topic 接缝重写 |

探针脚本:`agx_arm_ws/scripts/cpv_probe.py`(只读)、`cpv_motion_probe.py`(运动测试)。

> 若后续决定采用 CPV,需要新增一个驱动补丁任务:在 `_control_arm_joints` 中按
> `control_mode`(`j` / `js` / `cpv`)分派,并在启动时一次性配置 6 个关节的
> `set_cpv_acc/dcc/cv`(建议 `cv=0.5 rad/s`、`acc=dcc=2.0 rad/s²`,不要用默认
> `max_joint_spd=3.0`)。届时新增驱动补丁任务;Task 6/7 的单栈接缝不变。

## Review Focus

1. **检测年龄的时钟基准**:若实现者用"收到消息的时刻"而不是 detection 的 capture stamp,稳态年龄会少算 ~180ms,几何源会在阈值附近抖动。Task 3 的测试用显式 `stamp_ns` 钉死 capture-stamp 语义。
2. **`start_servo` 的假成功**:服务即使 `waitForCompleteState` 超时也返回 `success=true`。Task 7 的验收必须从 `/servo_node/status`(`std_msgs/Int8`)或命令输出确认真的启动。
3. **参考点必须是 `tcp_link`**:SRDF `arm` 组 tip 是 `tcp_link`,Servo 的雅可比/IK 都在它上面求值;
   若按 `link6` 算误差会凭空引入 12cm 偏置(1cm 死区永远达不到)。Task 2 的测试锚定"期望位姿 = `compute_tcp_pose` 输出,不做任何偏移换算"。
4. **odom TF 缺失时阻塞**:`lookup_transform` 在缺帧时会等满 timeout,饿死 100Hz 定时器。Task 3 只接受 `std::optional<tf2::Transform>`,缺失时返回 `ODOM` 不可用;Task 7 必须先 `can_transform(0)`。
5. **tracker 断流导致生命周期冻结**:`time_since_last_observation_ms` 只在 pipeline tick 时增长;相机/YOLOE 断流后它不再变化。Task 5 的看门狗测试用"数组到达时间"而不是 track 字段判断超时。

---

## File Structure

### 新增

| 文件 | 职责 |
|---|---|
| `src/lgw_bt_planner/srv/Takeover.srv` | 接管服务 |
| `src/lgw_bt_planner/msg/ServoState.msg` | 状态消息 |
| `src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp` | `SourceKind` / `ServoState` / `Pose6` / `SourceSelection` |
| `src/lgw_bt_planner/include/lgw_bt_planner/lookat_setpoint.hpp` | lookat 位姿纯逻辑 |
| `src/lgw_bt_planner/src/lookat_setpoint.cpp` | 同上 |
| `src/lgw_bt_planner/include/lgw_bt_planner/geometry_source.hpp` | 三档选择 + odom 反算纯逻辑 |
| `src/lgw_bt_planner/src/geometry_source.cpp` | 同上 |
| `src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp` | 目标选择 + reacquire 纯逻辑 |
| `src/lgw_bt_planner/src/target_selector.cpp` | 同上 |
| `src/lgw_bt_planner/include/lgw_bt_planner/servo_control_law.hpp` | P 环/死区/低通纯逻辑 |
| `src/lgw_bt_planner/src/servo_control_law.cpp` | 同上 |
| `src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp` | 状态机 + 流看门狗 |
| `src/lgw_bt_planner/src/servo_state_machine.cpp` | 同上 |
| `src/lgw_bt_planner/src/servo_node.cpp` | ROS 接口层 |
| `src/lgw_bt_planner/config/servo_params.yaml` | moveit_servo 参数 |
| `src/lgw_bt_planner/config/servo_node.yaml` | lgw_servo_node 参数 |
| `src/lgw_bt_planner/launch/lgw_full.launch.py` | 单栈 bringup |
| `src/lgw_bt_planner/tests/test_lookat_setpoint.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_geometry_source.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_target_selector.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_servo_control_law.cpp` | 单测 |
| `src/lgw_bt_planner/tests/test_servo_state_machine.cpp` | 单测 |
| `scripts/start_lgw_full.sh` | 单栈 bringup |
| `scripts/driver_mode_eval.py` | `move_j` vs `move_js` 评估 |
| `scripts/fake_track_publisher.py` | 台架静态 track(15Hz) |
| `src/lgw_3d_tracker/test/test_arm_base_geometry.py` | tracker 新字段单测 |

### 修改

| 文件 | 改动 |
|---|---|
| `src/lgw_bt_planner/CMakeLists.txt` | rosidl 加 srv/msg、新源文件、新 gtest、install config |
| `src/lgw_bt_planner/package.xml` | 加 `std_srvs`、`action_msgs`、`moveit_servo`(exec) |
| `src/yoloe_interfaces/msg/UnifiedTrack.msg` | 新增臂基座几何与贡献者字段 |
| `src/lgw_3d_tracker/.../track.py` | `TrackObservation` 增字段 |
| `src/lgw_3d_tracker/.../input_adapter.py` | 每条 detection 算 `position_arm_base` |
| `src/lgw_3d_tracker/.../output_formatter.py` | 输出新字段与贡献者数组 |

---

### Task 1: 接口定义与构建骨架

**Files:**
- Create: `src/lgw_bt_planner/srv/Takeover.srv`
- Create: `src/lgw_bt_planner/msg/ServoState.msg`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(rosidl 生成列表)
- Modify: `src/lgw_bt_planner/package.xml`

**Interfaces:**
- Produces: `lgw_bt_planner/srv/Takeover`、`lgw_bt_planner/msg/ServoState`、`lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp`

- [ ] **Step 1: 写接口文件**

`srv/Takeover.srv`:
```
uint64 track_id
string class_filter
---
bool success
string message
string state
```

`msg/ServoState.msg`:
```
string state
string geometry_source
uint64 track_id
float64 angular_error_rad
float64 position_error_m
float64 target_distance_m
bool converged
bool saturated
bool too_close
builtin_interfaces/Time stamp
```

`include/lgw_bt_planner/servo_types.hpp`: 定义 `enum class SourceKind { HEAD, ARM, ODOM, NONE }`、
`enum class ServoState { IDLE, PREEMPT, TRACKING, HOLD, RELEASING, FAULT }`、
`struct Pose6 { tf2::Vector3 p; tf2::Matrix3x3 R; }`、
`struct SourceSelection { SourceKind kind; tf2::Vector3 p_arm_base; double scale; std::string reason; }`。

- [ ] **Step 2: 运行,确认接口还不存在**

Run: `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && ros2 interface show lgw_bt_planner/srv/Takeover"`
Expected: FAIL(`Invalid message type` / 找不到接口)

- [ ] **Step 3: 接入构建**

`CMakeLists.txt`:`rosidl_generate_interfaces` 增加 `"msg/ServoState.msg"`;
`ament_target_dependencies(lgw_bt_planner_nodes ...)` 增加 `std_srvs action_msgs`;
把 `servo_types.hpp` 所在 include 目录已被现有 `install(DIRECTORY include/ ...)` 覆盖,无需新增。
`package.xml` 增加 `<depend>std_srvs</depend>`、`<depend>action_msgs</depend>`、
`<exec_depend>moveit_servo</exec_depend>`。

- [ ] **Step 4: 构建并确认接口可用**

Run:
```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && \
   colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON && \
   source install/setup.bash && \
   ros2 interface show lgw_bt_planner/srv/Takeover && \
   ros2 interface show lgw_bt_planner/msg/ServoState"
```
Expected: 两条接口定义完整打印;构建无错误。

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/srv/Takeover.srv src/lgw_bt_planner/msg/ServoState.msg \
        src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp \
        src/lgw_bt_planner/CMakeLists.txt src/lgw_bt_planner/package.xml
git commit -m "feat(lgw_bt_planner): add servo takeover interfaces"
```

---

### Task 2: lookat_setpoint 纯模块

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/lookat_setpoint.hpp`
- Create: `src/lgw_bt_planner/src/lookat_setpoint.cpp`
- Create: `src/lgw_bt_planner/tests/test_lookat_setpoint.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(源文件 + gtest)

**Interfaces:**
- Consumes: `build_lookat_orientation`、`compute_tcp_pose`(`compute_lookat_pose_node.hpp`)
- Produces:
```cpp
struct LookatParams {
  double xoy_distance_m{0.4};
  double z_fixed_m{0.3};
  tf2::Vector3 ee_anchor{0.4, 0.0, 0.2};
};
struct LookatResult {
  Pose6 tcp_desired;          // 期望的 tcp_link 位姿 = Servo 参考点
  bool used_anchor{false};
};
LookatResult compute_lookat_setpoint(const tf2::Vector3& p_target_base,
                                     const tf2::Matrix3x3& R_cam_in_tcp,
                                     const tf2::Vector3& t_cam_in_tcp,
                                     const LookatParams& params);
```

- [ ] **Step 1: 写失败测试**

`tests/test_lookat_setpoint.cpp`,至少四条:

```cpp
TEST(LookatSetpoint, CameraOpticalAxisPointsAtTarget) {
  LookatParams p;  // 0.4 / 0.3
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = compute_lookat_setpoint({0.5, 0.0, 0.3}, R, {0, 0, 0}, p);
  tf2::Vector3 cam_z(r.tcp_desired.R[0][2], r.tcp_desired.R[1][2], r.tcp_desired.R[2][2]);
  EXPECT_NEAR(cam_z.dot(tf2::Vector3(1, 0, 0)), 1.0, 1e-6);
}

TEST(LookatSetpoint, TcpPoseIsTheSetpointItself) {
  LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = compute_lookat_setpoint({0.8, 0.0, 0.3}, R, {0, 0, 0}, p);
  // 无标定偏置时,期望位姿就是 standoff 点本身;任何额外 12cm 偏移都会打破这
  // 三条断言(tcp_link 是 Servo 的雅可比求值点,不能"换算"到 link6)。
  EXPECT_NEAR(r.tcp_desired.p.x(), 0.4, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.y(), 0.0, 1e-9);
  EXPECT_NEAR(r.tcp_desired.p.z(), 0.3, 1e-9);
}

TEST(LookatSetpoint, DegenerateTargetFallsBackToAnchor) {
  LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = compute_lookat_setpoint({0.0, 0.0, 1.0}, R, {0, 0, 0}, p);
  EXPECT_TRUE(r.used_anchor);
}

TEST(LookatSetpoint, UprightnessFlipIsApplied) {
  LookatParams p;
  tf2::Matrix3x3 R = tf2::Matrix3x3::getIdentity();
  auto r = compute_lookat_setpoint({0.8, 0.0, 0.3}, R, {0, 0, 0}, p);
  // 与 BT 路径同一份翻转:cam_y·base_z 不得为正(det 仍为 +1)
  tf2::Vector3 cam_y(r.tcp_desired.R[0][1], r.tcp_desired.R[1][1],
                     r.tcp_desired.R[2][1]);
  EXPECT_LE(cam_y.dot(tf2::Vector3(0, 0, 1)), 0.0);
  EXPECT_NEAR(r.tcp_desired.R.determinant(), 1.0, 1e-9);
}
```

> 距离(`target_distance_m`)不在本模块:期望 standoff 恒为 0.4,拿期望位姿算距离永远是 0.4,
> 近距保护会失效。该值由节点用**当前**相机位姿算(spec §7.6)。

- [ ] **Step 2: 运行,确认失败**

Run: `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON && ./build/lgw_bt_planner/test_lookat_setpoint`
Expected: FAIL(编译错误:`compute_lookat_setpoint` 未声明)

- [ ] **Step 3: 实现**

`compute_lookat_setpoint` 内部:
1. `r_xy = hypot(p_target.x, p_target.y)`;`r_xy < 1e-3` → `P_ee = ee_anchor`,`used_anchor=true`。
2. 否则 `P_ee = (d·x/r_xy, d·y/r_xy, z_fixed)`。
3. `direction = (p_target - P_ee).normalized()`。
4. `R_arm = build_lookat_orientation(direction, {0,0,1})`。
5. `tcp = compute_tcp_pose(P_ee, R_arm, R_cam_in_tcp, t_cam_in_tcp)`,转换成 `Pose6`。
   注意:先对 `R_arm` 施加 `apply_uprightness_flip(R_arm, (0,0,1))`(与 BT 节点同一份实现)。
6. 不计算距离——见上方说明,距离由节点用当前位姿算。

- [ ] **Step 4: 运行,确认通过**

Run: `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON && ./build/lgw_bt_planner/test_lookat_setpoint`
Expected: 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/lookat_setpoint.hpp \
        src/lgw_bt_planner/src/lookat_setpoint.cpp \
        src/lgw_bt_planner/tests/test_lookat_setpoint.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add lookat setpoint module"
```

---

### Task 2A: target_selector 纯模块

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp`
- Create: `src/lgw_bt_planner/src/target_selector.cpp`
- Create: `src/lgw_bt_planner/tests/test_target_selector.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(源文件 + gtest)

**Interfaces:**
- Consumes: `servo_types.hpp`(Task 1)、`tf2::Vector3`
- Produces:
```cpp
struct TrackCandidate {
  uint64_t track_id{0};
  std::string status;              // "confirmed" / "coasting"
  std::string class_name;
  tf2::Vector3 p_odom;
  int64_t last_obs_stamp_ns{0};
  int64_t time_since_last_obs_ms{0};
};
struct TargetSelectorParams {
  uint64_t requested_track_id{0};  // 0 = auto
  std::string class_filter;        // 空 = 不限
  bool reacquire_on_loss{false};
  double reacquire_radius_m{0.5};
};
struct TargetSelectorState {
  uint64_t last_track_id{0};
  tf2::Vector3 last_known_p_odom;
  std::string last_class;
  bool has_last{false};
};
struct TargetSelection {
  bool ok{false};
  size_t index{0};                 // candidates 下标,ok=false 时无意义
  uint64_t track_id{0};
  std::string reason;              // requested / nearest / reacquired /
                                   // requested_missing / no_candidate / no_base_pose
};

TargetSelection select_target(const std::vector<TrackCandidate>& candidates,
                              const std::optional<tf2::Vector3>& p_base_in_odom,
                              const TargetSelectorParams& params,
                              TargetSelectorState& state);
```

- [ ] **Step 1: 写失败测试**

`tests/test_target_selector.cpp`,六条:

```cpp
TEST(TargetSelector, RequestedTrackWins) {
  // id=7 存在;另有更近的 id=3 → 仍选 7,reason="requested"
}
TEST(TargetSelector, AutoPicksNearestConfirmed) {
  // id=3 距 base 1m、id=7 距 base 3m(id=5 是 coasting)→ 选 3,reason="nearest"
}
TEST(TargetSelector, ClassFilterApplies) {
  // class_filter="cup";最近的 confirmed 是 "person" → 选 cup
}
TEST(TargetSelector, MissingRequestedHolds) {
  // requested=9 不在数组、reacquire_on_loss=false → ok=false, reason="requested_missing"
}
TEST(TargetSelector, ReacquireNeedsSameClassAndRadius) {
  // 上次 7("cup", p=(1,0,0));数组中 "cup" 在 0.4m 内 → 选中;0.6m 外或 "person" → 不选
}
TEST(TargetSelector, AutoWithoutBasePoseFails) {
  // p_base_in_odom = nullopt → ok=false, reason="no_base_pose"
}
```

- [ ] **Step 2: 运行,确认失败**

Run: `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON && ./build/lgw_bt_planner/test_target_selector`
Expected: FAIL(编译错误:未声明)

- [ ] **Step 3: 实现**

1. `requested_track_id != 0`:
   - 在 candidates 中找该 id → 命中即返回(`reason="requested"`),**不看 status**
     (过期由 Task 3 的几何源判);
   - 未命中且 `reacquire_on_loss && state.has_last`:同 `last_class`、`status=="confirmed"`、
     `‖p_odom - state.last_known_p_odom‖ < reacquire_radius_m` → 返回(`reason="reacquired"`);
   - 否则 `ok=false, reason="requested_missing"`。
2. `requested_track_id == 0`:
   - `class_filter` 非空 → 过滤 `class_name != filter`;
   - 只留 `status == "confirmed"`;
   - `p_base_in_odom` 为空 → `ok=false, reason="no_base_pose"`;
   - 取 `‖p_odom - p_base_in_odom‖` 最小者(`reason="nearest"`);空集 → `reason="no_candidate"`。
3. 命中后更新 `state.last_track_id / last_known_p_odom / last_class / has_last`。

- [ ] **Step 4: 运行,确认通过**

Run: `./build/lgw_bt_planner/test_target_selector`
Expected: 6 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp \
        src/lgw_bt_planner/src/target_selector.cpp \
        src/lgw_bt_planner/tests/test_target_selector.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add target selector module"
```

---

### Task 3: geometry_source 纯模块

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/geometry_source.hpp`
- Create: `src/lgw_bt_planner/src/geometry_source.cpp`
- Create: `src/lgw_bt_planner/tests/test_geometry_source.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`

**Interfaces:**
- Produces:
```cpp
struct Contributor {
  SourceKind kind{SourceKind::NONE};
  tf2::Vector3 p_arm_base;
  int64_t stamp_ns{0};
  float confidence{0.0f};
};
struct TrackSnapshot {
  uint64_t track_id{0};
  std::string status;
  tf2::Vector3 p_odom;
  std::vector<Contributor> contributors;
  int64_t last_obs_stamp_ns{0};
  int64_t time_since_last_obs_ms{0};
};
struct GeometrySourceParams {
  int64_t head_max_age_ms{400};
  int64_t arm_max_age_ms{400};
  int64_t odom_fallback_max_age_ms{1500};
  int upgrade_hysteresis_ticks{30};
  double head_scale{1.0};
  double arm_scale{0.7};
  double odom_scale{0.5};
};
struct SelectionState { SourceKind last{SourceKind::NONE}; int upgrade_ticks{0}; };
SourceSelection select_source(const TrackSnapshot& track, int64_t now_ns,
                              const std::optional<tf2::Transform>& T_base_odom,
                              const GeometrySourceParams& params,
                              SelectionState& state);
```

- [ ] **Step 1: 写失败测试**

```cpp
TEST(GeometrySource, HeadWinsWhenFresh) { /* head stamp = now-100ms → HEAD, scale 1.0 */ }
TEST(GeometrySource, HeadStaleFallsToArmImmediately) { /* head now-500ms, arm now-100ms → ARM, 0.7 */ }
TEST(GeometrySource, BothStaleFallsToOdom) {
  // T_base_odom = translate(1,2,3); p_odom=(4,5,6) → p_arm=(3,3,3), scale 0.5
}
TEST(GeometrySource, OdomExpiredReturnsNone) { /* time_since=1600ms → NONE */ }
TEST(GeometrySource, UpgradeNeedsHysteresis) {
  // 起始 ARM;head 连续 29 tick 有效仍为 ARM;第 30 tick → HEAD
}
TEST(GeometrySource, MissingTransformDisablesOdom) { /* std::nullopt, 两相机失效 → NONE */ }
```

- [ ] **Step 2: 运行,确认失败**

Run: `colcon build ... && ./build/lgw_bt_planner/test_geometry_source`
Expected: FAIL(未声明)

- [ ] **Step 3: 实现**

- 年龄一律 `now_ns - contributor.stamp_ns`(capture stamp),**不使用接收时刻**。
- 降级立即生效;升级只在候选源连续 `upgrade_hysteresis_ticks` 有效时发生。
- odom:`T_base_odom->inverse() * p_odom`;`T_base_odom` 为空或 `time_since_last_obs_ms > 1500` → `NONE`。
- 返回 `reason` 字符串,取值 `"head"` / `"arm"` / `"odom"` / `"stale"`。

- [ ] **Step 4: 运行,确认通过**

Run: `./build/lgw_bt_planner/test_geometry_source`
Expected: 6 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/geometry_source.hpp \
        src/lgw_bt_planner/src/geometry_source.cpp \
        src/lgw_bt_planner/tests/test_geometry_source.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add geometry source selection"
```

---

### Task 4: servo_control_law 纯模块

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/servo_control_law.hpp`
- Create: `src/lgw_bt_planner/src/servo_control_law.cpp`
- Create: `src/lgw_bt_planner/tests/test_servo_control_law.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`

**Interfaces:**
- Produces:
```cpp
struct ControlLawParams {
  double kp_pos{2.0}, kp_ang{3.0};
  double v_max{0.15}, w_max{0.6};
  double deadband_pos_m{0.01};
  double deadband_ang_rad{0.034906585};
  double lowpass_cutoff_hz{8.0};
  double loop_rate_hz{100.0};
};
struct ControlLawState {
  tf2::Vector3 v_filtered{0, 0, 0};
  tf2::Vector3 w_filtered{0, 0, 0};
};
struct TwistCommand { tf2::Vector3 v; tf2::Vector3 w; bool in_deadband{false}; };
TwistCommand compute_twist(const Pose6& desired, const Pose6& current,
                           const ControlLawParams& params,
                           ControlLawState& state, double source_scale);
```

- [ ] **Step 1: 写失败测试**

```cpp
TEST(ControlLaw, SaturatesAtMax) { /* 大误差 → |v|≈0.15, |w|≈0.6 */ }
TEST(ControlLaw, DeadbandOutputsZero) { /* 1° / 5mm → v=w=0, in_deadband */ }
TEST(ControlLaw, LowpassFirstStep) {
  // alpha = 1 - exp(-2π·8/100);首步输出 ≈ alpha·target
}
TEST(ControlLaw, SourceScaleApplied) { /* head=1.0 vs odom=0.5 → 输出减半 */ }
TEST(ControlLaw, AngularSignFollowsError) { /* desired 绕 z +0.1rad → w.z > 0 */ }
```

- [ ] **Step 2: 运行,确认失败**

Run: `colcon build ... && ./build/lgw_bt_planner/test_servo_control_law`
Expected: FAIL(未声明)

- [ ] **Step 3: 实现**

- 角误差:相对旋转 `R_err = R_des * R_cur^T`,转轴角;位置误差直接相减。
- `v = clamp(kp_pos·e_pos, v_max)·source_scale`,`w = clamp(kp_ang·e_ang, ω_max)·source_scale`。
- 一阶低通:`alpha = 1 - exp(-2π·fc/loop_rate)`;`filtered += alpha·(raw - filtered)`。
- 死区内把输出置零,但保留滤波状态(`in_deadband=true`)。

- [ ] **Step 4: 运行,确认通过**

Run: `./build/lgw_bt_planner/test_servo_control_law`
Expected: 5 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo_control_law.hpp \
        src/lgw_bt_planner/src/servo_control_law.cpp \
        src/lgw_bt_planner/tests/test_servo_control_law.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add servo control law"
```

---

### Task 5: servo_state_machine 纯模块 + 流看门狗

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp`
- Create: `src/lgw_bt_planner/src/servo_state_machine.cpp`
- Create: `src/lgw_bt_planner/tests/test_servo_state_machine.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`

**Interfaces:**
- Produces:
```cpp
enum class ServoEvent {
  TAKEOVER, SWITCH_IN_OK, SWITCH_IN_FAIL, RELEASE, SWITCH_OUT_OK,
  TARGET_LOST, TARGET_OK, PAUSE, RESUME, FAULT, RESET
};
class ServoStateMachine {
public:
  ServoState state() const;
  void set_move_group_busy(bool busy);
  bool handle(ServoEvent event);   // false = 事件被拒绝
private:
  ServoState state_{ServoState::IDLE};
  bool move_group_busy_{false};
};
class StreamWatchdog {
public:
  void on_track_array(int64_t now_ns);
  bool timed_out(int64_t now_ns, int64_t timeout_ms = 500) const;
};
```

- [ ] **Step 1: 写失败测试**

```cpp
TEST(StateMachine, HappyPath) { /* IDLE→PREEMPT→TRACKING→RELEASING→IDLE */ }
TEST(StateMachine, TakeoverRejectedWhenMoveGroupBusy) { /* set_move_group_busy(true) → handle=false, 仍 IDLE */ }
TEST(StateMachine, SwitchFailureGoesFault) { /* PREEMPT + SWITCH_IN_FAIL → FAULT */ }
TEST(StateMachine, HoldKeepsControlAndCanResume) { /* TRACKING→TARGET_LOST→HOLD→RESUME→TRACKING */ }
TEST(StateMachine, FaultNeedsReset) { /* FAULT + RELEASE → 仍 FAULT;FAULT + RESET → IDLE */ }
TEST(StreamWatchdog, FiresAfterTimeout) { /* on_track_array(0); timed_out(600ms)=true; timed_out(400ms)=false */ }
```

- [ ] **Step 2: 运行,确认失败**

Run: `colcon build ... && ./build/lgw_bt_planner/test_servo_state_machine`
Expected: FAIL(未声明)

- [ ] **Step 3: 实现**

状态迁移表严格按 spec §7.1;`handle` 对非法事件返回 `false` 且不改状态。
`StreamWatchdog` 记录最后一次 `UnifiedTrackArray` 到达的 `now_ns`,超时判定与 track 自带字段无关。

- [ ] **Step 4: 运行,确认通过**

Run: `./build/lgw_bt_planner/test_servo_state_machine`
Expected: 6 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp \
        src/lgw_bt_planner/src/servo_state_machine.cpp \
        src/lgw_bt_planner/tests/test_servo_state_machine.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add servo state machine and stream watchdog"
```

---

### Task 6: servo 参数 + 单栈 bringup launch

**Files:**
- Create: `src/lgw_bt_planner/config/servo_params.yaml`
- Create: `src/lgw_bt_planner/config/servo_node.yaml`
- Create: `src/lgw_bt_planner/launch/lgw_full.launch.py`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(install config)

**Interfaces:**
- Consumes: Task 1 的接口;spec §6.2 的参数值
- Produces: launch 入口 `ros2 launch lgw_bt_planner lgw_full.launch.py`

- [ ] **Step 1: 写 servo 参数**

`config/servo_params.yaml` 按 spec §6.2:
`command_out_type: trajectory_msgs/JointTrajectory`、
`command_out_topic: /arm_controller/joint_trajectory`、
`publish_joint_positions: true`、`publish_joint_velocities: true`、
`ee_frame_name: tcp_link`、`joint_topic: /feedback/joint_states`
(**只是默认值**;launch 按 `follow` 覆盖,见 Step 3)。

`config/servo_node.yaml` 暴露 `xoy_distance_m`、`z_fixed_m`、`ee_anchor`、
`arm_calibration_path`(空 = 默认 share 下 `calibration/arm_calibration.json`)、
`head_max_age_ms`、`arm_max_age_ms`、`odom_fallback_max_age_ms`、
`tracker_stream_timeout_ms`、`upgrade_hysteresis_ticks`、`Kp_*`、`v_max`、`ω_max`、
`min_observe_distance_m`、`frame_prefix`(默认 `arm1/`)、
`pose_watch_topic`(默认 `/control/joint_states`,由 launch 按 `follow` 覆盖)。

- [ ] **Step 2: 运行,确认 launch 还不存在**

Run: `ros2 launch lgw_bt_planner lgw_full.launch.py --show-args`
Expected: FAIL(找不到 launch)

- [ ] **Step 3: 写单栈 launch**

按 spec §5.2:
1. include `agx_arm_moveit/launch/demo.launch.py`,并把 `follow` 透传下去
   (真机 `true` → rsp 订阅 `feedback/joint_states`;台架 `false` → rsp 订阅 `control/joint_states`)
2. `moveit_servo/servo_node_main`(`servo_params.yaml` + `robot_description` /
   `robot_description_semantic` / `robot_description_kinematics`),
   并用参数字典覆盖 `joint_topic` = `follow ? /feedback/joint_states : /control/joint_states`
3. `lgw_bt_planner/servo_node`(`servo_node.yaml`,同样覆盖 `pose_watch_topic` 为同一个话题)
4. `apply_head_camera_collision_node`
5. include `lgw_bt_planner/launch/lgw_bt_planner.launch.py`(`bt_runner`)

**不新增 controller,不启动自己的 `ros2_control_node`** —— `demo.launch.py` 已经提供
rsp + move_group + JTC + broadcaster。
launch 参数:`arm_type`、`effector_type`、`revo2_type`、`tcp_offset`、`frame_prefix`、`follow`、`use_rviz`。
`tcp_offset` 默认 `[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]`,必须与 `agx_arm_ctrl` 的 `tcp_offset` 一致。

- [ ] **Step 4: 构建并确认 launch 参数可见**

Run:
```bash
colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON && \
source install/setup.bash && \
ros2 launch lgw_bt_planner lgw_full.launch.py --show-args
```
Expected: 列出 `arm_type` / `effector_type` / `tcp_offset` / `frame_prefix` 等参数

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/config/servo_params.yaml \
        src/lgw_bt_planner/config/servo_node.yaml \
        src/lgw_bt_planner/launch/lgw_full.launch.py \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add servo params and single-stack launch"
```

---

### Task 7: lgw_servo_node ROS 接口层

**Files:**
- Create: `src/lgw_bt_planner/src/servo_node.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(add_executable + install)

**Interfaces:**
- Consumes: Tasks 2、2A–5 的纯模块;Task 1 的 srv/msg
- Produces: 服务 `/lgw/servo/{takeover,release,pause,resume}`、话题 `/lgw/servo/state`、`/servo_node/delta_twist_cmds`

- [ ] **Step 1: 写节点骨架(先能启动)**

订阅:`/perception/tracks/unified`(`UnifiedTrackArray`)、`pose_watch_topic`(joint_states,只用收到时刻判活)、
`/servo_node/status`(`std_msgs/Int8`)、`/move_action/_action/status`(`action_msgs/GoalStatusArray`)、
`/arm_controller/follow_joint_trajectory/_action/status`(`action_msgs/GoalStatusArray`)。
tcp 位姿不订阅话题,用 `tf2_ros::Buffer` 查 `<frame_prefix>base_link ← <frame_prefix>tcp_link`。
发布:`/lgw/servo/state`、`/servo_node/delta_twist_cmds`。
客户端:`/servo_node/{start,pause,unpause}_servo`(不再需要 `switch_controller`)。
定时器:100Hz,独立 `MutuallyExclusive` callback group;订阅用 `Reentrant`;`MultiThreadedExecutor(2)`。

- [ ] **Step 2: 运行,确认节点可构建**

Run: `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON`
Expected: 编译通过(此时未实现逻辑)

- [ ] **Step 3: 实现 takeover / release / 控制循环**

- 构造时用 `parse_calibration_json` 加载 `arm_calibration.json`,得到 `R_cam_in_tcp` / `t_cam_in_tcp`;
  加载失败 → 构造即 `FAULT`,拒绝 takeover。
- 当前 link6 位姿 = `/feedback/tcp_pose` 左乘 `inv(T_link6_tcp)`(`tcp_offset_z_m` 沿 z)。
- 构造时用 `parse_calibration_json` 加载 `arm_calibration.json`,得到 `R_cam_in_tcp` / `t_cam_in_tcp`;
  加载失败 → 构造即 `FAULT`,拒绝 takeover。
- 当前 tcp 位姿 = TF `<prefix>base_link ← <prefix>tcp_link`(零超时 `can_transform` 先探测);
  查不到 → HOLD。**不要**再做任何 `link6 ↔ tcp` 换算。
- takeover:检查 `move_group_busy_`(见 Step 1 的两个 status);成功后 `unpause_servo`。
- release:`pause_servo`;Servo 停发;JTC 保持最后一点。
- 伺服中同时监控两个 status;任一出现活动 goal → `pause_servo` + HOLD。
- `start_servo` 在 `pose_watch_topic` 有数据后调用一次;
  **必须**从 `/servo_node/status` 或 `/arm_controller/joint_trajectory` 输出确认已启动,否则 `FAULT`。
- 目标选择:每收到一条 `UnifiedTrackArray` 调一次 `select_target()`;`p_base_in_odom`
  由 TF `odom ← <prefix>base_link` 取,查不到传 `nullopt`(自动模式会返回 `no_base_pose`)。
  选中后只把该 track 转成 `TrackSnapshot` 交给下一步。
- 控制循环:取最新 track → `select_source` → `compute_lookat_setpoint`(输出已是 tcp 位姿) →
  `compute_twist` → 发布 `TwistStamped`(`frame_id=base_link`)。
- 近距保护:每周期用**当前** tcp 位姿算相机位置
  `cam_now = tcp_now.p + tcp_now.R · t_cam_in_tcp`、`target_distance_m = ‖p_target − cam_now‖`;
  小于 `min_observe_distance_m` 置 `too_close=true`。不要用期望位姿算(恒为 standoff 0.4)。
- `StreamWatchdog` 超时 → `TARGET_LOST`。

- [ ] **Step 4: 台架验证 A1/A2**

Run(另开终端):
```bash
source /my_ws/l4t/l4t_setup.sh && source install/setup.bash
ros2 launch lgw_bt_planner lgw_full.launch.py \
  arm_type:=piper effector_type:=agx_gripper follow:=false use_rviz:=false
# 第二终端
ros2 topic hz /arm_controller/joint_trajectory
ros2 service call /lgw/servo/takeover lgw_bt_planner/srv/Takeover "{track_id: 0, class_filter: ''}"
ros2 topic hz /arm_controller/joint_trajectory
ros2 service call /lgw/servo/release std_srvs/srv/Trigger "{}"
```
Expected: takeover 后 `/arm_controller/joint_trajectory` 有稳定输出;release 后停止。
台架必须 `follow:=false`:`rsp` 订阅 `control/joint_states`,TF 由 mock broadcaster 驱动,
Servo 的 `joint_topic` 与节点 `pose_watch_topic` 都由 launch 自动切到同一话题(spec §10.2)。

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/src/servo_node.cpp src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw_bt_planner): add servo ROS node"
```

---

### Task 7A: 驱动模式评估(move_j vs move_js)

**目标**:在引入 CPV 之前,用数据判断 `move_j` / `move_js` 是否已经够用。
若够用,则不改驱动、不引入 CPV。

**Files:**
- Create: `scripts/driver_mode_eval.py`(测试脚本,发布 step twist 并记录反馈)
- Create: `docs/servo/2026-10-27-driver-mode-eval.md`(结果记录)

**Interfaces:**
- Consumes: `/servo_node/delta_twist_cmds`、`/feedback/tcp_pose`、`/feedback/joint_states`
- Produces: 两份 CSV 与控制台指标

- [ ] **Step 1: 写测试脚本**

`scripts/driver_mode_eval.py`:
- 发布 1s 恒定 twist(默认 `linear.x=0.02 m/s`),然后发零 twist;
- 以 200Hz 记录 `/feedback/tcp_pose` 与 `/feedback/joint_states` 到 CSV;
- 参数:`--linear`、`--angular`、`--duration`、`--out`。

- [ ] **Step 2: 运行 A(默认 `move_j`)**

启动伺服栈,`fast_mode` 保持 `false`;用保守参数
(`servo_velocity_scaling:=0.2`,以及节点侧 `v_max:=0.05`、`w_max:=0.2`)。
**不要用 `scale.linear`/`scale.rotational`** —— `command_in_type:=speed_units` 下它们不生效
(spec §6.2 的注记)。运行脚本,保存 `a_movej.csv`。

- [ ] **Step 3: 运行 B(`move_js`)**

**重启驱动**,传 `fast_mode:=true`;同一脚本、同一参数,保存 `b_movejs.csv`。
(`fast_mode` 无运行时切换,必须重启。)

- [ ] **Step 4: 计算指标**

上升时间(到 90%)、过冲(%)、稳态 RMS、最大关节速度、符号变化次数。

- [ ] **Step 5: 决策并记录**

- A 收敛 ≤2s、过冲 <10%、稳态 RMS <0.5° → 用 `move_j`,**不引入 CPV**;
- A 不满足、B 满足且无机械冲击 → 用 `move_js`,不引入 CPV;
- 两者都不满足 → 再评估 CPV(新增驱动补丁任务)。
把结论写入 `docs/servo/2026-10-27-driver-mode-eval.md`。

- [ ] **Step 6: Commit**

```bash
git add scripts/driver_mode_eval.py docs/servo/2026-10-27-driver-mode-eval.md
git commit -m "test(servo): evaluate move_j vs move_js driver modes"
```

---

### Task 8: start_lgw_full.sh(单栈 bringup)

**Files:**
- Create: `scripts/start_lgw_full.sh`

**Interfaces:**
- Consumes: Task 6 的 `lgw_full.launch.py`
- Produces: 一次启动整套框架的脚本(正常栈或伺服栈共用)

- [ ] **Step 1: 写脚本**

复用 `start_lgw_arm.sh` 的 Step 1(CAN)与 Step 2(driver),把 Step 3 改为
`ros2 launch lgw_bt_planner lgw_full.launch.py`(单栈:`demo.launch.py` + servo + bt_runner)。
增加:
- **互斥 preflight**:若容器内已有 `agx_arm_ctrl` / `move_group` / `controller_manager`
  进程 → 报错退出(避免两个驱动抢 `can0`)。
- **tracker precheck**:`ros2 topic echo /perception/tracks/unified --once --timeout 3` 失败 → 报错退出。
- **驱动模式**:通过环境变量 `DRIVER_MODE`(默认 `move_j`)传给 driver;`move_js` 时传 `fast_mode:=true`;
  将来 CPV 时传 `control_mode:=cpv`。
- **参数一致性**:`tcp_offset`(默认 `[0,0,0.12,0,0,0]`)必须同时传给 driver 与 `lgw_full.launch.py`;
  真机 `follow:=true`,台架 `follow:=false`。三者不一致不会报错,只会静默产生偏差。
- cleanup 只杀本脚本拉起的 PID。

- [ ] **Step 2: 运行,确认互斥生效**

Run: 先起 `start_lgw_arm.sh`,再跑 `bash scripts/start_lgw_full.sh`
Expected: 第二条命令报错退出,提示已有 driver/move_group

- [ ] **Step 3: 静态检查**

Run: `bash -n scripts/start_lgw_full.sh`
Expected: 无语法错误

- [ ] **Step 4: Commit**

```bash
git add scripts/start_lgw_full.sh
git commit -m "feat(scripts): add single-stack bringup script"
```

---

### Task 9: tracker 臂基座几何透传

**Files:**
- Modify: `src/yoloe_interfaces/msg/UnifiedTrack.msg`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py`
- Create: `src/lgw_3d_tracker/test/test_arm_base_geometry.py`

**Interfaces:**
- Produces: `UnifiedTrack.position_arm_base`、`geometry_frame`、`last_observation_stamp`、
  `contributor_cameras`、`contributor_positions_arm_base`、`contributor_stamps`、`contributor_confidences`

- [ ] **Step 1: 写失败测试**

```python
def test_head_observation_in_arm_base_ignores_odom():
    # head: p_armbase == lookup(arm1/base_link, camera_head_color_optical_frame) @ p_cam,
    #       与 T_base_odom 无关
def test_arm_observation_uses_camera_chain():
    # arm: p_armbase == lookup(arm1/base_link, camera_arm_color_optical_frame) @ p_cam
def test_contributors_preserve_per_camera_stamps():
    # 两相机各一条 → contributor_cameras 长度 2,stamps 各不相同
def test_odom_position_still_published():
    # position 语义不变
```

- [ ] **Step 2: 运行,确认失败**

Run:
```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -m pytest src/lgw_3d_tracker/test/test_arm_base_geometry.py -q"
```
Expected: FAIL(`position_arm_base` 不存在)

- [ ] **Step 3: 实现**

- `UnifiedTrack.msg` 按 spec §8.1 增加字段。
- `TrackObservation` 增加 `position_arm_base`(`np.ndarray (3,)`)。
- `Track` 增加 `last_obs_by_camera: Dict[str, TrackObservation]`,`apply_observation` 里按
  `obs.source_camera` 覆盖写入 —— 这是 `contributor_*` 的数据源,不能只留单个 `last_obs`。
- `input_adapter.adapt`:**两条链统一走 TF**(spec §8.2 / D23),不再用标定矩阵手算。
  - head:`p_armbase = lookup("arm1/base_link","camera_head_color_optical_frame", stamp) @ p_cam`。
  - arm:`p_armbase = lookup("arm1/base_link","camera_arm_color_optical_frame", stamp) @ p_cam`。
  - 两条链由 `broadcast_camera_mount_tf` 的安装桥接通;查不到 → `position_arm_base=None`。
  - `InputAdapter` 不再需要 `head_cam_to_arm_base` / `arm_cam_to_tcp` 参数(桥仍然要用它们)。
- `output_formatter`:
  - `position_arm_base` 取最近一次观测的值;`geometry_frame = "arm1/base_link"`。
  - `last_observation_stamp` = 最近一次观测的 stamp。
  - `contributor_*` 由 `last_obs_by_camera` 逐相机生成(顺序按 camera 名排序,保证稳定)。

- [ ] **Step 4: 构建并运行测试**

Run:
```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && \
   colcon build --packages-select yoloe_interfaces lgw_3d_tracker && \
   source install/setup.bash && \
   python3 -m pytest src/lgw_3d_tracker/test/test_arm_base_geometry.py -q"
```
Expected: 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/yoloe_interfaces/msg/UnifiedTrack.msg \
        src/lgw_3d_tracker/lgw_3d_tracker/unified_track/ \
        src/lgw_3d_tracker/test/test_arm_base_geometry.py
git commit -m "feat(lgw_3d_tracker): publish arm-base geometry for servo"
```

---

### Task 10: 台架与真机验收

**Files:**
- Create: `scripts/fake_track_publisher.py`(台架用,15Hz 静态 track)
- 产出验收记录到 `docs/servo/2026-10-26-servo-acceptance.md`

**Interfaces:**
- Consumes: Tasks 1–9

- [ ] **Step 1: 台架 A1/A2**

台架没有 tracker,A1 的"takeover 后有 Servo 输出"要求节点处于 `TRACKING`,
因此先起一个静态假 track:

```bash
# 15Hz /perception/tracks/unified,一条 confirmed track,
# position_arm_base 固定 (0.5, 0.0, 0.3),geometry_frame=arm1/base_link
python3 scripts/fake_track_publisher.py --x 0.5 --y 0.0 --z 0.3
```

脚本按 Task 9 之后的 `UnifiedTrack` 字段填:`track_id`、`status="confirmed"`、
`class_name`、`position`(odom 占位)、`position_arm_base`、`geometry_frame`、
`last_observation_stamp`、`contributor_*`;`header.stamp` 与 `wall_stamp` 都用当前时间。

然后按 Task 7 Step 4 执行 20 次 takeover/release。
Expected: 20/20 成功,无崩溃;takeover 后有稳定输出,release 后停止。

- [ ] **Step 2: 真机 A3(静止目标、head 源)**

启动 `start_lgw_yoloe.sh` + tracker + `start_lgw_full.sh`;指定 `track_id`;记录
`/lgw/servo/state` 的 `angular_error_rad` 与 `geometry_source`。
Expected: 10s 内 `angular_error_rad < 0.0349`, `geometry_source == "head"`。

- [ ] **Step 3: 真机 A4(遮挡 head)**

挡住 head 相机 2s。Expected: 1 个控制周期内 `geometry_source == "arm"`;恢复后 ≤300ms 回到 `head`。

- [ ] **Step 4: 真机 A5(双相机失效)**

挡住两台相机。Expected: `geometry_source == "odom"`;超过 1500ms 后进入 `HOLD`。

- [ ] **Step 5: 真机 A6(释放连续性)**

release 后执行一次 `MoveArmToPose`;采样 `/feedback/joint_states`。
Expected: 相邻 5ms 单关节变化 < 0.02 rad。JTC 最后指令 = Servo 最后一点,理论上无跳变。

- [ ] **Step 6: Commit 验收记录**

```bash
git add docs/servo/2026-10-26-servo-acceptance.md
git commit -m "docs(servo): record bench and on-robot acceptance"
```

---

## Self-Review

- **Spec coverage**: §5 接缝/接管 → Task 6/7;§6 Servo → Task 6;§7.1 状态机 → Task 5;§7.2 接口 → Task 1/7;
  §7.3 选择 → Task 2A;§7.4 几何源 → Task 3;§7.5 lookat → Task 2;§7.6 控制律 → Task 4;
  §7.8 错误 → Task 5/7;§7.9 生命周期 → Task 7;§8 tracker → Task 9;§10 测试 → 各 Task + Task 10;
  §5.4 驱动模式 → Task 7A。
- **Type consistency**: `Pose6`、`SourceKind`、`SourceSelection`、`TrackSnapshot`、`TrackCandidate`、
  `TargetSelection`、`ControlLawParams`、`ServoEvent`、`StreamWatchdog` 在 Task 1/2A/3/4/5 定义,
  后续 Task 引用同名。
- **Review Focus 对应测试**:年龄基准 → Task 3 `HeadStaleFallsToArmImmediately`;
  `start_servo` 假成功 → Task 7 Step 3/4;参考点必须是 `tcp_link` → Task 2 `TcpPoseIsTheSetpointItself`;
  odom TF 缺失 → Task 3 `MissingTransformDisablesOdom`;tracker 断流 → Task 5 `StreamWatchdog.FiresAfterTimeout`。
