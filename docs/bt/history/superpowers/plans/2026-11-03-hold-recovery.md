# HOLD 内自恢复(RECOVERING)实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** HOLD 内按判据自动调用 MoveIt 摆位并恢复 TRACKING,带 `RECOVERING` 状态、有界预算、可取消、可观测。

**Architecture:** 在现有 `ServoStateMachine` 增加 `RECOVERING` 顶层状态;恢复判据抽成纯函数模块;节点用 worker 线程执行 `posture_.compute` → `decide_preposition` → `MoveGroupPrepositioner::move_to` → 复核 → `start_servo` 重新播种 → `unpause`。`follower_lag` 走"冻结等待 gap 闭合",永不移动。

**Tech Stack:** C++17 / rclcpp / MoveIt 2.5.9 / gtest / rclpy bench 脚本。

**Spec:** `docs/superpowers/specs/2026-11-03-hold-recovery-design.md`

## Global Constraints

- 宿主机编辑;build/test/launch 一律 `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t`。
- 每条容器命令先 `source /my_ws/l4t/.l4t_env.sh`(4 ms);只有装新包才 `scripts/lgw_env.sh --refresh`。
- 不用 `colcon test`;直接跑 `build/lgw_bt_planner/test_*` 二进制,**必须先 source 环境**。
- 禁止 `pkill -f ros` / `killall`;清理按 PID `kill -15`。
- 所有 mock bench 必须先确认 `agx_arm_ctrl_single` 未运行(真机安全)。
- 不修改 `moveit_servo`;不新增第三方依赖。
- 新 yaml 参数只放在 `lgw_servo_node: ros__parameters` 下,与现有键同级。
- `hold_recovery_enable` 默认 **`true`**(2026-11-03 用户决定);置 `false` 时行为与改动前完全一致。
- 恢复移动假设平台静止或慢速。行走中恢复不在本计划内,但 `HoldRecoveryInput` 是
  唯一扩展点(见 spec §14.1),不要在本计划里加未使用的字段。
- 提交信息用中文或英文均可,但必须能独立回滚。

## Review Focus

- **RECOVERING 中 release**:必须在 `unpause` 之前取消 MoveGroup goal;否则 Servo 与 MoveGroup 会同时成为 JTC 写者,且 release 后臂继续运动。
- **`start_servo` 的 success 不可信**:内部 `waitForCompleteState` 超时也返回成功。恢复后必须确认 Servo 真的重新播种,否则会回到"首拍零指令复播旧姿态"。
- **恢复过程中目标消失**:决策时目标可用、执行完不可用,必须落到 `HOLD(exhausted)`,绝不能进 TRACKING。
- **MoveGroup busy 竞态**:外部 BT 在判据检查之后发起 goal,节点不得再发第二个 goal。
- **hold_reason 误判**:同一拍多个标志为真时的优先级必须固定并测试;`follower_lag` 被误判成姿态问题会触发一次无用的摆位。
- **resume 不移动时也不能跳**:未发生 MoveGroup 移动的 resume 不应 reseed 出一次姿态跳变。

---

### Task 1: `RECOVERING` 状态与状态机迁移

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp`(`ServoState` enum,约 41 行)
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp`(`ServoEvent`、`target_ok()`、迁移表注释)
- Modify: `src/lgw_bt_planner/src/servo_state_machine.cpp`(`handle()`)
- Modify: `src/lgw_bt_planner/src/servo_node.cpp`(`state_name()` 约 190 行)
- Modify: `src/lgw_bt_planner/msg/ServoState.msg`(state 取值注释)
- Modify: `docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md` §7.1 迁移表
- Test: `src/lgw_bt_planner/tests/test_servo_state_machine.cpp`

**Interfaces:**
- Produces: `ServoState::RECOVERING`
- Produces: `ServoEvent::RECOVER_START`, `ServoEvent::RECOVER_OK`, `ServoEvent::RECOVER_FAIL`
- Produces: `bool ServoStateMachine::target_ok() const`(供 `resumable` 字段)
- Consumes: 无

- [ ] **Step 1: 写失败的迁移测试**

在 `tests/test_servo_state_machine.cpp` 增加(命名与现有 TEST 风格一致):

```cpp
TEST(ServoStateMachine, RecoverStartFromHold) {
  ServoStateMachine sm;
  sm.handle(ServoEvent::TAKEOVER);
  sm.handle(ServoEvent::TARGET_LOST);      // PREEMPT 收到 TARGET_LOST 被拒
  sm.handle(ServoEvent::UNPAUSE_OK);       // -> TRACKING
  sm.handle(ServoEvent::TARGET_LOST);      // -> HOLD
  EXPECT_TRUE(sm.handle(ServoEvent::RECOVER_START));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}

TEST(ServoStateMachine, RecoverOkNeedsFreshTarget) {
  // HOLD -> RECOVERING -> TARGET_OK -> RECOVER_OK => TRACKING
  // HOLD -> RECOVERING -> RECOVER_OK(无 TARGET_OK) => HOLD
}

TEST(ServoStateMachine, RecoverFailReturnsToHold) { /* -> HOLD */ }
TEST(ServoStateMachine, ResumeRefusedWhileRecovering) {
  EXPECT_FALSE(sm.handle(ServoEvent::RESUME));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
}
TEST(ServoStateMachine, ReleaseFromRecovering) {
  EXPECT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);
}
TEST(ServoStateMachine, TargetOkArmsLatchButStaysRecovering) {
  EXPECT_TRUE(sm.handle(ServoEvent::TARGET_OK));
  EXPECT_EQ(sm.state(), ServoState::RECOVERING);
  EXPECT_TRUE(sm.target_ok());
}
TEST(ServoStateMachine, TargetLostInRecoveringClearsLatch) {
  /* TARGET_OK -> TARGET_LOST -> !target_ok(), state unchanged */
}
TEST(ServoStateMachine, TakeoverRefusedWhileRecovering) { /* false */ }
TEST(ServoStateMachine, FaultFromRecovering) { /* -> FAULT */ }
```

- [ ] **Step 2: 运行确认失败**

Run (容器): `source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner --symlink-install && /my_ws/l4t/lg_walker_ws/build/lgw_bt_planner/test_servo_state_machine`
Expected: 编译失败(`RECOVERING` / `RECOVER_START` 未定义)

- [ ] **Step 3: 实现枚举、事件与迁移**

`servo_types.hpp`:`enum class ServoState { IDLE, PREEMPT, TRACKING, HOLD, RECOVERING, RELEASING, FAULT };`

`servo_state_machine.hpp`:事件枚举加三个值;加 `bool target_ok() const { return target_ok_; }`;把 §7.1 注释表补上 §6.1 的 11 条迁移。

`servo_state_machine.cpp`:

- `TARGET_LOST` 的 switch 增加 `case ServoState::RECOVERING: return true;`(只清 latch)。
- `RELEASE` 的 switch 增加 `case ServoState::RECOVERING:` 与 PREEMPT/TRACKING/HOLD 同组 → RELEASING。
- `case ServoState::HOLD:` 增加 `if (event == ServoEvent::RECOVER_START) { state_ = RECOVERING; return true; }`。
- 新增 `case ServoState::RECOVERING:`:
  - `TARGET_OK` → `target_ok_ = true; return true;`
  - `PAUSE` → `return true;`(accepted no-op)
  - `RECOVER_OK` → `if (target_ok_) { state_ = TRACKING; target_ok_ = false; } else { state_ = HOLD; } return true;`
  - `RECOVER_FAIL` → `state_ = HOLD; return true;`
  - 其余(含 `RESUME`、`TAKEOVER`)→ `return false;`

`servo_node.cpp` 的 `state_name()` 增加 `case ServoState::RECOVERING: return "RECOVERING";`。

`ServoState.msg` 的 state 注释增加 `RECOVERING`。

`2026-10-26-lgw-servo-takeover-design.md` §7.1 的迁移表加入 §6.1 的行。

- [ ] **Step 4: 运行确认通过**

Run: 同上 build + 直接跑 `test_servo_state_machine`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp \
        src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp \
        src/lgw_bt_planner/src/servo_state_machine.cpp \
        src/lgw_bt_planner/src/servo_node.cpp \
        src/lgw_bt_planner/msg/ServoState.msg \
        src/lgw_bt_planner/tests/test_servo_state_machine.cpp \
        docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md
git commit -m "feat(lgw): add RECOVERING state to the servo state machine"
```

---

### Task 2: 纯判据模块 `hold_recovery_decision`

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/hold_recovery_decision.hpp`
- Create: `src/lgw_bt_planner/src/hold_recovery_decision.cpp`
- Create: `src/lgw_bt_planner/tests/test_hold_recovery_decision.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(源码加入 `lgw_bt_planner_nodes`;新增 gtest target,仿 `test_preposition_decision` 的写法)

**Interfaces:**
- Produces: `enum class HoldReason { kNone, kTargetLost, kTrackerStale, kPoseStale, kMoveGroupBusy, kServoHalt, kSaturated, kFollowerLag, kOperatorPause };`
- Produces: `const char * hold_reason_name(HoldReason);`
- Produces: `enum class RecoveryAction { kNone, kMove, kWaitFollower, kExhausted };`
- Produces: `struct HoldRecoveryParams { bool enable{false}; bool allow_target_lost{false}; int max_attempts{1}; int64_t deadline_ms{8000}; int64_t geometry_max_age_ms{1500}; int64_t follower_timeout_ms{3000}; };`
- Produces: `struct HoldRecoveryInput { HoldReason reason; int64_t hold_since_ns; int64_t now_ns; int attempts; bool move_group_busy; bool measured_valid; bool tcp_valid; bool geometry_available; int64_t geometry_age_ms; };`
  - 将来"行走中恢复"只会给这个结构加字段(如 `platform_motion_ok`),不改
    `decide_hold_recovery()` 的签名;本计划**不加**该字段(spec §14.1)。
- Produces: `RecoveryAction decide_hold_recovery(const HoldRecoveryInput &, const HoldRecoveryParams &);`
- Produces: `HoldReason classify_hold(bool target_ok, bool stream_alive, bool move_group_busy, bool servo_halted, bool pose_alive, bool not_following);`
- Produces: `bool follower_recovery_ready(double gap_rad, int64_t gap_ok_since_ns, int64_t now_ns, double gap_threshold_rad, int64_t stable_ms);`
- Consumes: 无(纯 C++17,无 ROS)

- [ ] **Step 1: 写失败测试**

测试断言(每条一个 TEST):

| 测试 | 输入 | 期望 |
|---|---|---|
| `DisabledAlwaysNone` | `enable=false`, reason=servo_halt | `kNone` |
| `FollowerLagWaits` | reason=follower_lag, 任意预算 | `kWaitFollower` |
| `PoseStaleNone` | reason=pose_stale | `kNone`(不消耗预算,姿态恢复后可重试) |
| `MoveGroupBusyNone` | reason=servo_halt, busy=true | `kNone` |
| `OperatorPauseNone` | reason=operator_pause | `kNone` |
| `ServoHaltMoves` | reason=servo_halt, 全部前置满足 | `kMove` |
| `SaturatedMoves` | reason=saturated | `kMove` |
| `TargetLostRequiresFlag` | reason=target_lost, allow=false | `kNone` |
| `TargetLostStaleGeometryExhausted` | allow=true, geometry_age=2000, max=1500 | `kExhausted` |
| `TargetLostMovesWhenFresh` | allow=true, age=500, available | `kMove` |
| `AttemptsExhausted` | attempts=1, max=1 | `kExhausted` |
| `DeadlineExceeded` | now-hold_since=9000ms, deadline=8000 | `kExhausted` |
| `NoMeasuredStateNone` | measured_valid=false | `kNone` |
| `ClassifyPriority` | 多个标志同时为真 | 按 target_lost/tracker_stale → busy → servo_halt → pose_stale → follower_lag 的顺序 |
| `FollowerReadyAfterStableWindow` | gap=0.05, since=0, stable=300ms, now=400ms | true |
| `FollowerNotReadyBelowWindow` | now=200ms | false |

- [ ] **Step 2: 运行确认失败**

Run: `colcon build --packages-select lgw_bt_planner --symlink-install && build/lgw_bt_planner/test_hold_recovery_decision`
Expected: 编译失败(头文件不存在)

- [ ] **Step 3: 实现头文件与实现**

头文件只放上表签名的声明与枚举;`.cpp` 实现优先级:

1. `!enable` → `kNone`
2. `reason == kFollowerLag` → `kWaitFollower`
3. `reason ∈ {kPoseStale, kMoveGroupBusy, kOperatorPause, kNone}` → `kNone`
4. `!measured_valid || !tcp_valid || move_group_busy` → `kNone`
5. `attempts >= max_attempts` 或 `now - hold_since_ns > deadline_ms*1e6` → `kExhausted`
6. `target_lost/tracker_stale`:若 `!allow_target_lost` → `kNone`;若 `!geometry_available` 或 `geometry_age_ms > geometry_max_age_ms` → `kExhausted`
7. 其余(servo_halt/saturated)→ `kMove`

`classify_hold` 的优先级与 `servo_node.cpp:866` 的条件顺序一致(见上表)。  
`follower_recovery_ready`: `gap_rad <= gap_threshold_rad && gap_ok_since_ns != 0 && now_ns - gap_ok_since_ns >= stable_ms*1e6`。

CMake:把 `src/hold_recovery_decision.cpp` 加到 `lgw_bt_planner_nodes` 的源列表;新增:

```cmake
ament_add_gtest(test_hold_recovery_decision
  tests/test_hold_recovery_decision.cpp
)
target_link_libraries(test_hold_recovery_decision lgw_bt_planner_nodes)
```

- [ ] **Step 4: 运行确认通过**

Run: 同上
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/hold_recovery_decision.hpp \
        src/lgw_bt_planner/src/hold_recovery_decision.cpp \
        src/lgw_bt_planner/tests/test_hold_recovery_decision.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw): pure HOLD recovery decision module"
```

---

### Task 3: 记录并导出 `hold_reason` / 恢复字段

**Files:**
- Modify: `src/lgw_bt_planner/msg/ServoState.msg`
- Modify: `src/lgw_bt_planner/src/servo_node.cpp`(`update_tracking_locked` 的 HOLD 触发点、saturated 分支、`handle_pause`、`clear_session_locked`、`publish_state_locked`)
- Test: 无新纯逻辑;验证用 bench 状态下发

**Interfaces:**
- Consumes: `classify_hold()`、`hold_reason_name()`(Task 2)
- Produces: `ServoState` 消息字段 `hold_reason`、`recovery_attempts`、`recovery_exhausted`、`resumable`、`hold_since`

- [ ] **Step 1: 扩展消息**

`msg/ServoState.msg` 追加:

```
string hold_reason
uint32 recovery_attempts
bool recovery_exhausted
bool resumable
builtin_interfaces/Time hold_since
```

- [ ] **Step 2: 节点记录原因**

在 `update_tracking_locked` 进入 HOLD 的分支里,用 `classify_hold(target_ok, stream_alive, busy, servo_halted, pose_alive, not_following)` 填 `hold_reason_`,并设 `hold_since_ns_ = now_ns`、`recovery_attempts_ = 0`、`recovery_exhausted_ = false`。  
`servo_halt` 优先于其他条件由 `classify_hold` 的顺序决定;`saturated` 在自己的分支里显式设 `hold_reason_ = HoldReason::kSaturated`。  
`handle_pause` 设 `kOperatorPause`。  
`clear_session_locked()` 重置为 `kNone`、`0`、`false`。

`publish_state_locked` 填:

```cpp
message.hold_reason = hold_reason_name(hold_reason_);
message.recovery_attempts = static_cast<uint32_t>(recovery_attempts_);
message.recovery_exhausted = recovery_exhausted_;
message.resumable = sm_.target_ok();
message.hold_since = ...;   // ns -> builtin_interfaces::msg::Time
```

- [ ] **Step 3: 构建并验证消息**

Run: `colcon build --packages-select lgw_bt_planner --symlink-install`
Expected: 编译通过。  
Run(容器): `ros2 interface show lgw_bt_planner/msg/ServoState`
Expected: 输出包含 5 个新字段。

- [ ] **Step 4: bench 观察一次 HOLD 的原因**

启动 mock 栈 + fake track,执行 takeover,停掉 fake track,`ros2 topic echo /lgw/servo/state`:
Expected: `state: HOLD` 且 `hold_reason` 为 `tracker_stale` 或 `target_lost`,`resumable: false`。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_bt_planner/msg/ServoState.msg src/lgw_bt_planner/src/servo_node.cpp
git commit -m "feat(lgw): report hold reason and recovery state"
```

---

### Task 4: `handle_resume` 重新播种 Servo 缓存(前置项)

**Files:**
- Modify: `src/lgw_bt_planner/src/servo_node.cpp`(`handle_resume`,约 1580-1620 行)
- Create: `scripts/check_servo_resume_reseed.py`

**Interfaces:**
- Consumes: `call_trigger_sync()`、`reseed_client_`、`kReseedTimeoutSec`(2026-11-02 已有)
- Produces: `/lgw/servo/resume` 在 unpause 前重新播种 `last_sent_command_`

- [ ] **Step 1: 先写验证脚本(预期失败)**

`scripts/check_servo_resume_reseed.py`(容器内运行):

1. 订阅 `/control/joint_states`、`/arm_controller/joint_trajectory`;
2. 读当前 `q0`;
3. 调 `/lgw/servo/pause`(或 `/servo_node/pause_servo`),确认 state=HOLD;
4. 用 `rclpy.action.ActionClient` 给 `/move_action` 发一个关节目标 `q1 = q0 + [0, +0.15, 0, 0, 0, 0]`(velocity scaling 0.1);
5. 调 `/lgw/servo/resume`;
6. 发零 twist(带当前 stamp)2 s,抓首条输出;
7. 断言 `max|first_cmd - q1| <= 0.05`,否则打印 FAIL。

- [ ] **Step 2: 运行确认失败**

前置:mock 栈已起、无真机驱动、`/lgw/servo/takeover` 已成功。
Run(容器): `source /my_ws/l4t/.l4t_env.sh && python3 scripts/check_servo_resume_reseed.py`
Expected: FAIL,首条输出回到 `q0` 附近的旧姿态(差值 > 0.1 rad)。

- [ ] **Step 3: 在 `handle_resume` 的 unpause 之前调用 reseed**

复用 `handle_takeover` 的写法:状态机接受 RESUME 之后、`call_trigger_sync(unpause_client_, ...)` 之前:

```cpp
const TriggerOutcome reseed =
  call_trigger_sync(reseed_client_, "start_servo(resume)", kReseedTimeoutSec);
if (!reseed.ok) {
  call_trigger_sync(pause_client_, "pause_servo", kPauseTimeoutSec);
  // 状态机 -> FAULT,返回失败
}
```

注意:`Servo::start()` 会先 unpause;失败路径必须重新 pause。

- [ ] **Step 4: 重跑验证**

Run: 同 Step 2
Expected: PASS,`max|first_cmd - q1| <= 0.05`。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_bt_planner/src/servo_node.cpp scripts/check_servo_resume_reseed.py
git commit -m "fix(lgw): re-seed Servo cache before resume after HOLD"
```

---

### Task 5: RECOVERING worker 与摆位恢复

**Files:**
- Modify: `src/lgw_bt_planner/src/servo_node.cpp`
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`
- Test: `scripts/bench_hold_recovery.sh`(Task 7 完成端到端;本任务先手工验证)

**Interfaces:**
- Consumes: Task 1 的 `RECOVER_START/RECOVER_OK/RECOVER_FAIL`、Task 2 的判据、`posture_`、`prepositioner_`、`preposition_in_progress_`、`call_trigger_sync()`、`reseed_client_`
- Produces: 参数 `hold_recovery_enable`、`recovery_reasons`、`recovery_allow_target_lost`、`recovery_max_attempts`、`recovery_deadline_ms`、`recovery_geometry_max_age_ms`、`recovery_velocity_scaling`、`recovery_timeout_s`、`recovery_tolerance_rad`、`recovery_settle_ms`

- [ ] **Step 1: 声明参数与状态成员**

yaml 与 `declare_parameter` 同名(默认值见 spec §8)。成员:

```cpp
HoldReason hold_reason_{HoldReason::kNone};
int64_t hold_since_ns_{0};
int recovery_attempts_{0};
bool recovery_exhausted_{false};
bool recovery_in_progress_{false};
bool recovery_cancel_{false};
int64_t recovery_unpause_ns_{0};
bool recovery_verify_pending_{false};
std::vector<double> recovery_expected_q_;
std::thread recovery_thread_;
```

- [ ] **Step 2: on_timer 中做判据与启动**

HOLD 分支(`else { if (target_ok) TARGET_OK; return false; }`)改为:

1. 计算 `HoldRecoveryInput`(geometry 优先取当前 source,其次最后已知快照;`geometry_age_ms` 取快照的 `time_since_last_obs_ms`);
2. `decide_hold_recovery()`:
   - `kNone` → 返回;
   - `kWaitFollower` → 进入 Task 6 的等待逻辑;
   - `kExhausted` → `recovery_exhausted_ = true`;
   - `kMove` → `sm_.handle(RECOVER_START)`;快照 `q_now/tcp_now/target`;`recovery_in_progress_=true`;启动 `recovery_thread_`(只启动一次)。
3. 线程内容在锁外执行 `posture_.compute` + `decide_preposition`:
   - `family.ok==false` 或 `decision.needed==false` → 前者 `RECOVER_FAIL`,后者直接跳到 reseed;
   - `prepositioner_->move_to(family.joints, names, recovery_velocity_scaling_, recovery_tolerance_rad_, timeout)`;
   - 成功:settle `recovery_settle_ms`;复核 `measured_positions_` 与 `family.joints` 的 max delta;失败 → `RECOVER_FAIL`;
   - `call_trigger_sync(reseed_client_, "start_servo(recovery)", kReseedTimeoutSec)`;失败 → pause + `RECOVER_FAIL` + 置 `recovery_fatal_`;
   - `call_trigger_sync(unpause_client_, "unpause_servo", kUnpauseTimeoutSec)`;失败 → FAULT;
   - 置 `recovery_verify_pending_ = true`、`recovery_expected_q_ = measured_positions_`、`recovery_unpause_ns_ = now`,然后 `sm_.handle(RECOVER_OK)`。

- [ ] **Step 3: 首条输出校验(缓存播种是否真的成功)**

不能直接比较"解停后的第一条输出"与 `measured_positions_`:控制律第一拍若在死区外,
输出 = 当前姿态 + delta,会被误判。复用 boot 路径的 CONFIRMING 做法,在
**仍处于 HOLD、尚未 `RECOVER_OK`** 时主动探一次:

1. worker 在 `start_servo` 成功返回后(此时 Servo 已 unpaused,但控制律还没开始发 twist),
   在 `mutex_` 内 `publish_twist_locked(0,0,0,0,0,0, now)`;
2. `on_servo_output()` 在 `mutex_` 内把 `points[0].positions` 存到 `recovery_probe_q_`,
   并更新 `servo_output_recv_ns_`;
3. worker 每 10 ms 轮询 `servo_output_recv_ns_` 是否前进,最多 300 ms;
4. 有输出 → 比较 `recovery_probe_q_` 与探针时刻的 `measured_positions_`,max delta > 0.10 rad
   → `pause_servo` + FAULT;无输出 → 同样 FAULT;
5. 通过后才 `sm_.handle(RECOVER_OK)`。

这样比较的是"零指令复播的缓存",与死区无关,能直接抓住"reseed 没生效"。

- [ ] **Step 4: 取消路径**

`handle_release()` 已有 `preposition_in_progress_` + `prepositioner_->cancel()`;增加 `recovery_cancel_ = true`,worker 在 move 返回后若发现 `recovery_cancel_` 则不 reseed/unpause,线程退出并等待 join。  
`RECOVERING` 的 RELEASE 由 Task 1 的状态机转到 RELEASING。

线程生命周期:worker 启动前必须确认没有存量线程(`recovery_thread_.joinable()` 则先 join);
节点析构、`clear_session_locked()`、进入 FAULT 时都要置 `recovery_cancel_` 并 join,
避免 detached 线程在节点销毁后访问成员。

- [ ] **Step 5: 构建与手工验证**

Run: `colcon build --packages-select lgw_bt_planner --symlink-install`
手工:mock 栈 → takeover → `ros2 param set /lgw_servo_node hold_recovery_enable true` → 停 fake track。
Expected: `/lgw/servo/state` 出现 `RECOVERING`,mock 关节被 MoveGroup 移动一次,随后 `HOLD` 且 `recovery_exhausted=true`(目标仍丢失)。

- [ ] **Step 6: 提交**

```bash
git add src/lgw_bt_planner/src/servo_node.cpp src/lgw_bt_planner/config/servo_node.yaml
git commit -m "feat(lgw): bounded MoveIt recovery from HOLD"
```

---

### Task 6: `follower_lag` 冻结等待与自动 resume

**Files:**
- Modify: `src/lgw_bt_planner/src/servo_node.cpp`
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`
- Test: Task 2 的 `follower_recovery_ready` 单测;真机验证见 Task 8

**Interfaces:**
- Consumes: `follow_gap_locked()`、`follower_recovery_ready()`、`follow_resume_gap_rad`、`follow_resume_stable_ms`、`follow_recovery_timeout_ms`
- Produces: HOLD(follower_lag) 在 gap 闭合后自动 `unpause`,超时 FAULT

- [ ] **Step 1: 参数**

yaml 与 `declare_parameter` 增加:

```
follow_resume_gap_rad: 0.10
follow_resume_stable_ms: 300
follow_recovery_timeout_ms: 3000
```

- [ ] **Step 2: 实现等待分支**

HOLD 且 `hold_reason_ == kFollowerLag` 时(每 tick):

1. `gap = follow_gap_locked()`;
2. gap ≤ `follow_resume_gap_rad` 时,若 `follow_resume_since_ns_ == 0` 则置 now;否则若 `follower_recovery_ready(...)` 为真 → `call_trigger_sync(unpause_client_, ...)`(在锁外),成功则 `sm_.handle(ServoEvent::RESUME)`(需先 `TARGET_OK` 已置位),失败 → FAULT;
3. gap 超阈值则清零 `follow_resume_since_ns_`;
4. `now - hold_since_ns_ > follow_recovery_timeout_ms` → FAULT(不是慢,是卡死)。

注意:

- resume 前必须确认 `sm_.target_ok()`(目标当前可用),否则继续等待并让超时接管;
- 本路径**不调用 MoveGroup、不 reseed**(命令未变,缓存仍有效);若未来这里也发生移动,必须补 reseed。
- `call_trigger_sync(unpause_client_, ...)` 阻塞 ≤1 s,必须在 `mutex_` 外执行(与现有 `pause_servo` 调用同一约定)。

- [ ] **Step 3: 构建**

Run: `colcon build --packages-select lgw_bt_planner --symlink-install`
Expected: 编译通过;`test_hold_recovery_decision` 仍 PASS。

- [ ] **Step 4: 提交**

```bash
git add src/lgw_bt_planner/src/servo_node.cpp src/lgw_bt_planner/config/servo_node.yaml
git commit -m "feat(lgw): auto-resume after follower gap closes"
```

---

### Task 7: 端到端 bench `scripts/bench_hold_recovery.sh`

**Files:**
- Create: `scripts/bench_hold_recovery.sh`

**Interfaces:**
- Consumes: 全部 Task 1–6
- Produces: 可重复验证 `target_lost` 恢复、release 打断恢复

- [ ] **Step 1: 写脚本**

脚本结构复用 `bench_posture_lookat.sh`:

1. 拒绝在 `agx_arm_ctrl_single` 运行时启动;
2. 起 mock 栈(`follow:=false use_bt:=false lookat_mode:=pinned_posture`);
3. 起 fake track + 两个 static TF;mock 摆到 `0.56 0.96 -0.48 -1.12 0.60 1.38`;
4. takeover;
5. `ros2 param set /lgw_servo_node hold_recovery_enable true` +
   `recovery_allow_target_lost true`;
6. 记录 `/lgw/servo/state` 到文件,等待 2 s 后 **杀掉 fake track**;
7. 等待 6 s,断言状态序列包含 `HOLD -> RECOVERING -> HOLD`,`recovery_exhausted` 最终为 true,且 `/control/joint_states` 在 RECOVERING 期间发生过变化;
8. 第二场景:恢复移动进行中调用 `/lgw/servo/release`,断言 goal 被取消、状态到 IDLE、Servo 保持 paused;
9. 按 PID 清理(不用 pkill)。

- [ ] **Step 2: 先运行,确认失败(实现前)**

Run: `bash scripts/bench_hold_recovery.sh`
Expected(在 Task 5 之前):没有 `RECOVERING`,脚本在状态序列断言处失败。

- [ ] **Step 3: 在 Task 5/6 完成后重跑**

Expected: 两个场景都 PASS,输出 `BENCH PASS`。

- [ ] **Step 4: 提交**

```bash
git add scripts/bench_hold_recovery.sh
git commit -m "test(scripts): bench HOLD recovery end to end"
```

---

### Task 8: 文档同步与真机清单

**Files:**
- Modify: `docs/superpowers/specs/2026-11-03-hold-recovery-design.md`(把实施中确认的默认值/假设写回)
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`(参数注释)

- [ ] **Step 1: 更新 spec 的"开放问题"与默认值**

把 §14 中已决定的项标为"已定",写入实际默认值;补一段"真机验证清单"。

- [ ] **Step 2: 真机清单**

```
1. agx_arm_ctrl 运行、臂在 home;
2. lgw_full.launch.py follow:=true ...;
3. hold_recovery_enable=true, recovery_velocity_scaling=0.10;
4. 触发一次 servo_halt(把目标放到腕奇异附近);
5. 检查 /lgw/servo/state: RECOVERING -> TRACKING;
6. 检查 Servo 首条输出 = 摆位后姿态(check_servo_zero_command.py 在线版);
7. release 打断一次恢复,确认臂停止且无第二个写者。
```

- [ ] **Step 3: 提交**

```bash
git add docs/superpowers/specs/2026-11-03-hold-recovery-design.md \
        src/lgw_bt_planner/config/servo_node.yaml
git commit -m "docs(servo): record HOLD recovery defaults and robot checklist"
```

---

## Self-Review

- **Spec coverage**:§3/§4 → Task 2 判据;§5 → Task 5;§6 → Task 1;§7/§8/§9 → Task 5;§10 → Task 3;§11 → 各任务;§12 → Task 4;§13 → Task 7/8。
- **已知偏差**:spec §4 曾把 `too_close` 列为 HOLD 原因,已在 spec 修正(它只是诊断标志)。`pose_stale` 在 Task 2 返回 `kNone` 而不是 `kExhausted`,避免瞬态 pose 丢失永久关闭恢复;spec 表已注明的"exhausted"应同步修正。
- **类型一致性**:`HoldReason`/`RecoveryAction`/`HoldRecoveryParams` 只在 Task 2 定义,Task 3/5/6 直接引用。
- **未覆盖**:真机 `follower_lag` 只能在真机验证(Task 8 清单);mock 下 gap 恒为 0。
