# servo_node 拆分重构 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 3246 行的 `servo_node.cpp` 拆成"节点只接线 + 业务下沉为可单测的 C++ 类"，用离线回放证明每一步行为不变，同时把参数迁移到 ROS 2 官方的 `generate_parameter_library`。

**Architecture:** 分十步、每步一个提交：先建可复现的验证装置（tick 接缝 + 回放台 + golden trace），再从"不改变判断"的搬家开始（参数、样板、enum），最后搬最危险的控制拍与恢复，节点最后只剩订阅/服务/定时器/实例持有。纯逻辑通过"显式入参（含 `now_ns`）"变得可测，不引入接口类。真机只在三个里程碑冒烟。

**Tech Stack:** C++17、rclcpp (Humble)、gtest、`launch_testing`、`generate_parameter_library`、`ament_lint_auto`、colcon。

**Spec:** `docs/superpowers/specs/2026-11-07-lgw-servo-node-refactor-design.md`

## Global Constraints

- 容器内一切构建/测试：`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && <cmd>"`。
- **禁止** `pkill -f ros` / `killall` / `kill -9 -1`；清理按 PID `kill -15`。
- 本容器 **不保证 `colcon test` 可用**（CLAUDE.md 有此记录，但包存在）——Task 0 先实测；不可用时 gtest/回放二进制直接执行，`launch_testing` 用 `launch_test <file>`。
- 除 D1–D8 卡片外，行为必须与基线提交 `3001f71` **逐拍等价**（浮点 1e-9）。
- 参数名保持不变；默认值改动必须由 V4 快照测试显式体现。
- 每个任务结束必须：build 干净 → 全部既有 gtest 绿 → 新测试绿 → 回放 diff 干净 → 单独提交。
- 分支：`refactor/servo-node-split`（已建，基线 `3001f71`）。

## Review Focus

1. **目标时间戳异常**（跨主机时钟偏移、未来时间戳、停更 >500 ms）：节点必须维持既有降级（HOLD / 几何源 none），不得除零或无限 HOLD。测试落在 Task 6。
2. **关节消息不完整或乱序**（`/feedback/joint_states` 缺关节、名字重复、长度不足）：必须忽略缺项而不是把 0 写进关节数组。测试落在 Task 6。
3. **非有限值**（TF/目标/前馈出现 NaN/Inf）：任何情况下不得把 NaN 传给 Servo。测试落在 Task 6。
4. **服务幂等**（FAULT/HOLD 下重复 release/pause/resume、takeover 连发）：不得重复启停 Servo 或泄漏线程。测试落在 Task 8。
5. **恢复超时与预算耗尽**（deadline 到点而 worker 仍阻塞、预算用尽后再次 halt）：状态必须落到既有语义（HOLD + exhausted），不得卡在 RECOVERING。测试落在 Task 7。

---

### Task 0: 确认测试基建并更新记录

**Files:**
- Modify: `CLAUDE.md`（若结论与现有记录不符）
- Test: 一次性手工验证，无代码

**Interfaces:**
- Produces: 结论"colcon test 可用/不可用"与 launch_testing 的执行方式，供后续所有任务引用。

- [ ] **Step 1: 实测 colcon test**

Run:
```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
 "source /my_ws/l4t/.l4t_env.sh && colcon test --packages-select lgw_bt_planner --event-handlers console_direct+ 2>&1 | tail -20"
```
Expected: 要么执行出 ctest 结果，要么明确报缺 `ament_cmake_test`。

- [ ] **Step 2: 实测 launch_test 直跑**

Run: 在 `src/lgw_bt_planner/test/` 写一个最小 `smoke_launch_test.py`（起 `talker`/`listener` 或空 launch 均可），执行 `launch_test <file> --disable-output`。
Expected: 退出码 0。

- [ ] **Step 3: 按结论改 CLAUDE.md**

若 `colcon test` 可用，删除"本容器 colcon test 不可用"的条目；若不可用但 `launch_test` 可用，把条目改成"colcon test 不可用；launch_testing 用 `launch_test <file>` 直跑"。

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record the container's actual test tooling status"
```

---

### Task 1: 节点类外置 + `tick(now_ns)` 接缝

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_node.hpp`
- Create: `src/lgw_bt_planner/src/servo/servo_node.cpp`（从现 `src/servo_node.cpp` 整体搬入，仅改文件位置与可见性）
- Modify: `src/lgw_bt_planner/src/servo_node_main.cpp`（若存在 main）或原 `servo_node.cpp` 仅保留 `main()`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`
- Test: `src/lgw_bt_planner/tests/test_servo_node_tick.cpp`

**Interfaces:**
- Produces:
  - `class lgw_bt_planner::ServoNode : public rclcpp::Node`
  - `void ServoNode::tick(int64_t now_ns)` —— 原 `on_timer()` 的全部逻辑，改为接收时间；`on_timer()` 仅调用 `tick(now().nanoseconds())`
  - `ServoNode(const rclcpp::NodeOptions &)`，便于测试用 `NodeOptions` 注入参数

- [ ] **Step 1: 写失败测试（tick 可从测试驱动且不发 twist）**

```cpp
TEST(ServoNodeTick, TicksWithoutSessionAndStaysIdle)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("frame_prefix", "arm1/")});
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(opts);
  EXPECT_EQ(node->state_for_test(), "IDLE");
  node->tick(1'000'000'000);
  EXPECT_EQ(node->state_for_test(), "IDLE");
}
```

- [ ] **Step 2: 跑一次确认失败**

Run: `./build/lgw_bt_planner/test_servo_node_tick`
Expected: 编译失败（找不到 `servo/servo_node.hpp`）。

- [ ] **Step 3: 整体搬文件并加最小可见性**

把类从匿名命名空间移到头文件（命名空间 `lgw_bt_planner`），`on_timer()` 改为 `tick(now().nanoseconds())` 的一行转调，新增
`std::string state_for_test() const;`（返回 `sm_.state()` 的名字）。**不拆分任何函数、不改任何逻辑**。

- [ ] **Step 4: 跑测试确认通过**

Run: `./build/lgw_bt_planner/test_servo_node_tick`
Expected: PASS。

- [ ] **Step 5: 跑既有 18 个 gtest + 回放基线（此步尚无回放，改为手工冒烟）**

Run: `for t in build/lgw_bt_planner/test_*; do ./$t | tail -1; done`
Expected: 全部 `PASSED` 且数量与基线一致（18 个）。

- [ ] **Step 6: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo src/lgw_bt_planner/src/servo \
        src/lgw_bt_planner/src/servo_node.cpp src/lgw_bt_planner/CMakeLists.txt \
        src/lgw_bt_planner/tests/test_servo_node_tick.cpp
git commit -m "refactor(servo): move the node class out of the anonymous namespace and add a time-parameterised tick"
```

---

### Task 2: 回放台与 golden trace

**Files:**
- Create: `src/lgw_bt_planner/test/servo_replay.cpp`（独立可执行，非 gtest）
- Create: `src/lgw_bt_planner/tests/test_replay_scenarios.cpp`（跑回放并与 golden 比对）
- Create: `src/lgw_bt_planner/test/scenarios/*.json`（场景输入）
- Create: `src/lgw_bt_planner/test/golden/*.trace`（由基线提交录制）
- Modify: `src/lgw_bt_planner/CMakeLists.txt`

**Interfaces:**
- Consumes: `ServoNode::tick(int64_t)`（Task 1）
- Produces:
  - 回放二进制 `servo_replay <scenario.json> <out.trace>`
  - trace 格式：每行 `t_ns,kind,payload`，kind ∈ {`state`,`twist`,`transition`,`service`}
  - 场景 json：`{"params":{...},"steps":[{"t_ns":..., "target":[...], "joint_cmd":[...], "joint_meas":[...], "servo_status":"..."}]}`

- [ ] **Step 1: 写失败测试（同一场景两次回放必须逐字节相同）**

```cpp
TEST(Replay, IsDeterministic)
{
  auto a = run_replay("scenarios/track_static.json");
  auto b = run_replay("scenarios/track_static.json");
  EXPECT_EQ(a, b);
}
```

- [ ] **Step 2: 跑一次确认失败**

Expected: 找不到 `servo_replay` / `run_replay`。

- [ ] **Step 3: 实现回放二进制与 `run_replay`**

构造 `ServoNode`（`NodeOptions` 注入场景参数），按 steps 顺序：先把目标/关节写入节点的测试注入接口（新增 `void inject_for_test(const ReplayStep&)`），再 `tick(step.t_ns)`，把每次的 twist/state/迁移写入 trace。**不允许起 timer、不允许 spin**。

- [ ] **Step 4: 用"归档副本"录制 golden（解决先有鸡还是先有蛋）**

基线 `3001f71` 的节点无法被确定性驱动（没有 tick 接缝），所以 golden 不能直接从它录。做法：

1. `git worktree add /tmp/baseline 3001f71`；
2. 在 `/tmp/baseline` 里**只应用 Task 1 的机械改动**（把类搬出头文件 + `tick(now_ns)` 转调，一行逻辑不改），打成一个 patch 文件存进 `test/golden/baseline_seam.patch`；
3. 在 `/tmp/baseline` 上 build `servo_replay`，对 8 个场景各录一份 `test/golden/*.trace`（场景清单见 §V5）。

- [ ] **Step 5: 双跑证明接缝本身零行为变化**

Run: `./build/lgw_bt_planner/test_replay_scenarios`
Expected: 当前分支（Task 1 之后的真实代码）与 `/tmp/baseline` + patch 录出的 trace **逐字节相同**。这同时证明了 Task 1 的搬家与接缝没有改变行为，并立住了 golden 的权威性。

- [ ] **Step 5b: 建 `launch_testing` 场景骨架**

写 `test/session_launch_test.py`：起 `lgw_full.launch.py follow:=false`（mock 硬件）+ `fake_track_publisher.py`，断言 `/lgw/servo/state` 出现且状态为 `IDLE`。执行方式按 Task 0 的结论（`colcon test` 或 `launch_test <file>`）。
Expected: 退出码 0。

- [ ] **Step 6: Commit**

```bash
git add src/lgw_bt_planner/test src/lgw_bt_planner/tests/test_replay_scenarios.cpp src/lgw_bt_planner/CMakeLists.txt
git commit -m "test(servo): add a deterministic replay harness and golden traces recorded from the baseline"
```

---

### Task 3: `ServoParams` 纯搬家（S1a）

**Files:**
- Create: `include/lgw_bt_planner/servo/servo_params.hpp`、`src/servo/servo_params.cpp`
- Create: `include/lgw_bt_planner/servo/lookat_mode.hpp`（`enum class LookatMode` + `parse_lookat_mode`，Task 5 才在节点里使用）
- Modify: `src/servo/servo_node.cpp`（`declare_and_load_parameters` 改为调用 `params_.load(*this)`）
- Test: `tests/test_servo_params.cpp`

**Interfaces:**
- Produces:
  - `struct ServoParams { /* 按域分组：geometry, control, follow, recovery, preposition, lookat, qp */ };`
  - `void ServoParams::load(rclcpp::Node & node);` —— **内部仍是 101 个 `declare_parameter`**，逐字搬运
  - `const ServoParams & ServoNode::params() const;`

- [ ] **Step 1: 写失败测试（参数快照）**

```cpp
// 注意：servo_node.yaml 是按节点名组织的（lgw_servo_node:），测试节点必须用同名，
// 否则整段参数不会生效，断言会无声地测到代码默认值。
static rclcpp::Node::SharedPtr node_with_shipped_yaml()
{
  const auto yaml = ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/config/servo_node.yaml";
  return std::make_shared<rclcpp::Node>(
    "lgw_servo_node",
    rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", yaml}));
}

TEST(ServoParams, DefaultsMatchTheShippedYaml)
{
  rclcpp::init(0, nullptr);
  auto node = node_with_shipped_yaml();
  ServoParams p; p.load(*node);
  EXPECT_DOUBLE_EQ(p.control.kp_pos, 3.0);      // yaml 覆盖代码默认 2.0
  EXPECT_DOUBLE_EQ(p.control.kp_ang, 5.0);
  EXPECT_DOUBLE_EQ(p.control.kd_pos, 0.30);
  EXPECT_EQ(p.follow.follow_timeout_ms, 8000);
  rclcpp::shutdown();
}
```

- [ ] **Step 2: 跑一次确认失败**（找不到 `servo_params.hpp`）

- [ ] **Step 3: 逐字搬运 101 个 `declare_parameter`，按域分组**

不改任何默认值、不改任何类型；`load()` 的调用点保持原位置、原顺序。

- [ ] **Step 4: 跑测试 + 回放**

Run: `./build/lgw_bt_planner/test_servo_params && ./build/lgw_bt_planner/test_replay_scenarios`
Expected: 全 PASS，回放 diff 为空。

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(servo): move parameter declaration into a ServoParams struct (no behaviour change)"
```

---

### Task 4: 迁移到 `generate_parameter_library`（S1b）

**Files:**
- Create: `src/lgw_bt_planner/config/servo_node_parameters.yaml`（schema）
- Modify: `src/lgw_bt_planner/CMakeLists.txt`（`generate_parameter_library`）
- Modify: `src/servo/servo_params.{hpp,cpp}`
- Modify: `config/servo_node.yaml`（仅在名字/结构需要时；默认值仍保留在这里）
- Test: `tests/test_servo_params.cpp`（扩充）

**Interfaces:**
- Consumes: Task 3 的 `ServoParams`
- Produces: `ServoParams::load(node)` 改为 `listener_ = std::make_shared<ParamListener>(node); params_ = listener_->get_params();`，字段名与 Task 3 一一对应。

- [ ] **Step 1: 扩充失败测试（校验生效）**

```cpp
TEST(ServoParams, RejectsOutOfRangeKp)
{
  auto node = make_node_with({"kp_pos", -1.0});
  EXPECT_THROW(ServoParams::load(*node), std::exception);
}
TEST(ServoParams, AcceptsDocumentedSentinels)
{
  auto node = make_node_with({"v_max", 0.0, "recovery_max_attempts", 0, "recovery_deadline_ms", 0});
  EXPECT_NO_THROW(ServoParams::load(*node));
}
```

- [ ] **Step 2: 跑一次确认失败**（当前无校验，`kp_pos=-1` 不抛）

- [ ] **Step 3: 写 schema 并替换加载路径**

schema 中每个参数保留原默认值与范围；互斥关系（`tcp_pose_domain=command` 时阻尼不生效）写自定义 validator。参数名不变。

- [ ] **Step 4: 跑测试 + 回放 + 参数快照**

Expected: 全部 PASS；回放 diff 为空；快照测试仅因"默认值来源从代码变成 schema"而更新，值本身不变。

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(servo): migrate node parameters to generate_parameter_library with schema validation"
```

---

### Task 5: 失败路径收敛与模式 enum（S2 / D3+D4）

**Files:**
- Modify: `src/servo/servo_node.cpp`（7 处样板、两个 bool）
- Test: `tests/test_lookat_mode.cpp`（`lookat_mode.hpp` 已在 Task 3 创建）、回放场景 `halted_*`

**Interfaces:**
- Produces:
  - `enum class LookatMode { kArmRelative, kTargetRelative, kPinnedPosture };`
  - `void ServoNode::hold_and_return(HoldReason reason, bool clear_geometry);`

- [ ] **Step 1: 写失败测试（enum 解析与非法值）**

```cpp
TEST(LookatMode, ParsesTheThreeShippedValues)
{
  EXPECT_EQ(parse_lookat_mode("arm_relative"), LookatMode::kArmRelative);
  EXPECT_EQ(parse_lookat_mode("target_relative"), LookatMode::kTargetRelative);
  EXPECT_EQ(parse_lookat_mode("pinned_posture"), LookatMode::kPinnedPosture);
  EXPECT_EQ(parse_lookat_mode("nonsense"), LookatMode::kPinnedPosture);  // 既有回退
}
```

- [ ] **Step 2: 跑一次确认失败**

- [ ] **Step 3: 实现 enum 与 `hold_and_return`**

7 处替换为一次调用；**saturated 那一处（原 2011 行）传 `clear_geometry=false`**，其余 `true`。`hold_reason` 保持每处原值（第一处继续用 `classify_hold`）。

- [ ] **Step 4: 跑测试 + 回放（必须零 diff）**

Expected: PASS，回放 diff 为空（这是"逐处等价"的证明）。

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(servo): collapse the duplicated hold paths and replace the mode bools with an enum"
```

---

### Task 6: 抽取 `TrackingLoop`（S3）

**Files:**
- Create: `include/lgw_bt_planner/servo/tracking_loop.hpp`、`src/servo/tracking_loop.cpp`
- Modify: `src/servo/servo_node.cpp`（`update_tracking_locked` 变薄）
- Test: `tests/test_tracking_loop.cpp`

**Interfaces:**
- Produces:
  - `struct TrackingInput { int64_t now_ns; lgw_bt_planner::GeometrySource source; Pose6 tcp_now; Pose6 desired; std::optional<ServoStatus> servo; bool follow_ok; ... };`（`GeometrySource` 复用既有的 `geometry_source.hpp`，不新增类型）
  - `struct TrackingOutput { TwistCommand twist; ServoEvent event; std::string source_name; bool publish; };`
  - `TrackingOutput TrackingLoop::step(const TrackingInput & in, const ServoParams & p, SessionState & st);`
  - `SessionState { HoldReason hold_reason; int recovery_attempts; std::string geometry_source; bool session_active; std::vector<double> last_sent_joints; }`（会话持有，按引用传入）

- [ ] **Step 1: 写失败测试（纯函数无 rclcpp 也能跑）**

```cpp
TEST(TrackingLoop, EmitsNoTwistWhenTheTargetIsLost)
{
  TrackingInput in; in.now_ns = 1'000'000'000;
  in.source.kind = SourceKind::NONE;          // 既有枚举
  TrackingLoop loop; SessionState st; ServoParams p = defaults();
  const auto out = loop.step(in, p, st);
  EXPECT_FALSE(out.publish);
  EXPECT_EQ(out.source_name, "none");
}
TEST(TrackingLoop, RejectsNonFinitePoseWithoutEmittingNaN)  // Review Focus #3
{
  TrackingInput in = valid_input();
  in.tcp_now.p.setValue(std::nan(""), 0, 0);
  SessionState st;
  const auto out = TrackingLoop::step(in, defaults(), st);
  EXPECT_FALSE(out.publish);
}
TEST(TrackingLoop, StaleTargetDegradesToHold)                // Review Focus #1
{
  TrackingInput in = valid_input();
  in.now_ns = in.target_stamp_ns + 900'000'000;   // 停更 900 ms > 500 ms
  SessionState st;
  const auto out = TrackingLoop::step(in, defaults(), st);
  EXPECT_FALSE(out.publish);
  EXPECT_EQ(st.geometry_source, "none");
}
TEST(TrackingLoop, IgnoresJointsMissingFromTheMessage)       // Review Focus #2
{
  auto in = valid_input_with_joints({"joint1", "joint3"});   // 只有两个关节
  SessionState st;
  const auto out = TrackingLoop::step(in, defaults(), st);
  EXPECT_FALSE(out.publish);   // 残缺输入不得用 0 补齐去驱动手臂
}
```

- [ ] **Step 2: 跑一次确认失败**

- [ ] **Step 3: 把 `update_tracking_locked`（1688–2028）拆为 pick/solve/apply/publish 四个私有函数**

控制拍内**不再查 TF、不再读时钟**；`lookup_tcp_locked`/`fk_tcp_locked`/`lookup_odom_base_locked` 的调用移到节点侧，结果放进 `TrackingInput`。调用顺序（tick → 恢复决策 → 发布）保持不变。

- [ ] **Step 4: 跑全部测试 + 回放**

Expected: 全部 PASS，回放 diff 为空。

- [ ] **Step 5: 真机里程碑 1（小幅扫掠）**

接管 → ±5–14 cm 扫掠 → 静止 → 关栈。判据：无 HOLD 抖动、静止段关节 p2p < 5 mrad、关栈不动臂。

- [ ] **Step 6: Commit**

```bash
git commit -m "refactor(servo): extract the per-tick pipeline into a pure TrackingLoop"
```

---

### Task 7: 抽取 `RecoveryOrchestrator`（S4）

**Files:**
- Create: `include/lgw_bt_planner/servo/recovery_orchestrator.hpp`、`src/servo/recovery_orchestrator.cpp`
- Modify: `src/servo/servo_node.cpp`
- Test: `tests/test_recovery_orchestrator.cpp`

**Interfaces:**
- Consumes: `SessionState`（Task 6）
- Produces:
  - `struct RecoveryPlan { bool needed; std::array<double,6> joints; std::string detail; };`
  - `RecoveryPlan RecoveryOrchestrator::decide(const RecoveryInput &, const ServoParams &, SessionState &);`
  - `std::function<MoveResult(const std::array<double,6>&)> preposition_fn`（构造注入）

- [ ] **Step 1: 写失败测试（预算耗尽 + deadline）** — Review Focus #5

```cpp
TEST(RecoveryOrchestrator, ExhaustsTheBudgetAndStopsRetrying)
{
  SessionState st; st.recovery_attempts = 1;
  ServoParams p = defaults(); p.recovery.max_attempts = 1;
  EXPECT_FALSE(RecoveryOrchestrator::decide(input_at(1'000'000'000), p, st).needed);
}
TEST(RecoveryOrchestrator, DeadlineDisablesRecovery)
{
  ServoParams p = defaults(); p.recovery.deadline_ms = 0;
  EXPECT_FALSE(RecoveryOrchestrator::decide(valid_input(), p, st).needed);
}
```

- [ ] **Step 2: 跑一次确认失败**

- [ ] **Step 3: 搬运决策逻辑**

`maybe_start_recovery_locked` 的判定部分进 `decide()`；`recovery_worker` 的线程与阻塞结构**原样保留**在节点侧（D8 本轮不做），只把"什么时候需要恢复、目标是什么"变成纯函数。MoveGroup 调用通过注入的 `preposition_fn`。

- [ ] **Step 4: 跑测试 + 回放（恢复场景用假 preposition_fn）**

Expected: PASS，回放 diff 为空。

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(servo): extract the recovery decision into a pure orchestrator with an injected prepositioner"
```

---

### Task 8: 抽取 `ServoSession`（S5）

**Files:**
- Create: `include/lgw_bt_planner/servo/servo_session.hpp`、`src/servo/servo_session.cpp`
- Create: `include/lgw_bt_planner/servo/servo_state_reporter.hpp`、`src/servo/servo_state_reporter.cpp`
- Modify: `src/servo/servo_node.cpp`（服务回调只剩转调）
- Test: `tests/test_servo_session.cpp`、`test/session_launch_test.py`

**Interfaces:**
- Consumes: `TrackingLoop`（Task 6）、`RecoveryOrchestrator`（Task 7）
- Produces: `ServoSession` 持有 `SessionState`、状态机、Servo 启停客户端；`bool take_over(id)`, `bool release()`, `bool pause()`, `bool resume()`；节点回调只做 `return session_->take_over(id);`

- [ ] **Step 1: 写失败测试（服务幂等）** — Review Focus #4

```cpp
TEST(ServoSession, RepeatedReleaseIsIdempotent)
{
  FakeServo servo; ServoSession s(&servo, defaults());
  EXPECT_TRUE(s.release());
  EXPECT_TRUE(s.release());          // 不重复 stop，不抛
  EXPECT_EQ(servo.stop_calls, 1);
}
```

- [ ] **Step 2: 跑一次确认失败**

- [ ] **Step 3: 搬运服务编排与 Servo 生命周期**

并发拓扑**原样保留**：`servo_sync_group_`(auto_add=false)、局部 `SingleThreadedExecutor`、`service_group_` MutuallyExclusive、控制拍全程持 `mutex_`。

- [ ] **Step 4: 跑测试 + `launch_testing` 场景 + 回放**

Expected: 全部 PASS。

- [ ] **Step 4b: 把状态发布搬进 `servo_state_reporter`**

`publish_state_locked`（109 行）移入该模块，输入只有 `SessionState` + 参数；节点侧只做一次转调。做完后回放 diff 必须为空（`/lgw/servo/state` 是回放比对项）。

- [ ] **Step 4c: 复核 D5（长锁），只做决策不做改动**

纯 tick 已经抽出，此时评估"锁内拷贝输入、锁外计算"是否可行，把结论写进设计文档 §9（改或继续不改都要记），**不在本任务里实施**。

- [ ] **Step 5: 真机里程碑 2**

release / pause / resume 各一次 + 一次恢复；状态迁移与重构前一致。

- [ ] **Step 6: Commit**

```bash
git commit -m "refactor(servo): move takeover/release/pause/resume and the servo lifecycle into ServoSession"
```

---

### Task 9: 缺陷清理（S6 / D1+D6+D7）

**Files:**
- Modify: `src/servo/servo_node.cpp`、`config/servo_node.yaml`、`CMakeLists.txt`
- Test: 参数快照（`force_cancel_moves` 消失）

- [ ] **Step 1: 写失败测试（参数表里不再有 force_cancel_moves）**

- [ ] **Step 2: 跑一次确认失败**

- [ ] **Step 3: 删除参数 + 同步删 yaml 条目；接 `ament_lint_auto`（uncrustify + clang-tidy 最小集，仅新增/重构文件）**

- [ ] **Step 4: 跑测试 + lint（只报告不阻断）+ 回放**

Expected: PASS；lint 报告归档到 commit message。

- [ ] **Step 5: Commit**

```bash
git commit -m "chore(servo): drop the dead force_cancel_moves switch and enable scoped lint"
```

---

### Task 10: 收尾

**Files:**
- Modify: `docs/superpowers/specs/2026-11-07-lgw-servo-node-refactor-design.md`（状态改为"已实施"）
- Modify: `CLAUDE.md`（servo_node 结构说明）
- Test: 全套回归

- [ ] **Step 1: 跑全部 gtest + 回放 + launch_testing**

Expected: 全绿。

- [ ] **Step 2: 真机里程碑 3（完整 smoke）**

判据沿用 `docs/dev/2026-11-06-vendor-modifications-and-rollback.md` 的实测表。

- [ ] **Step 3: 更新文档并提交**

```bash
git commit -m "docs(servo): mark the node split complete and record the new structure"
```
