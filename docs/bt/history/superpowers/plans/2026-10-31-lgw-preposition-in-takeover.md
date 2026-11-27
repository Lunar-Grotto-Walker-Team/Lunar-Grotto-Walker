# 接管内摆位(pre-position)实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `lgw_servo_node` 在 takeover 时自己判断臂是否在姿态族外,需要的话用 MoveGroup 摆进族内,再进入 TRACKING。

**Architecture:** 判据做成纯函数(可离线单测);节点在 PREEMPT 里持有一个 MoveGroup action 客户端,用"专用 callback group + 局部 SingleThreadedExecutor"阻塞等待(照抄现有 Servo pause/unpause 的模式),等待期间不持 `mutex_`。为让测试快、可测,先把纯数学模块拆成 STATIC 小库 `lgw_posture`。

**Tech Stack:** C++17、rclcpp、moveit_msgs action、kdl_parser、ament_cmake_gtest、Python 假 `/move_action` 服务端。

**Spec:** `docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md` §3.8(rev3)

## 组件划分(谁承担具体实现)

原则:**节点只做编排与持有,具体实现放在协作者里**;纯映射写成纯函数(无状态就别造类)。

| 组件 | 形态 | 拥有的东西 | 测试方式 |
|---|---|---|---|
| `decide_preposition()` | 纯函数 | 无 | gtest,合成输入 |
| `make_joint_goal()` | 纯函数 | 无 | gtest,断言 goal 字段 |
| `MoveGroupPrepositioner` | **类** | action 客户端、专用 callback group、局部 executor、超时 | 直接对着假 `/move_action` 服务端跑,不需要 servo 节点 |
| `servo_node` | 编排 | 持有上面三者 + `PostureFamily` | 集成 bench |

`MoveGroupPrepositioner` 是这次唯一有状态/有 ROS 句柄的协作者,所以它成类;判据和 goal
构造没有状态,保持纯函数(造类反而增加单测成本)。

## Global Constraints

- 宿主机编辑,容器内构建;`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && ..."`。
- 单元测试跑 build 下的 gtest 二进制(容器无 `ament_cmake_test`)。
- **不驱动真机**:任何 bench/集成运行前必须确认 `agx_arm_ctrl_single` 进程数为 0;清理只用 `scripts/bench_cleanup.sh`(按 PID),禁止 `pkill -f ros`。
- 判据默认值(spec §3.8):`preposition_joint_delta_rad 0.15`、`preposition_branch_delta_rad 1.571`(同侧兜底;主判据是 q5 异号)、`preposition_limit_margin_rad 0.10`、`preposition_tcp_distance_m 0.05`、`preposition_tcp_angle_rad 0.15`、`preposition_velocity_scaling 0.3`(0 = 不指定)、`preposition_timeout_s 10.0`、`preposition_enable true`。
- Piper 限位:j1 ±2.6179938、j2 [0, π]、j3 [−2.9670597, 0]、j4 ±1.7453292、j5 ±1.2217304、j6 ±2.0943951。
- 提交信息中文 + `Co-Authored-By: Claude Code <noreply@anthropic.com>`。

## 与后续重构的关系

`docs/superpowers/specs/2026-10-31-lgw-servo-node-refactor.md` 已排期在本功能真机验证
之后执行。本计划据此加三条约束,避免给那次重构添阻力:

1. 新逻辑一律进纯模块(`preposition_decision` / `preposition_goal`),节点侧只留"调用 + 分支";
2. 节点侧的摆位代码集中在 `handle_takeover` 和一个自包含的
   `bool maybe_preposition_locked(...)` 里,将来可整体搬进 `servo_session`;
3. **不新增模式类的 bool**;`preposition_enable` 是开关不是模式,保留。

## Review Focus

- **等待期间持锁**:MoveGroup 等待若持有 `mutex_`,100 Hz 控制拍会停 1–2 秒 —— 期望:等待期间无锁。
- **失败后仍 unpause**:goal 失败却继续接管,臂在族外起步 —— 期望:失败一律回 IDLE 且不 unpause。
- **Servo 未 paused 就发 MoveGroup**:两条链路同时写 `/control` —— 期望:发 goal 前确认 paused。
- **超时无出口**:`/move_action` 不响应时永久挂住 takeover —— 期望:超时后返回失败。
- **缩放 0 的语义**:`preposition_velocity_scaling = 0` 必须写成 goal 的 0(表示"不指定"),而不是被误当成"很慢"或"很快"。

---

### Task 1: 拆分 STATIC 小库 `lgw_posture`

**Files:**
- Create: `src/calibration_json.cpp`(把 `parse_calibration_json` 的**定义**从 `src/compute_lookat_pose_node.cpp` 搬出来;声明仍在 `include/lgw_bt_planner/compute_lookat_pose_node.hpp`)
- Modify: `CMakeLists.txt`、`package.xml`(`nlohmann-json3-dev` 已通过传递依赖可用,按需显式声明)
- Test: `tests/test_lookat_posture_family.cpp`(不改内容,只改链接)

**Interfaces:**
- Produces: `lgw_posture` (STATIC) 导出 `lgw_bt_planner::{PostureFamily, PostureParams, PostureResult, parse_calibration_json}`

- [ ] **Step 1: 记录基线**

Run: `docker exec ... ./build/lgw_bt_planner/test_lookat_posture_family`
Expected: `[ PASSED ] 12 tests`

- [ ] **Step 2: 改 CMake**

```cmake
add_library(lgw_posture STATIC
  src/lookat_posture_family.cpp
  src/calibration_json.cpp)
ament_target_dependencies(lgw_posture tf2 Eigen3)
# 巨型库改为链接,不再编译进去
target_link_libraries(lgw_bt_planner_nodes lgw_posture)
# 测试只链小库
target_link_libraries(test_lookat_posture_family lgw_posture ament_index_cpp)
```
从 `NODE_SOURCES` 删掉 `src/lookat_posture_family.cpp`。

- [ ] **Step 3: 搬 `parse_calibration_json`**

把定义原样移到 `src/calibration_json.cpp`(仅 include `nlohmann/json.hpp` + tf2,不得 include rclcpp)。

- [ ] **Step 4: 验证**

```bash
cmake --build build/lgw_bt_planner --target test_lookat_posture_family -j6
./build/lgw_bt_planner/test_lookat_posture_family            # 期望 12 passed
ldd build/lgw_bt_planner/test_lookat_posture_family | grep -c liblgw_bt_planner_nodes   # 期望 0
docker exec ... colcon build --packages-select lgw_bt_planner && ./build/.../servo_node --ros-args -p lookat_mode:=pinned_posture   # 起得来
```

- [ ] **Step 5: 提交**

---

### Task 2: 判据纯函数

**Files:**
- Create: `include/lgw_bt_planner/preposition_decision.hpp`、`src/preposition_decision.cpp`
- Modify: `CMakeLists.txt`(加入 `lgw_posture`)
- Test: `tests/test_preposition_decision.cpp`

**Interfaces:**
- Produces:
```cpp
struct PrepositionParams {
  bool enable{true};
  double joint_delta_rad{0.15};
  double branch_delta_rad{1.571};
  double limit_margin_rad{0.10};
  double tcp_distance_m{0.05};
  double tcp_angle_rad{0.15};
};
struct PrepositionDecision {
  bool needed{false};
  std::string reason;        // "ok" / "joint_delta" / "branch" / "limit" / "tcp" / "disabled"
  double max_joint_delta{0.0};
  double q5_delta{0.0};
  double min_limit_margin{0.0};   // 已计入 preposition 的 joint 限位表
  double tcp_distance{0.0};
  double tcp_angle{0.0};
};
PrepositionDecision decide_preposition(
  const std::array<double, 6> & q_family,
  const std::array<double, 6> & q_now,
  const Pose6 & tcp_family, const Pose6 & tcp_now,
  const PrepositionParams & params);
```

- [ ] **Step 1: 写失败的测试**(表驱动,至少 8 例)

```cpp
TEST(PrepositionDecision, AllCriteriaPassNeedsNothing);
TEST(PrepositionDecision, JointDeltaTriggers);        // 0.16 > 0.15
TEST(PrepositionDecision, JointDeltaBoundaryDoesNotTrigger);  // 恰好 0.15
TEST(PrepositionDecision, BranchDeltaTriggers);       // q5 差 π
TEST(PrepositionDecision, LimitMarginTriggers);       // q 贴到限位 0.05
TEST(PrepositionDecision, TcpDistanceTriggers);
TEST(PrepositionDecision, TcpAngleTriggers);
TEST(PrepositionDecision, DisabledNeverNeedsPreposition);
TEST(PrepositionDecision, ReasonNamesFirstFailingCriterion);
```

- [ ] **Step 2-5:** 跑 → 失败;实现;跑 → 全绿;提交

---

### Task 3: `MoveGroupPrepositioner`(action 客户端 + 阻塞等待)

**Files:**
- Create: `include/lgw_bt_planner/move_group_prepositioner.hpp`、`src/move_group_prepositioner.cpp`
- Modify: `CMakeLists.txt`(新目标,依赖 rclcpp / moveit_msgs;`make_joint_goal` 仍为纯函数)
- Test: `tests/test_move_group_prepositioner.cpp`(对着假服务端)

**Interfaces:**
- Produces:
```cpp
// 纯函数,单独可测
moveit_msgs::msg::MoveGroup::Goal make_joint_goal(
  const std::array<double, 6> & joints,
  const std::array<std::string, 6> & names,
  double velocity_scaling,   // 原样写入;0 = 让 MoveIt 用 joint_limits.yaml 默认
  double tolerance_rad);

// 有状态协作者:拥有 action 客户端、专用 callback group 和局部 executor
class MoveGroupPrepositioner
{
public:
  MoveGroupPrepositioner(rclcpp::Node & node, const std::string & action_name);
  // 阻塞直到结果/超时。**调用方不得持有控制锁。**
  bool move_to(const std::array<double, 6> & q,
               const std::array<std::string, 6> & names,
               double velocity_scaling, double tolerance_rad,
               std::chrono::milliseconds timeout, std::string & error);
  void cancel();                       // PREEMPT 期间收到 release 时调用
};
```

- [ ] **Step 1: 写失败的测试**

```cpp
TEST(PrepositionGoal, JointNamesAndPositionsMatch);
TEST(PrepositionGoal, ZeroScalingIsWrittenAsZero);     // 不能被替换成默认值
TEST(PrepositionGoal, ToleranceAppliedToEveryJoint);
TEST(PrepositionGoal, GroupIsArm);
// 类部分:对着假 /move_action 服务端
TEST(MoveGroupPrepositioner, SucceedsWhenServerAccepts);
TEST(MoveGroupPrepositioner, TimesOutWhenServerHangs);
TEST(MoveGroupPrepositioner, ReturnsFalseWhenServerAborts);
TEST(MoveGroupPrepositioner, CancelStopsWaiting);
TestSrv: 本任务的 Step 1 里同时写 `tests/fake_move_action_server.py`
```

- [ ] **Step 2-5:** 同上

---

### Task 4: 节点接线 + 假 `/move_action` 集成测试

**Files:**
- Modify: `src/servo_node.cpp`(takeover 流程、持有 `MoveGroupPrepositioner`、参数)
- Modify: `msg/ServoState.msg`(新增 `needs_preposition` / `max_joint_delta`)
- Test: `tests/test_preposition_integration.py`(节点级,复用 Task 3 的假服务端)

**Interfaces:**
- Consumes: `decide_preposition()`、`make_joint_goal()`、`PostureFamily::compute()`
- Produces: takeover 响应 message 中带 `preposition:` 前缀的诊断串

- [ ] **Step 1: 写假服务端与失败的集成测试**

假服务端支持 `--mode {ok,abort,reject,hang}`;测试脚本:
1. 起节点(`lookat_mode:=pinned_posture`,`preposition_enable:=true`) + 静态 TF + 假目标;
2. 把 mock 臂摆到族外(`fake_move_action_server --mode ok` 先接受一次 Server 的摆位 goal);
3. 调 `/lgw/servo/takeover`。

用例:`NeedsPrepositionAndMoves`、`PrepositionTimeoutRefusesTakeover`、`PrepositionAbortRefusesTakeover`、`StartsInFamilySkipsMove`、`DisabledSkipsMove`。

- [ ] **Step 2: 跑 → 失败**(takeover 仍直接成功,没有摆位)

- [ ] **Step 3: 实现**
```
- 参数声明(§3.8 表)
- 节点持有 `MoveGroupPrepositioner`(构造时传入 *this 和 action 名)
- 摆位逻辑封装成 `bool maybe_preposition_locked(...)`,handle_takeover 只调用它
  (该函数只做"取目标 → 调 decide → 调 prepositioner → 复判",具体实现都在协作者里)
- takeover: PREEMPT 内求 p_target/q_family/decision
- 需要时: **不持 mutex_** 发 goal,局部 executor 等结果,超时用 steady_clock
- 失败 → RELEASE 回 IDLE,response.message 说明
- 成功 → 重读 measured_positions_ 复判 → previous_ 播种 → unpause
- release 在 PREEMPT 期间 → cancel goal
```

- [ ] **Step 4: 跑 → 全绿**
- [ ] **Step 5: 提交**

---

### Task 5: 参数、状态字段与文档

**Files:**
- Modify: `config/servo_node.yaml`、`msg/ServoState.msg` 的填充处、`docs/servo/` 记录

- [ ] **Step 1: 写失败测试**(状态消息含新字段、参数默认值符合 §3.8)
- [ ] **Step 2-5:** 实现 → 全绿 → 提交

---

### Task 6: mock 端到端(含缩放 0.1 vs 0.3)

**Files:**
- Modify: `scripts/bench_posture_lookat.sh`(可选 `--preposition` 场景)
- Modify: `docs/servo/2026-10-31-posture-lookat-bench.md`

- [ ] **Step 1: 场景**

把臂摆到 q5 = −0.5(另一腕分支)→ 调 takeover → 断言:发生了摆位、耗时、进入 TRACKING、无 HOLD。
分别在 `preposition_velocity_scaling := 0.1 / 0.3 / 1.0` 下各跑一次,记录耗时与是否出现 JTC abort。

- [ ] **Step 2: 记录结果并提交**

---

## 不在本计划内

- 真机验证与离线碰撞检查。
- `servo_node.cpp` 拆文件(1831 行,与本功能无关)。
