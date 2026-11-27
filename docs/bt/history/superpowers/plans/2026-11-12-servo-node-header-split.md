# servo_node 实现下沉：从 2977 行头文件到实现文件

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `servo_node.hpp` 里的 2277 行函数体下沉到 `.cpp`，头文件只剩声明与成员，**逐行等价，不改任何功能逻辑**。

**Architecture:** 按类内已有的分节（参数 / 回调 / 控制拍 / 恢复 / takeover / 服务）分组搬迁，每搬一组就是一个提交，组内只做"删除函数体 → 在对应 .cpp 原样粘贴"。文件按职责拆成 4 个 .cpp，每个 400–800 行；共享的常量与自由函数放进一个私有的 `servo_node_detail.hpp`，用**具名命名空间**而非匿名命名空间（匿名命名空间在头文件里会让每个翻译单元各有一份，`ServoStartState` 因此触发 `-Wsubobject-linkage`）。

**Tech Stack:** C++17、rclcpp (Humble)、colcon、gtest、launch_testing。

**Spec:** `docs/superpowers/specs/2026-11-07-lgw-servo-node-refactor-design.md`；本次属其后续（2026-11-12 社区规范评审的"第一条问题"）。

## Global Constraints

- 容器内构建/测试：`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && <cmd>"`。
- **禁止** `pkill -f ros` / `killall` / `kill -9 -1`；清理按 PID `kill -15`。
- **行为必须逐拍等价**：只允许"函数体搬家 + 头文件声明化"，不允许任何逻辑、常量、注释语义的改动；发现需要改动时另开任务。
- 基线：当前全量 **302 tests, 0 failures**；每个任务结束必须仍是 302/0，且 `colcon build` 无 `-Wsubobject-linkage`。
- 测试文件**一行都不改**：`tests/test_servo_node_tick.cpp`、`tests/test_param_contract.cpp` 通过公共 API（`ServoNode(NodeOptions)`、`state_for_test()`、`tick(int64_t)`）使用节点，搬迁后必须原样编译通过。
- 构建产物是 SHARED 库 `lgw_bt_planner_nodes`，未设 `-fvisibility=hidden`，外置定义默认可被测试链接——不需要导出宏；若某任务后出现未定义符号，先查是不是漏搬，不要加导出宏掩盖。
- 头文件只保留声明后，**新 include 一律加到 .cpp**，不要往 .hpp 里加实现需要的头文件。

## Review Focus

1. **成员函数的 const / static / override 限定必须原样保留**：`state_for_test()` 是 public 非 const，`under_lock()` 等是 `override`；漏掉 `override` 不会编译失败但会静默断开接口——搬迁时逐个核对。
2. **匿名命名空间里的东西不能留在头文件**：`kTrackTopic` 等常量、`time_to_ns`、`rotation_angle`、`status_array_busy` 必须移出到具名命名空间；`ServoStartState` 是成员 `start_state_` 的类型，必须变成类的私有嵌套枚举。
3. **lambda 捕获语义不变**：`decltype(handle_)`、`timer_ = create_wall_timer(..., std::bind(&ServoNode::on_timer, this))` 之类搬到 .cpp 后捕获的仍是 `this`，不得改成捕获局部变量。
4. **构造顺序**：`ServoNode` 的成员初始化列表里有引用成员（`params_` 的引用）与 `param_listener_`，构造函数体搬到 .cpp 后初始化顺序仍由头文件中的声明顺序决定——不动声明顺序。
5. **`unique_ptr` / `shared_ptr` 的不完整类型**：`qp_solver_`、`param_listener_`、`prepositioner_`、`tf_listener_` 的析构需要完整类型；`~ServoNode()` 外置定义后，头文件可以把 `servo_node_params.hpp`、`lookat_qp_solver.hpp` 降级为前向声明或留给 .cpp——但**只在本计划最后一个任务里做**，前面任何一步都不许顺手改 include。

---

### Task 1: 清除头文件里的匿名命名空间

**Files:**
- Create: `include/lgw_bt_planner/servo/servo_node_detail.hpp`
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp:112-245`（匿名命名空间块）

**Interfaces:**
- Produces: `namespace lgw_bt_planner::servo_detail { constexpr char kTrackTopic[]; kTwistTopic; kStateTopic; kServoStatusTopic; int64_t time_to_ns(const builtin_interfaces::msg::Time &); double rotation_angle(const tf2::Matrix3x3 &, const tf2::Matrix3x3 &); bool status_array_busy(const action_msgs::msg::GoalStatusArray::SharedPtr &); }`
- Produces: `ServoNode::StartState`（private 嵌套枚举，底层值顺序不变：NOT_STARTED, REQUESTED, CONFIRMING, PAUSING, STARTED, FAILED）

- [ ] **Step 1: 建 `servo_node_detail.hpp`**

把 4 个 topic 常量、`time_to_ns`、`rotation_angle`、`status_array_busy` 原样搬进 `namespace lgw_bt_planner { namespace servo_detail { ... } }`；两个自由函数加 `inline`，其余不变（注释一并搬走）。

- [ ] **Step 2: 把 `ServoStartState` 变成类的私有枚举**

在 `class ServoNode` 的 `private:` 区加 `enum class StartState { ... }`（枚举值原样），把所有 `ServoStartState` 引用改成 `StartState`。

- [ ] **Step 3: 删掉头文件里的匿名命名空间与 `#include "servo_node_detail.hpp"` 到使用处**

`servo_node.hpp` 引入 detail 头并在文件内把 `time_to_ns(...)` 等调用改成 `servo_detail::time_to_ns(...)`（或 `using namespace servo_detail;` 在 `namespace lgw_bt_planner` 内——二选一，**建议显式限定**，避免以后重名）。

- [ ] **Step 4: 构建并确认警告消失**

Run: `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner 2>&1 | grep -c subobject-linkage"`
Expected: `0`（基线当前是 1）

- [ ] **Step 5: 全量测试**

Run: `colcon test --packages-select lgw_bt_planner && colcon test-result --verbose`
Expected: `302 tests, 0 errors, 0 failures`

- [ ] **Step 6: Commit**

`refactor(servo): move node-scope helpers out of the header's anonymous namespace`

---

### Task 2: 参数与标定加载下沉

**Files:**
- Create: `src/servo/servo_node.cpp`
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（`ServoNode::ServoNode`、`~ServoNode`、`declare_and_load_parameters`、`load_calibration` 改为声明）

**Interfaces:**
- Consumes: Task 1 的 `servo_detail`、`ServoNode::StartState`
- Produces: `ServoNode::ServoNode(const rclcpp::NodeOptions &)`, `ServoNode::~ServoNode()`, `void ServoNode::declare_and_load_parameters()`, `void ServoNode::load_calibration()` — 签名与当前完全一致

- [ ] **Step 1: 头文件中把 4 个函数改为声明**

保留原注释，只删函数体：`ServoNode(...)` 的构造体、`~ServoNode()` 的体、`declare_and_load_parameters()` 的体、`load_calibration()` 的体。构造函数的 `: rclcpp::Node("lgw_servo_node", options)` 初始化列表**随定义一起搬到 .cpp**；留在头文件里的是各成员的默认成员初始化器（`std::string & frame_prefix_{params_.frame_prefix};` 这一类），它们必须原样保留、顺序不动——成员初始化顺序由声明顺序决定，与定义写在哪无关。

- [ ] **Step 2: 在 `src/servo/servo_node.cpp` 写定义**

`#include` 该文件需要的全部头（含 `servo_node_params.hpp`、`ament_index_cpp`），函数体逐行原样粘贴，签名为 `ServoNode::ServoNode(const rclcpp::NodeOptions & options) : rclcpp::Node("lgw_servo_node", options) { ...原构造体... }`。

- [ ] **Step 3: 把新文件加入 CMake**

`set(NODE_SOURCES ...)` 追加 `src/servo/servo_node.cpp`。

- [ ] **Step 4: 构建 + 全量测试**

Run: build; `colcon test-result --verbose`
Expected: build 无 error；`302 tests, 0 failures`

- [ ] **Step 5: Commit**

`refactor(servo): move construction, parameter loading and calibration to servo_node.cpp`

---

### Task 3: 订阅回调下沉

**Files:**
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（11 个回调 + `state_for_test` 改声明）
- Modify: `src/servo/servo_node.cpp`（追加定义）

**Interfaces:**
- Produces: `on_track_array`, `on_pose_watch`, `measured_complete_locked() const`, `on_follow_reference`, `follow_gap_locked() const`, `commanded_advancing_locked()`, `on_servo_status`, `on_servo_output`, `on_move_status`, `on_ft_status`, `on_start_response`, `on_timer`, `state_for_test()` — 签名不变；`state_for_test()` 仍是 **public**（测试在用）

- [ ] **Step 1: 头文件把这 13 个成员改成声明**

逐字保留注释与 `const`/`override` 限定；`state_for_test()` 保持在 `public:` 区。

- [ ] **Step 2: `src/servo/servo_node.cpp` 追加定义**

粘贴函数体；lambda（如 `on_start_response` 的 future 处理）原样保留。

- [ ] **Step 3: 构建 + 全量测试**

Expected: `302 tests, 0 failures`

- [ ] **Step 4: Commit**

`refactor(servo): move subscription callbacks to servo_node.cpp`

---

### Task 4: 控制拍下沉

**Files:**
- Create: `src/servo/servo_tick.cpp`
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（5 个成员改声明）
- Modify: `CMakeLists.txt`（追加 `src/servo/servo_tick.cpp`）

**Interfaces:**
- Produces: `void tick(int64_t now_ns)`, `bool update_tracking_locked(const rclcpp::Time &, int64_t)`, `void publish_twist_locked(...)`, `void maybe_start_servo_locked(int64_t)`, `TriggerOutcome call_trigger_sync(...)` — 签名不变

- [ ] **Step 1: 头文件改声明**（保留原有分节注释）
- [ ] **Step 2: `servo_tick.cpp` 写定义**，函数体原样粘贴
- [ ] **Step 3: CMake 追加源文件**
- [ ] **Step 4: 构建 + 全量测试**

Expected: build 无 error；`302 tests, 0 failures`；`tests/test_servo_node_tick.cpp` 三个用例仍通过（它们直接调 `tick()`）

- [ ] **Step 5: Commit**

`refactor(servo): move the control tick and Servo start path to servo_tick.cpp`

---

### Task 5: 恢复路径下沉

**Files:**
- Create: `src/servo/servo_recovery.cpp`
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（13 个成员改声明）
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `hold_and_return(HoldReason, int64_t, bool clear_geometry)`, `enter_hold_locked`, `resolve_recovery_target_locked`, `finish_recovery_locked`, `finish_recovery_wait`, `finish_recovery_failed`, `recovery_abort_locked() const`, `worker_aborted()`, `finish_recovery_cancelled`, `verify_servo_seed(...) const`, `update_follower_recovery_locked`, `maybe_start_recovery_locked`, `recovery_worker()` — 签名不变

- [ ] **Step 1: 头文件改声明**
- [ ] **Step 2: `servo_recovery.cpp` 写定义**，`recovery_worker()` 的函数体逐行粘贴（线程/条件变量/析构协作一字不改）
- [ ] **Step 3: CMake 追加源文件**
- [ ] **Step 4: 构建 + 全量测试**

Expected: `302 tests, 0 failures`，其中 `test_servo_session`（9 例）与 `test_recovery_orchestrator` 必须仍绿

- [ ] **Step 5: Commit**

`refactor(servo): move the HOLD/recovery machine and worker to servo_recovery.cpp`

---

### Task 6: takeover 与服务下沉

**Files:**
- Create: `src/servo/servo_takeover.cpp`
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（`maybe_preposition`、`refuse_takeover_to_idle`、`handle_takeover`、`try_takeover_locked`、`handle_release`、`handle_pause`、`handle_resume` 及 `SessionHost` 的 9 个 override 改声明）
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: 上述 7 个成员 + `under_lock`、`sm`、`enter_hold`、`clear_session`、`arm_saturation_window`、`mark_dirty`、`cancel_preposition_unless_recovery`、`arm_recovery_cancel`、`measured_positions`、`verify_seed`、`rpc` 的 `override` 定义 — 全部签名不变

- [ ] **Step 1: 头文件改声明**，`SessionHost` 的 override 保持 `override` 关键字
- [ ] **Step 2: `servo_takeover.cpp` 写定义**
- [ ] **Step 3: CMake 追加源文件**
- [ ] **Step 4: 构建 + 全量测试**

Expected: `302 tests, 0 failures`；`test_servo_session` 仍 9/9

- [ ] **Step 5: Commit**

`refactor(servo): move takeover, preposition glue and service handlers to servo_takeover.cpp`

---

### Task 7: 头文件瘦身与自包含校验

**Files:**
- Modify: `include/lgw_bt_planner/servo/servo_node.hpp`（只留声明必需的 include）
- Create: `tests/test_servo_node_header_includes.cpp`（只 include 头文件、实例化一个指针，不链接行为）
- Modify: `CMakeLists.txt`（新增编译型检查目标）

**Interfaces:**
- Produces: 头文件从 2977 行降到 **≤ 500 行**（目标：声明 + 成员 + 注释）

- [ ] **Step 1: 记录瘦身前的行数**

Run: `wc -l include/lgw_bt_planner/servo/servo_node.hpp`
Expected: 先记下当前值（Task 6 后应约 700–900 行）

- [ ] **Step 2: 删除只有实现才需要的 include**

把 `servo_node_params.hpp`、`lookat_qp_solver.hpp`、`lookat_posture_family.hpp`、`target_selector.hpp`、`geometry_source.hpp`、`servo_control_law.hpp`、`servo_state_machine.hpp`、`move_group_prepositioner.hpp`、`tf2_ros/*` 等只在 .cpp 用到的头移到各自 .cpp；`std::shared_ptr<servo_params::ParamListener>` 用前向声明 `namespace servo_params { class ParamListener; }` 代替。**不动成员声明顺序**。

- [ ] **Step 3: 写自包含检查**

`tests/test_servo_node_header_includes.cpp` 内容只有：include 头文件 + 一个 `static_assert(sizeof(void*) > 0)` 或空的 `TEST`；CMake 里 `ament_add_gtest` 并将其链接到 `lgw_bt_planner_nodes`。目的是"任何遗漏的 include 都会在这个 TU 里暴露"，而不是靠实现文件碰巧带进来。

- [ ] **Step 4: 构建 + 全量测试**

Expected: build 无 error、无 `subobject-linkage`；`303 tests, 0 failures`

- [ ] **Step 5: 记录瘦身结果**

Run: `wc -l include/lgw_bt_planner/servo/servo_node.hpp src/servo/servo_node*.cpp`
Expected: 头文件 ≤ 500 行；四个 .cpp 各 300–900 行

- [ ] **Step 6: Commit**

`refactor(servo): slim servo_node.hpp to declarations and prove it is self-contained`

---

### Task 8: 真机冒烟（仅一个里程碑）

**Files:** 无改动

- [ ] **Step 1: 起驱动与栈**

Run: `bash scripts/start_lgw_arm.sh --driver-only`，随后按上次流程跑一轮 `real_walk_track.sh --seconds 60`（生产配速 0.9 / 1.0 / 0.8）。

- [ ] **Step 2: 判据**

Expected：`/lgw/servo/state` 进入 TRACKING；`hold episodes` 与本次基线（0 次）一致；`ros2 param list /lgw_servo_node` 仍是 99 个自身参数；启动日志与搬迁前的措辞一致（`lookat solver: qp` 等）。

- [ ] **Step 3: 收栈**

release → `bench_cleanup.sh`；驱动恢复 `speed_percent:=80 fast_mode:=false`。

---

## 交付判定

- `servo_node.hpp` ≤ 500 行，且不含任何函数体。
- 四个实现文件按职责分工，任何一个不超过 900 行。
- 每个任务独立提交，且该提交点的全量测试为绿（302，最后一个任务后 303）。
- 测试文件零改动；公共 API 零改动。
- `-Wsubobject-linkage` 警告消失。
