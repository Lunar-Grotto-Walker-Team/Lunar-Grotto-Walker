# lgw_bt_planner 共享库按子系统拆分

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `lgw_bt_planner_nodes`（42 个源文件、被 18 个测试链接）拆成按子系统的 4 个库，让"改 servo 只重链 servo 的测试"，且**不改变任何行为**。

**Architecture:** 依赖扫描（2026-11-12）显示现有依赖是无环 DAG：`bt_nodes → geometry`、`servo → lookat + geometry`、`lookat → (仅 piper_limits 头)`。据此拆成 `lgw_geometry` / `lgw_lookat` / `lgw_servo` / `lgw_bt_planner_nodes`（保留原名，只装 BT 节点与插件注册）。逐库下沉，每个任务一个提交，每个提交点全量测试必须为绿。

**Tech Stack:** CMake 3.22、colcon、gtest、BT.CPP 插件加载（`dlopen` + `share/<pkg>/bt_plugins`）。

**Spec:** 本文件；背景分析见会话记录 2026-11-12 与 `docs/superpowers/plans/2026-11-12-servo-node-header-split.md`。

## Global Constraints

- 容器内构建/测试：`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && <cmd>"`。
- **禁止** `pkill -f ros` / `killall`；按 PID `kill -15`。
- **行为零变化**：只动 CMake 目标划分与链接关系，不动任何 `.cpp/.hpp` 内容。
- 基线：**304 tests, 0 failures**；每个任务结束必须仍是 304/0。
- `lgw_bt_planner_nodes` 这个**名字必须保留**：BT.ROS2 的加载器按 `share/<pkg>/bt_plugins/*.so` 扫描并 `dlopen`，install 规则与目录名不能变。
- 头文件路径不变（仍在 `include/lgw_bt_planner/...`），因此**没有 `#include` 需要改**。

## Review Focus

1. **dlopen 的传递依赖**：`lgw_bt_planner_nodes.so` 里 dlopen 时，如果它链接了同包的其它 `.so`，加载器必须能在运行时找到它们。安装目录或 RPATH 处理错，BT 插件在真机启动时才会静默加载失败 —— 测试里不覆盖。
2. **静态初始化顺序**：BT 插件注册（`BT_REGISTER_NODES`）在 `lgw_bt_planner_nodes` 内；把辅助代码拆到别的库后，注册段必须仍随该库一起被 dlopen。
3. **`servo_node` 可执行文件的链接集**：它现在链 `lgw_bt_planner_nodes`；拆分后必须改为链它真正需要的（`lgw_servo`），否则又变成"改 servo 重编 BT"。
4. **测试的链接集**：每个测试只应链它被测模块所在的库；链多了等于没拆。
5. **未定义符号在链接期才暴露**：拆库时漏掉的依赖会在链接 `servo_node` / 测试时报 undefined，不会在编译期暴露 —— 每个任务必须真的链接过。

---

### Task 1: 拆出 `lgw_geometry`

**Files:**
- Modify: `CMakeLists.txt`（`NODE_SOURCES` 分组 + 新增 `add_library`）

**Interfaces:**
- Produces: `lgw_geometry`（STATIC），含 `calibration_json` `pose_string_parser` `compute_lookat_pose_node` `compute_arm_target_world` `compute_head_target_world` `compute_grasp_pose` `evaluate_detection` `make_pose_from_point_and_reference` `verify_head_arm_consistency` `select_arm_observation_pose_geom`

- [ ] **Step 1: 在 CMake 里把上述 10 个源文件从 `NODE_SOURCES` 移到新的 `GEOMETRY_SOURCES`，建 STATIC 库**

`add_library(lgw_geometry STATIC ${GEOMETRY_SOURCES})`，沿用现有的 `target_include_directories` / `ament_target_dependencies`（照抄 `lgw_bt_planner_nodes` 里这些文件需要的那部分依赖：rclcpp、yoloe_interfaces、geometry_msgs、sensor_msgs、tf2 等）。

- [ ] **Step 2: `lgw_bt_planner_nodes` 链接 `lgw_geometry`**

- [ ] **Step 3: 构建 + 全量测试**

Run: `colcon build --packages-select lgw_bt_planner && colcon test ... && colcon test-result --verbose`
Expected: build 无 error；`304 tests, 0 failures`

- [ ] **Step 4: Commit** `refactor(build): split the geometry helpers into lgw_geometry`

---

### Task 2: 拆出 `lgw_lookat`

**Files:** Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `lgw_lookat`（STATIC），含 `lookat_solver` `lookat_posture_family` `lookat_qp_solver`

- [ ] **Step 1: 建 `LOOKAT_SOURCES` + `add_library(lgw_lookat STATIC ...)`**

依赖：`orocos_kdl` `kdl_parser` `Eigen3` `qpmad`（照抄这些源文件当前在 `lgw_bt_planner_nodes` 里用到的那几项）。

- [ ] **Step 2: `lgw_bt_planner_nodes` 链接 `lgw_lookat`；把只测 lookat 的测试改链 `lgw_lookat`**

测试清单：`test_lookat_posture_family` `test_lookat_qp_solver` `test_lookat_solver_select` `test_lookat_solver_ab` `test_qpmad_smoke`。

- [ ] **Step 3: 构建 + 全量测试** — Expected: 304/0
- [ ] **Step 4: Commit** `refactor(build): split the look-at solvers into lgw_lookat`

---

### Task 3: 拆出 `lgw_servo`

**Files:** Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `lgw_servo`（STATIC），含 `servo/*`（node/tick/recovery/takeover/session/state_reporter/tracking_loop/recovery_orchestrator）、`servo_control_law` `servo_state_machine` `target_selector` `geometry_source` `preposition_decision` `hold_recovery_decision` `move_group_prepositioner`

- [ ] **Step 1: 建 `SERVO_SOURCES` + `add_library(lgw_servo STATIC ...)`，`target_link_libraries(lgw_servo lgw_lookat lgw_geometry lgw_posture lgw_preposition servo_node_params)`**
- [ ] **Step 2: `lgw_bt_planner_nodes` 与 `servo_node` 可执行文件改链 `lgw_servo`**

`servo_node` 可执行文件从此不再依赖 BT 那一坨。

- [ ] **Step 3: 12 个 servo 测试改链 `lgw_servo`**

`test_servo_params` `test_servo_control_law` `test_tracking_loop` `test_tracking_loop_replay` `test_recovery_orchestrator` `test_qp_j5_sweep` `test_servo_state_reporter` `test_servo_session` `test_param_contract` `test_servo_node_header_includes` `test_servo_node_tick` `test_servo_state_machine` `test_hold_recovery_decision` `test_preposition_decision` `test_move_group_prepositioner` `test_target_selector` `test_geometry_source`（按实际测的模块归组，链接多了也算偏差）。

- [ ] **Step 4: 构建 + 全量测试** — Expected: 304/0
- [ ] **Step 5: 记录重链面积**

Run: 改 `src/servo/servo_tick.cpp` 后构建，数 `Linking CXX` 行数
Expected: 明显少于 21（目标 ≤10）

- [ ] **Step 6: Commit** `refactor(build): split the servo subsystem into lgw_servo`

---

### Task 4: 验证 BT 插件加载未受影响

**Files:** 无改动（只验证）

- [ ] **Step 1: 起真机栈（不 takeover）**

`docker exec -d ... ros2 launch lgw_bt_planner lgw_full.launch.py follow:=true use_bt:=true use_rviz:=false`

- [ ] **Step 2: 判据**

Expected：`bt_runner` 启动日志里插件加载无 `Failed to load` / `dlopen` 错误；`ros2 node list` 含 `bt_runner`；`/lgw/servo/state` 为 IDLE。

- [ ] **Step 3: 收栈**

`bench_cleanup.sh`

---

## 交付判定

- 4 个库目标，依赖方向 `bt_nodes → servo → lookat → geometry` 单向成立。
- `servo_node` 可执行文件不依赖 BT 库；servo 测试不依赖 BT 库。
- 改 servo 源码触发的重链数量 ≤10（拆分前 21）。
- 全量测试 304/0 不变；真机 BT 插件加载正常。
