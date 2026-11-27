# agx topic-based 硬件接口 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use `- [ ]`.

**Goal:** 用 topic-based ros2_control 插件替换 FakeSystem：状态来自 `/feedback/joint_states`，命令发到 `/control/joint_states`；`hardware_backend` 默认 `fake`，可一键回退。

**Architecture:** 新包 `lg_walker_ws/src/agx_topic_hardware`（C++ `SystemInterface`）。插件内部起一个 rclcpp 节点（订阅反馈、发布命令）+ 独立 executor 线程；`read/write` 只做缓冲拷贝。

**Tech Stack:** ros2_control `hardware_interface`、rclcpp、pluginlib、sensor_msgs/JointState。

**Spec:** `docs/superpowers/specs/2026-11-06-agx-topic-hardware-design.md`
**回退记录:** `docs/dev/2026-11-06-vendor-modifications-and-rollback.md`

## Global Constraints

- `hardware_backend` 默认 `fake`；任何提交都不得改变默认启动行为。
- 插件不修改 agx 驱动；不改 Servo/控制律参数（JTC 容差除外）。
- 真机验证前先在 bench（无驱动 / 假反馈）跑通。
- vendor 文件的每次修改都要同步更新回退文档。

---

### Task 1: 包骨架与插件注册

**Files:**
- Create: `src/agx_topic_hardware/package.xml`、`CMakeLists.txt`、`agx_topic_hardware_plugin.xml`
- Create: `src/agx_topic_hardware/include/agx_topic_hardware/agx_topic_system.hpp`
- Create: `src/agx_topic_hardware/src/agx_topic_system.cpp`（空实现，先能加载）

- [ ] **Step 1**: 写 CMake：`add_library(agx_topic_hardware SHARED src/agx_topic_system.cpp)`，链 `hardware_interface::hardware_interface`、`rclcpp`、`pluginlib`，`ament_export_libraries` + `pluginlib_export_plugin_description_file`。
- [ ] **Step 2**: `plugin.xml`：`<class name="agx_topic_hardware/AgxTopicSystem" type="agx_topic_hardware::AgxTopicSystem" base_class_type="hardware_interface::SystemInterface"/>`。
- [ ] **Step 3**: `colcon build --packages-select agx_topic_hardware`，`ros2 plugin list`（或 `ros2 control list_hardware_components`，需 Manager）确认类可发现。
- [ ] **Step 4**: commit（`feat(lgw): scaffold topic-based agx hardware plugin`）。

### Task 2: read/write 与线程

**Files:**
- Modify: `src/agx_topic_hardware/src/agx_topic_system.cpp`
- Test: `src/agx_topic_hardware/test/test_topic_system.cpp`

**Interfaces:**
- `on_init(info)`: 读 `<param name="state_topic">`（默认 `/feedback/joint_states`）、`command_topic`（默认 `/control/joint_states`）；按 `info.joints` 建 state/command 索引。
- `read(time, period)`: `try_lock` 取最新 JointState，按名字填 `state_interfaces_[i].set_value(pos/vel)`；无消息时保持上一次。
- `write(time, period)`: 组 `JointState`（name=关节名, position=命令值, velocity=0）发布。

- [ ] **Step 1**: 写失败测试：喂一条 JointState（joint1=1.0）→ `read()` 后 state 接口 = 1.0；`write()` 发出的话题消息 position = 设定值。用 rclcpp 节点 + executor 在测试里跑。
- [ ] **Step 2**: 实现 `read/write` 与订阅/发布；`try_lock` 失败沿用旧值。
- [ ] **Step 3**: 测试通过；`colcon build` + 直接跑 gtest 二进制。
- [ ] **Step 4**: commit。

### Task 3: xacro 后端选择（默认 fake）

**Files:**
- Create: `src/agx_topic_hardware/urdf/agx_topic.ros2_control.xacro`
- Modify: `agx_arm_moveit/config/agx_arm.urdf.xacro`（vendor；更新回退文档）

- [ ] **Step 1**: 新宏内容与 `agx_arm.ros2_control.xacro` 相同，只是 `<plugin>agx_topic_hardware/AgxTopicSystem</plugin>` 并带 `state_topic`/`command_topic` 参数。
- [ ] **Step 2**: 在 `agx_arm.urdf.xacro` 增加 `<xacro:arg name="hardware_backend" default="fake"/>`，`<xacro:if value="$(eval hardware_backend == 'topic')">` 时 include 新宏，`<xacro:unless>` 时 include 原宏。
- [ ] **Step 3**: 用 `xacro` 直接渲染两个取值，确认 plugin 名不同。
- [ ] **Step 4**: commit + 更新回退文档。

### Task 4: launch 参数透传

**Files:**
- Modify: `agx_arm_moveit/launch/_moveit_config_builder.py`、`demo.launch.py`（vendor）
- Modify: `src/lgw_bt_planner/launch/lgw_full.launch.py`

- [ ] **Step 1**: builder 里 `urdf_mappings["hardware_backend"] = LaunchConfiguration("hardware_backend").perform(context)`；demo.launch 声明该参数（默认 `fake`）。
- [ ] **Step 2**: `lgw_full.launch.py` 加同名参数并传给 demo.launch 的 `launch_arguments`。
- [ ] **Step 3**: `ros2 launch ... hardware_backend:=fake` 启动，确认加载的仍是 `mock_components/GenericSystem`（日志 `Loading hardware 'FakeSystem'`）。
- [ ] **Step 4**: commit。

### Task 5: JTC 容差

**Files:**
- Modify: `agx_arm_moveit/launch/demo.launch.py` 的 `_build_ros2_controllers_file`（vendor；只在新后端下追加）

- [ ] **Step 1**: 当 `hardware_backend == topic` 时给 `arm_controller` 加：
  ```yaml
  constraints: {goal_time: 2.0, stopped_velocity_tolerance: 0.2}
  allow_nonzero_velocity_at_trajectory_end: true
  ```
  每个关节 trajectory 0.5 / goal 0.15。
- [ ] **Step 2**: 生成文件后 `cat` 确认 YAML 合法；fake 后端不带这些键（保持原样）。
- [ ] **Step 3**: commit。

### Task 6: bench 验证（无真机）

- [ ] **Step 1**: 起 `ros2_control_node`（topic 后端）+ 一个发布 `/feedback/joint_states` 的假源；JTC 激活。
- [ ] **Step 2**: 用 `ros2 action send_goal /arm_controller/follow_joint_trajectory` 发一个小轨迹；验证命令话题收到位置、状态接口跟随反馈。
- [ ] **Step 3**: 记录结果到 spec §5。

### Task 7: 真机验证与回退

- [ ] **Step 1**: `hardware_backend:=topic` 起栈，确认**起栈无运动**、takeover 正常。
- [ ] **Step 2**: 跑 120 s 假目标跟踪；确认恢复不再 FAULT（spec §5 第 3 条）。
- [ ] **Step 3**: `hardware_backend:=fake` 再起一次，确认与改动前行为一致。
- [ ] **Step 4**: 更新回退文档与 spec 结果；commit。
