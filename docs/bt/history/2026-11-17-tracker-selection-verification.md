# Tracker lost 退役与选择重构：验证记录

- 日期：2026-11-17
- 计划：`docs/superpowers/plans/2026-11-17-tracker-lost-retirement-and-selection.md`
- Spec：`docs/superpowers/specs/2026-11-17-tracker-lost-retirement-and-selection-design.md`

## 自动化 gate

- 容器内全量构建：
  `colcon build --packages-select yoloe_interfaces lgw_3d_tracker lgw_bt_planner`
  → `3 packages finished [2min 13s]`（`lgw_bt_planner` 有既有的
  `-Wmaybe-uninitialized` 警告，非本次引入）。
- 全量测试：
  `colcon test --packages-select lgw_3d_tracker lgw_bt_planner && colcon test-result --verbose`
  → **423 tests, 0 errors, 0 failures, 0 skipped**。
- 任务级 TDD：
  - tracker pytest 全量：55 passed（配置 legacy 警告为预期）。
  - servo gtest：`test_servo_scan_gate` 7/7，`test_param_contract` 5/5。

## 真机验证（2026-11-17 已完成）

环境：cameras + YOLOE 在 lo/domain 7，主栈在 domain 0，跨域桥自动启动；
trace 在 `.run/verify_20261117/`。

### 消息契约（Stage A）

- `selected_track_id` 非 0（选中最近的 `rocka`），`arm_window_ms == 3000`；
- 每条 active 都带 `arm_hits_in_window`（实测 0–15），`position_arm_base` 有效。

**类别名实测**：当前 YOLOE 输出 `rocka` / `rockb` / `wood`（不是旧文档的
`wood_block` / `red_block`）。`select_class_filter` 最终定为 `""`（三类全采）。

### 生命周期与选择（Stage B/D）

- **选择锁**：更近的 `wood`（XoY 0.93 m）出现时，`selected_track_id` 仍停在前一
  目标（`rocka`，1.02 m）。
- **TTL**：移出视野后轨道按"距最后观测 40 s"逐条消失；连续观测的 id 存活
  417 s 不误删。（2026-11-17 平台测试后，用户决定把该值收紧为 **15 s**；
  机制不变，单测已同步。）
- **陈旧重认**：id 6 断 9 s、id 55 断 26 s 后原位重认，ID 不变。
- **request / release / ignore**：`request(id)` 立即生效；`request(0)` 释放后下一
  tick 自动选最近的可用目标（场上仍有目标时观察不到 0，属设计）；`ignore` 后
  selected 清除，原位置 16 s 内未再生成新目标。
- **关联距离**：`matches[].dist` p50 0.0026 m、p90 0.0100 m、max 0.146 m →
  **0.15 m 门限合适**。

### Servo（Stage C）

- `takeover` → `TRACKING`，`ServoState.track_id` 跟随 `selected_track_id`；
- 60 s 内位置误差 8 mm、角度误差 ~0.03 rad、gap 0.001，**0 次 HOLD/FAULT**；
- 选择从 294 切到 308 时 servo 平滑切换（`reselect` 的 loop reset 生效），未进 HOLD。

### 0.234 m "跳变"调查（结论：两块石头，不是跳变）

- 机械臂从 tracking 姿态折叠到 `home`（关节 1.10→0.31、-0.75→-0.21）时，
  head 相机对同一石头的 `center_m` 波动仅 **1.1 mm** → 不是遮挡污染；
- 其他轨道位置最大变化 0.004–0.010 m → 没有 odom 整体跳变；
- trace 同一 tick 的 head 原始观测：
  `[('head','rockb',0.386,-0.049), ('head','rockb',0.532,-0.226)]`
  → **两块 rockb 相距 0.234 m**，tracker 正确建了两条 ID；
- 之后选中的 294 被手臂挡住（检测消失），40 s TTL 到期归档，选择切到一直可见的
  308。**关联门无需修改。**

### 已知既有问题（非本次引入）

- head/arm 跨相机一致性 p50 0.189 m（只有 33% 配对 ≤0.15 m）：标定/几何问题。
- dedupe churn：同一目标的"两个面"会建 candidate 再被同 tick dedupe 删除，
  不发布给下游，但会产生 trace 噪音。

## Review 修复记录（2026-11-17）

- **C1**：补提交 `CMakeLists.txt` 引用的 7 个源文件
  （`xacro_condition_filter.*`、`control_gripper_node.*`、`pose_error_gate.hpp`、
  `test_pose_error_gate.cpp`、`test_xacro_condition_filter.cpp`），
  HEAD 现在可以从干净检出构建。
- **I1**：servo 在 A→0（HOLD）→B 时重置 `TrackingLoop` 记忆。新增
  `ReselectResetsLoopAcrossGap`：先对旧条件观察到失败，改为
  `previous_track_id != selected_id` 后通过。
- **I2**：`scripts/fake_track_publisher.py` 设置 `selected_track_id`，
  bench/mock 的 takeover 不再退化为 scan-only。
- **I3/M6**：`MarkTrack.srv`（3 退役、4=REQUEST）与 `Takeover.srv`
  （`track_id`/`class_filter` deprecated）注释更新。
- **I5**：`_on_mark_track` 成功后立即发布，消除 MarkTrack→Takeover 的
  最长约 40 ms 竞态。
- **M5 覆盖**：新增"TF 丢失保持选择"、"容量不淘汰 selected"、
  "request 有效 id"三个测试。
- 回归：tracker pytest **58 passed**；servo `test_servo_scan_gate` **8/8**、
  `test_param_contract` **5/5**。
- 真机 trace 清单已执行，结果见上。
