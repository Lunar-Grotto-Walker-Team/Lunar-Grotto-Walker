# agx 机械臂 topic-based 硬件接口（设计）

- 日期：2026-11-06
- 状态：设计（待评审），实现未开始
- 相关：`docs/dev/2026-11-06-vendor-modifications-and-rollback.md`（回退记录）

## 1. 动机（真机事故）

现在的 MoveIt 链路把 **FakeSystem 的"状态"当指令**：

```
JTC → FakeSystem(命令镜像成状态) → JSB → /control/joint_states → agx 驱动 → 真机
```

三个已验证的后果：

| 现象 | 根因 |
|---|---|
| 起栈时机械臂被命令走（j3 压限位 → 保护失能 → 腕部砸落） | FakeSystem 初始状态来自 `initial_positions.yaml`，与真实姿态无关 |
| 恢复流程 `re-seed did not take (probe pose mismatch)` → FAULT | MoveGroup 在假系统上瞬间"完成"，真实臂仍在动（~283 ms 延迟） |
| MoveIt 的碰撞检查基于假状态 | 状态接口 = 命令镜像，不是真实反馈 |

## 2. 架构

新增 ros2_control `SystemInterface`（`agx_topic_hardware`）：

```
/feedback/joint_states ──订阅──► [read()]  状态接口（position/velocity）
/arm_controller 命令 ◄──[write()]── 发布 ──► /control/joint_states ──► agx 驱动
```

- `read()`/`write()` 在 controller_manager 的 200 Hz 实时线程里**只做内存拷贝**；
  DDS 收发在插件内部的独立 executor 线程，缓冲用 mutex 保护；
- 关节映射：joint1..joint6 + gripper（与驱动 `/control/joint_states` 一致）；
- **首个 write 不会发零位**：JTC 激活时以当前状态为保持点，插件只在 JTC 真正
  写命令时发布。

### 开关（回退方案）

`hardware_backend` 参数，默认 **`fake`**：

```bash
ros2 launch lgw_bt_planner lgw_full.launch.py hardware_backend:=topic  # 新插件
ros2 launch lgw_bt_planner lgw_full.launch.py                          # 默认=旧路径
```

默认值即回退，不需要改文件；vendor 侧只增加参数与宏选择。

## 3. 对 MoveIt / Servo 的兼容性

- **MoveIt 规划**：当前状态来自真实反馈，碰撞检查第一次有意义；
- **JTC**：轨迹执行按真实状态判到达。**必须放宽容差**（默认 path 0.1 rad 会在
  283 ms 延迟下 abort 每条轨迹）：
  ```yaml
  constraints:
    goal_time: 2.0
    stopped_velocity_tolerance: 0.2
    <joint>: {trajectory: 0.5, goal: 0.15}
  allow_nonzero_velocity_at_trajectory_end: true
  ```
- **Servo**：`joint_topic` 指向真实状态；奇异/限位判断基于真实关节；
- **恢复流程**：MoveGroup 的"完成"= 真实到达，`verify_servo_seed` 的探针不再
  因延迟误判。

## 4. 实时性

- `read/write` 不阻塞：`try_lock` 失败沿用上一帧；订阅回调用 `last_msg` 覆盖；
- 命令发布频率 200 Hz（与今天 JSB 相同），驱动行为不变；
- 若 controller_manager 的 RT 线程无法保证（本机已警告 FIFO 不可用），插件
  退化行为是"偶尔发旧帧"，不会引入新的阻塞点。

## 5. 验收

1. **起栈不产生运动**：`hardware_backend:=topic` 启动后，关节值与启动前一致；
2. **轨迹执行真实完成**：一次 MoveGroup 移动后，`/feedback/joint_states` 与
   目标差值 ≤ 0.05 rad，且 `arm_controller` 的 goal 状态为成功；
3. **恢复不再 FAULT**：连续 3 次 `servo_halt` 恢复，探针不报 mismatch；
4. **Servo 跟踪**：假目标 60 s，HOLD 次数 ≤ 基线（live 换支轮次：7/120 s）；
5. **回退**：`hardware_backend:=fake` 启动，行为与本文档之前完全一致。

## 6. 非目标

- 不改 agx 驱动本身（它仍然是话题级控制）；
- 不引入实时内核补丁；
- 不改变 Servo/控制律参数（除 JTC 容差）。

## 7. 风险

| 风险 | 影响 | 对策 |
|---|---|---|
| JTC 容差调不好 | 轨迹 abort、恢复失败 | 先在 bench 用假反馈跑通，再上真机 |
| 实时线程里 DDS | 抖动 | 独立线程 + try_lock + 缓冲 |
| 夹爪/手部关节映射 | 命令丢失 | 按驱动 `/control/joint_states` 的全关节表映射 |
| vendor 修改丢失 | 更新厂商包后失效 | 回退文档 + 开关默认 fake |
