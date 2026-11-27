# 2026-11-17 机械臂 BT 到位判据、MoveGroup 假成功与 SRDF 分支覆盖

范围：`lgw_bt_planner`（BT 动作节点 + named target 加载器）。
触发事件：UnloadHandle 树连续两次"成功"或"失败"得莫名其妙——一次全绿但机械臂没动，
一次卡在 park 且 j4 差了 0.411 rad。本文记录根因、修复与验证数据。

---

## 1. 核心结论（先看这段）

**`MoveGroup` 返回 `SUCCEEDED` 不等于机械臂到位。**

`ros2_control` 的硬件插件是 `FakeSystem`（mock），它把收到的指令直接镜像成状态，
于是控制器永远"执行成功"；而 MoveGroup 的终点判定完全依赖控制器的返回，从不读真机反馈。

```
MoveGroup ──指令──▶ arm_controller(JTC) ──指令──▶ FakeSystem ──/control/joint_states──▶ 驱动 ──▶ 真机
                                                                                          │
真机状态 ──/feedback/joint_states──▶ (仅供 move_group 当"当前状态") ◀─────────────────────┘
```

指令单向 feed-forward，真机反馈只喂给规划起点，**没有接回执行判定**。

启动时的 remap 印证了两套状态源是分开的：

```
move_group         -r joint_states:=feedback/joint_states    ← 规划看真机
ros2_control_node  -r joint_states:=control/joint_states     ← 执行看 FakeSystem
```

### MoveGroup 自带的监控为什么失效

| 机制 | 检查内容 | 在我们的栈里 |
|---|---|---|
| `allowed_start_tolerance` | 轨迹起点 vs 当前状态 | 用 `/feedback/joint_states` → **真** |
| `execution_duration_monitoring` | 实际时长 vs 规划时长×1.2 + 0.5 s | FakeSystem 秒级走完 → 必过 |
| 控制器 action 返回 | 控制器自报 SUCCEEDED | 控制器跑在 FakeSystem 上 → 必过 |

即：起点检查是真的（所以不会从错误状态出发），**终点检查纯属"听控制器汇报"**。

---

## 2. 事故一：全绿但机械臂没动

现象（15:28 那一轮）：19 节点树 `Tree finished with status: SUCCESS`，5 段 MoveArmToPose
全部 `MoveGroup SUCCEEDED`，但日志里的实测 tcp 位姿一个 bit 都没变：

```
目标 [0.0605, 0.2129, 0.1230]  实测 [0.2038, 0.0000, 0.2592]   err = 0.2905 m
目标 [0.0502, 0.2163, 0.0019]  实测 [0.2038, 0.0000, 0.2592]   err = 0.3696 m
目标 [0.2709, 0.2163, 0.0019]  实测 [0.2038, 0.0000, 0.2592]   err = 0.3428 m
目标 [0.0496, 0.2163, 0.0019]  实测 [0.2038, 0.0000, 0.2592]   err = 0.3698 m
目标 [0.0605, 0.2129, 0.1230]  实测 [0.2038, 0.0000, 0.2592]   err = 0.2905 m
```

根因：`move_arm_to_pose_action_node.cpp` 的旧实现只把 `||pos_err||` 写进 INFO 日志，
然后**无条件 `return SUCCESS`**。判据根本不存在，误差再大也放行。

同时排除了一个错误假设：`/compute_ik` 不能当可达性判据。它把 `above_handle`
判成无解，可同一轮 MoveGroup 明明规划并执行成功（`||pos_err||=0.0005 m`）。
IK 服务返回 false 只说明 KDL 在超时内没搜到解，不代表不可达。

---

## 3. 事故二：park 时 j4 差 0.411 rad 仍报成功

现象：树跑完，机械臂停在 park，但 `joint4 = 0.411 rad`（指令 ≈ 0）。

驱动侧证据：

| 字段 | 值 | 含义 |
|---|---|---|
| `ctrl_mode` | 1 | `CAN_CTRL`，正常控制模式 |
| `teach_status` | 2 | `STOP_RECORDING`（示教**已结束**，不是正在示教） |
| `motion_status` | **1** | `REACH_TARGET_POS_FAILED` |
| `err_status` | 0 | 无故障 |
| `joint_angle_limit[]` | 全 false | 没有关节撞限位 |

结论：机械臂执行了指令但有一轴没走到位，驱动自己**知道**失败（`motion_status=1`），
而 BT 完全不知情——因为 `MoveArmToNamed` 的旧实现只按 MoveGroup 的 result code 返回，
连 tcp/关节反馈都不读。

补充：`/feedback/*` 值三次采样逐位相同，是因为机械臂静止，不是反馈冻结——CAN 总线
2 秒内 RX +6300 帧/TX +1627 帧，链路是活的。

---

## 4. 事故三：SRDF 模板按臂型互相覆盖

`NamedTargetLoader` 读的是 **`agx_arm.srdf.xacro`（模板）**，不是生成的 `.srdf`
（后者是 install 时的陈旧产物，只有 `home` 一条）。模板里 `home` 在每个臂型分支
各有一份：

```
<xacro:unless value="$(eval arm_type == 'nero')">   ← piper: joint1..joint6
<xacro:if     value="$(eval arm_type == 'nero')">   ← nero : joint1..joint7
```

旧实现用正则直接扫全文、不判断分支，导致同名状态**后者覆盖前者**：`home` 最终变成
nero 版本，带着一个真机不存在的 `joint7`。bt_runner 日志因此是 `Loaded 7 named targets`。

影响面：
- MoveGroup 会忽略不在规划组内的关节，所以运动本身没出错；
- 但新加的关节校验会因 `joint7` 不在 `/feedback/joint_states` 里而误判失败；
- 更根本的是：**任何臂型的同名目标都会互相覆盖**，换臂型时会静默用错值。

---

## 5. 事故四（安全）：停链 / 跑测试会把机械臂带到全零位

两次观察到全零位形（`[0,0,0,0,0,0]`）：

1. 栈运行中执行 `colcon test` —— `session_launch_test.py` 会拉起一个 `servo_node`
   往真实话题发指令（CLAUDE.md 里"真机 + mock 绝对禁忌同启"那条）。
2. 单纯停链之后 —— 同样落到全零。

全零位是**危险位形**：驱动源码 `_move_home_callback` 注释写明 joint3 的 0 就是它的
上限（URDF `[-2.967, 0]`），压在上限上固件可能跌入 protective/limp 状态。

因此现在的操作纪律：

- 跑全量测试前必须 **停链** 且 `/control_enable` 置 false；跑完恢复。
- 停链后机械臂会去全零位，重新起栈会把它带回 park（j2=0.30, j3=-0.20）——起栈后先看一眼位形再触发树。

---

## 6. 修复清单

### 6.1 `MoveArmToPose`：位置 + 姿态双判据

- 新头文件 `include/lgw_bt_planner/pose_error_gate.hpp`：纯函数契约
  - `error_within_tolerance(err, max)`：`max<=0` 关闭；NaN 视为失败
  - `rotation_angle_error_rad(q1, q2)`：`2·acos(|q1·q2|)`，`q` 与 `−q` 视为同一旋转
  - `pose_within_tolerance(pos_err, rot_err, max_pos, max_rot)`
- `src/move_arm_to_pose_action_node.cpp:409-427`：采样 `/feedback/tcp_pose` 后比对，
  任一超差 → `FAILURE` + ERROR（打印目标/实测位姿，提示去查 `arm_status.motion_status`）
- 新增端口：`max_pos_err_m`（默认 **0.005 m**）、`max_rot_err_rad`（默认 **0.05 rad ≈ 2.9°**）

### 6.2 `MoveArmToNamed`：逐关节判据

- `src/move_arm_to_named_action_node.cpp:105-172`：MoveGroup 成功后等 300 ms 稳定，
  采样 `/feedback/joint_states`，逐关节比对
- 新增端口：`max_joint_err_rad`（默认 **0.05 rad**）
- 反馈里不存在的关节 → WARN 跳过；一个都校验不到才失败
- 选关节角而不是位姿：目标是 SRDF 的关节值，逐关节比对更直接也更严格
  （位姿需再调 FK 服务，多一个失败点）

### 6.3 SRDF 分支过滤

- 新模块 `include/lgw_bt_planner/xacro_condition_filter.hpp` +
  `src/xacro_condition_filter.cpp`
  - 支持 `$(eval ident == 'v')`、`!=`、`and` 链
  - **无法识别的条件一律判 false**（fail closed，避免漏进别的分支）
  - 递归处理嵌套 `<xacro:if>` / `<xacro:unless>`，剥掉标签本身
- `src/named_target_loader.cpp:58` 在正则解析前调用过滤
- `src/bt_runner_node.cpp:61-63` 新增参数：
  `srdf_arm_type`(piper) / `srdf_effector_type`(agx_gripper) / `srdf_revo2_type`(left)

---

## 7. 验证数据

修复后同一棵 12 节点树（UnloadHandle）实跑：

```
[bt_runner] Loaded 4 named targets from SRDF
            (arm_type=piper, effector_type=agx_gripper, revo2_type=left)

init_move_home     → named target reached: worst joint 'joint5' off by 0.0093 rad (max 0.05)
move_to_above      → err = 0.0002 m
move_to_at_handle  → err = 0.0006 m
move_to_pulled     → err = 0.0004 m
move_to_pushed_back→ err = 0.0005 m
return_to_park     → named target reached: worst joint 'joint6' off by 0.0093 rad
Tree finished with status: SUCCESS
```

对照项：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| named target 数量 | 7（全臂型混在一起） | 4（piper + agx_gripper） |
| `skipping joint 'joint7'` | 每轮 2 次 | 0 次 |
| 位置误差可见性 | 只有日志 | 超 5 mm 即 FAILURE |
| 关节误差可见性 | 无 | 超 0.05 rad 即 FAILURE |

测试：`colcon test` 全包 **366 tests / 0 failures**，其中新增 17 例
（`tests/test_pose_error_gate.cpp` 10 例、`tests/test_xacro_condition_filter.cpp` 7 例）。

---

## 8. BT 判据 vs 真实 hardware_interface（架构对比）

今天的修复是"在 BT 层补判据"。另一条路是把 ros2_control 的硬件插件从 `FakeSystem`
换成真机接口：订阅 `/feedback/joint_states` 作为 state interface，把 command 转发到
`/control/joint_states`。

| | BT 层判据（现状） | 真实 hardware_interface |
|---|---|---|
| 改动量 | 小，两个节点 | 大，需写 `SystemInterface` 插件 |
| 生效范围 | 只有显式比对的节点 | 全局：JTC 自身就会检测跟踪误差 |
| 错误传播 | 节点返回 FAILURE | JTC 返回 `PATH_TOLERANCE_VIOLATED`，MoveGroup 监控恢复有效 |
| 风险 | 每个新节点都要记得加 | 单写者、200 Hz 反馈延迟、使能状态都要处理 |
| 适合 | 现阶段快速可用 | 做真正可靠的自动流程时 |

**建议**：当前保留 BT 判据；若要长期自动运行，优先做硬件插件，那时 BT 判据可以退化成
兜底（或者干脆去掉）。

---

## 9. 遗留问题

1. **`MoveArmToNamed` 的判据依赖目标表**。若目标表里含有组外关节（如本次 `joint7`），
   该关节会被跳过而不是报错。臂型参数填错时可能会静默少校验几个关节。
2. **`max_pos_err_m` / `max_rot_err_rad` / `max_joint_err_rad` 的默认值**是按今天实测
   （0.2–0.6 mm / 8–9 mrad）留了约 2–8 倍余量，机械臂负载或速度变化后需要复核。
3. **停链导致全零位**的机制还没定位到具体代码路径（是 JTC 析构、还是 JSB 末帧），
   目前靠"起栈后确认位形"绕过。
4. `session_launch_test.py` 在真机栈运行时依然会干扰真实话题，属于测试环境隔离问题，
   未修，只加了操作纪律。

---

## 10. 复现 / 检查命令

```bash
# 机械臂真实状态（判断 BT 的"成功"是否可信）
docker exec self_l4t bash -c '
  source /my_ws/l4t/.l4t_env.sh
  ros2 topic echo /feedback/joint_states --once
  ros2 topic echo /feedback/arm_status --once    # 看 motion_status
'

# 驱动外部控制闸门（徒手摆位 / 跑测试前关闭）
ros2 service call /control_enable std_srvs/srv/SetBool "{data: false}"

# 触发树
ros2 action send_goal /lgw_bt_execution btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: UnloadHandle}" --feedback

# 判据是否触发
grep -aE "pose not reached|named target not reached|named target reached" \
  /tmp/lgw_chain_*.log | tail
```
