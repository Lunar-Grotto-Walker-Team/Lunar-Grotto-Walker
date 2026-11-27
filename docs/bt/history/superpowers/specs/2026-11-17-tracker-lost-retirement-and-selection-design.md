# Tracker：lost 退役、15 s TTL 与单所有者目标选择

> 2026-11-17 真机平台测试后调整：active TTL 由 40 s 收紧为 **15 s**（用户决定）。

- 日期：2026-11-17
- 范围：`lgw_3d_tracker`（Python，容器 Humble）+ `lgw_bt_planner` servo（C++）+ `yoloe_interfaces`（消息）
- 目标：把 tracker 简化成"注册表 + 候选闸门 + 任务目标选择 + 单一生命周期"，servo 退化为忠实执行器；为后续 BT 靠近-核实-采集流程提供唯一的目标 id。
- 相关文档：
  - `docs/superpowers/specs/2026-11-13-target-registry-and-servo-scan-design.md`（本设计取代其 §5 状态机与 §7 选择部分）
  - `docs/superpowers/specs/2026-10-19-lgw-3d-tracker-design.md`（旧 tracker 设计）
  - `docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md`（servo 接管）
  - `docs/2026-11-13-detection-to-tracking-dataflow.md`（当前链路）

---

## 1. 背景与问题

当前 tracker 有 5 个状态（`candidate / active / lost / consumed / archived`，
`track.py:17-23`），`lost` 承担"身份跨遮挡保持"的职责，通过独立的 re-ID 通道
（`tracker.py:137-165`）在 60 s 内复活。经过 2026-11-17 的真机讨论，这个设计已经
名不副实：

1. **re-ID 门与主关联门已相同**：`reid_distance_m` 从 0.30 收到 0.10 后，
   re-ID 与普通关联的半径、类别判据完全一致，独立通道只剩"KF 重置方式"和
   "进不进 active 关联池"两个差异。
2. **导航期由任务决定，不由可见性决定**：目标在靠近途中看不见是正常的，
   旧的 `visible_timeout_ms = 8000` 会在途中把它踢出发布列表，BT 的目标凭空消失。
3. **任务判据在 arm 相机 + BT**："目标到底在不在"由到达后的近距离核实决定，
   与"多久没看见"不是一个量级的概念。
4. **选择逻辑在 servo**（`target_selector.cpp`），BT 没有可消费的目标来源，
   而即将加入的"发现目标 → 靠近 → 核实 → 采集"流程需要三层共用同一个目标 id。

因此：退役 `lost`，用单一滚动 TTL 代替；把选择收进 tracker；让 arm 核实成为
主释放路径，TTL 只作兜底。

---

## 2. 目标（Goals）

1. 状态机简化为 `candidate / active / consumed / archived`，删除 `lost`。
2. `active` 的生存期改为**距最后一次观测 15 s**（滚动计时），到期归档。
3. 单一关联门 `0.15 m`（主关联与 candidate 二次关联共用）；删除 `reid_distance_m`
   与 `reid_reinit_distance_m`。
4. 看不见的轨道冻结 predict；重新匹配时重置 KF（`x = 观测`，`P = 0.05·I`）。
5. tracker 维护一个 `selected_track_id`（任务目标）：XoY 平面距离到
   `arm1/base_link` 最近、类别过滤、排除 ignored；**选中后锁定**，只在 TTL 到期 /
   consume / ignore / request 时改变。
6. 发布全量 active 列表 + `selected_track_id`。
7. servo 只执行 `selected_track_id`，删除自己的 nearest / requested 选择逻辑；
   保留安全状态上报（过近 / 陈旧 / TF 无效），不得把安全状态伪装成"没有目标"。
8. arm 相机核实：3 s 窗口内同目标命中 ≥ 3 次为"有"；成功刷新位置并 consume，
   失败调用 ignore 并启用 `consumed_suppress`。

---

## 3. 非目标（Non-goals）

- 不实现 BT 树、`NavigateToTarget` 节点、Nav2 集成（后续阶段）。
- **不实现 arm 核实的判定与联调**：§12 的"到位 / 机械臂对准"前置条件、窗口启动
  时机与 consume/ignore 决策属于 BT 阶段。**tracker 侧的窗口计数在本次交付内**
  （见 §12）。本次交付 tracker + servo + 消息。
- 不做多目标任务排队；同一时刻只有一个 `selected_track_id`。
- 不改 servo 控制律、状态机（`IDLE / PREEMPT / TRACKING / HOLD / RECOVERING /
  SCANNING / RELEASING / FAULT`）与扫描逻辑。
- 不改 YOLOE 检测、相机驱动、TF 标定。
- 不做跨目标的全局排序（例如按价值/距离的复杂优先级）。

## 4. 反目标（Anti-goals）

- ❌ 不允许把"暂时看不见"重新包装成状态（`lost` / `coasting` / `visible=false` 状态）。
  可见性只能作为字段 `time_since_last_observation_ms`，存在于发布消息中。
- ❌ 不允许在任务进行中因"出现更近目标"或"出现新鲜目标"而切换选择。
- ❌ 不允许 tracker 在没有任何证据的情况下把目标判为"不存在"；absence 只能来自
  arm 相机核实（或 15 s TTL 这个兜底）。
- ❌ 不允许 servo 自行重选目标。

---

## 5. 约束

1. **部署**：容器 `self_l4t`，`/my_ws/l4t/...`；构建走
   `colcon build --packages-select ...`；运行前 source `l4t_setup.sh` 或缓存环境。
2. **消息契约**：`UnifiedTrackArray` 增加字段需要重建 `yoloe_interfaces` 及所有
   Python/C++ 依赖；tracker 与 servo 必须同批升级。
3. **BT.ROS2 / BT.CPP**：后续 BT 节点使用现有 `RosTopicSubNode` 模式。
4. **MoveGroup 假成功**：`ros2_control` 跑在 `FakeSystem` 上，`MoveGroup SUCCEEDED`
   不代表机械臂到位（见 `docs/2026-11-17-bt-motion-verification-and-srdf-branching.md`）。
   arm 核实的前置条件必须用真机反馈（`/feedback/joint_states` 或 tcp 位姿）判断。
5. **测试**：`colcon test` 可用；改状态机必须同步更新 `src/lgw_3d_tracker/test/*`。
6. **当前工作区**：`association_distance_gate_m` 已被临时改为 `0.10`、
   `reid_distance_m=0.10`、`reid_reinit_distance_m=0.0`（未构建）。本设计落地时
   统一改为 §10 的最终值，reid 两项删除。

---

## 6. 状态机

```
candidate ──hits>=3──▶ active ──距最后观测 15 s──▶ archived
    │                     │
    └──2 s 未转正──▶ archived
                          └──mark_track(consume|ignore)──▶ consumed
```

### 6.1 candidate

- 由"未匹配任何 active 轨道的观测"创建（仅当该观测未落入 0.04 m 重复抑制）。
- 出生 `hits = 1`。
- 转正条件（`track.py` 的 `tick()`，每个数据 tick 与 25 Hz 心跳都检查）：
  `hits >= 3`；匹配要求**同类** + 距 candidate 的 KF 位置 `< 0.15 m`。
- `candidate_ttl_ms = 2000` 内未转正 → `archived`。
- candidate 不发布。

### 6.2 active

- 唯一发布的状态。
- **`now - last_seen_ms > active_ttl_ms (15000)` → `archived`**。计时为滚动：
  任何一次成功关联（主关联或 candidate 二次关联之外的、对该轨道的观测更新）
  都会把 `last_seen_ms` 重置为当前时间。
- 心跳 tick 同样推进该计时，因此检测管线完全断流时也能正常归档。

### 6.3 consumed / archived

- `consumed`：由 `MarkTrack` 置位，`consumed_reason ∈ {consumed, ignored}`。
  不参与关联、不参与选择、不发布。
- **必须新增 `consumed_since_ms`（置位时刻）**：`consumed_suppress_ttl_ms` 与
  `consumed_ttl_ms` 都从该时刻起算，**不能用 `last_seen_ms`**——目标在 arm 核实前
  可能已经十几秒没被观测，用 `last_seen_ms` 会让抑制在 ignore 的瞬间就过期。
- `consumed` 保留 `consumed_ttl_ms (60000)` 后归档（仅用于抑制与诊断）。
  **本项为本设计的决定**；如后续需要更长的抑制记忆再调整。
- `archived`：下一 tick 从注册表移除，不可复活。

### 6.4 明确删除

- `lost` 状态、`lost_ttl_ms`、`visible_timeout_ms`。
- `Track.apply_observation()` 里的 `LOST → ACTIVE` 迁移。
- `MarkTrack` 的 action 3（reactivate）；consumed/ignored 为终态。
- `reid_distance_m`、`reid_reinit_distance_m` 及独立 re-ID 通道。

---

## 7. 关联与 KF

### 7.1 关联

- 池：全部 `active` 轨道（不再区分 active / lost 池）。
- 两阶段置信关联保持不变（`high = 0.45`，`low = 0.30`）。
- 约束：`class_name` 必须一致；3D 欧氏距离 `< 0.15 m`。
- 看不见的 active 轨道照常参与主关联：重新被命中即沿用原 ID，这就是原 re-ID 的功能。
- 超过 0.15 m 的位移按新目标处理（新建 candidate），这是门限收紧后的明确取舍。

### 7.2 KF 策略

- 静态模型（`A = I`）不变。
- **predict 仅在 `time_since_last_obs_ms <= predict_freeze_after_ms (1000)` 时执行**；
  超过则冻结 `x` 与 `P`。目的：避免 15 s 丢失期间协方差无意义增长，
  并消除"检测流是否还在"对 P 的隐式影响。
- 匹配成功时：
  - 若 `time_since_last_obs_ms > predict_freeze_after_ms` → **重置**
    `x = 观测位置`，`P = 0.05·I`；
  - 否则正常 `kf.update`。
- 原 `reid_reinit_distance_m` 的语义被"按陈旧时间重置"取代。

---

## 8. 目标选择

### 8.1 状态

tracker 维护 `selected_track_id: uint64`（0 = 无任务目标）。

### 8.2 选择规则（仅在 `selected_track_id == 0` 时执行）

候选轨道必须同时满足：

1. `task_state == active`；
2. `time_since_last_obs_ms <= selection_fresh_ms (1000)`（**新鲜度只在"从无到有"时要求**）；
3. 类别过滤 `select_class_filter`：**空字符串 = 允许所有 class**，非空则必须完全相等。
   空值只用于联调/诊断或检测端只输出一类目标时；正式采集任务应显式配置目标类别
   （当前现场 trace 中出现过的类别为 `wood_block` 与 `red_block`，默认建议
   `wood_block`）；
4. 状态不是 consumed / ignored；
5. **XoY 平面距离** `hypot(dx, dy)` 到 `arm1/base_link` 原点，位于
   `[select_min_distance_m (0.20), select_max_distance_m (0.0=不限)]` 内；
   距离取目标 KF 的 odom 位置与 TF `odom ← arm1/base_link` 的原点。
   注意 servo 的 `min_observe_distance_m (0.35)` 是**相机原点**到目标的观测距离，
   与选择门的基准不同；0.20–0.35 m 的目标会被选中，servo 照常跟踪并上报
   `too_close=true`，是否停止靠近由 BT 决定。

**TF 要求**：选择专用一条 `odom ← arm1/base_link` 的 lookup（不要复用
`unified_track_node.py` 现有的 `_lookup_odom_base()`，后者查的是本体 `base_link`）。
lookup 失败、或变换时间戳早于 `robot_pose_stale_ms (500)` 时：

- **不进行新的选择**（保持 `selected_track_id` 不变，0 则仍为 0）；
- 打限频 WARN，说明"无 arm 基座位姿，跳过选择"。

已选中目标的跟踪不受影响。`position_arm_base` 目前实际投影到本体 `base_link`
却标注为 `arm1/base_link`（`arm_mount_offset = 0` 时两者重合），该命名问题不在
本次范围内，但实现选择器时不得复用它。

在满足条件的候选中取 **XoY 距离最小**者。

### 8.3 锁定语义（关键）

一旦选中，`selected_track_id` **只在以下事件改变**：

1. 该轨道被 15 s TTL 归档；
2. 对该 id 调用 `MarkTrack(consume)` 或 `MarkTrack(ignore)`；
3. `MarkTrack(request, id)` 显式指定另一个 id。

其他任何情况都不改变：出现更近目标、当前目标暂时看不见、旁边出现新目标，
全部保持原选择。**这是任务锁，不是就近保持。**

**一致性要求**：任何把 active 轨道移出注册表的路径都必须维护 `selected_track_id`：

- `dedupe_tracks()` 合并时**优先保留 selected 轨道**（成熟度排序中 selected 最高）；
- 若 selected 仍被合并掉，`selected_track_id` 改为 `kept_id`；
- 容量淘汰：优先淘汰最老的、不可见（`time_since_last_obs_ms > predict_freeze_after_ms`）
  且**非 selected** 的 active，其次 candidate；不淘汰 selected、新鲜 active 或 consumed。

### 8.4 request

- `MarkTrack(action=4, track_id=id)`：若 id 存在且为 active、非 consumed/ignored，
  则设为 `selected_track_id`，返回成功；否则失败。
- **`MarkTrack(action=4, track_id=0)` = 释放选择**：清空 `selected_track_id`，
  不产生其他副作用（不归档、不抑制）。供 BT 中止任务时放手使用。
- request **不负责复活** consumed/ignored，也不创建新轨道。

### 8.5 无 target 的判定

- `selected_track_id == 0` 即"当前没有任务目标"，与列表是否为空无关。
- 发布列表可能包含多条 active（包括陈旧的），消费者用
  `time_since_last_observation_ms` 判断新鲜度，不得用"列表非空"代替。

---

## 9. 发布契约

### 9.1 消息

`yoloe_interfaces/msg/UnifiedTrackArray.msg` 增加：

```
uint64 selected_track_id   # 0 = 无任务目标
uint32 arm_window_ms       # arm 命中计数的窗口长度
```

`UnifiedTrack.msg` 增加：

```
int32 arm_hits_in_window   # 该轨道在最近 arm_window_ms 内的 arm 相机命中数
```

`task_state` 在线上只会出现 `active`（与现状一致）。
`time_since_last_observation_ms` 是新鲜度的唯一来源。

### 9.2 发布时机

- 每次观测更新后立即发布；心跳 25 Hz 持续发布（推进生命周期与投影）。
- 心跳必须继续存在：它同时是 servo 的消息到达看门狗（500 ms）的保活来源。

---

## 10. 参数

| 参数 | 旧值 | 新值 | 说明 |
|---|---|---|---|
| `candidate_min_hits` | 3 | 3 | 不变 |
| `candidate_min_age_ms` | 0 | 0 | 不变 |
| `candidate_ttl_ms` | 2000 | 2000 | 不变 |
| `visible_timeout_ms` | 8000 | **删除** | 由 `active_ttl_ms` 取代 |
| `lost_ttl_ms` | 60000 | **删除** | — |
| `active_ttl_ms` | — | **15000** | 距最后一次观测，滚动（真机后由 40 s 收紧） |
| `association_distance_gate_m` | 0.10（工作区） | **0.15** | 主关联 + candidate 共用 |
| `reid_distance_m` | 0.10 | **删除** | — |
| `reid_reinit_distance_m` | 0.0 | **删除** | 由"陈旧时重置 KF"取代 |
| `duplicate_track_distance_m` | 0.04 | 0.04 | 不变 |
| `predict_freeze_after_ms` | — | **1000** | 超过则冻结 predict |
| `selection_fresh_ms` | — | **1000** | 首次选择的新鲜度要求 |
| `select_min_distance_m` | 0.30（servo） | **0.20** | 迁入 tracker |
| `select_max_distance_m` | 0.0（servo） | **0.0** | 迁入 tracker，0 = 不限 |
| `select_class_filter` | 空（servo） | `""`（全采） | 迁入 tracker；实测类别为 `rocka` / `rockb` / `wood`，三类均为采集目标 |
| `arm_window_ms` | — | **3000** | arm 命中计数窗口（随消息发布） |
| `consumed_suppress_enable` | false | **true** | 拒绝后防原地重检 |
| `consumed_suppress_radius_m` | 0.15 | 0.15 | 不变 |
| `consumed_suppress_ttl_ms` | 10000 | 10000 | 不变 |
| `consumed_ttl_ms` | — | **60000** | consumed 轨道的保留时长 |
| `max_registry_size` | 100 | 100 | 不变 |
| `publish_heartbeat_hz` | 25 | 25 | 不变 |

`consumed_suppress` 的半径/TTL、`consumed_ttl_ms`、两个 1 s 阈值均为本设计的默认
决定，实施时可根据真机 trace 调整。

---

## 11. Servo 改造

### 11.1 行为

- 只跟随 `selected_track_id`；`0` 视为无目标（进入现有 HOLD/SCANNING 路径）。
- 选中 id 在数组中找不到 → 上报 `selected_missing`，**不重选**。
- 删除 `target_selector` 的 auto/nearest/requested 逻辑（`target_selector.cpp`）；
  `servo_node.yaml` 中 `target_min_distance_m` / `target_max_distance_m` 迁入 tracker，
  servo 侧相应参数废弃。
- **`Takeover.srv` 的 `track_id` / `class_filter` 废弃为兼容字段**：servo 忽略二者，
  只跟 `selected_track_id`；`try_takeover_locked()` 的 scan-only 判据从
  `request.track_id == 0` 改为 `selected_track_id == 0`。BT 要指定目标时先调
  `MarkTrack(request, id)` 再 takeover；类别过滤统一由 tracker 的
  `select_class_filter` 管理。
- 保留安全状态并如实上报，不得因为安全原因把目标当作"不存在"：
  - 距离过近（`min_observe_distance_m = 0.35`）→ `too_close`；
  - 目标陈旧（`time_since_last_observation_ms` 超阈值）→ 保持跟随/保持指向，
    通过状态上报陈旧，由 BT 决定是否继续；
  - TF 无效 → 照实上报，不切换目标。
- `ServoState.track_id` 继续上报当前跟踪的 id，供 BT/调试使用。

### 11.2 不变

- servo 状态机、控制律、扫描行为与 `/lgw/servo/pause`、takeover/release 语义。
- `tracker_stream_timeout_ms` 的现有值 **1200**（2026-11-16 调参，非本次引入），
  以及 `servo_node.yaml` 中的扫描参数（`scan_start_delay_ms 1500`、
  `scan_require_base_static false`、`scan_period_s 10`、scaling 1.0 等）保持原样。
  本设计不修改它们；此处记录实际值，避免 spec 与代码不一致。

---

## 12. Arm 核实（tracker 计数，BT 判定）

### 12.1 tracker 侧（本次交付）

- 每条 `active` 轨道维护一个 **arm 相机命中时间戳窗口**：`apply_observation()`
  且来源相机为 arm（`short_camera_name(frame) == "arm"`）时追加当前 wall-clock ms；
  发布前丢弃早于 `arm_window_ms (3000)` 的条目。
- 发布 `UnifiedTrack.arm_hits_in_window`（int32），窗口长度经
  `UnifiedTrackArray.arm_window_ms` 一并发布，消费者不得假定固定 3000。
- 计数只统计**已关联到该 track** 的观测。arm 相机看到但关联失败的检测不算，
  语义是"tracker 认为同一目标又被 arm 相机看到了几次"。

### 12.2 BT 侧（后续阶段）

前置条件（全部满足才开始采信计数）：

1. 导航到达容差内；
2. 机械臂到达观察位姿**且经真机反馈确认到位**（不能用 MoveGroup 返回码）；
3. arm 相机话题在窗口内确实有消息在流（按消息到达时间判断）。

判据 `arm_hits_in_window >= 3` → **有**：

- 用 arm 相机的近场位置刷新目标位置（tracker 的 `arm > head` 选源已自动完成）；
- BT 调用 `MarkTrack(consume)`。

否则 → **无**：BT 调用 `MarkTrack(ignore)`，由 `consumed_suppress` 压制原地重检。

窗口是滚动的，命中可能在 BT 到位前就已存在。BT 若要求"到位之后的新证据"，
可在到位时记录当前计数作为基线、要求后续增量 ≥ 3；该策略属于 BT 阶段。

---

## 13. BT 契约（后续实现）

- BT 每 tick 读取 `selected_track_id`，不得把 id 写入节点参数或静态变量。
- id 变化即任务中断信号：
  - `A → 0`：目标归档或被忽略，中止当前导航，回待机；
  - `A → B`：A 到期后 tracker 立即选中了新目标，**不能沿用 A 的导航目标**，
    必须重新评估。
- tracker 不提供冷却期，由 BT 负责处理 id 跳变。
- BT 主动查看 id 即"是否接活"，不需要额外的"暂停目标"开关；若需暂停跟踪，
  使用 servo 的 takeover/pause。

---

## 14. 影响面

| 文件 / 包 | 变更 |
|---|---|
| `yoloe_interfaces/msg/UnifiedTrackArray.msg` | 新增 `selected_track_id`、`arm_window_ms` |
| `yoloe_interfaces/msg/UnifiedTrack.msg` | 新增 `arm_hits_in_window` |
| `lgw_3d_tracker/unified_track/track.py` | 删除 LOST 相关；`tick()` 改为 active TTL；KF 冻结/重置规则 |
| `lgw_3d_tracker/unified_track/tracker.py` | 删除 lost 池与 re-ID 通道；新增选择器；`mark_track` 增加 request、删除 reactivate；生命周�� |
| `lgw_3d_tracker/unified_track/config.py` | 参数增删（见 §10） |
| `lgw_3d_tracker/unified_track_node.py` | 心跳、发布 `selected_track_id`、TF 查 arm base |
| `lgw_3d_tracker/unified_track/output_formatter.py` | 输出新字段 |
| `lgw_3d_tracker/config/runtime.yaml` | 参数同步 |
| `lgw_bt_planner/src/target_selector.cpp` / `include/.../target_selector.hpp` | 删除或退化为校验 |
| `lgw_bt_planner/src/servo/servo_node.cpp` | 只消费 `selected_track_id` |
| `lgw_bt_planner/config/servo_node.yaml` / `servo_node_parameters.yaml` | 选择参数迁移/废弃 |
| 测试 | `test_task_state.py`、`test_registry.py`、`test_tracker_dedupe.py`、`test_trace_logging.py`、`test_config.py`、`test_target_selector.cpp` 等同步更新 |

---

## 15. 验证计划

1. **单元测试**：
   - candidate → active → 15 s → archived 的计时；
   - 门槛 0.15 的关联与 candidate 转正；
   - 陈旧轨道匹配时 KF 重置；
   - 选择只在无目标时发生；选中后不因更近/更新目标改变；
   - request / consume / ignore / reactivate(应失败) 的行为；
   - `selected_track_id` 正确发布。
2. **Trace 验证**（真机或回放）：
   - `matches[].dist` 分布确认 0.15 合适；
   - 无重复 ID（同一物理目标不出现两条 active）；
   - 看不见的目标在 40 s 时准时归档；
   - 选中 id 在导航期间保持稳定。
3. **联调**：servo 在 `selected_track_id=0` 与 id 缺失时分别进入正确的 HOLD 原因。

---

## 16. 留给实施者的决定

- servo 侧"目标陈旧"的具体上报方式（复用 `hold_reason` 还是新增字段）；
- `consumed_ttl_ms`、两个 1 s 阈值的最终数值；
- trace 日志字段的增删（`task_state` 不再有 `lost`）；
- `position_arm_base` 的坐标系命名修正（本次不做，但不得在选择器里复用）。

arm 核实节点（扩展 `IsArmDetectionConfirmed` vs 新写）不在本次交付内，
留待 BT 阶段，见 §3 非目标。
