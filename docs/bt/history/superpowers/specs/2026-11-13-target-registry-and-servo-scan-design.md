# 目标注册表 + Servo 搜索跟踪（Search-and-Track）设计

- 日期：2026-11-13（**2026-11-15 修订 rev4**：扫描改为 j1 小摆 + j4/j5 花瓣
  （Lissajous）+ j6 滚转锁定的一条长轨迹，逐 waypoint 的 MoveGroup 规划、
  重试计数和冷却全部删除，见 §10.5）
- 范围：`lgw_3d_tracker`（Python，容器 Humble）+ `lgw_bt_planner` servo（C++）
- 目标：把"目标状态维护"和"伺服执行"彻底分开，让 tracker 成为唯一状态所有者，
  servo 变成忠实的跟随执行器，并支持"列表为空 → 扫描搜索"。
- 相关文档：
  - `docs/2026-11-13-detection-to-tracking-dataflow.md`（当前链路）
  - `docs/superpowers/specs/2026-10-19-lgw-3d-tracker-design.md`（旧 tracker 设计）
  - `docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md`（servo 接管）
  - `docs/superpowers/specs/2026-11-03-hold-recovery-design.md`（HOLD/recovery）

---

## 1. 背景与问题

当前链路中 tracker 和 servo 的职责重叠：

- tracker 维护 `tentative / confirmed / coasting` 生命周期，1.5 s 未观测就删除轨迹；
- servo 内部又有一套 `HEAD > ARM > ODOM` 几何源选择，以及
  `head_max_age_ms` / `arm_max_age_ms` / `odom_fallback_max_age_ms` /
  `upgrade_hysteresis_ticks` / `*_scale` 等计时和增益；
- servo 在每条 `UnifiedTrackArray` 上重新选靶，auto 模式无 ID 滞回；
- tracker 不保留丢失目标的记忆，无法在目标重新出现时复用 ID；
- 没有"列表为空 → 搜索"的行为。

结果：几何源选择、状态过滤、新鲜度判断分散在两个模块，逻辑臃肿；
"采集所有样品"的任务无法表达"已知目标列表 + 已采集标记"。

---

## 2. 目标（Goals）

1. **单一状态所有者**：tracker 维护目标注册表、可见性、丢失记忆和几何源选择；
   servo 只消费"当前可用目标"。
2. **目标注册表**：每个被观测过的目标有稳定 `track_id`；5 s 内可见的发布，
   超过 5 s 标记 `lost` 但保留最后 odom 状态；重新观测到相近目标时复用 ID。
3. **Tracker 固定频率发布**：即使没有新检测，也按固定周期发布当前目标状态
   （`position` + 当前时刻的 `position_arm_base`），servo 不再需要 freshness tier。
4. **Servo 变薄**：删除几何源 tier、计时和 scale；只做选靶 + 跟随 + 安全过滤。
5. **自动搜索**：列表为空时 servo 进入 `SCANNING`，用 MoveIt planner 执行
   j1/j5 周期扫描；发现目标后退出扫描、恢复 Servo 跟踪。
6. **任务层变薄**：只负责 Servo 开/关、Servo 与 MoveIt planner 的切换、
   以及采集完成后标记 `consumed`。

---

## 3. 非目标（Non-goals）

- 不做 head/arm 观测融合；固定优先级 `arm > head > 最后已知`。
- 不做类别语义/任务排序；任务就是"采集所有可能样品，选最近 id"。
- 不在臂基座系再加第二个 KF；odom KF 是唯一状态估计。
- 不修改 YOLOE 检测、相机驱动、TF 标定。
- 不在本设计内实现 MoveIt 抓取规划；任务层只调用现有 planner。
- 不在本设计内修 head/arm ~4 cm 视角相关偏差（列为前置校准项）。

---

## 4. 反目标（Anti-goals）

- servo 里再出现"哪个相机新鲜/哪个 tier 可用"的计时机器。
- tracker 变成任务规划器（决定采哪个、先采哪个）。
- 目标注册表无限增长；丢失目标按位置无门控地复活（ID 别名）。
- Servo 与 MoveIt planner 同时写 JTC（双写）。
- 为了"平滑"而在 tracker 里做多源加权融合。

---

## 5. 约束

- tracker：Python 3.10 / rclpy / Humble；纯算法部分必须可脱离 ROS 单测。
- servo：C++17 / rclcpp / MoveIt Servo；纯算法（状态机、选择、扫描轨迹）可单测。
- 现有 MoveIt Servo 配置是 `command_in_type: speed_units` + Cartesian twist；
  扫描首版走 MoveGroup planner（关节空间），不扩展 Servo 的 joint jog。
- `UnifiedTrack` 只被 tracker 和 servo 消费（已确认无其他消费者）。
- 跨发行版消息：Foxy 侧 `yoloe_interfaces` 不包含 `UnifiedTrack`，无需改。
- 真机安全：任何 MoveGroup 运动前必须确保 Servo 已 pause/release；扫描同理。
- 现有测试必须继续通过（tracker 28 项、servo 304 项）。

---

## 6. 已确定的设计决策（用户决定）

1. 几何源优先级 **arm > head > 最后已知**，不融合；有哪个用哪个。
2. 任务是采集所有可能样品，**不做类别区分**，未指定时选**最近**的 id。
3. `visible_timeout_ms = 5000`（5 s 未观测才从发布列表移除）。
4. id 变 `lost`/`consumed` 后，servo **自主重选或进入扫描**；发布列表有目标时
   退出扫描。任务决策层只负责 Servo 开/关和 Servo/planner 切换。
5. 扫描：**j1 水平 + j5 俯仰周期移动，j2/j3 抬高，其余关节不动**。
6. 扫描首版使用 **MoveIt planner 轨迹**。
7. 扫描属于 **servo 节点内部**，类似 `RECOVERING` 时由规划器接管。
8. **只保留 tracker 的状态过滤**；servo 忠实执行。
9. tracker 维护全局注册表：**不因丢失彻底删除**；`lost` 保留最后 odom 坐标；
   重新观测到相近目标时复用 id；只发布非 lost 的 id。
10. servo 里的 odom tier 过期、head/arm max_age 等计时逻辑**删除**，由 tracker 负责。

---

## 7. 总体架构

```
YOLOE ObservationResult（不变）
        │
        ▼
┌─────────────────────────────────────────────────────────────┐
│ lgw_3d_tracker（唯一状态所有者）                            │
│  ├─ 目标注册表：candidate / active / lost / consumed / archived │
│  ├─ odom KF（static 3D）+ 关联 + re-ID                      │
│  ├─ 几何源：arm > head > 最后已知（优先级门控更新，不融合）  │
│  ├─ 固定频率发布（30 Hz）：position + position_arm_base     │
│  └─ 服务：/lgw/tracker/mark_track (consumed/ignored/reactivate) │
└─────────────────────────────────────────────────────────────┘
        │  UnifiedTrackArray（只含 active）
        ▼
┌─────────────────────────────────────────────────────────────┐
│ lgw_bt_planner/servo（忠实执行器）                          │
│  ├─ 选靶：requested_id 优先，否则 nearest active            │
│  ├─ 控制：直接使用 position_arm_base，无 tier/计时          │
│  ├─ 列表为空 → SCANNING（MoveIt planner 接管）              │
│  ├─ 发现 active → 取消扫描、start/unpause Servo → TRACKING  │
│  └─ 安全过滤：TCP/关节完整、follower、限位、过近、HOLD      │
└─────────────────────────────────────────────────────────────┘
        ▲
        │ /lgw/servo/{takeover,release,pause,resume,state}
┌─────────────────────────────────────────────────────────────┐
│ 任务层（薄）                                                 │
│  ├─ Servo 开/关；Servo ↔ MoveIt planner 互斥切换            │
│  ├─ 采集完成 → mark_track(consumed)                         │
│  └─ 不参与目标选择/扫描                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 接口契约

### 8.1 `yoloe_interfaces/msg/UnifiedTrack.msg`

目标字段（保留诊断字段，控制契约只依赖加粗项）：

| 字段 | 类型 | 语义 |
|---|---|---|
| **`track_id`** | `uint64` | 稳定 id |
| **`task_state`** | `string` | `candidate / active / lost / consumed`；发布话题上只会出现 `active` |
| **`position`** | `Point` | odom 系 KF 位置 |
| **`position_arm_base`** | `Point` | **当前时刻**的臂基座位置（由 odom 状态 + 当前 TF 投影） |
| **`geometry_frame`** | `string` | `arm1/base_link` |
| **`geometry_valid`** | `bool` | TF 是否可用；false 时 servo 不驱动 |
| `class_name` | `string` | 诊断/可选过滤 |
| `confidence` | `float32` | 诊断 |
| `last_observation_stamp` | `Time` | 最后一次真实观测时刻 |
| `time_since_last_observation_ms` | `int32` | 距最后观测的毫秒 |
| `source` | `string` | `arm / head / odom / stale`（本次几何源，诊断） |
| `position_covariance` | `float64[9]` | odom KF 协方差（诊断） |
| `width_m/length_m/height_m/size_valid` | | 保留，任务层/抓取可用 |
| `last_bbox_2d / last_source_camera / last_bbox_color` | | 保留，诊断 |
| `age / observation_count / confirmed_hits` | | 保留，诊断 |

**移除**：`status`（由 `task_state` 取代）、`contributor_cameras`、
`contributor_positions_arm_base`、`contributor_stamps`、`contributor_confidences`。
逐相机信息移到 trace JSONL；控制不再消费。
（这是 builder call；见 §14。）

`UnifiedTrackArray` 字段不变：`header / wall_stamp / frame_stamp_ms / sequence_id / tracks`。
发布时 `header.frame_id = "odom"`。

### 8.2 新服务 `yoloe_interfaces/srv/MarkTrack.srv`

```
uint64 track_id
uint8 action          # 1=CONSUMED, 2=IGNORED, 3=REACTIVATE
---
bool success
string message
```

- tracker 提供 `/lgw/tracker/mark_track`；
- `CONSUMED`：移出发布列表，不参与 re-ID，不再发布；
- `IGNORED`：同上，但语义是"不要选它"（可保留诊断）；
- `REACTIVATE`：把 consumed/ignored 恢复为 active（可选，调试用）。

需要在 `yoloe_interfaces/CMakeLists.txt` 的 `rosidl_generate_interfaces` 中增加
`srv/MarkTrack.srv`，`package.xml` 增加 `rosidl_default_generators` 的 srv 依赖
（现有 msg 已使用同一宏，无需新增包依赖）。

### 8.3 QoS

- tracker 发布 `UnifiedTrackArray`：RELIABLE depth 10（不变）；
- servo 订阅：RELIABLE depth 10（不变）；
- 固定频率发布保证 servo `tracker_stream_timeout_ms=500` 不会误触发。

---

## 9. Tracker 设计

### 9.1 统一状态机

只保留一套状态，取代 `tentative / confirmed / coasting / deleted`：

```
candidate ──(hits >= candidate_min_hits)──▶ active
active ──(now - last_seen > visible_timeout_ms)──▶ lost
lost ──(re-ID 匹配)──▶ active
lost ──(now - last_seen > lost_ttl_ms)──▶ archived
任何非 consumed/archived ──(mark_track CONSUMED/IGNORED)──▶ consumed/ignored
```

| 状态 | 含义 | 是否发布 | 是否参与关联/re-ID |
|---|---|---|---|
| `candidate` | 新轨迹，命中数 < `candidate_min_hits` | 否 | 与普通观测关联 |
| `active` | 已确认，且 `now - last_seen <= visible_timeout_ms` | **是** | 是 |
| `lost` | 超过 5 s 未观测，保留状态 | 否 | **仅 re-ID 二级关联** |
| `consumed` | 外部标记已采集/忽略 | 否 | 否 |
| `archived` | lost 超过 TTL 或注册表超限 | 否 | 否 |

- `coasting` 删除：短时空档由 `active` 的 5 s 窗口覆盖；
- `candidate` 是误检门槛：单帧检测不会直接发布/参与跟踪；
- `lost` 的 KF **冻结**（不再 `P += Q`），保留最后 `x/P` 和 `last_seen`；
- 重新激活（re-ID）时，如果新观测与冻结状态距离 > `reid_reinit_distance_m`，
  用新观测重置 `x`、`P`，否则正常 update。

### 9.2 目标注册表

- 内存结构：`dict[track_id] -> Track`；`track_id` 单调递增，不复用；
- 每个 `Track` 保留：`state / x / P / last_seen / last_obs_by_camera / class / size /
  source_cameras / created_ms / consumed_reason`；
- `max_registry_size`（默认 100）：超过时优先淘汰最老的 `archived`，再淘汰最老的 `lost`；
- `lost_ttl_ms`（默认 60 s）：超过后 `archived`，不再自动 re-ID；
- `consumed` 不自动过期（可由 `REACTIVATE` 恢复）。

### 9.3 关联与 re-ID

每次检测消息到达时：

1. 普通关联：observations ↔ `active` + `candidate`（沿用两阶段 Hungarian、
   class 相同、15 cm 门控、4 cm 重复抑制）；
2. 未匹配观测 → **re-ID 二级关联**：
   - 只和 `lost` 轨迹比；
   - class 相同；
   - `|p_odom - lost.x| < reid_distance_m`（默认 0.3 m）；
   - `now - lost.last_seen < lost_ttl_ms`；
   - 多个候选时取最近；仍不唯一则不复活，按新 `candidate` 处理；
   - 匹配成功 → `state=active`、`last_seen=now`、按 §9.1 决定 update/reset；
3. 仍未匹配 → 新建 `candidate`（命中数 1）。

`consumed / archived` 不参与任何关联。

### 9.4 状态更新与发布时源选择

**估计更新：每收到一条 `ObservationResult` 就更新一次，不节流。**

- 每个检测消息（head 或 arm）到达时立即执行：
  `adapt → associate → KF update → lifecycle`；
- 两台相机是两条独立消息，**各自独立更新 KF**（这就是状态估计器的职责）；
- `last_obs_by_camera` 保存每相机最新观测（odom 点、arm-base 点、stamp、conf），
  供发布时选源和诊断；
- `dt_ms` 是"距上一条被处理观测"的时间，不是固定的 33 ms；
- `candidate_min_hits` 统计的是观测次数。如果两台相机同时看到同一目标，
  确认会更快（约 2 帧）；需要更稳时加 `candidate_min_age_ms=200`
  （从首次观测到确认至少 200 ms），见 §17.1。

**发布时源选择：arm > head > 最后已知，单源，不做融合。**

- 发布 timer（30 Hz）只做 format + publish，不参与关联/KF；
- 对每条 `active` 轨迹，按优先级选**新鲜**观测：
  - `arm` 有观测且 `now - stamp <= observation_max_age_ms(500ms)` → 用 arm；
  - 否则 `head` 新鲜 → 用 head；
  - 否则用 odom KF 的最后状态；
- `position_arm_base` 用当前 TF 投影：
  - arm/head：`T_base_odom(current)⁻¹ · p_obs_odom`（**原始观测**，不是 KF；
    投影只是为了在底盘移动时保持当前）；
  - 最后已知：`T_base_odom(current)⁻¹ · p_kf_odom`；
- `source` 记录 `arm / head / odom`；
- 这样 servo 拿到的是"某台相机的原始观测"或"最后已知状态"，不是两源融合值；
- `T_base_odom` 查不到时：`geometry_valid=false`，`position_arm_base` 保持上一次
  有效投影（或零），servo 看到 `geometry_valid=false` 时不驱动。

### 9.5 发布：观测驱动 + 心跳投影

- **估计更新**：每条检测消息到达时立即执行（§9.4）；
- **发布**：
  1. **事件驱动**：每次状态更新（每条 `ObservationResult` 处理完）立即
     format + 投影 + publish，不设人为上限；发布频率 = 观测频率
     （每路 8–10 Hz，合计 ~18–19 Hz，可能短时成对到达）；
  2. **心跳/投影定时器**：没有新观测时，按 `publish_heartbeat_hz`
     （默认 25 Hz）发布当前 `active` 列表，用当前 TF 重新投影
     `position_arm_base`，保证底盘移动时坐标保持当前、servo watchdog 不超时；
  3. **无目标时**仍发布空数组（心跳），servo 据此维持流存活并触发扫描；
- 发布不做关联/KF/生命周期，只做 format + 源选择 + 投影 + publish；
- 不设固定 30 Hz 上限：有新观测就立即发；只有"没有新观测"时才由心跳补；
- 发布只包含 `active`；`candidate/lost/consumed/archived` 不上线；
- trace JSONL 仍然保留每条观测的输入/关联/状态，扩展 `task_state` 字段。

> 为什么不做"无限快"：发布频率高于观测频率时没有新信息，只会增加
> DDS 序列化、TF 查询和 servo 侧的选靶回调负载；Jetson 上 YOLOE 已占
> ~1 核。观测驱动（~18–19 Hz）+ 25 Hz 心跳已经覆盖 servo 需要的所有时刻。

### 9.6 服务

`/lgw/tracker/mark_track`（`yoloe_interfaces/srv/MarkTrack`）：

- `CONSUMED`：`state=consumed`，从发布列表移除，不参与 re-ID；
- `IGNORED`：`state=consumed`（v1 与 CONSUMED 合并），同上；
- `REACTIVATE`：`state=active`（如果仍在注册表），`last_seen=now`；
- 未知 id / 非法 action → `success=false`。

v1 只实现服务与状态；**调用时机和空间抑制留给 BT 接入时决定**
（`consumed_suppress_enable=false`）。这样不会在任务决策逻辑还没接好时
提前引入"抑制半径/时长"的调参。

### 9.7 配置

`lgw_3d_tracker/config/runtime.yaml` 的 `unified_track` 段：

| 参数 | 默认 | 说明 |
|---|---|---|
| `candidate_min_hits` | 3 | 升 active 的命中数（按观测次数） |
| `candidate_min_age_ms` | 0 | 可选：首次观测到确认的最小时间（双相机加速确认时设 200） |
| `candidate_ttl_ms` | 2000 | candidate 超过此时长未确认 → 删除 |
| `visible_timeout_ms` | 5000 | active → lost |
| `lost_ttl_ms` | 60000 | lost → archived |
| `observation_max_age_ms` | 500 | 发布时选源的观测新鲜度门槛 |
| `publish_heartbeat_hz` | 25 | 无新观测时的心跳/投影频率；有新观测时立即发布 |
| `source_priority` | `[arm, head]` | 几何源优先级 |
| `reid_distance_m` | 0.3 | lost 复活距离门控 |
| `reid_reinit_distance_m` | 0.1 | 复活时重置 KF 的距离 |
| `max_registry_size` | 100 | 注册表上限 |
| `consumed_suppress_enable` | false | v1 关闭，等 BT 接入再开 |
| `consumed_suppress_radius_m` | 0.15 | consumed 空间抑制半径 |
| `consumed_suppress_ttl_ms` | 10000 | consumed 空间抑制时长 |
| `association_distance_gate_m` | 0.15 | 保留 |
| `duplicate_track_distance_m` | 0.04 | 保留 |
| `process_noise_position` | 0.05 | 保留 |
| `observation_noise_position` | 0.03 | 保留 |
| `two_stage.high/low_conf_threshold` | 0.40/0.15 | 保留 |

删除/废弃：`coast_max_age_ms`、`coast_enter_ms`（由 `visible_timeout_ms` 取代；
若保留旧 YAML 字段，加载时忽略并打印 warning）。

---

## 10. Servo 设计

### 10.1 消费契约

`TrackSnapshot` 简化为：

```cpp
struct TrackSnapshot {
  uint64_t track_id;
  std::string task_state;     // 只有 "active"
  tf2::Vector3 p_odom;
  tf2::Vector3 p_arm_base;
  bool geometry_valid;
  int64_t last_seen_ns;
};
```

- `make_snapshot` 直接读 `track.position_arm_base`、`track.geometry_valid`；
- 控制 tick 直接使用 `selected_snapshot_.p_arm_base`；
- 删除 `geometry_source.cpp` 在控制路径中的使用、`Contributor`、`select_source`、
  `head_max_age_ms`、`arm_max_age_ms`、`odom_fallback_max_age_ms`、
  `upgrade_hysteresis_ticks`、`head_scale`、`arm_scale`、`odom_scale`。
- `geometry_source_` 字段可以保留为诊断字符串，值取 `msg.source`。
- 保留 `geometry_frame` 硬校验：与 `base_frame_` 不一致 → FAULT/HOLD
  （不再只 WARN）。
- `geometry_valid=false` → HOLD（`HoldReason::kTrackerStale` 或新原因）。

### 10.2 选靶

`target_selector` 简化：

- `TrackCandidate`：`track_id / task_state / class_name / p_odom / last_seen`；
- `requested_track_id != 0`：
  - 找到该 id → 跟随（不检查 task_state，因为发布列表只有 active）；
  - 找不到 → 失败（`requested_missing`）并 HOLD，**不自动切换**到其他 id；
    re-ID 由 tracker 负责，等原 id 重新出现；
  - 删除 `reacquire_on_loss`（servo 不再做第二套重捕获）。
- `requested_track_id == 0`：
  - 只考虑 `task_state == "active"`；
  - 选 `|p_odom - p_base_in_odom|` 最小；
  - 没有 → `no_candidate`。
- 无类别过滤（任务不需要）；`class_filter` 可保留为调试参数，默认空。
- **扫描只在 auto 模式启用**；requested 模式列表为空时 HOLD。

### 10.3 目标切换

- 当选中的 `track_id` 变化时：
  - `TrackingLoop::reset()`（清 FF/控制律滤波）；
  - 可选：短暂 HOLD 100–300 ms 再跟（防止 setpoint 阶跃）；
  - 不切换 `geometry_source` tier（已删除）。
- 同一 id 的 `p_arm_base` 更新不重置。

### 10.4 状态机：新增 SCANNING

现有状态：`IDLE / PREEMPT / TRACKING / HOLD / RECOVERING / RELEASING / FAULT`。
新增 `SCANNING`，事件：

```
SCAN_START    // 节点决定开始扫描（列表为空 + auto_scan）
SCAN_FOUND    // 扫描期间发现 active 目标
SCAN_TIMEOUT  // 扫描超时
SCAN_FAIL     // MoveGroup 规划/执行失败
```

新增转移（只列新增，其余保持不变）：

| 当前 | 事件 | 目标 | 说明 |
|---|---|---|---|
| `HOLD`（`kTargetLost`） | `SCAN_START` | `SCANNING` | Servo 保持 paused；扫描 worker 直写 JTC |
| `SCANNING` | `SCAN_FOUND` | `PREEMPT` | 取消扫描；走现有 unpause/verify → TRACKING |
| `SCANNING` | `SCAN_TIMEOUT` | `HOLD` | 只在起点段失败/轨迹生成失败时发出；正常扫完不触发 |
| `SCANNING` | `SCAN_FAIL` | `HOLD` | |
| `SCANNING` | `RELEASE` | `RELEASING` | 取消扫描 |
| `SCANNING` | `PAUSE` | `SCANNING` | accepted no-op |
| `SCANNING` | `TARGET_OK/LOST` | `SCANNING` | latch only |
| `SCANNING` | `RESUME/TAKEOVER` | rejected | |

触发逻辑（安全门控）：

- **BT 启动且无目标（2026-11-14 补充）**：`/lgw/servo/takeover`
  (`track_id=0`) 在发布列表中没有 `active` 目标时**不再返回 `no_candidate`**，
  而是 `IDLE + TAKEOVER → PREEMPT + SCAN_START → SCANNING`，**立即**启动扫描
  （不等待 `scan_start_delay_ms`）。这条路径不要求 Servo 已 start（扫描由
  扫描 worker 直写 JTC 驱动，只有起点段用 MoveGroup）、也不要求 odom 基座
  位姿；requested id 缺失仍然失败，不会偷偷扫描。scan-only 会话用
  `scan_session_active_` 标记，允许在没有 tracker 流的情况下持续扫描；
  发现目标后回到正常的 stream/health 门控。
- **只允许从 `HOLD + HoldReason::kTargetLost` 进入扫描**（自动路径）；
  operator pause / follower lag / servo halt / pose stale / tracker stale
  一律不扫；
- 进入前必须满足：tracker stream alive、TCP/关节状态有效、
  `scan_require_base_static=true` 时底盘静止；否则保持 HOLD；
- `auto_scan=true` 且 `requested_track_id==0`（auto 模式）才自动触发；
  requested 模式列表为空时 HOLD，不扫描；
- `active` 目标为空持续 `scan_start_delay_ms`（默认 500 ms）后发 `SCAN_START`；
- `SCAN_FOUND` 判据：存在 `task_state=="active" && geometry_valid=true`
  且 `target_selector` 能选中的目标；仅"列表非空"不够；
- 扫描 worker 每 10 ms 检查 `scan_target_found_` / `scan_cancel_`（10 ms 轮询，
  发现即打断当前轨迹，不等它跑完）；
- 打断后先发一条定姿停止轨迹抢占 JTC，再走 `start_servo`/unpause、re-seed、
  零 twist 探测，成功后 `PREEMPT + UNPAUSE_OK → TRACKING`；
- **没有重试计数**：扫描只有三种结束方式——发现目标、任务层 release、起点段
  或轨迹生成失败（后者回 HOLD，可被下一次 `SCAN_START` 重新拉起）。

### 10.5 扫描轨迹与 worker

> **2026-11-15 改版（rev4，当前实现）**：扫描目的从「拼覆盖」明确为
> **扩大机械臂相机的有效视野、提高发现目标的概率**。实测该相机 RGB 为
> 640×480、内参 fx=604.7 / fy=603.7，视场只有 **55.8° × 43.4°**（不是 D435
> 深度传感器的 87°×58°），所以手腕每多扫一度都是净增益。

- 复用 `MoveGroupPrepositioner`（`move_to()` / `cancel()`）和 recovery worker
  的线程模式；
- **姿态**：j1 以 0 为中心慢摆 ±15°，j2/j3 固定在观测姿态（0.890 / -0.720），
  j6 由滚转锁定决定；j4/j5 各 ±60° 做花瓣扫描；
- **轨迹 = Lissajous（花瓣线）**：
  ```
  j1(t) = j1_rad + A1·sin(ωt)        A1=0.2618 (±15°), 1 周期
  j4(t) = A4·sin(2ωt)                A4=1.047  (±60°), 2 周期
  j5(t) = A5·sin(3ωt)                A5=1.047  (±60°), 3 周期
  ω = 2π / scan_period_s             scan_period_s=12 s
  ```
  2:3 的频率比让曲线闭合、处处平滑；**两个关节不会同时停下**，全程没有
  零速折返点（这正是旧栅格方案换向抖动的来源）；
- **下发方式**：一条轨迹覆盖整数朵花瓣（`scan_traj_duration_s=60`，5 朵），
  首尾位姿与速度完全一致；到点自动续发下一条，接棒前等真机追上当前姿态
  （≤0.5 s），每分钟一次；
- **限速**：`scan_traj_max_velocity_rad_s` 是硬上限；参数若导致超速，整朵花
  按时间**等比拉长**（形状不变），绝不削波；
- **滚转锁定（`scan_roll_lock_enable`）**：j4/j5 会把相机绕光轴滚转，检测器
  看到的物体是歪的。用 j6（转过几乎同一轴）逐点抵消，参考角取花朵中心的
  实测姿态。滚转对 j6 是非线性的，求解用**迭代牛顿**（单步会残留约 10°），
  实测全轨迹残差 0.1°；j6 限位 ±1.99 rad，超限时钳制；
- **运动模糊**：该相机曝光上限 10 ms（`rgb_camera.exposure` 范围 1–10000 µs，
  自动曝光也封顶在这里）。角速度 22–85 °/s，对应模糊 **2.5–10 px**（12 s 周期）；
  再快（8 s）会到 5–20 px；
- **起点段**：一次 MoveGroup 规划把臂送到花朵中心（j4=j5=0），碰撞检查保留
  在这里；`scan_velocity_scaling` / `scan_accel_scaling` / `scan_tolerance_rad`
  只作用于这一段；
- **单写者**：扫描期间 Servo 必须 paused，轨迹直发
  `/arm_controller/joint_trajectory`（与 Servo 同一条话题）；
- **停止**：目标出现 / release / 取消时先发 `scan_hold_s` 的定姿保持轨迹
  抢占 JTC，再走 re-seed → unpause 或回 HOLD；
- **没有尝试计数和重试冷却**：SCANNING 是状态，扫完一朵接着下一朵，直到
  目标出现或任务层 release；
- **碰撞检查**：当前没有 planning scene 维护者，除起点段外不做碰撞检查；
  j4/j5 的 ±60° 包络和相机 11.4 cm 偏置需现场确认无自碰撞；用软件关节
  限位兜底。

### 10.6 保留的安全过滤

- TCP/关节状态完整（`measured_complete_locked`）；
- `lookup_tcp_locked` 失败 → HOLD；
- follower lag → HOLD（现有）；
- 关节限位/奇异点/Servo halt → HOLD（现有）；
- `too_close` → 停止/保持（现有）；
- tracker 流 watchdog：事件驱动 + 心跳发布后仍保留 500 ms；
- `geometry_frame` 不一致 → HOLD/FAULT（硬失败）；
- `geometry_valid=false` → HOLD。

### 10.7 配置

`lgw_bt_planner/config/servo_node.yaml`：

删除：`head_max_age_ms`、`arm_max_age_ms`、`odom_fallback_max_age_ms`、
`upgrade_hysteresis_ticks`、`head_scale`、`arm_scale`、`odom_scale`。

新增：

| 参数 | 默认 | 说明 |
|---|---|---|
| `auto_scan` | true | 列表为空时自动扫描（仅 auto 模式 + HOLD/target_lost + 健康检查通过） |
| `scan_start_delay_ms` | 500 | 空列表持续多久后进入扫描 |
| `auto_resume_on_target` | true | HOLD 中出现可选目标时自动 RESUME |
| `scan_require_base_static` | true | 从 HOLD 自动进入扫描前要求底盘静止（BT 冷启动路径不经过这条门控） |
| `scan_j1_rad` | 0.0 | j1 摆动的中心 |
| `scan_j1_amplitude_rad` | 0.2618 | j1 ±15° 慢摆 |
| `scan_j1_cycles` | 1 | 每个花瓣周期内 j1 摆几个来回 |
| `scan_j2_rad` / `scan_j3_rad` | 0.890 / -0.720 | 观测姿态（现场校准值，不再是占位） |
| `scan_j4_amplitude_rad` | 1.047 | j4 ±60° 摆扫 |
| `scan_j5_amplitude_rad` | 1.047 | j5 ±60° 俯仰 |
| `scan_period_s` | 12 | 一朵花瓣的周期（曝光 10 ms 下模糊 2.5–10 px） |
| `scan_traj_duration_s` | 60 | 一条轨迹覆盖的时长（对齐整周期，5 朵） |
| `scan_traj_sample_rate_hz` | 50 | |
| `scan_traj_max_velocity_rad_s` | 2.5 | 硬上限；超速时整朵花按时间拉长 |
| `scan_roll_lock_enable` | true | 用 j6 保持画面竖直 |
| `scan_hold_s` | 0.20 | 目标/取消时的定姿停止轨迹 |
| `scan_velocity_scaling` | 0.15 | 只作用于起点段的 MoveGroup 规划 |
| `scan_accel_scaling` | 0.2 | 同上 |
| `scan_tolerance_rad` | 0.02 | 同上 |
| `consumed_suppress_enable` | false | v1 关闭，等 BT 接入再开 |
| `consumed_suppress_radius_m` | 0.15 | |
| `consumed_suppress_ttl_ms` | 10000 | |

~~`scan_timeout_s` / `scan_max_attempts` / `scan_retry_cooldown_s`~~ 已删除：
SCANNING 是状态而不是有界尝试，扫完一朵直接接着下一朵。

> 删除 `arm_scale` 后 arm 源等效增益为 1.0（原 0.7）；计划中必须用现有 bench
> 重新验证 `kp/v_max`，必要时把 0.7 折进控制增益（§17.2 第 6 条）。

---

## 11. 任务层（薄）

不在本次实现范围内，只固定接口：

- `/lgw/servo/takeover`、`/release`、`/pause`、`/resume`；
- `/lgw/tracker/mark_track`；
- 采集流程：servo 跟踪/对准 → 任务层 `pause` 或 `release` → MoveIt planner 抓取 →
  `mark_track(CONSUMED)` → 重新 `takeover`/`resume`；
- 任务层可以订阅 `/lgw/servo/state`（`track_id`、`geometry_source`、`hold_reason`）
  和 `/perception/tracks/unified`（目标位置）；
- 不允许 servo 与 planner 同时写 JTC。

---

## 12. 文件与改动清单（预估）

### 12.1 `yoloe_interfaces`
- `msg/UnifiedTrack.msg`：字段调整（§8.1）；
- `srv/MarkTrack.srv`：新增；
- `CMakeLists.txt` / `package.xml`：加入 srv 生成。

### 12.2 `lgw_3d_tracker`
- `unified_track/track.py`：统一状态机（candidate/active/lost/consumed/archived），
  删除 `TrackStatus` 的 tentative/confirmed/coasting；
- `unified_track/tracker.py`：注册表、re-ID、candidate 确认、lost 冻结、consumed；
- `unified_track/config.py` / `config/runtime.yaml`：新参数；
- `unified_track/input_adapter.py`：保留 TF 适配；增加 `source` 标记；
- `unified_track/output_formatter.py`：只发 active；`position_arm_base` 投影；
  删除 contributor 字段；
- `unified_track_node.py`：固定频率发布 timer、mark_track 服务、TF 投影；
- `unified_track/geometry_projector.py`（可选新文件）：`T_base_odom⁻¹ · p_odom`；
- `test/`：新增/更新单测；
- `scripts/`：`fake_track_publisher.py`、`analyze_tracker_trace.py` 适配新字段。

### 12.3 `lgw_bt_planner`
- `include/lgw_bt_planner/servo/servo_node.hpp`：删除 tier 相关成员；新增 scan 成员；
- `src/servo/servo_node.cpp`：`make_snapshot`、`on_track_array`、`reselect_locked` 简化；
- `src/servo/servo_tick.cpp`：直接使用 `p_arm_base`；删除 `select_source`；
- `src/servo/servo_recovery.cpp`：recovery target 改用 published `p_arm_base`；
- `include/lgw_bt_planner/servo_state_machine.hpp`：新增 `SCANNING` / scan 事件；
- `src/servo_state_machine.cpp`：转移表；
- `include/lgw_bt_planner/target_selector.hpp` / `src/target_selector.cpp`：
  `task_state`、无 class 过滤；
- `src/servo/servo_scan.cpp`（新）：扫描 worker / waypoint 生成；
- `include/lgw_bt_planner/servo/servo_scan.hpp`（新）：扫描轨迹纯函数；
- `src/servo/servo_node.cpp`：scan 触发、worker 启动/取消；
- `config/servo_node.yaml`：删 tier 参数、加 scan 参数；
- `launch/lgw_full.launch.py`：删除/透传 scan 参数；
- `tests/`：状态机、选靶、扫描轨迹、node 级测试；
- `CMakeLists.txt`：新增 scan 源文件/测试。

### 12.4 文档/脚本
- `docs/2026-11-13-detection-to-tracking-dataflow.md`：更新为新契约；
- `scripts/fake_track_publisher.py`：改发 `position_arm_base` / `task_state`；
- `scripts/bench_hold_recovery*.sh` / `real_walk_track.sh`：适配。

---

## 13. 测试策略（TDD）

### 13.1 Tracker 纯算法测试（pytest，不依赖 ROS）
- `test_task_state.py`：
  - candidate 命中 3 次 → active；
  - active 5 s 无观测 → lost；
  - lost 被 re-ID → active；
  - lost 超过 TTL → archived；
  - consumed 不发布、不 re-ID；
  - `mark_track` action 的状态变化。
- `test_source_priority.py`：
  - arm + head 同时新鲜 → 用 arm；
  - 仅 head → 用 head；
  - 都不新鲜 → 保持最后状态，`source=stale`；
  - 单条更新，不融合（断言 KF 输入）。
- `test_projection.py`：
  - 给定 `T_base_odom`，`p_arm_base = T⁻¹ · p_odom`；
  - 无新观测、底盘移动 → `p_arm_base` 随 TF 更新；
  - TF 缺失 → `geometry_valid=false`。
- `test_registry.py`：
  - 注册表容量淘汰；
  - re-ID 距离/类别/TTL 门控；
  - id 不复用。

### 13.2 Tracker 节点测试（rclpy，可选）
- 固定频率发布（假时钟/定时器）；
- 发布只含 active；
- `MarkTrack` 服务；
- QoS/消息字段。

### 13.3 Servo 纯算法测试（gtest）
- `test_target_selector`：requested/auto/empty、task_state、无 class；
- `test_servo_state_machine`：SCANNING 转移表（合法/非法事件）；
- `test_scan_trajectory`：waypoint 生成（j1/j5 范围、j2/j3 抬高、j5 不穿 0）；
- `test_tracking_loop`：目标切换 reset、geometry_valid=false 拒绝；
- `test_servo_node`：`make_snapshot` 字段、`on_track_array` 选靶、无 tier。

### 13.4 集成测试
- 假 tracker 发布 `UnifiedTrackArray` + 假 TF → servo 选靶/切换；
- 列表清空 → 状态机进入 SCANNING；
- 扫描期间注入 active target → SCAN_FOUND → TRACKING（用 fake MoveGroup）；
- 现有 304 项 servo 测试 + 28 项 tracker 测试必须通过（更新受影响的用例）。

---

## 14. Builder 需要确认/校准的点（我的 call 标注）

以下是设计文档里我做的判断，builder check 可以提出异议：

1. **`geometry_valid` 字段**：TF 不可用时发布 `geometry_valid=false` 并保留最后投影，
   servo HOLD；而不是不上线。**我的 call。**
2. **`contributor_*` 字段删除**：控制不再需要，逐相机信息进 trace JSONL。
   **我的 call。**
3. **只发布 active**：candidate/lost/consumed 不上线；`task_state` 在线上恒为 active。
   **我的 call。**
4. **固定频率 30 Hz 发布**：即使无新检测也发，用于实时投影和 servo watchdog。
   **我的 call。**
5. **`observation_max_age_ms=500`**：tracker 内部决定"这条观测是否可用于更新 KF"；
   servo 不再有任何 freshness 计时。**我的 call。**
6. **`mark_track` 放在 `yoloe_interfaces/srv`**：tracker 和 servo 都已依赖该包；
   如果 builder 认为加 srv 影响 Foxy/Humble 消息一致性，可改为
   `std_msgs/UInt64` 话题 + 无 ack。**我的 call，需 builder 确认。**
7. **扫描触发**：session active + 空列表持续 500 ms + `auto_scan=true`；
   同时保留手动 `/lgw/servo/scan` 服务（可选）。**我的 call。**
8. **扫描从 HOLD 进入**：TRACKING 先转 HOLD，再 SCAN_START；避免在跟踪中直接切扫描。
   **我的 call。**
9. ~~`scan_j2_raise_rad` / `scan_j3_raise_rad` 默认 0 是占位~~：2026-11-14 起
   改为 `scan_j2_rad=0.890` / `scan_j3_rad=-0.720`（与
   `posture_j2_nominal_rad` / `posture_j3_rad` 一致），已是真机验证值。
10. **`consumed` vs `ignored`**：首版可以合并为 `consumed`；`IGNORED` 作为同一状态。
    **我的 call。**
11. **`two_stage` 配置**：tracker 的 `TwoStageConfig` 保留用于关联阈值；
    不再与 YOLOE 的 two-stage 模式混淆。**我的 call。**
12. **`arm_mount_offset` 仍是单位阵**：本设计不修；如果 head 观测的 odom 位置有偏，
    会直接影响 `position_arm_base`。**前置校准项，不是本设计范围。**

---

## 15. 实施阶段（供 plan 使用）

1. **接口**：`UnifiedTrack.msg` + `MarkTrack.srv`；更新 `yoloe_interfaces` 构建。
2. **Tracker 纯算法**：统一状态机、注册表、re-ID、source priority、投影（TDD）。
3. **Tracker 节点**：固定频率发布、mark_track 服务、trace 适配（TDD）。
4. **Servo 纯算法**：target selector、snapshot、控制 tick 去 tier（TDD）。
5. **Servo 状态机**：SCANNING 转移表（TDD）。
6. **Servo 扫描 worker**：waypoint 生成、MoveGroup 执行、取消/超时（TDD + 集成）。
7. **端到端**：假 tracker + 假 TF + 假 MoveGroup；真机前先跑 mock。
8. **文档/脚本**：更新 dataflow、fake publisher、bench 脚本。

---

## 16. 风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| re-ID 别名 | 新目标复用旧 id | 距离/类别/TTL 门控；不唯一不复活 |
| 注册表增长 | 关联成本/内存 | `max_registry_size` + `lost_ttl` |
| 固定频率发布 + TF 查询 | CPU/阻塞 | 零超时 `canTransform`；缓存 TF；发布线程与回调分离 |
| `geometry_valid=false` | servo HOLD | 明确 HOLD 原因；TF 恢复后自动继续 |
| 扫描与 Servo 双写 | JTC 冲突 | 扫描前 pause Servo；worker 单写 |
| 扫描碰撞/奇异 | 硬件风险 | MoveIt 碰撞检查；j5 不穿 0；限速/超时 |
| 目标切换阶跃 | 控制瞬态 | 切换时 `loop_.reset()` + 可选短 HOLD |
| head/arm 视角偏差 | 切换跳变 | 前置校准；必要时 ramp |
| `arm_mount_offset` 未实测 | odom 位置偏 | 前置校准项；本设计不修 |

---

## 17. Builder check 结果

### 17.1 已在文档中定死（builder call）

1. **估计更新按观测到达触发；发布是事件驱动 + 心跳**：
   - 每条 `ObservationResult` 到达 → 立即 adapt/associate/KF update/lifecycle；
   - 两台相机是两条独立消息，各自更新 KF（这是状态估计器的职责）；
   - 每次状态更新后立即发布；没有新观测时由 `publish_heartbeat_hz=25`
     的心跳定时器补发（主要用于 TF 投影和 servo watchdog）；
   - 不设固定发布上限；发布不做关联/KF/生命周期；
   - 关联仍是每消息一对一 Hungarian；同一消息内的重复检测由 4 cm 抑制/去重处理；
   - `candidate_min_hits` 统计观测次数；两台相机同时看同一目标会加快确认
     （约 2 帧），需要更稳时加 `candidate_min_age_ms=200`；
   - `dt_ms` 用"距上一条被处理观测"的时间，不用固定周期。
2. **`candidate` 生命周期**：`candidate_ttl_ms=2000`；超过仍未确认 → 直接删除
   （不进 `lost`，不参与 re-ID）。注册表淘汰顺序：
   `archived`（最老）→ `lost`（最老）→ `candidate`（最老）。
3. ~~扫描超时/失败后的重试~~：2026-11-15 删除。SCANNING 是状态，扫完一朵
   直接接着下一朵，直到出现可选中的 active 目标或任务层 release；
   只有起点段/轨迹生成失败才回 HOLD。
4. **HOLD → TRACKING 自动恢复**：新增 `auto_resume_on_target=true`；
   当 `HOLD + TARGET_OK + geometry_valid` 时，节点内部发 `RESUME`
   （复用现有 re-seed/verify），不再要求外部服务调用。`kOperatorPause`
   的 HOLD 不自动恢复（操作员暂停优先）。
5. **`SCAN_FOUND` 判据**：存在 `task_state=="active" && geometry_valid=true`
   且 `target_selector` 能选中的目标；仅仅"列表非空"不够。
6. **扫描起始关节**：j4/j6 取 `/feedback/joint_states` 的**实测值**；
   扫描前复用 recovery 的 pre-settle：等待
   `recovery_presettle_gap_rad` / `recovery_presettle_stable_ms` 满足后
   再发第一个 MoveGroup goal，避免 JTC 以错误的起点拒绝/跳变。
7. **`reacquire_on_loss` 删除**：re-ID 已由 tracker 负责，servo 不再做第二套重捕获。
8. **consumed 空间抑制**：tracker 记录 consumed 的最后 odom 位置，在
   `consumed_suppress_radius_m=0.15`、`consumed_suppress_ttl_ms=10000` 内
   不把落在该范围内的同类检测建成新 candidate。任务层应在抓取闭合后**立即**
   调用 `mark_track(CONSUMED)`。
9. **`arm_scale` 0.7 → 1.0 的等效性**：arm 成为主源后 1.0 是目标值，但现有
   `kp/v_max` 是按 0.7 调的；计划中必须加一步"用现有 bench 重新验证或把 0.7
   折进控制增益"，不能默认直接提速 43%。

### 17.2 用户已确认的决定

1. **扫描进入门控**：同意。只允许 `HOLD + HoldReason::kTargetLost` 进入扫描；
   operator pause / follower lag / servo halt / pose stale / tracker stale
   一律不扫。扫描还要求 tracker stream alive、TCP/关节有效；底盘静止作为
   **默认安全要求**（`scan_require_base_static=true`），可显式关闭。
2. **扫描 j5 分支**：正负支都需要。因为用 MoveIt planner，j5 奇异影响不大；
   扫描轨迹覆盖 `[-1.0, -0.3]` 与 `[+0.3, +1.0]` 两个区间（中间留 0.3 rad
   避开精确奇异），j2/j3 抬高，j4/j6 取实测值。
3. **Planning scene**：当前没有维护者，v1 不依赖 planning scene；
   MoveIt 仍做关节限位/自身碰撞检查，但**不保证**桌面/狗身碰撞；
   因此扫描必须低速、软件关节限位兜底。底盘静止默认开启。
4. **requested id 模式**：同意。requested 模式 HOLD 等 re-ID，不自动切目标；
   只有 auto 模式才扫描/重选。
5. **consumed 时机**：先不处理，等接 BT 时再定决策逻辑。v1 实现
   `mark_track` 服务与 `consumed` 状态，但 `consumed_suppress_*` 默认关闭
   （`consumed_suppress_enable=false`），等 BT 接入时再开。
6. **arm_scale**：同意。arm 源用 1.0，并在计划里加一步用现有 bench
   重新验证 `kp/v_max`，必要时把 0.7 折进控制增益。
