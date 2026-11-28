# user-log.md — 本仓库的本地改动记录

> **路径说明**：本文写于代码收编到主仓库之前，路径按当时的工作区 `nav_ws/go2-nav2-terrain-pointlio` 记录。代码现在位于 `src/base/`、`src/nav/` 与 `src/frontier_exploration_ros2/`，构建改在主仓库里做。

上游:`frontier_exploration_ros2`(Mert Güler)。本文件只记录**我们在这里做的
本地改动**, 上游 README / docs 保持原样。

关联文档:`l4t/nav_ws/go2-nav2-terrain-pointlio/debug-log.md`
(2026-11-06 自主探索时 `map->odom` 反复回退 + costmap 走廊重影)。

---

## 1. 2026-11-06 提升 frontier goal 的可达率(回退 + clearance + 可达性 BFS)

### 背景

Go2 + Unitree L2 + point_lio/SLC + Nav2(terrain costmap) 这套栈上自主探索时:

- global costmap 上同一条走廊被画成**多条方向不同的走廊**(重影), 重启 Nav2
  又恢复正常;
- 狗经常在原地反复"重置 / 回访", 同一个 frontier 被反复选中、到不了。

排查结论(详见上述 `debug-log.md`): 触发源不是链路带宽, 而是**探索点质量**——
frontier 的派发点贴在已知/未知交界, Nav2 规划或接近到点时失败, 进而触发
恢复行为(清 costmap / spin / back up), 而 `TraversabilityLayer` 只增量写入、
不清理当前栅格之外的旧格, 于是旧对齐的墙被留在原地, 和新的观测叠成"多条走廊"。

### 改动前的取点逻辑

- frontier 点本身是**未知格**(`is_frontier_point`, 要求 `NoInformation` 且邻接 free);
- 候选 goal 取该 frontier 格的 **free 邻格**, 规则是"离 frontier 簇质心最近"
  (`build_frontier_candidate`);
- 派发直接使用这个点(`frontier_position()` 返回 `goal_point`);
- 校验只有两条: map 里 free、global/local costmap `cost < occ_threshold`。

也就是说 goal **永远贴着未知区一格**, 常常落在膨胀带外沿或地形层 lethal 边缘上;
整条链路没有回退、没有 clearance、也没有路径可达性检查。

### 改动

1. **goal 往已知区回退**
   `goal_backoff_from_frontier_m`: 沿"机器人 → goal"方向把派发点往回推,
   再在 `goal_backoff_search_radius_cells` 半径内吸附到最近的合格格。
   - 回退量有上限: 不会把点推到机器人身后(至少保留
     `max(0.5, candidate_min_goal_distance_m)` 余量);
   - 回退失败时退回原 goal, 但仍需通过下面的检查。

2. **clearance 检查**
   `goal_clearance_check_enabled` / `goal_clearance_radius_cells`:
   goal 周围邻域内不允许有 lethal 格或 costmap 阻塞格。

3. **可达性 BFS**
   `goal_reachability_check_enabled`: 从机器人所在格在 map 网格上做一次
   4 邻域 BFS, 通行判据 = map free 且 global costmap `cost < occ_threshold`
   (与候选筛选完全一致)。走不到的候选**直接丢弃**, 不再派发注定失败的目标。
   BFS 每轮搜索只算一次, 按起点格缓存, 多个候选共享。

4. **等价判定容差**
   `frontier_signature()` / `are_frontiers_equivalent()` 的容差改为
   `frontier_visit_tolerance + goal_backoff_from_frontier_m`。
   回退让派发点随机器人位置轻微漂移(方向指向机器人), 容差不放宽会让同一个
   frontier 被判成"换了点", 触发多余的预抢占和压制。

改动文件:

| 文件 | 内容 |
|---|---|
| `include/frontier_exploration_ros2/frontier_search.hpp` | 新增 5 个 `FrontierSearchOptions` 字段 + 可达性 BFS 接口 |
| `src/frontier_search.cpp` | 回退/clearance/可达性实现 + `goal_cell_acceptable` |
| `include/frontier_exploration_ros2/frontier_explorer_core.hpp` | 对应 `FrontierExplorerCoreParams` 字段 |
| `src/frontier_explorer_core_init.cpp` | 参数 → `FrontierSearchOptions` |
| `src/frontier_explorer_node.cpp` | 新 ROS 参数声明/读取 |
| `src/frontier_explorer_core_frontiers.cpp` | 等价容差计入回退距离 |
| `config/params.yaml` | 新参数文档(默认全关) |
| `test/test_frontier_snapshot_and_search.cpp` | 新增 2 个用例 |

### 参数

默认值保持**旧行为**(全部关闭), 需要显式开启:

```yaml
goal_backoff_from_frontier_m: 0.0     # 0 = 不回退
goal_backoff_search_radius_cells: 3
goal_clearance_check_enabled: false
goal_clearance_radius_cells: 2
goal_reachability_check_enabled: false
```

Go2 实机配置(`l4t/nav_ws/go2-nav2-terrain-pointlio/src/go2_navigation2/config/frontier_go2_params.yaml`)
已开启:

```yaml
goal_backoff_from_frontier_m: 0.5     # 12 m 雷达量程下几乎不损失视野增益
goal_backoff_search_radius_cells: 3   # 0.2 m 池化 → 0.6 m 吸附半径
goal_clearance_check_enabled: true
goal_clearance_radius_cells: 2        # 0.4 m
goal_reachability_check_enabled: true
```

### 验证

- 容器内 Release 构建通过(`colcon build --packages-select frontier_exploration_ros2`)。
- 单测 **136 个全部通过**, 新增两个:
  - `GoalBackoffMovesDispatchPointAwayFromBoundary` — 回退后 goal 离机器人明显
    更近, 且仍落在 free 格;
  - `ReachabilityCheckDropsFrontierBehindBlockedDoorway` — costmap 封死走廊中点
    后, 门后的 frontier 不再被派发。
- 实现过程中修掉一个方向 bug: 回退方向一开始写成"远离机器人", 会推到未知区,
  被新测试直接抓到。

### 后续 / 注意

- **尚未真机验证**: 下一次探索要同时看 `/explore/selected_frontier`(新 goal
  应明显离开未知边界)和 `map->odom` 跳变次数(恢复风暴减少后应同步下降)。
- 语义变保守: 被膨胀层封死的门洞后面的 frontier 会被直接丢掉。如果真机上
  发现可走的窄门被误判, 把 `occ_threshold` 从 65 放到 70~75。
- 可达性 BFS 的代价与 map 格数成正比, `/exploration_map` 若达到 1000×1000
  量级, 可视情况把 `map_processing_rate_hz` 降到 0.5。

---

## 2. 2026-11-11 干跑(dry_run)+ 探索完成去抖 + 三个 bug 修复

### 背景

为了在**离线 bag + 隔离域**里检查自主探索选点的有效性, 加了一个 `dry_run`
模式: 照常算 frontier、照常发 `/explore/frontiers` 与
`/explore/selected_frontier`, 但**不调用 `navigate_to_pose`**,
`goal_in_progress` 始终为 false, 于是每个地图 tick 都会重新选点(1 Hz)。
配合 `l4t/nav_ws/go2-nav2-terrain-pointlio/scripts/large/run_large_test.sh
--frontier-dry-run --rviz` 使用, 见该工作区 `scripts/large/README` 与
`frontier_dry_run.py` / `frontier_dry_run.rviz`。

### 新增参数

```yaml
dry_run: false                              # 只显示/记录选点, 不发 Nav2 action
exploration_complete_confirm_scans: 5       # 连续 N 次无 frontier 才判定完成
exploration_complete_min_known_cells: 500   # 且地图已知格数达到该门限
```

### 干跑过程中修掉的三个 bug

1. **"探索完成"一次锁存, 之前会永久停死**
   `return_to_start_on_complete: false` 时, `handle_exploration_complete()`
   会把 `return_to_start_completed` 置位, 而 `try_send_next_goal()` 开头就
   return —— 只要**任何一次**扫描没有 frontier(启动空地图、SLAM 重定位、
   costmap 清空), 之后永远不会再探索, 日志只有一句 `No more frontiers found`。
   现在:
   - `dry_run` 模式完全不锁存;
   - 正常模式要求**连续 `exploration_complete_confirm_scans` 次**无 frontier,
     并且地图已知格数 ≥ `exploration_complete_min_known_cells`, 才判定完成
     (完成后打 `Exploration finished: no frontier for N consecutive scans`);
   - 中间态打 `No frontiers in this scan (k/N)` 与"地图还太小"(每次 streak
     只打一次)。

2. **可达性 BFS 的种子格取错**
   `build_frontier_candidate()` 的 `start_cell` 是 frontier 簇的**第一个未知格**
   (用于 MRTSP 的 `start_world_point`)。第一版直接拿它当 BFS 种子 →
   起点不可通行 → 可达集为空 → **所有候选被丢弃**, 现象是
   `frontier_counts: raw=0` / RViz 上一个 frontier 都没有。
   现在由 `get_frontier()` 额外传入机器人侧的 free 格(`free_point`)作为种子。

3. **clearance 把 NO_INFORMATION 当障碍**
   Go2 的 global costmap 把未观测格写成 255(int8 读出来是 -1),
   第一版 clearance 用 `global_cost_blocked`(>= occ_threshold)会把 goal 周围
   的未知区当成墙, 4~5 格邻域内只要有一片未知就整片否决。新增
   `FrontierSearchContext::global_cost_obstacle()`: 只有真实代价/致命格
   (且 != 255) 才算障碍, 未知不算。

### 离线实测(corridor_20261105_112401, 域 77, 90~150 s)

- 修完后 D 端 90 s 内做出 **35 次选点**, `/explore/selected_frontier` 在 RViz
  里以绿色箭头显示, 随地图延伸更新; `/explore/frontiers` 同步刷新。
- 150 s 那轮统计 44 条选点: 距机器人中位 2.98 m(1.24–14.11), 到最近阻塞格
  中位 0.58 m(0.36–1.26), 到最近未知格中位 0.20 m,
  **30/44 落在 costmap 的 NO_INFORMATION 格**。
- 后者不是回退失效(实测 dispatch 点确实比 frontier 质心内移 ~0.47 m),
  而是探索读的是**决策图**(`frontier_map_optimization_enabled` 会做双边滤波
  + 1 格膨胀), 膨胀把未知抹成自由; 加上 1.5 m 窄走廊本身侧向就贴着未知。
  是否可接受需要和 `goal_backoff_from_frontier_m: 0` 做 A/B。

### 验证

- Release 构建通过; 单测 **138 个全部通过**(新增 2 个: 完成去抖需要连续空扫描、
  地图太小时不判完成; 并修正了可达性用例——原来断言在空列表上恒真)。

### 后续

- 干跑只验证"选点", 不验证 Nav2 能否执行; 真机仍需看 `map->odom` 跳变与
  恢复行为频率。
- 可选 A/B: `goal_backoff_from_frontier_m` 0 vs 0.5, 对比 CSV 里
  `cost_global=-1` 占比与 `dist_unknown_m`。

## 3. 2026-11-16 洞穴探索模式 + 目标抢占联调接口

本分支新增 `cave_mode_enabled`（默认 false，Go2 现场 yaml 为 true）。开启后启用：
入口半径/门线过滤、方向分层（前→右→左→后）、DFS 岔路口退栈、完成分段返程、
目标抢占（PAUSE/HOLDING/RESUME）、目标迟滞。设计见
`nav_ws/go2-nav2-terrain-pointlio/docs/superpowers/specs/2026-11-16-cave-exploration-target-handoff-design.md`，
实现计划与实现记录见同目录 `plans/2026-11-16-cave-exploration-target-handoff.md`。

### 3.1 对外接口

**输入**

| 接口 | 类型 | QoS | 语义 |
|---|---|---|---|
| `/exploration/target_pose_base` | `geometry_msgs/msg/PointStamped` | reliable, depth 10 | 识别端发现目标。`frame_id` 任意可 TF 的帧（推荐 `base_link`）；`header.stamp` = 采集时刻（0 = 用最新 TF）；**每个目标只发一次** |
| `/exploration/grab_done` | `std_msgs/msg/Empty` | reliable, depth 10 | 抓取结束。**只在 `holding` 生效** |
| `/exploration/pause` | `std_msgs/msg/Empty` | reliable, depth 10 | 操作员暂停，保留地图/栈/轨迹 |
| `/exploration/resume` | `std_msgs/msg/Empty` | reliable, depth 10 | 恢复；从目标暂停（`paused`）中也能强制接管 |
| `/exploration/return` | `std_msgs/msg/Empty` | reliable, depth 10 | 结束巡逻、回洞口。`paused`/`holding` 时只锁存，抓取结束/恢复后才生效 |
| `control_exploration` 服务 | `ControlExploration` | — | `ACTION_START=1` / `STOP=2` / `PAUSE=3` / `RESUME=4`；返回 `STATE_PAUSED=6` |

`STOP` 是冷启动（清地图、清栈、回 `COLD_IDLE`）；`PAUSE` 是轻暂停。

**输出**

| 接口 | 类型 | QoS | 语义 |
|---|---|---|---|
| `/exploration/state` | `frontier_exploration_ros2/msg/ExplorationState` | reliable, depth 10, **volatile** | 1 Hz 心跳 + 每次状态迁移；**必须在 START 前订阅** |
| `/exploration_complete` | `std_msgs/msg/Empty` | reliable, transient_local, depth 1 | 每会话一次，兼容旧接口 |

`ExplorationState`：

```
std_msgs/Header header
string mission_state     # idle|exploring|backtracking|paused|holding|resuming|returning|complete
string reason
geometry_msgs/PoseStamped robot_pose     # map 系；TF 暂缺时为空
geometry_msgs/PoseStamped entrance_pose  # 未记录时为空
geometry_msgs/PoseStamped current_goal
geometry_msgs/PointStamped target_map
uint32 junction_depth
bool exploration_complete
```

`reason` 取值：

| state | reason | 含义 |
|---|---|---|
| `paused` | `target` | 已取消探索 goal，正在去停靠位姿；**不要开始抓取** |
| `paused` | `service` / `external` | 人工暂停 |
| `holding` | `target_reached` | 到达停靠位姿且朝向误差 ≤ 20°，可开始前倾/抓取 |
| `holding` | `heading_not_aligned` | 到达但朝向未对齐，`/spin` 已发但不阻塞 |
| `resuming` | `grab_done` / `grab_done_timeout` | 正在回"路中间"；**先收臂** |
| `exploring` | `resume` / `resumed` / `backtrack_arrived` | 继续探索 |
| `backtracking` | `backtrack` | 从尽头退回岔路口 |
| `returning` | `exploration_complete` | 洞内摸完，正在分段回洞口 |
| `complete` | `returned` / `returned_to_entrance` / `return_failed` / `return_rejected` / `return_dispatch_failed` | 结束，可移交"洞口→基地"模块 |
| `exploring` | `target_unreachable` | 停靠点导航失败/被拒，目标已放弃 |

### 3.2 与识别端的约定

- 坐标：`base_link`（x 前、y 左、z 上）；其他帧会按 `header.stamp` 做 TF，
  有界等待 0.2 s，超时丢弃并 WARN。z 不参与决策。
- 只发一次：重复发布在 5 s / 0.5 m 内去重；第一次 TF 失败即丢失，不重试。
  建议发布前确认 `map->base_link` 可用。
- 多目标：`paused` / `holding` / `complete` 期间的新目标静默忽略。
- 目标靠墙没问题：探索侧在目标周围 0.7 m 环上选净空最大、且到目标有视线的
  停靠点，偏好靠近已走轨迹（路中间）。
- 环上找不到合格点（全占/无视线）：目标被丢弃，只打 WARN，不改变探索状态。

### 3.3 与决策树的约定

- 抓取结束后必须发 `/exploration/grab_done`。抓取树根是 `Repeat num_cycles="-1"`，
  不会自己结束，**必须在树里加显式发布节点**。
- 发 `grab_done` 前先收臂：探索侧收到后会立刻驱动狗离开目标（先回"路中间"）。
- 60 s 未收到会自动恢复；抓取更久就调大 `grab_done_timeout_s`。
- 不要混用 `grab_done` 和 `/exploration/pause`：`pause` 会进入 `paused`，
  而 `grab_done` 只在 `holding` 生效，此时会被忽略。
- 中途接管用 `/exploration/resume`：能从目标暂停强制退出、取消在飞 goal、清 `target_map`。

### 3.4 一次目标的时序

```
识别端 → /exploration/target_pose_base (base_link 点)
探索侧 : 取消当前 frontier goal → 去停靠位姿 → state=paused, target_map=...
到点   : state=holding (reason=target_reached|heading_not_aligned)
决策树 : 前倾 + 抓取
抓完   : /exploration/grab_done
探索侧 : state=resuming → 去轨迹上最近净空点 → state=exploring（按当时地图重新选点）
超时   : 60 s 无 grab_done → 同样进入 resuming
```

### 3.5 现场校验

```bash
ros2 topic echo /exploration/state              # 先订阅再 START
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
ros2 topic echo /exploration/state --field entrance_pose --once
```

`entrance_pose` 为空 ⇒ START 后还没拿到 `map->base_link`，此时不做洞内过滤。
交接模块必须让狗朝向洞内，否则门线方向错；可临时 `entrance_gate_enabled: false`
只保留 3 m 半径球。

手动模拟目标（先确认 TF 在）：

```bash
ros2 topic pub --once /exploration/target_pose_base geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link, stamp: {sec: 0, nanosec: 0}}, point: {x: 1.5, y: 0.0, z: 0.0}}"
ros2 topic pub --once /exploration/grab_done std_msgs/msg/Empty "{}"
```

### 3.6 现场可调参数

| 参数 | 默认 | 何时改 |
|---|---|---|
| `entrance_radius_m` | 3.0 | 洞口直路更长就调大 |
| `entrance_gate_enabled` | true | 交接朝向不可靠时关掉 |
| `front_cone_half_deg` | 60 | 狗老往侧前方走就收窄 |
| `right_arc_max_deg` | 150 | 决定"后方"边界 |
| `junction_angle_sep_deg` | 45 | 一个开口被误判成两条路时调大 |
| `goal_hysteresis_threshold` | 0.15 | 仍抖就调大 |
| `exploration_complete_confirm_scans` | 10 | 洞大/地形更新慢就调大 |
| `suppressed_branch_retries` | 3 | 导航失败多时调大，避免过早判分支耗尽 |
| `grab_done_timeout_s` | 60 | 抓取耗时 |
| `target_standoff_radius_m` | 0.7 | 机械臂够不到就调小 |

### 3.7 地形探索完成后的巡逻（patrolling）

地形摸完**不再立即回洞口**。完成判定满足后进入 `patrolling`：

- 巡逻路线 = 已记录的轨迹按 `patrol_waypoint_spacing_m`（默认 3 m）抽稀，
  丢掉洞口 `entrance_radius_m` 内的点；DFS 走过每条岔路，所以轨迹覆盖所有岔道。
- 逐段 `NavigateToPose`，走到一端后**反向继续**（ping-pong），一直覆盖。
- 巡逻中收到目标 → 走 `paused → holding → (grab_done) → resuming`，
  恢复后**回到 `patrolling`**，不是回到 `exploring`。
- 决策树在时间阈值到达后发 `/exploration/return` → 立刻切 `returning`，
  走现有的 5 m 粗路点回洞口。
- 返回命令如果在 `paused`/`holding` 时到达 → 只锁存；抓取结束或恢复时直接
  `returning`，不再继续巡逻。
- 兜底：`patrol_timeout_s`（默认 0 = 关闭）到期自动返回，防决策树漏发。
- 路点连续失败超过 `return_max_consecutive_failures` 也会自动回洞口。

`/exploration/state` 里会出现 `mission_state: patrolling`，`reason:
exploration_complete`；巡逻期间 `junction_depth` 不再增长。

### 3.8 已知限制

- 尽头没有显式 `/spin` 掉头：掉头靠 NavFn+DWB；窄洞转不过身时表现为 goal 失败
  → suppression → 退栈。
- 返程是 5 m 粗路点分段重规划，不是精确沿原路。
- `STOP` 不清 session 级状态（`target_map`、岔路口栈、返程段），STOP→START 后
  会短暂保留。
- `resume` 不能从 `holding` 提前接管，只能等超时或发 `grab_done`。
- 测试：`colcon test` 12 个目标全通过（`test_mission_state_machine` 41 例）。

## 4. 2026-11-19 完成判定兜底 + SOFT 地形不再当墙

### 背景(真机日志)

`/root/.ros/log/frontier_explorer_19574_1795030274410.log`(11-18 19:31 UTC):

- 3 个 frontier goal 全部被 `visible reveal gain exhausted` 抢占取消, 狗每次只走了
  4~11 s, 从未到达 frontier;
- 之后连续 10 帧 `frontier_counts: raw=0, decision=0` → `Exploration finished`
  → 进入 `patrolling`, 巡逻路线只有 3 个点(3 m 抽稀 + 入口半径剔除)。
- 根因: `try_send_next_goal()` 的"空候选"信号同时代表了"地图探完"和"候选被
  costmap/min-size/backoff/suppression 过滤光", 无法区分; 去抖只按地图帧计数,
  狗停在原地 9 s 就凑满 10 帧。

### 改动

1. **raw 未知边界护栏**(新参数 `exploration_complete_raw_boundary_guard`, 默认
   `true`): 空候选时先在 raw map 上统计"`NoInformation` 且 8 邻接可通行
   (`0 <= cost < occ_threshold`)" 的格数。
   - `> 0`: 清零 `no_frontier_streak`, 打一次
     `frontiers exist but unreachable/suppressed (raw unknown boundary cells=N);
     not declaring completion`, 继续探索;
   - `== 0`: 才允许进入原有的 confirm_scans 去抖与最小已知格门限。
   - cave 模式下用 `is_outside_cave()` 忽略入口门线/半径外的格, 否则洞口外的
     未知区会让洞内任务永远无法进入巡逻; 记录入口时失效计数缓存。
   - 计数按 `map_generation` 缓存, 上限 1000 格提前退出, 避免 costmap 回调全图重扫。

2. **SOFT 地形不再当墙(仓库 B `pointlio_cloud_relay`)**:
   `exploration_map_relay` 把 `/terrain_traversability` 的 `128(SOFT)` 从
   `OCCUPIED(100)` 改为 `SOFT=50`; 池化优先级改为 `lethal > soft > free >
   unknown`。决策图把 `cost < occ_threshold(65)` 判为 free, 所以 50 对 frontier
   搜索等价于可通行, 但 RViz/调试仍能区分软地形。
   背景: 平地形档地形层把 15~20 cm 高度突变 / 0.12~0.35 m 粗糙带判为 SOFT,
   这些正是 Go2 能走的; 0.2 m 池化下 ≥3 格(0.6 m)的软地形带会截断
   `get_frontier()` 的地图 BFS, 之后的未知区永远不成为 frontier。

### 验证

- `colcon build` + `colcon test --packages-select frontier_exploration_ros2`:
  **227 tests, 0 failures**(新增 3 例: 边界阻止完成、门线外边界忽略、SOFT 视为可通行)。
- relay pytest: `test_exploration_map_relay.py` **15 passed**; 新增 2 例覆盖
  `SOFT` 池化优先级。
- 待真机验证: 下一轮走廊测试应看到"候选为空但 raw 边界存在"时不再进入
  `patrolling`, 以及软地形带后面的 frontier 能被选中。
