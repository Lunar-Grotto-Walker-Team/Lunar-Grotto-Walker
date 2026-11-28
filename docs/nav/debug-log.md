# debug-log.md — 问题排查记录

> **路径说明**：本文写于代码收编到主仓库之前，路径按当时的工作区 `nav_ws/go2-nav2-terrain-pointlio` 记录。代码现在位于 `src/base/`、`src/nav/` 与 `src/frontier_exploration_ros2/`，构建改在主仓库里做。

本文件记录**尚未定论或刚定位但未根治**的问题：现象、证据、已排除的假设、
候选机制、验证方法和修复方向。已闭环的结论进 `user-log.md`。

---

## 2026-11-06 自主探索时 `map->odom` 反复回退 + global costmap 走廊重影

### 状态

| 结论 | 把握 |
|---|---|
| `/aft_pgo_odom` 名义 10 Hz，由 message_filters 同步回调发布，与 1 Hz 的回环/可视化线程无关 | 已确认（代码 + bag 实测） |
| 手动点目标正常、加自主探索后异常，**不是** DDS 链路带宽问题 | 已排除（见 §3.1） |
| 探索节点与 SLAM / `map_odom_correction` 无直接数据通路，只能通过"运动模式 + Nav2 恢复行为 + CPU"间接影响 | 已确认（代码） |
| 走廊重影的直接机制 = 回环校正使 `T_map_body` 跳变 + costmap 只增量写入不重算 | 高度可能（§4.A） |
| `回退到 TF` 的直接机制 = 探索带来的连续大角度转向 → 关键帧/回环负载暴涨 → 同进程发布线程被压到 ~1 Hz | 待验证（§4.C，需把 Terminal A 日志落盘对齐） |
| 探索频繁触发 Nav2 恢复（清 costmap / spin / backup）会把 `TraversabilityLayer` 的陈旧格语义放大 | 高度可能（§4.B） |

---

### 1. 现象

1. 手动在 RViz 选目标点：规划、执行、到点都正常，`map->odom` 平稳，costmap 正常。
2. 同一地形、同一次会话改用 `frontier_exploration_ros2` 自主探索后：
   - global costmap 上**同一条走廊被画成多条方向不同的走廊**（重影/变形）；
   - `map_odom_correction` 终端反复刷
     `no /aft_pgo_odom within 1.0 s, falling back to TF map<-aft_mapped`
     以及 `map->odom 改用 PGO 位姿`；
   - 重启 Terminal C（清空 costmap）后恢复正常，但继续探索会再次出现。
3. 用户关键观察：**同样的地图大小，手动模式不会出现，加了自主探索才出现。**

### 2. 相关代码路径

| 位置 | 作用 |
|---|---|
| `pointlio_ws/src/simple_loop_closure_ros2/src/simple_loop_closure_node.cpp:270` | `/pgo_odom`（→`/aft_pgo_odom`）在**同步回调开头**发布 |
| 同上 `:611` | `getLastOptimizedPose()` 锁 `mtx_res_` |
| 同上 `:878` / `:898` | `visualizeThread` / `loopCloseThread`，各自 `rclcpp::Rate(1)` |
| 同上 `:852` | 每次检测到回环置 `vis_cache_rebuild_requested_ = true` |
| `pointlio_ws/src/simple_loop_closure_ros2/config/simple_loop_closure.yaml` | `keyframe_dist_th: 0.15`、`keyframe_angular_dist_th: 0.05`、`loop_search_dist_diff_th: 15`、`fitness_score_th: 0.3`、`vis_incremental: true`、`vis_full_rebuild_interval_s: 60` |
| `src/base/go2_driver/src/map_odom_correction.cpp:346` | `last_pgo_time_ = this->now()`，age 用**接收时刻**，不是 header stamp |
| 同上 `:371-400` | 超时 → 回退 TF；恢复 → 切回 PGO（11-06 已加 3 s 超时 + 2 s 滞回） |
| `src/base/go2_driver/src/driver.cpp:100` | `odom → base_footprint` 由 `/utlidar/robot_pose` 直接转发（~18.5 Hz） |
| `pointlio_ws/src/map_accumulator/config/accumulator.yaml` | 整张累积图 1 Hz 发 `/global_pgo_map`（0.05 m） |
| `l4t/frontier_exploration_ros2/src/frontier_explorer_node.cpp:267` | 探索节点只通过 `NavigateToPose` action 影响系统 |

**TF 关系**：`map_odom_correction` 每 20 Hz 计算

```
T_map_odom = T_map_body · (T_base_body)⁻¹ · T_odom_base⁻¹
```

使 `map→odom→base_footprint` 恒等于 SLAM 位姿 `map→camera_init→aft_mapped→base_footprint`。

### 3. 已排除的假设

#### 3.1 DDS / eth10 带宽拥塞（**排除为主要原因**）

最初的猜测：`.l4t_env.sh` 把所有 DDS 钉在 eth10（1 Gbps），`/aft_pgo_map`、
`/global_pgo_map` 每条都是几十 MB 的 RELIABLE 点云，小消息被 head-of-line 阻塞。

排除理由：

- 手动点目标时**这两条大图同样在 1 Hz 发布**（`visualizeThread` 每 tick 都发
  `publishCachedMapCloud`，与是否回环无关），RViz 也照样订阅；带宽在两种跑法里
  没有量级差异。
- 探索用的 RViz 配置是 nav2 默认视图（`go2_nav2_slam_explore.launch.py:104`），
  **不订阅** `/aft_pgo_map` / `/global_pgo_map`，探索反而让大图订阅者更少。
- 实测干净的 bag（`bags/corridor_20261105_112401`，295.7 s）：

  | topic | 条数 | 平均间隔 | 最大间隔 |
  |---|---|---|---|
  | `/aft_pgo_odom` | 2956 | 100.0 ms | 0.278 s |
  | `/cloud_registered` | 2956 | 100.0 ms | 0.259 s |

  两者严格 1:1 —— 输入链路本身是健康的 10 Hz，没有丢失。

带宽仍是**加重因素**（链路越挤，被压住的批次恢复越慢），但解释不了
"手动正常 / 探索异常" 这个对照，不再作为主因。

#### 3.2 tf2 ABBA 死锁（无关）

那是 `controller_server` 进程内的 TF buffer 冻结（0.25.23 → 0.25.24 已修），
表现为进程内 TF 永久停更，与本次"周期性回退后又自动恢复"形态不同。

#### 3.3 `map_odom_correction` 自身停摆（无关）

看门狗线程未报过 "未发布"；本次日志里节点一直在 20 Hz 发布，只是数据源在切。

### 4. 候选机制

#### A. 回环风暴导致 `T_map_body` 跳变 → costmap 重影（解释"走廊变形"）

- 探索是**长时、连续、反复重访**的运动；手动点目标通常是短程、点到为止。
- 11-05 一次真机日志：37 s 内 **70 次 `Loop Detected`**，且形如
  `643->2`、`644->2`、`648->2`、`652->2` —— **每个新关键帧都匹配回同一个旧帧**，
  这是自相似走廊里的**误检回环**（`loop_search_dist_diff_th` 已被放宽到 15 m、
  `fitness_score_th: 0.3` 偏松）。
- 每次回环把一批旧位姿拖走几十厘米，`T_map_body` 跳一次，`map->odom` 跟着跳。
- global costmap 的 terrain / obstacle 层**只增量写入、不重算已写过的格**，
  `TraversabilityLayer` 又只写"落在当前地形栅格内"的格、`isClearable()==false`
  → 旧对齐下的墙留在原地，新对齐下再画一条 = 多条方向不同的走廊。

手动模式重访少、几乎不触发回环，所以不会出现。

#### B. Nav2 恢复行为放大 costmap 陈旧格（解释"地形像被重置"）

探索频繁派出到不了 / 落在 ghost 区的目标 → planner 失败 → 行为树进恢复：
`ClearEntireCostmap` / `Spin` / `BackUp`。

- 清空后 `TraversabilityLayer::reset()` 丢掉 `latest_occupancy_grid_`，重绘时
  **只覆盖当前地形栅格范围**，窗口内其余区域回到未知 / 旧值；
- 若清空发生在 `map->odom` 跳变前后，重绘落点对齐不同 → "地形被重新确定"的观感；
- `Spin` 让狗原地大角度转，直接引出机制 C。

#### C. 探索的运动模式把 SLC 负载推过阈值（解释"回退日志"）

- `/aft_pgo_odom` 的发布点在 `simple_loop_closure_node.cpp:270`，跑在
  `rclcpp::spin` 的**单线程**上，且 `publishPoseGraphOptimizedOdometry`
  （`:236`）要经 `getLastOptimizedPose`（`:611`）拿 `mtx_res_`。
- 回环线程每秒在做：KD 树构建、ICP、`isam2_.update()`、
  `optimization_result_ = isam2_.calculateEstimate()`（锁内整表拷贝），
  以及**每个候选/每个目标帧各拿一次 `mtx_res_`**。
- 可视化线程每秒：无回环时走增量缓存；**一旦有回环就退化成全量重建**
  （`:852` → 下一 tick `clearVisualizationMapCache()` + 重变换全部关键帧 +
  逐点 hash 去重），重建期间持续抢 CPU。
- 进一步放大：`keyframe_angular_dist_th: 0.05 rad ≈ 2.9°` —— 探索时频繁转向
  （对 frontier 朝向、恢复 spin），**每转 3° 就存一帧**，关键帧密度比直线行走
  高一个量级，回环搜索 / iSAM2 / 重建代价一起涨。
- 结果：发布线程被压到 ~0.7–1 Hz，出现 >1 s 空档 → 超时回退 → 消息成串到达后
  又立刻切回。

日志里的形态支持"成串到达而非丢包"：19 次回退**全部**在 50–200 ms 内就切回：

```
414.620 回退 → 414.670 改用  (50 ms)
416.120 回退 → 416.170 改用  (50 ms)
417.420 回退 → 417.520 改用  (100 ms)
...
```

（11-06 12:33–12:34 那段，回退间隔 1.0–2.2 s，持续 ~40 s。）

但**同一窗口内没有 `Loop Detected`**（日志里最近的是 383 s 和 459 s），
所以 C 的触发未必是"回环"本身，也可能是探索带来的持续高转向负载。这正是
下面实验 1 要区分的地方。

### 5. 验证计划（真机，按顺序做）

1. **把 Terminal A 的日志落盘并对齐时间轴**（当前只有 `map_odom_correction`
   的日志有痕迹，SLC 的 `Loop Detected` 没对齐到回退时刻）：

   ```bash
   docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
     "source /my_ws/l4t/.l4t_env.sh && ros2 launch ... 2>&1 | tee /tmp/slc_run.log"
   ```

   判据：`回退到 TF` 的时刻是否紧跟在 `Loop Detected` /
   `vis map: full rebuild requested` 之后。**对齐 → C 的"回环"分支成立；
   不对齐 → 只剩"转向/关键帧密度"或 A/B。**

2. **探索节点在跑但不发目标**：`frontier_exploration_ctl stop`，然后手动发
   远处目标跑 20 分钟。
   - 仍回退 → 与探索决策无关，是时长 / 距离 / 转向积累；
   - 不复现 → 是探索的运动模式（A/C）。

3. **A/B 关键帧阈值**（最小改动、最快出结论）：

   ```yaml
   keyframe_dist_th: 0.15 -> 0.30
   keyframe_angular_dist_th: 0.05 -> 0.20
   ```

   重启 Terminal A 再探索。关键帧密度下来后若回退与重建频率同步下降 → C 成立。

4. **收紧回环接受条件**，看走廊重影是否减少：

   ```yaml
   loop_search_dist_diff_th: 15.0 -> 30.0
   fitness_score_th: 0.3 -> 0.2
   ```

   重影明显减少 → A 成立。

5. 需要时再看资源：`tegrastats`、`top -H -p <simple_loop_closure pid>`，
   同时 `ros2 topic hz /aft_pgo_odom /cloud_registered` 一起观察。

### 6. 修复方向

已做（11-06，commit `11fced2`）：

- `pgo_timeout_sec: 1.0 → 3.0`：短暂断流时沿用最后一条 PGO 位姿；
- `pgo_recover_sec: 2.0` 滞回：回退后要求连续新鲜 2 s 才切回；
- 真正切源且 `map->odom` 跳变超阈值时清 global costmap
  （`clear_costmap_on_switch`，0.15 m / 0.10 rad）。

待做（按优先级）：

1. **掐掉触发源**：关键帧阈值回到 0.3 m / 0.2 rad；回环接受条件收紧
   （15 m → 30 m、fitness 0.2，必要时要求同一目标帧连续命中）。
   这一步同时降低 CPU 负载和误检回环。
2. **把发布与输入解耦**：`/pgo_odom` 改由独立 timer（10–20 Hz）发布
   "最近一次 PGO 位姿"，与同步回调 / 拥堵解耦。这是最对症的一条。
3. **costmap 语义**：`TraversabilityLayer` 清理"当前地形栅格之外"的格，
   或至少让 `reset()` 后按当前栅格全量重绘（`isClearable()` 语义修正）。
4. **降低可视化开销**：`vis_map_voxel_leaf_size` 0.05 → 0.10（体积降约 4 倍），
   两条大图改 BEST_EFFORT（需同步改 map_accumulator 与 RViz 的 QoS）。
5. 可选：SLC 用独立 timer 以固定频率重发最后一条 `/aft_pgo_odom`
   （上游改法，属于治本但改动面大）。

### 7. 诊断脚本 `scripts/ros_diag.py`

专门抓"运行日志反映不出来"的证据。下一次复现时**在 Terminal E 里并跑**：

```bash
docker exec -it -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash
source /my_ws/l4t/.l4t_env.sh
python3 scripts/ros_diag.py                 # 一直跑到 Ctrl+C
python3 scripts/ros_diag.py --duration 600  # 或固定 10 分钟
```

它记录并落盘（默认 `/tmp/ros_diag/`）：

| 输出 | 内容 |
|---|---|
| `diag_<t>.csv` | 1 Hz 时序：pgo/cloud 频率与最大间隔、stamp 滞后漂移、三条 TF edge 的新鲜度、`map->odom` 位姿、eth10 收发、loadavg、各进程 CPU% |
| `events_<t>.jsonl` | 每个异常事件的完整快照（当时的 CPU / 链路 / load） |
| `summary_<t>.txt` | 结束时统计 + 启发式判读 |

关键设计——**用"到达间隔"和"消息 stamp 间隔"一起判定**：

```
arrival_gap 大 + stamp_gap 小  → 消息是按点产出的，只是路上/回调里晚了 (发布线程被压)
arrival_gap 大 + stamp_gap 大  → 上游压根没发 (point_lio / 同步器没有输出)
```

另外它直接给出两个替代日志的量：

- `PGO_JUMP`：`/aft_pgo_odom` 位姿增量**扣掉正常运动**后的残差 > 0.10 m，
  即回环校正发生的时刻（不需要读 Terminal A 的 `Loop Detected`）；
- `MAP_ODOM_JUMP`：`map->odom` 超出正常运动预期的跳变（驱动 costmap 重影的量）；
- `PGO_SILENT`：静默 >1 s 时的即时快照（话题彻底停发时到达侧判定不会触发）；
- `CLOCK_JUMP`：ROS 时钟跳变（排查 age 计算被时钟影响）；
- 启动 2 s 后自检话题发布者数量（多个发布者会制造假跳变）。

配套：仍然建议把 Terminal A 落盘，事后把 `Loop Detected` 与
`PGO_JUMP` / `MAP_ODOM_JUMP` 对齐：

```bash
ros2 launch ... 2>&1 | tee /tmp/slc_run.log
```

判读顺序：先看 summary 里"长空档"的 `publish_delayed` / `input_missing` 占比，
再看空档时刻的 SLC CPU 与 eth10 吞吐，最后用 `PGO_JUMP` 的时刻和
`Loop Detected` 对齐确认回环假设。

### 8. 附：本次排查看走弯路的教训

- 只看 `map_odom_correction` 自己的日志，会把它当成"发布链路问题"；
  真相是**订阅端的 age 定义（接收时刻）** + **发布端被同进程其它工作压住**。
- 一开始把 DDS 带宽排成主因，是只看了"消息大"没做对照实验；用户提出的
  "手动正常 / 探索异常"这个对照，比任何静态分析都更有判别力。
- `keyframe_angular_dist_th: 0.05` 这种"看着合理"的小阈值，在频繁转向的
  场景下会把所有下游代价都放大，排查时应当先换算成"每秒多少关键帧"。

---

## 2026-11-10 global costmap 重影：去掉 global 的 obstacle_layer（已闭环）

### 状态

| 结论 | 把握 |
|---|---|
| 回放期间 **0 次** PGO↔TF 切源；`map_odom_correction` 20 Hz 正常、watchdog 无告警 | 已确认（日志） |
| costmap 实际用的 `map->base_footprint` 在 50 ms 内跳 0.1–1.5 m，来源是 **SLC 每秒一次的优化**，不是驱动 odom、不是切源 | 已确认（50 ms TF 探针） |
| `obstacle_layer` 的标记是 costmap 里唯一"只写一次、不重算"的内容；地形层每周期整块重写所以自愈 | 已确认（Nav2 源码 + 快照） |
| 去掉 global 的 obstacle_layer 后，墙边重影/毛刺消失，costmap 与地形层重合 | 已确认（同一 bag 前后快照） |
| 用户看到的"整条走廊多方向叠加"在 1 Hz 快照里没有重现；复现到的是墙边毛刺/重复墙片 | 部分确认（采样可能漏掉亚秒瞬态） |
| local 在 T 型口大角度转圈的轻微向外鬼影 = `ground_filter` 用最新 LIO TF 放点云的 50–100 ms 姿态滞后 | 高度可能（未单独复现） |

### 1. 复现与工具

corridor bag（`corridor_20261105_112401`，295 s 环形走廊），域 77，
A 用 `/tmp/slc_go2.yaml`（go2 z/attitude 因子），B band-wide，C Nav2。
新增的探针都在 `scripts/large/`（gitignored）：

| 脚本 | 作用 |
|---|---|
| `analyze_bag_traj.py` | 离线看 bag 的 `/aft_pgo_odom`：转速、跳变、选复现窗口 |
| `tf_edge_probe.py` | 独立轻进程按 `/tf` 逐条记录 map->odom（按 stamp 排序分析） |
| `pose_chain_probe.py` | 同时记 map->odom、odom->base，合成 **map->base**（costmap 真正用的位姿） |
| `costmap_ghost_probe.py` | 每帧 costmap：terrain 层内容、当前点云支持度、无支持 occupied、PGO 步长 |
| `costmap_snapshot.py` + `render_costmap.py` | 录整圈 costmap/terrain 快照并渲染 PNG 对比 |

### 2. 排除项（同一份 bag）

- **切源抖动**：全程 0 次 PGO↔TF 切换（只有启动 TF→PGO、bag 结束后 PGO→TF
  各一次）；watchdog 无告警；`/aft_pgo_odom` 10 Hz、costmap 链 1.00 Hz。
- **bag 数据**：录制的 `/aft_pgo_odom` 最大单步 0.146 m；`/utlidar/robot_pose`
  最大 4.2°/帧、中位 0.4°——都平滑。
- **走廊特征点/上游降频**：`/aft_pgo_map`、`/terrain_traversability`、
  global costmap 全程 1.00 Hz（68–71 次回环仍保持）。
- **map_odom_correction 停摆**：不存在；20 Hz 发布，唯一"未就绪"告警出现在
  bag 结束后的切源（此时 planner 已在收尾）。

### 3. 直接测到的跳变

`pose_chain_probe`（50 ms 分辨率，60 s 窗口，回环密集段）：

- `map->base` 跳变 >0.15 m 或 >2°：**53 次/60 s**；
- 最大 **1.5 m / 6.8°**（一个 50 ms 步长内）；
- 同一时刻 `odom->base` 完全没动（增量 0.000 m）→ 跳变来自 `/aft_pgo_odom`
  （SLC 优化结果），不是 driver。

注意：不要用 `map->odom` 的平移判断跳变——odom 原点离机器人 10–20 m，
几度 yaw 变化会放大成米级平移，看起来像"跳 17 m"，实际 `map->base` 只变
几厘米。必须合成 `map->base` 或直接看 yaw。

### 4. 机制（Nav2 源码）

- `LayeredCostmap::updateMap()` 每周期对 combined costmap 的更新窗口
  `resetMap()`，然后按 `plugins` 顺序重写；
- `terrain_traversability_layer` 每个周期把整个地形栅格重写进 costmap
  （unknown_policy=no_information → 85 写成 255），**地图一变就自愈**；
- `obstacle_layer::updateBounds()` 把 `/cloud_obstacles` 的点直接写进
  **自己的持久 `costmap_`**（`costmap_[index] = LETHAL_OBSTACLE`），之后只靠
  raytrace 清；点在 `base_footprint`，进 `map` 必须过 `map->odom`；
- SLC 每秒优化一次 → `map->odom` 跳 → 旧标记留在旧对齐位置，raytrace 的射线
  又扫不到 → 转弯/回环处多出墙片。

补充：2026-11-04 的 `base_footprint` 修复（commit `462f821`）只根治了
**local**（odom 系，只用 odom->base）；同一个 `/cloud_obstacles` 也喂
**global**，global 必须经过 `map->odom`，于是问题转移到 global。
2026-11-06 的"切源清图"只处理 PGO↔TF 切源，**同源内部的优化跳变不触发**。

### 5. 实验 A（已做）

`global_costmap.plugins` 去掉 `obstacle_layer`，只留
`terrain_traversability_layer + inflation_layer`；local 不变。

同一 bag（1 Hz 快照，front/back）：

| t | occupied（含 obstacle 层） | occupied（实验 A） | 现象 |
|---|---:|---:|---|
| ≈214 s | 16,718 | 11,473 | 改前墙边大量斜向毛刺、墙片分叉；改后只剩 inflation 光晕 |
| ≈289 s | 19,241 | 12,773 | 改前内墙/左上重复墙片；改后与地形层基本重合 |

地形层本身不变（t≈289 lethal ≈1.4 万格），全链仍 1.00 Hz。
代价：global 规划不再看点级细柱/动态物；由 local（5×5 m、3 Hz、odom 系）兜底。

### 6. 方案 B（未实施）

`map_odom_correction` 里把"跳变清图"从只判切源扩展到每周期：比较
`map->base` 增量与 `odom->base` 增量，残差 >0.15 m / 0.1 rad 就调
`/global_costmap/clear_entirely_global_costmap`（1–2 s 去抖）。地形层 1 s
内会重画，清图代价低。**不能用 `map->odom` 平移判断**（见 §3）。
若现场发现 global 规划贴着细柱/门框走，再叠加 B（可和 A 共存）。

### 7. local 在 T 型口的向外鬼影（已做第一步）

`ground_filter` 把点云从 `camera_init` 转到 `base_footprint` 时，原来用的是
**最新 LIO TF**（`lookupTransform(..., Time())`），不是点云时刻的 TF。
实测这个"最新"通常比点云**旧一个 sweep（~100 ms）**——抛出的异常是
"Requested time ...107.947 but the latest data is at ...107.846"——所以旋转
时姿态误差 ≈ ω·Δt（90°/s、0.1 s ≈ 9°，10 m 处 ≈1.6 m），点被甩到墙外；
local 的旧标记只靠 raytrace 清，于是向外发散。local 在 odom 系、不经过
`map->odom`，所以比 global 轻。

**处理（2026-11-10, 第一步）**：改成 **stamp 精确查询 + 失败回退最新**，
并累计 `_stamp_fallbacks`。中段 120 s 实跑：只回退 **2 次**（都是 TF 比点云
晚 100 ms 的瞬间），其余全部精确；点云处理无中断，链路 1 Hz / local 3 Hz。
观感"好了一点"。

**残余（为什么没完全消失）**：点被放在 `t_sweep` 的机体系后，输出又被
`ground_filter` 用 `now()` 重新盖章；local costmap 再用 `odom->base(now)`
去旋转它。误差 = 处理+排队延迟内机器人的转动（几十 ms，90°/s 时 2–3°，
5 m 处 0.2–0.3 m）。另外狗自身 `/utlidar/robot_pose` 的延迟是另一条下限。

**要继续消除的前提**：把雷达 stamp 一路带到 costmap。实机是**同钟**的——
Livox Mid360S 用 Jetson 的软件时间戳（NoSync + eth10 无 PTP），point_lio 的
`get_ros_time()` 又是原样透传，所以点云 stamp ≈ Jetson 时间；driver 虽然
用 `now()` 盖 TF，但两者时间域一致，按点云 stamp 查 `odom->base` 可行。
**只有离线回放是特殊的**：bag 里的 stamp 是录制日期（2026-11-05），和 Jetson
`now()`（2026-11-10）差 5 天，任何按 stamp 的跨节点查询都会外推失败——所以
回放时必须重盖。要做零误差，需区分回放/实机（或用 `--clock` + `use_sim_time`
让 ROS 时间跟着 bag 走）。

**已实现（2026-11-11, 第二步）**：`ground_filter` 新增
`preserve_input_stamp: auto|always|never`（默认 `auto`）和
`stamp_match_tolerance_sec`（默认 1.0 s）。`auto` 的判据：
输入 stamp 与本机时钟的 age 在 ±tolerance 内、**且**位姿是 stamp 精确查到的，
就保留雷达 stamp；否则（回放差 5 天、或走了 latest 回退）重盖 `now()`。
回退帧必须重盖，否则 stamp 和实际用的位姿描述的不是同一时刻。
`always` 只保留精确帧（调试用），`never` 是旧行为。
日志每 10 s 带一行 `stamp auto: preserved=… restamped=… fallbacks=…`。
bag 回放实测：`preserved=0 restamped=全部`（符合预期）；实机预期
`preserved≈全部`，需要真机再确认 local costmap 的 message filter
（`transform_tolerance`）能吃下几十至一百多 ms 的点云 age。

### 8. 测量坑（下次直接避开）

- `nav_msgs/OccupancyGrid.data` 是 **int8**：bridge 的 128/254 读出来是
  -128/-2。必须 `& 0xFF` 或在布局上按 uint8 比较，否则"terrain 没有障碍"
  这种错误结论非常自然。
- 合成 map->base 时把角度当弧度用（`cos(yaw_deg)`）会让结果看起来在乱跳；
  统一单位后再算。
- terrain 栅格尺寸会随地图增长变化，`np.stack` 会炸；按帧存 pickle。
- `ros2 bag play` 不播 `/tf`；LIO TF 与 `/cloud_registered` 都是 point_lio
  用 lidar 时钟盖的章，driver 的 `odom->base` 用的是 Jetson `now()` 章——
  跨时钟的 TF 查询才会外推失败。

---

## 2026-11-11 global costmap 发布节奏 + /aft_pgo_map 后半段降频

### 状态

| 结论 | 把握 |
|---|---|
| global costmap 并非 1 Hz 发布，而是 **1 s / 2 s 交替**，平均 ~0.66 Hz | 已确认（逐条到达时间） |
| 降频根源是 **B（投影器）消费慢 + 大数据传输**，不是 SLC 自身计算 | 已确认（A-only 对照） |
| 改 BEST_EFFORT 订阅**没有帮助** | 已确认 |
| mean / 5 cm 分箱 median 都不能替代 median（精度损失过大） | 已确认（逐格对比） |
| 下一步：C++ rasterize（pybind11），保留 median 语义 | 计划 |

### 1. 发布节奏：1 s / 2 s 交替

`/global_costmap/costmap` 逐条到达间隔（300 s，197 条）：

- 84 次 ≈1.0 s，112 次 ≈2.0 s，**1.3–1.7 s 之间 0 次**，没有 <0.5 s 的成对消息；
- 平均 0.66 Hz；topic 只有 1 个 publisher。

原因在 `Costmap2DROS::mapUpdateLoop`（单线程，1 Hz）：`updateMap()` 之后按
`last_publish_ + publish_cycle_ < now` 决定是否发布，而 `last_publish_`
盖在更新结束时。等频时，条件只在"本次更新比上次发布那次更慢"时成立，于是
间隔被量化成 1 s/2 s。不是重复发布，不是重影。

### 2. 后半段降频：A-only vs A+B vs A+B+C（同一份全圈 bag）

`/aft_pgo_map` 每分消息数 / 时长（min1→min5）：

| 配置 | 频率 |
|---|---|
| 只有 A（72 次回环） | 1.02, 1.02, **1.02, 0.97** |
| A + B（BEST_EFFORT 订阅） | 1.02, 1.00, **0.75, 0.58** |
| A + B + C | 1.00, 0.95, **0.68, 0.54** |

- 这轮 A+B 里 `/aft_pgo_odom` 只从 10 Hz 降到 8.5 Hz（−15%），说明 LIO
  同步/位姿线程正常，慢的是 1 Hz 可视化发布线程；
- `ros2 topic info -v /aft_pgo_map` 确认投影器订阅已是 BEST_EFFORT，仍然降频
  → **不是 RELIABLE 背压**；
- 结论：A 的可视化线程往一个"处理慢的读者"发送越来越大（13.5 MB / 84.6 万点）
  的样本时，传输/流控开销把 1 Hz 线程拖慢。SLC 自身计算不是瓶颈。
- `/terrain_traversability` 与 `/aft_pgo_map` 每分条数完全相同 → B 没丢帧，
  只是慢；local costmap 3.2 Hz、odom 18.5 Hz、TF 均不受影响。

### 3. 精度对照：median 不能换掉

同一帧、同 extent、同 band 下比较 `rasterize(median)` vs `mean` / 5 cm 分箱
median（大帧约 50 万点、18k 有效格）：

| 指标 | mean vs median | 分箱 median vs median |
|---|---:|---:|
| `|dz|` p95 | 8.3–8.5 cm | 9.3–9.9 cm |
| `|dz|` max | 40–43 cm | 90–117 cm |
| `|dz|>2cm` 格 | ~22% | ~25% |
| slope p95 差 | 21–22° | — |
| roughness >0.05 m 格数 | +2.4% | — |
| roughness >0.15 m（lethal）格数 | **−12%** | — |
| slope >35° 格数 | −1.3% ~ +0.1% | — |

0.1 m 格里经常同时有地面和墙面点，median 正是为此存在；mean/分箱会让墙边
和粗糙区判定明显变松。**降采样/换聚合否决**。

Python 侧精确 median 也没有便宜替代：846k 点 `np.lexsort` 590 ms，两趟
stable argsort 489 ms（1.2×），bincount mean 22 ms（但不能用）。

### 4. C++ rasterize：已实现，但没有解决降频

分支 `feature/cpp-rasterizer`（commit `bcb5d14`，elevmap_fastlio）：

- 新包 `src/pgo_raster_cpp`（ament_cmake）编译 `libpgo_raster.so`：按格
  计数排序（O(N)）+ 每格 `std::nth_element` + OpenMP；
- `rasterizer.py` 用 ctypes 加载，`aggregation=median` 走 native，
  找不到库或 `PGO_RASTER_NATIVE=0` 回退 numpy；结果与 numpy **逐格 bit 级
  一致**（随机点 + NaN + z 门限 + 越界、以及真实 519k 帧都验过）；
- 846k 点 **31 ms**（numpy 797 ms），真实 519k 帧 21 ms vs 74 ms；
  投影器进程 CPU 降约 40%；启动日志带 `rasterizer=native|numpy`。

**但全圈 bag 的频率没有改善**：minute 3/4 仍然是 0.68/0.58 Hz（numpy 是
0.75/0.58，BEST_EFFORT 是 0.68/0.54）。各 run 里 A 组 CPU 都是 ~50%（没有
升降），`/aft_pgo_odom` 只掉 15% → A 的可视化线程是在**等待**而不是在算；
把消费者 CPU 降下来、换 QoS 都不改变结果。所以栅格化不是瓶颈，
"消费者慢"要改成"13.5 MB 样本 + 第二个读者存在时的发布/传输路径"。

待做（下次继续）：

1. dummy 读者对照：一个收到立刻丢弃、一个 sleep 1 s 释放，比较 A 的发布率，
   区分"消息体积/序列化"与"读者取数速率/流控"；
2. 或接受末段 ~0.6 Hz（TF/local/到达判断都不受影响，只是全局图新鲜度差
   ~2 s）；3. 拆块/降体积是最后手段（精度损失已在 §3 量化）。

### 5. 工具

`scripts/large/` 下新增（gitignored）：`costmap_rate_probe.py`（逐条到达
时间）、`capture_map_frames.py`（存 `/aft_pgo_map` 帧）、
`compare_aggregation.py`（逐格精度对比）、`aonly/abfull/abonly` 等 run 目录。

---

## 2026-11-12 `/aft_pgo_map` 降频定因：CycloneDDS 把同机数据发到了 eth10

### 状态

| 结论 | 把握 |
|---|---|
| 降频不是栅格化 / Python 反序列化 / CPU / 读者处理速度 | 已确认（对照实验，§2） |
| 与 QoS 无关：BEST_EFFORT 与 RELIABLE 完全一样 | 已确认（§2 `be_drop2`/`be_hold`） |
| 触发条件是 **`/aft_pgo_map` 同时有 ≥2 个匹配读者 + 大帧**，不是"SLC 算得慢" | 已确认（§3 隔离基准） |
| 机制：≥2 读者时 Cyclone 改走组播，而 `CYCLONEDDS_URI` 只声明了 eth10，同机进程间数据实际出了物理网口，有效吞吐 ~12 MB/s（≈96 Mbps） | 高度可能（§4 字节计数） |
| 修法：接口列表加 `lo`，2 读者时 publish 1155 ms → 32 ms | 已确认（§4、§5） |
| 只有投影器一个读者时，现行 eth10 配置下就是稳定 1 Hz | 已确认（§5） |

### 1. 前情修正（2026-11-11 §4）

之前写"消费者慢要改成传输/流控"，并据此怀疑 RELIABLE 背压。两处修正：

- 发布端**周期其实一直是 ~1 Hz**（每分钟 58–60 次），被拖慢的是
  `dds_write` 本身（末段 0.8–1.0 s）；订阅端只收到 ~0.6 Hz，是因为一帧要
  传 1 s，而读者队列深度是 1，中间帧被覆盖。
- 因此"降频"应表述为"**每帧传输 ~1 s，送达率被压到 0.6 Hz**"，不是
  "SLC 可视化线程算不过来"。

### 2. dummy 读者矩阵（A 单独跑，250 s 全圈 bag）

给 SLC 可视化线程临时加计时（补丁存
`scripts/large/patches/slc_vis_diag.patch`，验证后已还原源码并重编）：

```cpp
updateVisualizationMapCache();         // up_ms
publishCachedMapCloud();               // pub_ms  ← rclcpp::publish 阻塞时长
```

| 组 | 读者 | publish 中位数 min1→min4 | 周期数/min |
|---|---|---|---|
| `none` | 无 | 0.8 → 7.5 ms | 59–60 |
| `drop` | 1 个 C++ 序列化读者（取到即丢） | 2.2 → 18.4 ms | 58–60 |
| `sleep` | 1 个读者（take 后持 1 s） | 2.2 → 17.8 ms | 58–59 |
| `hold` | 1 个读者（订阅但从不 take） | 54 → 855 ms | 55（末段 31） |
| `drop2` | **2 个快速读者** | 51 → 856 ms | 55 |
| `be_drop2` | 2 个快速读者，BEST_EFFORT | 54 → 854 ms | 55 |
| `be_hold` | 1 个不 take 读者，BEST_EFFORT | 54 → 855 ms | 55 |

要点：

- **1 个读者取完还持有 1 s 完全没事**——`take` 已经回了 ACK，DDS 不关心
  回调里处理多久；所以"投影器处理慢"从来不是原因。
- 2 个读者无论 QoS 都掉到 ~0.85 s/次（末段 1.0 s）。
- 顺带排除：rclpy 反序列化 13.5 MB PointCloud2 实测 **7–8 ms**（`data` 是
  `array.array`），序列化 ~72 ms，都不是瓶颈。

### 3. 隔离基准：与 SLAM 无关

`scripts/large/dds_bench.py`（rclpy 发 13.5 MB 合成 PointCloud2，1 Hz）+
`aft_pgo_probe`（C++ generic 订阅，**不做反序列化**）：

| 匹配读者数 | publish p50 | 读者收到的间隔 |
|---|---|---|
| 0 | 11.5 ms | — |
| 1 | 27.7 ms | 1.00 s |
| 2 | **1155.6 ms** | 1.16 s |
| 3 | **1155.6 ms** | 1.16 s |

2 读者下扫尺寸（同一配置）：

| 帧大小 | 0.16 MB | 1.6 MB | 4.8 MB | 8.0 MB | 13.5 MB |
|---|---|---|---|---|---|
| publish p50 | 1.8 ms | 123.7 ms | 399.3 ms | 672.3 ms | 1152.1 ms |

→ 稳定 **~12 MB/s**，与字节数线性；2 和 3 读者代价相同，说明是"路径变了"
而不是"按读者数分摊"。

### 4. 定因：接口 / 组播路径

| 2 读者、13.5 MB | publish p50 | 是否收到 |
|---|---|---|
| `CYCLONEDDS_URI` 只绑 eth10（现行） | 1155.6 ms | ✅ |
| 只绑 `lo` | 32.9 ms | ✅ |
| `lo` + `eth10` | 31.7 ms | ✅ |
| eth10 + `AllowMulticast=false` | 9.7 ms | ❌ 0 条（发现阶段也依赖组播） |

同一基准 8 s 的字节计数（7 帧 ≈ 95 MB）：

| 配置 | lo TX/RX | eth10 TX | eth10 RX |
|---|---|---|---|
| 只绑 eth10 | +0.1 MB | **+97.8 MB** | +42.8 MB |
| `lo` + eth10 | **+245.8 MB**（单向 ≈123 MB） | +0.0 MB | +42.6 MB（背景流量，两组相同） |

结论：只绑 eth10 时，同机两个进程之间的 95 MB 数据**全部从物理网口出去了**，
有效吞吐 ~12 MB/s（≈96 Mbps，而 eth10 自报 1 GbE）；把 `lo` 加进接口列表后
全部走回环，publish 回到 30 ms 量级。

推断（未逐项证实）：1 个读者时 Cyclone 用单播，内核把发往本机地址的包路由
到 `lo`；≥2 个读者时改走组播，而组播出口由 `CYCLONEDDS_URI` 指定的接口决定，
于是出 eth10 再绕回来，被那条路径限到 ~96 Mbps。`AllowMulticast=false` 会连
SPDP 发现一起关掉、对端直接匹配不上，所以不能这么修。

### 5. 真栈验证（A + 投影器链，250 s）

| 配置 | 读者 | publish min2→min4 | 送达 / 投影器输出 |
|---|---|---|---|
| 只投影器 | 1 | 9.0 → 19.3 ms | 投影器输出 **0.97 / 1.00 / 1.00 / 0.98 Hz** |
| 投影器 + 1 探针 | 2（eth10） | 353 → 842 ms | 送达 1.00 → **0.87 Hz**（末段更低） |
| 投影器 + 1 探针 | 2（`lo`+eth10） | 13 → 28 ms | 送达 **1.00 Hz**（到 11.8 MB 仍是 1.00） |

**回答"只有一个投影器读者是否稳定 1 Hz"：是。** 现行 eth10 配置下，
publish ≤ 20 ms、每分钟 59–60 帧、投影器自己的输出逐分钟 0.97/1.00/1.00/0.98 Hz。

### 6. 建议与待办

1. **加 `lo` 在 CycloneDDS 0.10.5 上不可行（2026-11-12 当天回滚）。**
   `setup.sh` 里加上 `<NetworkInterface name="lo" .../>` 之后，Cyclone 会把 lo
   选作组播接口并打印
   `selected interface "lo" is not multicast-capable: disabling multicast`，
   然后**全局关掉组播发现**：狗内置主机上的 participant 收不到，
   `/utlidar/*` 整路消失（121 个话题 → 2 个；某些接口顺序下 121 → 114，
   缺的正好是 7 个 `/utlidar/*`）。已逐一试过且全部无效：
   接口顺序（eth10 在 lo 前 / 后）、`priority`（`default`、`10` vs `0`）、
   `multicast="true"/"false"`、`multicast="prefer_multicast"`（0.10.5 直接
   报 undefined value）。所以 `setup.sh` 保持 eth10-only。
2. **真正的修法是 `AllowMulticast=spdp`（当天追加验证，已落到 setup.sh）。**
   在 `<General>` 里加 `<AllowMulticast>spdp</AllowMulticast>`：只有发现阶段
   用组播，用户数据一律单播 —— 同机单播包由内核路由到 `lo`，不需要动接口
   列表，也就不碰发现。2 读者、13.5 MB、1 Hz，全部 RMW=cyclonedds：

   | 配置 | publish p50 | 两个读者收到 | 流量落点 | 发现 |
   |---|---:|---|---|---|
   | `AllowMulticast=true`（≈默认） | 1153.9 ms | 7/7 | eth10 TX +98 MB，lo 0 | ✅ |
   | `AllowMulticast=spdp` | **32.5 ms** | 9/9 | eth10 TX **0**，lo +246 MB | ✅ |
   | `AllowMulticast=false` | — | **0** | — | ❌ 连 SPDP 一起关 |
   | 接口表加 `lo`（已否决） | 32.9 ms | 2/2 | lo | ❌ `/utlidar/*` 全丢 |

   `spdp` 下用缓存环境实测：121 个话题、18 个 `/utlidar/*`、
   `/utlidar/robot_pose` 正常收数；2 读者基准 33.1 ms。
   **"只能保持单个订阅者"的旧建议作废**，RViz/monitor 再加订阅者不会退化了。
3. 顺带澄清"加 lo 到底提速没有"：**没有**。1 个订阅者时 eth10 默认 27.7 ms、
   加 lo 31.7 ms（本来就走单播/lo）；lo 只是绕开"≥2 订阅者时切组播、绕物理
   网口"这条劣化路径。`spdp` 用一行配置达到同样效果且不碰发现。
4. 真栈旁证：SLC 可视化线程已被改成 5 Hz（`vis_rate_hz`，同组同学改的），
   2 读者、帧 12 MB 下送达仍是 4.42–5.00 Hz（≈53 MB/s）。旧的组播路径上限
   只有 12 MB/s，这个吞吐在修复前不可能出现。
5. 未解的小问题：为什么 eth10 那条组播路径只有 ~96 Mbps（NIC 自报 1 GbE，
   组播回环可能绕了狗内部交换机的 100M 口）。
6. **测量坑（新增）**：验证 DDS 配置时，光 `source /opt/ros/humble/setup.bash`
   而没设 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，`ros2` CLI 用的是默认
   FastDDS —— 它**根本不读 `CYCLONEDDS_URI`**，三种接口配置看起来完全一样
   （都 121 个话题）。本次第一版"加 lo 不影响发现"的结论就是这样来的，作废。
   判据：先 `echo $RMW_IMPLEMENTATION`，或看 stderr 里有没有 Cyclone 的
   `selected interface` 提示。另外 `ros2` daemon 会缓存节点图并继承启动它的
   环境，改完配置要 `ros2 daemon stop` 再查。

### 7. 工具（均在 `scripts/large/`，gitignored）

- `probe_ws/src/aft_pgo_probe`：C++ generic 订阅探针，`drop|sleep|hold` 三种模式，
  `--qos`、`--topic`、`--type` 可调；
- `dummy_reader_run.sh`：A（可选 +B）回放 + 指定模式探针，域 77 隔离，
  顺带记录 UDP / `ip -s link` 计数；
- `run_dummy_matrix.sh`：矩阵顺序跑；`dds_bench.py` / `dds_bench.sh`：隔离基准；
- `probe_rate.py`、`diag_vis.py`：按分钟统计送达率、update/publish 耗时；
- `probe_ws/dds_{lo,lo_eth,nomulticast,default}.xml`：CycloneDDS 配置变体；
- `patches/slc_vis_diag.patch`：SLC 可视化线程计时补丁（源码已还原）。

---

## 2026-11-14 地形链提到 2 Hz：栅格尺寸、消息打包、QoS 三件事

### 状态

| 结论 | 把握 |
|---|---|
| 长走廊把地形栅格撑到 100.8 万格，是后处理掉到 0.26 Hz 的直接原因 | 已确认 |
| `.tolist()` 填 GridMap / OccupancyGrid 是更大的隐藏成本（1.15 µs/格） | 已确认（离线逐段计时） |
| 70 m 机器人中心窗口 + `array.array` 打包后，`_raw`/`_post`/`terrain` 全程 2.00 Hz | 已确认（466 s bag，A+B+C） |
| spdp 下 RELIABLE 不拖慢上游，连"从不 take"的读者也不阻塞 | 已确认（6 组 QoS 矩阵） |
| BEST_EFFORT 会丢大样本（11 帧只收到 5 帧），大话题别用 BE 测送达率 | 已确认 |

### 1. 背景

上游 SLC 改成 `vis_rate_hz: 2.0`（`loop_rate_hz` 仍 5.0），下游原先没有限速、
纯回调驱动，所以整链会跟着 2 Hz 跑——前提是跑得动。global costmap 之前是
1 Hz（`update_frequency/publish_frequency: 1.0`），是唯一的地形消费者
（local costmap 5 Hz，只吃 `/cloud_obstacles`），已同步改成 2.0。

### 2. 三轮实测（同一份 `data_20261111_132032`，466 s，A+B+C）

| 指标 | 无窗口（100 万格 + tolist） | 70 m 窗口（仍 tolist） | 窗口 + array |
|---|---:|---:|---:|
| 格数 | 1,008,000 | 490,000 | 490,000 |
| `/elevation_map_raw` | 1.98 → 1.14 Hz | 1.89 → 1.80 | **2.00 每分钟** |
| `/elevation_map_raw_post` | 1.92 → **0.26** | 1.34 → 0.96 | **2.00 每分钟** |
| `/terrain_traversability` | 1.97 → **0.47** | 1.32 → 1.19 | **2.00**（m7 一分钟 1.0） |
| projector CPU | 107% | 57% | **27%** |
| postprocessor CPU | 100%（饱和） | 100%（饱和） | **24%** |
| bridge CPU | 47% | 49% | 45% |

run 目录：`scripts/large/runs/{rate2hz,window70,window70_fast}_1114`。

### 3. 根因一：栅格尺寸

`auto_extent` 跟着累积点云 bbox 长，`extent_max_length` 默认 120 m；这条走廊
长，栅格一路涨到 **1008×1000 = 100.8 万格**（100 m × 100 m @ 0.1 m），末段
每帧 `/aft_pgo_map` 25 MB、`_post` 16.1 MB。下游三个节点都是 O(格数)。

**修法**：新增 `window_centered_on_pose` / `window_length`（70 m）/
`window_quantum`（1 m），`extent.py::window_spec()` 返回以机器人位姿为中心、
边长固定的几何；band 和 reference 两条路径都走 `_next_spec()`。70 m 覆盖
global costmap 的 60×60 m rolling 视口，格数钉死在 49 万。`quantum` 把中心
吸附到整数米，否则格子会随位姿逐帧亚格漂移（等价于 `_snap_spec` 防的假台阶）。

### 4. 根因二：`.tolist()` 打包（比栅格更隐蔽）

四个图层用 `array.data = flat.tolist()` 填消息：49 万个 float32 每层变成
49 万个 Python float 对象，rclpy 序列化时再逐个转回字节。离线逐段计时
（700×700 附近，含 45% NaN）：

| 格数 | compute_layers | 组装 tolist | 组装 array | 序列化 | 合计 tolist | 合计 array |
|---:|---:|---:|---:|---:|---:|---:|
| 9 万 | 21.8 ms | 101.8 | 2.7 | 3.0 | 127 ms | 27 ms |
| 25 万 | 57.4 | 285.4 | 7.1 | 5.1 | 348 ms | 70 ms |
| 49 万 | 107.4 | 563.1 | 13.2 | 7.9 | **678 ms** | **128 ms** |
| 100 万 | 221.4 | 1151.4 | 21.0 | 9.7 | **1382 ms** | **252 ms** |

两条都线性，但斜率差 ~50 倍：`compute_layers` 0.22 µs/格、`tolist` 1.15 µs/格、
`array` 0.02 µs/格。按旧代码，2 Hz（每帧 ≤500 ms）最多撑到 **~25 万格**；
换 `array` 后能撑到 **~170 万格**。所以"栅格大小"在旧代码里是硬上限，
在新代码里只是余量问题。

**修法**：三处都改为 `array.array`：
`elevation_fast_postprocessor/gridmap_out.py`（`array("f", flat.tobytes())`）、
`pgo_gridmap_projector/gridmap_msg.py`（同款）、
`elevation_traversability_bridge/bridge_processor.py`
（`array("b", uint8.view(int8).tobytes())`，保住 128/254/85 的线上字节）。
往返验证：4 层逐值一致（含 NaN 位置），OccupancyGrid 的 `[0,85,128,254]`
字节原样。

### 5. QoS 矩阵：RELIABLE 到底会不会拖慢上游

`AllowMulticast=spdp` 之后，1 读者、13.5 MB、1 Hz：

| 读者行为 | QoS | publish p50 | 读者收到 |
|---|---|---:|---:|
| 取到即丢 | RELIABLE | 18.6 ms | **11/11** |
| 取到即丢 | BEST_EFFORT | 17.9 ms | **5/11** |
| 取后持 1 s | RELIABLE | 18.7 ms | 11/11 |
| 取后持 1 s | BEST_EFFORT | 18.5 ms | 11/11 |
| 从不 take | RELIABLE | **18.2 ms** | 0 |
| 从不 take | BEST_EFFORT | 21.2 ms | 0 |

- **RELIABLE 不拖慢上游**：六组都在 18–22 ms，与 QoS、读者快慢、是否取数
  无关；`KEEP_LAST(1)` 允许写者覆盖未 ack 的样本，所以不会卡在 `dds_write`。
- 之前那 1.15 s 的阻塞完全是组播/eth10 路径造成的，不是 QoS 语义。
- **BEST_EFFORT 反而丢大样本**（11 → 5）。`/aft_pgo_map` 保持 RELIABLE；
  用 BE 读者测大话题送达率会得到假的"掉到 1 Hz/0 Hz"（本次 topics.csv 里
  `/aft_pgo_map` 在 m3/m5 显示 1.00、m6 起 0.00 都是这个原因，同一分钟
  projector 的 `_raw` 仍是 2.00，证明发布端正常）。

### 6. 现状与后续

- 链路上最贵的是 **bridge（45%）**，其次是 projector（27%）/post（24%）；
  global costmap 从 1 Hz 提到 2 Hz 后，controller/planner/bt_navigator 仍是 5–12%。
- 若将来要 5 Hz：先优化 bridge 的 `process_gridmap`（上采样 + 小簇清理 + 阈值），
  再考虑把 postprocessor 整体 C++ 化；只靠 Python 现结构大约能到 3–4 Hz。
- 工具：`scripts/large/cpu_sampler.py`（按 /proc 增量算 CPU，`ps %cpu` 是进程
  生命周期均值，看不出速率切换）、`dds_bench.sh` 的 `PROBE_MODE/QOS` 环境变量。

### 7. 2026-11-14 追加：窗口与自主探索的相互作用

`/exploration_map` 是 `exploration_map_relay` 直接从 `/terrain_traversability`
转出来的（0.1 → pool 2 → 0.2 m），而 `frontier_exploration_ros2` 的决策图就建在
它上面（`ingestRawMapUpdate` 只用**收到的这一张栅格**，栅格外不存在、也不产生
frontier）。所以 70 m 窗口会让探索只看得到机器人周围 ±35 m：

- 往回 40 m 有"看过但没进去"的未知块时，它已经不在图上了 → 可能提前判定探索完成。

**修法（已落地）**：relay 增加 `PersistentMap`，把每帧窗口按世界坐标合并进一张
持久全图（`accumulate: true`，默认；`max_cells: 4_000_000` 上限，超限时退化回
只发窗口并打一次 error）。known 覆盖 unknown、unknown 不覆盖 known，origin
吸附到格点、重复合并幂等。`/terrain_traversability` 仍是 70 m/2 Hz 给 costmap，
`/exploration_map` 恢复成整张已探索地图。

验证（corridor 295 s，A+B+C + D 干跑 + RViz）：`/exploration_map` 累积到
**232,500 格（0.2 m ≈ 93×100 m）**，而单帧窗口只有 122,500 格；
干跑 53 次选点、46 个不同目标，目标距机器人 1.0–6.9 m（中位 3.6 m），
随机器人前进依次更新，RViz 观察选点正常。

**前置过滤（窗口内裁剪点云）否决**：实测 220 万点、38% 在窗口内时，
直接栅格化 115.2 ms，先裁再栅格 129.5 ms（裁剪单独 76.9 ms）——原生内核算完
row/col 后就跳过窗口外的点，numpy 侧再裁一遍只是多了一次 83 万行的拷贝。
代码已移除，不做。

---

## 2026-11-14（续）reference 模式性能改造：0.5 Hz → 1.8 Hz

### 状态

| 结论 | 把握 |
|---|---|
| reference 的瓶颈是四处 Python 级热点，不是算法本身 | 已确认（逐段计时） |
| 四段改造后，斜坡 bag 上 `_raw` 0.88 → **~1.8 Hz**，projector CPU 100% → 20–78% | 已确认 |
| `despike_sparse` 的 `np.nanmedian(9,H,W)` 是最大的单点（924 ms/帧） | 已确认 |
| 仍未到 2.0：偶发掉帧（回环重算尖峰），不是稳态成本 | 观察 |

### 四段耗时（1M 点、70 m 窗口、0.5 m 粗格）

| 阶段 | 原始 | 改造后 | 做法 |
|---|---:|---:|---|
| `cell_stats` | 1378 ms | **135 ms** | `pgo_cell_sort`：计数排序 + 每格 `std::sort`（`np.lexsort` 消失） |
| `build_reference`（含上面） | 1580 ms | **140 ms** | `pgo_ground_flood`：C++ BFS，同 8 邻域顺序、`lower/upper_bound` |
| `select_ground` | 182 ms | 122 ms | 未改（per-point） |
| `despike_sparse` | **924 ms** | ~5 ms | 只对稀疏格（`count<min_support`）收集 3×3 邻域做 nanmedian |
| `output_band_counts` | 122 ms/帧 | 每 60 s 一次 | 纯诊断统计，跟汇报同频 |
| 栅格化 | 60 ms | 10 ms | 原生 median 内核 |

合计 **~1.8 s → ~130 ms**（离线）。两端都有对照测试：native `pgo_cell_sort` vs 朴素 lexsort、
native `pgo_ground_flood` vs Python list/bisect 版，均逐格一致。

两个踩过的坑：flat index 是 `col*n_rows+row`，`divmod` 要写成 `col, row = divmod(cur, n_rows)`；
BFS 的初始入队顺序必须保持 `np.argwhere` 的行优先序，否则斜坡上"谁先接受某格"会变，
`test_moderate_slope_connects` 会红。

### 默认配置的变化

`projector_params.yaml`：`ground_mode: reference` + `profile: default`，
`out_band_above: 1.00`（每格局部地面以上 1 m = 狗身 + 余量）。
band 的参数保留在文件里作 A/B 与 `/aft_pgo_odom` 缺失时的回退。

### profile: default vs cave

`cave` 是 `ground.py::cave_profile()` 对 `default` 的 5 个覆盖（1.5 m 洞窟调出来的）：

| 参数 | default | cave | 作用 |
|---|---:|---:|---|
| `seed_radius` | 1.5 | 1.2 | 种子盘更小，窄洞窟里少把墙算进地面 |
| `ring_check_tol` | 0.15 | 0.25 | 窄洞里环上像素少、噪声大，放宽 |
| `ref_band_above` | 0.30 | 0.35 | 洪水向上更宽容（洞顶低、地面起伏相对大）|
| `out_band_above` | yaml 值（现 1.00） | **0.50** | 关键差异：cave 把输出带上界压到 0.5 m，主动切掉约 1.1 m 高的洞顶，避免顶被当地面 |
| `max_gap_cells` | 4 | 3 | 空格跨越更保守 |

注意 profile 在读完 yaml **之后**应用，所以 `profile: cave` 会把 `out_band_above`
强制改成 0.50，yaml 里写 1.00 也没用。低矮顶场景用 cave，普通走廊/坡面用 default。
