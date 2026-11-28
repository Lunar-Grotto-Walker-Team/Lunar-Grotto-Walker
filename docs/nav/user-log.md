# go2-nav2-terrain-pointlio 用户日志

> **路径说明**：本文写于代码收编到主仓库之前，路径按当时的工作区 `nav_ws/go2-nav2-terrain-pointlio` 记录。代码现在位于 `src/base/`、`src/nav/` 与 `src/frontier_exploration_ros2/`，构建改在主仓库里做。

> 当前仓库对接 **point_lio (Mid360S) + elevmap 地形链**：
> A(point_lio) → B(elevmap) → C(本工作区 Nav2) → D(自主探索)。
> 旧 `go2-nav2-terrain` 的 `/scan`、静态地图、slam_toolbox 流程已删除，
> 不要再按那套命令跑。

---

## 1. 适用范围与不支持项

本分支的 costmap 数据流：

```
Terminal A (pointlio_ws)
  point_lio + simple_loop_closure
    -> /aft_pgo_map (~1Hz)  /aft_pgo_odom  map->camera_init
            |
Terminal B (elevmap_fastlio)
  pgo_gridmap_projector  /elevation_map_raw (auto_extent, 0.10 m, 随探索增长)
    -> elevation_fast_postprocessor  /elevation_map_raw_post
    -> terrain_traversability_bridge  /terrain_traversability (0/85/128/254)
            |
Terminal C (本工作区)
  go2_navigation2/go2_nav2_slam_explore.launch.py
    - go2_driver (odom->base_footprint)
      + map_odom_correction (动态 map->odom, 跟 /aft_pgo_odom)
    - pointlio_cloud_relay:
        cloud_to_base   /cloud_registered -> /cloud_base (调试用)
        ground_filter   -> /cloud_ground + /cloud_obstacles (base_footprint)
    - Nav2: global_costmap 60x60 m rolling @0.10
              = terrain_traversability_layer
                + obstacle_layer (/cloud_obstacles, 6 m) + inflation
            local_costmap  5x5 m rolling @0.05
              = obstacle_layer (/cloud_ground 只清, /cloud_obstacles 标记)
                + inflation
    - exploration_map_relay -> /exploration_map (0.2 m, 给 Terminal D)
            |
Terminal D (frontier_exploration_ros2)
  frontier 选点 -> navigate_to_pose (完成判据 XY-only)
```

- **没有 `/scan`**：本分支删掉了 `pointcloud_to_laserscan` / `cloud_accumulation`
  以及 launch 里的 `/scan` include；也删掉了 launch 内的
  `terrain_traversability_bridge`。
- **`/terrain_traversability` 的唯一发布者** 是 Terminal B 的
  `pgo_gridmap_projector/launch/go2_pointlio_projected.launch.py`
  （`with_bridge` 默认 true）。**绝对不要**在别处再起一个 bridge。
- **`go2_explore_bringup.launch.py` 在本分支不受支持**：它 include 本分支的
  `go2_nav2_slam_explore.launch.py`，但其 `slam_toolbox` 需要已被删除的
  `/scan`。自主探索改走 **Terminal D / `scripts/run_explore.sh`**（消费
  Terminal C 的 `/exploration_map`），不要用 `go2_explore_bringup`。

## 2. 工作区 source 规则（重要）

宿主机 `l4t_setup.sh` 要串六个工作空间（约 7.3 s）。日常用它的缓存
`/my_ws/l4t/.l4t_env.sh`（约 0.004 s，导出的 24 个变量与 source 完
`l4t_setup.sh` 等价）。**只有 `colcon build` 装了新包才需要刷新缓存**：

```bash
bash /my_ws/l4t/lg_walker_ws/scripts/lgw_env.sh --refresh
```

规则：**每个终端先 source 缓存，再叠加一个业务工作区的 `install/setup.bash`**。

```bash
# 每个终端的第一行都一样
source /my_ws/l4t/.l4t_env.sh
```

| 终端 | 再加这一行 | 说明 |
|---|---|---|
| A | `source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash` | point_lio / SLC |
| B | `source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash` | projector / filters / bridge |
| C | `source /my_ws/l4t/nav_ws/traversability-nav2/install/setup.bash`<br>`source /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/install/setup.bash` | 本工作区 **+ 插件工作区**（或直接用 `scripts/run_nav2.sh`，见 §4） |

**C 为什么多 source 一个 `traversability-nav2`**：本工作区 `go2_navigation2` 的
global_costmap 使用 `nav2_traversability_plugin::TraversabilityLayer`，这个插件的
`.so` 不在本工作区，装在上游 `traversability-nav2` 工作区的
`install/nav2_traversability_plugin/`。只 source 本工作区时运行期会报：

```
According to the loaded plugin descriptions the class
nav2_traversability_plugin::TraversabilityLayer with base class type
nav2_costmap_2d::Layer does not exist
```

→ `global_costmap` configure 失败 → `bt_navigator` 起不来 →
`lifecycle_manager` 打印 `Failed to bring up all requested nodes. Aborting bringup.`，
Nav2 整体不可用。`traversability-nav2` 只装这一个插件包，与本工作区的包名不冲突，
可以同 shell 叠加（这是 §2 "一个终端一个业务工作区" 的唯一例外）。

> 排查时注意：**只看 topic 频率 / TF 会漏掉这个故障** —— 插件缺失时
> `/cloud_base`、`/terrain_traversability`、`map->odom` 都照常，只有
> lifecycle 是死的。`scripts/check_pointlio_chain.sh` 的第 7 项就是查这个。

不要同时 source `go2-nav2-terrain`（旧分支）：它与本工作区安装同名包
（`go2_navigation2` / `go2_driver` / …），叠加后 `ros2 run/launch` 会随机
解析到另一套二进制，节点行为不可预期。B（`elevmap_fastlio`）只装
`pgo_gridmap_projector` / `elevation_traversability_bridge` 等，与 C 的包名
不冲突，技术上可以同 shell 叠加；为免环境混乱，仍建议每个终端只叠一个业务工作区。
**唯一例外是 C 必须再叠 `traversability-nav2`**（global_costmap 插件来源，见上）。

## 3. 整工作区编译（首次运行必做）

本仓库拷贝过来时**没有 build/install/log**，端到端跑之前先把整个工作区编完：

```bash
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --parallel-workers 4"
```

装了新包（AMENT_PREFIX_PATH 变化）后记得刷新一次缓存，见 §2。

- `--parallel-workers 4`：Orin NX 8 核共享机，留一半给 IDE / RViz / 调试。
- 只改 `go2_navigation2` 时可：
  `colcon build --packages-select go2_navigation2 --parallel-workers 4`；
  只改 relay 时用 `--packages-select pointlio_cloud_relay`。
- `install(DIRECTORY launch ...)` 的包在 **rename / 删除源文件**后不会自动清
  install，遇到旧文件残留先 `rm -rf build/<pkg> install/<pkg>` 再 build。

## 4. 四终端拓扑

真机（Terminal A 用真实雷达驱动）：

```bash
# Terminal A: point_lio 全栈 (LIO + SLC + map->camera_init)
# 2026-11-20 更正: 默认 profile 已是 cave_proxy (3-DOF 回环 + 世界系里程计平移
#   + livox 重力先验 + 回环平移上限 1 m)，leg-z 与先验地图默认都关闭。
#   旧的 /tmp/slc_go2.yaml (enable_go2_z / enable_go2_attitude) 配方已废弃，
#   launch 不传 slc_params_file 就是 cave_proxy。
#   leg-z A/B      : slc_params_file:=.../config/simple_loop_closure_cave_legz.yaml
#   基地原点建图   : 加 wait_for_start:=true；把狗摆到基地原点后调一次
#                    ros2 service call /start_mapping std_srvs/srv/Empty {}
# 环境: 导航链一律用 .l4t_nav_env.sh；缓存缺失先跑
#   bash l4t/nav_ws/scripts/lgw_nav_env.sh --refresh（宿主机）
docker exec -it -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && source install/setup.bash && \
   ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
     use_livox_driver:=true show_rviz:=false"

# Terminal B: elevmap 地形链 (projector + filters + bridge; with_bridge 默认 true)
# 2026-11-10: 默认 = band-wide (ground_mode=band, ignore_points_below=-1.2,
#   即相对地面约 -0.81 ~ +0.89 m)，启动日志应显示 ground_mode=band、z in [-1.2, 0.5]。
docker exec -it -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && source install/setup.bash && \
   ros2 launch pgo_gridmap_projector go2_pointlio_projected.launch.py show_rviz:=false"
# B 的第二条路（洞穴/低矮顶）：reference + profile=cave
#   把上面 launch 的 show_rviz:=false 换成：
#     show_rviz:=false \
#     projector_params_file:=/my_ws/l4t/nav_ws/elevmap_fastlio/install/pgo_gridmap_projector/share/pgo_gridmap_projector/config/projector_params_cave.yaml
#   RViz 有两份配置，按模式选（launch 参数 rviz_config，默认 band-wide 那份）：
#     band-wide      : .../elevation_mapping/share/elevation_mapping/rviz2/go2_elevmap_fastlio.rviz
#     reference+cave : .../elevation_mapping/share/elevation_mapping/rviz2/go2_elevmap_fastlio_cave.rviz
#   cave 那份按 slope 上色（0-0.61 rad / 0-35 deg），band-wide 那份是原始
#   elevation 上色。launch 里 show_rviz:=true 时用默认即可，只有 standalone
#   起 rviz2 或要看 cave 配色时才需要显式传 rviz_config:=<cave 路径>。

# Terminal C: 本工作区 (driver + relay + Nav2)
# 必须走包装脚本: 它在一个 shell 里 source .l4t_env.sh + traversability-nav2(插件) +
# 本工作区, 少一层就会插件加载失败 (见 §2)
docker exec -it -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "bash scripts/run_nav2.sh rviz:=false"

# Terminal D: 自主探索 (可选; C 起稳之后)
# 包装脚本 source .l4t_env.sh + frontier_exploration_ros2 + 本工作区
# 参数文件用本工作区的 frontier_go2_params.yaml, 默认 idle (autostart:=false)
docker exec -it -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "bash scripts/run_explore.sh"
# 手动开始/停止:
docker exec -it -w /my_ws/l4t self_l4t bash
source /my_ws/l4t/l4t_setup.sh \
  && source /my_ws/l4t/frontier_exploration_ros2/install/setup.bash \
  && source /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/install/setup.bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop

# 录制规划过程
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && \
   ros2 bag record -o /tmp/nav_diag \
     /cmd_vel /cmd_vel_nav /api/sport/request /api/sport/response \
     /lf/sportmodestate /odom /utlidar/robot_pose /aft_pgo_odom /tf \
     /local_costmap/costmap_raw /global_costmap/costmap_raw /terrain_traversability \
     /local_costmap/published_footprint /local_costmap/costmap"
docker cp self_l4t:/tmp/nav_diag /tmp/

# Terminal E (可选, 诊断): /tf 边健康监控
# 逐 edge 区分 "DDS 投递停滞" 与 "发布端 stamp 冻结"。
docker exec -it -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && source install/setup.bash && \
   python3 scripts/tf_health_monitor.py"

# 录制地形数据
docker exec -it -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash

source /my_ws/l4t/.l4t_env.sh && \
source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash && \
source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
source /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/install/setup.bash

OUT=/my_ws/l4t/nav_ws/bags/corridor_$(date +%Y%m%d_%H%M%S)
# 后三行可不录
ros2 bag record -s sqlite3 -o "$OUT" \
  /livox/lidar /livox/imu /cloud_registered \
  /utlidar/cloud_deskewed /utlidar/robot_odom /utlidar/robot_pose \
  /odom /aft_pgo_odom /lf/sportmodestate /tf /tf_static \
  /cmd_vel /vis_pose_graph \
  /terrain_traversability /global_costmap/costmap /local_costmap/costmap_raw \
  /cloud_obstacles /cloud_ground /local_costmap/published_footprint  \
  /elevation_map_raw /elevation_map_raw_post \
  /exploration_map

# 另开一个，只在末段录 15–20 s 的 /aft_pgo_map（离线测投影器/发布代价用）
PGO=/my_ws/l4t/nav_ws/bags/pgo_map_$(date +%Y%m%d_%H%M%S)
ros2 bag record -o "$OUT" /aft_pgo_map
```

`scripts/run_nav2.sh` 等价的手工写法（要自己开交互 shell 时）：

```bash
source /my_ws/l4t/.l4t_env.sh
source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash
source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash
source /my_ws/l4t/nav_ws/traversability-nav2/install/setup.bash   # 插件所在
source /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/install/setup.bash
ros2 launch go2_navigation2 go2_nav2_slam_explore.launch.py rviz:=false
```

脚本特性：`set -e`；任一 source 文件不存在直接 `ERROR: [run_nav2] ...` 退出码 1；
source 完还校验 `ros2 pkg prefix nav2_traversability_plugin` 可解析；
额外参数原样透传给 launch（`rviz:=`、`dynamic_map_to_odom:=`、`map_to_odom_*` 都能用）。

- A 必须先起：B 的 projector 消费 A/SLC 发布的 `/aft_pgo_map`（projector
  本身不做 TF lookup，不依赖 `map->aft_mapped`）；C 的 `map_odom_correction`
  依赖 A 的 `/aft_pgo_odom`。顺序 A -> B -> C -> D。
- B 的 `with_bridge:=true` 是默认值，**不要显式关掉**；也不要另外单起 bridge。
- **2026-11-20 起**：A 直接用默认 profile `cave_proxy`（launch 不传
  `slc_params_file`），不再复制 `/tmp/slc_go2.yaml`；leg-z A/B 才显式传
  `simple_loop_closure_cave_legz.yaml`。B 默认 **band-wide**（`ground_mode=band`、
  `ignore_points_below=-1.2`）；洞穴用 `projector_params_cave.yaml`
  （`ground_mode=reference` + `profile=cave`）。启动日志应显示 `ground_mode=band`
  或 `ground_mode=reference`，band 的 z 范围是 `[-1.2, 0.5]`（相对 pose）。
- B 的 RViz 用 `show_rviz:=true`。如果 launch 拉起的 `rviz2` 启动瞬间就退（已知
  SIGINT → rcl guard condition 崩），用独立进程：
  ```bash
  docker exec -d self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && \
    source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
    export ROS_DOMAIN_ID=77 DISPLAY=:1 && \
    exec rviz2 -d /my_ws/l4t/nav_ws/elevmap_fastlio/install/elevation_mapping/share/elevation_mapping/rviz2/go2_elevmap_fastlio.rviz"
  # cave 模式把最后一段换成 go2_elevmap_fastlio_cave.rviz（按 slope 上色）。
  ```
- B 默认 `use_fast_postprocessor:=true`（numpy 后处理）；A/B 对比旧滤波链时传
  `use_fast_postprocessor:=false`，两者互斥，不要同时开。
- C 默认 `dynamic_map_to_odom:=true`，**不要**再传 `publish_map_to_odom:=true`
  （两者互斥）；没有 A 时 C 的 global_costmap 会激活失败，这是预期。
- D 依赖 C 的 `/exploration_map`（`exploration_map_relay`）与 `navigate_to_pose`；
  C 没起来 D 会一直等，不会误发目标。
- 三个容器都跑在 `self_l4t` 里；每个 `docker exec` 是独立 shell，所以各自
  source 各自的工作区。C 用 `scripts/run_nav2.sh` 统一处理（含插件工作区）。
- 需要看画面时给某一个终端加 `rviz:=true`（建议只开一个，省 CPU）。
- **停止 D 不要只按 Ctrl+C**：`FrontierExplorerNode` 析构只置 shutdown
  标志并释放 goal handle（`request_shutdown()`），**不会取消已经发给 Nav2 的
  `navigate_to_pose` goal**，C 里的 bt_navigator 会继续把狗带到目标。
  正确顺序：
  1. `ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop`
     （走 `stop_exploration_session` → `async_cancel_goal`，会真正取消当前目标）；
  2. 确认 D 回到 idle 后再 Ctrl+C；
  3. 紧急情况优先用狗自带遥控急停 / Unitree App 停止；必要时在 C 停 Nav2
     后再向 `/cmd_vel` 发零速（twist_bridge 会转成 sport 请求）。

## 5. 只用 bag 的离线验证（无硬件）

没有真机时用 `testdata/livox_data_01` 回放；此时 **Terminal A 必须
`use_livox_driver:=false`**，并且**不要**同时开真机 livox 驱动（两路雷达数据
会互相污染）。

```bash
# A: 关雷达驱动（2026-11-20 更正：不再需要 /tmp/slc_go2.yaml，默认 cave_proxy）
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && source install/setup.bash && \
   ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
     use_livox_driver:=false show_rviz:=false"

# B/C: 用 scripts/large/run_large_test.sh（强制 ROS_DOMAIN_ID=77，只播传感器 topic）。
# 默认 B = band-wide；洞穴把 PROJECTOR_PARAMS 指向 projector_params_cave.yaml。
# C 的 RViz 用 --rviz；B 的 RViz 用 B_RVIZ=true（launch 版可能 SIGINT，见 §4）。
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "bash scripts/large/run_large_test.sh \
     --bag /my_ws/l4t/nav_ws/bags/corridor_20261105_112401 \
     --terminals ABC --rviz --growth --pgo-stats"
# 只 A+B: --terminals AB（B 的 RViz 用 B_RVIZ=true）
# 洞穴 B:
#   PROJECTOR_PARAMS=/my_ws/l4t/nav_ws/elevmap_fastlio/install/pgo_gridmap_projector/share/pgo_gridmap_projector/config/projector_params_cave.yaml
# 小 bag: --bag /my_ws/l4t/nav_ws/testdata/livox_data_01

# 手工 bag 回放（脚本已内置，通常不需要单独跑）
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && source install/setup.bash && \
   ros2 bag play /my_ws/l4t/nav_ws/testdata/livox_data_01 --rate 1.0"
```

## 6. 一键只读检查

A/B/C 起来后执行（第 8 项由 C 里的 relay 提供；D 是探索终端，可选）：

```bash
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "bash /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/scripts/check_pointlio_chain.sh"
```

脚本只做查询，不启动/停止/发布任何东西。检查项（第一项失败即
`FAIL: <item>` + 退出码 1，全过打印 `PASS`）：

1. `/scan` 没有发布者（话题不存在或 `Publisher count: 0`）；
2. `/terrain_traversability` 8 s 内至少 1 条；
3. `/cloud_obstacles` 5 s 内至少 25 条（sensor-data QoS，脚本用 best_effort 订阅）；
4. `/elevation_map_raw` 的 `info.length_x` 与 `length_y` **各自**落在 10–120 m
   （地形图随探索增长，长条场地两边本来就不等；注意 projector 发的是
   `grid_map_msgs/GridMap`，不是 OccupancyGrid）；
5. global_costmap 的 `width` == 60 且 `rolling_window == true`（依次尝试
   `/global_costmap/global_costmap`、`/global_costmap`、
   `/planner_server/global_costmap`）；
6. `map->odom` TF 可解析，**且时间戳在前进**：脚本先 `tf2_echo` 确认能查，
   再直接订阅 `/tf`，要求 `(map, odom)` 的 `header.stamp` 在 4 s 内前进 > 1 s。
   冻结的 TF 仍然"能解析"，只看 `tf2_echo` 会假 PASS；
7. **Nav2 lifecycle 全部 active**：`ros2 lifecycle get` 检查
   `/global_costmap/global_costmap`（候选名同上）、`/bt_navigator`、
   `/planner_server`、`/controller_server`，状态必须是 `active`。
   节点不存在 → `FAIL: <label> 节点不存在 ...`（提示检查是否 source 了
   `traversability-nav2` 插件工作区）；状态不对 →
   `FAIL: <node> lifecycle is '<state>'`；
8. `/exploration_map` 5 s 内至少 1 条且 `info.resolution == 0.2`
   （由 Terminal C 的 `exploration_map_relay` 发布，D 依赖它）。

脚本自身会 `source /my_ws/l4t/.l4t_env.sh`（不存在时回退到 `l4t_setup.sh`）；
不需要 source 本工作区 install（lifecycle 检查只查 ROS graph）。

## 7. 验收清单

> 本期只在**平地 / 近水平洞道**验收；坡面过滤不是目标。

| # | 验收项 | 命令 / 判据 |
|---|---|---|
| 1 | 无旧链路节点与 `/scan` | `ros2 node list \| grep -E 'cloud_accumulation_node\|pointcloud_to_laserscan_node'` 为空；`ros2 topic list \| grep -w /scan` 为空 |
| 2 | 地形图几何 | `ros2 topic echo --once --field info /terrain_traversability` -> `resolution=0.1`、两轴各自随探索增长（长条场地两轴不等）；`--field header.frame_id` -> `map`；约 1 Hz |
| 3 | `/cloud_base` | `--field header.frame_id` -> `base_footprint`；~10 Hz；stamp 与 `now()` 差 < 1 s（脚本已隐式覆盖频率） |
| 4 | global costmap | `python3 /my_ws/l4t/nav_ws/costmap_check.py /global_costmap/costmap` -> `size=600x600 res=0.100`、`rolling_window=true`；随机抽 3 个地形 lethal 格，在同一世界坐标的 costmap 里也应为 lethal（狗在 ±30 m 内） |
| 5 | map->odom 与 PGO 位姿一致 | `ros2 run tf2_ros tf2_echo map aft_mapped` 与 `ros2 topic echo --once /aft_pgo_odom` 对比；狗静止时误差 < 0.05 m。**不要在 `base_link` 上比**（中间隔着安装外参） |
| 6 | 规划 + 跟随 | RViz 给 ±20 m 内 goal：planner 成功、controller 跟随、狗实际移动 |
| 7 | 快速目视 | RViz 中地形 lethal 区域与地形图墙体重合；狗位姿不漂到地图轮廓外 |
| 8 | fastlio 工作区未被改动 | `git -C /my_ws/l4t/nav_ws/go2-nav2-terrain status` 干净；其 build 目录时间戳不变 |
| 9 | 地图随探索增长 | 狗走远后 `/elevation_map_raw` 的 `length_x` 增大且为 1 m 的整数倍；静止时同一世界点跨帧落在同一 cell |
| 10 | rolling 视口不丢历史 | 狗开出 30 m 视口再走回原区域，RViz 中原地形/costmap 恢复（地形层每帧整块重写） |
| 11 | 自主探索 | D 起来后 `frontier_exploration_ctl start`；连续走通 ≥3 个 frontier、无原地打转；结束后 `/exploration_complete` 发一次；`stop` 能中断 |

## 8. 停止

不要 `pkill -f ros`。先列 PID，再逐个 `kill -15`：

```bash
docker exec self_l4t bash -c "ps -eo pid,cmd | grep -E \
  'pointlio_ws/install|elevmap_fastlio/install|go2-nav2-terrain-pointlio/install|nav2_|rviz2' \
  | grep -v grep"
```

前台 launch 直接 Ctrl-C；`docker exec -d` 起的先 `kill -INT <launch_pid>`，
等 5 s 再清残留子进程。注意区分容器里其它栈的 `robot_state_publisher`。

## 9. 验证记录

- **2026-10-30**：本分支 Task 1–4 完成（relay `cloud_to_base`、`map_odom_correction`
  跟 `/aft_pgo_odom`、global costmap 改 terrain-only 30x30@0.10、local 改吃
  `/cloud_base`、launch 删除 `/scan` 与 bridge）。Task 5 只新增本 runbook 与
  `scripts/check_pointlio_chain.sh`，**未启动长跑栈**。
- **2026-10-30（端到端验收首轮，失败 → 已修复）**：三轮 E2E 全部在
  `global_costmap` configure 阶段失败，日志为
  `the class nav2_traversability_plugin::TraversabilityLayer with base class
  type nav2_costmap_2d::Layer does not exist`，随后 `bt_navigator` 失败、
  `lifecycle_manager` abort。根因：Terminal C 的命令只 source 了
  `l4t_setup.sh + 本工作区`，漏掉插件所在的 `traversability-nav2`。
  同时 `scripts/check_pointlio_chain.sh` 在 Nav2 已死的情况下仍 6/6 PASS
  （只查 topic / 参数 / TF）。
  修复：(a) 新增 `scripts/run_nav2.sh`，在一个 shell 内 source 三层环境并
  `exec ros2 launch`；(b) 检查脚本新增第 7 项 lifecycle 检查。修复后
  待重跑 E2E 验证。
- **端到端验收（§7，bag 或真机）由控制流程执行后在此追加**：
  命令、`check_pointlio_chain.sh` 输出、goal 结果、RViz 现象。
- **2026-11-04（自适应地图 + 探索接入，实现完成）**：
  - 地形栅格改自适应 bbox（`auto_extent`），后处理链换成
    `elevation_fast_postprocessor`（numpy/OpenCV，O(N)），global_costmap 改
    60 m rolling，新增 `/exploration_map` 中继与 Terminal D。
  - **实测后处理耗时（Orin NX）**：300² 134 ms、500² 328 ms、1000² 1314 ms、
    1200² 1927 ms；其中 `compute_layers` 18/48/170/252 ms，其余是 rclpy 消息
    构造（~240 ms/层/100 万格）。因此 **≤50 m 约 1 Hz；100 m 约 0.5–0.7 Hz**。
  - 离线数据通路验证（合成点云 → projector → 后处理 → bridge → 中继）：
    平地 `0`、墙 `100`、未观测 `-1`，符合预期。
  - **A/B（`use_fast_postprocessor:=true/false`）与真机探索尚未执行**，
    待控制流程按 §4 起 A/B/C/D 后补录。
- **2026-11-05（真机）`map->odom` 冻结一次**：controller_server 连续 465 条
  `Transform data too old`，`Transform time` 停在 `1793887753.967`；RViz 从
  同一时刻开始丢 odom 系消息，behavior_server 报 extrapolation。节点进程存活，
  但日志在整个冻结期间没有任何 WARN —— 三个提前 return 路径都会打 WARN，
  因此推测 `timerCallback` 卡在 `tf_broadcaster_->sendTransform()`（`/tf` 的
  RELIABLE 发布）。11-06 复现时未再出现（20 Hz，最大间隔 59 ms）。
  为此新增：
  - `map_odom_correction` 独立看门狗线程：>1 s 未发布时区分
    "回调没运行 / 卡在回调内 / 返回但未发布" 三种情况打 WARN，恢复时打 INFO；
  - `check_pointlio_chain.sh` 第 6 项增加时间戳前进检查，冻结 TF 会 FAIL
    （此前冻结的 TF 仍能 `tf2_echo` 出来，是假 PASS）。
- **2026-11-06（真机，再次复现）**：设目标后狗不动，controller_server 报
  `Resulting plan has 0 poses in it` / `Received plan with zero length`；
  `tf_help` 的 `Transform time` 停在 `1793933380.657`（约 121 s 前）。
  同一时刻 behavior_server 用的机器人位姿 stamp 也是 `3380.655`，RViz 持续
  丢弃 stamp 为 `3380.655` 的 odom 系消息 —— 说明不止 map->odom，至少还有
  一条与机器人位姿相关的 TF edge 同时停更。
  **新看门狗没有报"未发布"** → map_odom_correction 的 timer 回调仍在跑、
  `sendTransform` 仍返回。故障更像 **/tf 投递层**（多个 edge 同时停更），
  而不是单节点阻塞；也可能是发布端 `now()` 冻结（但同进程控制器时钟正常）。
  为此新增 `scripts/tf_health_monitor.py`：按 edge 区分
  "N 秒未收到新消息（DDS 投递停滞）" 与 "消息在到但 stamp 落后（发布端冻结）"，
  真机复现时在 Terminal E 并跑即可定位。
- **2026-11-06（根因定位：tf2 ABBA 死锁 → 已升级修复）**：TF monitor 证明
  `map->odom` 在 topic 上一直正常（另一个进程持续收到、stamp 前进），但
  controller 进程的 TF buffer 停在某一时刻。对 `controller_server` 进程
  `gdb -p <pid> -batch -ex 'thread apply all bt'` 抓到两个互等线程：
  - tf listener 线程：`BufferCore::setTransform -> testTransformableRequests
    -> tf2_ros callback`，等 `timer_to_request_map_mutex_`；
  - costmap obstacle layer 的 message_filter 线程：`Buffer::waitForTransform`
    持有 `timer_to_request_map_mutex_`，在 `addTransformableRequest` 里等
    `frame_mutex_` / `transformable_requests_mutex_`。

  → 上游 tf2 的经典 **ABBA 死锁**。死锁后该进程的 tf listener 永久停更，
  进程内所有 TF 冻结（controller 拿到空路径、狗不停/不动），其他进程不受影响。
  上游 geometry2 **0.25.24（2026-09-15）** 修复：
  "Fix ABBA deadlock between waitForTransform and testTransformableRequests
  (backport #982)"。容器内原为 **0.25.23**（2026-09-07 构建，早于修复）。
  处理：容器内把整套 tf2 升级到 0.25.24（tf2 / tf2-ros / tf2-msgs / tf2-py /
  tf2-ros-py / tf2-kdl / tf2-eigen / tf2-geometry-msgs / tf2-sensor-msgs /
  tf2-tools），已做 tf2 冒烟测试。**待真机验证后固化到 Dockerfile。**
  注意：这是间歇性概率问题，与 `source .l4t_env.sh`、看门狗、TF monitor 无关
  （11-05 在没有任何新工具时已复现；monitor 在停摆期间仍持续收到 map->odom）。
- **2026-11-06（启动方式）**：`nav2_minimal.launch.py` 去掉逐批启动。
  分批是 FAST-LIO 时代为压制启动瞬间 CPU 尖峰加的（会把 livox IMU 线程
  饿死 >1s，触发一次性 self-sync 时间戳平移）；上游换成 point_lio（逐帧处理）
  后不再需要。现在 5 个 Nav2 节点 + lifecycle_manager 一次性拉起，
  `start_delay` 5 → 3 s（只给 go2_driver / cloud relay 发 TF 和首帧点云）。
  隔离 domain 冒烟：6 个进程同时启动、lifecycle 顺序 configure；没有
  driver/TF 时 local_costmap activation 等 20 s 后失败属预期。总启动延迟
  从 17 s 降到 3 s（另加节点自身启动时间）。
  **同一个死锁 2026-10-29 已经遇到过一次**（commit `28500bf`，见
  `pointcloud_to_laserscan_node.cpp` 注释）：当时是**我们自己的节点**用
  `tf2_ros::MessageFilter`，所以用"去掉 MessageFilter、直接订阅 + 同步
  `tf2_->transform()`、取不到就丢帧"绕过了。本分支删掉了 `/scan`，那段代码
  不在运行链路上；现在触发死锁的是 **Nav2 `nav2_costmap_2d` 的 obstacle
  layer** 内部（apt 安装的库，无法用同样方式摘掉 MessageFilter），所以这次
  用上游 0.25.24 补丁做根治。旧 workaround 不需要回退。
- **2026-11-06（costmap 出现多条方向不同的走廊）**：规划失败或运行中出现
  "同一个走廊在 costmap 上变成几条不同方向"的重影，重启 Terminal C 后恢复。
  排查结论：**既不是地形重置，也不是 `map_odom_correction` 停摆**——那一轮
  0 条 controller TF 报错、0 次 progress 失败，`map->odom` 一直在发。
  根因是 **PGO 位姿源抖动**：SLC 回环优化期间 `/aft_pgo_odom` 会短暂断流
  1~2 s（那轮触发 69 次回环），旧的 `pgo_timeout_sec: 1.0` 一到就回退到原始
  LIO TF，PGO 一恢复又切回。两套对齐差一个回环修正量，`map->odom` 在两者
  之间反复跳，而 global costmap 的 terrain / obstacle 层**只增量写入、不重算
  已写过的格**，于是每次跳变都把旧对齐下的观测叠成一条"新走廊"。
  修复（三条一起）：
  - `pgo_timeout_sec: 1.0 -> 3.0`：短暂断流时沿用最后一条 PGO 位姿（位姿
    短暂冻结 3 s 对 Nav2 无害），超过 3 s 才回退；
  - `pgo_recover_sec: 2.0` 滞回：回退到 TF 后，要求 PGO 连续新鲜 2 s 才切回，
    避免在超时边界 ping-pong（`map_odom_correction` 内用状态机实现，
    `active_source_` + `pgo_fresh_streak_sec_`）；
  - `clear_costmap_on_switch: true`：**真正的源切换**且 `map->odom` 跳变超过
    `switch_clear_translation_m: 0.15` / `switch_clear_yaw_rad: 0.10` 时，
    异步调用 `/global_costmap/clear_entirely_global_costmap`，把旧对齐的
    残留清掉（同源时不动，避免每次回环都清全图）。
  隔离 domain 42 用假 `/tf` + `/aft_pgo_odom` 验证时序：PGO 在 t=5 s 断流，
  t=8 s 回退（Δ=1.000 m）并清图；t=10 s 恢复，t=12 s 切回并清图。
  遗留：`TraversabilityLayer` 只写"落在当前地形栅格内"的格，栅格外的旧格
  永不清理（`isClearable()` 为 false），所以仍可能在长时间运行后残留；彻底
  解决需要改 upstream 插件，暂用"切源清图 + 必要时重启 C"兜底。
- **2026-11-10（global costmap 重影：去掉 global 的 obstacle_layer）**：
  SLC 每秒一次的优化会让 costmap 用的 `map->base` 在 50 ms 内跳 0.1–1.5 m
  （实测 53 次/60 s），而 `obstacle_layer` 的点标记是 costmap 里唯一
  "只写一次、不重算"的内容（点云在 `base_footprint`，进 map 必过
  `map->odom`），旧标记就留在旧对齐位置 → 转弯/回环处出现多方向墙片。
  地形层每周期整块重写、自愈，所以 elevmap 正常。
  **处理**：`global_costmap.plugins` 去掉 `obstacle_layer`（只留 terrain +
  inflation，配置块保留便于回退）；local 不变，继续用 `/cloud_obstacles`
  兜细柱。同一 bag：t≈214 occupied 16,718→11,473，t≈289 19,241→12,773，
  墙边毛刺/重复墙片消失，全链仍 1.00 Hz。
  代价：global 不再看点级细柱；方案 B（按 `map->base` 跳变清 global
  costmap）未实施，需要时再叠加。
  **local 的向外发散**：`ground_filter` 改为 **stamp 精确查询 + 失败回退
  最新**（120 s 只回退 2 次），并把输出戳做成
  `preserve_input_stamp: auto|always|never`（默认 auto）：实机同钟
  （Livox 软件时间戳 ≈ Jetson 时间）时保留雷达 stamp，下游 costmap 按
  "扫到那一刻"查 odom->base；回放或回退帧重盖 `now()`。回放实测全部
  重盖，观感明显改善但未归零（残余 = 狗自身位姿延迟），实机需再验证
  costmap 的 transform_tolerance。
  完整证据、探针工具和测量坑见 `debug-log.md` 的 2026-11-10 一节。

## 10. 离线 bag 验证汇总（go2_cave_20261031_104118）

- **隔离**：真机在 `ROS_DOMAIN_ID=0`；离线测试全部 `export ROS_DOMAIN_ID=77`
  （必须在 source 之后，`.l4t_env.sh` 会把域设回 0）。bag 只播
  `/livox/*`、`/utlidar/robot_pose`、`/lf/*`，**不播 `/tf` `/tf_static`**。
- **不能重复 play 而不重启 A**：point_lio / SLC 的建图状态不会因 bag 结束而
  清空，第二次回放会把两段轨迹叠在一张图上。每次重放前重启 A（最好 A/B/C）。
- **只重启 A 而不重启 C**：costmap 会残留上一轮的点级障碍，表现为"地形 free
  但 costmap lethal / 远处发散"。要么 A/B/C 一起重启，要么在 C 里调用
  `clear_entirely_global_costmap` / `clear_entirely_local_costmap`。
- **频率要用单进程同窗口计数**：逐个 `ros2 topic hz` 的窗口错开、CLI 自身有负载，
  会产生 0.5 Hz 级别的假象。实测 1:1：
  `/aft_pgo_map 1.00`、`elevation_map_raw 1.00`、`raw_post 1.00`、
  `terrain 1.00`、`exploration 1.00`。
- **SLC 降频根因**：可视化线程每秒重建全图 + 全图 0.05 m 体素滤波（O(全部点)），
  KD 树与 `calculateEstimate()` 每轮全量拷贝（O(N)），且三个 C++ 包
  `CMAKE_BUILD_TYPE` 为空（-O0）。处理：SLC 用 `-DCMAKE_BUILD_TYPE=Release`
  重编 + 增量可视化缓存（`vis_incremental`，回环/每 60 s 全量重建）。
  同一 bag：`1.0/1.0/0.9/1.0/0.9/0.6` → 全程约 1.0 Hz；新旧地图 IoU 0.995。
  **cave bag 没有回环**，回环触发的重建尚未实测。
- **local costmap**：`publish_frequency` 2→5 Hz（RViz 滞后主因）；`update`
  10 Hz 试过并回退——更新循环 CPU 受限，实测发布只有 3.8 Hz，实机观感更差。
  5/5 下实测发布约 3.2–3.8 Hz（Nav2 在 update 循环里做截止判断，等频会漏发）。
  窗口 4×4 → **5×5 m**（0.05 m，100×100 格）。
- **旋转时 local costmap 墙发散**：local 在 `odom` 系，点云在 `camera_init`
  （世界）系，每帧经动态 `map→odom` 放置（实测全程 yaw 摆动 9°），旧标记不
  重算 → 扇形；global 每周期从持久地形图重写，所以自愈。修复：`ground_filter`
  输出改到 **`base_footprint`**（commit `462f821`），已实测扇形消失；
  `output_frame: ""` 可回退。
- **D 的停机语义**：Ctrl+C **不会**取消已发给 Nav2 的目标，必须先
  `frontier_exploration_ctl stop`，见 §4。
- **Terminal D 干跑（bag，只选点不移动）**：
  - 初始化正常：`map=/exploration_map`、global/local costmap、
    `navigate_to_pose`、QoS `map[transient_local,reliable]`；`ctl start`
    返回 `state=running, accepted=true`。
  - 选点正常：MRTSP-dp 每轮派发目标并收到 `Frontier goal accepted`，样例
    `reference=(1.09,2.85) dispatch=(1.20,3.00) size=17`、
    `(5.15,1.65)→(5.00,1.70) size=4`；目标距狗 1.3–6.1 m，全部在 60 m
    global 窗口内，未出现"窗口外目标被 blocked"。
  - 干跑 artifact：机器人位姿来自 bag、不会真的动，所以每 ~5 s 重发同一个
    目标（真机到位后会切换，到不了由 suppression 的 20 s no_progress 压掉，
    待真机验证）；bag 放完后 `/utlidar/robot_pose` 停止，出现
    `Could not transform base_link -> map: extrapolation into the past`
    并停止选点——实时运行不会出现，试跑窗口应覆盖 bag 播放期。
  - 目标簇偏小（4–17 格 @0.2 m）；若真机出现抖动小目标，把
    `min_frontier_size_cells` 从 3 调到 5–8。盲区环本次未被选中。

## 11. 到点转圈根因：DWB 最终朝向极限环

**现象**：设目标点后狗在目标附近来回转圈、迟迟不到。开阔处/顺行进方向设点
正常；靠墙或箭头朝向与接近方向差得大时必现。

**证据**（`/tmp/nav_diag` bag + controller_server 日志）：

- `controller_server` 每 10 s 报 `Failed to make progress`；progress_checker
  是 `required_movement_radius=0.5 / movement_time_allowance=10`，**只统计平移**，
  原地旋转不算前进。
- bag 重建全局位姿：狗已到目标 0.18–0.27 m，yaw 在 ~33°–73° 之间反复摆；
  `/cmd_vel_nav` 在两组 DWB 采样值之间切换：
  - `dist<=0.25` → `vx=0, wz=+0.29/+0.57`（RotateToGoal 阶段，只允许旋转）
  - `dist>0.25` → `vx=0.14, wz=-0.57`（退出窗口，PathAlign 把车头拉回路径方向）
  距离一直在 `FollowPath.xy_goal_tolerance=0.25` 边界上下穿越 → 两个阶段
  反复切换，形成**极限环**。
- goal checker `stateful=true` 已锁存 XY，只剩 `yaw_goal_tolerance=0.25 rad
  (14.3°)`；`symmetric_yaw_tolerance` 未开。目标箭头朝向与接近方向差 >90° 时
  永远命不中，于是每 10 s abort → BT 清 local costmap 重试 → recovery spin
  （spin 还报过 `Collision Ahead` / TF extrapolation）。
- **代价地图不是原因**：global_costmap 目标格 cost=0（0.5 m 内 lethal 1%），
  local_costmap 目标 0.4 m 内 100% free。
- 本次 bag 内 `map->odom` 20 Hz、最大间隔 59 ms，**没有复现** 11-05 那次的
  40 s 冻结；`/aft_pgo_odom` 平均延迟 160 ms（p90 234 ms）。

**修复**（`nav2_params.yaml`）：

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 3.15     # 只判 XY，不再要求最终朝向
progress_checker:
  movement_time_allowance: 20.0  # 10 -> 20 s，避免到点旋转被误杀
```

**影响边界**：

- 行驶中的转向/顺通道走**不受影响**：由 DWB 的 PathAlign/PathDist/GoalAlign/
  GoalDist 决定；`vy_samples=0` + `min_vel_x=0` 保证只前进+转向，不螃蟹走、
  不倒车。DWB 源码里 goal checker 参数未使用（`dwb_local_planner.cpp`），
  改 `yaw_goal_tolerance` 不会改变跟路径行为。
- 行驶中遇到折返/死胡同仍会原地掉头（纯旋转轨迹，180° 约 1–2 s）。
- 到点后**不再**为对齐箭头而转身。需要"到点面向某方向"的任务，应在 BT 里加
  显式后置旋转（rotate-to-yaw / Spin），与到达判定解耦。

**与自主探索的关系**（`l4t/frontier_exploration_ros2`）：

- `build_goal_pose` 把 goal yaw 设为**派发时刻**机器人指向 frontier 的方位角
  （`frontier_explorer_core_frontiers.cpp:475-492`），不是到达时的接近方向。
- explorer 自己的完成判据是 XY-only（`is_pose_within_xy_tolerance`，注释明确
  忽略 yaw），且 `goal_preemption_complete_if_within_m=0.5`；可见揭示/预抢占
  模型自己算 heading，不用 Nav2 goal yaw。
- 洞内路径绕弯时，派发方位角与到达方向可能差 90–180°。保留严格 yaw 会产生
  假失败：Nav2 progress abort → `get_result_callback` 记 failed attempt →
  `frontier_suppression_attempt_threshold=2` → 该区域被抑制 90 s，造成漏探索。
  因此关掉最终朝向对熔洞探索不仅无害，而且更安全。

**复现/诊断**：用 §4 的录包命令记录 `/cmd_vel_nav`、`/cmd_vel`、
`/local_costmap/costmap_raw`、`/global_costmap/costmap_raw`、`/tf` 等，
到位后看 `dist` 是否在 0.25 m 边界穿越、`vx` 是否在"纯旋转/前进+转向"间切换。

## 12. 大数据量压测（corridor 2.9 GB，2026-11-06）

用 `nav_ws/bags/corridor_20261105_112401`（2.9 GB / 295 s，环形走廊一圈）在
域 77 跑 A/B/C，采样每个进程的 CPU/RSS 与每条主话题的频率/负载。

工具在 `scripts/large/`（详见其 README）：

- `run_large_test.sh`：起 A→B→C、播 bag、看门狗、收尾、出报告
- `monitor.py`：每秒采进程 CPU/RSS + 话题频率/字节/栅格尺寸 + 系统温度/频率
- `report.py`：逐分钟画像 + 首尾趋势
- `run_qos_ab.sh` + `qos_probe.py`：单变量对照 RELIABLE vs BEST_EFFORT

### 12.1 踩过的坑（脚本已内置处理）

1. **`run_nav2.sh` 会把域改回 0**：它内部 `source /my_ws/l4t/.l4t_env.sh`，
   那里写着 `ROS_DOMAIN_ID=0`，C 于是跑在真机域上，A/B 在 77 互相看不见。
   修法：给 C 传 `L4T_SETUP=<包装文件>`，包装文件 source 完再 export 回 77；
   并加了"`/bt_navigator` 必须出现在域 77 的节点图里"的硬检查。
2. **`ros2 node list` 走 daemon 会缓存已退出的节点**，造成预检误报/等待超时。
   统一用 `--no-daemon`。
3. Nav2 的 global_costmap 激活会阻塞等 `map` 帧，而 `map->odom` 要等
   `/aft_pgo_odom`（即 bag 开始播）。所以 **不能** 在播放前等 Nav2 active，
   要在播放开始后再等（实测 1–2 s 内激活）。
4. rclpy 的 `GridMap.data` 是 `Float32MultiArray[]`，算字节数要
   `len(d.data)`；`SingleThreadedExecutor` 不 `add_node()` 时订阅/定时器
   全不执行（只有 /proc 线程在跑）。

### 12.2 满负载曲线（A+B+C，带 map_accumulator）

| 分钟 | A rss | B rss | C rss | A cpu | B cpu | C cpu | pgo Hz | pgo MB |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 260 | 263 | 379 | 73 | 18 | 78 | 0.88 | 2.4 |
| 1 | 363 | 318 | 391 | 115 | 51 | 100 | 0.87 | 7.2 |
| 2 | 447 | 337 | 392 | 120 | 46 | 98 | 0.70 | 11.1 |
| 3 | 492 | 350 | 393 | 144 | 39 | 99 | 0.52 | 14.7 |
| 4 | 585 | 366 | 395 | 166 | 37 | 97 | 0.45 | 17.6 |
| 5 | 776 | 374 | 397 | 162 | 38 | 76 | 0.47 | 19.2 |

- `/aft_pgo_map`：0.1 MB → **19.3 MB / 601k 点**，频率 0.96 → **0.45 Hz**
  （header stamp 间隔实测 ~2.0 s，是**源头**变慢，不是监控丢帧）。
- `/elevation_map_raw` 42×46 m @0.10（193k cells，0.8 MB），
  `/elevation_map_raw_post` 3.1 MB，`/terrain_traversability` 0.2 MB，
  三者与 pgo **1:1 跟随**（同样 0.45 Hz）。**B 的 projector 没有任何慢帧告警**。
- global_costmap 600×600 @0.10 全程 ~1 Hz；local_costmap 100×100 @0.05 ~3 Hz。
- 内存：A 216 → 760 MB（峰 **908 MB**），B 240 → 338，C 359 → 398。
- 系统：MemAvailable 最低 8.18 GB，温度峰 66 °C，load1 峰 10.8，CPU 频率
  883–1498 MHz（未降频）。A CPU 中位 124 %（峰 219 %），B 42 %，C 98 %。

### 12.3 RELIABLE 到底有没有影响：没有

`run_qos_ab.sh` 只起 A，用轻量探针每 55 s 切一次可靠性（BE/RE/BE/RE/BE）：

| 窗口 | 大小 | 带 map_accumulator | 去掉 map_accumulator |
|---|---:|---:|---:|
| 1 BE | 5.4 MB | 1.000 | 1.000 |
| 2 RE | 10.3 MB | 0.981 | 1.000 |
| 3 BE | 13.5 MB | 0.927 | 0.909 |
| 4 RE | 17.3 MB | 0.618 | 1.000 |
| 5 BE | 19.5–20.5 MB | 0.509 | 0.909 |

- **轻 reader 下 20 MB 仍能 1 Hz**，BE 与 RE 基本无差别 → RELIABLE 协议本身
  不是瓶颈。
- 带 `map_accumulator` 时整体掉到 0.5–0.6 Hz：它是 `/aft_pgo_map` 的
  **RELIABLE 慢消费者**（整图 voxel 哈希，后期 65–90 % 单核），会反压
  SLC 的 publish。去掉它后单跑恢复 ~1 Hz。
- 但**全链**去掉 accumulator 后仍然 0.45 Hz（full2）；把 projector 的订阅
  从 RELIABLE 改成 BEST_EFFORT 也**完全没有变化**（full3）；只开 A+B
  （无 C）同样 0.47 Hz（full4）。→ 反压来自 B 的 projector 这条消费链，
  与 QoS 无关。

### 12.4 结论

1. `/aft_pgo_map` 的 1 Hz 退化**不是 RELIABLE 造成的**；BE/RE 实测无差别。
2. 是**慢消费者反压**：只要链路上有消化不完的 reader（map_accumulator 或
   Python projector），SLC 的 publish 就会被拖到每 2 s 一帧。
3. `map_accumulator` 在本栈**没有任何消费者**（`/global_pgo_map` 无人订阅），
   却占 0.9 核 + ~350 MB，还会把上游频率砍半 → 建议从 A 的 launch 移除。
4. 42×46 m / 600k 点时 B 链 CPU 仅 ~40 %（projector 13–15 %），CPU 不是瓶颈，
   但已把上游拖到 0.45 Hz；更大的场地会先卡在这一环。要 100 m 级稳定 1 Hz，
   得减消息体积（降采样/增量发布）或把 projector 换成 C++（对应之前 C2 的判断）。
5. 内存增长主要在 A（SLC keyframes + point_lio），B/C 全程稳定在 ~400 MB。

### 12.5 优化实测：去掉 map_accumulator + 高度门限 1.0 m

改动（都在 pointlio_ws，非本仓库）：

- `simple_loop_closure_ros2/launch/full_stack_simple_lc_mid360s.launch.py`：
  `acc_node` 注释掉（保留完整说明和"需要时单独起"的命令）。
  `/aft_pgo_map` 本身已是当前 PGO 位姿下的 0.05 m 去重整图，accumulator
  再按同样分辨率去重没有新增，且它的 hash 只增不减、回环后会残留旧位姿点。
- `point_lio_ros2/config/mid360.yaml`：`preprocess.z_max` 1.5 → 1.0 m
  （雷达系）。

同一 2.9 GB corridor bag，满链 A+B+C：

| 配置 | pgo 单帧 | 点数 | 末分钟频率 | A RSS 峰 | 地形栅格 |
|---|---:|---:|---:|---:|---|
| 带 acc, z=1.5 | 19.3 MB | 601k | 0.47 Hz | 908 MB | 42×46 m |
| 无 acc, z=1.5 | 19.2 MB | 600k | 0.47 Hz | 416 MB | 42×46 m |
| 无 acc, z=1.0 | 17.2 MB | 538k | 0.53 Hz | 394 MB | 37×46 m |

- **z 1.5→1.0 只减 10 %**：雷达离地约 0.4 m，z=1.5 m（雷达系）≈ 离地 1.9 m，
  天花板早就被砍掉了，这一刀切掉的主要是 1.4–1.9 m 的上部墙面。
- 频率 0.47 → 0.53 Hz，仍受"整图每秒重发"的体积限制：全链有效吞吐约
  **9 MB/s**，而 A 单跑 + 轻 reader 能到 **18.6 MB/s**（20.5 MB × 0.909 Hz）。
- 去掉 accumulator 后 A 内存峰值 908 → 416 MB（省下的主要是 accumulator
  自己的 voxel hash 与它每秒重建的整图副本）。
- 地形栅格从 42×46 缩到 37×46 m：auto_extent 的 bbox 由点云决定，上部墙面
  点被裁掉后边缘少了几米；走廊地面范围不受影响。
- 下一步候选（按性价比）：只发 XYZ（PCL `PointXYZI` 32 B/点 → `PointXYZ`
  16 B/点，体积直接减半）；或 SLC 改增量关键帧发布 + 回环时 full rebuild。

## 2026-11-20 `go2_twist_bridge`：去掉周期性步态重发，空闲改用 Move(0,0,0)

现场现象：nav2 起来后，狗**每隔一秒左右会被"重新调一次步态"**（身体抖一下 /
踏步感）。原因在 `go2_twist_bridge`：

- 11-20 那版加了一个 `gait_timer_`（`gait_period_sec` 默认 1.0 s）周期性重发
  经典步态 flag（2049）；
- 同时**空闲时发的是 `BALANCESTAND`(1002)**，而 2026-11-05 的真机结论是
  `BalanceStand` / `StopMove` 都会让固件**丢弃经典步态 flag、回落 FreeWalk**——
  于是必须靠那个定时器再补回来，形成"周期性调整"。

改动：

- **删掉 `gait_timer_` 与 `gait_period_sec` 参数**，步态只在「空闲 → 运动」的
  那一帧补一次；
- **空闲分支从 `BALANCESTAND` 改成 `Move(0,0,0)`**（1008）。`Move(0,0,0)` 不会
  让固件丢步态，所以不再需要周期性重发；
- 文件头注释同步改成 2026-11-05 的三条结论（1002/1003 会复位、1008 零速不会、
  纯空闲不会）。

实测（零速度，狗趴着，不发任何运动指令）：8 s 内 `/api/sport/request` 共 67 条，
**全部是 1008，没有一条 2049 或 1002**。

顺带说明：`scripts/stop_robot.sh` 早先就是因为同一个原因改成发 `Move(0,0,0)`
而不是 `StopMove(1003)`——现在两者口径一致了。
