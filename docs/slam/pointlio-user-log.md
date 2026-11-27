# pointlio_ws — SLAM 全栈运行手册

> 当前栈: `livox_ros_driver2 (Mid360S) → point_lio → simple_loop_closure_ros2 → map_accumulator → elevation_mapping / frontier_exploration`
>
> 容器: `self_l4t`,host 路径 `/home/unitree/my_ws/l4t/nav_ws/pointlio_ws` ⇄ container `/my_ws/l4t/nav_ws/pointlio_ws`。

---

## 0. 安装重建(改 code 后)

```bash
# 三包任意一个改动都要重新 build 一次
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  colcon build --packages-select point_lio simple_loop_closure map_accumulator --parallel-workers 4 &&
  source install/setup.bash
"

# 单包快编(只改了 map_accumulator)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source install/setup.bash &&
  colcon build --packages-select map_accumulator --parallel-workers 4
"

# gtest(本容器 colcon test 不可用,直接跑 build 下二进制)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash &&
  /my_ws/l4t/nav_ws/pointlio_ws/build/map_accumulator/test_voxel_hash
"

# 只启动建图
cd /my_ws/l4t/nav_ws/pointlio_ws
source /opt/ros/humble/setup.bash
source /my_ws/l4t/unitree_ros2/setup.sh
source /my_ws/l4t/ros2_ws/install/setup.bash
export DISPLAY=:1
colcon build --packages-select point_lio --parallel-workers 4
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py

```

---

## 1. 启动完整 SLAM 栈

```bash
# 默认带 RViz
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source install/setup.bash &&
  ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py
"

# 跑 bag 时把 driver 关掉,LIO/SLC/ACC 自己从 bag 拿数据
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source install/setup.bash &&
  ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py use_livox_driver:=false show_rviz:=false
"

# 只跑 map_accumulator 调试(LIO/SLC 已经在跑)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source install/setup.bash &&
  ros2 launch map_accumulator accumulator.launch.py
"
```

启动后期望进程清单:

| Node | package | 备注 |
|---|---|---|
| `livox_ros_driver2_node_MID360s` | livox_ros_driver2 | 跑 bag 时关掉 |
| `laserMapping` | point_lio | Mid360 CustomMsg |
| `point_lio_odom` | point_lio | dock |
| `simple_loop_closure` | simple_loop_closure_ros2 | iSAM2 后端 |
| `map_accumulator` | map_accumulator | **去重累积,新增** |
| `rviz_full_stack` | rviz2 | 顶层 |

---

## 2. 验证 / smoke test

```bash
# 2.1 进程清单
docker exec self_l4t bash -c "ros2 node list | grep -E 'livox|point_lio|simple_loop|map_acc|rviz_full'"

# 2.2 关键话题
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source install/setup.bash &&
  ros2 topic list | grep -E 'aft_pgo|global_pgo|cloud_registered'
"

# 2.3 frame_id 校验
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source install/setup.bash &&
  ros2 topic echo --once /global_pgo_map | grep -E 'frame_id|height|width'
"

# 2.4 发布频率确认(应该 ~1 Hz)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source install/setup.bash &&
  timeout 10 ros2 topic hz /global_pgo_map
"

# 2.5 单元测试(VoxelHash 内存模型)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash &&
  /my_ws/l4t/nav_ws/pointlio_ws/build/map_accumulator/test_voxel_hash
"

# 2.6 map_accumulator 端到端去重(验证用,不用整栈)
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source install/setup.bash &&
  ros2 run map_accumulator map_accumulator_node
"   # 另一个终端跑下面 pub
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && source install/setup.bash &&
  ros2 topic pub --once /aft_pgo_map sensor_msgs/msg/PointCloud2 '{header: {frame_id: camera_init}, height: 1, width: 3, fields: [{name: x, offset: 0, datatype: 7, count: 1},{name: y, offset: 4, datatype: 7, count: 1},{name: z, offset: 8, datatype: 7, count: 1},{name: intensity, offset: 12, datatype: 7, count: 1}], point_step: 16, row_step: 48, is_bigendian: false, is_dense: true}'
"   # 期望 ROS_INFO: Voxel count: 2(/global_pgo_map width=2)

# 点数验证
# ① accumulator 去重后总点数
ros2 topic echo --once /global_pgo_map | grep -E '^height|^width'
# ② SLC 可视地图原始点数,确认两个网格没再互相吃掉
ros2 topic echo --once /aft_pgo_map | grep -E '^height|^width'
# ③ 单帧原始密度(这个数决定之后要不要动 filter_size_surf)
ros2 topic echo --once /cloud_registered | grep -E '^height|^width'
# ④ CPU / 内存对比基线(LIO 57.8 / SLC 57.0 / acc 11.7 / rviz 54.3;全栈 RSS ~520 MB)
ps -eo pid,pcpu,rss,cmd | grep -E 'simple_loop_closure|map_accumulator|laserMapping|pointlio_mapping' | grep -v grep
# 频率
ros2 topic hz /aft_mapped_to_init # 必须守住 10 Hz

```

---

## 3. 清理进程(per CLAUDE.md,禁用 pkill -f ros)

```bash
# 单 PID 干净退出
docker exec self_l4t bash -c "ps -eo pid,cmd | grep -E 'simple_loop_closure|map_accumulator|laserMapping|rviz_full_stack|livox_ros_driver' | grep -v grep"
# 拿到 PID 后逐个 kill -15 <pid>

# 一键清理整栈(用一个 shim)
for n in simple_loop_closure map_accumulator laserMapping rviz_full_stack livox_ros_driver2_node_MID360s point_lio_odom; do
  pid=$(docker exec self_l4t bash -c "ros2 node list 2>/dev/null | grep -E '/\$n\$' | head -1" )
  [ -n "\$pid" ] && docker exec self_l4t bash -c "kill -15 \$(ros2 node info \$pid 2>/dev/null | grep Pid | awk '{print \$2}')"
done
```

---

## 4. 拓扑 & 数据流(写规档时直接抄)

```
livox_ros_driver2_node_MID360S
  └─ /livox/lidar (CustomMsg) ──┐
  └─ /livox/imu                │
                              point_lio
                                ├─ /aft_mapped_to_init (odom)
                                ├─ /cloud_registered (10Hz)
                                └─ /Laser_map (一次性,不用)
                                    │
                              simple_loop_closure_ros2
                                ├─ /aft_pgo_map (长期累积,会爆内存)
                                ├─ /aft_pgo_odom
                                └─ /pgo_path /vis_pose_graph
                                            │
                              map_accumulator(voxel-hash 去重)
                                └─ /global_pgo_map (持久 1Hz,稳态内存)
                                            │
                                  ┌─────────┴─────────┐
                            elevation_mapping    frontier_exploration
                            (2.5D GridMap)       (costmap_2d + 探索)
```

---

## 5. 关键参数(常用覆盖项)

`full_stack_simple_lc_mid360s.launch.py` 和子 yaml 暴露的覆盖入口:

| 参数 | 默认 | 何时调 |
|---|---|---|
| `use_livox_driver` | true | 跑 bag → false |
| `show_rviz` | true | 无头跑 → false |
| `acc_params_file` | `…/map_accumulator/config/accumulator.yaml` | 临时换 res / topic |
| `slc_params_file` | `…/simple_loop_closure/config/simple_loop_closure.yaml` | 试不同 keyframe 阈值 |

### 5.1 密度链 —— 最常调的一组

`/global_pgo_map` 的密度由**四级串联**决定,查问题要从上往下逐级量,别直接改参数:

| 级 | 参数 | 值 | 位置 |
|---|---|---|---|
| 1 单帧扫描体素 | `filter_size_surf` | **0.15** | `point_lio_ros2/config/mid360.yaml` |
| 1' 单帧近场盲区 | `preprocess.blind` | **0.35** | 同上 |
| 1'' preprocess 抽稀 | `point_filter_num` | **1** | 同上 |
| 2 可视地图关键帧节流 | `vis_map_cloud_frame_interval` | **0** | `simple_loop_closure/config/simple_loop_closure.yaml` |
| 3 SLC 可视地图体素 | `vis_map_voxel_leaf_size` | **0.05** | 同上 |
| 4 accumulator 去重体素 | `resolution` | **0.05** | `map_accumulator/config/accumulator.yaml` |

**两条硬约束**(2026-10-20 实测踩过):

1. **`resolution` 必须 `<= vis_map_voxel_leaf_size`**。上游细、下游粗时下游网格会把密度重新
   拍回去,点数纹丝不动且无任何日志。
2. **`0.05 是终值,不要再往下压**。数据本征分辨率下限 ≈ 0.15/√10 ≈ 0.047(单帧 0.15 体素 ×
   约 10 视角叠加),且 0.05 已细于 `elevation_mapping` 的 0.08 网格,再细纯属超采样。

**`point_lio` 那三个键只写在 `config/mid360.yaml`**。它们原先同时出现在
`mapping_mid360.launch.py` 的内联 dict 里,而 `Node(parameters=[yaml, dict])` 会把两者都变成
`--params-file`,**靠后的 dict 静默夺权且无日志** —— 改 yaml 永远不生效。改完用
`ros2 param get /laserMapping filter_size_surf` 验运行时真值。

`preprocess.z_max: 1.5` 是用户自加的高度截断(丢天花板等无用点),离线测丢 31% 原始点,
是观感取舍不是密度问题,维持不动。

### 5.2 `simple_loop_closure.yaml` 其余项

| 参数 | 值 | 备注 |
|---|---|---|
| `keyframe_dist_th` | 0.15 | 关键帧空间间隔 |
| `keyframe_angular_dist_th` | 0.05 | 关键帧角度间隔 |
| `target_frame_num` | 10 | ICP 目标帧数 |
| `target/source_voxel_leaf_size` | 0.15 | ICP 输入分辨率(与可视化无关) |
| `loop_search_time/distance/angular_th` | 30s / 30m / π | 回环搜索窗口 |
| `loop_search_frame_interval` | 3 | 每 3 帧做一次回环候选搜索 |
| `fitness_score_th` | 0.3 | ICP 接受门限 |
| `time_stamp_tolerance` | 0.05 | Go2 host-clock Livox 抖动 ~30ms |

> `vis_map_cloud_frame_interval` 只出现在 `visualizeThread`,ICP 走 `keyframes_cloud_copied_` +
> `target/source_voxel_leaf_size`,**完全不受它影响** —— 调到 2/3 不损 ICP 精度,但会掉可视地图点数。
> 它是 SLC CPU 从 44% 涨到 57% 的主因之一,需要省 CPU 时是第一旋钮。

### 5.3 `accumulator.yaml`

| 参数 | 值 | 备注 |
|---|---|---|
| `resolution` | **0.05** | voxel 边长,必须同步 5.1 第 3 级 |
| `input_topic` | `/aft_pgo_map` | |
| `output_topic` | `/global_pgo_map` | |
| `frame_id` | `camera_init` | |
| `publish_rate_hz` | 1.0 | 下游消费频率 |

### 5.4 密度实测基线(2026-10-20,Mid360S,每轮走约 10 m)

| 配置 | `/cloud_registered` | `/aft_pgo_map` | `/global_pgo_map` | CPU 合计 |
|---|---|---|---|---|
| 起点(leaf 0.2) | 1775 | 5994 | 5347 | 154.7% |
| leaf 0.1 | 1902 | 24210 | 23066 | 159.2% |
| leaf 0.08 | 1859 | 37981 | 36583 | 137.0% |
| **leaf 0.05(当前)** | 1859 | **84056** | **82273** | **180.8%** |

内存不构成约束:8.2 万 × 32 B = 2.6 MB 裸数据,accumulator RSS 39 MB(97% 是 ROS/PCL 基线),
全栈约 520 MB,其中 rviz2 193 MB。**rviz2 是全栈最大 CPU 消耗(54.3%),超过 SLC,纯可视化,
先关它再谈算法优化。**

详细推导见 `CHANGELOG_simple_loop_closure_integration.md` 第 9 节。

---

## 6. RViz 显示现状

`full_stack_simple_lc_mid360s.rviz` 内 5 个 Display:

| Topic | 颜色 | Alpha | 用途 |
|---|---|---|---|
| `/Laser_map` | 淡蓝 | 0.3 | Point-LIO 原始地图(对比,不参与下游) |
| `/aft_pgo_map` | 红 | 0.6 | PGO 校正后的当前帧 — **会无限增长,是 accumulator 的输入** |
| `/global_pgo_map` | 绿 | 0.8 | **去重累积的稳态地图,下游直接消费** |
| `/aft_pgo_odom` | 绿 | 1.0 | 位姿箭头 |
| `/path` (aft_pgo) | 白 | 1.0 | 轨迹 Path |
| `/vis_pose_graph` | — | — | MarkerArray |

关闭 `/aft_pgo_map` 时,**不要同时关** `/Laser_map`,否则看不到 LIO 原始输出做对比。

---

## 7. 下游高程图链路(pgo_gridmap_projector)

上游 `/aft_pgo_map` → 下游 elevation 图 → traversability costmap。

```
simple_loop_closure (/aft_pgo_map, 1 Hz, camera_init)
  → pgo_gridmap_projector      → /elevation_map_raw        (map, 300x300 @0.10)
  → grid_map_filters_demo      → /elevation_map_raw_post   (8 层,含 slope/roughness)
  → elevation_traversability_bridge → /terrain_traversability (600x600 @0.05)
```

**为什么不用 `elevation_mapping`**:它逐 cell 做 Kalman 融合,回环之前写进 cell 的
旧值清不掉,重访地形会出现两份错开的副本(鬼影墙)。projector 是无状态的,每帧
从零重建,永远反映当前 PGO 解。

### 7.1 构建(改 code 后)

```bash
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && colcon build --parallel-workers 4"
```

> **改了 `src/elevation_mapping/` 里的 rviz 或 launch,必须单独 rebuild
> `elevation_mapping`**,否则下游 launch 读的是 `install/` 里的旧副本 ——
> 表现是 rviz 起来了但**什么都没有**(主 display 还指着已不存在的
> `/elevation_map`),而源码看着是对的。
> ```bash
> docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
>   "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select elevation_mapping"
> ```

### 7.2 真机跑(两个终端)

终端 A — 上游(含 livox driver):

```bash
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
     use_livox_driver:=true show_rviz:=false"
```

终端 B — 下游:

```bash
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
   ros2 launch pgo_gridmap_projector go2_pointlio_projected.launch.py \
     show_rviz:=true"
```

参数:`show_rviz`(默认 true)/ `with_bridge`(默认 true,起 traversability bridge)/
`filter_delay`(默认 5 s,滤波链晚起的量)。

> **rviz 是全栈最大 CPU 消耗**(此前测到 54.3%),调试时可以先
> `show_rviz:=false`,只留数据链。

### 7.3 真机验收检查

一条命令看完整条链(**只读,不发布任何东西**,真机上安全):

```bash
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
   python3 /my_ws/l4t/nav_ws/elevmap_fastlio/install/pgo_gridmap_projector/lib/pgo_gridmap_projector/scripts/offline_chain_check.py \
     --observe-only --seconds 30"
```

期望输出(2026-10-22 真机前用 bag 跑的实际结果):

```
observed for 15 s

  /aft_pgo_map (upstream)       13 msg  0.91 Hz
  /elevation_map_raw            14 msg  0.90 Hz  frame='map'
  /elevation_map_raw_post       13 msg  0.90 Hz  frame='map'
  /terrain_traversability       13 msg  0.90 Hz  frame='map'

  raw grid: 300x300 @ 0.1 m, layers=['elevation']
  elevation: 3211/90000 cells filled (3.6%)
  post layers: 8 OK
  traversability: 600x600 @ 0.050 m
    0=FREE(2212), 85=UNKNOWN(347156), 128=SOFT(784), 254=LETHAL(9848)

PASS: the running chain is healthy
```

它会自动判这些(PASS 前有任何 FAILURES 就是没通):

- 四跳都有数据
- 下游 frame 必须是 `map`(上游是 `camera_init`,projector 负责改标签)
- 下游速率不能低于上游的 **50%** —— 低于就是滤波链饱和
- `/elevation_map_raw_post` 必须有 `slope` + `roughness`(缺了说明滤波链静默直通)
- `slope` / `roughness` 不能恒为 0(恒 0 = `mean_in_radius` 半径退化)
- traversability 取值只能是 `0/85/128/254`,且不能 100% UNKNOWN

单独想看的项:

```bash
# map -> camera_init 必须是纯 identity(全零)
timeout 5 ros2 run tf2_ros tf2_echo map camera_init 2>&1 | head -8

# 频率天平:三处都看,瓶颈一眼看出
for t in /aft_pgo_map /elevation_map_raw /elevation_map_raw_post; do
  echo "=== $t ==="; timeout 8 ros2 topic hz $t 2>&1 | tail -2
done

# 点云密度随建图增长(应该持续变大,不是恒定)
ros2 topic echo /aft_pgo_map --once --field width 2>&1 | head -1
```

**核心验收:重访测试。** 开出去再沿原路走回来,同一面墙在 `/elevation_map_raw`
里只能有**一套** cell。旧 `elevation_mapping` 方案在这里会看到两套互相错开的墙
(鬼影墙)。这是整个方案存在的理由,别只看"有没有图"。

在 rviz 里看最直接:主 display 已经指向 `/elevation_map_raw_post`,
墙体应该是一条单线,不是两条平行线。

### 7.4 离线测试(不用动硬件)

**单元测试(42 个)**:

```bash
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash >/dev/null && \
   PYTHONPATH=/my_ws/l4t/nav_ws/elevmap_fastlio/src/pgo_gridmap_projector:\$PYTHONPATH \
   python3 -m pytest src/pgo_gridmap_projector/test/ -q"
```

**bag 回放(真实传感器数据,不用真机)**:上游关掉 driver,下游照常:

```bash
# 终端 A:上游不吃真机,从 bag 拿数据
docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
     use_livox_driver:=false show_rviz:=false"

# 终端 B:下游
#   (同上 7.2 终端 B,show_rviz 可设 false)

# 终端 C:放 bag
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash && \
   ros2 bag play /my_ws/l4t/nav_ws/testdata/livox_data_01 --rate 1.0"
```

2026-10-22 实测基线(该 bag 32.5 s,终点 86 146 点):

| topic | 频率 | frame |
|---|---|---|
| `/aft_pgo_map` | 0.87 Hz | camera_init |
| `/elevation_map_raw` | 0.87 Hz | map |
| `/elevation_map_raw_post` | 0.86 Hz | map |
| `/terrain_traversability` | 0.86 Hz | map |

**合成世界自检(不需要上游,`/aft_pgo_map` 由脚本自己发)**:

```bash
# 先起下游,不要起上游
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   ros2 launch pgo_gridmap_projector go2_pointlio_projected.launch.py \
     show_rviz:=false filter_delay:=3.0" &

sleep 14
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
   python3 /my_ws/l4t/nav_ws/elevmap_fastlio/install/pgo_gridmap_projector/lib/pgo_gridmap_projector/scripts/offline_chain_check.py"
```

它发一个 20 m × 20 m 房间(地面 + 四堵 1 m 墙)并检查四跳 + 无状态性,
末行 `PASS: all four hops verified` 即通过。

> ⚠️ **默认模式会往 `/aft_pgo_map` 发假点云。绝对不要在真机上游还活着的时候
> 跑它** —— 两个发布者会同时往同一话题写,假房间会混进真地图。
> 只在"上游没起"或"上游从 bag 跑"的场景用。真机验收一律加 `--observe-only`
> (该模式不创建 publisher,只订阅)。同理,bag 回放时也别开 livox driver
> (会双路发布)。

### 7.5 性能:瓶颈在哪

**滤波链是唯一跟不上的环节**,projector 和上游都跟得上。

| 网格 | cells | `/elevation_map_raw_post` | 滞后 |
|---|---|---|---|
| 50×50 @0.10 | 250 000 | 0.31 Hz | 4.1 s |
| **30×30 @0.10(当前)** | 90 000 | **0.86 Hz** | 1.93 s |

单体计时(500×500、3200 格填充、无竞争):inpaint 0.07 s / normals 0.03 s /
**MeanInRadiusFilter 1.14 s** / 全链 1.29 s。

**上游发得更快不会让下游变快。** projector 无状态全图重栅格化,滤波链每帧扫
全部 cell,成本只跟**格数**走。上游提速只是把余量变成积压。要提速只能减格数
(`length_x/y`,成本 ∝ 面积)或换掉 `MeanInRadiusFilter`。

---

## 8. 注意事项

- **不要 `pkill -f ros` / `killall`** — 容器清理用 `kill -15 <pid>`(已记入 CLAUDE.md)。
- **apt 系统级变更先容器内验证**再固化到 Dockerfile(探索期别动 Dockerfile)。
- **l4t_setup.sh 没 source pointlio_ws**,终端 source `/my_ws/l4t/nav_ws/pointlio_ws/install/setup.bash` 后再 `ros2 launch …`。
- **`/Laser_map` 是 one-shot**(laserMapping.cpp:924 发一次再不发),不要误认为是 SLAM 输出频率。
- **`simple_loop_closure` 自身内存 3GB+/小时**(keyframes_cloud + GTSAM iSAM2 滑窗 + /aft_pgo_map re-voxelize 3GB 瞬时)
  → 长程建图必须挂 `map_accumulator`;它对原始 /aft_pgo_map 做 voxel-hash 去重,稳态 ~几十 MB/小时(取决于场景实际 repeat 率)。
- **static TF livox_frame→camera_init 在 full_stack launch 里**(x=0.20,y=0,z=0.15,跟 Point-LIO 文档示例对齐;真机做完 fastcalib 后用真值替换那一段)。
- **保存地图**:`ros2 topic pub --once /save_req std_msgs/msg/String "{data: /root/lio_maps/data/}"`(没有 idle 信号,机器人静止时再发;`save_node` 还没写)。
