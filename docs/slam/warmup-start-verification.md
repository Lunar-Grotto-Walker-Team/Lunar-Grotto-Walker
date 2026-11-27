# 暖机启动门控验证记录

**被测功能**：`point_lio` 的 `wait_for_start` 参数 + `/start_mapping` 服务
（实现见 `Lunar-Grotto-Walker-PointLIO` 仓库的 `src/laserMapping.cpp`，
`deps.repos` 拉到本仓库的 `src/point_lio_ros2/`）。

**目的**：让 A/B/C 全栈先起来暖机（下游就绪、TF 就绪、参数就绪），但不建图；
等机器人摆到期望的起点后发一个信号，LIO 才开始吃数据，**使世界原点等于发信号那一刻的机体位姿**。

---

## 1. 验证协议

不需要真机传感器，bag 回放即可（隔离 `ROS_DOMAIN_ID`，避免和真机/其它测试互相污染）。

```bash
# 终端 1：A 端，开暖机门控，不吃真机雷达
ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
  use_livox_driver:=false show_rviz:=false wait_for_start:=true

# 终端 2：B 端
ros2 launch pgo_gridmap_projector go2_pointlio_projected_cave.launch.py show_rviz:=false

# 终端 3：C 端
bash go2-nav2-terrain-pointlio/scripts/run_nav2.sh rviz:=false

# 终端 4：回放 bag
ros2 bag play <bag>/raw --rate 1.0 \
  --topics /livox/lidar /livox/imu /utlidar/robot_pose /lf/sportmodestate /lf/lowstate /utlidar/cloud_deskewed

# 等 30 s，确认 A 在刷 "waiting for /start_mapping"，然后触发
ros2 service call /start_mapping std_srvs/srv/Empty '{}'
```

## 2. 验收判据

| # | 判据 | 通过条件 |
|---|---|---|
| 1 | 门控期丢弃输入 | A 日志持续刷 `waiting for /start_mapping`，且**没有** `preprocess: ... this frame` 帧处理日志 |
| 2 | 触发即开始 | 触发后出现 `mapping started`，紧接着 `first lidar time` |
| 3 | 缓冲区未被污染 | `IMU Initializing` 从 **1.0%** 开始（不是从缓存的中间值续上） |
| 4 | **原点语义** | 关键帧 0 的位姿 ≈ (0, 0, 0)，误差 < 5 cm |
| 5 | B 端接管 | `/elevation_map_raw_post`、`/terrain_traversability` 出图 |
| 6 | C 端接管 | `Managed nodes are active` 出现，`/global_costmap/costmap` 有输出 |
| 7 | 暖机期 TF 报错收敛 | `Could not find a connection` 在触发时刻之后**归零** |

## 3. 实测结果（2026-11-26，bag `cave_20261120_092710`，域 79）

| # | 判据 | 实测 |
|---|---|---|
| 1 | 门控期丢弃输入 | ✅ 30 s 内 `waiting for /start_mapping` 持续刷；`preprocess` 仅 2 条启动日志（`self_filter on` / `manip_box ready`），零帧处理 |
| 2 | 触发即开始 | ✅ `mapping started` → `first lidar time` → 首帧 `self_filter: ... of 20064 in / 13779 out` |
| 3 | 缓冲区未被污染 | ✅ `IMU Initializing: 1.0 % → 10% → 37% → 82% → 100%` |
| 4 | 原点语义 | ✅ 关键帧 0 = **(-0.015, 0.018, -0.004) m** |
| 5 | B 端接管 | ✅ `/elevation_map_raw_post` 1.0 Hz、`/terrain_traversability` 1.0 Hz |
| 6 | C 端接管 | ✅ `Managed nodes are active` 出现，`bt_navigator` active，`/global_costmap/costmap` 0.5 Hz |
| 7 | TF 报错收敛 | ✅ 报错时间范围 `1795685617 → 1795685663`，触发那一秒之后再无一条 |

关键证据（`/save_req` 落盘的 `poses.csv`）：

```
# frame: body
index, timestamp, x, y, z, qx, qy, qz, qw
0, 1795166894.904996, -0.014904, 0.017965, -0.004030, -0.037900, 0.124642, 0.070210, 0.988989
1, 1795166895.005372, -0.029016, 0.033629, 0.007167, ...
```

## 4. 历史记录

- **2026-11-20 16:14（真机）**：`start_chain.sh --warmup` 真机运行，门控持续 11 分 33 秒后触发，
  A 端行为正确（判据 1–3 通过）。但当时 C 端 `robot_base_frame` 仍是 `base_link`
  （已修复），Nav2 全程未激活，判据 6 未验证；判据 4 也未量化。因此本次（11-26）补测。
- **2026-11-26（bag 回放）**：判据 1–7 全部通过，见上表。

## 5. 已知限制

- 门控没有单元测试覆盖（gtest 里没有 `start_mapping` 相关用例），只有端到端验证。
- 行为树节点 `StartMapping`（`lgw_bt_planner`）编译并注册进 plugin lib，但**尚未在真机上跑过**；
  上述两次验证的触发都是 `ros2 service call`。
- 暖机期间 A 端会每 5 s 打一条 `waiting for /start_mapping`，属于预期日志，不是故障。
