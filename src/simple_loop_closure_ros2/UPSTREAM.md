# 上游来源与本地改动

## 上游

| 项 | 值 |
|---|---|
| 直接上游 | https://github.com/ashBabu/simple_loop_closure（`ros2` 分支） |
| 更上游 | https://github.com/vectr-ucla/SimpleLoopClosure（Takahiro Aoyagi，BSD-3） |
| base commit | **未记录**（工作区 `.git` 于 2026-11-20 事故中丢失，无法回溯确切 SHA） |
| 同步方式 | 普通目录 vendor（非 submodule、非 fork），升级上游需手工 diff |

## License

BSD 3-Clause（见 `LICENSE`），无传染性，可自由修改与分发。

`thirdparty/nanoflann` 以**普通源码目录**入库（原 `.gitmodules` 已删除），因为它没有 `.git`，
当 submodule 处理会让 `git submodule` 命令行为异常。

## 本地改动清单

上游只做"里程计 + 半径搜索回环 + GTSAM 位姿图"三件事。本仓库在它之上补了一整套
面向四足机器人（Go2 + Mid360S）的因子、门控与工程化能力。

### 1. 位姿图因子（`include/simple_loop_closure/factors.hpp`）

| 因子 | 用途 |
|---|---|
| `WorldXYPhiFactor` | 3-DOF 回环边：只约束世界系 xy + yaw，残差顺序 `[yaw, x, y]`，z/roll/pitch 不进因子（坡面上 yaw 修正不会耦合进 z） |
| `WorldXYPhiPriorFactor` | **一元**绝对约束，供先验地图定位使用（target 是先验图里的常量位姿） |
| `WorldTransDiffFactor` | 世界系平移连续性（`use_world_odom_trans`），避免姿态修正被机体系里程计边积分成 xy 弯曲 |
| `ZDiffFactor` / `ZPriorFactor` | 世界 z 连续性 / Go2 相对 z 软因子 |

### 2. 回环质量控制（`loop_quality.hpp`、`loop_noise.hpp`）

进图前 6 道门：fitness → ICP overlap → yaw 修正量 → 候选 Δz → 平移一致性
（`min(max(下限, 比例×路径), 硬上限)`）→ 去重窗口。噪声模型把回环方差取 ICP fitness
并设下限，使回环与累计里程计不确定度同量级（否则图会被里程计链钉死）。

### 3. IMU 重力软先验（`gravity_buffer.hpp`、`gravity_math.hpp`）

取代旧的 Go2 2.5D 姿态先验（后者在坡面上会把 pitch 拉平）。支持三种重力源：
`lowstate`（机身 IMU）/ `livox`（Mid360S 内部 IMU）/ `utlidar`（L2），带低通、
动态门控（比力模长偏离过大时跳过该帧）与鲁棒核。

### 4. 腿运动学 z 因子（`leg_kinematics.hpp`、`leg_time_buffer.hpp`、`leg_z_integrator.hpp`）

由足端正运动学 + 足底力滞回 + 多足共识估计机体 z。**已实测失效**
（2026-11-19 field_1119 沙土大坡累计 +1.51 m），默认关闭，仅保留做 A/B。
离线评估工具：`tools/leg_z_offline.cpp`。

### 5. 先验地图定位（`prior_map.hpp`、`tools/make_prior_map.cpp`）

- `make_prior_map`：把一次 `/save_req` 落盘（`poses.csv` + `frames/*.pcd`）平移/裁剪/重采样成先验资产。
- 运行时：每个关键帧在 `prior_map_dir` 里找最近邻，ICP 通过门控后加一条
  `WorldXYPhiPriorFactor` 一元绝对约束。
- 默认关闭（`enable_prior_map: false`）；资产要求 `poses.csv` 首行带 `# frame: body`。

### 6. 可视化与落盘

- **增量可视地图缓存**（`vis_incremental`）：旧实现每秒重建全图，地图变大后 `/aft_pgo_map`
  掉到 0.6 Hz；新实现每个关键帧只变换/体素化一次，回环或每 60 s 做一次全量重建。
- `/save_req` 落盘时 `poses.csv` 写入 `# frame: body` 头，供先验地图加载器判断坐标系。

### 7. 配置与启动

| 文件 | 说明 |
|---|---|
| `config/simple_loop_closure.yaml` | 平地图默认 profile（Go2 相对 z 开、gravity 源 auto） |
| `config/simple_loop_closure_cave_proxy.yaml` | **当前默认**，洞穴/坡面起始值（leg-z 关、gravity 源 livox、半径 3 m） |
| `config/simple_loop_closure_cave_legz.yaml` | cave_proxy + leg-z 全开，仅作 A/B |
| `config/simple_loop_closure_p0_repro.yaml` | P0 回归基线（`loop_factor_mode: diagonal`），不作场地配置 |
| `launch/full_stack_simple_lc_mid360s.launch.py` | 本仓库新增：Livox → Point-LIO → SLC 一键栈，含静态 TF 与顶层 RViz |
| `rviz_cfg/*.rviz` | 本仓库新增；`full_stack_simple_lc_colored.rviz` 用 AxisColor 按高度着色 |

### 8. 构建

`CMakeLists.txt` 新增 `unitree_go` 依赖（leg-z 需要 `/lf/lowstate`）、
`leg_z_offline` / `make_prior_map` 两个工具目标，以及 11 个 gtest 目标。

## 相关文档

- `Parameters.md` — 参数总表、先验地图与建图门控说明
- `docs/slam/pointlio-user-log.md` — 全栈运行手册
