# 上游来源与本地改动

## 上游

| 项 | 值 |
|---|---|
| 直接上游 | https://github.com/dfloreaa/Point-LIO-ROS2 — Point-LIO 的 ROS2 移植，含 Unitree Unilidar L1/L2 支持 |
| 更上游 | https://github.com/hku-mars/Point-LIO — 原始 Point-LIO 算法 |
| ROS2 移植作者 | Daniel Florea（dflorea@uc.cl） |
| base commit | **未记录**（工作区 `.git` 于 2026-11-20 事故中丢失，无法回溯确切 SHA） |
| 同步方式 | 普通目录 vendor（非 submodule、非 fork），升级上游需手工 diff |

## License

`LICENSE` 是 **GNU GPL v2** 全文；`package.xml` 里写的 `<license>BSD</license>` 与 LICENSE 不一致，
这是上游 ROS2 移植时带进来的问题，本仓库不改动它，以 `LICENSE` 文件为准。

> ⚠️ **GPL-2.0 有传染性**。本包可以公开分发（保留 LICENSE 即可），但如果整个项目将来要闭源或商业化，
> 这一部分会受限。若必须规避，替代方案是换用 MIT/BSD 许可的 LIO 前端。

## 本地改动清单

按功能分组，均在本仓库 `src/point_lio_ros2/` 内。

### 1. 解禁 Livox Mid360S 的 CustomMsg 通路

上游默认把整条 Livox 路径注释掉，只支持通用 `sensor_msgs/PointCloud2`。

| 文件 | 改动 |
|---|---|
| `CMakeLists.txt` | 解禁 `find_package(livox_ros_driver2)` + `Boost::date_time`（Livox driver 透传时间戳处理，缺它会链接失败） |
| `package.xml` | 解禁 `<depend>livox_ros_driver2</depend>`，新增 `std_msgs` / `std_srvs` |
| `src/preprocess.h` | 解禁 `process(CustomMsg)` 与 `avia_handler` 声明 |
| `src/preprocess.cpp` | 解禁 `process(CustomMsg)` 与 `avia_handler` 实现 |
| `src/laserMapping.cpp` | 解禁 `livox_pcl_cbk` 与按 `lidar_type == AVIA` 分支订阅 |

`avia_handler` 把每点 `offset_time`（μs）写进 `curvature` 字段供后续去畸变使用。

### 2. `avia_handler` 近距过滤（blind）优先级修复

上游把 FAST-LIO2 的两个嵌套 `if` 合并成单表达式，`&&` 优先级高于 `||`，导致 blind 近距过滤
只在 z 变化且 x/y 完全不变时才生效。实测 0.5 m 内有 17% 的点漏进 `pl_surf`。已补括号修复。

### 3. 高度截断 `z_max`

`avia_handler` 中按雷达系 z 丢点（`config/mid360.yaml` 的 `preprocess.z_max`，默认 2.0 m），
用于丢掉天花板等无用点。**这是观感取舍**：离线测得会丢掉约 31% 的原始点。

### 4. 自身点过滤（self filter）与机械臂遮挡盒（manip box）

| 文件 | 改动 |
|---|---|
| `src/preprocess.h/.cpp` | `set_self_filter()` / `set_manip_box()` / `set_manip_active()`；两个轴对齐盒子，`r_max` 快速剔除 |
| `src/parameters.h` | 新增 `extern std::string manipulation_topic;` |
| `config/mid360.yaml` | `self_filter_en`、盒子尺寸、雷达安装位姿、`manip_timeout_s` |

- **self filter**：剔除机身/雷达自身返回点（盒子在 `base_link` 系）。
- **manip box**：一个更大的盒子，只在机械臂动作时生效，由 `/manipulation_mode`
  （`std_msgs/Bool`）动态开关；超时自动关闭，避免卡在 active 状态挡住视野。

### 5. 建图起始门控（start gate）

`src/laserMapping.cpp` 新增参数 `wait_for_start` 与服务 `/start_mapping`（`std_srvs/Empty`）：

- `wait_for_start=false`（默认）：行为与上游一致。
- `wait_for_start=true`：节点照常初始化（地图、TF、参数全部就绪），但在收到
  `/start_mapping` 之前，`imu_cbk` / `standard_pcb_cbk` / `livox_pcl_cbk` **丢弃所有输入**。

用途：真机上先暖机（A/B/C 全栈起来、下游等待上游数据），等机器人摆到期望的原点再触发建图，
让 LIO 世界原点落在操作员放置的位姿上。服务是幂等的，重复调用无害。

验证记录见 [`docs/slam/warmup-start-verification.md`](../../docs/slam/warmup-start-verification.md)。

### 6. `config/mid360.yaml` 的 Mid360S 适配

`lidar_type: 1`（AVIA）、`scan_line: 4`、`blind: 0.35`、`point_filter_num: 1`、
`filter_size_surf: 0.15`、`filter_size_map: 0.2`、`gravity_align: true`、
`extrinsic_est_en: false`。密度链的取值理由见 `docs/slam/pointlio-user-log.md`。

### 7. `launch/mapping_mid360.launch.py`

新增 `rviz`（默认 true）与 `wait_for_start`（默认 false）两个 launch 参数；
修掉灾后恢复时被截断的 `generate_launch_description()` 尾部。

## 相关文档

- `docs/slam/pointlio-user-log.md` — 全栈运行手册（参数、密度链、离线回放）
- `docs/slam/warmup-start-verification.md` — 暖机门控的验证协议与实测结果
- `src/simple_loop_closure_ros2/Parameters.md` — 后端参数与先验地图说明
