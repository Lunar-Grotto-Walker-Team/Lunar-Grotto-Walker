# terrain/ —— 地形高程与通行性链路

Point-LIO 建图栈的下游：把 PGO 校正后的点云地图栅格化成高程图，
再转成 Nav2 能消费的通行性代价地图。

## 链路

```
A 端  /aft_pgo_map (1 Hz, PGO 校正后的全图)
        │
        ├─> pgo_gridmap_projector ──> /elevation_map_raw
        │      └─ pgo_raster_cpp（C++ 中值聚合后端）
        │
        └─> utlidar_terrain_fill（L2 近场补洞，补 Livox 盲区）

B 端  /elevation_map_raw
        └─> elevation_fast_postprocessor ──> /elevation_map_raw_post
              └─> elevation_traversability_bridge ──> /terrain_traversability

C 端  /terrain_traversability
        └─> nav2_traversability_plugin（global_costmap 的一个图层）
```

## 包说明

| 包 | 作用 | 语言 |
|---|---|---|
| `pgo_gridmap_projector` | 把 `/aft_pgo_map` 栅格化成 GridMap；无状态，每帧按当前 PGO 位姿重算 | Python |
| `pgo_raster_cpp` | projector 的中值聚合内核（ctypes 加载的 `.so`） | C++ |
| `utlidar_terrain_fill` | 用 L2 点云补 Livox 近场盲区的地面 | Python |
| `elevation_fast_postprocessor` | O(N) 后处理：补洞 + 平滑 + slope/roughness | Python |
| `elevation_traversability_bridge` | GridMap → Nav2 `OccupancyGrid` | Python |
| `nav2_traversability_plugin` | Nav2 costmap 图层，消费 `/terrain_traversability` | C++ |

## 构建

```bash
colcon build --packages-select \
  pgo_raster_cpp utlidar_terrain_fill pgo_gridmap_projector \
  elevation_fast_postprocessor elevation_traversability_bridge \
  nav2_traversability_plugin
```

colcon 会自动解析依赖顺序（`pgo_gridmap_projector` 依赖 `pgo_raster_cpp` 与
`utlidar_terrain_fill`）。直接 `colcon build` 也可以。

### 外部依赖

| 依赖 | 来源 |
|---|---|
| `grid_map_core` / `grid_map_ros` / `grid_map_msgs` / `grid_map_cv` / `grid_map_rviz_plugin` | apt `ros-humble-grid-map-*`（2.0.1） |
| `unitree_go` | vendor 仓库 `src/unitree_ros2`（`utlidar_terrain_fill` 用） |
| `nav2_costmap_2d` / `pluginlib` | apt `ros-humble-nav2-*` |

## 启动

```bash
# 洞穴 profile（默认）
ros2 launch pgo_gridmap_projector go2_pointlio_projected_cave.launch.py

# 通用 profile
ros2 launch pgo_gridmap_projector go2_pointlio_projected.launch.py
```

常用参数：`show_rviz`、`with_bridge`、`with_utlidar_fill`、`filter_delay`、
`projector_params_file`、`rviz_config`。

> 2026-11-26 移除了旧的 `use_fast_postprocessor:=false` 路径（`grid_map_filters_demo`
> 滤波链）。它依赖 `grid_map_demos`，而 apt 没有这个包、源码目录又被
> `COLCON_IGNORE` 掉了，实际跑不起来；`elevation_fast_postprocessor` 是它的完全替代。

## 相关文档

- `pgo_gridmap_projector/config/projector_params_cave.yaml` —— 洞穴 profile 的参数与理由
- `elevation_traversability_bridge/config/bridge_params.yaml` —— 通行性阈值
- 主仓库 `docs/dependencies.md` —— 完整依赖与两阶段部署流程
