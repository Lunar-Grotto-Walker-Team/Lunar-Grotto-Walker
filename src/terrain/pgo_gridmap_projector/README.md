# pgo_gridmap_projector

把 PGO 校正后的整图 `/aft_pgo_map`（`sensor_msgs/PointCloud2`）投影成
**elevation 栅格**（`grid_map_msgs/GridMap` 的 `elevation` 层），作为地形链路
的第一跳。

```
/aft_pgo_map ──► pgo_gridmap_projector ──► /elevation_map_raw
                                             │
                        elevation_fast_postprocessor ──► /elevation_map_raw_post
                                             │
                        elevation_traversability_bridge ──► /terrain_traversability
                                                                 │
                                                    Nav2 TraversabilityLayer
```

节点**无状态**：每帧重新栅格化，没有历史可损坏；重启最多丢一个输入周期。

## 关键参数（`config/projector_params.yaml`）

- `input_topic` / `output_topic`：默认 `/aft_pgo_map` → `/elevation_map_raw`；
  不要指向 `/global_pgo_map`（map_accumulator 的 hash 只增不减，回环后残留旧位姿点）。
- `output_frame_id`：默认 `map`；节点自己不做 TF，只改 label，靠
  `camera_init→map` 的 identity 静态 TF 对齐。
- 栅格几何：`length_x` / `length_y` / `resolution`（默认 30 m × 30 m @ 0.10 m；
  放大到 50 m 会把 filter chain 拖到 0.31 Hz，成本只跟格数走）。
- `z_aggregation`：默认 `median`。**不要改成 mean / 分箱中位数**：实测格内常同时
  有地面和墙面点，p95 |dz| 会涨 8–10 cm、roughness lethal 格少 12%、slope p95
  差 21–22°（见 `go2-nav2-terrain-pointlio/debug-log.md`）。
- `ground_mode`：
  - `band`（默认，band-wide）：全局高度带 `ignore_points_below/above`
    （-1.2 / +0.5 m），适合平地/走廊；
  - `reference` + `profile: cave`：逐格地面参考，适合标高变化大的洞窟。
- `auto_extent`：按点云 bbox 扩栅格（以 5:1 量子对齐）。

## 运行

```bash
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && \
   ros2 launch pgo_gridmap_projector go2_pointlio_projected.launch.py show_rviz:=false"
```

launch 参数：`show_rviz`、`rviz_config`（band 用默认配置，cave 用
`go2_elevmap_fastlio_cave.rviz`）、`with_bridge`、`with_utlidar_fill`、
`projector_params_file`。

> 2026-11-26 起没有 `use_fast_postprocessor` 开关：旧的 `grid_map_filters_demo`
> 滤波链（依赖 apt 里没有的 `grid_map_demos`）已删除，
> `elevation_fast_postprocessor` 是无条件启动的唯一后处理。

## 栅格化：numpy 与 native

`aggregation=median` 时优先走 `pgo_raster_cpp`（C++ 内核，ctypes 加载）；
找不到库、或设了 `PGO_RASTER_NATIVE=0` 时回退 numpy。启动日志里有一行
`rasterizer=native|numpy`，可以确认实际路径。两者逐格 bit 级一致。

## 测试

```bash
colcon test --packages-select pgo_gridmap_projector && colcon test-result --verbose
```

覆盖：栅格几何（grid_map 的列优先平铺、中心 origin）、z 门限、越界、NaN 点、
偶数中位数、聚合模式、native/numpy 等价。

## 注意：订阅者数量会影响上游发布节奏

上游 `/aft_pgo_map` 是 RELIABLE depth-1 的大消息（末段 13.5 MB）。在
`CYCLONEDDS_URI` 只声明 `eth10` 的配置下，**≥2 个订阅者**会让 Cyclone 走组播、
同机数据实际过物理网口（~12 MB/s），上游 `publish()` 被拖到 ~1 s，送达率掉到
~0.6 Hz；只有 1 个订阅者时走快路径、稳定 1 Hz。修法是在 DDS 接口列表里加 `lo`。
细节见 `go2-nav2-terrain-pointlio/debug-log.md` 2026-11-12 节。
