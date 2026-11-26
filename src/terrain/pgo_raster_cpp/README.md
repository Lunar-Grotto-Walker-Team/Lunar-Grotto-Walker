# pgo_raster_cpp

`pgo_gridmap_projector` 的 C++ 栅格化内核：把 `(N,3)` 世界坐标点按格取
**逐格 z 中位数**，输出 elevation 栅格。语义与
`pgo_gridmap_projector.rasterizer.rasterize(aggregation="median")` **逐位一致**，
只是快一个数量级以上。

## 为什么存在

numpy 实现用 `np.lexsort((z, cell))` 分组，成本 O(N log N)：

| 点数 | numpy | 本内核 |
|---|---:|---:|
| 519k（真实帧） | 74 ms | 21 ms |
| 846k（走廊 bag 末段） | 797 ms | 31 ms |

对 1 Hz 的 `/aft_pgo_map`，numpy 在 846k 点时要吃掉一整个周期的 80%；
投影器 CPU 占用也从 ~26% 降到 ~16%。

## 实现

1. 计数排序按格聚合（一格一次的 O(N) 分桶，不做比较排序）；
2. 每格用 `std::nth_element` 取中位数；偶数个取中间两个的平均；
3. 空格保持 NaN（grid_map 的 invalid 哨兵）；
4. 格子循环用 OpenMP 并行（`find_package(OpenMP)` 可选，找不到就串行）。

平铺索引是 grid_map 的列优先约定：`flat = col * n_rows + row`。

## 构建

```bash
docker exec -w /my_ws/l4t/nav_ws/elevmap_fastlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select pgo_raster_cpp"
```

产出 `install/pgo_raster_cpp/lib/libpgo_raster.so`（C ABI，无 rclcpp 依赖）。

## 使用

`pgo_gridmap_projector` 通过 `ctypes` 按 `get_package_prefix("pgo_raster_cpp")`
加载；`aggregation=median` 时走 native，其余聚合与回退都走 numpy。

- 强制回退：`PGO_RASTER_NATIVE=0`（调试对照用）；
- 未编译本包时自动回退，投影器照常工作；
- 节点启动日志里带 `rasterizer=native|numpy`，可确认实际走的哪条路径。

## 测试

中位数的等价性由 `pgo_gridmap_projector` 的
`test/test_rasterizer.py::test_native_median_matches_numpy` 守护（随机点 + NaN +
z 门限 + 越界窗口，逐格比较，NaN 位置也一致）。native 不可用时该用例自动 skip。

```bash
colcon test --packages-select pgo_gridmap_projector && colcon test-result --verbose
```

## 注意

- 本包**不是** `/aft_pgo_map` 降频的解法。降频的根因是 CycloneDDS 只绑了 eth10，
  ≥2 个订阅者时同机数据走物理网口（~12 MB/s），见
  `go2-nav2-terrain-pointlio/debug-log.md` 2026-11-12 节。
- API 是 C 函数，无全局状态，可并发调用；输出数组由调用方分配。
