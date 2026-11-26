# SLAM 子系统文档

| 文档 | 内容 | 时效 |
|---|---|---|
| [pointlio-user-log.md](pointlio-user-log.md) | Point-LIO + simple_loop_closure 全栈运行手册：启动命令、参数表、点云密度链、离线 bag 回放、smoke test | **最后更新 2026-10-22**，见下方过时说明 |
| [warmup-start-verification.md](warmup-start-verification.md) | 暖机启动门控（`wait_for_start` / `/start_mapping`）的验证协议与实测结果 | 2026-11-26 实测 |
| [../../src/simple_loop_closure_ros2/Parameters.md](../../src/simple_loop_closure_ros2/Parameters.md) | 后端参数总表、先验地图、建图门控 | 与代码同步 |
| [../../src/point_lio_ros2/UPSTREAM.md](../../src/point_lio_ros2/UPSTREAM.md) | Point-LIO 上游来源与本地改动清单 | 与代码同步 |
| [../../src/simple_loop_closure_ros2/UPSTREAM.md](../../src/simple_loop_closure_ros2/UPSTREAM.md) | simple_loop_closure 上游来源与本地改动清单 | 与代码同步 |

## ⚠️ pointlio-user-log.md 的过时章节

该文档写于 2026-10-22，以下内容之后已失效，阅读时请以代码和 `Parameters.md` 为准：

1. **`map_accumulator` 相关段落**（第 1、2、9 节）：该包自 2026-11-06 起**默认不启动**，
   全工程无订阅者。`/aft_pgo_map` 本身已是"按当前 PGO 位姿重建 + 0.05 m 去重"的整图，
   不需要再叠一层 accumulator。
2. **`/global_pgo_map`**：不再发布。下游（`pgo_gridmap_projector`）直接消费 `/aft_pgo_map`。
3. **默认 SLC 参数文件**：文档写的是 `simple_loop_closure.yaml`，现默认
   `simple_loop_closure_cave_proxy.yaml`。
4. **建图门控**：文档写作时尚无 `wait_for_start` / `/start_mapping`，见 `Parameters.md` 末节
   与 [warmup-start-verification.md](warmup-start-verification.md)。

其余章节（启动命令、话题与 frame 校验、发布频率基线、离线回放）仍然有效。
