# 上游来源与本地改动

## 上游

| 项 | 值 |
|---|---|
| 仓库 | https://github.com/mertgulerx/frontier-exploration-ros2 |
| 基线 | tag `v1.6.1`（commit `b0fad50`，2026-06-11） |
| License | Apache-2.0（见 `LICENSE`，上游原文未改） |
| 引入方式 | 平铺 vendor：上游树 + 我们的改动，无上游提交历史 |
| 本地改动 | 35 个文件，**+5804 / −43** |

上游最新是 `v1.6.2`，我们停在 `v1.6.1`：我们的改动面很大（见下），
升级上游前必须先确认新版本的 frontier 选点/状态机没有被重写。

## 为什么放主仓库而不是独立仓库

Apache-2.0 是宽松许可，与主仓库其余代码（BSD-3 / Apache-2.0 / 团队自写）兼容，
不存在 `point_lio`（GPL-2.0）那种「必须隔离」的问题。按《第三方仓库修改与入库
通用指南》§2 属于 **D 平铺 vendor**，与 `simple_loop_closure_ros2` 同等处理。

## 本地改动

按功能分组。没有按组拆成独立提交——原始工作目录没有 `.git`，历史无法回溯；
下面的分组是代码层面的划分，便于 review 和后续跟上游。

### 1. 洞穴几何与地面参考（新增）

| 文件 | 内容 |
|---|---|
| `include/.../cave_geometry.hpp` + `src/cave_geometry.cpp` | 由决策图反推洞穴边界/地面高度，供选点避开洞外区域 |
| `src/frontier_explorer_core_maps.cpp` | 地图回调里接入 `cave_geometry` |

### 2. 方向策略（新增）

| 文件 | 内容 |
|---|---|
| `include/.../direction_policy.hpp` + `src/direction_policy.cpp` | 给候选 frontier 打分时的朝向约束（避免选到需要掉头/侧移的目标） |

### 3. 路口栈与轨迹缓冲（新增）

| 文件 | 内容 |
|---|---|
| `include/.../junction_stack.hpp` + `src/junction_stack.cpp` | 记录走过的路口，回退/重访时用 |
| `include/.../trajectory_buffer.hpp` + `src/trajectory_buffer.cpp` | 最近轨迹缓存，用于「离已走路径过近」的抑制判据 |

### 4. 任务状态机（新增，最大一块）

| 文件 | 内容 |
|---|---|
| `include/.../frontier_explorer_core.hpp`（+258） | 状态机与对外接口声明 |
| `src/frontier_explorer_core_mission.cpp`（+889） | 探索任务的阶段流转：初始化 → 选点 → 执行 → 完成/失败去抖 |
| `msg/ExplorationState.msg`（新增） | 对外发布任务状态 |
| `src/frontier_explorer_core_dispatch.cpp`（+395） | 按状态分发回调 |

### 5. 选点与抑制

| 文件 | 改动 |
|---|---|
| `src/frontier_search.cpp`（+353） | 候选打分、过滤 |
| `src/frontier_suppression.cpp`（+23）、`include/.../frontier_suppression.hpp` | 抑制规则（近路径、年轻 frontier 等） |

### 6. 节点与控制服务

| 文件 | 改动 |
|---|---|
| `src/frontier_explorer_node.cpp`（+424） | 节点主体接状态机 |
| `srv/ControlExploration.srv`（+3） | 控制服务增加动作 |

### 7. 配置与构建

| 文件 | 改动 |
|---|---|
| `CMakeLists.txt`（+63） | 新源文件、`msg` 生成 |
| `config/params.yaml`（+94） | 新参数（洞穴、方向、抑制、状态机） |
| `README.md`（+34/-…） | 补充我们的用法说明 |

### 8. 测试（新增 4 个，扩充 3 个）

| 文件 | 行数 |
|---|---|
| `test/test_mission_state_machine.cpp` | +1169 |
| `test/test_junction_stack.cpp` | +71 |
| `test/test_trajectory_buffer.cpp` | +90 |
| `test/test_direction_policy.cpp` | +88 |
| `test/test_cave_geometry.cpp` | +60 |
| `test/test_frontier_snapshot_and_search.cpp` | +374 |
| `test/test_control_service_and_idle.cpp` / `test_preemption_flow.cpp` | 少量适配 |

## 文档

原先随包放在 `user-log.md` 的那份运行笔记已移到
[`docs/nav/frontier-user-log.md`](../../docs/nav/frontier-user-log.md)，
与导航栈的其它文档放在一起。

## 构建与测试

```bash
colcon build --packages-select frontier_exploration_ros2
colcon test --packages-select frontier_exploration_ros2 && colcon test-result --verbose
```

## 跟上游更新

没有保留上游提交历史，只能手工比对：

```bash
# 在临时目录
git clone https://github.com/mertgulerx/frontier-exploration-ros2.git /tmp/fe
cd /tmp/fe && git checkout v1.6.2
diff -ru /tmp/fe <本目录> --exclude=build --exclude=install --exclude=log
```

冲突面集中在上面第 4、5 组（`frontier_explorer_core*` 与 `frontier_search`）——
上游若改了选点算法，需要逐函数对照合并，不能整文件覆盖。

> 若将来上游改动变频繁，建议改成 `git subtree`（与 vendor 仓库的做法一致），
> 那时可以用 `git subtree pull` 而不是手工 diff。
