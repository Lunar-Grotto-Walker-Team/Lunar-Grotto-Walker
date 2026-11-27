# 摆渡节点 `lgw_transit` 设计（两点直走 + 转向对齐）

- 日期：2026-11-17（v2：收窄为"只做节点、不建树"）
- 状态：待评审
- 仓库：`l4t/lg_walker_ws`（新包 `lgw_cave_transit`）

---

## 1. 背景与动机

洞穴任务里，机器狗需要在**洞外**完成两件很朴素的事：

1. 从基地走到洞口（≤ 30 m，斜向前方）；
2. 在洞口把朝向摆正（朝向洞内）；
3. 探索结束后从洞口走回基地，站着待命。

洞内那一段由 `frontier_exploration_ros2` + 决策树负责，**不在本设计范围**。
本该由决策树安排的任务流，本次也**不做**——先把"能执行两件事"的底层能力做出来，
留好接口，后续再用一棵树把整条任务链串起来。

现有链路里完全没有洞外移动能力：`lgw_bt_planner` 的节点全是机械臂/姿态/夹爪/检测，
没有任何一处引用 `cmd_vel`、`NavigateToPose` 或 `FollowPath`。

**本设计只交付一个节点**，用法与 `frontier_exploration_ros2` 同构：常驻运行，
收到指令就执行，执行完回结果。

---

## 2. 目标 / 非目标 / 反目标

### 2.1 目标

- **G1**：提供「从当前位姿直线走到某个二维位姿」的能力。
- **G2**：提供「原地转到某个绝对朝向」的能力。
- **G3**：避障复用 nav2 的 local costmap / DWB，**不使用全局规划器、不建全局地图**。
- **G4**：节点全程不直接发布 `/cmd_vel`，所有运动都经由 nav2 的 action。
- **G5**：接口对上层（未来的 BT）友好：动作带 feedback 和结构化 result，
  失败原因可枚举。
- **G6**：目标坐标系可配：默认机器狗自身 `odom`，也可切到上游 `map`。

### 2.2 非目标

- ❌ 不建行为树、不做子树、不碰 `lgw_bt_planner`。
- ❌ 不做全局规划、不做绕障路径搜索、不做多点路径。
- ❌ 不启动/停止自主探索，不管 `frontier_exploration_ros2`。
- ❌ 不做目标抢占、抓取、卸货、巡逻。
- ❌ 不改 `frontier_exploration_ros2` 的任何代码。

### 2.3 反目标（技术上跑通但仍是失败）

- ❌ 直线被挡时 DWB 卡在局部极小值，需要人去推。
- ❌ 节点和 nav2 抢 controller —— 自己发 `cmd_vel` 绕过速度链路。
- ❌ nav2 还没起来就发 goal，然后失败退出，冷启动无人干预下任务直接挂。
- ❌ 目标点写死在代码里，现场改坐标要重编译。

---

## 3. 约束（已核实的环境事实）

| 事实 | 来源 |
|---|---|
| `odom` 系 = 机器狗自身里程计；开机原点、x 轴 = 开机朝向 | `go2_driver/src/driver.cpp:95-134` |
| `odom → base_footprint` TF 由 `go2_driver` 发布 | `driver.cpp:109-120` |
| local costmap：`global_frame: odom`、5×5 m 滚动、吃 `/cloud_ground` + `/cloud_obstacles` | `nav2_params.yaml:211-280` |
| `controller_server` 提供 `FollowPath`（有 `path` / `controller_id` / `goal_checker_id`，feedback 带 `distance_to_goal`） | `nav2_msgs/action/FollowPath` 实测 |
| `behavior_server` 已启用 `spin`，`global_frame: odom`，`max_rotational_vel: 1.2`；`Spin` 取绝对 `target_yaw` + `time_allowance` | `nav2_params.yaml:432-454`；`nav2_msgs/action/Spin` 实测 |
| `xy_goal_tolerance: 0.25`、`yaw_goal_tolerance: 3.15`（等于不判朝向） | `nav2_params.yaml` `general_goal_checker` |
| 洞穴低速档：`max_speed_xy 0.40`、`acc_lim_x 1.0`、`max_vel_theta 1.2` | `nav2_params.yaml` `FollowPath` |
| 速度链路 `controller → cmd_vel_nav → velocity_smoother → cmd_vel → twist_bridge → /api/sport/request` | `nav2_minimal.launch.py:122,131` |
| `odom` 未填 `twist`，`velocity_smoother` 必须 OPEN_LOOP | `nav2_params.yaml:476` |
| 底层 point_lio / elevmap / costmap 开机即起，本节点不负责拉起 | 用户确认 |

**推论**：本节点是"消费方"，nav2 是"能力提供方"。节点要做的只是
**把二维位姿翻译成 nav2 能吃的 action 调用**，剩下的避障、限速、速度下发
全部复用现成链路。

---

## 4. 架构

```
            上层（未来的 BT / 决策树 / 手动 ros2 action send_goal）
                              │
              TransitToPose / RotateToYaw / StopTransit
                              ▼
┌──────────────────── lgw_cave_transit ────────────────────┐
│  transit_node                                            │
│   ├ 目标解析（frame 换算 / 参数表）                       │
│   ├ 状态机（单线程 + 定时器，不阻塞）                     │
│   └ 结果/反馈                                             │
└───────────┬──────────────────────────┬───────────────────┘
            │ FollowPath               │ Spin
            ▼                          ▼
    controller_server            behavior_server        (nav2，已在跑)
            │
            ▼ cmd_vel_nav → velocity_smoother → cmd_vel → twist_bridge
```

---

## 5. 为什么不用全局规划

洞外是**开阔地、无障碍直达**。用全局规划要维护 60×60 m 滚动全局代价地图，
还要解决"未知格被当障碍"——`point_lio` 开机才开始建图，狗前方一开始全是
`NO_INFORMATION`，规划器很容易在前面直接拒规划。

直线 Path + DWB 跟踪把未知区当自由空间，既省事又不会卡在"未知道路"上。

**代价**：没有全局绕障能力。直线被挡住时 DWB 只能在 5×5 m 窗口里绕，
绕不过去就卡住。当前场景（开阔地直达）可接受，但这是明确的已知限制。

---

## 6. 接口

### 6.1 动作：`TransitToPose`（两点直走）

`action/TransitToPose.action`

```
# goal
geometry_msgs/PoseStamped target   # 只取 position；frame_id 可为 odom 或 map
float32 xy_tolerance               # 0 = 用节点默认值
bool    align_yaw                  # true 时到点后再转到 target 的朝向
float32 yaw_tolerance              # align_yaw=true 时生效；0 = 用默认值
---
# result
bool    success
string  reason                     # ok|timeout|unreachable|nav2_unavailable|tf_unavailable|invalid_goal|canceled
float32 final_xy_error
float32 final_yaw_error
---
# feedback
string  phase                      # pre_align|traverse|final_align|verify
float32 distance_remaining
float32 heading_error
```

### 6.2 动作：`RotateToYaw`（原地转）

`action/RotateToYaw.action`

```
# goal
float32 target_yaw                 # 在 target_frame 下的绝对角（rad）
float32 yaw_tolerance              # 0 = 用默认值
---
# result
bool    success
string  reason
float32 final_yaw_error
---
# feedback
float32 angular_remaining
```

### 6.3 停止：`/lgw_transit/stop`（`std_msgs/Empty`）

收到后取消在飞的 action，状态回 `IDLE`。留给上层"急停"用，也方便现场手动打断。

### 6.4 状态：`/lgw_transit/status`（`std_msgs/String`，2 Hz）

`phase=traverse;target=entrance;dist=12.31;heading=0.42`
用纯字符串是为了让未来的 BT 和其它观察方**零类型依赖**。

### 6.5 为什么用动作而不是纯话题

上层将来是一棵 BT，`behaviortree_ros2` 有现成的 `RosActionNode`，动作天然带
"成功/失败/反馈/可取消"，比话题 + 自己拼状态可靠得多。手动调试也一样：

```bash
ros2 action send_goal /lgw_transit/transit_to_pose \
  lgw_cave_transit/action/TransitToPose \
  "{target: {header: {frame_id: odom}, pose: {position: {x: 12.0, y: 4.0}}}}" --feedback
```

---

## 7. 节点设计

### 7.1 参数

```yaml
transit_node:
  ros__parameters:
    planning_frame: odom        # 或 map
    base_frame: base_footprint

    default_xy_tolerance: 0.25  # 与 nav2 xy_goal_tolerance 对齐
    default_yaw_tolerance: 0.17 # ≈10°
    path_spacing_m: 0.25        # 直线路径点间距

    pre_align_threshold_rad: 0.5   # 大角度先原地转；<=0 关闭预对齐
    pre_align_fallback: true       # 预对齐失败时改为"直接让 DWB 边走边拐"

    follow_path_timeout_s: 300.0
    spin_timeout_s: 30.0
    total_timeout_s: 360.0
    ready_wait_s: 20.0             # 等 nav2/TF 就绪的上限，而不是立刻放弃
    max_retries: 2

    controller_id: FollowPath
    goal_checker_id: general_goal_checker

    dry_run: false                 # true：只算路径/打日志，不发 action
    status_rate_hz: 2.0
    action_namespace: lgw_transit
```

### 7.2 执行流程（单线程 + 定时器状态机，不阻塞）

沿用 `frontier_explorer` 的成熟做法：单线程执行器 + 定时器驱动 + `async_send_goal`
/ `async_get_result`。**任何回调里都不做阻塞等待**（TF 用 `tf2::TimePointZero`）。

```
IDLE
 └ 收到 goal → WAIT_READY
      ├ TF(planning_frame→base_frame) 可取、/follow_path（和需要时的 /spin）就绪
      │    → PRE_ALIGN
      └ ready_wait_s 内没就绪 → ABORT(nav2_unavailable / tf_unavailable)

PRE_ALIGN  （仅当 |方位角 - 当前yaw| > pre_align_threshold_rad）
 ├ Spin(到目标的方位角) 成功 → TRAVERSE
 ├ Spin 失败/超时
 │    ├ pre_align_fallback=true → 直接 TRAVERSE（让 DWB 边走边拐）
 │    └ false → 计入重试

TRAVERSE
 ├ 取当前位姿 → 生成 [current → target] 直线 Path（planning_frame）
 ├ FollowPath(path, controller_id, goal_checker_id)
 ├ SUCCEEDED → （align_yaw ? FINAL_ALIGN : VERIFY）
 ├ ABORTED/CANCELED → 重试计数+1；未超 max_retries → 回 PRE_ALIGN（重新取位姿）
 └ 超时 → 同失败分支

FINAL_ALIGN （仅 align_yaw=true）
 ├ Spin(target yaw) → VERIFY
 └ 失败 → 重试计数+1

VERIFY
 ├ 重新取位姿算 final_xy_error / final_yaw_error
 ├ 在容差内 → SUCCEEDED
 └ 超限 → 重试；超过 max_retries → ABORT(unreachable)

任意阶段：total_timeout_s 到 → ABORT(timeout)
任意阶段：cancel / /lgw_transit/stop → 取消在飞的 FollowPath 和 Spin → CANCELED
```

`RotateToYaw` 走同一个骨架，只是跳过 PRE_ALIGN 和 TRAVERSE，直接 Spin + VERIFY。

### 7.3 直线路径生成

- 起点 = 当前 `base_frame` 在 `planning_frame` 下的位姿；
- 终点 = 目标；
- 按 `path_spacing_m` 等距插值，**至少 2 个点**；
- 每点的 orientation 填"指向终点的航向"（DWB 的 `PathAlign` 会用它），只有最后一个点
  按 `align_yaw` 填目标朝向；
- 目标距离 < 一个间距时直接给两点路径（当前 + 目标）。

### 7.4 失败处理

| reason | 含义 | 上层该怎么做 |
|---|---|---|
| `ok` | 到点且满足容差 | 继续 |
| `timeout` | 总闸到期 | 报警 / 重试整体任务 |
| `unreachable` | 重试耗尽 | 报警，人等 |
| `nav2_unavailable` | `ready_wait_s` 内 action server 没起来 | 检查 nav2 是否在跑 |
| `tf_unavailable` | 拿不到 TF | 检查 `go2_driver` / 数据源 |
| `invalid_goal` | 目标在 NaN/inf，或 frame 无法换算 | 上层传参有 bug |
| `canceled` | 被取消 | 上层主动打断 |

---

## 8. 为将来接树留的接口

用户之后要用一棵树把整条链串起来。本节点已经按"树友好"设计，届时**不需要改节点**：

- `TransitToPose` / `RotateToYaw` 都是标准 action → BT 侧用
  `BT::RosActionNode<...>` 一行封装；
- feedback 的 `phase` / `distance_remaining` 可直接映射成 BT 的进度；
- `reason` 可以让树做分支（例如 `unreachable` → 报警子树）；
- `/lgw_transit/stop` 对应 BT 的 `onHalted()`。
- 节点的 action 就绪后，树侧的 `WaitForExplorationComplete` 之类与探索握手
  的节点属于**后续另一期**，不在本包里。

---

## 9. 测试策略

### 9.1 单元测试（gtest）

纯逻辑抽到 `TransitCore`（可注入"当前位姿"和"动作完成事件"），覆盖：

- 直线路径生成：点数、间距、首尾点、退化情形（已在目标上、目标极近）。
- 角度数学：最短角差、归一化、yaw_tolerance 边界（0.169 / 0.171）。
- 容差判定：xy 各自边界（0.249 / 0.251）。
- 状态机：正常路径、FollowPath 失败重试、重试耗尽、`pre_align_fallback` 分支、
  超时、cancel、`ready_wait` 到期的 reason 选择。
- 目标 frame 换算：目标在 `map`、规划在 `odom` 时的数值正确性。

### 9.2 干跑

`dry_run: true` 时照常算路径、发 status、打日志，但**不发 action**，直接 SUCCEEDED。
可以在不动机器狗的情况下用 RViz 看生成的 Path。

### 9.3 真机

1. `dry_run:=true` 起节点，手动发一次 `TransitToPose`，确认路径合理；
2. `dry_run:=false`，先测 1~2 m 短距离，再测 30 m 全程；
3. 单独测 `RotateToYaw`（0° / 90° / 180°）；
4. 最后用 `ros2 action cancel` 和 `/lgw_transit/stop` 验证打断。

---

## 10. 风险与已知限制

| 风险 | 说明 | 缓解 |
|---|---|---|
| DWB 局部极小值 | 直线被挡时只能在 5×5 m 内绕 | 场景是开阔地；重试耗尽后 FAILURE 停在原地，不硬闯 |
| 里程计漂移 | 30 m 腿部估距 ~1–2%（0.3–0.6 m） | 用户已确认精度足够；`planning_frame: map` 可缓解 |
| 和 nav2 抢 controller | 探索侧若还在飞 NavigateToPose | FollowPath 会自然抢占；上层在发车前应确认探索已停 |
| 洞口原地掉头空间不足 | 1.5 m 洞宽 | `pre_align_fallback` 让 DWB 边走边拐兜底；实在不行把目标点配在洞外 |
| 未知区被当障碍 | 场景开阔时 DWB 会绕 | 若真出现，调 local costmap 的 unknown 处理，不改本节点 |
| `velocity_smoother` OPEN_LOOP | `odom` 没填 twist | 已确认配置正确，本节点不碰 |

---

## 11. 我在设计里替用户做的判断（可推翻）

1. **两个独立动作**（直走 / 转向）而不是一个带 flag 的动作 —— 上层组合更自由。
2. **`TransitToPose` 默认不校准朝向**，要朝向就显式 `align_yaw:=true`。
3. **`planning_frame` 默认 `odom`**，可切 `map`。
4. **就绪等待而非立即失败**（`ready_wait_s: 20`）—— 冷启动时 nav2 可能比节点起得慢。
5. **`pre_align_fallback: true`** —— 转不动时退回 DWB 边走边拐，而不是直接失败。
6. **失败重试 2 次后停在原地**，不自动趴下、不自动回基地。
7. **状态用 `std_msgs/String`** —— 让将来的 BT 和其它观察方零类型依赖。
8. **包名 `lgw_cave_transit`**，节点名 `transit_node`，action 命名空间 `/lgw_transit`。
