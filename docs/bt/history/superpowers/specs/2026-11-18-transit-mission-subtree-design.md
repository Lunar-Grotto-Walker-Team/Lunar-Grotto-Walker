# 摆渡任务子树 `TransitMission` 设计

- 日期：2026-11-18
- 状态：待评审
- 涉及仓库：`l4t/lg_walker_ws`（新树 + 精简 launch）、`l4t/nav_ws/go2-nav2-terrain-pointlio`（`go2_transit` 新增 BT 插件节点）
- 依赖的前置实现：`docs/superpowers/specs/2026-11-17-cave-base-entrance-transit-design.md`（`go2_transit` 节点本身）

---

## 1. 背景与动机

`go2_transit` 节点已经完成并真机验证（直走、斜走、预转、原地转、三种打断方式）。
现在需要一个**行为树入口**，让操作员在比赛现场：

1. 一键起 A+B+C（`nav_ws/scripts/realstart/start_chain.sh`）；
2. 人手动把狗叫起来站着；
3. 把当天才公布的 `(x, y)` 敲进行为树文件；
4. 运行树 → 狗走到那个点 → 把朝向摆回"树开始那一刻的朝向"。

`(x, y)` **只有比赛当天才公布**，所以必须能当场修改。用户选定的做法是
**直接写进 BT XML 的字面量**（参考 `bt_trees/track_collect_reactive.xml`、
`bt_trees/unload_handle.xml` 里 `SetPoseStamped value="world;x;y;z;qx;qy;qz;qw"`
的写法）——改文件、重启节点即可，**不需要 `colcon build`**。

---

## 2. 目标 / 非目标 / 反目标

### 2.1 目标

- **G1**：一棵可独立执行的 BT 树 `TransitMission`，输入是写在 XML 里的两个数 `(X, Y)`。
- **G2**：`(X, Y)` 的含义是"**以树开始那一刻狗的位置为原点、那一刻的朝向为 x 轴正向**"的局部坐标。
- **G3**：到点后把朝向摆回 `yaw0`（树开始那一刻的朝向）。
- **G4**：失败自动重试，**总共 3 次尝试**（首次 + 2 次重试）。
- **G5**：现场改 `(X, Y)` 只需改 XML + 重启节点，无需编译。

### 2.2 非目标

- ❌ 不在树里做 `StandUp` / `BalanceStand`——操作员手动让狗站起来。
- ❌ 不做障碍物绕行/退让预案——那是 nav2 行为树的职责（用户明确）。
- ❌ 不启动自主探索、不做目标抓取、不做回基地。
- ❌ 不改 `go2_transit` 的动作接口与节点行为。

### 2.3 反目标

- ❌ `(X, Y)` 被当成 odom 系绝对坐标 —— 会偏出去上百米。
- ❌ 重试时重新采样原点 —— 三次尝试打的不是同一个点。
- ❌ `lgw_walker_ws` 反向依赖 `nav_ws` 的包 —— 构建环境会变得极脆。

---

## 3. 约束（已核实的事实）

| 事实 | 来源 |
|---|---|
| `/odom` 是雷达位姿（`go2_driver` 转发 `/utlidar/robot_pose`），**原点不是"A+B+C 启动时狗站的位置"** | `go2_driver/src/driver.cpp:52` 订阅 `/utlidar/robot_pose`；2026-11-18 实测：狗站着不动时 odom 读到 (-112, -65) |
| `go2_transit` 的 `planning_frame` 默认 `odom`，`base_frame` 默认 `base_footprint` | `src/go2_transit/config/cave_transit.yaml` |
| `TransitToPose` goal：`target`(PoseStamped, frame 可为 odom/map/空)、`xy_tolerance`、`align_yaw`、`yaw_tolerance`；`align_yaw=true` 时用的是 `target.pose.orientation` 的 yaw | `src/go2_transit/action/TransitToPose.action` |
| `SetPoseStamped` 解析 `frame;x;y;z;qx;qy;qz;qw` → 写 `geometry_msgs::PoseStamped` 到 blackboard，`frame_id` 可为任意非空串 | `src/lgw_bt_planner/src/pose_string_parser.cpp`、`src/set_pose_stamped_node.cpp` |
| `bt_runner` 把 `tf_buffer`、`named_targets` 注入全局 blackboard，BT 节点用 `@{key}` 取 | `src/lgw_bt_planner/src/bt_runner_node.cpp` |
| BT 插件按 `plugins: ['pkg/lib_name']` 加载，树按 `behavior_trees: ['pkg/bt_trees']` 目录自动发现 | `lgw_bt_planner.launch.py`、`behaviortree_ros2/src/bt_utils.cpp` |
| `RetryUntilSuccessful num_attempts="N"` 是"最多跑 N 次" | `go2_gait_and_light_demo.xml` 现有用法 |
| `go2_transit` 在 `nav_ws/go2-nav2-terrain-pointlio` 构建，`lgw_bt_planner` 在 `lg_walker_ws` 构建，两者互不 source | 两个工作区各自 `install/` |

---

## 4. 架构

```
lgw_bt_planner 包                                go2_transit 包
┌──────────────────────────────┐        ┌───────────────────────────────┐
│ bt_trees/transit_mission.xml │        │ plugins: TransitToLocalPoint  │
│   SetPoseStamped (字面量)     │  BT    │   ├ 捕获局部原点(x0,y0,yaw0)   │
│   TransitToLocalPoint ───────┼───────►│   ├ 局部→odom 换算             │
└──────────────────────────────┘        │   └ 调 TransitToPose 动作      │
                                        └──────────────┬────────────────┘
                                                       │ action
                                                       ▼
                                                 transit_node
                                          (FollowPath / Spin → nav2)
```

**插件放在 `go2_transit` 的理由**：BT 节点要引用 `go2_transit/action/TransitToPose`
类型。若把它放 `lgw_bt_planner`，`lgw_walker_ws` 构建时就必须 source `nav_ws` 的
install——跨工作区反向依赖，环境一变就构建失败。放在 `go2_transit` 里，
`lgw_bt_planner` 只在 launch 的 `plugins` 列表里多一个字符串，零编译依赖。

---

## 5. 行为树

`lgw_bt_planner/bt_trees/transit_mission.xml`：

```xml
<root BTCPP_format="4" main_tree_to_execute="TransitMission">
  <BehaviorTree ID="TransitMission">
    <RetryUntilSuccessful num_attempts="3">
      <Sequence>
        <!-- 比赛当天改这两个数：X 往正前方、Y 往左；单位米 -->
        <SetPoseStamped name="set_goal"
                        output_key="goal_local"
                        value="local;3.0;0.0;0.0;0;0;0;1"/>
        <TransitToLocalPoint name="go_there" goal="{goal_local}"/>
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>
</root>
```

- `frame_id` 固定写 `local`，节点据此判断需要做原点换算。
- `num_attempts="3"` = 首次 + 2 次重试（用户确认）。
- 树跑完即结束：狗停在目标点、朝向 `yaw0`。

---

## 6. `TransitToLocalPoint` 节点（go2_transit 的 BT 插件）

### 6.1 端口

| 端口 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `goal` | 入 | `geometry_msgs::PoseStamped` | 来自 `SetPoseStamped`，`frame_id` 必须是 `local` |
| `planning_frame` | 入 | string，默认 `odom` | 与 transit 节点一致 |
| `base_frame` | 入 | string，默认 `base_footprint` | |
| `xy_tolerance` | 入 | double，默认 0（用 transit 默认） | |
| `yaw_tolerance` | 入 | double，默认 0（用 transit 默认） | |
| `ready_wait_s` | 入 | double，默认 10 | 等 TF / 动作服务器的上限 |
| `result_reason` | 出 | string | 失败时把 `reason` 写到 blackboard，便于日志 |

**TF 自建，不走 `@{tf_buffer}` 端口**：`bt_runner` 虽然把 `tf_buffer` 注入了全局
blackboard，但现有 BT 节点里没有任何一个真正用过那个端口，blackboard 的类型匹配
没人验证过。节点在构造时用 `params.nh.lock()` 自建
`tf2_ros::Buffer` + `TransformListener`，便宜且确定。

### 6.2 行为

继承 `BT::RosActionNode<go2_transit::action::TransitToPose>`。

`on_tick()`：

1. **有界等待就绪**（默认 `ready_wait_s = 10`）：用 `tf_buffer` 查
   `planning_frame → base_frame`（零超时），并在需要时确认 transit 动作服务器可用。
   两者都就绪才往下走；超时才返回 `FAILURE`。
   - **不能把就绪检测甩给 `RetryUntilSuccessful`**：BT.CPP 的重试是下一个 tick 立刻
     重来（100 ms 量级），TF 没发布时三次尝试会在 200 ms 内烧完、整树直接失败。
   - 原点就在"第一次成功拿到 TF"的那一刻捕获，存 `(x0, y0, yaw0)`，
     **一旦捕获成功永不重采**（三次重试打的是同一个点）。
2. 读 `goal`，校验 `frame_id == "local"`（否则 `FAILURE` 并打 ERROR）。
3. 换算：
   `tx = x0 + X·cos yaw0 − Y·sin yaw0`
   `ty = y0 + X·sin yaw0 + Y·cos yaw0`
4. 构造动作 goal：`target.header.frame_id = planning_frame`、`position=(tx,ty,0)`、
   `orientation = quat(yaw0)`、`align_yaw = true`、容差按端口（0 表示交给节点默认）。
5. `setGoal(...)`，返回 `RUNNING`。

**结果回调要覆盖 `onFailure()`，不是 `on_result_received()`**：BT.ROS2 的
`bt_action_node.hpp` 把 ABORTED/CANCELED 走 `onFailure(...)`，只有 SUCCEEDED 走
`onResultReceived(...)`。`TransitToPose::Result.reason` 要在 `onFailure` 的
`result` 参数里取，再写进 `result_reason` 端口。

`onHalted()`：取消在飞动作（`cancel_goal_async`），清 `origin_captured_`
（下次重新执行树时重新捕获）。

### 6.3 插件注册

新文件 `src/transit_bt_nodes.cpp`：
`BT_REGISTER_ROS_NODES(factory, params)` 里
`factory.registerNodeType<TransitToLocalPoint>("TransitToLocalPoint");`

同一 `.so` 编成共享库，**install 到 `share/go2_transit/bt_plugins/`**，
launch 里写 `plugins: ['go2_transit/bt_plugins']`。

> 已核实：`lgw_bt_planner` 的插件就是这么装的
> （`install(TARGETS lgw_bt_planner_nodes LIBRARY DESTINATION share/lgw_bt_planner/bt_plugins)`），
> 实际产物是 `share/lgw_bt_planner/bt_plugins/liblgw_bt_planner_nodes.so`，
> launch 里写的是 `plugins: ['lgw_bt_planner/bt_plugins']`。
> `bt_utils.cpp` 的 `plugins` 元素格式是 **`<包名>/<share 下的子目录>`**（不是 .so 文件名），
> 目录不存在时 `RegisterPlugins` 会直接抛异常而不是跳过——路径写错是启动即崩。

---

## 7. 启动流程

`start_chain.sh` 只负责 A+B+C，不含 transit 与 BT。新增
`lgw_bt_planner/launch/transit_mission.launch.py`，只起两个进程：

- `go2_transit/transit_node`：参数用包内 `config/cave_transit.yaml`，但**必须覆盖
  `max_retries: 0`**（见下）。
- `lgw_bt_planner/bt_runner`：`plugins: ['lgw_bt_planner/bt_plugins',
  'go2_transit/bt_plugins']`；`behavior_trees: ['lgw_bt_planner/bt_trees']`；
  `action_name: lgw_bt_execution`。

**重试只留一层（`max_retries: 0` 的理由）**：`transit_node` 默认 `max_retries: 2`，
状态机会自己把整段路径重来一遍。如果树外面再包 `num_attempts="3"`，最坏情况是
**3 × 3 = 9 次 nav2 往返**，每次受 `total_timeout_s: 360` 约束，狗会在同一个点附近
反复试到十几分钟。用户说的"总共 3 次"按整件事理解，所以把这层关掉，让
`RetryUntilSuccessful` 成为唯一的重试层：失败一次 → 树重来一次 → 最多 3 轮，
最坏约 3 × 360 s。

**`bt_runner` 强依赖机械臂 SRDF**：`bt_runner_node.cpp` 构造时无条件调
`NamedTargetLoader::load_from_share()`，读不到 `agx_arm_moveit` 的 SRDF 会直接
`throw`。所以这个"精简 launch"仍要求 `agx_arm_ws` 已安装并 source
（`l4t_setup.sh` 已经包含，现状可直接跑）。

操作步骤：

```bash
# 1) A+B+C
bash /my_ws/l4t/nav_ws/scripts/realstart/start_chain.sh

# 2) 手动用遥控器让狗站起来

# 3) 起 transit + BT
docker exec -it -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && source install/setup.bash && \
   ros2 launch lgw_bt_planner transit_mission.launch.py"

# 4) 跑树
ros2 action send_goal /lgw_bt_execution btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: TransitMission}" --feedback
```

**跨工作区**：`go2_transit` 装在 `nav_ws/go2-nav2-terrain-pointlio`，而
`l4t_setup.sh` / `.l4t_env.sh` **都不含**这个工作区。所以不能直接 `ros2 launch`，
要像仓库里 `run_nav2.sh` / `run_explore.sh` 那样，用一个**包装脚本**在一个 shell 里
把两个 install 都叠上：

`lg_walker_ws/scripts/start_transit_mission.sh`：

```bash
# .l4t_nav_env.sh = .l4t_env.sh + A/B/C/D 四个导航工作区，9 ms 加载完
source /my_ws/l4t/.l4t_nav_env.sh
exec ros2 launch lgw_bt_planner transit_mission.launch.py "$@"
```

`.l4t_nav_env.sh` 由 `l4t/nav_ws/scripts/lgw_nav_env.sh --refresh` 生成，
叠加顺序 `.l4t_env.sh → pointlio_ws → elevmap_fastlio → traversability-nav2 →
go2-nav2-terrain-pointlio → frontier_exploration_ros2`，详见 `CLAUDE.md` 的环境缓存一节。

`transit_mission.launch.py` 本身留在 `lgw_bt_planner/launch/`（树和 bt_runner 都在这边）。

---

## 8. 测试策略

### 8.1 单元测试（gtest，`go2_transit/test/`）

把换算抽成纯函数放在 `transit_core`：

```cpp
Pose2D local_to_planning(const Pose2D& origin, double local_x, double local_y);
```

测试用例：
- 原点在 (0,0,0)、输入 (3,0) → (3,0)
- 原点朝向 90°、输入 (3,0) → (0,3)
- 原点 (10,20,π)、输入 (3,0) → (7,20)
- 输入 (0,0) → 原点本身

### 8.2 干跑

`transit_node` 的 `dry_run:=true` 已存在。树侧可以在 `launch` 里透传，
验证"读到 XML 的 (X,Y)、算出正确的 odom 目标、朝对方向"，不动机器狗。

### 8.3 真机

1. 狗站着不动，树里写 `(0,0)` → 应当**立即成功**（已在目标、朝向已对）；
2. 写 `(1,0)` → 前进约 1 m；
3. 写 `(0,1)` → 左移约 1 m（预转 + 直走）；
4. 写 `(2,-1)` → 斜向走，验证局部坐标系正确（不是 odom 绝对坐标）。

---

## 9. 风险与已知限制

| 风险 | 说明 | 缓解 |
|---|---|---|
| TF 尚未就绪就 tick | `bt_runner` 起得比 `go2_driver` 早 | 捕获失败返回 FAILURE，`RetryUntilSuccessful` 会重试；同时 launch 里给的 `plugins`/树加载本身有延迟 |
| 原点捕获时机 | 树开始那一刻的位姿 = 原点。**如果操作员在树启动后、第一次 tick 前推了狗**，原点是推过之后的位置 | 文档写明"按 ExecuteTree 之前不要动狗" |
| 三次尝试打同一个点 | 已按"捕获一次"设计 | — |
| 部分失败后狗已偏离 | 重试用同一个原点，目标点不变，属于预期行为 | — |
| 插件 `.so` 找不到 | `bt_runner` 需要两个工作区的 install | launch 命令里显式 source 两次；README 写清楚 |
| 障碍物挡路 | 本树只重试 3 次，不做绕行 | 用户明确：绕行预案归 nav2 行为树 |
| 到达后朝向未被复验 | `align_yaw` 的复验在 `transit_node` 里（`VERIFY`，容差 0.17 rad） | 已覆盖 |
| 重试层叠加导致时间失控 | 节点内 `max_retries: 2` × 树 `num_attempts: 3` = 最多 9 次往返 | launch 覆盖 `max_retries: 0`，只留树这一层 |
| 就绪检测被重试"烧掉" | BT 重试是下一个 tick 立刻重来，200 ms 内三次就没了 | 节点内有界等待（`ready_wait_s`），不依赖重试 |
| 新增树的首次构建 | `lg_walker_ws` 是 symlink-install：**改已有 XML 不用 build**，但**新增** `transit_mission.xml` 要 build 一次才会建软链 | 现场只改数字 → 不 build；首次部署 build 一次 |
| `bt_runner` 缺机械臂 SRDF 起不来 | 构造函数无条件加载 SRDF | 依赖 `agx_arm_ws`（`l4t_setup.sh` 已含）；README 写明 |

---

## 10. 我在设计里替用户做的判断（可推翻）

1. **插件放 `go2_transit` 而非 `lgw_bt_planner`** —— 避免跨工作区反向依赖。
2. **原点只在第一次 tick 捕获一次** —— 保证三次重试打同一个点。
3. **`onHalted()` 里清 `origin_captured_`** —— 树被取消后重新执行时要重新定位。
4. **`frame_id` 用 `local` 作为约定** —— 与 `world`、`base_link` 等现有用法区分，
   节点看到非 `local` 直接失败，避免把 odom 绝对坐标误当局部坐标。
5. **新增精简 launch 而不是改 `lgw_bt_planner.launch.py`** —— 后者会拉起
   `select_arm_observation_pose_service_node` 和 `body_pose_bridge`，导航任务用不到。
6. **"总共 3 次"按整件事理解** —— 所以把 `transit_node` 的 `max_retries` 覆盖成 0，
   `RetryUntilSuccessful` 是唯一重试层。若你要的是"节点内重试 3 次 + 整件事再 3 次"，
   把 launch 里的覆盖删掉即可。
7. **就绪等待放在节点里而不是树里** —— 树的 `Wait` 节点会把"等 TF"和"重试"混在一起，
   难以判断失败原因；节点内有界等待能给出明确日志。
8. **用包装脚本而不是让操作员手敲两条 source** —— 与仓库里 `run_nav2.sh` /
   `run_explore.sh` 的既有做法一致，减少现场手滑。
