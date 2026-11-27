# TransitMission 子树 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 提供一棵可在赛场上现场改坐标的 BT 树：狗从"树启动那一刻的位置"出发，走到局部坐标 `(X, Y)`，再把朝向摆回那一刻的朝向。

**Architecture:** 新增 BT 节点 `TransitToLocalPoint`（放在 `go2_transit` 包里，随该包一起导出成 BT 插件），负责"捕获局部原点 → 局部→odom 换算 → 调 `TransitToPose` 动作"；树 XML 放在 `lgw_bt_planner/bt_trees/`，坐标直接写成 `SetPoseStamped` 的字面量。

**Tech Stack:** ROS 2 Humble、C++17、BehaviorTree.CPP v4 + BT.ROS2 v0.3.0、ament_cmake、gtest。

**Spec:** `docs/superpowers/specs/2026-11-18-transit-mission-subtree-design.md`

## Global Constraints

- 编译/测试在容器内，统一用导航链缓存：`source /my_ws/l4t/.l4t_nav_env.sh`（已含 `.l4t_env.sh` 与 A/B/C/D 四个导航工作区，9 ms）。
- `go2_transit` 在 `/my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio` 构建；`lgw_bt_planner` 在 `/my_ws/l4t/lg_walker_ws` 构建。**两个方向都不能有编译期依赖**：BT 节点放 `go2_transit`，`lgw_bt_planner` 只在 launch 的 `plugins` 串里多一项。
- 编译加 `--parallel-workers 4`（Orin NX 共享机）。
- BT 插件产物必须落在 `share/<包名>/bt_plugins/`，`plugins` 里的写法是 `<包名>/bt_plugins`；目录不存在时 `bt_utils.cpp` 会抛异常。
- BT 节点不发布任何 `cmd_vel`、不阻塞、不在 `onRunning()` 里 sleep。
- 树里 `num_attempts="3"` = 首次 + 2 次重试；transit 节点在这套 launch 里 `max_retries` 覆盖为 `0`（只留树这一层重试）。
- 中文注释，风格与 `lgw_bt_planner` 现有节点一致（`stand_up_node.hpp`、`set_pose_stamped_node.cpp`）。

## Review Focus

1. **树启动后、第一次 tick 前狗被推动** —— 局部原点会记在"被推之后"的位置。测试：Task 1 的换算测试 + Task 3 验收里"先启动树再发 goal"的时序说明。
2. **TF / 动作服务器未就绪就 tick** —— 不能靠 `RetryUntilSuccessful` 烧重试（BT 重试是下一个 tick 立刻重来，200 ms 内三次就没了）。测试：Task 2 的 `onStart` 在 TF 不可用时必须返回 RUNNING（守到 `ready_wait_s` 超时才 FAILURE），Task 3 验收里用"nav2 未起"场景验证。
3. **目标是 `(0,0)`** —— 必须立刻成功，不能发零长度路径给 nav2。测试：Task 3 验收第 1 条。
4. **`goal` 的 `frame_id` 不是 `local`** —— 必须 FAILURE 并打 ERROR，不能当 odom 绝对坐标用。测试：Task 1 的 `is_local_frame` 纯函数测试 + Task 2 的校验分支。
5. **TF 抖动/查不到** —— 绝不能用 `{0,0,0}` 兜底发 goal。测试：Task 2 在 TF 缺失时 `onRunning` 只等待、不发送；Task 3 验收用"nav2 未起"场景确认没有 goal 被发出。

---

## File Structure

```
nav_ws/go2-nav2-terrain-pointlio/src/go2_transit/
├── include/go2_transit/
│   ├── transit_core.hpp          # + local_to_planning / is_local_frame 声明
│   └── transit_bt_nodes.hpp      # 新：TransitToLocalPoint
├── src/
│   ├── transit_core.cpp          # + 两个纯函数实现
│   └── transit_bt_nodes.cpp      # 新：BT 节点实现 + BT_REGISTER_ROS_NODES
├── CMakeLists.txt                # + go2_transit_bt 共享库 + install 到 share/.../bt_plugins
├── package.xml                   # + behaviortree_cpp / behaviortree_ros2
└── test/test_local_frame.cpp     # 新

lg_walker_ws/
├── src/lgw_bt_planner/bt_trees/transit_mission.xml     # 新
├── src/lgw_bt_planner/launch/transit_mission.launch.py # 新
└── scripts/start_transit_mission.sh                    # 新
```

---

### Task 1: 局部坐标换算与 frame 校验（纯函数）

**Files:**
- Modify: `src/go2_transit/include/go2_transit/transit_core.hpp`
- Modify: `src/go2_transit/src/transit_core.cpp`
- Create: `src/go2_transit/test/test_local_frame.cpp`
- Modify: `src/go2_transit/CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose2D`（已存在于 `transit_types.hpp`）
- Produces:
  - `Pose2D local_to_planning(const Pose2D & origin, double local_x, double local_y);`
  - `bool is_local_frame(const std::string & frame_id);`

- [ ] **Step 1: 写失败测试 `test/test_local_frame.cpp`**

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "go2_transit/transit_core.hpp"

using go2_transit::Pose2D;

TEST(LocalFrame, IdentityOriginIsPassThrough) {
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, 0}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 3.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
}

TEST(LocalFrame, OriginYawRotatesTheLocalXAxis) {
  // 原点朝 +y（90°）：往前走 3 m 应当落在 (0, 3)
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, M_PI_2}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 3.0, 1e-9);
}

TEST(LocalFrame, OriginOffsetIsAdded) {
  // 原点 (10,20) 朝 -x（π）：往前走 3 m 应当落在 (7, 20)
  const auto p = go2_transit::local_to_planning(Pose2D{10.0, 20.0, M_PI}, 3.0, 0.0);
  EXPECT_NEAR(p.x, 7.0, 1e-9);
  EXPECT_NEAR(p.y, 20.0, 1e-9);
}

TEST(LocalFrame, LocalYIsLeftPositive) {
  // 朝 +x，往左 2 m → (0, 2)
  const auto p = go2_transit::local_to_planning(Pose2D{0, 0, 0}, 0.0, 2.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 2.0, 1e-9);
}

TEST(LocalFrame, ZeroGoalIsTheOriginItself) {
  const auto p = go2_transit::local_to_planning(Pose2D{5.0, -3.0, 0.7}, 0.0, 0.0);
  EXPECT_NEAR(p.x, 5.0, 1e-9);
  EXPECT_NEAR(p.y, -3.0, 1e-9);
}

TEST(LocalFrame, FrameIdMustBeExactlyLocal) {
  EXPECT_TRUE(go2_transit::is_local_frame("local"));
  EXPECT_FALSE(go2_transit::is_local_frame(""));
  EXPECT_FALSE(go2_transit::is_local_frame("odom"));
  EXPECT_FALSE(go2_transit::is_local_frame("Local"));
}
```

- [ ] **Step 2: 运行确认失败**

```bash
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && colcon build --parallel-workers 4 --packages-select go2_transit && \
   source install/setup.bash && ./build/go2_transit/test_local_frame"
```
Expected: 编译失败（`local_to_planning` 未声明）。**确认 CMake 里已注册 `test_local_frame` 目标**，否则构建会静默成功、测试从未被编译。

- [ ] **Step 3: 实现两个函数**

`local_to_planning`：
`x = origin.x + local_x·cos(origin.yaw) − local_y·sin(origin.yaw)`，
`y = origin.y + local_x·sin(origin.yaw) + local_y·cos(origin.yaw)`，
`yaw = origin.yaw`。

`is_local_frame`：`return frame_id == "local";`（大小写敏感，允许前后空白由调用方 trim）。

- [ ] **Step 4: 运行确认通过**（6 个用例 PASS）

- [ ] **Step 5: Commit**

```bash
git add src/go2_transit/include src/go2_transit/src/go2_transit/test 2>/dev/null; \
git add src/go2_transit && git commit -m "feat(transit): 局部坐标换算与 frame 校验"
```

---

### Task 2: BT 节点 `TransitToLocalPoint` 与插件打包

**Files:**
- Create: `src/go2_transit/include/go2_transit/transit_bt_nodes.hpp`
- Create: `src/go2_transit/src/transit_bt_nodes.cpp`
- Modify: `src/go2_transit/CMakeLists.txt`
- Modify: `src/go2_transit/package.xml`

**Interfaces:**
- Consumes: `local_to_planning` / `is_local_frame`（Task 1）、`go2_transit/action/TransitToPose`（已存在）
- Produces:
  - BT 节点名 `"TransitToLocalPoint"`
  - 端口：入 `goal`(`geometry_msgs::PoseStamped`)、`planning_frame`(string, 默认 `odom`)、`base_frame`(string, 默认 `base_footprint`)、`xy_tolerance`(double, 默认 0)、`yaw_tolerance`(double, 默认 0)、`ready_wait_s`(double, 默认 10)；出 `result_reason`(string)
  - 插件库 `go2_transit_bt` → `share/go2_transit/bt_plugins/`

- [ ] **Step 1: 写 `transit_bt_nodes.hpp`**

```cpp
class TransitToLocalPoint : public BT::StatefulActionNode
{
public:
  TransitToLocalPoint(const std::string & name, const BT::NodeConfig & config,
                      const BT::RosNodeParams & params);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;   // 读端口 + 建 TF 监听；TF 没好也返回 RUNNING
  BT::NodeStatus onRunning() override; // 等待就绪 → 捕获原点 → 发 goal → 等结果
  void onHalted() override;            // 取消在飞动作，清原点

private:
  enum class Phase { WaitReady, WaitResult };
  BT::NodeStatus send_goal();
  bool ready() const;                  // TF 可查 && action server 在线
  ...
};
```

- [ ] **Step 2: 写 `transit_bt_nodes.cpp`**

必须做到的要点：

1. **自建 TF**：用 `params.nh.lock()` 创建自己的 `tf2_ros::Buffer` + `TransformListener`
   （**不走 `@{tf_buffer}` 端口**——现有代码里没有 BT 节点用过那个端口，
   blackboard 的类型匹配没人验证过；自建监听便宜且确定）。
2. **`onStart` 不阻塞**：读 `goal` 端口，校验 `is_local_frame(goal.header.frame_id)`，
   不合法就 ERROR + `FAILURE`。其余状态置初值后返回 `RUNNING`。
3. **`onRunning` 的就绪等待**：TF 查不到或 action server 不在线 → 若未超
   `ready_wait_s` 返回 `RUNNING`，超时返回 `FAILURE`。**根据不得下发任何 goal**。
4. **原点只捕获一次**：`ready` 第一次成立时用零超时 TF 查一次
   `planning_frame → base_frame` 存成 `origin_`，置 `origin_captured_`；之后不再重采。
5. **发 goal**：`local_to_planning(origin_, X, Y)` 得到 `(tx,ty)`；
   `TransitToPose::Goal{ target(PoseStamped, frame_id=planning_frame, position=(tx,ty,0),
   orientation=quat(origin_.yaw)), xy_tolerance, align_yaw=true, yaw_tolerance }`；
   `async_send_goal` + `async_get_result`，回到 `WaitResult`。
6. **结果**：`SUCCEEDED` → `SUCCESS`；否则把 `reason` 写 `result_reason` 端口后返回
   `FAILURE`（`SUCCEEDED` 时也写一个 `"ok"`，便于日志）。
7. **`onHalted`**：`async_cancel_goal`（有句柄时）、清 `origin_captured_`、回 `WaitReady`。
8. 注册：文件末尾

```cpp
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<go2_transit::TransitToLocalPoint>("TransitToLocalPoint");
}
```

- [ ] **Step 3: 改 `package.xml`**

加 `<depend>behaviortree_cpp</depend>`、`<depend>behaviortree_ros2</depend>`。

- [ ] **Step 4: 改 `CMakeLists.txt`**

- `find_package(behaviortree_cpp REQUIRED)`、`find_package(behaviortree_ros2 REQUIRED)`
- 新目标（名字与 install 目的地都是硬要求）：

```cmake
add_library(go2_transit_bt SHARED src/transit_bt_nodes.cpp)
target_compile_definitions(go2_transit_bt PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_transit_bt ${PROJECT_NAME}_core ${transit_typesupport})
target_include_directories(go2_transit_bt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(go2_transit_bt
  rclcpp rclcpp_action geometry_msgs nav2_msgs std_msgs tf2 tf2_ros behaviortree_cpp behaviortree_ros2)
install(TARGETS go2_transit_bt
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins)
```

> `BT_PLUGIN_EXPORT` 与 `lgw_bt_planner_nodes` 的写法一致（`lgw_bt_planner/CMakeLists.txt:328`）。

- [ ] **Step 5: 构建并确认产物落在正确目录**

```bash
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && colcon build --parallel-workers 4 --packages-select go2_transit && \
   ls -l install/go2_transit/share/go2_transit/bt_plugins/"
```
Expected: 构建成功；目录里出现 `libgo2_transit_bt.so`。

- [ ] **Step 6: Commit**

```bash
git add src/go2_transit && git commit -m "feat(transit): TransitToLocalPoint BT 节点与插件打包"
```

---

### Task 3: 树、launch、包装脚本与干跑验收

**Files:**
- Create: `lg_walker_ws/src/lgw_bt_planner/bt_trees/transit_mission.xml`
- Create: `lg_walker_ws/src/lgw_bt_planner/launch/transit_mission.launch.py`
- Create: `lg_walker_ws/scripts/start_transit_mission.sh`

**Interfaces:**
- Consumes: BT 节点 `TransitToLocalPoint`（Task 2）
- Produces: 树 `TransitMission`

- [ ] **Step 1: 写 `transit_mission.xml`**

```xml
<root BTCPP_format="4" main_tree_to_execute="TransitMission">
  <BehaviorTree ID="TransitMission">
    <RetryUntilSuccessful num_attempts="3">
      <Sequence>
        <!-- 现场改这两个数：X 往正前方、Y 往左，单位米 -->
        <SetPoseStamped name="set_goal" output_key="goal_local"
                        value="local;3.0;0.0;0.0;0;0;0;1"/>
        <TransitToLocalPoint name="go_there" goal="{goal_local}"/>
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>
</root>
```

文件头加注释块，写明：坐标含义、改哪里、`local` 是约定、不需要编译（symlink-install）。

- [ ] **Step 2: 写 `transit_mission.launch.py`**

只起两个 `Node`：
- `go2_transit/transit_node`，参数 `[pkg_share/go2_transit/config/cave_transit.yaml, {'max_retries': 0}]`
  （dict 在后会覆盖 yaml，这是刻意的，注释写明）。
- `lgw_bt_planner/bt_runner`，参数：`plugins: ['lgw_bt_planner/bt_plugins', 'go2_transit/bt_plugins']`、
  `behavior_trees: ['lgw_bt_planner/bt_trees']`、`action_name: lgw_bt_execution`、`tick_frequency: 10`、
  `output='screen'`。

- [ ] **Step 3: 写 `scripts/start_transit_mission.sh`**

```bash
#!/usr/bin/env bash
set -e
source /my_ws/l4t/.l4t_nav_env.sh
exec ros2 launch lgw_bt_planner transit_mission.launch.py "$@"
```
`chmod +x`。

- [ ] **Step 4: 构建 `lgw_bt_planner`（新增 XML 才需要）**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && source install/setup.bash && \
   colcon build --parallel-workers 4 --packages-select lgw_bt_planner"
```
Expected: 成功；`install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/transit_mission.xml` 是软链。

- [ ] **Step 5: 干跑验收（1）：(0,0) 必须立即成功**

把 XML 的 `value` 临时改成 `local;0.0;0.0;...`，`transit_node` 用 `dry_run:=true` 起
（launch 里加一个 `dry_run` 参数透传），然后发 `ExecuteTree`。

Expected：树在 **1 秒内**返回 SUCCESS，`transit_node` 日志里**没有** `FollowPath` 行
（`(0,0)` 已在其位，状态机直接 `finish_ok`）。

- [ ] **Step 6: 干跑验收（2）：就绪等待不烧重试**

nav2 **不启动**，只起本 launch（transit_node 也起不来 action server）。
Expected：树在 `ready_wait_s`（10 s）之前**保持 RUNNING**，之后 FAILURE；
日志里**没有**任何 goal 被发出。把 `ready_wait_s` 改成 2 可加快验证。

- [ ] **Step 7: 干跑验收（3）：frame 校验**

把 XML 的 `local;` 改成 `odom;`。
Expected：树立刻 FAILURE，日志 `frame_id must be 'local'`。

- [ ] **Step 8: Commit**

```bash
git add src/lgw_bt_planner/bt_trees/transit_mission.xml \
        src/lgw_bt_planner/launch/transit_mission.launch.py \
        scripts/start_transit_mission.sh
git commit -m "feat(bt): TransitMission 子树与精简 launch"
```

---

### Task 4: 记录与收尾

**Files:**
- Modify: `lgw_walker_ws/docs/`（或对应 README）记录现场操作步骤

- [ ] **Step 1: 写现场操作步骤**

一页纸：起 A+B+C（`start_chain.sh`）→ 遥控器让狗站起来 → `start_transit_mission.sh` →
改 XML 的两个数 → `send_goal ExecuteTree`。附上"停之前先跑 `stop_robot.sh`"。

- [ ] **Step 2: 全量测试**

```bash
docker exec -w /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio self_l4t bash -c \
  "source /my_ws/l4t/.l4t_nav_env.sh && colcon test --packages-select go2_transit && \
   colcon test-result --test-result-base build/go2_transit --verbose"
```
Expected: 0 failures。

- [ ] **Step 3: Commit**

```bash
git add -A && git commit -m "docs(bt): TransitMission 现场操作步骤"
```

---

## Self-Review

- **Spec 覆盖**：§5 树 → Task 3；§6.1/6.2 节点 → Task 2；§6.3 插件打包 → Task 2 Step 4/5；§7 启动流程 → Task 3；§8.1 单测 → Task 1；§8.2 干跑 → Task 3 Step 5-7；§10 各项判断体现在对应任务。
- **与 spec 的偏差（已同步进 spec）**：BT 节点不用 `@{tf_buffer}` 端口，改为自建 TF 监听——理由写在 Task 2 Step 2 第 1 条。
- **类型一致性**：`local_to_planning(Pose2D, double, double) -> Pose2D` 在 Task 1 定义、Task 2 消费；端口名 `goal` / `planning_frame` / `base_frame` / `xy_tolerance` / `yaw_tolerance` / `ready_wait_s` / `result_reason` 在 Task 2 与 Task 3 的 XML 中一致。
- **Review Focus**：5 条分别由 Task 1（1、4）、Task 2 Step 2 第 3/4 条（2、5）、Task 3 Step 5-7（1、3、4、5）覆盖。
