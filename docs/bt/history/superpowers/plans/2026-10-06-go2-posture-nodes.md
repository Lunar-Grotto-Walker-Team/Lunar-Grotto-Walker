# Go2 Posture BT Nodes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `lgw_bt_planner` 中新增 3 个 BT 节点(`StandUp` / `BalanceStand` / `StandDown`)驱动 Go2 机器狗的高层运动模式,加 `Go2PostureDemo` BT 树与 launch,真机可触发完整"站 → 平衡 → 站 2s → 趴"循环。

**Architecture:**
- 3 个独立 BT StatefulActionNode,各自直接继承 `BT::StatefulActionNode`,**无共享中间基类**(spec §3)。共享常量集中在 `sport_api_constants.hpp`(topic 名 + API ID + 阈值 + timeout)
- 每个节点在 onStart 一次性 publish `unitree_api/msg/Request` 到 `/api/sport/request`;onRunning 跨 tick 累计 `lf/sportmodestate` 状态(标准 RELIABLE publisher / BEST_EFFORT subscriber,spec §6)
- BT 树 `go2_posture_demo.xml` 用 BT.CPP v4 自带的 `Sequence` / `RetryUntilSuccessful` / `Sleep` 拼装,无需额外注册
- 本轮**不写 gtest**(spec §2 out-of-scope);验证靠容器内 `colcon build` + `nm -D` 符号导出 + `ros2 launch` 加载 demo XML
- 真机验证由用户在 follow-up 跑(spec §8.2)

**Tech Stack:** C++17, BT.CPP v4, BT.ROS2 v0.3.0, ROS2 Humble, `unitree_api` / `unitree_go` (cyclonedds_ws 产物), ament_cmake, rclcpp

**Spec:** `l4t/lg_walker_ws/src/lgw_bt_planner/docs/superpowers/specs/2026-10-06-go2-posture-nodes-design.md`

---

## Global Constraints

按 spec + 项目 CLAUDE.md,以下约束贯穿所有任务:

1. **容器内路径以 `/my_ws/...` 写**(host 路径 `/home/unitree/my_ws/l4t/lg_walker_ws` = container `/my_ws/l4t/lg_walker_ws`)。
2. **host 编辑文件,容器内 build/test/launch**:统一 `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "..."`。
3. **build 前 source `/my_ws/l4t/l4t_setup.sh`**:colcon build 命令统一前缀 `source /my_ws/l4t/l4t_setup.sh && colcon build ...`。
4. **不写主循环 / 不 `rclcpp::Rate` / 不 `while (rclcpp::ok())`**(BT 节点无 main loop)。
5. **本容器 `colcon test` 不可用**,验证改跑 `build/` 下 gtest 二进制(本轮无 gtest,见 Task 4-6 验证方式)。
6. **真机 + mock 禁忌**:本 BT 仅真机测试,**不写 mock_go2_driver**(spec §2 out-of-scope)。
7. **真机操作硬件安全优先**(过热 / 摔机 / 电池):停手汇报,不擅自继续。
8. **commit 信息以 `lgw:` 前缀**(沿用现有 commit 历史风格)。
9. **每步 commit 必须独立**(便于 reviewer 拒收单个步骤)。
10. **publisher QoS = RELIABLE**(KEEP_LAST(1)),subscriber QoS = BEST_EFFORT(`rclcpp::SensorDataQoS()`)(spec §6)。
11. **topic 名硬编码在常量头**(`/api/sport/request` 有前导 `/`,`lf/sportmodestate` 无前导 `/`)。

---

## 文件结构总览

```
src/lgw_bt_planner/
├── include/lgw_bt_planner/
│   ├── sport_api_constants.hpp               (Task 1 新增:topic + API ID + 阈值 + timeout)
│   ├── stand_up_node.hpp                     (Task 2 新增:StandUpNode 类声明)
│   ├── balance_stand_node.hpp                (Task 3 新增:BalanceStandNode 类声明)
│   └── stand_down_node.hpp                   (Task 4 新增:StandDownNode 类声明)
├── src/
│   ├── stand_up_node.cpp                     (Task 2 新增:StandUpNode 实现)
│   ├── balance_stand_node.cpp                (Task 3 新增:BalanceStandNode 实现)
│   ├── stand_down_node.cpp                   (Task 4 新增:StandDownNode 实现)
│   └── bt_plugin_registration.cpp            (Tasks 2/3/4 累积修改:3 行 registerNodeType)
├── bt_trees/
│   └── go2_posture_demo.xml                  (Task 5 新增:demo BT 树)
├── launch/
│   └── go2_posture_demo.launch.py            (Task 5 新增:launch wrapper)
├── CMakeLists.txt                            (Task 1 修改:find_package + NODE_SOURCES)
└── package.xml                               (Task 1 修改:加 unitree_api / unitree_go)
```

---

## Task 1: package.xml + CMakeLists.txt + sport_api_constants.hpp

**Files:**
- Modify: `src/lgw_bt_planner/package.xml`(在 `<depend>yoloe_interfaces</depend>` 后加 2 行)
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(加 2 个 find_package + 3 行 NODE_SOURCES + 2 个 ament_target_dependencies + 1 个 ament_export_dependencies)
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/sport_api_constants.hpp`

**目的:** 把 Go2 sport API 接入所需依赖装上,把所有节点共用的 topic 名/API ID/阈值/timeout 集中在 `sport_api_constants.hpp`,为后续 3 个 BT 节点提供 SSoT 常量。

**为什么先做这个:** Tasks 2-4 的 BT 节点 `.cpp` 都要 `#include` 这个常量头;Tasks 2-4 的注册都在 `bt_plugin_registration.cpp` 累积。CMakeLists.txt 改动是 build 通过的前置,任何 .cpp 文件缺失都会导致 link 失败。

### Step 1: 修改 package.xml 加依赖

在 `src/lgw_bt_planner/package.xml` line 25 (`<depend>yoloe_interfaces</depend>` 之后)插入:

```xml
  <depend>unitree_api</depend>
  <depend>unitree_go</depend>
```

### Step 2: 修改 CMakeLists.txt 加 find_package + NODE_SOURCES + ament 依赖

在 `src/lgw_bt_planner/CMakeLists.txt` 找到 `find_package(yoloe_interfaces REQUIRED)` 行(line 31),在其后插入:

```cmake
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
```

在 `set(NODE_SOURCES ...)` block 末尾(`src/compute_lookat_pose_node.cpp` 之后,line 65 左右)插入:

```cmake
  # Go2 sport API BT nodes (added 2026-10-06) — 3 independent
  # StatefulActionNode subclasses that publish /api/sport/request and
  # subscribe lf/sportmodestate. See
  # docs/superpowers/specs/2026-10-06-go2-posture-nodes-design.md §3.
  src/stand_up_node.cpp
  src/balance_stand_node.cpp
  src/stand_down_node.cpp
```

在 `ament_target_dependencies(lgw_bt_planner_nodes ...)` 列表末尾(在 `yoloe_interfaces` 后)插入:

```cmake
  unitree_api
  unitree_go
```

在 `ament_export_dependencies(...)` 列表末尾(在 `yoloe_interfaces` 后)插入:

```cmake
  unitree_api
  unitree_go
```

### Step 3: 创建 `sport_api_constants.hpp`

新建 `src/lgw_bt_planner/include/lgw_bt_planner/sport_api_constants.hpp`:

```cpp
// sport_api_constants.hpp
//
// Shared constants for Go2 sport API BT nodes (StandUp / BalanceStand /
// StandDown). Single source of truth — three .cpp files #include this
// and reference constants directly, no inheritance involved (see spec §3
// "Why no shared base class").
//
// Topic names + API IDs are mirrored from go2_bt_ws/CLAUDE.md and
// unitree_ros2/example/src/include/common/ros2_sport_client.h.
// Thresholds are the verified success/failure gates from
// go2_bt_ws/CLAUDE.md node table.

#pragma once

#include <cstdint>

namespace lgw_bt_planner::sport_api
{

// ---- Topic names ----
// Note: /api/sport/request has a leading "/", lf/sportmodestate does NOT.
inline constexpr const char* kSportRequestTopic   = "/api/sport/request";
inline constexpr const char* kSportModeStateTopic = "lf/sportmodestate";

// ---- Sport API IDs (see unitree_ros2/example/.../ros2_sport_client.h) ----
inline constexpr int32_t kApiIdStandUp      = 1004;
inline constexpr int32_t kApiIdStandDown    = 1005;
inline constexpr int32_t kApiIdBalanceStand = 1002;
inline constexpr int32_t kApiIdStopMove     = 1003;

// ---- Completion thresholds (from go2_bt_ws/CLAUDE.md node table) ----
inline constexpr float  kStandHeight    = 0.30f;
inline constexpr float  kLyingHeight    = 0.20f;
inline constexpr int    kStableFrames   = 5;
inline constexpr int8_t kBalanceMode    = 1;

// ---- Per-node timeouts (seconds) ----
inline constexpr double kStandUpTimeoutSec      = 5.0;
inline constexpr double kStandDownTimeoutSec    = 5.0;
inline constexpr double kBalanceStandTimeoutSec = 2.0;

}  // namespace lgw_bt_planner::sport_api
```

### Step 4: build 验证(CMakeLists 改动能解析)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -15
"
```

Expected: `Finished <<< lgw_bt_planner [Xs]` 出现,无 error。新 .hpp header 被识别(`sport_api_constants.hpp` 会被安装到 `install/lgw_bt_planner/include/lgw_bt_planner/`)。

### Step 5: 验证 install 路径

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  ls install/lgw_bt_planner/include/lgw_bt_planner/sport_api_constants.hpp && \
  ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/ | wc -l
"
```

Expected: header 存在;`bt_trees/` 下还是 7 个文件(本任务没改 demo XML)。

### Step 6: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/package.xml \
        src/lgw_bt_planner/CMakeLists.txt \
        src/lgw_bt_planner/include/lgw_bt_planner/sport_api_constants.hpp
git commit -m "$(cat <<'EOF'
lgw: add Go2 sport API deps + shared constants header

package.xml: depend on unitree_api, unitree_go (cyclonedds_ws
products exposed by l4t_setup.sh).
CMakeLists.txt: find_package both, append 3 NODE_SOURCES
(resolved by Tasks 2-4), extend ament_*_dependencies.
sport_api_constants.hpp: topic names (with/without leading /),
sport API IDs (1002/1003/1004/1005), completion thresholds
(0.30 m stand / 0.20 m lie / mode==1), per-node timeouts.
3 BT nodes (#include this) share no other code (spec §3 "no
shared base class" decision).
EOF
)"
```

---

## Task 2: StandUpNode — hpp + cpp + 注册 + build 验证

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/stand_up_node.hpp`
- Create: `src/lgw_bt_planner/src/stand_up_node.cpp`
- Modify: `src/lgw_bt_planner/src/bt_plugin_registration.cpp`(加 1 行 include + 1 行 registerNodeType)

**目的:** 第一个 BT 节点完整端到端走通(头 → 声明 → 实现 → 注册 → build → 符号验证),给后续 2 个节点(同模式)立模板。

**为什么先做 StandUp:** StandUp 的"5 帧稳定阈值"是 3 个节点中最复杂的稳定逻辑,先打通这个,BalanceStand/StandDown 复用同模式。

### Step 1: 创建 `stand_up_node.hpp`

新建 `src/lgw_bt_planner/include/lgw_bt_planner/stand_up_node.hpp`:

```cpp
// stand_up_node.hpp
//
// BT::StatefulActionNode that publishes a StandUp command
// (api_id=1004) to /api/sport/request and waits until
// lf/sportmodestate.body_height >= kStandHeight (0.30 m) for
// kStableFrames (5) consecutive ticks. Fails after kStandUpTimeoutSec
// (5 s). See spec §4.2 + §5.1.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace lgw_bt_planner
{

class StandUpNode : public BT::StatefulActionNode
{
public:
  StandUpNode(const std::string& name, const BT::NodeConfig& config,
              const BT::RosNodeParams& params);

  // Required by BT.CPP v4 static_assert even when no ports.
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  std::shared_ptr<rclcpp::Subscription<unitree_go::msg::SportModeState>>
      subscriber_;
  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  int stable_count_{0};
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace lgw_bt_planner
```

### Step 2: 创建 `stand_up_node.cpp`

新建 `src/lgw_bt_planner/src/stand_up_node.cpp`:

```cpp
// stand_up_node.cpp
//
// Implements StandUpNode: publish api_id=1004 once in onStart, then
// poll lf/sportmodestate.body_height >= 0.30 for 5 consecutive
// onRunning ticks. StopMove (1003) on halt. See spec §5.

#include "lgw_bt_planner/stand_up_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

namespace lgw_bt_planner
{

StandUpNode::StandUpNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "StandUpNode: RosNodeParams::nh weak_ptr expired");
  }

  // RELIABLE publisher on /api/sport/request (spec §6).
  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  // BEST_EFFORT subscriber on lf/sportmodestate (spec §6).
  subscriber_ = node->create_subscription<unitree_go::msg::SportModeState>(
    sport_api::kSportModeStateTopic,
    rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *msg;
    });
}

BT::PortsList StandUpNode::providedPorts()
{
  return {};
}

BT::NodeStatus StandUpNode::onStart()
{
  // One-shot publish of the StandUp command (spec §5.1).
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStandUp;
  publisher_->publish(req);

  // Reset state cache + stability counter + deadline.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = unitree_go::msg::SportModeState{};
  }
  stable_count_ = 0;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration<double>(sport_api::kStandUpTimeoutSec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StandUpNode::onRunning()
{
  // Copy latest state under lock; release before threshold logic.
  unitree_go::msg::SportModeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
  }

  // Threshold: body_height >= 0.30 for 5 consecutive frames (spec §4.2).
  if (state.body_height >= sport_api::kStandHeight) {
    if (++stable_count_ >= sport_api::kStableFrames) {
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_count_ = 0;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void StandUpNode::onHalted()
{
  // Spec §5.3: StandUp/StandDown halt publishes StopMove (1003).
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStopMove;
  publisher_->publish(req);
}

}  // namespace lgw_bt_planner
```

### Step 3: 在 `bt_plugin_registration.cpp` 注册 StandUp

修改 `src/lgw_bt_planner/src/bt_plugin_registration.cpp`:

在 line 15 (`#include "lgw_bt_planner/wait_for_user_input_bt_node.hpp"` 之后)加:

```cpp
#include "lgw_bt_planner/stand_up_node.hpp"
```

在 line 55-56 (`ComputeLookatPose` 注册行之后)加:

```cpp
  factory.registerNodeType<lgw_bt_planner::StandUpNode>(
    "StandUp", BT::RosNodeParams(nh_shared));
```

### Step 4: build 验证

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -15
"
```

Expected: 无 error,`Finished <<< lgw_bt_planner [Xs]` 出现。

### Step 5: 验证 plugin symbol 导出

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  nm -D install/lgw_bt_planner/lib/liblgw_bt_planner_nodes.so | grep -E 'StandUpNode|BT_Register' | head -10
"
```

Expected: 看到 `StandUpNode::onStart`、`StandUpNode::onRunning`、`StandUpNode::onHalted`、`StandUpNode::providedPorts` 的 mangled 符号,以及 `BT_RegisterRosNodeFromPlugin`。

### Step 6: 验证 launch 仍能加载(无回归)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  timeout 5 ros2 launch lgw_bt_planner lgw_bt_planner.launch.py 2>&1 | grep -E 'Loaded (ROS Plugin|BehaviorTree)' | head -10
"
```

Expected: 看到现有 7 个 BT XML(`go2_posture_demo.xml` 还没建,不在列)+ `liblgw_bt_planner_nodes.so` plugin。StandUp **尚未被任何 XML 引用**,但已注册到 factory,无 fatal error。

### Step 7: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/include/lgw_bt_planner/stand_up_node.hpp \
        src/lgw_bt_planner/src/stand_up_node.cpp \
        src/lgw_bt_planner/src/bt_plugin_registration.cpp
git commit -m "$(cat <<'EOF'
lgw: add StandUpNode BT leaf for Go2 sport API

StatefulActionNode that publishes api_id=1004 to /api/sport/request
once in onStart, then polls lf/sportmodestate.body_height >=
0.30 m for 5 consecutive onRunning ticks. Returns SUCCESS on
threshold, FAILURE after 5 s timeout, StopMove (1003) on halt.

RELIABLE publisher + BEST_EFFORT subscriber (spec §6). Mutex
guards latest_state_ against DDS-thread vs BT-thread races.

Serves as the template for BalanceStandNode (Task 3) and
StandDownNode (Task 4). No base class shared (spec §3).
EOF
)"
```

---

## Task 3: BalanceStandNode — hpp + cpp + 注册 + build 验证

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/balance_stand_node.hpp`
- Create: `src/lgw_bt_planner/src/balance_stand_node.cpp`
- Modify: `src/lgw_bt_planner/src/bt_plugin_registration.cpp`(加 1 行 include + 1 行 registerNodeType)

**目的:** 第二个节点,跟 StandUp 同模式但完成判据改成 `state.mode == 1` 单帧即 SUCCESS,timeout 缩短到 2s(spec §4.2)。onHalted 不发 StopMove(避免 mode 回 0,spec §5.3)。

**为什么独立成 task:** onHalted 行为不同是 BalanceStand 跟 StandUp 的关键差异,reviewer 应该单独 review 这个"不发 StopMove"的决策。

### Step 1: 创建 `balance_stand_node.hpp`

新建 `src/lgw_bt_planner/include/lgw_bt_planner/balance_stand_node.hpp`:

```cpp
// balance_stand_node.hpp
//
// BT::StatefulActionNode that publishes a BalanceStand command
// (api_id=1002) to /api/sport/request and waits until
// lf/sportmodestate.mode == 1. Single-frame check (no stability
// count). Fails after kBalanceStandTimeoutSec (2 s). See spec §4.2.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace lgw_bt_planner
{

class BalanceStandNode : public BT::StatefulActionNode
{
public:
  BalanceStandNode(const std::string& name, const BT::NodeConfig& config,
                   const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  std::shared_ptr<rclcpp::Subscription<unitree_go::msg::SportModeState>>
      subscriber_;
  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace lgw_bt_planner
```

### Step 2: 创建 `balance_stand_node.cpp`

新建 `src/lgw_bt_planner/src/balance_stand_node.cpp`:

```cpp
// balance_stand_node.cpp
//
// Implements BalanceStandNode: publish api_id=1002 once in onStart,
// then poll lf/sportmodestate.mode == 1. No stability count (mode
// transition is a one-shot event). No StopMove on halt (would
// knock mode back to 0 — spec §5.3).

#include "lgw_bt_planner/balance_stand_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

namespace lgw_bt_planner
{

BalanceStandNode::BalanceStandNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "BalanceStandNode: RosNodeParams::nh weak_ptr expired");
  }

  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  subscriber_ = node->create_subscription<unitree_go::msg::SportModeState>(
    sport_api::kSportModeStateTopic,
    rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *msg;
    });
}

BT::PortsList BalanceStandNode::providedPorts()
{
  return {};
}

BT::NodeStatus BalanceStandNode::onStart()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdBalanceStand;
  publisher_->publish(req);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = unitree_go::msg::SportModeState{};
  }
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration<double>(
                sport_api::kBalanceStandTimeoutSec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BalanceStandNode::onRunning()
{
  unitree_go::msg::SportModeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
  }

  // Single-frame check: mode transition is event-like, not gradual.
  if (state.mode == sport_api::kBalanceMode) {
    return BT::NodeStatus::SUCCESS;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void BalanceStandNode::onHalted()
{
  // Spec §5.3: BalanceStand onHalted is a no-op. Sending StopMove here
  // would knock state.mode back to 0, which is the opposite of what
  // we just achieved. Leave the dog in whatever state it reached.
}

}  // namespace lgw_bt_planner
```

### Step 3: 在 `bt_plugin_registration.cpp` 注册 BalanceStand

修改 `src/lgw_bt_planner/src/bt_plugin_registration.cpp`:

在 Step 2 加的 `#include "lgw_bt_planner/stand_up_node.hpp"` 之后追加:

```cpp
#include "lgw_bt_planner/balance_stand_node.hpp"
```

在 Step 2 加的 `StandUpNode` 注册行之后追加:

```cpp
  factory.registerNodeType<lgw_bt_planner::BalanceStandNode>(
    "BalanceStand", BT::RosNodeParams(nh_shared));
```

### Step 4: build 验证

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -15
"
```

Expected: 无 error。

### Step 5: 验证 plugin symbol 导出

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  nm -D install/lgw_bt_planner/lib/liblgw_bt_planner_nodes.so | grep -E 'BalanceStandNode' | head -10
"
```

Expected: 看到 `BalanceStandNode::onStart`、`onRunning`、`onHalted`、`providedPorts` 符号。

### Step 6: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/include/lgw_bt_planner/balance_stand_node.hpp \
        src/lgw_bt_planner/src/balance_stand_node.cpp \
        src/lgw_bt_planner/src/bt_plugin_registration.cpp
git commit -m "$(cat <<'EOF'
lgw: add BalanceStandNode BT leaf for Go2 sport API

StatefulActionNode that publishes api_id=1002 once in onStart,
then polls lf/sportmodestate.mode == 1 (single-frame check, no
stability counter — mode transition is event-like). Returns
SUCCESS on mode==1, FAILURE after 2 s timeout.

onHalted is intentionally a no-op (spec §5.3): emitting StopMove
(1003) here would knock state.mode back to 0, undoing the very
transition we just observed. StandUp/StandDown halt does emit
StopMove; BalanceStand does not.
EOF
)"
```

---

## Task 4: StandDownNode — hpp + cpp + 注册 + build 验证

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/stand_down_node.hpp`
- Create: `src/lgw_bt_planner/src/stand_down_node.cpp`
- Modify: `src/lgw_bt_planner/src/bt_plugin_registration.cpp`(加 1 行 include + 1 行 registerNodeType)

**目的:** 第三个节点,跟 StandUp 同模式但阈值改成 `body_height < 0.20`,发 api_id=1005,onHalted 发 StopMove。完成"起身"序列的对称反操作。

**为什么独立成 task:** 跟 StandUp 在阈值方向(`>=` vs `<`)和 API ID 上有差异,reviewer 应该单独 review 这个对称性。

### Step 1: 创建 `stand_down_node.hpp`

新建 `src/lgw_bt_planner/include/lgw_bt_planner/stand_down_node.hpp`:

```cpp
// stand_down_node.hpp
//
// BT::StatefulActionNode that publishes a StandDown command
// (api_id=1005) to /api/sport/request and waits until
// lf/sportmodestate.body_height < kLyingHeight (0.20 m) for
// kStableFrames (5) consecutive ticks. Fails after
// kStandDownTimeoutSec (5 s). See spec §4.2.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace lgw_bt_planner
{

class StandDownNode : public BT::StatefulActionNode
{
public:
  StandDownNode(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  std::shared_ptr<rclcpp::Subscription<unitree_go::msg::SportModeState>>
      subscriber_;
  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  int stable_count_{0};
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace lgw_bt_planner
```

### Step 2: 创建 `stand_down_node.cpp`

新建 `src/lgw_bt_planner/src/stand_down_node.cpp`:

```cpp
// stand_down_node.cpp
//
// Implements StandDownNode: publish api_id=1005 once in onStart, then
// poll lf/sportmodestate.body_height < 0.20 for 5 consecutive
// onRunning ticks. StopMove (1003) on halt. Mirror of StandUpNode —
// same pattern, opposite threshold direction. See spec §4.2 + §5.1.

#include "lgw_bt_planner/stand_down_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>

namespace lgw_bt_planner
{

StandDownNode::StandDownNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "StandDownNode: RosNodeParams::nh weak_ptr expired");
  }

  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  subscriber_ = node->create_subscription<unitree_go::msg::SportModeState>(
    sport_api::kSportModeStateTopic,
    rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *msg;
    });
}

BT::PortsList StandDownNode::providedPorts()
{
  return {};
}

BT::NodeStatus StandDownNode::onStart()
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStandDown;
  publisher_->publish(req);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = unitree_go::msg::SportModeState{};
  }
  stable_count_ = 0;
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration<double>(
                sport_api::kStandDownTimeoutSec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StandDownNode::onRunning()
{
  unitree_go::msg::SportModeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
  }

  // Threshold: body_height < 0.20 for 5 consecutive frames (spec §4.2).
  if (state.body_height < sport_api::kLyingHeight) {
    if (++stable_count_ >= sport_api::kStableFrames) {
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_count_ = 0;
  }

  if (std::chrono::steady_clock::now() > deadline_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void StandDownNode::onHalted()
{
  // Spec §5.3: StandDown halt publishes StopMove (1003), symmetric
  // with StandUp.
  unitree_api::msg::Request req;
  req.header.identity.api_id = sport_api::kApiIdStopMove;
  publisher_->publish(req);
}

}  // namespace lgw_bt_planner
```

### Step 3: 在 `bt_plugin_registration.cpp` 注册 StandDown

修改 `src/lgw_bt_planner/src/bt_plugin_registration.cpp`:

在 Step 3 加的 `#include "lgw_bt_planner/balance_stand_node.hpp"` 之后追加:

```cpp
#include "lgw_bt_planner/stand_down_node.hpp"
```

在 Step 3 加的 `BalanceStandNode` 注册行之后追加:

```cpp
  factory.registerNodeType<lgw_bt_planner::StandDownNode>(
    "StandDown", BT::RosNodeParams(nh_shared));
```

### Step 4: build 验证

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -15
"
```

Expected: 无 error。

### Step 5: 验证所有 3 个节点符号都已导出

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  nm -D install/lgw_bt_planner/lib/liblgw_bt_planner_nodes.so | \
    grep -E 'StandUpNode|BalanceStandNode|StandDownNode' | \
    grep -E 'onStart|onRunning|onHalted|providedPorts' | wc -l
"
```

Expected: 输出 ≥ 12(3 节点 × 4 符号 = 12;有时 compiler 还会内联,可能更多)。

### Step 6: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/include/lgw_bt_planner/stand_down_node.hpp \
        src/lgw_bt_planner/src/stand_down_node.cpp \
        src/lgw_bt_planner/src/bt_plugin_registration.cpp
git commit -m "$(cat <<'EOF'
lgw: add StandDownNode BT leaf for Go2 sport API

StatefulActionNode that publishes api_id=1005 to /api/sport/request
once in onStart, then polls lf/sportmodestate.body_height <
0.20 m for 5 consecutive onRunning ticks. Returns SUCCESS on
threshold, FAILURE after 5 s timeout, StopMove (1003) on halt.

Mirror of StandUpNode: same template, opposite threshold direction
(>= vs <), opposite API ID (1004 vs 1005). Both nodes share
the same publisher + subscriber + mutex scaffolding via duplication,
not inheritance (spec §3).
EOF
)"
```

---

## Task 5: go2_posture_demo.xml + go2_posture_demo.launch.py + launch 验证

**Files:**
- Create: `src/lgw_bt_planner/bt_trees/go2_posture_demo.xml`
- Create: `src/lgw_bt_planner/launch/go2_posture_demo.launch.py`

**目的:** 把 Tasks 2-4 注册的 3 个节点拼成 `Go2PostureDemo` BT 树,加 launch wrapper。**整个 spec §4.3 / §4.4 在这一个 task 里集中交付**,让 reviewer 一眼能 review 完整 demo。

**为什么放在最后:** 前 4 个 task 是独立组件;Task 5 是组合 + 启动入口,是"做完"的检验点。1 个 task 交付所有交付物,因为它们是 demo tree 不可分割的两半(树 + 启动方式)。

### Step 1: 创建 `go2_posture_demo.xml`

新建 `src/lgw_bt_planner/bt_trees/go2_posture_demo.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  go2_posture_demo.xml — StandUp → BalanceStand → 2s settle → StandDown.

  Tree ID: Go2PostureDemo (PascalCase, matches PickAndPlaceDemo /
    HeadToArmLookat convention in this package).

  Nodes used:
    - StandUp, BalanceStand, StandDown: registered in
      bt_plugin_registration.cpp (Tasks 2-4).
    - Sequence, RetryUntilSuccessful, Sleep: BT.CPP v4 built-in, no
      registration needed.

  Topology:
    Sequence "posture_round_trip" (all SUCCESS to return SUCCESS)
      RetryUntilSuccessful num_attempts=3
        StandUp           (publish 1004, wait body_h >= 0.30 x5)
      BalanceStand        (publish 1002, wait mode == 1)
      Sleep msec=2000     (BT.CPP built-in: let posture settle)
      RetryUntilSuccessful num_attempts=3
        StandDown         (publish 1005, wait body_h < 0.20 x5)

  Retry rationale: StandUp/StandDown may fail transiently if the
  machine is mid-motion when the BT tick fires. 3 retries gives a
  cheap robustness bump without hiding real faults.

  Trigger:
    ros2 action send_goal /lgw_bt_execution \
      btcpp_ros2_interfaces/action/ExecuteTree \
      "{target_tree: Go2PostureDemo}"
-->
<root BTCPP_format="4" main_tree_to_execute="Go2PostureDemo">
  <BehaviorTree ID="Go2PostureDemo">
    <Sequence name="posture_round_trip">
      <RetryUntilSuccessful num_attempts="3">
        <StandUp name="stand_up"/>
      </RetryUntilSuccessful>
      <BalanceStand name="balance_stand"/>
      <Sleep name="settle" msec="2000"/>
      <RetryUntilSuccessful num_attempts="3">
        <StandDown name="stand_down"/>
      </RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>
</root>
```

### Step 2: 创建 `go2_posture_demo.launch.py`

新建 `src/lgw_bt_planner/launch/go2_posture_demo.launch.py`:

```python
"""Launch for the Go2 posture BT demo.

Brings up bt_runner with the Go2PostureDemo tree discoverable. Same
shape as lgw_bt_planner.launch.py; we keep it as a separate file so
the demo tree can be run without the pick-and-place moveit chain.

Trigger the tree after launch with:
  ros2 action send_goal /lgw_bt_execution \
    btcpp_ros2_interfaces/action/ExecuteTree \
    "{target_tree: Go2PostureDemo}"
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lgw_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            parameters=[{
                'plugins': ['lgw_bt_planner/bt_plugins'],
                'behavior_trees': ['lgw_bt_planner/bt_trees'],
                'action_name': 'lgw_bt_execution',
                'tick_frequency': 10,
            }],
        ),
    ])
```

### Step 3: build 验证 install 部署

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -10 && \
  ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/go2_posture_demo.xml && \
  ls install/lgw_bt_planner/share/lgw_bt_planner/launch/go2_posture_demo.launch.py
"
```

Expected: build 无 error;两个文件都在 install 路径下。

### Step 4: launch 加载验证(无真机)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  timeout 5 ros2 launch lgw_bt_planner go2_posture_demo.launch.py 2>&1 | \
    grep -E 'Loaded (ROS Plugin|BehaviorTree)' | sort -u
"
```

Expected: 
- `Loaded ROS Plugin: liblgw_bt_planner_nodes.so` (含 3 个新节点 + 已有节点)
- `Loaded BehaviorTree: go2_posture_demo.xml` (新)
- 其他 7 个原有 BT XML 也加载成功(无回归)

### Step 5: BT XML 内容级 sanity check(节点 ID 严格匹配)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  cat install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/go2_posture_demo.xml | \
    grep -E 'StandUp|BalanceStand|StandDown|Sleep|Sequence|RetryUntilSuccessful'
"
```

Expected: 6 行匹配(`<StandUp .../>` / `<BalanceStand .../>` / `<StandDown .../>` / `<Sleep .../>` / `<Sequence .../>` / `<RetryUntilSuccessful .../>`),无 typo。

### Step 6: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/bt_trees/go2_posture_demo.xml \
        src/lgw_bt_planner/launch/go2_posture_demo.launch.py
git commit -m "$(cat <<'EOF'
lgw: add Go2PostureDemo BT tree + demo launch

go2_posture_demo.xml: StandUp → BalanceStand → Sleep 2s → StandDown,
wrapped in RetryUntilSuccessful (3 attempts each on stand motions).
Tree ID = Go2PostureDemo (PascalCase, matches existing demo IDs).
Uses only BT.CPP v4 built-in Sequence / RetryUntilSuccessful / Sleep
plus the 3 nodes registered in Tasks 2-4.

go2_posture_demo.launch.py: launches bt_runner with the demo tree
discoverable, no moveit bringup chain (per spec §4.4: keep launch
narrow so users running the posture demo don't read the pick-and-place
launch stack).

Trigger:
  ros2 action send_goal /lgw_bt_execution \
    btcpp_ros2_interfaces/action/ExecuteTree \
    "{target_tree: Go2PostureDemo}"
EOF
)"
```

---

## Task 6: 容器端总验收清单

**目的:** 跨 5 个 task 的总检查,无代码改动,只跑验证命令。

### Step 1: 全量 build 一次,确认无回归

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -15
"
```

Expected: `Finished <<< lgw_bt_planner [Xs]`,无 error,无 warning(关于新文件)。

### Step 2: 8 个 BT XML 全部仍能加载

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  timeout 5 ros2 launch lgw_bt_planner go2_posture_demo.launch.py 2>&1 | \
    grep -E 'Loaded BehaviorTree' | wc -l
"
```

Expected: 输出 ≥ 8(7 个原有 + 1 个新增 `go2_posture_demo.xml`)。

### Step 3: 3 个节点注册符号全导出

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  nm -D install/lgw_bt_planner/lib/liblgw_bt_planner_nodes.so | \
    grep -E 'StandUpNode|BalanceStandNode|StandDownNode' | grep -E 'providedPorts' | wc -l
"
```

Expected: 输出 = 3(每个节点 1 个 providedPorts 符号)。

### Step 4: 现有 9 个 BT leaf 节点无回归(已有节点仍注册)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  timeout 5 ros2 launch lgw_bt_planner go2_posture_demo.launch.py 2>&1 | \
    grep -E 'MoveArmToPose|MoveArmToNamed|SetPoseStamped|IsHead|IsArm|WaitForUserInput|ComputeLookatPose|StandUp|BalanceStand|StandDown' | \
    grep -iE 'register|loading|createNode' | wc -l
"
```

Expected: 输出 ≥ 10(7 个原有 + 3 个新增),证明注册块无 syntax error 把后续节点的注册也卡住。

### Step 5: 验收清单(全部勾选才视为完成)

- [ ] Step 1: `colcon build --packages-select lgw_bt_planner` 无 error
- [ ] Step 2: ≥ 8 个 BT XML 加载成功(含新增 `go2_posture_demo.xml`)
- [ ] Step 3: 3 个新节点的 `providedPorts` 符号均导出
- [ ] Step 4: 已有 7 个 BT 节点仍能注册到 factory(无回归)
- [ ] work.log 追加一段:「2026-10-06 Go2 起身/趴下 BT 节点实施完成,3 节点 + demo 树 + launch 已交付容器端验收。真机验证(spec §8.2)留给用户。」

---

## 真机验证(留给用户,无代码改动)

按 spec §8.2 跑完整真机流程。前置(spec §8.2 / §9 #8):
- Go2 driver + 机械臂硬件 + 双 realsense + YOLOE workflow 全部 up
- 周围 1.0m 范围无障碍物,操作员不离开控制台
- E-stop 按钮随手可触

### Step 1: 启动 launch

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  ros2 launch lgw_bt_planner go2_posture_demo.launch.py
"
```

Expected: log 显示 plugin + 8 个 BT XML 加载成功。

### Step 2: send_goal 触发 BT

另开终端:
```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  source install/setup.bash && \
  ros2 action send_goal /lgw_bt_execution \
    btcpp_ros2_interfaces/action/ExecuteTree \
    '{target_tree: Go2PostureDemo}' --feedback
"
```

Expected: 狗起身 → 站 2 秒 → 趴下;`send_goal` 终端在最后收到 SUCCESS feedback。

### Step 3: 验证清单

- [ ] `body_height` ≥0.30 持续 5 帧,StandUp SUCCESS
- [ ] `state.mode == 1`,BalanceStand SUCCESS
- [ ] 2 秒后 StandDown 触发,`body_height` <0.20 持续 5 帧,SUCCESS
- [ ] 整个 Sequence SUCCESS,树自然终止
- [ ] `/api/sport/request` 上能看到 1004 → 1002 → 1005 三条指令(按序)
- [ ] FAILURE 路径:拔 eth10,5s 后 StandUp 应 FAILURE,不是挂起
- [ ] onHalted 路径:中途 Ctrl-C,`/api/sport/request` 上能看到 1003(StopMove);BalanceStand 不会发 1003

---

## 执行选项

plan 完成,已保存到 `docs/superpowers/plans/2026-10-06-go2-posture-nodes.md`。两种执行选项:

1. **Subagent-Driven (推荐)** - 每个 Task 派一个新 subagent,task 间做两阶段 review,迭代快
2. **Inline Execution** - 当前会话连续执行,用 executing-plans,批量执行 + 检查点
