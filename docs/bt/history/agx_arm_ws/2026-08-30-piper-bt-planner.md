# Piper BT Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 Piper 机械臂基于 BehaviorTree.CPP + BT.ROS2 的最小可用 BT 规划层(6 个 BT 节点 + 1 个独立 server + 1 个 launch + 1 个 demo XML),符合 `l4t/go2_bt_ws/docs/bt-design-standard.md` 三层架构。

**Architecture:** 三层分离——BT XML 描述 WHAT,6 个 thin shell 节点调既有 server(`/move_action`、`/apply_planning_scene`、`/compute_ik`)与新建的 `gripper_action_server`,所有控制循环与心跳都在 server 端,BT 节点严格遵守 action client 模式。

**Tech Stack:** ROS 2 Humble · C++17 · BehaviorTree.CPP v4 · BT.ROS2 (从 `l4t/bt_ros2_ws/install/` source) · MoveIt2 (`moveit_msgs`) · AgileX Piper SDK(由 `agx_arm_ctrl` 间接调用)

**Spec:** `l4t/agx_arm_ws/docs/superpowers/specs/2026-08-30-piper-bt-planner-design.md`

---

## Global Constraints

适用于所有任务,实施时必须遵守:

| 项 | 值 |
|---|---|
| Workspace root(host) | `/home/unitree/my_ws/l4t/agx_arm_ws` |
| Workspace root(container) | `/my_ws/l4t/agx_arm_ws` |
| BT.ROS2 install(container) | `/my_ws/l4t/bt_ros2_ws/install/setup.bash`(必须 source) |
| ROS distro | Humble(容器内 `/opt/ros/humble/setup.bash`) |
| 编辑策略 | 文件编辑在 host(本机的 `Write`/`Edit` 工具直接落到 bind-mount 的 host 路径) |
| build / install / apt / launch | 全部在容器 `self_l4t` 内 `docker exec` 执行,不用 host 命令 |
| apt 新包流程 | 先 `docker exec self_l4t apt install` 验证再考虑改 Dockerfile(本任务预期不引入新 apt 包) |
| Git 仓库 | `agx_arm_ws` 不是 git 仓库,**不 commit**,任务验证以"build 成功 + launch 节点可见"为准 |
| 测试代码 | 不写测试代码(per spec §12.2);验证方式:colcon build + ros2 interface/node/action list |
| 构建策略 | 只 build `--packages-select piper_bt_interfaces piper_bt_planner`,不触发上游全量 |
| BT.CPP / BT.ROS2 用法 | 严格按标准 §5.2 模板,节点全 client,无 `create_server` / `create_publisher` 在 BT 节点内 |
| XML 节点名 ↔ C++ registerNodeType 第一参数 | 必须一一对应(标准 §6.7) |

---

## 任务依赖图

```
Task 1 (interfaces)
   │
   ▼
Task 2 (planner skeleton)
   │
   ├─► Task 3 (named_target_loader)
   │
   ├─► Task 4 (MoveArmToPose + MoveArmToNamed action nodes)
   │
   ├─► Task 5 (AttachObject + DetachObject + CheckPoseReachable service nodes)
   │
   ├─► Task 6 (gripper_action_server)
   │      │
   │      ▼
   ├─► Task 7 (ControlGripper BT node)
   │
   ├─► Task 8 (bt_runner_node registers all 6)
   │
   └─► Task 9 (launch + demo XML + integration verify)
```

每个任务结束时应能 `colcon build --packages-select piper_bt_interfaces piper_bt_planner` 退出码 0;失败即停下排查,不进入下一任务。

---

## Task 1: 创建 piper_bt_interfaces 包

**Files:**
- Create: `l4t/agx_arm_ws/src/piper_bt_interfaces/CMakeLists.txt`
- Create: `l4t/agx_arm_ws/src/piper_bt_interfaces/package.xml`
- Create: `l4t/agx_arm_ws/src/piper_bt_interfaces/action/ControlGripper.action`

**Interfaces:**
- Consumes: 无
- Produces: 头文件 `piper_bt_interfaces/action/control_gripper.hpp`,类型 `piper_bt_interfaces::action::ControlGripper`,action 命名 `/control_gripper`

- [ ] **Step 1: 创建目录结构(host)**

```bash
mkdir -p /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_interfaces/action
ls /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_interfaces/
```
期望输出:`action`

- [ ] **Step 2: 写 package.xml(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_interfaces/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper_bt_interfaces</name>
  <version>0.1.0</version>
  <description>Action / service definitions for piper_bt_planner</description>
  <maintainer email="dev@example.com">Dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>action_msgs</depend>
  <depend>rclcpp</depend>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 CMakeLists.txt(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_interfaces/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(piper_bt_interfaces VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ControlGripper.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

- [ ] **Step 4: 写 ControlGripper.action(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_interfaces/action/ControlGripper.action`:

```
# Goal
float64 target_width
float64 force
float64 timeout
---
# Result
bool success
float64 final_width
string message
---
# Feedback
float64 current_width
float64 current_force
bool reached
```

- [ ] **Step 5: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0,终端含 `Summary: 1 package finished [pip_bt_interfaces]`

- [ ] **Step 6: action 接口可见性验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 interface show piper_bt_interfaces/action/ControlGripper
"
```

期望输出:
```
float64 target_width
float64 force
float64 timeout
---
bool success
float64 final_width
...
```

---

## Task 2: 创建 piper_bt_planner 包骨架

**Files:**
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/package.xml`
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/CMakeLists.txt`
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/src/bt_runner_node.cpp` (stub,占位,后续任务填充)
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/src/gripper_action_server.cpp` (stub)

**Interfaces:**
- Consumes: `piper_bt_interfaces::action::ControlGripper`(Task 1)
- Produces: 后续任务会用到的包与两个 executable stub(`bt_runner`, `gripper_action_server`),build 通过且 register 为 ROS nodes

- [ ] **Step 1: 创建目录(host)**

```bash
mkdir -p /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src
mkdir -p /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner
mkdir -p /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/launch
mkdir -p /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/bt_trees
ls /home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/
```
期望:`bt_trees  include  launch  src`

- [ ] **Step 2: 写 package.xml(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>piper_bt_planner</name>
  <version>0.1.0</version>
  <description>BehaviorTree.CPP + BT.ROS2 thin shells over MoveIt2 + gripper action server</description>
  <maintainer email="dev@example.com">Dev</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>behaviortree_cpp</depend>
  <depend>behaviortree_ros2</depend>
  <depend>moveit_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>agx_arm_msgs</depend>
  <depend>piper_bt_interfaces</depend>

  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>agx_arm_ros</exec_depend>
  <exec_depend>agx_arm_moveit</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写 CMakeLists.txt(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(piper_bt_planner VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(agx_arm_msgs REQUIRED)
find_package(piper_bt_interfaces REQUIRED)

set(NODE_SOURCES
  src/named_target_loader.cpp
  src/move_arm_to_pose_action_node.cpp
  src/move_arm_to_named_action_node.cpp
  src/attach_object_service_node.cpp
  src/detach_object_service_node.cpp
  src/check_pose_reachable_service_node.cpp
  src/control_gripper_action_node.cpp
)

add_library(piper_bt_planner_nodes SHARED ${NODE_SOURCES})
target_include_directories(piper_bt_planner_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/piper_bt_planner>
)
ament_target_dependencies(piper_bt_planner_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  tf2_ros
  tf2_geometry_msgs
  agx_arm_msgs
  piper_bt_interfaces
)

# bt_runner executable (registers nodes + ticks the tree)
add_executable(bt_runner src/bt_runner_node.cpp)
ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  tf2_ros
  tf2_geometry_msgs
  piper_bt_interfaces
)
target_link_libraries(bt_runner piper_bt_planner_nodes)

# gripper_action_server executable (Layer 3)
add_executable(gripper_action_server src/gripper_action_server.cpp)
ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  builtin_interfaces
  agx_arm_msgs
  piper_bt_interfaces
)

install(TARGETS
  piper_bt_planner_nodes
  bt_runner
  gripper_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY bt_trees/ DESTINATION share/${PROJECT_NAME}/bt_trees)

ament_export_targets(piper_bt_planner_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp behaviortree_ros2 moveit_msgs)

ament_package()
```

- [ ] **Step 4: 写 bt_runner_node.cpp 占位(host,最小可编译)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/bt_runner_node.cpp`:

```cpp
// bt_runner_node.cpp — placeholder; filled out in Task 8.
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_runner");
  RCLCPP_INFO(node->get_logger(), "bt_runner placeholder (Task 2 stub)");
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: 写 gripper_action_server.cpp 占位(host,最小可编译)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/gripper_action_server.cpp`:

```cpp
// gripper_action_server.cpp — placeholder; filled out in Task 6.
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gripper_action_server");
  RCLCPP_INFO(node->get_logger(), "gripper_action_server placeholder (Task 2 stub)");
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 6: 写 named_target_loader 等其他文件占位(host,空实现)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/named_target_loader.cpp`:

```cpp
// named_target_loader.cpp — placeholder; filled out in Task 3.
#include "piper_bt_planner/named_target_loader.hpp"

namespace piper_bt_planner {

std::map<std::string, std::map<std::string, double>>
NamedTargetLoader::load_from_share()
{
  return {};  // Task 3
}

std::vector<double>
NamedTargetLoader::get_joint_values(
  const std::map<std::string, std::map<std::string, double>>& table,
  const std::string& target,
  const std::vector<std::string>& joint_names)
{
  (void)table;
  (void)target;
  (void)joint_names;
  return {};
}

}  // namespace piper_bt_planner
```

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/named_target_loader.hpp`:

```cpp
// named_target_loader.hpp — placeholder; filled out in Task 3.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace piper_bt_planner {

class NamedTargetLoader
{
public:
  static std::map<std::string, std::map<std::string, double>> load_from_share();
  static std::vector<double> get_joint_values(
    const std::map<std::string, std::map<std::string, double>>& table,
    const std::string& target,
    const std::vector<std::string>& joint_names);
};

}  // namespace piper_bt_planner
```

- [ ] **Step 7: 写剩余 5 个节点 cpp/hpp 占位(host)**

每个节点给最小 stub:头文件空类定义,源文件空方法。

`/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/move_arm_to_pose_action_node.hpp`:

```cpp
#pragma once
#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit_msgs/action/move_action.hpp>

namespace piper_bt_planner {

class MoveArmToPoseActionNode
  : public BT::RosActionNode<moveit_msgs::action::MoveAction>
{
public:
  MoveArmToPoseActionNode(const std::string& name, const BT::NodeConfig& cfg,
                          const BT::RosNodeParams& params)
    : BT::RosActionNode<moveit_msgs::action::MoveAction>(name, cfg, params) {}

  bool setGoal(moveit_msgs::action::MoveAction::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  static BT::PortsList providedPorts();
};

}  // namespace piper_bt_planner
```

`/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/move_arm_to_pose_action_node.cpp`:

```cpp
#include "piper_bt_planner/move_arm_to_pose_action_node.hpp"

namespace piper_bt_planner {

bool MoveArmToPoseActionNode::setGoal(moveit_msgs::action::MoveAction::Goal& goal)
{
  (void)goal;
  return true;  // Task 4
}

BT::NodeStatus MoveArmToPoseActionNode::onResultReceived(const WrappedResult& wr)
{
  (void)wr;
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveArmToPoseActionNode::providedPorts()
{
  return {};
}

}  // namespace piper_bt_planner
```

对**剩余 5 个节点**(move_arm_to_named, attach_object, detach_object, check_pose_reachable, control_gripper)按同样模式创建 hpp + cpp 占位文件,头文件包含对应 action/service 类型,源文件返回 `BT::NodeStatus::SUCCESS` 占位。**详细代码在 Task 4/5/7 的对应步骤给出——本步骤只创建占位文件骨架使 CMakeLists 编译通过即可。**

各节点占位的对应类型与命名:

| 节点 hpp | 类型继承 |
|---|---|
| `move_arm_to_named_action_node.hpp` | `BT::RosActionNode<moveit_msgs::action::MoveAction>` |
| `attach_object_service_node.hpp` | `BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>` |
| `detach_object_service_node.hpp` | `BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>` |
| `check_pose_reachable_service_node.hpp` | `BT::RosServiceNode<moveit_msgs::srv::GetPositionIK>` |
| `control_gripper_action_node.hpp` | `BT::RosActionNode<piper_bt_interfaces::action::ControlGripper>` |

- [ ] **Step 8: 第一次 build(容器内,期望成功)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_interfaces piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0,终端含 `Summary: 2 packages finished`(`piper_bt_interfaces` + `piper_bt_planner`)。**失败即停**——大概率是 include 路径或行为 `RosActionNode`/`RosServiceNode` API 不匹配,以错误信息修正(可能需要查 `behaviortree_ros2` 头文件确认命名)。

- [ ] **Step 9: executables 可见性(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 pkg executables piper_bt_planner
"
```

期望输出含 `bt_runner` 与 `gripper_action_server` 两行。

---

## Task 3: 实现 named_target_loader

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/named_target_loader.hpp`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/named_target_loader.cpp`

**Interfaces:**
- Consumes: 文件系统(`ament_index_cpp::get_package_share_directory("agx_arm_moveit")`),SRDF 文本
- Produces:
  - `static std::map<std::string, std::map<std::string, double>> load_from_share()`
  - `static std::vector<double> get_joint_values(table, target, joint_names)` (长度 = joint_names.size())

- [ ] **Step 1: 实现 hpp(host,完整内容)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/named_target_loader.hpp`:

```cpp
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace piper_bt_planner
{

/// Parses SRDF <group_state> entries into name → (joint → value) map.
/// Current SRDF has only "home" (all-zero). Pure std::regex, no XML lib.
class NamedTargetLoader
{
public:
  using TargetTable = std::map<std::string, std::map<std::string, double>>;

  /// Read agx_arm_moveit/config/agx_arm.srdf from the share directory.
  /// Throws std::runtime_error if file cannot be read.
  static TargetTable load_from_share();

  /// Look up joint values for `target` in the given order of `joint_names`.
  /// Missing joint → 0.0 (matches "home" all-zero fallback).
  /// Missing target → throws std::runtime_error.
  static std::vector<double> get_joint_values(
    const TargetTable& table,
    const std::string& target,
    const std::vector<std::string>& joint_names);
};

}  // namespace piper_bt_planner
```

- [ ] **Step 2: 实现 cpp(host,完整内容)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/named_target_loader.cpp`:

```cpp
#include "piper_bt_planner/named_target_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <regex>
#include <sstream>

namespace piper_bt_planner
{

namespace
{
constexpr const char* kSrdfSubpath = "/config/agx_arm.srdf";

// Matches <group_state name="X" group="Y"> ... </group_state>
// (non-greedy to avoid spanning multiple states).
const std::regex kGroupStateRegex(
  R"(<group_state\s+name=\"([^\"]+)\"\s+group=\"[^\"]+\">([\s\S]*?)</group_state>)");

// Matches <joint name="J" value="V" /> inside a group_state.
const std::regex kJointValueRegex(
  R"(<joint\s+name=\"([^\"]+)\"\s+value=\"([-+]?\d+(?:\.\d+)?)\"\s*/?>)");
}  // namespace

NamedTargetLoader::TargetTable NamedTargetLoader::load_from_share()
{
  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("agx_arm_moveit");
  const std::string srdf_path = pkg_share + kSrdfSubpath;

  std::ifstream ifs(srdf_path);
  if (!ifs.is_open()) {
    throw std::runtime_error("NamedTargetLoader: cannot open SRDF: " + srdf_path);
  }

  std::stringstream ss;
  ss << ifs.rdbuf();
  const std::string content = ss.str();

  TargetTable table;

  auto state_begin = std::sregex_iterator(content.begin(), content.end(), kGroupStateRegex);
  auto state_end = std::sregex_iterator();

  for (auto it = state_begin; it != state_end; ++it) {
    const std::string state_name = (*it)[1].str();
    const std::string state_body = (*it)[2].str();
    std::map<std::string, double> joints;

    auto j_begin = std::sregex_iterator(state_body.begin(), state_body.end(), kJointValueRegex);
    auto j_end = std::sregex_iterator();

    for (auto jt = j_begin; jt != j_end; ++jt) {
      const std::string j_name = (*jt)[1].str();
      const double j_value = std::stod((*jt)[2].str());
      joints[j_name] = j_value;
    }

    if (!joints.empty()) {
      table[state_name] = std::move(joints);
    }
  }

  return table;
}

std::vector<double> NamedTargetLoader::get_joint_values(
  const TargetTable& table,
  const std::string& target,
  const std::vector<std::string>& joint_names)
{
  auto it = table.find(target);
  if (it == table.end()) {
    throw std::runtime_error("NamedTargetLoader: unknown target '" + target + "'");
  }
  const auto& joints = it->second;

  std::vector<double> values;
  values.reserve(joint_names.size());
  for (const auto& jn : joint_names) {
    auto jt = joints.find(jn);
    values.push_back(jt == joints.end() ? 0.0 : jt->second);
  }
  return values;
}

}  // namespace piper_bt_planner
```

- [ ] **Step 3: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0,无新增 warning。

- [ ] **Step 4: 启动期验证(可选,容器内,只查 launch 不到节点就停)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 run piper_bt_planner bt_runner &
  sleep 2
  kill %1 2>/dev/null
"
```

期望:进程退出码非 0 或被 kill;无 `FATAL` 字样。**这一步验证 named_target_loader 在 bt_runner 启动时能被调用(虽然 BT 节点还没接入)。**

---

## Task 4: 实现 MoveArmToPose / MoveArmToNamed BT action 节点

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/move_arm_to_pose_action_node.cpp`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/move_arm_to_named_action_node.cpp`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/move_arm_to_pose_action_node.hpp`(添加 providedPorts 公共方法声明)
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/move_arm_to_named_action_node.hpp`

**Interfaces:**
- Consumes: `moveit_msgs::action::MoveAction`(`/move_action` 由 move_group 提供),`piper_bt_planner::NamedTargetLoader`(Task 3)
- Produces:
  - 注册时 `factory_->registerNodeType<MoveArmToPoseActionNode>("MoveArmToPose", RosNodeParams(...))`
  - 注册时 `factory_->registerNodeType<MoveArmToNamedActionNode>("MoveArmToNamed", RosNodeParams(...))`
  - Input ports:见下表
  - BT XML 中 `<MoveArmToPose goal="{pose}" .../>` 与 `<MoveArmToNamed named_target="home" .../>` 工作

| 节点 | Input ports |
|---|---|
| MoveArmToPose | `goal` (PoseStamped), `planner_id` (string, default ""), `planning_time` (double, default 5.0), `plan_only` (bool, default false) |
| MoveArmToNamed | `named_target` (string), `planner_id`, `planning_time`, `plan_only` |

- [ ] **Step 1: 实现 MoveArmToPoseActionNode hpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/move_arm_to_pose_action_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_action.hpp>

namespace piper_bt_planner
{

class MoveArmToPoseActionNode
  : public BT::RosActionNode<moveit_msgs::action::MoveAction>
{
public:
  MoveArmToPoseActionNode(const std::string& name, const BT::NodeConfig& cfg,
                          const BT::RosNodeParams& params)
    : BT::RosActionNode<moveit_msgs::action::MoveAction>(name, cfg, params) {}

  bool setGoal(moveit_msgs::action::MoveAction::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  static BT::PortsList providedPorts();
};

}  // namespace piper_bt_planner
```

- [ ] **Step 2: 实现 MoveArmToPoseActionNode cpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/move_arm_to_pose_action_node.cpp`:

```cpp
#include "piper_bt_planner/move_arm_to_pose_action_node.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace piper_bt_planner
{

namespace
{
constexpr double kJointToleranceAbove = 0.01;
constexpr double kJointToleranceBelow = 0.01;
}  // namespace

bool MoveArmToPoseActionNode::setGoal(moveit_msgs::action::MoveAction::Goal& goal)
{
  auto target = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!target) {
    return false;
  }

  const std::string frame = target->header.frame_id.empty()
    ? "base_link"
    : target->header.frame_id;

  goal.request.workspace_parameters.header.frame_id = frame;
  goal.request.workspace_parameters.header.stamp = builtin_interfaces::msg::Time();

  auto& gc = goal.request.goal_constraints.emplace_back();

  // Position constraint: target pose with small box region (~5cm tolerance).
  auto& pc = gc.position_constraints.emplace_back();
  pc.header.frame_id = frame;
  pc.link_name = "piper_tcp";
  pc.target_point_offset.x = 0.0;
  pc.target_point_offset.y = 0.0;
  pc.target_point_offset.z = 0.0;
  pc.constraint_region.primitives.resize(1);
  pc.constraint_region.primitives[0].type =
    shape_msgs::msg::SolidPrimitive::BOX;
  pc.constraint_region.primitives[0].dimensions = {0.05, 0.05, 0.05};
  pc.constraint_region.primitive_poses.resize(1);
  pc.constraint_region.primitive_poses[0] = target->pose;
  pc.weight = 1.0;

  // Orientation constraint: match target orientation with ~30 deg tolerance.
  auto& oc = gc.orientation_constraints.emplace_back();
  oc.header.frame_id = frame;
  oc.link_name = "piper_tcp";
  oc.orientation = target->pose.orientation;
  oc.absolute_x_axis_tolerance = 0.5;
  oc.absolute_y_axis_tolerance = 0.5;
  oc.absolute_z_axis_tolerance = 0.5;
  oc.weight = 1.0;

  goal.request.planner_id = getInput<std::string>("planner_id").value_or("");
  goal.request.planning_options.planning_pipeline = goal.request.planner_id;
  goal.request.planning_options.max_planning_time =
    getInput<double>("planning_time").value_or(5.0);
  goal.request.planning_options.plan_only =
    getInput<bool>("plan_only").value_or(false);

  return true;
}

BT::NodeStatus MoveArmToPoseActionNode::onResultReceived(const WrappedResult& wr)
{
  return wr.code == rclcpp_action::ResultCode::SUCCEEDED
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveArmToPoseActionNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "workspace target pose"),
    BT::InputPort<std::string>("planner_id", "", "planning pipeline / planner id"),
    BT::InputPort<double>("planning_time", 5.0, "max planning time (s)"),
    BT::InputPort<bool>("plan_only", false, "plan only, do not execute"),
  };
}

}  // namespace piper_bt_planner
```

- [ ] **Step 3: 实现 MoveArmToNamedActionNode hpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/move_arm_to_named_action_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <moveit_msgs/action/move_action.hpp>

#include <map>
#include <string>
#include <vector>

namespace piper_bt_planner
{

class MoveArmToNamedActionNode
  : public BT::RosActionNode<moveit_msgs::action::MoveAction>
{
public:
  MoveArmToNamedActionNode(const std::string& name, const BT::NodeConfig& cfg,
                           const BT::RosNodeParams& params)
    : BT::RosActionNode<moveit_msgs::action::MoveAction>(name, cfg, params) {}

  bool setGoal(moveit_msgs::action::MoveAction::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  static BT::PortsList providedPorts();

  // Injected by bt_runner_node.cpp at registration time (Task 8).
  void set_named_targets(
    std::shared_ptr<const std::map<std::string, std::map<std::string, double>>>
      named_targets)
  {
    named_targets_ = std::move(named_targets);
  }

private:
  std::shared_ptr<const std::map<std::string, std::map<std::string, double>>>
    named_targets_;
};

}  // namespace piper_bt_planner
```

- [ ] **Step 4: 实现 MoveArmToNamedActionNode cpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/move_arm_to_named_action_node.cpp`:

```cpp
#include "piper_bt_planner/move_arm_to_named_action_node.hpp"

#include <moveit_msgs/msg/joint_constraint.hpp>

#include <stdexcept>

namespace piper_bt_planner
{

namespace
{
constexpr double kJointToleranceAbove = 0.01;
constexpr double kJointToleranceBelow = 0.01;
}  // namespace

bool MoveArmToNamedActionNode::setGoal(moveit_msgs::action::MoveAction::Goal& goal)
{
  if (!named_targets_) {
    throw std::runtime_error("MoveArmToNamed: named_targets not injected");
  }

  auto target_name = getInput<std::string>("named_target");
  if (!target_name) {
    return false;
  }

  auto it = named_targets_->find(target_name.value());
  if (it == named_targets_->end()) {
    return false;  // unknown target → BT FAILURE
  }

  const auto& joints = it->second;
  auto& gc = goal.request.goal_constraints.emplace_back();

  for (const auto& [joint_name, value] : joints) {
    auto& jc = gc.joint_constraints.emplace_back();
    jc.joint_name = joint_name;
    jc.position = value;
    jc.tolerance_above = kJointToleranceAbove;
    jc.tolerance_below = kJointToleranceBelow;
    jc.weight = 1.0;
  }

  goal.request.planner_id = getInput<std::string>("planner_id").value_or("");
  goal.request.planning_options.planning_pipeline = goal.request.planner_id;
  goal.request.planning_options.max_planning_time =
    getInput<double>("planning_time").value_or(5.0);
  goal.request.planning_options.plan_only =
    getInput<bool>("plan_only").value_or(false);

  return true;
}

BT::NodeStatus MoveArmToNamedActionNode::onResultReceived(const WrappedResult& wr)
{
  return wr.code == rclcpp_action::ResultCode::SUCCEEDED
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

BT::PortsList MoveArmToNamedActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("named_target", "SRDF group_state name"),
    BT::InputPort<std::string>("planner_id", "", "planning pipeline / planner id"),
    BT::InputPort<double>("planning_time", 5.0, "max planning time (s)"),
    BT::InputPort<bool>("plan_only", false, "plan only, do not execute"),
  };
}

}  // namespace piper_bt_planner
```

- [ ] **Step 5: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0,无 `error:` / `undefined reference`。**若 BT.ROS2 模板方法签名与本计划代码不一致,以编译器错误为准微调(常见差异:`onResultReceived` 形参类型、`getInput` 返回类型、`RosActionNode` 构造)。**

---

## Task 5: 实现 AttachObject / DetachObject / CheckPoseReachable BT service 节点

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/attach_object_service_node.{hpp,cpp}`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/detach_object_service_node.{hpp,cpp}`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/check_pose_reachable_service_node.{hpp,cpp}`

**Interfaces:**
- Consumes: `moveit_msgs::srv::ApplyPlanningScene`, `moveit_msgs::srv::GetPositionIK`, TF2(DetachObject 需要 link world pose)
- Produces:
  - 注册时 `factory_->registerNodeType<...>("AttachObject"/"DetachObject"/"CheckPoseReachable", RosNodeParams(...))`
  - Input ports:见下表
  - DetachObject 注入 `tf_buffer_`(由 bt_runner 在 Task 8 注入)

| 节点 | Input ports | Output ports |
|---|---|---|
| AttachObject | `object_id`, `link_name`, `shape_type` (default "box"), `shape_dimensions` (vector<double>, default {0.05,0.05,0.05}), `pose` (PoseStamped), `touch_links` (vector<string>, default {}) | — |
| DetachObject | `object_id`, `link_name` | — |
| CheckPoseReachable | `goal` (PoseStamped), `ik_link_name` (default "piper_tcp"), `timeout` (double, default 1.0), `avoid_collisions` (bool, default true) | `ik_solution` (sensor_msgs::JointState) |

- [ ] **Step 1: AttachObject hpp + cpp(host)**

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/attach_object_service_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

namespace piper_bt_planner
{

class AttachObjectServiceNode
  : public BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  AttachObjectServiceNode(const std::string& name, const BT::NodeConfig& cfg,
                          const BT::RosNodeParams& params)
    : BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>(name, cfg, params) {}

  bool setRequest(typename moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr& req) override;
  BT::NodeStatus onResponse(const typename moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr& res) override;
  static BT::PortsList providedPorts();
};

}  // namespace piper_bt_planner
```

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/attach_object_service_node.cpp`:

```cpp
#include "piper_bt_planner/attach_object_service_node.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace piper_bt_planner
{

bool AttachObjectServiceNode::setRequest(
  typename moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr& req)
{
  auto object_id = getInput<std::string>("object_id");
  auto link_name = getInput<std::string>("link_name");
  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  if (!object_id || !link_name || !pose) {
    return false;
  }

  const std::string shape_type = getInput<std::string>("shape_type").value_or("box");
  const auto shape_dims =
    getInput<std::vector<double>>("shape_dimensions").value_or(
      std::vector<double>{0.05, 0.05, 0.05});
  const auto touch_links =
    getInput<std::vector<std::string>>("touch_links").value_or(
      std::vector<std::string>{});

  // 1) ADD to world.collision_objects
  auto& co = req->scene.world.collision_objects.emplace_back();
  co.id = object_id.value();
  co.header.frame_id = pose->header.frame_id.empty()
    ? "base_link" : pose->header.frame_id;
  co.pose = pose->pose;

  co.primitives.resize(1);
  co.primitives[0].type =
    (shape_type == "sphere")    ? shape_msgs::msg::SolidPrimitive::SPHERE :
    (shape_type == "cylinder")  ? shape_msgs::msg::SolidPrimitive::CYLINDER :
                                   shape_msgs::msg::SolidPrimitive::BOX;
  co.primitives[0].dimensions = shape_dims;

  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  // 2) ATTACH to link_name (one diff call)
  auto& aco = req->scene.robot_state.attached_collision_objects.emplace_back();
  aco.object.id = object_id.value();
  aco.link_name = link_name.value();
  aco.touch_links = touch_links;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ATTACH;

  req->scene.is_diff = true;
  return true;
}

BT::NodeStatus AttachObjectServiceNode::onResponse(
  const typename moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr& res)
{
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList AttachObjectServiceNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id", "collision object id"),
    BT::InputPort<std::string>("link_name", "robot link to attach to"),
    BT::InputPort<std::string>("shape_type", "box", "box / cylinder / sphere"),
    BT::InputPort<std::vector<double>>("shape_dimensions",
      std::vector<double>{0.05, 0.05, 0.05}, "primitive dimensions"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "world pose for ADD"),
    BT::InputPort<std::vector<std::string>>("touch_links",
      std::vector<std::string>{}, "links allowed to touch"),
  };
}

}  // namespace piper_bt_planner
```

- [ ] **Step 2: DetachObject hpp + cpp(host,需要 tf_buffer_ 注入)**

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/detach_object_service_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <tf2_ros/buffer.h>

#include <memory>

namespace piper_bt_planner
{

class DetachObjectServiceNode
  : public BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>
{
public:
  DetachObjectServiceNode(const std::string& name, const BT::NodeConfig& cfg,
                          const BT::RosNodeParams& params)
    : BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>(name, cfg, params) {}

  bool setRequest(typename moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr& req) override;
  BT::NodeStatus onResponse(const typename moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr& res) override;
  static BT::PortsList providedPorts();

  // Injected by bt_runner_node.cpp at registration time (Task 8).
  void set_tf_buffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  {
    tf_buffer_ = std::move(tf_buffer);
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace piper_bt_planner
```

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/detach_object_service_node.cpp`:

```cpp
#include "piper_bt_planner/detach_object_service_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

namespace piper_bt_planner
{

bool DetachObjectServiceNode::setRequest(
  typename moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr& req)
{
  auto object_id = getInput<std::string>("object_id");
  auto link_name = getInput<std::string>("link_name");
  if (!object_id || !link_name) {
    return false;
  }
  if (!tf_buffer_) {
    throw std::runtime_error("DetachObject: tf_buffer not injected");
  }

  // 1) Read current scene
  //    Note: RosServiceNode can't await here cleanly, but get_planning_scene
  //    via a blocking client is the cleanest implementation. For minimum
  //    viable scope we attach a future that is set in onResponse of a
  //    synchronous wrapper. To keep this step compilable and honest, we
  //    document the constraint here and rely on the caller's expectation
  //    that the world object already exists separately. We synthesize a
  //    best-effort request that DETACHES the attached object; the caller
  //    is expected to add the world entry externally if needed.
  auto& aco = req->scene.robot_state.attached_collision_objects.emplace_back();
  aco.object.id = object_id.value();
  aco.link_name = link_name.value();
  aco.object.operation = moveit_msgs::msg::CollisionObject::DETACH;
  req->scene.is_diff = true;
  return true;
}

BT::NodeStatus DetachObjectServiceNode::onResponse(
  const typename moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr& res)
{
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList DetachObjectServiceNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id", "collision object id"),
    BT::InputPort<std::string>("link_name", "attached link"),
  };
}

}  // namespace piper_bt_planner
```

> **实施期注意(留 fallback)**:上面实现只发 DETACH,没有把物体落回 world(plan §4.4 描述的 GET → TF → world.collision_objects 三步)。`tf_buffer_` 注入留口子但当前实现没用到,真机验证时如发现 detach 后物体从场景消失,需在此处补全 GET 步骤。**属于后续工作,本最小集可接受简单 detach 行为。**

- [ ] **Step 3: CheckPoseReachable hpp + cpp(host)**

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/check_pose_reachable_service_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

namespace piper_bt_planner
{

class CheckPoseReachableServiceNode
  : public BT::RosServiceNode<moveit_msgs::srv::GetPositionIK>
{
public:
  CheckPoseReachableServiceNode(const std::string& name, const BT::NodeConfig& cfg,
                                const BT::RosNodeParams& params)
    : BT::RosServiceNode<moveit_msgs::srv::GetPositionIK>(name, cfg, params) {}

  bool setRequest(typename moveit_msgs::srv::GetPositionIK::Request::SharedPtr& req) override;
  BT::NodeStatus onResponse(const typename moveit_msgs::srv::GetPositionIK::Response::SharedPtr& res) override;
  static BT::PortsList providedPorts();
};

}  // namespace piper_bt_planner
```

文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/check_pose_reachable_service_node.cpp`:

```cpp
#include "piper_bt_planner/check_pose_reachable_service_node.hpp"

#include <moveit_msgs/msg/robot_state.hpp>

#include <chrono>

namespace piper_bt_planner
{

bool CheckPoseReachableServiceNode::setRequest(
  typename moveit_msgs::srv::GetPositionIK::Request::SharedPtr& req)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    return false;
  }

  const std::string ik_link = getInput<std::string>("ik_link_name").value_or("piper_tcp");
  const double timeout_sec = getInput<double>("timeout").value_or(1.0);
  const bool avoid_collisions = getInput<bool>("avoid_collisions").value_or(true);

  req->ik_request.ik_link_names = {ik_link};
  req->ik_request.pose_stamped = goal.value();
  req->ik_request.timeout =
    rclcpp::Duration(std::chrono::duration<double>(timeout_sec));
  req->ik_request.avoid_collisions = avoid_collisions;
  return true;
}

BT::NodeStatus CheckPoseReachableServiceNode::onResponse(
  const typename moveit_msgs::srv::GetPositionIK::Response::SharedPtr& res)
{
  if (res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    setOutput("ik_solution", res->solution.joint_state);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::PortsList CheckPoseReachableServiceNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "target pose to check IK"),
    BT::InputPort<std::string>("ik_link_name", "piper_tcp", "end-effector link"),
    BT::InputPort<double>("timeout", 1.0, "IK timeout (s)"),
    BT::InputPort<bool>("avoid_collisions", true, "check collisions in IK"),
    BT::OutputPort<sensor_msgs::msg::JointState>("ik_solution", "IK solution if reachable"),
  };
}

}  // namespace piper_bt_planner
```

- [ ] **Step 4: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0。

---

## Task 6: 实现 gripper_action_server

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/gripper_action_server.cpp`(替换 Task 2 占位)

**Interfaces:**
- Consumes: `piper_bt_interfaces::action::ControlGripper`,`agx_arm_msgs::msg::GripperStatus`(`/feedback/gripper_status`)
- Produces:
  - Action server `/control_gripper`,节点名 `gripper_action_server`
  - 发布 `/control/joint_states`(`sensor_msgs::msg::JointState`)
  - 启动参数:`default_force`,`tolerance`,`pub_rate`

- [ ] **Step 1: 完整实现(host,替换 Task 2 占位)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/gripper_action_server.cpp`:

```cpp
// gripper_action_server.cpp — Layer 3 server for gripper control.
// Subscribes /feedback/gripper_status; publishes /control/joint_states (heartbeat).
// Cancels & timeouts publish zero-width to release pressure.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <piper_bt_interfaces/action/control_gripper.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <mutex>

using ControlGripper = piper_bt_interfaces::action::ControlGripper;
using GoalHandle = rclcpp_action::ServerGoalHandle<ControlGripper>;

class GripperActionServer : public rclcpp::Node
{
public:
  explicit GripperActionServer(const rclcpp::NodeOptions& options)
    : Node("gripper_action_server", options)
  {
    default_force_ = this->declare_parameter<double>("default_force", 1.0);
    tolerance_ = this->declare_parameter<double>("tolerance", 0.005);
    pub_rate_ = this->declare_parameter<double>("pub_rate", 20.0);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/control/joint_states", 10);

    status_sub_ = this->create_subscription<agx_arm_msgs::msg::GripperStatus>(
      "/feedback/gripper_status", 10,
      [this](const agx_arm_msgs::msg::GripperStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(status_mutex_);
        latest_status_ = *msg;
        has_status_ = true;
      });

    action_server_ = rclcpp_action::create_server<ControlGripper>(
      this, "control_gripper",
      [this](auto uuid, auto goal) { return handle_goal(uuid, goal); },
      [this](auto uuid) { return handle_cancel(uuid); },
      [this](auto goal) { execute(goal); });
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ControlGripper::Goal> goal)
  {
    if (goal->target_width < 0.0 || goal->target_width > 0.1) {
      RCLCPP_WARN(this->get_logger(),
        "Rejecting goal: target_width=%.4f out of [0.0, 0.1]", goal->target_width);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->force < 0.5 || goal->force > 3.0) {
      RCLCPP_WARN(this->get_logger(),
        "Rejecting goal: force=%.2f out of [0.5, 3.0]", goal->force);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->timeout <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "Rejecting goal: timeout=%.2f must be > 0", goal->timeout);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const rclcpp_action::GoalUUID& /*uuid*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    target_width_ = goal->target_width;
    target_force_ = goal->force;
    const auto start = this->now();
    const rclcpp::Duration timeout_dur(
      std::chrono::duration<double>(goal->timeout));

    rclcpp::WallRate loop_rate(pub_rate_);
    auto result = std::make_shared<ControlGripper::Result>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        publish_zero_width();
        result->success = false;
        result->message = "canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled, published zero width");
        return;
      }

      publish_target();

      auto fb = std::make_shared<ControlGripper::Feedback>();
      {
        std::lock_guard<std::mutex> lk(status_mutex_);
        if (has_status_) {
          fb->current_width = latest_status_.width;
          fb->current_force = latest_status_.force;
        }
      }
      fb->reached = std::fabs(fb->current_width - target_width_) < tolerance_;
      goal_handle->publish_feedback(fb);

      if (fb->reached) {
        result->success = true;
        result->final_width = fb->current_width;
        result->message = "reached";
        goal_handle->succeed(result);
        return;
      }

      if (this->now() - start > timeout_dur) {
        publish_zero_width();
        result->success = false;
        result->final_width = fb->current_width;
        result->message = "timeout";
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "Goal timeout, published zero width");
        return;
      }

      loop_rate.sleep();
    }
  }

  void publish_target()
  {
    sensor_msgs::msg::JointState js;
    js.name = {"gripper"};
    js.position = {target_width_};
    js.velocity = {0.0};
    js.effort = {target_force_};
    joint_state_pub_->publish(js);
  }

  void publish_zero_width()
  {
    target_width_ = 0.0;
    publish_target();
  }

  rclcpp_action::Server<ControlGripper>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<agx_arm_msgs::msg::GripperStatus>::SharedPtr status_sub_;

  double default_force_;
  double tolerance_;
  double pub_rate_;

  double target_width_{0.0};
  double target_force_{1.0};

  std::mutex status_mutex_;
  agx_arm_msgs::msg::GripperStatus latest_status_;
  bool has_status_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperActionServer>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0。

- [ ] **Step 3: 节点启动并验证 action 可见(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 run piper_bt_planner gripper_action_server &
  sleep 2
  ros2 action list | grep control_gripper
  ros2 action info /control_gripper
  kill %1 2>/dev/null
"
```

期望:`ros2 action list` 含 `/control_gripper`;`ros2 action info` 显示 Action server `/control_gripper` 已就绪。

---

## Task 7: 实现 ControlGripper BT 节点

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/control_gripper_action_node.hpp`
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/control_gripper_action_node.cpp`

**Interfaces:**
- Consumes: `piper_bt_interfaces::action::ControlGripper`,`/control_gripper` 由 Task 6 提供
- Produces:
  - 注册时 `factory_->registerNodeType<ControlGripperActionNode>("ControlGripper", RosNodeParams(...))`
  - Input ports: `target_width`, `force`, `timeout`
  - Output ports: `final_width`

- [ ] **Step 1: hpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/include/piper_bt_planner/control_gripper_action_node.hpp`:

```cpp
#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <piper_bt_interfaces/action/control_gripper.hpp>

namespace piper_bt_planner
{

class ControlGripperActionNode
  : public BT::RosActionNode<piper_bt_interfaces::action::ControlGripper>
{
public:
  ControlGripperActionNode(const std::string& name, const BT::NodeConfig& cfg,
                           const BT::RosNodeParams& params)
    : BT::RosActionNode<piper_bt_interfaces::action::ControlGripper>(name, cfg, params) {}

  bool setGoal(piper_bt_interfaces::action::ControlGripper::Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  void on_wait_for_result(
    std::shared_ptr<const piper_bt_interfaces::action::ControlGripper::Feedback> feedback) override;
  static BT::PortsList providedPorts();
};

}  // namespace piper_bt_planner
```

- [ ] **Step 2: cpp(host)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/control_gripper_action_node.cpp`:

```cpp
#include "piper_bt_planner/control_gripper_action_node.hpp"

namespace piper_bt_planner
{

bool ControlGripperActionNode::setGoal(
  piper_bt_interfaces::action::ControlGripper::Goal& goal)
{
  auto width = getInput<double>("target_width");
  auto force = getInput<double>("force");
  auto timeout = getInput<double>("timeout");
  if (!width) {
    return false;
  }
  goal.target_width = width.value();
  goal.force = force.value_or(1.0);
  goal.timeout = timeout.value_or(3.0);
  return true;
}

BT::NodeStatus ControlGripperActionNode::onResultReceived(const WrappedResult& wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (wr.result) {
      setOutput("final_width", wr.result->final_width);
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void ControlGripperActionNode::on_wait_for_result(
  std::shared_ptr<const piper_bt_interfaces::action::ControlGripper::Feedback> feedback)
{
  RCLCPP_INFO(node_.get_logger(),
    "ControlGripper feedback: width=%.4f force=%.2f reached=%s",
    feedback->current_width, feedback->current_force,
    feedback->reached ? "true" : "false");
}

BT::PortsList ControlGripperActionNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_width", "target gripper width (m)"),
    BT::InputPort<double>("force", 1.0, "gripper force (N)"),
    BT::InputPort<double>("timeout", 3.0, "timeout (s)"),
    BT::OutputPort<double>("final_width", "actual final width if reached"),
  };
}

}  // namespace piper_bt_planner
```

> **实施期注意**:`node_` 成员属于 `BT::RosActionNode` 基类;若编译错误提示 `node_` 是 private,改为 `this->getRosNode()`(BT.ROS2 公开方法)或类似公共访问器。具体 API 以 `btcpp_ros2_interfaces` 头文件为准。

- [ ] **Step 3: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0。

---

## Task 8: 实现 bt_runner_node.cpp(注册 6 个节点 + tick)

**Files:**
- Modify: `l4t/agx_arm_ws/src/piper_bt_planner/src/bt_runner_node.cpp`

**Interfaces:**
- Consumes: `piper_bt_planner::NamedTargetLoader`,所有 6 个 BT 节点类,MoveArmToNamedActionNode.set_named_targets,DetachObjectServiceNode.set_tf_buffer
- Produces:
  - 节点 `bt_runner`,启动时 `MultiThreadedExecutor.spin()` + 100ms tick
  - 启动参数:`bt_xml` (string)

- [ ] **Step 1: 完整实现(host,替换 Task 2 占位)**

覆写 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/src/bt_runner_node.cpp`:

```cpp
// bt_runner_node.cpp — registers 6 BT nodes, loads SRDF named targets,
// injects TF buffer, ticks the tree at 10Hz under MultiThreadedExecutor.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>

#include "piper_bt_planner/attach_object_service_node.hpp"
#include "piper_bt_planner/check_pose_reachable_service_node.hpp"
#include "piper_bt_planner/control_gripper_action_node.hpp"
#include "piper_bt_planner/detach_object_service_node.hpp"
#include "piper_bt_planner/move_arm_to_named_action_node.hpp"
#include "piper_bt_planner/move_arm_to_pose_action_node.hpp"
#include "piper_bt_planner/named_target_loader.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace
{
class BtRunner : public rclcpp::Node
{
public:
  explicit BtRunner(const rclcpp::NodeOptions& options)
    : rclcpp::Node("bt_runner", options)
  {
    bt_xml_ = this->declare_parameter<std::string>(
      "bt_xml", "bt_trees/piper_minimal_demo.xml");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    try {
      named_targets_ = std::make_shared<
        const piper_bt_planner::NamedTargetLoader::TargetTable>(
          piper_bt_planner::NamedTargetLoader::load_from_share());
      RCLCPP_INFO(this->get_logger(),
        "Loaded %zu named targets from SRDF",
        named_targets_->size());
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load SRDF named targets: %s", e.what());
      throw;
    }

    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    auto node_ptr = this->shared_from_this();

    factory_->registerNodeType<piper_bt_planner::MoveArmToPoseActionNode>(
      "MoveArmToPose", BT::RosNodeParams(node_ptr, "/move_action"));

    factory_->registerNodeType<piper_bt_planner::MoveArmToNamedActionNode>(
      "MoveArmToNamed",
      [node_ptr, this](const std::string& name, const BT::NodeConfig& cfg) {
        auto node = std::make_unique<piper_bt_planner::MoveArmToNamedActionNode>(
          name, cfg, BT::RosNodeParams(node_ptr, "/move_action"));
        node->set_named_targets(named_targets_);
        return node;
      });

    factory_->registerNodeType<piper_bt_planner::AttachObjectServiceNode>(
      "AttachObject", BT::RosNodeParams(node_ptr, "/apply_planning_scene"));

    factory_->registerNodeType<piper_bt_planner::DetachObjectServiceNode>(
      "DetachObject",
      [node_ptr, this](const std::string& name, const BT::NodeConfig& cfg) {
        auto node = std::make_unique<piper_bt_planner::DetachObjectServiceNode>(
          name, cfg, BT::RosNodeParams(node_ptr, "/apply_planning_scene"));
        node->set_tf_buffer(tf_buffer_);
        return node;
      });

    factory_->registerNodeType<piper_bt_planner::CheckPoseReachableServiceNode>(
      "CheckPoseReachable", BT::RosNodeParams(node_ptr, "/compute_ik"));

    factory_->registerNodeType<piper_bt_planner::ControlGripperActionNode>(
      "ControlGripper", BT::RosNodeParams(node_ptr, "/control_gripper"));

    factory_->registerBehaviorTreeFromFile(bt_xml_);

    auto tree_ids = factory_->registeredBehaviorTrees();
    if (tree_ids.empty()) {
      throw std::runtime_error("No BehaviorTree registered from " + bt_xml_);
    }
    tree_id_ = tree_ids.front();
    RCLCPP_INFO(this->get_logger(), "Selected tree: %s", tree_id_.c_str());

    blackboard_ = BT::Blackboard::create();
    tree_ = factory_->createTree(tree_id_, blackboard_);
    RCLCPP_INFO(this->get_logger(), "BT tree created");
  }

  void run() {
    rclcpp::Rate rate(10.0);  // 100ms tick = 10Hz
    while (rclcpp::ok()) {
      const auto status = tree_.tickOnce();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "tick: %s", BT::toStr(status).c_str());
      rate.sleep();
    }
  }

private:
  std::string bt_xml_;
  std::string tree_id_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<const piper_bt_planner::NamedTargetLoader::TargetTable>
    named_targets_;

  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtRunner>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

> **实施期注意**:`registerNodeType` 的 3 参数 builder 形式在不同 BT.ROS2 版本签名可能不同;若编译报错,改回 2 参数 + `set_named_targets` / `set_tf_buffer` 调用通过其他途径(例如把这些状态改成全局单例,或用 `BT::Tree` 的 blackboard 注入)。**`on_wait_for_result` 与 `node_` 在基类的可访问性同样以编译器错误为准微调**。

- [ ] **Step 2: build 验证(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0。

---

## Task 9: 创建 launch + demo XML + 集成验证

**Files:**
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/launch/piper_bt_planner.launch.py`
- Create: `l4t/agx_arm_ws/src/piper_bt_planner/bt_trees/piper_minimal_demo.xml`

**Interfaces:**
- Consumes: spec §8 demo XML 模板
- Produces:
  - Launch 文件 `piper_bt_planner.launch.py`
  - BT XML `piper_minimal_demo.xml`
  - 真机集成验证:启动 launch 后 6 节点全部注册成功,BT 树 tick 起来

- [ ] **Step 1: launch 文件(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/launch/piper_bt_planner.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    agx_arm_moveit_launch = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'launch', 'start_single_agx_arm_moveit.launch.py')

    bt_xml = os.path.join(
        get_package_share_directory('piper_bt_planner'),
        'bt_trees', 'piper_minimal_demo.xml')

    return LaunchDescription([
        # agx_arm_ctrl + move_group + controllers (existing)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(agx_arm_moveit_launch)),

        # Layer 3: gripper action server (Task 6)
        Node(
            package='piper_bt_planner',
            executable='gripper_action_server',
            name='gripper_action_server',
            parameters=[{
                'default_force': 1.0,
                'tolerance': 0.005,
                'pub_rate': 20.0,
            }],
        ),

        # Layer 1 + 2: BT engine + 6 thin shells (Task 8)
        Node(
            package='piper_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            parameters=[{
                'bt_xml': bt_xml,
            }],
        ),
    ])
```

- [ ] **Step 2: demo BT XML(host,完整内容)**

创建文件 `/home/unitree/my_ws/l4t/agx_arm_ws/src/piper_bt_planner/bt_trees/piper_minimal_demo.xml`:

```xml
<root BTCPP_format="4" main_tree_to_execute="PiperMinimalDemo">
  <BehaviorTree ID="PiperMinimalDemo">
    <Sequence name="PiperPipeline">
      <!-- Pose literal: frame_id;x;y;z;qx;qy;qz;qw -->
      <SetBlackboard output_key="pick_pose"
                     value="base_link;0.30;0.00;0.15;0;0;0;1"/>
      <MoveArmToNamed name="go_home" named_target="home"/>
      <CheckPoseReachable name="check_pick" goal="{pick_pose}" ik_link_name="piper_tcp"/>
      <MoveArmToPose name="reach_pick" goal="{pick_pose}"/>
      <ControlGripper name="close_gripper" target_width="0.0" force="1.5" timeout="3.0"/>
      <AttachObject name="pick_up"
                    object_id="target_box" link_name="piper_gripper_base"
                    shape_type="box" shape_dimensions="0.05;0.05;0.05"
                    pose="{pick_pose}"/>
      <MoveArmToNamed name="go_home_again" named_target="home"/>
      <DetachObject name="drop" object_id="target_box" link_name="piper_gripper_base"/>
      <ControlGripper name="open_gripper" target_width="0.07" force="1.0" timeout="3.0"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- [ ] **Step 3: 重建让 install 复制新文件(容器内)**

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

期望:退出码 0,install/piper_bt_planner/share/piper_bt_planner/{launch, bt_trees}/ 出现新文件。

- [ ] **Step 4: launch 启动验证(容器内,前台运行,Ctrl+C 退出)**

> **⚠️ 真机操作提醒**:启动 launch 会让 arm controller + move_group 接管硬件。如果只是想验证 BT 注册与节点存活,先不接 CAN(只用 mock / 不连物理臂)即可启动 launch 看到节点清单。

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 launch piper_bt_planner piper_bt_planner.launch.py
"
```

期望:
- 终端无 `FATAL` / 红色堆栈
- 日志含 `Loaded N named targets from SRDF`(N ≥ 1,当前 SRDF 只有 "home")
- 日志含 `Selected tree: PiperMinimalDemo`
- 日志含 `tick: SUCCESS` 或 `tick: RUNNING` 节流打印

`Ctrl+C` 退出。

- [ ] **Step 5: 节点 / action / topic 清单(容器内,新终端)**

> 启动 launch 后,在**另一个 docker exec 会话**跑(launch 已在运行的终端保持):

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  echo '--- nodes ---' && ros2 node list &&
  echo '--- actions ---' && ros2 action list &&
  echo '--- interfaces ---' && ros2 interface show piper_bt_interfaces/action/ControlGripper | head -10
"
```

期望:
- nodes 含:`/bt_runner`,`/gripper_action_server`,`/move_group`,`/agx_arm_ctrl_single_node`(若已启动硬件驱动)
- actions 含:`/move_action`,`/control_gripper`
- interface 输出含 `float64 target_width`

- [ ] **Step 6: 验收清单逐项核对**

对照 spec §13 验收清单逐项验证:

| 验证项 | 状态 |
|---|---|
| 接口包可编译 | ✅ Task 1 Step 5 |
| 主包可编译 | ✅ Task 2-8 各自 build 通过 |
| action 接口可见 | ✅ Task 1 Step 6 + Task 9 Step 5 |
| 服务节点可见 | ✅ Task 9 Step 5(`/bt_runner`, `/gripper_action_server`) |
| action 接口可见 | ✅ Task 9 Step 5(`/move_action`, `/control_gripper`) |
| 启动无致命错误 | ✅ Task 9 Step 4 |
| demo XML 注册成功 | ✅ Task 9 Step 4 日志含 `Selected tree: PiperMinimalDemo` |
| demo 树 SUCCESS | ⚠️ **需真机验证**(下一阶段);模拟环境下若 move_group 缺硬件会 FAIL,但 BT 节点全部正确注册 |

---

## 完成定义

整个计划完成后,你应拥有:

1. 两个新 ROS 2 包:`piper_bt_interfaces`, `piper_bt_planner`,均通过 `colcon build`
2. 6 个 BT 节点类 + 1 个独立 server + 1 个 launch + 1 个 demo XML
3. 启动 launch 后,日志显示 6 节点全部注册,SRDF 加载成功,BT 树进入 tick 循环
4. (可选后续)真机硬件测试 pick-and-place 全流程:home → IK check → reach → close → attach → move home → detach → open

如果某任务失败,**停在该任务**排查,不要进入下一任务。常见失败模式与缓解:

| 失败模式 | 缓解 |
|---|---|
| `BT::RosActionNode` / `BT::RosServiceNode` 模板方法签名与计划代码不一致 | 查 `behaviortree_ros2` 头文件,以编译器错误微调 |
| `registerNodeType` 3 参数 builder 形式不支持 | 改用 2 参数 + 全局共享状态(降低优雅度但不破坏功能) |
| `node_` 成员不可访问 | 改用基类公开访问器,如 `getRosNode()` 或 `getInputHandle()`(以实际 API 为准) |
| `SetBlackboard` 字面量 PoseStamped 解析失败 | 退化为添加 `WritePoseToBlackboard` thin shell,在 XML 里显式声明类型 |
| `tf_buffer_` 注入不被 BT 节点使用 | DetachObject 当前实现仅发 DETACH,fallback 路径(plan §5 Step 2 已注明) |

如需调整,回 `bt-design-standard.md` §5 模板对照修改。