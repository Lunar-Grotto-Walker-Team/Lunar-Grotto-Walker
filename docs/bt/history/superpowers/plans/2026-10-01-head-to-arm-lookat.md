# Head → Arm Camera Lookat BT Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `lgw_bt_planner` 中新增 `ComputeLookatPose` BT 节点(读 head 相机观测 + 2 份手眼标定 JSON → 输出 tcp_link 目标 PoseStamped),并新增 `head_to_arm_lookat.xml` BT 树实现 head 发现目标 → 算末端 lookat 位姿 → MoveIt 移动 → arm 相机确认 → 回 home 的视觉伺服回路。

**Architecture:**
- 复用 `lgw_bt_planner_nodes` SHARED 库 + 单 TU `BT_REGISTER_ROS_NODES` 注册模式(沿用 `bt_plugin_registration.cpp:20` 已有结构)
- 6 步 lookat 算法抽 3 个纯函数(`parse_calibration_json` / `build_lookat_orientation` / `compute_tcp_pose`)到 `compute_lookat_pose_node.cpp`,BT 节点方法只做参数解析 + 黑板读写
- 标定 JSON 在构造期一次性加载(避免每 tick IO);**节点零 tcp_offset 补偿** —— 用 `_tcp.json` 变体(`T_tcp_link → camera_arm_color_optical_frame`)
- 单元测试 11 TC 覆盖 3 纯函数 + 完整 6 步集成冒烟;集成测试仅真机(不动 mock)

**Tech Stack:** C++17, BT.CPP v4, BT.ROS2 v0.3.0, ROS2 Humble, `tf2` / `tf2::Matrix3x3` / `tf2::Quaternion`, `nlohmann::json` (apt: `nlohmann-json3-dev` 3.10.5,已装), GTest, ament_cmake

**Spec:** `l4t/lg_walker_ws/src/lgw_bt_planner/docs/superpowers/specs/2026-10-01-lgw-head-to-arm-lookat-design.md`

---

## Global Constraints

按 spec + 项目 CLAUDE.md,以下约束贯穿所有任务:

1. **容器内路径以 `/my_ws/...` 写**(host 路径 `/home/unitree/my_ws/l4t/lg_walker_ws` = container `/my_ws/l4t/lg_walker_ws`)。
2. **host 编辑文件,容器内 build/test/launch**:统一 `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "..."`。
3. **build 前 source `/my_ws/l4t/l4t_setup.sh`**:colcon build 命令统一前缀 `source /my_ws/l4t/l4t_setup.sh && colcon build ...`。
4. **不写主循环 / 不 `rclcpp::Rate` / 不 `while (rclcpp::ok())`**(SyncActionNode 同步决策)。
5. **本容器 `colcon test` 不可用**,单测改跑 `build/lgw_bt_planner/test_compute_lookat_pose` 直接执行(spec §9.1 / CLAUDE.md)。
6. **真机 + mock 禁忌**:本 BT 仅真机测试(spec §8.2),不写 mock driver。
7. **真机操作硬件安全优先**(过热 / 摔机 / 电池):停手汇报,不擅自继续(spec §9 #8)。
8. **commit 信息以 `lgw:` 前缀**(沿用现有 commit 历史风格)。
9. **每步 commit 必须独立**(便于 reviewer 拒收单个步骤)。
10. **节点零 tcp_offset 补偿**:`arm_calibration.json` 必须用 `_tcp.json` 变体,launch 改 `tcp_offset` 或换 gripper 必须**重新生成 `_tcp.json` 并重启 bt_runner**(spec §9 #4)。

---

## 文件结构总览

```
src/lgw_bt_planner/
├── calibration/                                     (Task 1 新增)
│   ├── head_calibration.json                        (从 ../../result/2026-09-30_12-54-33_calibration.json 复制)
│   └── arm_calibration.json                         (从 ../../result/2026-09-02_08-34-16_calibration_tcp.json 复制)
├── include/lgw_bt_planner/
│   └── compute_lookat_pose_node.hpp                 (Task 6 新增:BT 类 + 3 纯函数声明)
├── src/
│   ├── compute_lookat_pose_node.cpp                 (Task 6 新增:BT 类 + 3 纯函数实现 + 6 步算法)
│   └── bt_plugin_registration.cpp                   (Task 6 修改:加 1 行 registerNode)
├── tests/
│   └── test_compute_lookat_pose.cpp                 (Tasks 2-5 新增:gtest 11 条 case)
├── bt_trees/
│   └── head_to_arm_lookat.xml                       (Task 7 新增:BT 树 XML)
└── CMakeLists.txt                                   (Tasks 1 + 6 修改:install calibration + NODE_SOURCES)
└── package.xml                                      (Task 1 修改:加 nlohmann-json3-dev build_depend)
```

注:start script 在 workspace 级 `scripts/`,不在 package 内。Task 8 在 `scripts/start_lgw_head_to_arm_lookat.sh`。

---

## Task 1: 复制标定 JSON + CMakeLists install 规则 + package.xml 依赖

**Files:**
- Create: `src/lgw_bt_planner/calibration/head_calibration.json`(从 `l4t/result/2026-09-30_12-54-33_calibration.json` 复制,内容不变)
- Create: `src/lgw_bt_planner/calibration/arm_calibration.json`(从 `l4t/result/2026-09-02_08-34-16_calibration_tcp.json` 复制,内容不变)
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(在 install 段加 `install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)`)
- Modify: `src/lgw_bt_planner/package.xml`(加 `<build_depend>nlohmann-json3-dev</build_depend>`)

**目的:** 把 spec §4.1 选定的 2 份标定 JSON 落到 `calibration/` 目录并通过 install 部署到 `share/lgw_bt_planner/calibration/`,供节点构造期 `ament_index_cpp::get_package_share_directory` 解析。

**为什么先做这个:** Task 6 的 `ComputeLookatPoseNode` 构造期会读这两个文件,先把文件就位 + install 规则确认,后面节点构造期 sanity check 才有路径。

### Step 1: 复制 head_calibration.json

```bash
cp /home/unitree/my_ws/l4t/result/2026-09-30_12-54-33_calibration.json \
   /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_bt_planner/calibration/head_calibration.json
```

Expected: 文件创建成功,内容应含 `position: [0.30852098743377065, 0.02912782275342961, -0.009751166832372224]` 与 `orientation: [-0.5695219071708808, 0.5785480954782661, -0.408567323469893, 0.4171326415704736]`(对应 `T_base_link → camera_head_color_optical_frame`)。

### Step 2: 复制 arm_calibration.json(**必须 _tcp.json 变体**)

```bash
cp /home/unitree/my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json \
   /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_bt_planner/calibration/arm_calibration.json
```

Expected: 文件创建成功,内容应含 `_meta.transformation = "tcp_link -> camera_arm_color_optical_frame"` 与 `position: [-0.07596965491001527, -0.000518235624476887, -0.0815325830105699]`(注意 z=-0.0815,不是原版的 +0.0385)。**绝对不要用原版 `2026-09-02_08-34-16_calibration.json`,那是 link6 系不是 tcp 系**(spec §4.1 / §9 #2)。

### Step 3: 修改 CMakeLists.txt 加 calibration install

在 `CMakeLists.txt` line 130 后(`install(DIRECTORY bt_trees/ ...)` 之后)插入:

```cmake
# Calibration JSONs for ComputeLookatPose node — loaded at node
# construction time via ament_index_cpp::get_package_share_directory.
# arm_calibration.json MUST be the _tcp.json variant (T_tcp_link →
# camera_arm_color_optical_frame); see spec §4.1, §9 #4.
install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)
```

### Step 4: 修改 package.xml 加 nlohmann-json3-dev 依赖

在 `<depend>yoloe_interfaces</depend>` 之后(line 24)插入:

```xml
<build_depend>nlohmann-json3-dev</build_depend>
```

不需要 `<exec_depend>`,因为 JSON 解析只发生在 build/construct 期,运行时节点不需要它。

### Step 5: build 验证 install 部署

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -20
"
```

Expected: 无 error,看到 `Starting >>> lgw_bt_planner` 与 `Finished`。

### Step 6: 验证 install 路径下 JSON 存在

```bash
ls /my_ws/l4t/lg_walker_ws/install/lgw_bt_planner/share/lgw_bt_planner/calibration/
```

Expected: 两个文件都存在:`arm_calibration.json`  `head_calibration.json`。

### Step 7: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/calibration/ src/lgw_bt_planner/CMakeLists.txt src/lgw_bt_planner/package.xml
git commit -m "$(cat <<'EOF'
lgw: add handeye calibration JSONs + CMakeLists install rule

head_calibration.json = T_base_link → camera_head_color_optical_frame
arm_calibration.json  = T_tcp_link → camera_arm_color_optical_frame
  (must use _tcp.json variant; tcp_offset=[0,0,0.12,...] pre-applied)

Add nlohmann-json3-dev build_depend for JSON parsing in
ComputeLookatPose (Task 6).
EOF
)"
```

---

## Task 2: 纯函数 `parse_calibration_json()` + gtest TC7-10

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp`
- Create: `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp`
- Create: `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`(加 `tests/test_compute_lookat_pose.cpp` 到 gtest target)

**目的:** 把"加载标定 JSON 文件 → 转 tf2 矩阵"做成无 ROS runtime 依赖的纯函数,TC7-10 覆盖合法 / 缺字段 / quat 模 0 / 文件不存在四种情况。TDD 优先。

**为什么先做这个:** 节点构造期第一步是 load JSON。如果 JSON 加载的失败路径(文件不存在 / 字段缺失)没测过,Task 6 节点构造期的 `initialized_` 标志会失效。

### Step 1: 写失败测试 TC7-10

新建 `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp`,先写 4 个测试(本任务只跑 TC7-10,TC1-6 / TC11 后续任务补):

```cpp
// tests/test_compute_lookat_pose.cpp
// Pure-function unit tests for ComputeLookatPose.
//
// No ROS runtime needed for TC1-6 (math only). TC11 (integration smoke)
// uses geometry_msgs::msg::PoseStamped which is header-only.
//
// Run directly from build/lgw_bt_planner/test_compute_lookat_pose
// (colcon test not available in this container — see CLAUDE.md).

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{
// Verify a 3x3 rotation matrix is a valid rotation: R*R^T = I,
// det(R) = +1.
void ExpectRotation(const tf2::Matrix3x3& R)
{
  tf2::Matrix3x3 rt = R.transpose();
  tf2::Matrix3x3 rr = R * rt;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(rr[i][j], expected, 1e-9)
        << "R*R^T[" << i << "][" << j << "]";
    }
  }
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

// Write a JSON literal to a temp file and return its path. Caller
// must unlink().
std::string write_temp_json(const std::string& body)
{
  std::string path = std::string(std::tmpnam(nullptr)) + ".json";
  std::ofstream f(path);
  f << body;
  f.close();
  return path;
}
}  // namespace

// === parse_calibration_json tests (TC7-10) ==========================

// TC7: 合法 JSON → 数值正确解出,quat 模 = 1
TEST(ParseCalibration, ValidJsonParsesCorrectly)
{
  const std::string body = R"({
    "position":    [-0.076, -0.0005, -0.0815],
    "orientation": [-0.122, 0.123, -0.683, 0.709],
    "rpy":         [[-0.348, 0.0075, -1.534]]
  })";
  const std::string path = write_temp_json(body);

  auto [R, t] = lgw_bt_planner::parse_calibration_json(path);

  EXPECT_NEAR(t.x(), -0.076, 1e-6);
  EXPECT_NEAR(t.y(), -0.0005, 1e-6);
  EXPECT_NEAR(t.z(), -0.0815, 1e-6);
  ExpectRotation(R);

  std::remove(path.c_str());
}

// TC8: 缺 orientation 字段 → 抛异常
TEST(ParseCalibration, MissingOrientationThrows)
{
  const std::string body = R"({"position": [0.0, 0.0, 0.0]})";
  const std::string path = write_temp_json(body);

  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json(path),
    std::runtime_error);

  std::remove(path.c_str());
}

// TC9: orientation 模 = 0(全 0 四元数) → 抛异常
TEST(ParseCalibration, ZeroQuaternionThrows)
{
  const std::string body = R"({
    "position":    [0.0, 0.0, 0.0],
    "orientation": [0.0, 0.0, 0.0, 0.0]
  })";
  const std::string path = write_temp_json(body);

  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json(path),
    std::runtime_error);

  std::remove(path.c_str());
}

// TC10: 文件不存在 → 抛异常
TEST(ParseCalibration, NonexistentFileThrows)
{
  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json("/tmp/__no_such_calibration__.json"),
    std::runtime_error);
}
```

新建 `src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp`(hpp 占位,只声明 3 纯函数 + 前向声明,BT 类留给 Task 6):

```cpp
// include/lgw_bt_planner/compute_lookat_pose_node.hpp
//
// ComputeLookatPose BT node + pure lookat math utilities.
//
// Pure functions (Task 2-5):
//   - parse_calibration_json(path) -> (R, t) as tf2 types
//   - build_lookat_orientation(direction, world_up) -> R_arm_in_base
//   - compute_tcp_pose(P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp)
//       -> PoseStamped
//
// BT class (Task 6): ComputeLookatPoseNode (SyncActionNode).
#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <utility>

namespace lgw_bt_planner
{

// Load a handeye calibration JSON file and return (R, t) where R is
// the rotation (quaternion w,x,y,z -> Matrix3x3) and t is the
// translation, both as tf2 types. JSON format:
//   { "position": [x, y, z], "orientation": [w, x, y, z], "rpy": [...] }
// Throws std::runtime_error on: file missing, parse error, missing
// fields, or quaternion with norm == 0.
std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& path);

// Build arm-optical-in-base rotation from a unit direction vector.
// Convention: arm_optical z = direction (forward), arm_optical y =
// direction × x, arm_optical x = world_up × direction (Gram-Schmidt).
// If direction ≈ ±world_up (Gram-Schmidt degenerate), fall back to
// world_x = (1,0,0) as the orthogonalization reference.
// Returns Matrix3x3 with columns [x_axis, y_axis, z_axis].
tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& direction,
  const tf2::Vector3& world_up);

// Given the desired arm-camera-origin position P_ee in base, plus the
// rotation/translation of the arm camera in the tcp frame
// (T_tcp -> camera_arm_color_optical_frame), compute the tcp pose in
// the base frame such that the arm camera ends up at P_ee.
// Math:
//   R_tcp_in_base = R_arm_in_base * inverse(R_arm_in_tcp)
//   P_tcp_in_base = P_ee - R_tcp_in_base * t_arm_in_tcp
geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& P_ee_in_base,
  const tf2::Matrix3x3& R_arm_in_base,
  const tf2::Matrix3x3& R_arm_in_tcp,
  const tf2::Vector3& t_arm_in_tcp);

}  // namespace lgw_bt_planner
```

新建 `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp`(本任务只 stub 实现,Task 3-5 补):

```cpp
// src/compute_lookat_pose_node.cpp
// Stub for Task 2 — full implementation in Tasks 3-5.

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <stdexcept>

namespace lgw_bt_planner
{

std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& /*path*/)
{
  throw std::runtime_error("not yet implemented (Task 2 stub)");
}

tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& /*direction*/,
  const tf2::Vector3& /*world_up*/)
{
  throw std::runtime_error("not yet implemented (Task 2 stub)");
}

geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& /*P_ee_in_base*/,
  const tf2::Matrix3x3& /*R_arm_in_base*/,
  const tf2::Matrix3x3& /*R_arm_in_tcp*/,
  const tf2::Vector3& /*t_arm_in_tcp*/)
{
  throw std::runtime_error("not yet implemented (Task 2 stub)");
}

}  // namespace lgw_bt_planner
```

### Step 2: 跑测试确认失败

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10
"
```

Expected: build 成功(TODO stub 函数已 stub)。

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  ./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='ParseCalibration.*' 2>&1 | tail -30
"
```

Expected: 4 个 ParseCalibration 测试全 FAIL(`std::runtime_error: not yet implemented (Task 2 stub)`)。

注:CMakeLists 已有的 `test_evaluate_detection` 块(`ament_add_gtest(test_compute_lookat_pose ...)`)镜像加在 line 138-148 后;具体语法见 spec §7 "单元测试 CMakeLists 修改"。

### Step 3: 在 CMakeLists 加 test target

在 `CMakeLists.txt` line 148 后(`endif()` 关闭 `BUILD_TESTING` 块之前)插入:

```cmake
ament_add_gtest(test_compute_lookat_pose tests/test_compute_lookat_pose.cpp)
target_link_libraries(test_compute_lookat_pose lgw_bt_planner_nodes)
ament_target_dependencies(test_compute_lookat_pose
  rclcpp tf2 tf2_geometry_msgs geometry_msgs yoloe_interfaces)
```

注:同时需要把 `src/compute_lookat_pose_node.cpp` 加到 `NODE_SOURCES` 列表(line 32-52),否则 stub 函数不在 shared 库里:

在 `NODE_SOURCES` block(line 46 `src/wait_for_user_input_bt_node.cpp` 之后)插入:

```cmake
  src/compute_lookat_pose_node.cpp
```

### Step 4: 跑测试确认 4 个 ParseCalibration 仍 FAIL(stub 阶段)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10
"
./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='ParseCalibration.*'
"
```

Expected: 4 FAIL,原因 `not yet implemented (Task 2 stub)`。

### Step 5: 实现 `parse_calibration_json`

替换 `src/compute_lookat_pose_node.cpp` 的 stub 实现:

```cpp
// src/compute_lookat_pose_node.cpp
// Pure lookat math + JSON parser (Task 2 complete; Tasks 3-5 fill the rest).

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lgw_bt_planner
{

std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error(
      "parse_calibration_json: cannot open " + path);
  }

  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    throw std::runtime_error(
      std::string("parse_calibration_json: JSON parse error in ") +
      path + ": " + e.what());
  }

  // Required fields
  if (!j.contains("position") || !j.contains("orientation")) {
    throw std::runtime_error(
      "parse_calibration_json: missing 'position' or 'orientation' in " +
      path);
  }

  const auto& pos = j["position"];
  const auto& q   = j["orientation"];
  if (pos.size() != 3 || q.size() != 4) {
    throw std::runtime_error(
      "parse_calibration_json: position must be 3-vector, orientation 4-vector");
  }

  // Quaternion in file is w-first (w, x, y, z) per spec §4.1.
  const double qw = q[0].get<double>();
  const double qx = q[1].get<double>();
  const double qy = q[2].get<double>();
  const double qz = q[3].get<double>();

  const double qnorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qnorm < 1e-9) {
    throw std::runtime_error(
      "parse_calibration_json: quaternion norm ≈ 0 in " + path);
  }
  // Normalize so tf2::Matrix3x3 sees a unit quaternion.
  const double k = 1.0 / qnorm;

  tf2::Quaternion tfq;
  // tf2::Quaternion(x, y, z, w) constructor takes (x, y, z, w).
  tfq.setX(qx * k);
  tfq.setY(qy * k);
  tfq.setZ(qz * k);
  tfq.setW(qw * k);

  tf2::Matrix3x3 R(tfq);
  tf2::Vector3 t(pos[0].get<double>(), pos[1].get<double>(),
                 pos[2].get<double>());

  return {R, t};
}

}  // namespace lgw_bt_planner
```

(其他 2 个纯函数 stub 保留到 Task 3-5 实现)

### Step 6: 跑测试确认 4 个 ParseCalibration 全 PASS

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='ParseCalibration.*'
"
```

Expected: 4 个 ParseCalibration 测试全 PASS(0 FAIL)。

### Step 7: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp \
        src/lgw_bt_planner/src/compute_lookat_pose_node.cpp \
        src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "$(cat <<'EOF'
lgw: add parse_calibration_json pure function + TC7-10

nlohmann::json reader for {position, orientation} (w,x,y,z) ->
(R=tf2::Matrix3x3, t=tf2::Vector3). Throws on missing file, missing
fields, or zero-norm quaternion. Tests cover valid/missing-field/
zero-quat/nonexistent cases (TC7-10).
EOF
)"
```

---

## Task 3: 纯函数 `build_lookat_orientation()` + gtest TC1-4

**Files:**
- Modify: `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp`(实现 `build_lookat_orientation`)
- Modify: `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp`(加 TC1-4)

**目的:** 实现 Gram-Schmidt 构造 arm optical 在 base 下旋转矩阵的纯函数,TC1-4 覆盖正常方向 / direction 与 world_up 平行退化 / 零向量 / R 矩阵正交性验证(spec §3 Step 3)。

**为什么独立成 task:** 这个函数是算法核心,几何不变量最多(正交性、determinant=+1、与输入 direction 对齐),需要单独 review。

### Step 1: 在 test 文件追加 TC1-4

在 `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp` 末尾(`TC10` 之后)追加:

```cpp
// === build_lookat_orientation tests (TC1-4) ========================

// TC1: direction = (0, 1, 0) (沿 +y), world_up = (0, 0, 1) →
//   z_axis = direction = (0, 1, 0)
//   x_axis = direction × world_up = (0,1,0) × (0,0,1)
//         = (1·1 - 0·0, 0·0 - 0·1, 0·0 - 1·0) = (1, 0, 0)
//   y_axis = z_axis × x_axis = (0,1,0) × (1,0,0)
//         = (1·0 - 0·0, 0·1 - 0·0, 0·0 - 1·1) = (0, 0, -1)
// (NOTE: 早期 TC1 用 direction=(0,0,-1)/world_up=(0,0,1) 是退化 case,
// 触发 world_x fallback,不是标准 case,Task 3 implementer 报告后已修)
TEST(BuildLookatOrientation, DirectionAlongYGivesCanonicalBasis)
{
  const tf2::Vector3 dir(0, 1, 0);
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  // tf2::Matrix3x3 element access via operator[](row) returns row-vector
  // of 3. R has columns [x_axis, y_axis, z_axis].
  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  tf2::Vector3 y_axis(R[0][1], R[1][1], R[2][1]);
  tf2::Vector3 z_axis(R[0][2], R[1][2], R[2][2]);

  EXPECT_NEAR(x_axis.x(), 1.0, 1e-9);
  EXPECT_NEAR(x_axis.y(), 0.0, 1e-9);
  EXPECT_NEAR(x_axis.z(), 0.0, 1e-9);

  EXPECT_NEAR(y_axis.x(),  0.0, 1e-9);
  EXPECT_NEAR(y_axis.y(),  0.0, 1e-9);
  EXPECT_NEAR(y_axis.z(), -1.0, 1e-9);

  EXPECT_NEAR(z_axis.x(), 0.0, 1e-9);
  EXPECT_NEAR(z_axis.y(), 1.0, 1e-9);
  EXPECT_NEAR(z_axis.z(), 0.0, 1e-9);

  ExpectRotation(R);
}

// TC2: direction = (1, 0, 0), world_up = (0, 0, 1) →
//   x_axis = direction × world_up = (0·1 - 0·0, 0·0 - 1·1, 1·0 - 0·0)
//       = (0, -1, 0)
//   y_axis = z_axis × x_axis = (1,0,0) × (0,-1,0) = (0·0 - 0·(-1),
//                                                   0·0 - 1·0,
//                                                   1·(-1) - 0·0)
//       = (0, 0, -1)
TEST(BuildLookatOrientation, XDirectionYieldsOrthonormalBasis)
{
  const tf2::Vector3 dir(1, 0, 0);
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  tf2::Vector3 y_axis(R[0][1], R[1][1], R[2][1]);
  tf2::Vector3 z_axis(R[0][2], R[1][2], R[2][2]);

  EXPECT_NEAR(x_axis.y(), -1.0, 1e-9);
  EXPECT_NEAR(y_axis.z(), -1.0, 1e-9);
  EXPECT_NEAR(z_axis.x(),  1.0, 1e-9);

  ExpectRotation(R);
}

// TC3: direction ≈ world_up (平行) → Gram-Schmidt 退化,fallback
// 用 world_x = (1, 0, 0)。验证 x_axis ≠ 0 向量且 R 仍合法旋转。
TEST(BuildLookatOrientation, DegenerateCaseUsesFallback)
{
  const tf2::Vector3 dir(0, 0, 1);   // 与 world_up 完全平行
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  EXPECT_GT(x_axis.length(), 1e-6);  // 不是零向量
  ExpectRotation(R);
}

// TC4: direction 范数 0 → 抛异常(防除零)
TEST(BuildLookatOrientation, ZeroDirectionThrows)
{
  const tf2::Vector3 dir(0, 0, 0);
  const tf2::Vector3 up(0, 0, 1);

  EXPECT_THROW(
    lgw_bt_planner::build_lookat_orientation(dir, up),
    std::runtime_error);
}
```

### Step 2: 跑测试确认 TC1-4 FAIL(stub 阶段)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='BuildLookatOrientation.*' 2>&1 | tail -30
"
```

Expected: 4 个 BuildLookatOrientation 测试全 FAIL(`not yet implemented (Task 3 stub)`)。

### Step 3: 实现 `build_lookat_orientation`

在 `src/compute_lookat_pose_node.cpp` 替换 stub:

```cpp
tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& direction,
  const tf2::Vector3& world_up)
{
  // Algorithm: spec §3 Step 3 (Gram-Schmidt with world_up fallback).
  //   raw_x = direction × world_up
  //   if ||raw_x|| < epsilon: raw_x = direction × world_x  (fallback)
  //   x_axis = raw_x.normalize()
  //   y_axis = direction × x_axis
  //   R = [x_axis | y_axis | direction]   (columns)
  //
  // Camera optical frame convention (ROS): z = forward, y = down,
  // x = right. Our direction = forward → z_axis = direction.

  const double dir_norm = direction.length();
  if (dir_norm < 1e-6) {
    throw std::runtime_error(
      "build_lookat_orientation: direction vector has zero norm");
  }

  tf2::Vector3 dir_unit = direction * (1.0 / dir_norm);

  tf2::Vector3 raw_x = dir_unit.cross(world_up);
  if (raw_x.length() < 1e-6) {
    // direction ≈ ±world_up: Gram-Schmidt degenerate, fall back to
    // world_x as orthogonalization reference.
    raw_x = dir_unit.cross(tf2::Vector3(1, 0, 0));
  }
  if (raw_x.length() < 1e-6) {
    throw std::runtime_error(
      "build_lookat_orientation: cannot construct orthogonal basis "
      "(direction parallel to both world_up and world_x)");
  }

  tf2::Vector3 x_axis = raw_x.normalized();
  tf2::Vector3 y_axis = dir_unit.cross(x_axis).normalized();

  // Build R with columns [x_axis, y_axis, z_axis=dir_unit].
  tf2::Matrix3x3 R;
  R.setValue(
    x_axis.x(),  y_axis.x(),  dir_unit.x(),
    x_axis.y(),  y_axis.y(),  dir_unit.y(),
    x_axis.z(),  y_axis.z(),  dir_unit.z());
  return R;
}
```

### Step 4: 跑测试确认 TC1-4 全 PASS

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='BuildLookatOrientation.*'
"
```

Expected: 4 PASS。同时跑一下之前 Task 2 的 ParseCalibration 4 个,确认没破:

```bash
./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='ParseCalibration.*'
```

Expected: 4 PASS。

### Step 5: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/src/compute_lookat_pose_node.cpp \
        src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp
git commit -m "$(cat <<'EOF'
lgw: add build_lookat_orientation pure function + TC1-4

Gram-Schmidt construction of arm_optical_in_base rotation:
  z_axis = direction (unit)
  x_axis = normalize(direction × world_up)
  y_axis = normalize(direction × x_axis)
Falls back to world_x = (1,0,0) when direction ≈ ±world_up.
Throws on zero-norm direction. Tests cover standard, degenerate,
and R*R^T=I / det=+1 invariant checks (TC1-4).
EOF
)"
```

---

## Task 4: 纯函数 `compute_tcp_pose()` + gtest TC5-6

**Files:**
- Modify: `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp`(实现 `compute_tcp_pose`)
- Modify: `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp`(加 TC5-6)

**目的:** 把"已知 arm camera 目标位置 + arm camera 相对 tcp 的 offset → 算 tcp 目标位姿"封成纯函数。TC5(零偏移校)+ TC6(旋转偏移)。

**为什么独立成 task:** 这个函数是连接 `build_lookat_orientation` 与节点本体的桥梁,公式本身只有 2 行但语义易错(矩阵左右乘方向、t 取反符号)。

**Brief correction (carry-over from BLOCKED first dispatch):** 原 TC5-6 用 `tf2::Matrix3x3 R_arm_in_base;  // default = I` 是错的 —— `tf2::Matrix3x3` 默认 ctor **不**初始化成员(`Vector3 m_el[3]` 在 `Matrix3x3.hpp:36` 是裸成员),必须显式 `setIdentity()`。修正后版本见 Step 1。`compute_tcp_pose` 实现本身无错。

### Step 1: 在 test 文件追加 TC5-6(修正版,带显式 `setIdentity()`)

```cpp
// === compute_tcp_pose tests (TC5-6) ===============================

// TC5: R_arm_in_base = I, R_arm_in_tcp = I, t_arm_in_tcp = (0,0,-0.12),
//   P_ee = (0.2, 0, 0.3) →
//   R_tcp = I * I = I,
//   P_tcp = P_ee - I * (0,0,-0.12) = (0.2, 0, 0.3) - (0,0,-0.12)
//         = (0.2, 0, 0.42)
// 验证:R_tcp 是 identity 四元数(0,0,0,1),P_tcp = (0.2, 0, 0.42)。
TEST(ComputeTcpPose, IdentityCaseSolvesOffset)
{
  // tf2::Matrix3x3 default ctor leaves m_el[] uninitialized;
  // explicit setIdentity() is mandatory.
  tf2::Matrix3x3 R_arm_in_base;
  R_arm_in_base.setIdentity();
  tf2::Matrix3x3 R_arm_in_tcp;
  R_arm_in_tcp.setIdentity();
  tf2::Vector3 t_arm_in_tcp(0, 0, -0.12);
  tf2::Vector3 P_ee(0.2, 0, 0.3);

  auto pose = lgw_bt_planner::compute_tcp_pose(
    P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp);

  EXPECT_NEAR(pose.pose.position.x, 0.2,  1e-9);
  EXPECT_NEAR(pose.pose.position.y, 0.0,  1e-9);
  EXPECT_NEAR(pose.pose.position.z, 0.42, 1e-9);

  // orientation = identity → (x=0, y=0, z=0, w=1)
  EXPECT_NEAR(pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, 1.0, 1e-9);
}

// TC6: R_arm_in_base = 绕 z 旋转 90°,R_arm_in_tcp = I,
//   t_arm_in_tcp = (0, 0.1, 0),P_ee = (0.5, 0, 0.3) →
//   R_tcp = R_arm_in_base (因为 R_arm_in_tcp = I),
//   P_tcp = P_ee - R_tcp * (0, 0.1, 0)
//   R_z(90°) 把 +y 转到 -x,所以 (0, 0.1, 0) → (-0.1, 0, 0)
//        = (0.5, 0, 0.3) - (-0.1, 0, 0)
//        = (0.6, 0, 0.3)
// 验证:R_tcp 四元数 = (0, 0, sin(45°), cos(45°)) = (0, 0, 0.7071, 0.7071)。
TEST(ComputeTcpPose, YRotatedBaseInvertsAtTcp)
{
  // R_arm_in_base = rotation 90° around z
  tf2::Matrix3x3 R_arm_in_base;
  R_arm_in_base.setRPY(0, 0, M_PI / 2);

  // tf2::Matrix3x3 default ctor leaves m_el[] uninitialized;
  // explicit setIdentity() is mandatory.
  tf2::Matrix3x3 R_arm_in_tcp;
  R_arm_in_tcp.setIdentity();
  tf2::Vector3 t_arm_in_tcp(0, 0.1, 0);
  tf2::Vector3 P_ee(0.5, 0, 0.3);

  auto pose = lgw_bt_planner::compute_tcp_pose(
    P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp);

  EXPECT_NEAR(pose.pose.position.x,  0.6, 1e-9);
  EXPECT_NEAR(pose.pose.position.y,  0.0, 1e-9);
  EXPECT_NEAR(pose.pose.position.z,  0.3, 1e-9);

  // orientation: 90° around z → (x=0, y=0, z=sin(45°), w=cos(45°))
  EXPECT_NEAR(pose.pose.orientation.x, 0.0,     1e-9);
  EXPECT_NEAR(pose.pose.orientation.y, 0.0,     1e-9);
  EXPECT_NEAR(pose.pose.orientation.z, M_SQRT1_2, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, M_SQRT1_2, 1e-9);
}
```

注意:TC5/TC6 需要 `<cmath>` 提供 `M_PI` 与 `M_SQRT1_2`。`test_compute_lookat_pose.cpp` 顶部已 include `<cmath>`,无需重复。

### Step 2: 跑测试确认 TC5-6 状态

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  export LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash && \
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose --gtest_filter='ComputeTcpPose.*' 2>&1 | tail -20
"
```

Expected: TC5-6 PASS。如果只有 8 PASS → 2 FAIL with NaN,说明 working tree 仍残留旧版 buggy TC5-6(来自第一轮 dispatch 的 uncommitted diff),需要先**编辑 test 文件**把 `// default = I` 注释替换成 `setIdentity()` 调用(brief Step 1 修正版),再重新 build + test。

如果 impl 还没在 working tree 中,会看到 "not yet implemented (Task 4 stub)" 而不是 NaN —— 那才是真正的 RED,继续 Step 3。

### Step 3: 实现 `compute_tcp_pose`(如果 working tree 还没有)

在 `src/compute_lookat_pose_node.cpp` 替换 stub:

```cpp
geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& P_ee_in_base,
  const tf2::Matrix3x3& R_arm_in_base,
  const tf2::Matrix3x3& R_arm_in_tcp,
  const tf2::Vector3& t_arm_in_tcp)
{
  // spec §3 Step 4-5:
  //   R_tcp_in_base = R_arm_in_base * inverse(R_arm_in_tcp)
  //   P_tcp_in_base = P_ee - R_tcp_in_base * t_arm_in_tcp
  //
  // The "inverse(R_arm_in_tcp)" is R_arm_in_tcp^T since R is a
  // rotation matrix; tf2 provides .inverse() that handles this.
  tf2::Matrix3x3 R_tcp_in_base = R_arm_in_base * R_arm_in_tcp.inverse();
  tf2::Vector3 P_tcp_in_base =
    P_ee_in_base - R_tcp_in_base * t_arm_in_tcp;

  // Convert R_tcp_in_base to quaternion (x, y, z, w).
  tf2::Quaternion q;
  R_tcp_in_base.getRotation(q);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "";   // caller fills in
  pose.pose.position.x = P_tcp_in_base.x();
  pose.pose.position.y = P_tcp_in_base.y();
  pose.pose.position.z = P_tcp_in_base.z();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}
```

### Step 4: 跑测试确认 TC5-6 全 PASS

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  export LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash && \
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose
"
```

Expected: 全部 10 个测试(ParseCalibration 4 + BuildLookatOrientation 4 + ComputeTcpPose 2)PASS,0 FAIL。

### Step 5: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/src/compute_lookat_pose_node.cpp \
        src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp
git commit -m "$(cat <<'EOF'
lgw: add compute_tcp_pose pure function + TC5-6

  R_tcp_in_base = R_arm_in_base * inverse(R_arm_in_tcp)
  P_tcp_in_base = P_ee - R_tcp_in_base * t_arm_in_tcp

Tests cover identity case (zero offset correction) and 90°-around-z
rotation case (TC5-6). Note: tf2::Matrix3x3 default ctor leaves
m_el[] uninitialized; tests use explicit setIdentity() calls.
EOF
)"
```

---

## Task 5: 集成冒烟测试 TC11 — 完整 6 步算法

**Files:**
- Modify: `src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp`(加 TC11)

**目的:** 把"读 head obs + 两份标定 fixture → 跑完整 6 步算法 → 验证输出 PoseStamped 数值合理"做成一个端到端冒烟测试。这是 Task 6 节点构造 + tick 路径的 fast-feedback 等价物。

**为什么独立成 task:** 这个测试在节点本体写出来之前就锁住了算法契约 —— review TC11 就能 catch 6 步算法里任何一处公式错误。

### Step 1: 在 test 文件追加 TC11

```cpp
// === Integration smoke test (TC11) ================================
//
// Drives the full 6-step algorithm with synthetic fixtures:
//   - head calibration (T_base_link → camera_head): pos (0.3, 0, -0.01)
//   - arm calibration  (T_tcp_link → camera_arm):   pos (-0.07, 0, -0.08)
//   - head observation: target at (0, 0, 1) in head optical frame
//   - desired ee_position: (0.2, 0, 0.3) in world
//
// Expected output sanity (not exact values — Gram-Schmidt sign may
// differ; we only check P_tcp is reasonable and frame is correct):

#include <yoloe_interfaces/msg/observation_result.hpp>
#include <yoloe_interfaces/msg/yoloe_detection3_d.hpp>

namespace
{
tf2::Matrix3x3 quat_to_R(double w, double x, double y, double z)
{
  tf2::Quaternion q(x, y, z, w);
  return tf2::Matrix3x3(q);
}
}  // namespace

TEST(IntegrationSmoke, FullAlgorithmProducesReasonableTcpPose)
{
  // ---- Fixtures (inline strings for temp files) ----
  const std::string head_body = R"({
    "position":    [0.3, 0.0, -0.01],
    "orientation": [1.0, 0.0, 0.0, 0.0]
  })";
  const std::string arm_body = R"({
    "position":    [-0.07, 0.0, -0.08],
    "orientation": [1.0, 0.0, 0.0, 0.0]
  })";
  const std::string head_json = write_temp_json(head_body);
  const std::string arm_json  = write_temp_json(arm_body);

  // ---- Step 1-2: parse + transform target to base ----
  auto [R_h2b, t_h2b] = lgw_bt_planner::parse_calibration_json(head_json);
  auto [R_a2t, t_a2t] = lgw_bt_planner::parse_calibration_json(arm_json);

  // head_calibration is identity quat (R_h2b = I).
  tf2::Vector3 P_target_head(0, 0, 1);  // 1m straight ahead in head frame
  tf2::Vector3 P_target_base = R_h2b * P_target_head + t_h2b;
  EXPECT_NEAR(P_target_base.x(), 0.30, 1e-9);
  EXPECT_NEAR(P_target_base.y(), 0.00, 1e-9);
  EXPECT_NEAR(P_target_base.z(), 0.99, 1e-9);  // 1.0 - 0.01

  // ---- Step 2: direction ----
  const tf2::Vector3 P_ee_target_world(0.2, 0, 0.3);
  tf2::Vector3 direction = (P_target_base - P_ee_target_world);
  const double dir_len = direction.length();
  ASSERT_GT(dir_len, 1e-6);
  direction = direction * (1.0 / dir_len);

  // ---- Step 3: build lookat rotation ----
  const tf2::Vector3 world_up(0, 0, 1);
  tf2::Matrix3x3 R_arm_in_base =
    lgw_bt_planner::build_lookat_orientation(direction, world_up);
  ExpectRotation(R_arm_in_base);

  // ---- Step 4-5: tcp pose ----
  auto pose = lgw_bt_planner::compute_tcp_pose(
    P_ee_target_world, R_arm_in_base, R_a2t, t_a2t);

  // Sanity checks (loose bounds — exact values depend on Gram-Schmidt
  // orientation choices which are not over-constrained):
  //   1. P_tcp should be within 0.5 m of P_ee (camera offset is small).
  //   2. Orientation should be a unit quaternion.
  tf2::Vector3 P_tcp(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);
  EXPECT_LT((P_tcp - P_ee_target_world).length(), 0.5);

  const double qnorm = std::sqrt(
    pose.pose.orientation.x * pose.pose.orientation.x +
    pose.pose.orientation.y * pose.pose.orientation.y +
    pose.pose.orientation.z * pose.pose.orientation.z +
    pose.pose.orientation.w * pose.pose.orientation.w);
  EXPECT_NEAR(qnorm, 1.0, 1e-9);

  std::remove(head_json.c_str());
  std::remove(arm_json.c_str());
}
```

### Step 2: 跑测试确认 TC11 PASS

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5 && \
  ./build/lgw_bt_planner/test_compute_lookat_pose
"
```

Expected: 11/11 PASS(0 FAIL)。如果 fail,检查:`build_lookat_orientation` 在 direction ≈ ±world_up 退化时的 fallback 路径是否触发(本测试 direction 不退化,不该触发)。

### Step 3: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/tests/test_compute_lookat_pose.cpp
git commit -m "$(cat <<'EOF'
lgw: add integration smoke test TC11 for full 6-step algorithm

Drives the complete pipeline (parse → transform → Gram-Schmidt →
tcp pose) with synthetic fixtures. Verifies output is a unit
quaternion and tcp position is within 0.5m of desired ee position.

Confirms algorithm contract end-to-end before Task 6 wires it into
the BT node.
EOF
)"
```

---

## Task 6: BT 节点 `ComputeLookatPoseNode` + 插件注册

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp`(加 BT 类声明)
- Modify: `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp`(加 BT 类实现)
- Modify: `src/lgw_bt_planner/src/bt_plugin_registration.cpp`(加 1 行 `registerNodeType<ComputeLookatPoseNode>`)

**目的:** 把 Task 2-5 的 3 纯函数包到 BT SyncActionNode 里:构造期一次性 load 2 份 JSON → onTick 读 head 观测 + ee_position → 6 步算法 → 写 PoseStamped 到黑板。沿用现有 `set_pose_stamped_node.hpp` 的 SyncActionNode 模式。

**为什么放在纯函数之后:** Tasks 2-5 已经把算法核心锁住 + 11 TC 覆盖;Task 6 只做 BT 框架胶水(参数解析 + 黑板读写 + 错误处理映射),reviewer 关注的是 BT 契约而非数学正确性。

### Step 1: 扩展 hpp — 加 BT 类声明

替换 `src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp` 内容(纯函数部分保留,加 BT 类):

```cpp
// include/lgw_bt_planner/compute_lookat_pose_node.hpp
//
// ComputeLookatPose BT node + pure lookat math utilities.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yoloe_interfaces/msg/observation_result.hpp>

#include <memory>
#include <string>
#include <utility>

namespace lgw_bt_planner
{

// === Pure functions (Tasks 2-5) ===

std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& path);

tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& direction,
  const tf2::Vector3& world_up);

geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& P_ee_in_base,
  const tf2::Matrix3x3& R_arm_in_base,
  const tf2::Matrix3x3& R_arm_in_tcp,
  const tf2::Vector3& t_arm_in_tcp);

// === BT node (Task 6) ===

// SyncActionNode that takes a head ObservationResult + the desired
// arm-camera-origin world position + 2 handeye calibration JSONs
// (loaded at construction time) and writes a PoseStamped for the
// tcp_link onto the blackboard.
//
// Construction: loads 2 calibration JSONs once. On any load failure,
// sets `initialized_ = false` and `onTick` always returns FAILURE.
// This keeps the node safe under Repeat num_cycles="-1" — it
// degrades gracefully instead of crashing.
class ComputeLookatPoseNode : public BT::SyncActionNode
{
public:
  ComputeLookatPoseNode(const std::string& name,
                        const BT::NodeConfig& config);

  ComputeLookatPoseNode() = delete;
  ComputeLookatPoseNode(const ComputeLookatPoseNode&) = delete;
  ComputeLookatPoseNode& operator=(const ComputeLookatPoseNode&) = delete;

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_{false};
  tf2::Matrix3x3 R_h2b_;     // head optical in base
  tf2::Vector3   t_h2b_;
  tf2::Matrix3x3 R_a2t_;     // arm optical in tcp
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner
```

### Step 2: 扩展 cpp — 加 BT 类实现

在 `src/lgw_bt_planner/src/compute_lookat_pose_node.cpp` 末尾追加(纯函数实现保留):

```cpp
// === BT node implementation (Task 6) ===============================

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>

#include <stdexcept>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultWorldFrame[] = "world";
constexpr char kDefaultOutputKey[]  = "lookat_pose";
constexpr char kDefaultEePosition[] = "0.2;0.0;0.3";
constexpr char kHeadCalibName[]     = "head_calibration.json";
constexpr char kArmCalibName[]      = "arm_calibration.json";

// Split "x;y;z" string into a Vector3.
tf2::Vector3 parse_semicolon_xyz(const std::string& s)
{
  tf2::Vector3 v;
  size_t p1 = s.find(';');
  size_t p2 = s.find(';', p1 + 1);
  if (p1 == std::string::npos || p2 == std::string::npos) {
    throw std::runtime_error(
      "parse_semicolon_xyz: expected 'x;y;z' format, got '" + s + "'");
  }
  v.setX(std::stod(s.substr(0, p1)));
  v.setY(std::stod(s.substr(p1 + 1, p2 - p1 - 1)));
  v.setZ(std::stod(s.substr(p2 + 1)));
  return v;
}

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}
}  // namespace

ComputeLookatPoseNode::ComputeLookatPoseNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  // Pull optional InputPorts for calibration paths. If not set, fall
  // back to the package share directory.
  std::string head_path, arm_path;
  if (auto p = getInput<std::string>("head_calibration_path"); p) {
    head_path = *p;
  } else {
    head_path = resolve_default_calib_path(kHeadCalibName);
  }
  if (auto p = getInput<std::string>("arm_calibration_path"); p) {
    arm_path = *p;
  } else {
    arm_path = resolve_default_calib_path(kArmCalibName);
  }

  try {
    std::tie(R_h2b_, t_h2b_) = parse_calibration_json(head_path);
    std::tie(R_a2t_, t_a2t_) = parse_calibration_json(arm_path);
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
      "Loaded calibration: head=%s arm=%s", head_path.c_str(),
      arm_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList ComputeLookatPoseNode::providedPorts()
{
  return {
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "head_observation", "Last CONFIRMED head ObservationResult"),
    BT::InputPort<std::string>(
      "ee_position_world", kDefaultEePosition,
      "Desired arm-camera-origin position in world; 'x;y;z'"),
    BT::InputPort<std::string>(
      "world_frame", kDefaultWorldFrame,
      "frame_id for output PoseStamped (also base for ee_position)"),
    BT::InputPort<std::string>(
      "head_calibration_path", "",
      "Path to head handeye JSON; empty = default share/.../head_calibration.json"),
    BT::InputPort<std::string>(
      "arm_calibration_path", "",
      "Path to arm handeye JSON (must be _tcp.json variant); "
      "empty = default share/.../arm_calibration.json"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "lookat_pose",
      "Computed tcp_link target pose; also written to blackboard"),
  };
}

BT::NodeStatus ComputeLookatPoseNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  // 1. Read head observation.
  auto obs = getInput<yoloe_interfaces::msg::ObservationResult>(
    "head_observation");
  if (!obs || obs->detections.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto& det = obs->detections.front();

  // 2. Read ee_position + world_frame.
  std::string ee_str = kDefaultEePosition;
  if (auto p = getInput<std::string>("ee_position_world"); p) {
    ee_str = *p;
  }
  std::string world_frame = kDefaultWorldFrame;
  if (auto p = getInput<std::string>("world_frame"); p) {
    world_frame = *p;
  }

  tf2::Vector3 P_ee_in_base;
  try {
    P_ee_in_base = parse_semicolon_xyz(ee_str);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "Bad ee_position_world '%s': %s", ee_str.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  // 3. Step 1: target in base.
  tf2::Vector3 P_target_head(det.center_m.x, det.center_m.y,
                             det.center_m.z);
  tf2::Vector3 P_target_base = R_h2b_ * P_target_head + t_h2b_;

  // 4. Step 2: direction.
  tf2::Vector3 raw_dir = P_target_base - P_ee_in_base;
  if (raw_dir.length() < 1e-3) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeLookatPose"),
      "target and ee_position coincide; geometric degeneracy");
    return BT::NodeStatus::FAILURE;
  }
  tf2::Vector3 direction = raw_dir.normalized();

  // 5. Step 3: build arm optical rotation in base.
  tf2::Matrix3x3 R_arm_in_base;
  try {
    R_arm_in_base = build_lookat_orientation(direction,
                                             tf2::Vector3(0, 0, 1));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "build_lookat_orientation failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // 6. Step 4-5: compute tcp pose.
  geometry_msgs::msg::PoseStamped pose;
  try {
    pose = compute_tcp_pose(P_ee_in_base, R_arm_in_base, R_a2t_, t_a2t_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "compute_tcp_pose failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  pose.header.frame_id = world_frame;
  pose.header.stamp = obs->header.stamp;

  // 7. Write blackboard + output port.
  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_pose_key"); p && !p->empty()) {
    output_key = *p;
  }
  setOutput("lookat_pose", pose);
  config().blackboard->set(output_key, pose);

  // Optional debug logging (gated on BT_PLANNER_DEBUG=1, matching
  // MoveArmToPoseActionNode convention).
  if (const char* env = std::getenv("BT_PLANNER_DEBUG");
      env && std::string(env) == "1")
  {
    RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
      "P_ee_in_base=(%.4f, %.4f, %.4f) P_target_base=(%.4f, %.4f, %.4f) "
      "P_tcp=(%.4f, %.4f, %.4f)",
      P_ee_in_base.x(), P_ee_in_base.y(), P_ee_in_base.z(),
      P_target_base.x(), P_target_base.y(), P_target_base.z(),
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
```

注:`getInput<std::string>("output_pose_key")` 没在 providedPorts 里列。补一下:`providedPorts()` 改为含 `BT::InputPort<std::string>("output_pose_key", "", "Blackboard key override (default 'lookat_pose')")` — 我把这一步合并到 Step 1 修订。

修订 Step 1 hpp 末尾的 `providedPorts` 注释 — 实际写在 cpp 里,hpp 不用改。在 cpp 的 `providedPorts()` 实现里加这一行(上面已含)。

### Step 3: 在 bt_plugin_registration.cpp 注册节点

修改 `src/lgw_bt_planner/src/bt_plugin_registration.cpp`:

在 line 14 `#include` block 末尾(`wait_for_user_input_bt_node.hpp` 之后)加:

```cpp
#include "lgw_bt_planner/compute_lookat_pose_node.hpp"
```

在 line 51-52(`WaitForUserInput` 注册行之后)加:

```cpp
  factory.registerNodeType<lgw_bt_planner::ComputeLookatPoseNode>(
    "ComputeLookatPose");
```

### Step 4: build 验证

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10
"
```

Expected: 无 error。

### Step 5: 跑所有单元测试,确认没破

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  ./build/lgw_bt_planner/test_compute_lookat_pose 2>&1 | tail -5
"
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  ./build/lgw_bt_planner/test_evaluate_detection 2>&1 | tail -5
"
```

Expected: 11/11 PASS(test_compute_lookat_pose),test_evaluate_detection 6 PASS(无回归)。

### Step 6: 验证 plugin symbol 导出

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  nm -D install/lgw_bt_planner/share/lgw_bt_planner/bt_plugins/lgw_bt_planner_nodes.so | grep -i ComputeLookatPose
"
```

Expected: 看到 `ComputeLookatPose` 注册符号(如 `_ZN16lgw_bt_planner21ComputeLookatPoseNode...`)。

### Step 7: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/include/lgw_bt_planner/compute_lookat_pose_node.hpp \
        src/lgw_bt_planner/src/compute_lookat_pose_node.cpp \
        src/lgw_bt_planner/src/bt_plugin_registration.cpp
git commit -m "$(cat <<'EOF'
lgw: add ComputeLookatPose BT node + plugin registration

SyncActionNode:
  - Constructor: loads 2 calibration JSONs (default paths under
    share/lgw_bt_planner/calibration/), sets initialized_=false
    on any load failure so onTick degrades to FAILURE forever
    (Repeat-safe).
  - tick(): head ObservationResult + ee_position_world + world_frame
    -> 6-step lookat algorithm -> PoseStamped on blackboard under
    `lookat_pose` (or output_pose_key override).

Algorithm delegates to the pure functions from Tasks 2-5 (parse,
build_lookat_orientation, compute_tcp_pose). All math correctness
is covered by 11 unit tests there; this task is BT framework glue
only.

Registered in bt_plugin_registration.cpp alongside WaitForUserInput.
EOF
)"
```

---

## Task 7: BT 树 XML `head_to_arm_lookat.xml`

**Files:**
- Modify: `src/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml`(修正 review 发现的 BT 语义 bug)
- (First commit a757a48 already created the file; this task adds a fix commit on top)

**目的:** 把 ReactiveFallback + Repeat + 内层 Sequence/RF 串起来,完整复刻 spec §6.1 的树结构。**不**引入新节点类型(只用 Task 6 注册的 `ComputeLookatPose` + 现有 5 个节点)。

**为什么独立成 task:** XML 是 BT 的核心契约,reviewer 重点检查 ReactiveFallback 子节点顺序、max_age_sec、named_targets 黑板引用。代码改动与 BT 节点完全分离,reviewer 可以单独 review XML。

**Brief correction (carry-over from BLOCKED review of first commit a757a48):**

First-dispatch XML had a BT semantics bug: when `refine_step` Sequence fails (e.g., head didn't see target), the brief's outer Sequence `refine_then_home` returns FAILURE → Repeat FAILURE (no loop) → outer RF FAILURE → BT terminates. This contradicts spec §6.1 which promises "refine_step FAILURE → home → 下一轮".

**Fix**: wrap `<Sequence name="refine_step">` in `<ForceSuccess>` decorator (BT.CPP v4 built-in). ForceSuccess maps any child status (FAILURE/SKIP/RUNNING) to SUCCESS, so the outer Sequence proceeds to `return_home` regardless of refine outcome.

`ForceSuccess` is registered by `behaviortree_cpp::BT::ForceSuccessNode` — same library as Repeat/Sequence/ReactiveFallback, no extra registration needed.

### Step 1: 编辑 XML(在现有 a757a48 commit 之上加 fix)

修改 `src/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml`(已经存在的 a757a48 commit 创建):

1. 更新文件顶部注释块,把"refine_step FAILURE → Sequence FAILURE → RF SUCCESS"那段改成"ForceSuccess 装饰器把 refine FAILURE 转 SUCCESS → 继续 home → 下一轮"。

2. 在 XML tree 里,包裹 `<Sequence name="refine_step">` 在 `<ForceSuccess>` 里。

完整替换后的 XML 内容如下(replace the entire file):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  head_to_arm_lookat.xml — head 看到目标 → 计算 lookat pose →
  arm 移到 → 检测 arm 是否也看到 → 是则回 home。

  结构(spec §6.1):
    ReactiveFallback root
      child 1: WaitForUserInput        (user msg → SUCCESS,整树终止)
      child 2: Repeat num_cycles="-1"
          child: Sequence "refine_then_home"
            step 1: ReactiveFallback "arm_seen_or_refine"
              child 1: IsArmDetectionConfirmed   (arm 看到 → 跳过 refine)
              child 2: ForceSuccess "refine_step_wrapper"  (review fix:
                把 refine 任何 outcome 转 SUCCESS,refine FAILURE 也能继续 home)
                  child: Sequence "refine_step"
                    IsHeadDetectionConfirmed          (head 必须看到)
                    ComputeLookatPose                 (head obs + 标定 JSON)
                    MoveArmToPose                    (MoveIt 到 tcp_link target)
            step 2: MoveArmToNamed "home"         (每轮回到 home)

  行为:
    - 无 user msg → Repeat 无限循环
    - arm 看到 → 内 RF SUCCESS → 跳 refine → home
    - arm 未看到 → 走 refine_step(ForceSuccess 包裹,FAILURE 也转 SUCCESS)
    - refine_step FAILURE → ForceSuccess 转 SUCCESS → Sequence SUCCESS → 下一轮
    - refine_step SUCCESS → Sequence SUCCESS → home → 下一轮
    - user msg → WaitForUserInput SUCCESS → 整树 SUCCESS → 终止

  ForceSuccess 装饰器是 BT.CPP v4 内置,无需额外注册。

  Pose literal 格式(SetPoseStamped):frame_id;x;y;z;qx;qy;qz;qw
  参考系:world (== base_link, world→base_link fixed joint xyz=0 rpy=0)
-->
<root BTCPP_format="4" main_tree_to_execute="HeadToArmLookat">
  <BehaviorTree ID="HeadToArmLookat">
    <ReactiveFallback name="root">
      <WaitForUserInput name="wait_user_input" topic_name="/bt_user_input"/>

      <Repeat name="auto_loop" num_cycles="-1">
        <Sequence name="refine_then_home">
          <ReactiveFallback name="arm_seen_or_refine">
            <IsArmDetectionConfirmed name="check_arm"
              topic_name="/perception/two_stage/camera_arm/result"
              max_age_sec="0.3"
              last_observation="{last_arm_obs}"/>

            <ForceSuccess name="refine_step_wrapper">
              <Sequence name="refine_step">
                <IsHeadDetectionConfirmed name="check_head"
                  topic_name="/perception/two_stage/camera_head/result"
                  max_age_sec="0.3"
                  last_observation="{last_head_obs}"/>

                <ComputeLookatPose name="compute_lookat"
                  head_observation="{last_head_obs}"
                  ee_position_world="0.2;0.0;0.3"
                  world_frame="world"
                  output_pose_key="lookat_pose"/>

                <MoveArmToPose name="move_to_lookat" goal="{lookat_pose}"/>
              </Sequence>
            </ForceSuccess>
          </ReactiveFallback>

          <MoveArmToNamed name="return_home" named_target="home"
            named_targets="{@named_targets}"/>
        </Sequence>
      </Repeat>
    </ReactiveFallback>
  </BehaviorTree>
</root>
```

### Step 2: 验证 install 路径(launch glob 会自动发现)

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "
  export LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash && \
  source /my_ws/l4t/l4t_setup.sh && \
  colcon build --packages-select lgw_bt_planner 2>&1 | tail -5 && \
  ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/
"
```

Expected: 看到 `head_to_arm_lookat.xml` 在列出的 XML 列表中。

### Step 3: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add src/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml
git commit -m "$(cat <<'EOF'
lgw: wrap refine_step in ForceSuccess (Task 7 review fix)

First commit (a757a48) had a BT semantics bug: when refine_step failed
(e.g., no head detection), the outer Sequence refine_then_home would
FAILURE → Repeat FAILURE (no loop) → outer RF FAILURE → BT terminates.
This contradicts spec §6.1 which promises refine FAILURE → home → next
cycle.

Fix: wrap <Sequence name="refine_step"> in BT.CPP v4's <ForceSuccess>
decorator. ForceSuccess maps any child status (FAILURE/SKIP/RUNNING)
to SUCCESS, so refine_then_home proceeds to return_home regardless
of refine outcome.

ForceSuccess is built into BT.CPP v4, no extra plugin registration
needed.

TreeID = HeadToArmLookat. Send via:
  ros2 action send_goal /lgw_bt_execution \
    btcpp_ros2_interfaces/action/ExecuteTree \
    "{target_tree: HeadToArmLookat}"
EOF
)"
```

---

## Task 8: 启动脚本 `start_lgw_head_to_arm_lookat.sh`

**Files:**
- Create: `scripts/start_lgw_head_to_arm_lookat.sh`(workspace 级,不在 lgw_bt_planner 包内)

**目的:** 复刻 `scripts/start_lgw_user_gated_auto_loop.sh`(2026-10-01 已用)的 bringup + cleanup 约定,只换 BT_XML 默认 + log 路径 + TreeID。**不**复制整套 L2 wrapper(`/pose_replay.sh` 的 bringup 顺序、cleanup_on_signal、LG_STACK_PIDS_PATTERN)。

**为什么独立成 task:** 启动脚本与 BT 节点完全独立,reviewer 可以单独 review shell 脚本的清理逻辑(CLAUDE.md:禁 pkill,必须 PID 阶梯)。

### Step 1: 创建脚本

新建 `scripts/start_lgw_head_to_arm_lookat.sh`:

```bash
#!/bin/bash
# scripts/start_lgw_head_to_arm_lookat.sh
#
# 一键启动 HeadToArmLookat BT(ReactiveFallback + Repeat num_cycles="-1"
# 视觉伺服回路)。head 看到目标 → ComputeLookatPose 算末端 lookat →
# MoveArmToPose 移动 → arm 相机确认 → 回 home。无限循环,直到 user
# msg halt。
#
# 这是 scripts/start_lgw_user_gated_auto_loop.sh 的 fork 专用变种
# (只换 BT_XML / log 路径 / TreeID,其他保持对齐 —— 同一份 cleanup /
# bringup 约定)。

set -u
CONTAINER="self_l4t"

readonly WS_FORK_HOST="/home/unitree/my_ws/l4t/lg_walker_ws"
readonly WS_FORK_CT="/my_ws/l4t/lg_walker_ws"

# 与 start_lgw_user_gated_auto_loop.sh 完全一致的清理/grep pattern
# (CLAUDE.md 反模式:pkill / killall / kill -1 全禁用)
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller'

# BT_XML 默认指向新树(TreeID=HeadToArmLookat)
readonly DEFAULT_BT_XML="${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml"
BT_XML="${BT_XML:-${DEFAULT_BT_XML}}"
readonly TREE_ID="HeadToArmLookat"

# ---- Ctrl-C 自动清理(沿用 user_gated 脚本约定) ----
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[cleanup] Ctrl-C / 信号触发,清理 launch 进程..."
  if [ -f /tmp/lgw_launch_pids.txt ]; then
    while read -r pid; do
      [ -z "${pid}" ] && continue
      if kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] TERM pid=${pid}"
        kill -15 "${pid}" 2>/dev/null || true
      fi
    done < /tmp/lgw_launch_pids.txt
    sleep 3
    while read -r pid; do
      [ -z "${pid}" ] && continue
      if kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] KILL pid=${pid}"
        kill -9 "${pid}" 2>/dev/null || true
      fi
    done < /tmp/lgw_launch_pids.txt
    rm -f /tmp/lgw_launch_pids.txt
  fi
  echo "[cleanup] done"
}
trap cleanup_on_signal INT TERM

# ---- 启动前检查 ----
docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

# CAN + hardware driver + MoveIt + bt_runner L2 wrapper
# (与 start_lgw_user_gated_auto_loop.sh L2 顺序一致;真机前置:can0 up、
# 机械臂 + 夹爪上电、Go2 driver 已起、YOLOE detection workflow 已起、
# 双 realsense 相机 stream 已起)
echo "[bringup] L2 wrapper (CAN → agx_arm_ctrl → MoveIt → bt_runner) ..."
: "${LGW_INSTALL:?LGW_INSTALL must be exported; see l4t_setup.sh:20}"
export LGW_INSTALL
echo "[bringup] LGW_INSTALL=${LGW_INSTALL}"

mkdir -p "${WS_FORK_HOST}/.run"
LOG_TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${WS_FORK_HOST}/.run/lgw_head_to_arm_lookat_${LOG_TS}.log"
echo "[bringup] log → ${LOG_FILE}"

docker exec -e LGW_INSTALL="${LGW_INSTALL}" \
  -w "${WS_FORK_CT}" "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    bash src/agx_arm_ros/scripts/can_activate.sh can0 1000000 || true
    source src/agx_arm_ros/install/setup.bash
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
      effector_type:=agx_gripper tcp_offset:='[0.0,0.0,0.12,0.0,0.0,0.0]' &
    sleep 3
    ros2 launch agx_arm_moveit demo.launch.py &
    sleep 5
    source install/setup.bash
    ros2 launch lgw_bt_planner lgw_bt_planner.launch.py
  " >"${LOG_FILE}" 2>&1 &
LGW_LAUNCH_PID=$!
echo "${LGW_LAUNCH_PID}" > /tmp/lgw_launch_pids.txt

echo "[bringup] launch pid=${LGW_LAUNCH_PID},等 30s 让 /lgw_bt_exaction 就绪..."
sleep 30

# TreeExecutionServer 的 action server 名(spec §2.2 launch: action_name)
echo "[bringup] ready. TreeID=${TREE_ID}. send_goal:"
echo "  ros2 action send_goal /lgw_bt_execution \\"
echo "    btcpp_ros2_interfaces/action/ExecuteTree \\"
echo "    '{\"target_tree\": \"${TREE_ID}\"}' --feedback"
echo "[bringup] halt: ros2 topic pub --once /bt_user_input std_msgs/Bool 'data: true'"

wait "${LGW_LAUNCH_PID}"
```

### Step 2: chmod +x

```bash
chmod +x /home/unitree/my_ws/l4t/lg_walker_ws/scripts/start_lgw_head_to_arm_lookat.sh
```

### Step 3: shellcheck(如果可用)

```bash
which shellcheck >/dev/null && \
  shellcheck /home/unitree/my_ws/l4t/lg_walker_ws/scripts/start_lgw_head_to_arm_lookat.sh \
  || echo "shellcheck not installed, skip"
```

Expected: 无 error 或 skip 信息。

### Step 4: commit

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git add scripts/start_lgw_head_to_arm_lookat.sh
git commit -m "$(cat <<'EOF'
lgw: add start_lgw_head_to_arm_lookat.sh bringup script

Fork of start_lgw_user_gated_auto_loop.sh: same L2 bringup order
(CAN → agx_arm_ctrl → MoveIt → bt_runner), same PID-ladder
cleanup_on_signal (TERM → 3s → KILL, no pkill/killall/kill -1),
different BT_XML default (head_to_arm_lookat.xml) and TreeID
(HeadToArmLookat).

Usage:
  ./scripts/start_lgw_head_to_arm_lookat.sh
EOF
)"
```

---

## Task 9: 真机集成测试(验证清单,无代码改动)

**目的:** 按 spec §8.2 跑完整真机流程,逐项核对验收标准。无 commit。

**前置**(spec §8.2 / §9 #8):
- Go2 driver + 机械臂硬件 + head+arm realsense + YOLOE workflow 全部 up
- 周围 1.0m 范围无障碍物,操作员不离开控制台
- E-stop 按钮随手可触

### Step 1: 启动 bringup

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
./scripts/start_lgw_head_to_arm_lookat.sh
```

Expected: log 显示 launch 就绪 + TreeID=HeadToArmLookat 提示。

### Step 2: 等 action server 就绪

```bash
docker exec -e LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash \
  -it self_l4t bash -lc '
    source /my_ws/l4t/l4t_setup.sh
    ros2 action list | grep lgw_bt_execution
  '
```

Expected: 看到 `/lgw_bt_execution` action server 在 list 里。

### Step 3: 物理目标放置 + 启动 YOLOE detection

把实物(print marker / 标定板 / 醒目标物体)放在机械臂工作空间内 ~30cm,让 head 相机能看到。启动 YOLOE detection(按现有流程)。

Expected: `/perception/two_stage/camera_head/result` 周期发 CONF + detections。

### Step 4: send_goal 触发 BT

```bash
docker exec -e LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash \
  -it self_l4t bash -lc '
    source /my_ws/l4t/l4t_setup.sh
    ros2 action send_goal /lgw_bt_execution \
      btcpp_ros2_interfaces/action/ExecuteTree \
      "{target_tree: HeadToArmLookat}" --feedback
  '
```

Expected: action 收到,BT 树 RUNNING(等 WaitForUserInput FAIL,Repeat 进入循环)。

### Step 5: Groot2 / log 观察 BT 状态机

用 Groot2(可选)或 `tail -f .run/lgw_head_to_arm_lookat_*.log` 观察:
- IsHeadDetectionConfirmed SUCCESS
- ComputeLookatPose SUCCESS(`BT_PLANNER_DEBUG=1` 会打印 P_ee / P_target / P_tcp)
- MoveArmToPose RUNNING → 机械臂按目标位姿移动
- 到位后 arm 相机视野出现目标 → IsArmDetectionConfirmed SUCCESS
- 下轮 Sequence 跳过 refine → MoveArmToNamed home → 下一轮 idle

验证:`P_tcp` 与 `P_ee_in_base` 几何关系正确(差 < 10cm),机械臂实际末端位姿符合预期。

### Step 6: halt 路径测试

另一终端:

```bash
docker exec -e LGW_INSTALL=/my_ws/l4t/lg_walker_ws/install/setup.bash \
  -it self_l4t bash -lc '
    source /my_ws/l4t/l4t_setup.sh
    ros2 topic pub --once /bt_user_input std_msgs/Bool "data: true"
  '
```

Expected: WaitForUserInput SUCCESS → ReactiveFallback 整树 SUCCESS → Repeat halt → 机械臂当前帧停。`send_goal` 终端收到 SUCCESS feedback。

### Step 7: FAILURE 路径测试

用布盖住 head 相机(或关闭 YOLOE detection 让 head 检测不到目标)。

Expected: IsHeadDetectionConfirmed FAILURE → Sequence FAILURE → ReactiveFallback SUCCESS(继续) → MoveArmToNamed home → 下一轮重试。**树不崩**,Repeat 继续循环。

### Step 8: 回归测试 5 棵现有 demo XML

```bash
# 启动 start_lgw_user_gated_auto_loop.sh 跑 pick_and_place_demo
# 启动 start_lgw_pose_replay.sh 跑 pose_via_set_pose_stamped
# 手动 send_goal 跑 detection_condition_test, go_to_target_and_back, user_gated_auto_pick_and_place
```

Expected: 5 棵现有 demo 都能跑通(没有意外回归)。

### Step 9: cleanup

在启动脚本所在终端按 Ctrl-C。trap 自动按 PID 阶梯清理(TERM → 3s → KILL)。

Expected: 所有 launch 子进程清理,无 `pkill` / `killall` / `kill -1`。

### Step 10: 验收清单(全部勾选才视为完成)

- [ ] `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON` 无 warning
- [ ] `build/lgw_bt_planner/test_compute_lookat_pose` 11/11 PASS
- [ ] `colcon build --packages-select lgw_bt_planner` 无 warning
- [ ] `nm -D liblgw_bt_planner_nodes.so` 看到 `ComputeLookatPose` 注册符号
- [ ] `ls install/lgw_bt_planner/share/lgw_bt_planner/calibration/{head,arm}_calibration.json` 存在
- [ ] `ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml` 存在
- [ ] 真机 Step 1-9 全过
- [ ] 现有 5 棵 demo XML 回归测试无回归
- [ ] cleanup PID 阶梯正常工作

---

## 执行选项

plan 完成,已保存到 `docs/superpowers/plans/2026-10-01-head-to-arm-lookat.md`。两种执行选项:

1. **Subagent-Driven (推荐)** - 每个 Task 派一个新 subagent,task 间做两阶段 review,迭代快
2. **Inline Execution** - 当前会话连续执行,用 executing-plans,批量执行 + 检查点
