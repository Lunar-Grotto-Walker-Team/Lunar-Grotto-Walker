# 固定姿态族 lookat 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用一个"钉死 j3/j5、联立解 j1/j4/j6、外层枚举 j2"的纯函数模块替换 lookat 的唯一
期望位姿来源,并在 mock 链路上与 MoveIt Servo 联调,量化实时性、跟踪速度与奇异行为
(**不驱动真机**)。

**Architecture:** 新增 `lookat_posture_family.{hpp,cpp}`(KDL 链 + FK + 投影牛顿 +
候选枚举)。`servo_node.cpp` 增加 `lookat_mode: pinned_posture` 分支,只替换"期望 tcp
位姿从哪来"这一环;控制律、状态机、Servo 参数、输出接口全部不动。

**Tech Stack:** C++17、rclcpp、kdl_parser/orocos_kdl、tf2、ament_cmake_gtest。

**Spec:** `docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md`(rev2)

## Global Constraints

- 宿主机编辑,容器内构建:`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select lgw_bt_planner"`
- 单元测试不用 `colcon test`(容器缺 `ament_cmake_test`),直接跑 build 下的 gtest 二进制。
- **不驱动真机**:任何 bench 运行前必须确认 `agx_arm_ctrl_single` 没有在跑;清理只能按 PID `kill -15`,禁止 `pkill -f ros`。
- 姿态参数(来自 spec §4):`j3 = −0.484`、`j5 = +0.600`、`j2 ∈ [0.100, 1.571]`、`j2_nominal = 0.890`、`cam_min_height = 0.200`、`cam_max_radius = 0.350`、`joint_margin = 0.100`、`cond_max = 50.0`、`aim_tol = 0.035`、`roll_weight = 0.25`。
- Piper 关节限位(URDF):j1 ±2.6179938、j2 [0, π]、j3 [−2.9670597, 0]、j4 ±1.7453292、j5 ±1.2217304、j6 ±2.0943951。
- 参考实现:`scripts/lookat_posture_proto.py`(Python 原型),C++ 行为必须与它一致。
- 提交信息用中文描述 + `Co-Authored-By: Claude Code <noreply@anthropic.com>`。

## Review Focus

- **除零/退化**:目标落在基座 z 轴上或与相机重合时,链条里任何 `normalize` 都可能产出 NaN —— 期望是抛异常让节点 HOLD,不是输出 NaN 位姿。
- **求解超时**:单周期解算超过 10 ms 会拖住 100 Hz 控制律 —— 期望是沿用上一拍结果并置 `kOverrun`,不是阻塞。
- **守卫顺序**:相机高度/半径守卫与"退回上一拍"冲突时,守卫必须优先 —— 期望永不输出违反守卫的位姿。
- **frame 一致性**:模块内部的 tcp 必须与 TF 里 `arm1/tcp_link` 是同一帧 —— 期望 FK 结果与 `tcp_now` 在稳态下差 < 2 mm。
- **j2 边界滞环**:目标在候选边界附近抖动时 j2 不得来回跳 —— 期望相邻两拍 |Δj2| ≤ 0.15。

---

### Task 1: KDL 链 + FK

**Files:**
- Create: `include/lgw_bt_planner/lookat_posture_family.hpp`
- Create: `src/lookat_posture_family.cpp`
- Modify: `CMakeLists.txt`(加入 NODE_SOURCES / servo_node 目标;新增 `kdl_parser`、`orocos_kdl`)
- Modify: `package.xml`(同上)
- Test: `test/test_lookat_posture_family.cpp`

**Interfaces:**
- Produces: `struct PostureParams`、`struct PostureResult`、`class PostureFamily`(定义见下)

- [ ] **Step 1: 写失败的测试**

```cpp
// test/test_lookat_posture_family.cpp
#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lgw_bt_planner/lookat_posture_family.hpp"

using lgw_bt_planner::PostureFamily;
using lgw_bt_planner::PostureParams;

static std::string piperUrdf() {
  return ament_index_cpp::get_package_share_directory("agx_arm_description") +
         "/agx_arm_urdf/piper/urdf/piper_description.urdf";
}

TEST(PostureFamily, FkMatchesPrototype) {
  PostureFamily fam;
  std::string err;
  // 测试用 link6 作末端,把 tcp_offset(0,0,0.12) 并进相机偏置
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
                             "base_link", "link6", err)) << err;
  const std::array<double, 6> q{-1.4064, 0.8898, -0.4841, 1.0463, 1.2338, -1.7653};
  tf2::Matrix3x3 R_cam;  R_cam.setIdentity();
  const tf2::Vector3 t_cam(0.0, 0.0, 0.12);   // link6 -> tcp (测试专用)
  tf2::Vector3 p_cam; tf2::Matrix3x3 R_out;
  ASSERT_TRUE(fam.cameraPose(q, R_cam, t_cam, p_cam, R_out, err)) << err;
  // 原型 scripts/ 实测值
  EXPECT_NEAR(p_cam.x(), 0.162, 2e-3);
  EXPECT_NEAR(p_cam.y(), -0.099, 2e-3);
  EXPECT_NEAR(p_cam.z(), 0.303, 2e-3);
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select lgw_bt_planner && ./build/lgw_bt_planner/test_lookat_posture_family"`
Expected: 编译失败(`lookat_posture_family.hpp` 不存在)

- [ ] **Step 3: 实现头文件与 FK**

```cpp
// include/lgw_bt_planner/lookat_posture_family.hpp
#ifndef LGW_BT_PLANNER__LOOKAT_POSTURE_FAMILY_HPP_
#define LGW_BT_PLANNER__LOOKAT_POSTURE_FAMILY_HPP_

#include <array>
#include <string>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace lgw_bt_planner
{

struct PostureParams
{
  double posture_j3_rad{-0.484};
  double posture_j5_rad{0.600};
  double posture_j2_nominal_rad{0.890};
  double posture_j2_min_rad{0.100};
  double posture_j2_max_rad{1.571};
  double cam_min_height_m{0.200};
  double cam_max_radius_m{0.350};
  double joint_margin{0.100};
  double cond_max{50.0};
  double aim_tol_rad{0.035};
  double roll_weight{0.25};
};

struct PostureResult
{
  bool ok{false};                 // 解算成功且通过全部守卫
  bool out_of_envelope{false};    // 目标超出包络,用的是最接近的可行姿态
  bool guarded{false};            // 有候选被守卫夹住
  Pose6 tcp_desired;              // 期望 tcp 位姿
  std::array<double, 6> joints{}; // 对应的关节角
  double aim_error_rad{0.0};
  double cond{0.0};
  std::string status;             // "ok" / "out_of_envelope" / "overrun" / "degenerate"
};

class PostureFamily
{
public:
  bool initialize(const std::string & urdf_path_or_xml, bool is_file,
                  const std::string & base_link, const std::string & tip_link,
                  std::string & error);

  bool cameraPose(const std::array<double, 6> & joints,
                  const tf2::Matrix3x3 & R_cam_in_tip,
                  const tf2::Vector3 & t_cam_in_tip,
                  tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
                  std::string & error) const;
  ...
};
```

FK 用 `kdl_parser::treeFromFile` / `treeFromString` + `tree.getChain(base, tip, chain)` 建链,
`KDL::ChainFkSolverPos_recursive` 求位姿,`KDL::ChainJntToJacSolver` 求 6×6 雅可比。

- [ ] **Step 4: 运行测试确认通过**

Run: 同 Step 2
Expected: `[  PASSED  ] 1 test.`

- [ ] **Step 5: 提交**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/lookat_posture_family.hpp \
        src/lgw_bt_planner/src/lookat_posture_family.cpp \
        src/lgw_bt_planner/test/test_lookat_posture_family.cpp \
        src/lgw_bt_planner/CMakeLists.txt src/lgw_bt_planner/package.xml
git commit -m "feat(lgw): posture family KDL chain and FK

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 2: 误差向量 + 投影牛顿(j1/j4/j6)

**Files:**
- Modify: `src/lookat_posture_family.cpp`
- Test: `test/test_lookat_posture_family.cpp`

**Interfaces:**
- Consumes: `PostureFamily::cameraPose()`
- Produces: `bool solveFree(const tf2::Vector3& p_target, const std::array<double,6>& seed, const PostureParams&, std::array<double,6>& q_out, double& aim_error, double& cond, std::string& error) const`

- [ ] **Step 1: 写失败的测试**

```cpp
TEST(PostureFamily, AimsAtGroundTarget) {
  // j3/j5 钉死,j2 = 0.96(原型解),只解 j1/j4/j6
  PostureFamily fam; std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
                             "base_link", "link6", err));
  PostureParams p;
  tf2::Matrix3x3 R_cam; R_cam.setIdentity();
  const tf2::Vector3 t_cam(0.0, 0.0, 0.12);
  const tf2::Vector3 target(0.75, 0.0, -0.37);
  std::array<double, 6> seed{0.0, 0.96, p.posture_j3_rad, 0.0, p.posture_j5_rad, 0.0};
  std::array<double, 6> q{}; double aim = 0.0, cond = 0.0;
  ASSERT_TRUE(fam.solveFree(target, R_cam, t_cam, seed, p, q, aim, cond, err)) << err;
  EXPECT_LT(aim, 0.02);                        // < 1.15 deg
  EXPECT_EQ(q[2], p.posture_j3_rad);           // j3 不动
  EXPECT_EQ(q[4], p.posture_j5_rad);           // j5 不动
  EXPECT_LE(cond, p.cond_max);
}
```

`solveFree` 的完整签名:

```cpp
bool solveFree(const tf2::Vector3 & p_target,
               const tf2::Matrix3x3 & R_cam_in_tip,
               const tf2::Vector3 & t_cam_in_tip,
               const std::array<double, 6> & seed,
               const PostureParams & params,
               std::array<double, 6> & q_out,
               double & aim_error_rad,
               double & cond,
               std::string & error) const;
```

- [ ] **Step 2: 运行测试确认失败**

Run: 同 Task 1 Step 2
Expected: 编译失败(`solveFree` 不存在)

- [ ] **Step 3: 实现**

```cpp
// 误差向量:solveFree 内部
// u = normalize(target - cam);  x_des = normalize(z_world x u)
// e_axis = cross(cam_axis, u)  (3 维)
// e_roll = cam_x . (u x x_des) (1 维)
// e = [e_axis; roll_weight * e_roll]
// J = 有限差分 (3 次额外 FK),dq = -pinv(J) e,单步截断 0.2 rad,
// q <- clamp(限位 ± joint_margin),最多 8 次迭代,|e_axis| < 1e-5 提前退出
// cond = sigma_max(J6) / sigma_min(J6),J6 为 6x6 全雅可比
```

- [ ] **Step 4: 运行测试确认通过**

- [ ] **Step 5: 提交**(`feat(lgw): posture family damped-least-squares aim`)

---

### Task 3: j2 候选枚举 + 守卫

**Files:**
- Modify: `src/lookat_posture_family.cpp`
- Test: `test/test_lookat_posture_family.cpp`

**Interfaces:**
- Produces: `PostureResult compute(const tf2::Vector3& p_target, const tf2::Matrix3x3& R_cam_in_tip, const tf2::Vector3& t_cam_in_tip, const PostureParams&)`

- [ ] **Step 1: 写失败的测试**

```cpp
static PostureResult solveAt(const tf2::Vector3& t, PostureParams p = {}) {
  static PostureFamily fam = [] { PostureFamily f; std::string e;
    f.initialize(piperUrdf(), /*is_file=*/true, "base_link", "link6", e);
    return f; }();
  tf2::Matrix3x3 I; I.setIdentity();
  return fam.compute(t, I, tf2::Vector3(0, 0, 0.12), p);
}

TEST(PostureFamily, GroundTargetsAreCovered) {
  for (double d : {0.40, 0.75, 1.20}) {
    const auto r = solveAt(tf2::Vector3(d, 0.0, -0.37));
    EXPECT_TRUE(r.ok) << d;
    EXPECT_LT(r.aim_error_rad, 0.035) << d;
  }
}

TEST(PostureFamily, GuardsAlwaysHold) {
  tf2::Matrix3x3 I; I.setIdentity();
  for (double az = -2.618; az <= 2.618; az += 0.10) {
    for (double d : {0.40, 0.75, 1.20, 2.0}) {
      tf2::Vector3 t(d * std::cos(az), d * std::sin(az), -0.37);
      const auto r = solveAt(t);
      tf2::Vector3 p; tf2::Matrix3x3 R;
      // 用返回的 tcp 位姿回算相机高度
      p = r.tcp_desired.p + r.tcp_desired.R * tf2::Vector3(0, 0, 0.12);
      EXPECT_GE(p.z(), 0.20 - 1e-6);
      EXPECT_LE(std::hypot(p.x(), p.y()), 0.350 + 1e-6);
      EXPECT_LE(r.cond, 50.0);
      for (int j = 0; j < 6; ++j) { /* 距限位 >= 0.10 */ }
    }
  }
}

TEST(PostureFamily, WallTargetIsOutOfEnvelope) {
  const auto r = solveAt(tf2::Vector3(0.8, 0.0, 1.23));   // 0.8m 外 1.6m 高
  EXPECT_TRUE(r.ok == false || r.aim_error_rad > 0.035);
}

TEST(PostureFamily, GuardBeatsOutOfEnvelope) {
  // 目标高到所有候选都过不了守卫:结果必须仍满足守卫(退回上一拍),
  // 而不是输出一个"最接近但越界"的姿态
  solveAt(tf2::Vector3(0.75, 0.0, -0.37));          // 先建立一个合法上一拍
  const auto r = solveAt(tf2::Vector3(0.0, 0.0, 3.0));   // 正上方 3 m
  const tf2::Vector3 cam = r.tcp_desired.p + r.tcp_desired.R * tf2::Vector3(0, 0, 0.12);
  EXPECT_GE(cam.z(), 0.20 - 1e-6);
  EXPECT_LE(std::hypot(cam.x(), cam.y()), 0.350 + 1e-6);
  EXPECT_EQ(r.status, "overrun");
}
```

- [ ] **Step 2: 运行测试确认失败**

Expected: `compute` 不存在 / 守卫断言失败

- [ ] **Step 3: 实现候选顺序与守卫**

```
候选 = [上一拍 j2, nominal, prev±0.15, prev±0.30, j2_min, j2_max]
       去重、夹到 [j2_min, j2_max]
对每个候选跑 solveFree;第一个 aim<=aim_tol && cond<=cond_max &&
相机高度>=cam_min && 半径<=cam_max_radius 的即为结果
全部失败 -> 取 aim 最小者,status="out_of_envelope",但守卫仍需满足;
仍不满足守卫 -> 返回上一拍的合法结果,status="overrun"
```

- [ ] **Step 4: 运行测试确认通过**

- [ ] **Step 5: 提交**(`feat(lgw): posture family j2 candidate search and guards`)

---

### Task 4: 连续性 / 退化 / 限位

**Files:**
- Modify: `src/lookat_posture_family.cpp`
- Test: `test/test_lookat_posture_family.cpp`

- [ ] **Step 1: 写失败的测试**

```cpp
TEST(PostureFamily, ContinuousAcrossBearing) {
  tf2::Vector3 prev_p; std::array<double, 6> prev_q{}; bool first = true;
  for (double az = -2.44; az <= 2.44; az += 0.0175) {      // ±140 deg
    const auto r = solveAt(tf2::Vector3(0.75 * std::cos(az), 0.75 * std::sin(az), -0.37));
    ASSERT_TRUE(r.ok) << az;
    if (!first) {
      EXPECT_LT((r.tcp_desired.p - prev_p).length(), 0.005) << az;   // < 5 mm
      EXPECT_LE(std::abs(r.joints[1] - prev_q[1]), 0.15) << az;      // j2 不跳
    }
    prev_p = r.tcp_desired.p; prev_q = r.joints; first = false;
  }
}

TEST(PostureFamily, DegenerateTargetThrows) {
  EXPECT_THROW(solveAt(tf2::Vector3(0.0, 0.0, -0.37)), std::runtime_error);
}

TEST(PostureFamily, JointLimitsRespected) {
  // 复用 GuardsAlwaysHold 的扫描,断言每个关节距限位 >= 0.10
}
```

- [ ] **Step 2: 运行测试确认失败**
- [ ] **Step 3: 实现**(退化判定 `r_xy < 1e-3` 或 `|target−cam| < 1e-6` 抛 `std::runtime_error`)
- [ ] **Step 4: 运行测试确认通过**
- [ ] **Step 5: 提交**(`feat(lgw): posture family continuity and degenerate handling`)

---

### Task 5: 接入 servo_node + launch

**Files:**
- Modify: `src/servo_node.cpp`(参数声明 + `pinned_posture` 分支 + 计时日志)
- Modify: `launch/lgw_full.launch.py`(给 `lgw_servo_node` 传 `robot_description`)
- Modify: `config/servo_node.yaml`(§4 参数)
- Test: `test/test_lookat_posture_family.cpp`(补充"tcp 与 TF 一致"不在此任务,改由 bench 验)

**Interfaces:**
- Consumes: `PostureFamily::initialize/2`、`compute/4`
- Produces: `lookat_mode: pinned_posture` 可运行路径

- [ ] **Step 1: 写失败的测试**

```cpp
TEST(PostureFamily, NodeDefaultsMatchSpec) {
  PostureParams p;
  EXPECT_DOUBLE_EQ(p.posture_j3_rad, -0.484);
  EXPECT_DOUBLE_EQ(p.posture_j5_rad, 0.600);
  EXPECT_DOUBLE_EQ(p.posture_j2_min_rad, 0.100);
  EXPECT_DOUBLE_EQ(p.posture_j2_max_rad, 1.571);
  EXPECT_DOUBLE_EQ(p.cam_min_height_m, 0.200);
}
```

- [ ] **Step 2: 运行测试确认失败**(字段名不存在前会编译失败)

- [ ] **Step 3: 实现接线**

```
servo_node.cpp:
  declare_parameter("robot_description", "")       // launch 注入
  declare_parameter("lookat_mode", "legacy")       // 增加 "pinned_posture"
  构造时如果 mode == pinned_posture:
      posture_.initialize(robot_description, /*is_file=*/false,
                          "base_link", "tcp_link", err)
  控制 tick 中:
      if (pinned_posture) { desired_tcp = posture_.compute(...).tcp_desired; }
  计时:steady_clock 包住 compute,每 5 s 打一条
      "[posture] solve mean %.2f ms p99 %.2f ms n=%u" 的节流日志

lgw_full.launch.py:
  lgw_servo_node parameters 增加 moveit_config.robot_description
```

- [ ] **Step 4: 构建 + 运行全部 gtest**

Run: `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select lgw_bt_planner && ./build/lgw_bt_planner/test_lookat_posture_family"`
Expected: 全部 PASSED

- [ ] **Step 5: 提交**(`feat(lgw): wire pinned_posture lookat mode into servo node`)

---

### Task 6: mock 链路联调(实时性 / 跟踪 / 奇异)

**Files:**
- Create: `scripts/bench_posture_lookat.sh`
- Create: `scripts/measure_posture_bench.py`
- Create: `docs/servo/2026-10-31-posture-lookat-bench.md`(结果记录)

- [ ] **Step 1: 确认没在驱动真机**

```bash
docker exec self_l4t bash -c "ps aux | grep -c '[a]gx_arm_ctrl_single'"
```
Expected: `0`。不为 0 则按 PID `kill -15` 逐个停掉,再复查。

- [ ] **Step 2: 写 bench 脚本**

`bench_posture_lookat.sh`:以 `follow:=false use_bt:=false` 启动 `lgw_full.launch.py`,
传入 `lookat_mode:=pinned_posture`,等 `/control/joint_states` 有数据后启动
`fake_track_publisher.py --sweep-amp 0.15 --sweep-hz 0.2`(模拟平台 0.19 m/s 走动),
跑 60 s 后清理(按 PID)。

- [ ] **Step 3: 写测量脚本**

`measure_posture_bench.py` 同步记录:`/lgw/servo/state`(state/e_ang/e_pos)、
`/servo_node/status`、`/control/joint_states`;输出:

```
解算耗时      mean / p99(来自节点节流日志,脚本 grep 解析)
跟踪          e_ang 稳态均值 / 峰值;mock 关节最大速度
奇异          status != 0 的时长;模块日志里的 cond 最大值
守卫          相机高度/半径最小最大值(由关节角 FK 回算)
帧一致        稳态下 FK 算出的 tcp 与 TF 的 arm1/tcp_link 差(mm),期望 < 2
```

- [ ] **Step 4: 跑 bench 并记录**

验收阈值(spec §7):

| 指标 | 阈值 |
|---|---|
| 解算 p99 | < 3 ms |
| `servo_node/status != 0` 累计时长 | 0 s |
| e_ang 稳态均值 | < 0.05 rad |
| 任一关节距限位 | ≥ 0.10 rad |
| 相机高度最小值 | ≥ 0.20 m |

- [ ] **Step 5: 写结果文档并提交**

`docs/servo/2026-10-31-posture-lookat-bench.md` 记录实测表格 + 与原型预测的差异。

---

## 执行顺序与依赖

Task 1 → 2 → 3 → 4 严格串行(同一文件、接口递进)。Task 5 依赖 1–3。Task 6 依赖 5。

## 不在本计划内

- `preposition_lookat.sh`(spec §3.8):不驱动真机时无意义,留到真机阶段。
- 离线碰撞验证(spec §7.2):需要完整 planning scene,留到真机阶段。
- 墙面覆盖扩展:spec §9 待定项。
