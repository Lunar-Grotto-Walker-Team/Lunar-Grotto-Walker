# Self-Selecting Arm Observation Pose — Design

**Status**: Draft (待用户评审)
**Date**: 2026-10-09
**Author**: Claude (design session with user; follow-up to head/arm 链路标定 bug fix 2026-10-09)
**Workspace**: 修改 `l4t/lg_walker_ws/`
**Goal**: 把 `head_arm_independent_lookat.xml` 里 arm 相机的 lookat 锚点从**硬编码** (`ee_position_world="0.4;0.0;0.3"`) 改成**目标感知** 的自适应选择(fixed yaw + 自适应俯角 + 自适应距离);按**方案 β** 拆成 3 层 — 纯几何/评分模块 + 独立 ROS service + BT 薄壳(RosServiceNode)。

---

## 1. Background

### 1.1 现状

`head_arm_independent_lookat.xml:100` 的 step 1 当前 anchor 写死:

```xml
<ComputeLookatPose name="compute_lookat_pose"
  head_observation="{last_head_obs}"
  ee_position_world="0.4;0.0;0.3"
  world_frame="world"
  output_pose_key="lookat_pose"/>
```

XML header 注释(l.21-26)指出该硬编码的来历:

> anchor = head_obs.target_world_xyz 即 `0.4;0.0;0.3`;`0.5;0.0;0.4` 距离 0.58m + 接近垂直向下 → near-singular OMPL 15s 采不到 valid state;退到 `0.4;0.0;0.3` 距 base ~0.42m。

**3 个被动后果:**

1. **目标在世界系下位置变了 anchor 仍是固定值** → 离目标远的 lookat pose 仍可能视野不准。
2. **目标在桌沿或墙根** → 硬编码 anchor 距离 base 0.42m 但目标在 anchor 对侧,arm 越过 target 反而看不到。
3. **目标很高(立面上)** → 硬编码 0.3m z + 0.4m xy 让 arm 处于中俯,看不到侧面特征 → YOLOE 识别 conf 下降。

### 1.2 用户需求

> "我希望 arm 相机沿机械臂基坐标系到目标方向看(yaw 固定),但与 Z 轴的夹角(俯角 α)可以自适应决定,如 45°、60°、90°;距离 = 0.7 × 头相机到目标距离,达不到就提高比例。"

提炼成 3 个变量:

| 维度 | 策略 | 候选 |
|---|---|---|
| 机位 yaw (xoy 方向) | **固定** = 沿 `base_link → 目标` 方向 | 唯一 |
| 俯角 α(相对水平面) | **自适应** | 90° → 60° → 45°(优先级) |
| 距离比例 r = d_arm / d_head | **0.7** 起步,失败升 | 0.7 → 0.85 → 1.0 → 1.2 |

### 1.3 为什么这是一个非平凡变化

- 不仅是替换 anchor,而是**用一个候选枚举 + IK 验证 + 评分** 算法取代单点计算。
- 需要调用 MoveIt IK 服务(`/compute_ik`),现有 `lgw_bt_planner` 没有直接 IK 调用 pattern——只有 `MoveArmToPose` 通过 `/move_action` 整条管线走。前例是兄弟工作区 `piper_bt_planner_old/` 里被精简掉的 `CheckPoseReachableServiceNode`。

### 1.4 为什么选方案 β(独立 service + BT 薄壳)

两个备选架构被讨论过:

| 架构 | 描述 | 取舍 |
|---|---|---|
| α:BT::SyncActionNode 直跑 12 IK | BT 节点内 12×50ms 阻塞 | 简单,但算力堆 BT 线程,12 IK 抖动影响 BT rhythm |
| **β:独立 service + BT 薄壳** | service 节点持有 IK client,12 IK + 评分在 service 线程;BT 节点只发 1 次 RPC | **已选** — 算力隔离、可单测、可被其他客户端(RViz / debug 脚本)复用,且新增的纯几何模块可独立 gtest |

---

## 2. Goal / Non-goals

### 2.1 Goal

1. **纯几何/评分模块**(无 ROS 依赖,放在 `lgw_bt_planner/include/`):
   `select_arm_observation_pose_geom.hpp` + `.cpp`,只接受数学输入,产出 12 候选 + 可选评分。可独立 gtest 覆盖。
2. **ROS service 节点**(独立 `rclcpp::Node`):
   提供 `/select_arm_observation_pose` 服务;内部订阅 `/joint_states` 与 `/feedback/tcp_pose` 缓存当前关节状态;持有 `rclcpp::Client<moveit_msgs::srv::GetPositionIK>` 调 `/compute_ik`;在 service handler 内循环 12 IK + 评分 + early-exit,响应。
3. **BT 薄壳节点**(`BT::RosServiceNode<lgw_interfaces::srv::SelectArmObservationPose>`):
   XML tag `SelectArmObservationPose`,`setRequest` 拷入 `{head_target_world}`,`onResponseReceived` 把 `selected_tcp_pose_in_world` 等写黑板。
4. 头部 YOLOE 链路不动,只换 step 1 的 lookat 计算节点。
5. 新节点保证 5 个真机点位误差 ≤ 8cm,IK 失败率 < 2%。
6. 复用现有 `MoveArmToPose` 节点(下游),不重写 arm 移动逻辑。
7. `ComputeLookatPose` **保留**,作为 fallback / debug 对照。
8. 提供 gtest 单元测试覆盖纯几何 + 评分,真机 ros2 bag 回放验证一致率。

### 2.2 Non-goals

1. 不引入网格之外的随机采样 / 球面采样 / Fibonacci 网格(本 spec 用确定性网格 + JointDisplacement 评分;扩展留给后续 spec)。
2. 不优化 IK 调用本身(KDL 50ms 一轮够用,不在此 spec 调 KDL 参数)。
3. 不改 `MoveArmToPose`、`ComputeHeadTargetWorld`、`ComputeArmTargetWorld` 现有逻辑。
4. 不实现"自检(observation succeeded)"——observation 是否成功由 downstream `IsArmDetectionConfirmed` 判定。
5. 不做 gripper 控制、不做多目标并发。
6. 不重写 `head_to_arm_lookat.xml` 或 `head_to_arm_verify_grasp.xml`(只在 `head_arm_independent_lookat.xml` 上落地一次)。
7. 不改 MoveIt config 包 `agx_arm_moveit`。
8. 不拆分独立 `lgw_pose_selector` 包——本 spec 内全部功能留在 `lgw_bt_planner` 包,**简化 CMake / launch**;将来若被多个包复用再做包分离。

---

## 3. Design Constraints

### 3.1 三层架构

| 层 | 类型 | ROS 依赖 | 单元测试路径 |
|---|---|---|---|
| 几何/评分纯模块 | C++ class,template 或 inline 头 | **零**(只用 `tf2` + `Eigen`) | 直接 link 头,gtest 直接调,无需起 ROS |
| Service 节点 | `rclcpp::Node` | 全套(`tf2_ros`、`rclcpp::Client` 等) | gtest 用真服务或 mock(更费事) |
| BT 薄壳 | `BT::RosServiceNode<S>` 派生 | BT.ROS2 | gtest 通过 `BT::BehaviorTreeFactory` mock |

**关键约束**:

- 纯模块必须无 ROS 依赖(连 `tf2_ros` 都不用,只用 `tf2/LinearMath/*`),否则被 `ament_cmake` 编译系统拽进 ROS include 路径,难独立测试。
- service 节点必须**在 move_group 起来后才起**,否则 `/compute_ik` 不可用——`wait_for_service(2s)` 后失败 → 节点进入 degraded 模式(每次请求直接返 success=false,等用户重启)。

### 3.2 MoveIt IK 服务

- 服务名:`/compute_ik`
- 类型:`moveit_msgs::srv::GetPositionIK`
- `group_name="arm"`(SRDF `<group name="arm"><chain base_link="base_link" tip_link="tcp_link"/>`)
- `ik_link_names={"tcp_link"}`(见 `agx_arm.urdf.xacro:28-34` `tcp_joint` 为 fixed joint,affine 偏移)
- 求解器:`kdl_kinematics_plugin/KDLKinematicsPlugin`,`kinematics_solver_timeout=0.005s`(每候选 <50ms)
- MoveIt 已在 `bt_with_lgw_moveit.launch.py:26-29, 37-52` 通过 `agx_arm_moveit/demo.launch.py` bring up,`/compute_ik` 默认暴露。

### 3.3 Frame 约定

| 量 | frame | 来源 |
|---|---|---|
| 目标世界系位置 T | `base_link`(==`world`,identity fixed joint) | 上游 `ComputeHeadTargetWorld` 输出 `head_target_world` (Point) |
| base_link 位置 B | (0,0,0) | URDF `world → base_link` identity fixed joint |
| tcp_pose 输入输出 | `base_link` | 与 `MoveArmToPose` 一致(l.135-137:frame 默认 `base_link`) |
| Yaw 方向 | `base_link → T` xoy 单位向量 | 计算 |
| 头相机到目标距离 | `d_head = \|\|center_m\|\|`(head 相机系下 3D 坐标长度) | `head_obs.detections[0].center_m`,不依赖 head pose in world |

### 3.4 真机 / 性能约束

- **BT 薄壳 tick** ≤ 100ms(p99):只是 1 次 RPC + 响应解析,不应超。
- **service handler 单次** ≤ 600ms(p99):12×50ms IK worst case,加评分 + 状态读。
- service 节点 crash → BT 端 `RosServiceNode::onFailure` 返回 FAILURE,与现状 `ComputeLookatPose` 失败语义一致(Fallback + ForceSuccess 跳过 step)。

---

## 4. Design

### 4.1 三层架构图

```
                 ┌─────────────────────────────────────────────┐
                 │  Pure C++ Geom/Score Module (NO ROS)        │
                 │  select_arm_observation_pose_geom.{hpp,cpp}  │
                 │  ────────────────────────────────────────    │
                 │  Input: P_target_world, d_head,              │
                 │         alphas_deg[], ratios[],              │
                 │         q_current[6] (or empty)              │
                 │  Output: std::vector<PoseCandidate>          │
                 │   每个 candidate = {tcp_pose, alpha, ratio,   │
                 │                    ik_solution(q or empty),  │
                 │                    displacement, score}      │
                 │  gtest-only, header-only tf2                 │
                 └─────────────────────────────────────────────┘
                                       ▲
                                       │ 调用几何/评分
                                       │
                 ┌─────────────────────────────────────────────┐
                 │  ROS Service Node  (rclcpp::Node)            │
                 │  select_arm_observation_pose_service_node    │
                 │  ────────────────────────────────────────    │
                 │  Subscribes: /joint_states(sensor_msgs),     │
                 │              /feedback/tcp_pose (opt,用于    │
                 │              displacement 评分)              │
                 │  Holds: rclcpp::Client<moveit_msgs::srv::    │
                 │         GetPositionIK> (/compute_ik)         │
                 │  Service: /select_arm_observation_pose       │
                 │   handler(req) → 调 geom → 12×IK → 评 → 返  │
                 └─────────────────────────────────────────────┘
                                       ▲
                                       │ 1 次 RPC
                                       │
                 ┌─────────────────────────────────────────────┐
                 │  BT Thin Shell (RosServiceNode)              │
                 │  SelectArmObservationPoseBTNode              │
                 │  ────────────────────────────────────────    │
                 │  Inputs (blackboard):                        │
                 │    {head_target_world: Point}                │
                 │    {current_tcp_pose_in_world: PoseStamped}  │
                 │    {current_joint_state: JointState}         │
                 │  Calls: /select_arm_observation_pose         │
                 │  Outputs (blackboard):                       │
                 │    {arm_lookat_pose: PoseStamped}            │
                 │    {selected_alpha_deg: double}              │
                 │    {selected_ratio: double}                  │
                 │    {n_reachable: int}                        │
                 └─────────────────────────────────────────────┘
```

### 4.2 自定义 Service 消息

新文件 `srv/SelectArmObservationPose.srv`:

```srv
# Service: 选最优 arm 观测位姿
# BT 节点调此服务;输入目标世界位置 + 当前状态,得选中 tcp 位姿 + 调试信息

# ---- Request ----
geometry_msgs/Point target_world        # 目标在 base_link 系下 3D 坐标
float64 d_head                            # head 相机到目标距离(m),BT 端算后传入
string preferred_ratios_csv               # "0.7,0.85,1.0,1.2"
string preferred_alphas_deg_csv           # "90,60,45"
int32 ik_call_timeout_ms                  # 单次 IK 超时,默认 50
string arm_base_frame                     # 默认 "base_link"

# ---- Response ----
bool success
geometry_msgs/PoseStamped selected_tcp_pose_in_world
float64 selected_alpha_deg
float64 selected_ratio
int32 n_reachable
int32 n_total_iterated
string message
```

**说明**:
- 当前关节状态由 **service 节点自订阅 `/joint_states`** 缓存,不通过 request 传入(2026-10-09 简化:避免 BT 黑板注入冗余)。
- d_head 由 BT 端从 `last_head_obs.detections[0].center_m` 算 L2 范数后传入。
- `selected_tcp_pose_in_world.header.frame_id = "base_link"`,与下游 `MoveArmToPose` 对齐。

### 4.3 纯几何/评分模块

```
File: include/lgw_bt_planner/select_arm_observation_pose_geom.hpp
      src/select_arm_observation_pose_geom.cpp
Class: PoseSelector (无 ROS 依赖)
依赖: tf2/LinearMath/{Vector3, Quaternion, Matrix3x3}, std::vector, std::optional
```

**类签名**:

```cpp
namespace lgw_bt_planner
{
struct PoseCandidate
{
    geometry_msgs::msg::Pose tcp_pose;          // PoseStamped.pose 的内容
    double alpha_rad;
    double ratio;
    std::vector<double> ik_q_solution;          // 6 维,IK 成功后填入
    double joint_displacement_norm;             // ||q_sol - q_current||_2
    double score;                               // 综合评分(见 §4.7)
};

class PoseSelector
{
public:
    struct Config
    {
        std::vector<double> alphas_deg;           // 默认 {90,60,45}
        std::vector<double> ratios;               // 默认 {0.7,0.85,1.0,1.2}
        std::vector<double> q_current;            // 默认空,跳过 displacement
        double weight_displacement;               // 默认 1.0
        double weight_ratio_deviation;            // 默认 2.0
        double weight_alpha_deviation_deg;        // 默认 1.0
        double joint_limit_fraction;              // 默认 0.9(软极限 = 90% × q_max_i)
        std::vector<double> q_max_per_joint;      // 默认 6 维,各关节上限
    };

    // 输入:目标世界位 + 头相机距离 + 臂基参考
    PoseSelector(geometry_msgs::msg::Point target,
                 double d_head,
                 std::string arm_base_frame = "base_link");

    // 枚举所有候选(优先级顺序:α outer, r inner);不调 IK
    std::vector<PoseCandidate> enumerate(const Config& cfg) const;

    // 对一组(已填 ik_q_solution 的)候选评分并选最优;ik_q_solution 全空时返 std::nullopt。
    // 评分需要 q_current(从 /joint_states 订阅拿到;若 q_current 为空则只用 r + α 偏好)
    std::optional<PoseCandidate> select_best(
        std::vector<PoseCandidate>& candidates, const Config& cfg) const;

    // 静态工具:给定 (α, r) 算单个 tcp_pose(纯几何)
    static PoseCandidate make_pose(
        geometry_msgs::msg::Point target, double alpha_rad, double ratio,
        std::string arm_base_frame);

private:
    geometry_msgs::msg::Point target_;
    double d_head_;
    std::string arm_base_frame_;
};
}
```

### 4.4 几何(每候选)

设目标世界系位置 `T = (Tx, Ty, Tz)`,base_link 在世界系下 `B = (0,0,0)`,头相机到目标距离 `d_head` (输入参数)。

对候选 `(α, r)`:

```
d_arm  = r · d_head
unit   = ((Tx-Bx)/||Tx-Bx||, (Ty-By)/||Ty-By||)        # 单位向量,xoy 平面
h_disp = d_arm · cos(α)
v_disp = d_arm · sin(α)
cam.x  = Tx - h_disp · unit.x                           # 沿 base→target 延长线目标外端
cam.y  = Ty - h_disp · unit.y
cam.z  = Tz + v_disp                                    # 抬高
forward = normalize(T - cam)
right   = normalize(cross(forward, world_up))           # world_up = (0,0,1)
up_cam  = cross(right, forward)
quat    = rotation_from_basis(right, up_cam, forward)   # tf2::Quaternion,roll=0
```

**性质**:

- 光轴**自动**指向 T,无需手算 yaw / pitch(几何保证)
- roll = 0(由 basis 推导时无 roll)

**边缘 case**:

- `||Tx-Bx, Ty-By|| < ε`(目标正下方):`unit` 退化 → `enumerate()` 抛 `INVALID_TARGET`,service 节点返 `success=false, message="target below base"`,BT 返 FAILURE
- `α = 90°` 时 v_disp = d_arm,水平距离 = 0,机位正上方 → 适合桌面物体(纯俯视),但机位可能与目标碰撞 → IK `avoid_collisions=true` 兜底
- `r=0.7` 距离过近(`d_head` 小):arm 与目标/自身几何干涉 → IK 失败 → 自动 fallback r

### 4.5 候选网格(12 个)

```
α candidates: [90°, 60°, 45°]              # 优先级:90° → 60° → 45°
r candidates: [0.7, 0.85, 1.0, 1.2]        # 距离比例
order:
  for α in α_candidates:                   # 俯角优先外循环
    for r in r_candidates:                 # 距离内循环
      yield PoseCandidate(α, r)
```

优先级由外层循环保证(早退出):**`(r=0.7, α=90°)` 是最佳首选**,大多数桌面目标可达即选;不可达才尝试 `(r=0.7, α=60°)`;以此类推。

### 4.6 Fallback / Early Exit 逻辑(service handler 内)

```cpp
// service handler (在 select_arm_observation_pose_service_node.cpp)
void handle_select_arm_obs_pose(
  const std::shared_ptr<Request> req,
  std::shared_ptr<Response> resp)
{
  PoseSelector sel(req->target_world, req->d_head, req->arm_base_frame);

  Config cfg = parse_config(req);

  // 1. 枚举 12 候选(纯几何,无 IK)
  auto candidates = sel.enumerate(cfg);
  resp->n_total_iterated = 0;
  resp->n_reachable = 0;

  // 2. 12 次 IK 验证 + 评分
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    auto& c = candidates[i];

    // 2a. IK 验证
    auto req_ik = std::make_shared<GetPositionIK::Request>();
    req_ik->ik_request.group_name = "arm";
    req_ik->ik_request.ik_link_names = {"tcp_link"};
    req_ik->ik_request.pose_stamped.header.frame_id = "base_link";
    req_ik->ik_request.pose_stamped.pose = c.tcp_pose;
    req_ik->ik_request.timeout =
      rclcpp::Duration(std::chrono::milliseconds(req->ik_call_timeout_ms));
    req_ik->ik_request.avoid_collisions = true;

    auto fut = ik_client_->async_send_request(req_ik);
    auto status = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), fut,
      std::chrono::milliseconds(req->ik_call_timeout_ms + 20));
    resp->n_total_iterated++;

    if (status != rclcpp::FutureReturnCode::SUCCESS) continue;
    auto resp_ik = fut.get();
    if (resp_ik->error_code.val != MoveItErrorCodes::SUCCESS) continue;

    // 2b. 记录 q_solution
    c.ik_q_solution.resize(6);
    for (int j = 0; j < 6; ++j)
      c.ik_q_solution[j] = resp_ik->solution.joint_state.position[j];

    // 2c. displacement
    if (!cfg.q_current.empty())
      c.joint_displacement_norm = l2_norm(c.ik_q_solution, cfg.q_current);

    resp->n_reachable++;

    // 2d. 早退出
    if (c.alpha_rad == deg2rad(90°) && c.ratio == 0.7)
    {
      // 最佳首选可达 → 直接填 resp 并 return
      fill_response(resp, c);
      RCLCPP_INFO(get_logger(),
        "early-exit on (α=90°, r=0.7) after %d iterations",
        resp->n_total_iterated);
      return;
    }
  }

  // 3. 全部候选跑完,从可达池里选 best
  auto best = sel.select_best(candidates, cfg);
  if (!best)
  {
    resp->success = false;
    resp->message = "no reachable observation pose in 12 candidates";
    RCLCPP_WARN(get_logger(), "%s", resp->message.c_str());
    return;
  }
  fill_response(resp, *best);
}
```

### 4.7 评分公式

```
score(c) = w_d · joint_displacement_norm
         + w_r · |r(c) - r_nominal|              # r_nominal = 0.7
         + w_α · |α_deg(c) - α_pref| / 90        # α_pref = 90
         + w_lim · near_joint_limit(c.q_solution)

默认权值:
  w_d = 1.0, w_r = 2.0, w_α = 1.0, w_lim = 1.0
near_joint_limit(q) = Σ max(0, |q_i| - 0.9 · q_max_i) / q_max_i   # 软极限 90% × q_max_i
```

**早退出 (r=0.7, α=90°) 可达时跳过评分**(已在 §4.6)。其它候选全部跑完,从可达池里选 score 最低的。

### 4.8 BT 薄壳节点

```
File: include/lgw_bt_planner/select_arm_observation_pose_bt.hpp
      src/select_arm_observation_pose_bt.cpp
Class: SelectArmObservationPoseBTNode : public BT::RosServiceNode<lgw_interfaces::srv::SelectArmObservationPose>
```

**Ports**:

| 方向 | 名称 | 类型 | 含义 |
|---|---|---|---|
| Input | `head_target_world` | `geometry_msgs/Point` | 上游 `ComputeHeadTargetWorld` 输出 |
| Input | `last_head_obs` | `yoloe_interfaces::msg::ObservationResult` | 用于在 BT 端算 `d_head = \|\|detections[0].center_m\|\|` |
| Input | `preferred_ratios_csv` | `string` (默认 `"0.7,0.85,1.0,1.2"`) | 透传给 service |
| Input | `preferred_alphas_deg_csv` | `string` (默认 `"90,60,45"`) | 透传给 service |
| Input | `ik_call_timeout_ms` | `int` (默认 50) | 透传给 service |
| Input | `arm_base_frame` | `string` (默认 `"base_link"`) | 透传给 service |
| Output | `tcp_pose_in_world` | `geometry_msgs/PoseStamped` | 选中 tcp_pose, write 到黑板, downstream `MoveArmToPose` |
| Output | `selected_alpha_deg` | `double` | debug |
| Output | `selected_ratio` | `double` | debug |
| Output | `n_reachable` | `int` | debug |
| Output | `n_total_iterated` | `int` | debug(可观察早退出) |

**实现**(薄):

```cpp
class SelectArmObservationPoseBTNode
  : public BT::RosServiceNode<lgw_interfaces::srv::SelectArmObservationPose>
{
public:
  SelectArmObservationPoseBTNode(const std::string& name,
                                 const BT::NodeConfig& cfg,
                                 const BT::RosNodeParams& params)
    : BT::RosServiceNode<...>(name, cfg, params) {}

  bool setRequest(typename Service::Request::SharedPtr& req) override
  {
    if (auto p = getInput<Point>("head_target_world"); p) req->target_world = *p;
    else return false;

    // d_head 来自 last_head_obs.center_m 的 L2 范数(就是 head 相机到目标的距离)
    auto obs_in = getInput<ObservationResult>("last_head_obs");
    if (!obs_in || obs_in->detections.empty()) return false;
    const auto& c = obs_in->detections.front().center_m;
    req->d_head = std::sqrt(c.x*c.x + c.y*c.y + c.z*c.z);

    /* ... defaults for csv / timeout / frame ... */
    return true;
  }

  BT::NodeStatus onResponseReceived(
    typename Service::Response::SharedPtr resp) override
  {
    if (!resp->success) return BT::NodeStatus::FAILURE;
    setOutput("tcp_pose_in_world", resp->selected_tcp_pose_in_world);
    setOutput("selected_alpha_deg", resp->selected_alpha_deg);
    setOutput("selected_ratio", resp->selected_ratio);
    setOutput("n_reachable", resp->n_reachable);
    setOutput("n_total_iterated", resp->n_total_iterated);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() { /* ports 表 */ }
};
```

**`compute_d_head_from_blackboard`**:读 `{last_head_obs}` (ObservationResult),取 `detections[0].center_m` 的 L2 范数。

### 4.9 与 `ComputeLookatPose` 关系

新 XML 默认两步走:

```
ComputeHeadTargetWorld   # 算 head 看到的目标 T
  ↓ outputs {head_target_world: Point}
SelectArmObservationPose  # BT 薄壳,1 次 RPC
  ↓ outputs {arm_lookat_pose: PoseStamped, ...}
MoveArmToPose (goal={arm_lookat_pose})
```

`ComputeLookatPose` 旧方式保留不删(作为 fallback / 对照),新节点是主路径。

### 4.10 BT XML 改动

`head_arm_independent_lookat.xml` step 1 改动:

```xml
<!-- OLD -->
<Sequence name="lookat_step">
  <IsHeadDetectionConfirmed name="check_head_for_lookat"
    topic_name="/perception/two_stage/camera_head/result"
    max_age_sec="0.3"
    last_observation="{last_head_obs}"/>
  <ComputeLookatPose name="compute_lookat_pose"
    head_observation="{last_head_obs}"
    ee_position_world="0.4;0.0;0.3"     <!-- hardcoded anchor -->
    world_frame="world"
    output_pose_key="lookat_pose"/>
  <MoveArmToPose name="move_to_lookat_pose" goal="{lookat_pose}"
    planning_time="5.0" pos_box="0.001;0.001;0.001"/>
</Sequence>

<!-- NEW -->
<Sequence name="lookat_step">
  <IsHeadDetectionConfirmed name="check_head_for_lookat"
    topic_name="/perception/two_stage/camera_head/result"
    max_age_sec="0.3"
    last_observation="{last_head_obs}"/>
  <ComputeHeadTargetWorld name="project_head_to_world"
    head_observation="{last_head_obs}"
    world_frame="base_link"
    output_key="head_target_world"/>
  <SelectArmObservationPose name="select_arm_lookat"
    head_target_world="{head_target_world}"
    last_head_obs="{last_head_obs}"/>
  <MoveArmToPose name="move_to_arm_lookat" goal="{arm_lookat_pose}"
    planning_time="5.0" pos_box="0.001;0.001;0.001"/>
</Sequence>
```

step 2 内 `tcp_pose_in_world="{lookat_pose}"` → 改为 `{arm_lookat_pose}`。

`{last_head_obs}` 已经在上游 `IsHeadDetectionConfirmed` 写入黑板,本节点直接读,无需新注入。

### 4.11 Launch 改动

`launch/bt_with_lgw_moveit.launch.py` 在 include 完 `agx_arm_moveit/demo.launch.py` 之后、include `lgw_bt_planner.launch.py` 之前,加入 service_node 的启动:

```python
Node(
    package='lgw_bt_planner',
    executable='select_arm_observation_pose_service_node',
    name='select_arm_observation_pose_service',
    output='screen',
    parameters=[{
        'ik_call_timeout_ms': 50,
        # ... 其他可调参数
    }],
)
```

保证:**service_node 启动晚于 move_group**(否则 `/compute_ik` 不可用,服务节点进入 degraded)。

### 4.12 失败路径汇总

| 场景 | 行为 |
|---|---|
| `/compute_ik` service 2s 不可达(节点构造时) | service 节点降级,所有 RPC 返 `success=false, message="ik not ready"`;BT 薄壳 → FAILURE |
| `/compute_ik` 单次超时(per-call) | 该候选失败,继续下一个 |
| service_node 进程 crash | BT 薄壳 `onFailure` → FAILURE;Sequence 上游 Fallback + ForceSuccess 跳过 |
| `head_target_world` 缺失 | BT 薄壳 `setRequest` 返 false → FAILURE;上游 `ComputeHeadTargetWorld` 已拦 |
| 12 候选全 IK 失败 | service 返 `success=false, message="no reachable"`,RCLCPP_WARN;BT FAILURE |
| 12 候选都可达但都是 near-singular | 选 score 最小者(joint_displacement + 关节软极限权重) |
| `/joint_states` 长时间未到达(订阅断流) | service 节点最后一次收到的 q_current 缓存使用,加 staleness check;超 2s 视为过期,跳过 displacement 评分,只用 r + α 偏好 |

---

## 5. Acceptance Criteria

### 5.1 单元测试 (gtest,加到 `test_lgw_bt_planner` 包)

| Test | 验证 |
|---|---|
| `PoseSelector.StaticGeometry` | 给定 P_target=(0.45, 0.10, 0.05), B=(0,0,0), d_head=0.50,验证 `(r=0.7, α=90°)` 候选: `cam.z == 0.40`, `cam.x ≈ 0.45`, `forward.z ≈ -1`, `forward.x ≈ 0`(纯俯) |
| `PoseSelector.CandidateGrid` | 12 候选数量、优先级、(r, α) 标签;enumerate 顺序 |
| `PoseSelector.SelectBestByDisplacement` | Mock 12 个候选其中 4 个 ik_q_solution 不同,验证 `select_best` 选 displacement 最小者 |
| `PoseSelector.SelectBestByAlphaRatio` | q_current 为空时,验证选 score 最低者(r=0.7, α=90° 优先) |
| `PoseSelector.AllFail` | 12 个 ik_q_solution 全空 → `select_best` 返 nullopt |
| `PoseSelector.INVALID_TARGET` | P_target.z=0 且 P_target.x=B.x 且 P_target.y=B.y(目标在 base 正下方) → enumerate 抛异常 |
| `SelectArmObservationPoseBTNode.SetRequest` | mock `{head_target_world}` + `{last_head_obs}`,验证 request 字段正确(d_head = \|\|center_m\|\|, csv 默认值透传) |
| `SelectArmObservationPoseBTNode.OnFailure` | mock RPC 服务返 success=false,验证 BT 节点 FAILURE |
| `SelectArmObservationPoseBTNode.OnSuccess` | mock RPC 服务返 success=true + 全字段,验证 outputs 写黑板 |

### 5.2 真机回归(5 个点位 × 3 次重复)

| 指标 | 通过标准 |
|---|---|
| arm→world 链路定位误差 | < 8cm(原方案基准 12cm @ 0.4;0.0;0.3 anchor) |
| arm→world 链路误差均值 | < 5cm |
| IK 失败率(per cycle) | < 2%(60 轮里 <1 次) |
| **BT 薄壳 tick** 时延 p50 / p99 | < 30ms / < 100ms |
| **service handler** 时延 p50 / p99(全集 12 候选) | < 200ms / < 600ms;早退出时 < 80ms |
| 真机 YOLOE 重新检测 confusion | < 5%(conf 分布与原方案一致) |

### 5.3 Build / Lint

- `docker exec self_l4t colcon build --packages-select lgw_bt_planner` 无 warning 增长
- 容器内 `ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/` 输出包含 `head_arm_independent_lookat.xml`
- 容器内 `ls install/lgw_bt_planner/share/lgw_bt_planner/srv/` 包含 `SelectArmObservationPose.srv`
- service_node 可独立启动:`ros2 run lgw_bt_planner select_arm_observation_pose_service_node` 不报错
- service 可被调:`ros2 service call /select_arm_observation_pose ...` 返正常响应

---

## 6. Risk Register

| 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|
| **service_node crash 影响 BT** | 低 | 高:整轮 BT 失效 | (1) service_node 自带崩溃重启(NAS 鸡尾酒);(2) BT 薄壳 `onFailure` FAILURE,Sequence 上游 Fallback + ForceSuccess 跳过 |
| **`/compute_ik` 启动延迟导致 service_node 进 degraded** | 中 | 中:BT 全 FAILURE 直到手动重启 | (1) launch 加 `<node on_exit="...">` 自动重启;(2) `wait_for_service(2s)` + retry 一次 |
| **d_head 极小(目标非常近 head)→ r=0.7·d_head 过近,IK 失败** | 中 | 中:IK 失败 | 自动 fallback r ∈ {0.7, 0.85, 1.0, 1.2} |
| **α=90° 撞桌面/物体** | 高 | 中:实际不可达 | 自动 fallback α ∈ {90°, 60°, 45°};KDL avoid_collisions=true 兜底 |
| **12 候选都不可达** | 低 | 低 | service 返 success=false;BT FAILURE,跟原 ComputeLookatPose 一致 |
| **`current_joint_state` 黑板键不存在** | 中 | 低:跳过 displacement 评分 | **已关闭**:service 自订阅 `/joint_states`,无需 BT 黑板注入;若订阅断流则 staleness check 兜底 |
| **`base_link` 在 TF 实际不为原点(机器人偏移)** | 低 | 中:yaw 方向错 | `arm_base_frame` port 默认 `base_link`,XML 可改 |
| **`ComputeHeadTargetWorld` 输出 P_target_world 被 head 投影噪声污染** | 中 | 中:整选 pose 都偏 | (非本 spec 范围,记录给后续 perception phase 修) |
| **srv 文件生成失败(rosidl_generator 没装)** | 低 | 高:build 失败 | 检查 `ros-humble-rosidl-default-generators` 已装;若缺,加入 Dockerfile |
| **新增 srv 让 package 多一个 msg-gen 阶段,build 变慢** | 低 | 低 | 单 srv 文件,~1s |

---

## 7. Implementation Checklist

| 文件 | 改动 |
|---|---|
| `srv/SelectArmObservationPose.srv` | **新增**:自定义 service 消息定义 |
| `srv/CMakeLists.txt` | **新增**:rosidl_generator for srv |
| `include/lgw_bt_planner/select_arm_observation_pose_geom.hpp` | **新增**:**纯 C++** 几何/评分模块声明(无 ROS 依赖) |
| `src/select_arm_observation_pose_geom.cpp` | **新增**:同上实现 |
| `src/select_arm_observation_pose_service_node.cpp` | **新增**:ROS service 节点,`rclcpp::Node` + 12 IK loop |
| `include/lgw_bt_planner/select_arm_observation_pose_bt.hpp` | **新增**:BT 薄壳声明 |
| `src/select_arm_observation_pose_bt.cpp` | **新增**:BT 薄壳实现(`BT::RosServiceNode<S>` 派生) |
| `src/bt_plugin_registration.cpp` | **修改**:在 `BT_REGISTER_ROS_NODES(factory, params)` 块里加 `factory.registerNodeType<SelectArmObservationPoseBTNode>("SelectArmObservationPose", params);` |
| `CMakeLists.txt` | **修改**:加 srv_gen + 新 .cpp 到 `add_executable` + 新 srv install 段 |
| `launch/bt_with_lgw_moveit.launch.py` | **修改**:加 `Node(...)` 启动 service_node |
| `bt_trees/head_arm_independent_lookat.xml` | **修改**:step 1 加 `ComputeHeadTargetWorld` + 替换成 `SelectArmObservationPose`,step 2 改读 `{arm_lookat_pose}` |
| `package.xml` | **确认**:`moveit_msgs` 已声明(`move_arm_to_pose_action_node.cpp` 已用);`rosidl_default_generators` 已声明 |
| `test/test_select_arm_observation_pose_geom.cpp` | **新增**:纯几何 gtest(无 ROS) |
| `test/test_select_arm_observation_pose_bt.cpp` | **新增**:BT 薄壳 gtest(用 BT::BehaviorTreeFactory + mock service) |
| `CMakeLists.txt` (test 段) | **修改**:注册 `ament_add_gtest` 测试 |

预估 effort:

- 纯几何模块 + gtest:1h
- srv 文件 + CMake srv-gen:15 min
- service_node:1.5h
- BT 薄壳 + bt_plugin_registration + CMake:30 min
- Launch + XML 改动:30 min
- BT 薄壳 gtest:45 min
- 容器内 build + sanity:30 min
- 真机回归:1.5h
- **合计 ~6h**

---

## 8. Open Questions / Out of Scope

1. **已关闭:启动时基线关节状态从哪儿来?** —— service 节点自订阅 `/joint_states`,加 staleness check(>2s 视为过期)。`bt_runner_node` 无需任何改动。
2. **service_node 启动失败重试策略?** — 目前 spec 用 `wait_for_service(2s)` 失败一次然后持续 degraded,是否要再加 retry timer 待定。
3. **`MoveGroup` 是否会自动跑 collision-aware KDL IK?** — 此处设 `avoid_collisions=true`,依赖 move_group collision scene。若发现 IK 解与 OMPL 规划后实际姿态不一致,需在 spec 后续追加 scene_diff。
4. **`ComputeHeadTargetWorld` 现有数学是否考虑 head 相机内参?** — 若它只是从 `center_m` 投到 base_link 而没用 depth buffer,误差 5-15cm 会污染本节点 α 选择。**实测前不解决,记录风险。**
5. **`service_node` 与 `bt_runner_node` 跑同一容器 vs 跨容器?** — 假定同容器;若将来要做远程调用,需要再加 ROS_DOMAIN_ID / 网络配置。

---

## 9. References

- `l4t/lg_walker_ws/src/lgw_bt_planner/src/move_arm_to_pose_action_node.cpp` — 现有 `/move_action` 模式参考
- `l4t/agx_arm_ws/piper_bt_planner_old/src/check_pose_reachable_service_node.cpp` — 已删除的 `CheckPoseReachableServiceNode`,IK 服务调用前例
- `l4t/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.srdf.xacro` — `<group name="arm">` 定义
- `l4t/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/kinematics.yaml` — KDL 求解器参数
- `l4t/lg_walker_ws/src/lgw_bt_planner/src/compute_lookat_pose_node.cpp` — 现 fixed anchor 的 lookat 计算,后续被新链路取代
- `l4t/lg_walker_ws/src/lgw_bt_planner/bt_trees/head_arm_independent_lookat.xml` — 本 spec 改动目标
- `l4t/lg_walker_ws/src/lgw_bt_planner/launch/bt_with_lgw_moveit.launch.py` — 加 service_node 启动
- ROS 2 Humble 文档:`BT::RosServiceNode<S>` 用法;[Writing a service and client (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)

---

## 10. Decision Log

- 2026-10-09:用户选定变量策略 — yaw 固定沿 `base_link → 目标`,俯角 α ∈ {90°, 60°, 45°} 优先级 90° → 60° → 45°,距离比例 r ∈ {0.7, 0.85, 1.0, 1.2}。
- 2026-10-09:早退出逻辑仅在最优先级 `(r=0.7, α=90°)` 可达时生效,避免评分系统本身在最优解存在时被覆盖。
- 2026-10-09:机位位于 `base_link → 目标` 的**延长线目标外端**(目标往反方向再走 `d_arm·cosα`),保持 yaw 锁定的同时抬高 `d_arm·sinα` 提供俯视。
- 2026-10-09:**架构选方案 β(独立 ROS service + BT 薄壳)而非方案 α(BT::SyncActionNode 直跑 12 IK)**,理由:
  1. 算力隔离,service 线程 12 IK 不阻塞 BT rhythm;
  2. 纯几何/评分模块可独立 gtest,无需起 ROS;
  3. service 可被 RViz / debug 脚本等其他客户端复用;
  4. 12×50ms 服务调用 + IPC vs 12×50ms 直接调 — 总耗时基本持平,但架构清晰得多。
- 2026-10-09:**保留纯几何模块**(无 ROS 依赖),作为 service_node 的内部分层,而不把所有逻辑塞进 service_node。理由:可 gtest、可在未来被其他 service / 脚本复用。
- 2026-10-09:不拆独立 `lgw_pose_selector` 包,留在 `lgw_bt_planner` 内。理由:目前只有 1 个 service_node 使用 + 1 个 BT 包消费,拆包增加 CMake/launch 复杂度而无实质收益;将来复用者增多再分离。
- 2026-10-09:**自定义 srv**(`SelectArmObservationPose.srv`)而非复用 `moveit_msgs::srv::GetPositionIK`。理由:response 需要传多个字段(选中 pose、α、r、n_reachable、message),单纯 `GetPositionIK` 响应塞不下。
- 2026-10-09:**request 不传 current_joint_state / current_tcp_pose_in_world**(简化版)。理由:service 节点自订阅 `/joint_states` 取 q_current 缓存,BT 黑板注入完全冗余(走两次管道还会引入不一致)。d_head 改由 BT 端从 `last_head_obs.center_m` 算 L2 范数后传入。
