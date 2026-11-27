# Piper 机械臂 BT 规划层 —— 最小可用集设计

| 项 | 值 |
|---|---|
| 文档日期 | 2026-08-30 |
| 范围 | `l4t/agx_arm_ws/src/piper_bt_interfaces/`, `l4t/agx_arm_ws/src/piper_bt_planner/` |
| 目的 | 为 AgileX Piper 机械臂提供基于 BehaviorTree.CPP + BT.ROS2 的任务规划层,符合 `go2_bt_ws/docs/bt-design-standard.md` |
| 上位架构标准 | `l4t/go2_bt_ws/docs/bt-design-standard.md` |
| 状态 | 设计中(self-review 已完成,待用户 review) |
| 关联硬件 | Piper 6-DOF 臂 + agx_gripper 夹爪 |

---

## 1. 概览

### 1.1 问题

Piper 官方驱动 `agx_arm_ctrl_single_node` 只暴露 **topic + service**,不暴露 action server;夹爪走 `/control/joint_states` topic,既无心跳语义也无反馈等待机制。直接拿来当 BT action client 不可行。

### 1.2 解决方案

按 `bt-design-standard.md` 的三层架构:

```
Layer 1: BT XML (piper_minimal_demo.xml)
         ↓ 引用
Layer 2: BT Action Node 薄壳 (6 个,继承 BT::RosActionNode / BT::RosServiceNode)
         ↓ 调用
Layer 3: ROS Action/Service Server
         ├─ move_group (existing, /move_action + /compute_ik + /apply_planning_scene)
         └─ gripper_action_server (new, /control_gripper)
         ↓ 发布 / 订阅
         agx_arm_ctrl_single_node (existing, /control/joint_states, /feedback/gripper_status)
         ↓ CAN 总线
         Piper 硬件
```

### 1.3 范围(最小可用集)

6 个 BT 节点 + 1 个独立 server + 1 个 action 接口包 + 1 个 launch + 1 个 demo XML。

| 项 | 新建 | 复用 |
|---|------|------|
| BT 节点 | `MoveArmToPose`, `MoveArmToNamed`, `AttachObject`, `DetachObject`, `CheckPoseReachable`, `ControlGripper` | — |
| Server | `gripper_action_server` | `move_group` (agx_arm_moveit 已有) |
| 接口包 | `piper_bt_interfaces` (ControlGripper.action) | — |
| 编排 | `piper_bt_planner.launch.py` | `start_single_agx_arm_moveit.launch.py` |
| 演示 | `piper_minimal_demo.xml` | — |

### 1.4 不在范围(后续按需)

- `MoveArmCartesian`(`/get_cartesian_path` + `/execute_trajectory`)
- `MoveArmPlanOnly`(`/plan_kinematic_path`)
- `CheckSelfCollision`(`/check_state_validity`)
- `WhereAmI`(`/compute_fk`)
- 完整的 `<RecoveryNode>` + `<RoundRobin>` 编排(本设计只用 `<Sequence>`)
- 视觉伺服动态目标(已通过方案 B 在架构上支持,具体实现留给后续)
- 多臂 / 多 gripper 切换

---

## 2. 包与文件清单

```
l4t/agx_arm_ws/src/
├── piper_bt_interfaces/                            # NEW 接口包
│   ├── CMakeLists.txt                              # rosidl_generate_interfaces
│   ├── package.xml
│   └── action/
│       └── ControlGripper.action
└── piper_bt_planner/                               # NEW 主包
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/piper_bt_planner/
    │   └── named_target_loader.hpp                 # SRDF → map 加载器
    ├── src/
    │   ├── named_target_loader.cpp
    │   ├── move_arm_to_pose_action_node.hpp / .cpp
    │   ├── move_arm_to_named_action_node.hpp / .cpp
    │   ├── attach_object_service_node.hpp / .cpp
    │   ├── detach_object_service_node.hpp / .cpp
    │   ├── check_pose_reachable_service_node.hpp / .cpp
    │   ├── control_gripper_action_node.hpp / .cpp
    │   ├── gripper_action_server.cpp               # executable
    │   └── bt_runner_node.cpp                      # executable,MultiThreadedExecutor
    ├── launch/
    │   └── piper_bt_planner.launch.py
    └── bt_trees/
        └── piper_minimal_demo.xml
```

### 2.1 依赖(package.xml)

**piper_bt_interfaces**:
- `ament_cmake`
- `rosidl_default_generators`
- `rclcpp`, `builtin_interfaces`
- `action_msgs`

**piper_bt_planner**:
- `ament_cmake`
- `rclcpp`, `rclcpp_action`
- `behaviortree_cpp`
- `behaviortree_ros2` ← 关键,标准强制
- `moveit_msgs`
- `geometry_msgs`, `sensor_msgs`, `std_msgs`, `builtin_interfaces`
- `tf2_ros`, `tf2_geometry_msgs`
- `piper_bt_interfaces`
- `agx_arm_msgs`(可选,用于 /feedback/gripper_status 解析)
- 运行时依赖:`moveit_ros_move_group`(move_group), `agx_arm_ros`(agx_arm_ctrl)

---

## 3. Action / Service 接口约定

### 3.1 ControlGripper.action

```
# Goal
float64 target_width     # 目标宽度,单位 m,范围 [0.0, 0.1]
float64 force            # 目标夹持力,单位 N,范围 [0.5, 3.0],默认 1.0
float64 timeout          # 超时阈值,单位 s,默认 3.0
---
# Result
bool success             # 是否到位 / 未超时 / 未过流
float64 final_width      # 实际最终宽度
string message           # 错误描述(成功时为空)
---
# Feedback
float64 current_width    # 当前位置宽度(从 /feedback/gripper_status 读)
float64 current_force    # 当前夹持力
bool reached             # |current - target| < 0.005 视为到位
```

### 3.2 `/move_action` (`moveit_msgs::action::MoveAction`)

由 `move_group` 自带,无需新建。本设计的薄壳节点通过 `MotionPlanRequest.planning_options.plan_only` 决定 plan-only 还是 plan+execute(默认 plan+execute)。

### 3.3 `/apply_planning_scene` (`moveit_msgs::srv::ApplyPlanningScene`)

由 `move_group` 自带。本设计的 AttachObject / DetachObject 走 GET-改-APPLY 三步:
1. GET 当前 scene(获取 attached_collision_objects / world.collision_objects)
2. 修改 scene:AttachObject 是 ADD + ATTACH;DetachObject 是 DETACH + 落回 world
3. APPLY 修改后的 scene

### 3.4 `/compute_ik` (`moveit_msgs::srv::GetPositionIK`)

由 `move_group` 自带,CheckPoseReachable 直接调。`error_code.val == 1` (SUCCESS) → SUCCESS,否则 FAILURE。

### 3.5 `/control_gripper` (本设计新建)

见 §3.1。

### 3.6 命名 target 解析

启动时 `named_target_loader` 一次性读取 `agx_arm_moveit/config/agx_arm.srdf`,解析 `<group_state name="X" group="Y">...<joint name="J" value="V" />...</group_state>`,存为 `std::map<std::string, std::map<std::string, double>>`。**当前 SRDF 只有 "home" 一条**(joint1-6 全 0)。

---

## 4. 6 个 BT 节点详细规格

所有节点遵循标准 §5.2 模板:

```cpp
class XxxActionNode : public BT::RosActionNode<ActionT> {
  // 构造:BT::RosActionNode<ActionT>(name, cfg, params)
  // setGoal(ActionT::Goal&) -> bool
  // onResultReceived(const WrappedResult&) -> BT::NodeStatus
  // providedPorts() -> BT::PortsList
};
```

注册:
```cpp
factory_->registerNodeType<XxxActionNode>(
  "XxxName", BT::RosNodeParams(node_ptr, "<action_or_service_name>"));
```

### 4.1 MoveArmToPose

| 项 | 值 |
|---|---|
| 类型继承 | `BT::RosActionNode<moveit_msgs::action::MoveAction>` |
| Action 名 | `/move_action` |
| Input ports | `goal` (PoseStamped), `planner_id` (string, default ""), `planning_time` (double, default 5.0), `plan_only` (bool, default false) |
| Output ports | — |
| setGoal | `goal.request.workspace_parameters` + `goal_constraints[0].position_constraints` / `orientation_constraints`(从 `goal` PoseStamped 转);`goal.request.planning_options.plan_only = plan_only`, `planning_options.max_planning_time = planning_time`, `planning_options.planning_pipeline = planner_id` |
| onResultReceived | `SUCCEEDED → SUCCESS`,其他 → `FAILURE` |

### 4.2 MoveArmToNamed

| 项 | 值 |
|---|---|
| 类型继承 | 同上 |
| Action 名 | `/move_action` |
| Input ports | `named_target` (string), `planner_id`, `planning_time`, `plan_only` |
| Output ports | — |
| setGoal | 从 `named_target_loader_` 查 `named_target` → joint 值,填 `goal_constraints[0].joint_constraints`(每个 joint 一条 `JointConstraint`,`position = value`, `tolerance_above / below = 0.01`, `max_velocity / acceleration = 1.0`) |
| 找不到 named_target | setGoal 返回 false → RosActionNode 自动转 FAILURE |
| onResultReceived | 同上 |

### 4.3 AttachObject (方案 B: ADD + ATTACH 一次完成)

| 项 | 值 |
|---|---|
| 类型继承 | `BT::RosServiceNode<moveit_msgs::srv::ApplyPlanningScene>` |
| Service 名 | `/apply_planning_scene` |
| Input ports | `object_id` (string), `link_name` (string), `shape_type` (string, default "box"), `shape_dimensions` (vector<double>, default {0.05, 0.05, 0.05}), `pose` (PoseStamped), `touch_links` (vector<string>, default {}) |
| Output ports | — |
| onRequest | 构造 `PlanningScene`:`world.collision_objects.push_back({object_id, shape, pose})` + `robot_state.attached_collision_objects.push_back({object_id, link_name, touch_links})`,`is_diff = true` |
| onResponse | success → SUCCESS,否则 FAILURE |

`shape_type` 支持 `"box"` / `"cylinder"` / `"sphere"`(其余形状后续按需)。

**`shape_dimensions` 端口约定**:类型 `std::vector<double>`,XML 写作 `"0.05;0.05;0.05"` 分号串,BT.ROS2 默认支持该格式解析;实现里解析后按 `shape_type` 分配到 `solid.primitives[0].dimensions`(box:3 元素、cylinder:2 元素、sphere:1 元素,其余元素忽略)。

### 4.4 DetachObject

| 项 | 值 |
|---|---|
| 类型继承 | 同上 |
| Service 名 | `/apply_planning_scene` |
| Input ports | `object_id` (string), `link_name` (string) |
| Output ports | — |
| onRequest | GET 当前 scene → 在 `robot_state.attached_collision_objects` 里找 `object_id`(记录其 `object.operation` 与 `primitives[0]` shape/dimensions 与 attach 时的 `pose` in link frame)→ 通过 TF 拿 `link_name` 在 world 下的位姿 `T_world_link` → 计算 `T_world_obj = T_world_link * attached_pose_in_link` → 从 `attached_collision_objects` 删除该条目 → 在 `world.collision_objects` 加回(pose = `T_world_obj`)→ APPLY。任一步骤失败(找不到 object / TF 不可用)→ FAILURE |
| onResponse | success → SUCCESS,否则 FAILURE |

### 4.5 CheckPoseReachable

| 项 | 值 |
|---|---|
| 类型继承 | `BT::RosServiceNode<moveit_msgs::srv::GetPositionIK>` |
| Service 名 | `/compute_ik` |
| Input ports | `goal` (PoseStamped), `ik_link_name` (string, default "piper_tcp"), `timeout` (double, default 1.0), `avoid_collisions` (bool, default true) |
| Output ports | `ik_solution` (sensor_msgs::JointState, optional) |
| onRequest | 填 `ik_request.ik_link_names = {ik_link_name}`, `pose_stamped = goal`, `timeout = ros::Duration(timeout)`, `avoid_collisions` |
| onResponse | `error_code.val == 1` → SUCCESS + 写 `ik_solution`,否则 FAILURE |

### 4.6 ControlGripper

| 项 | 值 |
|---|---|
| 类型继承 | `BT::RosActionNode<piper_bt_interfaces::action::ControlGripper>` |
| Action 名 | `/control_gripper` |
| Input ports | `target_width` (double), `force` (double, default 1.0), `timeout` (double, default 3.0) |
| Output ports | `final_width` (double, optional) |
| setGoal | 三字段直传 |
| onResultReceived | `SUCCEEDED` → SUCCESS + 写 `final_width`,其他 → FAILURE |
| on_wait_for_result (可选) | 打印 feedback 日志,便于调试 |

---

## 5. gripper_action_server 详细设计

### 5.1 节点规格

| 项 | 值 |
|---|---|
| 节点名 | `gripper_action_server` |
| 命名空间 | 默认根命名空间(`/control_gripper`) |
| Action 名 | `/control_gripper`(同 §3.5) |
| 启动参数 | `default_force` (double,=1.0)、`tolerance` (double,=0.005)、`pub_rate` (double,=20.0) |
| 订阅 | `/feedback/gripper_status` (`agx_arm_msgs::msg::GripperStatus`) |
| 发布 | `/control/joint_states` (`sensor_msgs::msg::JointState`) |
| 校验 | width ∈ [0.0, 0.1];force ∈ [0.5, 3.0](超界 → fail with message) |

### 5.2 execute() 状态机

```
on goal received:
  validate(goal.target_width, goal.force)
  if invalid: fail, return
  set target_width_, force_, start_time_, timeout_

  loop at WallRate(pub_rate_):
    if is_canceling:
      publish_zero_width()           ← 关键:主动清零,防止误动作
      goal_handle->canceled(result)
      return
    publish_joint_states(target_width_, force_)   ← 心跳,20Hz
    fb.current_width = latest_gripper_status.width
    fb.current_force = latest_gripper_status.force
    fb.reached = |fb.current_width - target_width_| < tolerance_
    goal_handle->publish_feedback(fb)
    if fb.reached:
      goal_handle->succeed(result)
      return
    if elapsed > timeout_:
      publish_zero_width()
      goal_handle->abort(result with "timeout" message)
      return
    loop_rate.sleep()
```

### 5.3 publish_joint_states() 辅助

构造 `sensor_msgs::msg::JointState`:
```cpp
msg.name = {"gripper"};
msg.position = {target_width};
msg.velocity = {0.0};
msg.effort = {force};
```

`publish_zero_width()` 设 `target_width_ = 0.0`,然后 `publish_joint_states(...)`。

### 5.4 executor

`rclcpp::executors::MultiThreadedExecutor`(标准 §6.6)。

---

## 6. SRDF 加载器(named_target_loader)

### 6.1 接口

```cpp
namespace piper_bt_planner {

class NamedTargetLoader {
 public:
  // 读取 agx_arm_moveit/config/agx_arm.srdf
  // 返回: { "home" → {joint1: 0.0, joint2: 0.0, ..., joint6: 0.0}, ... }
  static std::map<std::string, std::map<std::string, double>> load_from_share();

  // 给定 target name 和 joint_name 列表,返回 [v1, v2, ...],长度与 joint_names 一致
  // 找不到 target → throw std::runtime_error
  static std::vector<double> get_joint_values(
    const std::map<std::string, std::map<std::string, double>>& table,
    const std::string& target,
    const std::vector<std::string>& joint_names);
};
}  // namespace
```

### 6.2 实现要点

- 路径:`ament_index_cpp::get_package_share_directory("agx_arm_moveit") + "/config/agx_arm.srdf"`
- 解析方式:`std::regex` 提取 `<group_state name="X" group="Y">` 与内部 `<joint name="J" value="V" />`(避免引入 tinyxml2 依赖)
- 异常:读不到 SRDF → `RCLCPP_FATAL` + `exit(1)`(启动期硬失败,运行期不进入)
- 线程安全:加载一次,只读访问,无需锁

---

## 7. Launch 编排

`l4t/agx_arm_ws/src/piper_bt_planner/launch/piper_bt_planner.launch.py`:

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
        'launch', 'start_single_agx_arm_moveit.launch.py'
    )

    return LaunchDescription([
        # 启动 agx_arm_ctrl + move_group + controllers(已有)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(agx_arm_moveit_launch)),

        # Layer 3: 夹爪 action server
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

        # Layer 1+2: BT 引擎 + 薄壳节点
        Node(
            package='piper_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            parameters=[{
                'bt_xml': os.path.join(
                    get_package_share_directory('piper_bt_planner'),
                    'bt_trees', 'piper_minimal_demo.xml'
                ),
            }],
        ),
    ])
```

---

## 8. Demo BT XML

`l4t/agx_arm_ws/src/piper_bt_planner/bt_trees/piper_minimal_demo.xml`:

```xml
<root BTCPP_format="4" main_tree_to_execute="PiperMinimalDemo">
  <BehaviorTree ID="PiperMinimalDemo">
    <Sequence name="PiperPipeline">
      <!-- SetBlackboard 注入 pick_pose 到黑板;后续 CheckPoseReachable / MoveArmToPose / AttachObject 引用。
           PoseStamped 字面量表示:frame_id="base_link";position (0.30, 0.0, 0.15);orientation 四元数 (0,0,0,1)。
           BT.CPP 4.x 支持 PoseStamped 字面量在 SetBlackboard 中以 YAML-like 字符串给出。 -->
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

物理时序:reach → close → attach → move → detach → open(attach 在 close 后,detach 在 open 前,与标准 §6 反模式说明一致)。

**`pick_pose` 注入**:本最小集用 `SetBlackboard` 把字面量写入黑板,运行前无需手动 `ros2 param set`。格式 `frame_id;x;y;z;qx;qy;qz;qw`(分号分隔 8 字段);实现里由 `SetBlackboard` 输出 `PoseStamped` 到 key `pick_pose`,下游节点通过 `goal="{pick_pose}"` 引用。

后续替换路径:把首行 `SetBlackboard` 换成 `ComputePickPose` 节点(视觉伺服)即可,无需改其他节点。

**PoseStamped 黑板兼容性提醒**:`SetBlackboard` 输出 `PoseStamped` 在 BT.CPP 4.x 默认需要手动解析字面量或用 YAML 风格;若实际运行时发现 `pick_pose` 解析失败,fallback 是给 `SetBlackboard` 写一个 thin shell(`WritePoseToBlackboard`:`std::string` → `PoseStamped`)并在 XML 里用 `<WritePoseToBlackboard output_key="pick_pose" value="base_link;0.30;...;1"/>` 显式声明类型。**这一步作为实施期间的快速 fallback,留待 writing-plans 中明确。**

---

## 9. bt_runner_node.cpp 设计

### 9.1 节点规格

| 项 | 值 |
|---|---|
| 节点名 | `bt_runner` |
| 启动参数 | `bt_xml` (string,默认 `bt_trees/piper_minimal_demo.xml`) |
| 内部 | `BT::BehaviorTreeFactory`、注册的 6 个节点类、SRDF 加载、tick 循环 |
| Executor | `MultiThreadedExecutor`(标准 §6.6) |

### 9.2 注册顺序(与 §4 一一对应)

```cpp
factory_->registerNodeType<MoveArmToPoseActionNode>(
    "MoveArmToPose", BT::RosNodeParams(node_ptr, "/move_action"));
factory_->registerNodeType<MoveArmToNamedActionNode>(
    "MoveArmToNamed", BT::RosNodeParams(node_ptr, "/move_action"));
factory_->registerNodeType<AttachObjectServiceNode>(
    "AttachObject", BT::RosNodeParams(node_ptr, "/apply_planning_scene"));
factory_->registerNodeType<DetachObjectServiceNode>(
    "DetachObject", BT::RosNodeParams(node_ptr, "/apply_planning_scene"));
factory_->registerNodeType<CheckPoseReachableServiceNode>(
    "CheckPoseReachable", BT::RosNodeParams(node_ptr, "/compute_ik"));
factory_->registerNodeType<ControlGripperActionNode>(
    "ControlGripper", BT::RosNodeParams(node_ptr, "/control_gripper"));
```

### 9.3 tick 循环

```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtRunner>(rclcpp::NodeOptions());
  node->init_tree();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate rate(10.0);  // 100ms tick = 10Hz
  while (rclcpp::ok()) {
    executor.spin_some();
    node->tick();           // tree_.tickOnce()
    rate.sleep();
  }
  rclcpp::shutdown();
}
```

---

## 10. Build 命令模板

执行 build 前必须先 source `behaviortree_ros2` 的 install(`/my_ws/l4t/bt_ros2_ws/install/`)。

```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  cd /my_ws/l4t/agx_arm_ws &&
  colcon build --packages-select piper_bt_interfaces piper_bt_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
"
```

只 build 这两个包,避免触发上游 `agx_arm_ros` / `agx_arm_moveit` 的全量编译。

启动:
```bash
docker exec -e ROS_DISTRO=humble self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/bt_ros2_ws/install/setup.bash &&
  source /my_ws/l4t/agx_arm_ws/install/setup.bash &&
  ros2 launch piper_bt_planner piper_bt_planner.launch.py
"
```

`pick_pose` 不需要运行时手动注入:`piper_minimal_demo.xml` 内的 `SetBlackboard` 节点直接写死。

---

## 11. 合规对照(btdesign-standard.md)

| 标准章节 | 检查点 | 本设计 |
|---|---|---|
| §1.2 判别红线 | BT 节点无 sleep / while / WallRate / 直发 publish / is_canceling | ✅ 6 节点全是 action/service client,无自建循环;心跳/取消语义在 server 进程 |
| §3.1 节点是 action client | 无 create_server | ✅ 6 节点全部为 client |
| §3.2 黑板数据流 | 输入从黑板流入,不硬编码 | ✅ demo XML 全黑板键,input port 有 default |
| §3.3 tick 与控制分离 | BT tick 频率独立于控制频率 | ✅ tick 10Hz,gripper server 20Hz,互不影响 |
| §3.4 标准控制流节点 | 用库标准控制流节点 | ✅ demo 用 `<Sequence>`(本最小集范围) |
| §3.5 反馈机制 | server 发 feedback | ✅ gripper_action_server 在 loop 内发 feedback;move_group 自带;CheckPoseReachable 是 service 无需 |
| §3.6 取消走标准 cancel | BT 节点 onHalted() 不 publish | ✅ 所有 BT 节点 onHalted 只置本地状态;gripper server 检测 cancel → 发零 width + goal_handle->canceled |
| §3.7 server 独立进程 | 每个 server 一个 main | ✅ move_group(已有)、gripper_action_server(新)、bt_runner 三个独立节点 |
| §6.1 BT 节点含控制循环 | 反例 | ✅ 无 |
| §6.2 BT 节点直发 topic | 反例 | ✅ 无 |
| §6.3 自造 retry | 反例 | ✅ 无(只用 `<Sequence>`) |
| §6.4 端口硬编码 | 反例 | ✅ XML 全黑板键 |
| §6.5 server 逻辑内嵌 runner | 反例 | ✅ runner 只 tick |
| §6.6 MultiThreadedExecutor 缺失 | 反例 | ✅ bt_runner 与 gripper_action_server 均使用 |
| §6.7 XML 节点名 ≠ register 字符串 | 反例 | ✅ 6 节点注册字符串与 XML 节点名一一对齐 |
| §2 三层架构 | XML / BT 薄壳 / Server 三层 | ✅ 严格三层 |
| §4 Nav2 对齐 | 编排层与控制层分离,频率独立 | ✅ |

---

## 12. 范围与不做的事

### 12.1 不在范围

- 单元测试 / 集成测试代码(由用户在新终端用 launch 真机验证)
- `<RecoveryNode>` + `<RoundRobin>` 完整恢复策略
- 多臂 / 多 gripper / revo2 灵巧手
- 视觉伺服 / 动态目标识别
- 离线轨迹回放
- MoveIt Servo(实时 jog 控制)

### 12.2 显式不做

- 不改 `agx_arm_ros` 与 `agx_arm_moveit` 任何现有源码
- 不改 `go2_bt_ws` / `bt_ros2_ws` 任何源码
- 不引入新的 apt 包(若发现缺包,先 `docker exec self_l4t apt install` 验证再改 Dockerfile)
- 不写 `tinyxml2` / 其他 XML 库依赖,用 `std::regex` 解析 SRDF
- 不写测试代码(本任务由 launch 集成验证覆盖)

---

## 13. 验收清单(用户执行时用)

按 writing-plans skill 输出的实施计划逐项执行后,应能验证:

| 验证项 | 命令 / 观察 |
|---|---|
| 接口包可编译 | `colcon build --packages-select piper_bt_interfaces` 退出码 0 |
| 主包可编译 | `colcon build --packages-select piper_bt_planner` 退出码 0 |
| action 接口可见 | `ros2 interface show piper_bt_interfaces/action/ControlGripper` 输出定义 |
| 服务节点可见 | `ros2 node list` 含 `/move_group`, `/agx_arm_ctrl_single_node`, `/gripper_action_server`, `/bt_runner` |
| action 接口可见 | `ros2 action list` 含 `/move_action`, `/control_gripper` |
| 启动无致命错误 | launch 终端无 `FATAL` / 红色堆栈 |
| demo XML 注册成功 | 终端日志 `registered BT nodes: MoveArmToPose, MoveArmToNamed, AttachObject, DetachObject, CheckPoseReachable, ControlGripper` |
| demo 树 SUCCESS | 终端日志 `BT root returned SUCCESS`(因 `SetBlackboard` 内置,无需手动注入 pick_pose) |

---

## 14. 风险与缓解

| 风险 | 缓解 |
|---|---|
| `behaviortree_ros2` 头文件找不到 | build 命令强制 source `bt_ros2_ws/install/setup.bash`(§10) |
| MoveGroup 没起来就启动 bt_runner | launch 通过 IncludeLaunchDescription 保证 move_group 在前 |
| TF 树未就绪 → DetachObject 失败 | 失败返回 FAILURE,BT 自然 FAIL,用户加 retry(后续工作) |
| TF 树没 `/piper_gripper_base` link | DetachObject 报 link not found → FAILURE,需查 SRDF 或 URDF |
| SRDF 解析失败 | 启动期 hard fail,日志清晰指明原因 |
| `/control/joint_states` 与 `/control/move_j` 共用冲突 | 启动 `start_single_agx_arm_moveit.launch.py` 时 `control_enabled:=false` 或仅用 BT 控制臂(具体取决于 launch 参数) |

---

## 15. 关联文档

- `l4t/go2_bt_ws/docs/bt-design-standard.md` — 上位架构标准
- `l4t/go2_bt_ws/docs/superpowers/specs/2026-08-29-bt-action-extension-conventions-design.md` — 项目级动作扩展约定(命名、文件布局、注册顺序)
- `l4t/agx_arm_ws/src/agx_arm_ros/README.md` — 官方驱动接口(control/feedback topic、service 列表)
- `l4t/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py` — 驱动源码(本设计只读参考,不改)
- `l4t/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.srdf` — SRDF(本设计只读参考,parse 出 named target)
- `l4t/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/moveit_controllers_*.yaml` — MoveIt controller 配置(本设计只读参考)