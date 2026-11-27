# Detection Condition Nodes for lgw_bt_planner

| 字段 | 值 |
|---|---|
| Spec 路径 | `l4t/lg_walker_ws/src/lgw_bt_planner/docs/superpowers/specs/2026-09-24-lgw-detection-condition-nodes-design.md` |
| 创建日期 | 2026-09-24 |
| 状态 | Draft(待用户审阅) |
| 范围 | lgw_bt_planner 第一阶段:head/arm 检测结果的条件判断 BT 节点 |
| 不在范围 | 整棵 BT 树编排、目标选择、TF 转换、抓取位姿计算、底盘导航 |

---

## 1. 背景与动机

`lgw_bt_planner` 当前 3 个 leaf 节点全部是 MoveIt 动作包装(`MoveArmToPose` / `MoveArmToNamed` / `SetPoseStamped`),BT 树是硬编码 Sequence,没有任何感知输入。

要实现 lgw 项目"自主抓取"任务(见 `docs/bt-tech-summary.md` §5),感知结果(YOLOE 双相机检测)需要接入 BT 行为选择。本 spec 是这个接入工作的**第一阶段**:实现两个**条件判断型** BT 节点,让 BT 控制节点(Fallback / Sequence / Repeat)能根据"有没有检测到目标"做分支。

后续阶段(spec 范围外):
- 目标选择策略(多目标排序)
- TF2 转换(camera optical → base_link)
- 抓取位姿生成
- 整棵 `mission_grotto_walker.xml` 编排

---

## 2. 节点族

新增 **2 个 leaf 节点**,均为 BT 条件判断节点(SyncActionNode,通过 BT.ROS2 的 `RosTopicSubNode<T>` 基类实现):

| 节点 ID | 订阅话题 | 消息类型 |
|---|---|---|
| `IsHeadDetectionConfirmed` | `/perception/two_stage/camera_head/result` | `yoloe_interfaces::msg::ObservationResult` |
| `IsArmDetectionConfirmed` | `/perception/two_stage/camera_arm/result` | `yoloe_interfaces::msg::ObservationResult` |

**命名约定**:`Is<CAMERA>DetectionConfirmed`,C 大写凸出相机类型(head / arm)。

---

## 3. 节点接口

### 3.1 InputPort

| 端口 | 类型 | 默认值 | 含义 |
|---|---|---|---|
| `max_age_sec` | `double` | `2.5` | 上一条检测消息允许的最大年龄(秒);超过此值视为过时 |

### 3.2 OutputPort

| 端口 | 类型 | 写入时机 | 含义 |
|---|---|---|---|
| `last_observation` | `yoloe_interfaces::msg::ObservationResult` | **仅 SUCCESS 时** | 触发当前 SUCCESS 的那条完整 `ObservationResult`(包括 `detections[]`)。FAILURE 时**不写**,黑板保留上一次 SUCCESS 写入的值(或未被写入前为空) |

### 3.3 返回值语义

`BT::NodeStatus::SUCCESS`:四个条件**全部**满足
1. `last_msg.has_value() == true`(至少收到过一帧)
2. `last_msg->status == ObservationResult::CONFIRMED`(`uint8 == 1`)
3. `last_msg->detections` 非空
4. `this->now() - rclcpp::Time(last_msg->header.stamp) <= max_age_sec`

`BT::NodeStatus::FAILURE`:以上任一不满足

`BT::NodeStatus::RUNNING`:不使用(条件判断是无状态瞬间决策)

---

## 4. 实现细节

### 4.1 基类与代码模板

继承 `BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>`,框架自动负责:
- 订阅生命周期(create / shutdown / halt 时清理)
- 最新消息缓存(`std::optional<T> last_msg`)
- 反序列化 / QoS / 节点参数注入

派生类只实现 `onTick(const std::optional<ObservationResult>&)`,无主循环、无 `rclcpp::Rate`、无 `while(rclcpp::ok())` —— 遵循 `bt-tech-summary.md §1` 关键设计点。

### 4.2 两个节点的代码复用

两个节点类的 `onTick` 逻辑相同,差别只在:
- **类名**(节点注册 ID):`IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed`
- **类继承时的具体类型**(都是 `RosTopicSubNode<ObservationResult>`,无差别)

话题名由 `RosNodeParams` 注入(见 §5),**不在 C++ 中硬编码**。

实现层面有两种方案:

**方案 A:两个独立类**(`IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed` 各自一份)
- 优点:每个节点独立,代码自解释
- 缺点:小量代码重复(~20 行)

**方案 B:CRTP / 模板基类共享判断逻辑**
- 优点:无重复
- 缺点:BT.ROS2 注册时需要展开;增加编译复杂度

**本 spec 采用方案 A**:代码量小(~50 行/类),可读性优先;话题注入走 `RosNodeParams`,类本身不绑死话题名。

### 4.3 关键代码片段

```cpp
// include/lgw_bt_planner/is_detection_confirmed_node.hpp
class IsHeadDetectionConfirmed
    : public BT::RosTopicSubNode<yoloe_interfaces::msg::ObservationResult>
{
public:
    IsHeadDetectionConfirmed(const std::string& name,
                             const BT::NodeConfig& config,
                             const BT::RosNodeParams& params);

    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(
        const std::optional<yoloe_interfaces::msg::ObservationResult>& last_msg)
        override;
};
```

```cpp
// src/is_detection_confirmed_node.cpp
BT::NodeStatus IsHeadDetectionConfirmed::onTick(
    const std::optional<yoloe_interfaces::msg::ObservationResult>& last_msg)
{
    if (!last_msg) return BT::NodeStatus::FAILURE;
    if (last_msg->status !=
        yoloe_interfaces::msg::ObservationResult::CONFIRMED)
        return BT::NodeStatus::FAILURE;
    if (last_msg->detections.empty())
        return BT::NodeStatus::FAILURE;

    const auto max_age_sec = getInput<double>("max_age_sec").value();
    const auto msg_stamp = rclcpp::Time(last_msg->header.stamp);
    const auto age = this->now() - msg_stamp;
    if (age.seconds() > max_age_sec)
        return BT::NodeStatus::FAILURE;

    setOutput("last_observation", *last_msg);
    return BT::NodeStatus::SUCCESS;
}
```

### 4.4 `this->now()` 来源

`BT::RosTopicSubNode` 继承自 BT.CPP `SyncActionNode`,`this->now()` 返回的是 BT.ROS2 维护的 node handle 的 `now()`(基于 ROS2 clock)。这与 `header.stamp` 同源,做差有物理意义。

---

## 5. 配置文件注入

节点不硬编码话题名 / QoS,统一通过 `BT::RosNodeParams` 注入。launch 层在 `bt_runner` 的 ROS 参数中声明节点实例:

```yaml
# 在 bt_runner 的 ROS 参数中(略,具体在 lgw_bt_planner.launch.py 里配)
IsHeadDetectionConfirmed:
  ros__parameters:
    topic_name: "/perception/two_stage/camera_head/result"
    # 默认走 RELIABLE,与 yoloe publisher 一致
IsArmDetectionConfirmed:
  ros__parameters:
    topic_name: "/perception/two_stage/camera_arm/result"
```

具体配置位置见 `launch/lgw_bt_planner.launch.py`,launch 层遵循 BT.ROS2 标准,不修改 C++ 代码。

---

## 6. 错误处理

| 场景 | 节点行为 | BT 行为 |
|---|---|---|
| 节点从未收到过消息(`last_msg == nullopt`) | `onTick` 返回 FAILURE | 父控制节点(Fallback / Sequence)按其语义处理 |
| 收到 NO_TARGET / TIMEOUT / CANCELLED / ERROR | `onTick` 返回 FAILURE | 同上 |
| 收到 CONFIRMED 但 `detections` 空 | `onTick` 返回 FAILURE | 同上 |
| 收到 CONFIRMED 但消息时间戳超过 `max_age_sec` | `onTick` 返回 FAILURE | 同上 |
| 节点 halt 时(BT 整体中止) | BT.ROS2 自动清理 subscription | — |
| 黑板读取空(节点 SUCCESS 后 BT halt 重启) | 黑板被重新初始化,旧值消失 | 后续节点需自己处理"黑板为空"的情况(超出本 spec 范围) |

**TIMEOUT / CANCELLED 两种状态的特别说明**:`.msg` 文件定义了这两种 status(detection.txt §九 描述里漏了),本节点一律视为 FAILURE,与 NO_TARGET / ERROR 同等处理。后续节点如需区分,在自己的判断里再做。

---

## 7. XML 用法示例

### 7.1 最小烟雾测试(`detection_condition_test.xml`)

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
    <BehaviorTree ID="DetectionConditionTest">
        <Sequence>
            <IsHeadDetectionConfirmed max_age_sec="2.5"
                                      last_observation="{last_head_obs}"/>
            <IsArmDetectionConfirmed max_age_sec="2.5"
                                     last_observation="{last_arm_obs}"/>
        </Sequence>
    </BehaviorTree>
</root>
```

### 7.2 与未来节点的典型组合(伪代码,不在本 spec 实现)

以下 XML 列出**未来节点**(`SelectTargetFromObservations` / `TransformObservationToBaseLink`)与本 spec 节点的组合用法,仅作设计参考。**这些未来节点不在本 spec 范围内**,本 spec 只交付 `IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed`。

```xml
<Sequence>
    <!-- 1. 等头部看到目标 -->
    <IsHeadDetectionConfirmed max_age_sec="2.5"
                              last_observation="{last_head_obs}"/>

    <!-- 2. 后续节点(本 spec 范围外)从 @{last_head_obs} 读 detections -->
    <!--    做目标选择 / TF 转换 / 生成 approach pose -->
    <SelectTargetFromObservations
        observation="@{last_head_obs}"
        selected="{selected}"/>
    <TransformObservationToBaseLink
        observation="@{last_head_obs}"
        target="@{selected}"
        output_pose="{approach_pose}"/>

    <!-- 3. 机械臂抵近 -->
    <MoveArmToPose goal="@{approach_pose}"/>

    <!-- 4. 抵近后用 arm 相机二次确认 -->
    <IsArmDetectionConfirmed max_age_sec="2.5"
                             last_observation="{last_arm_obs}"/>

    <!-- 5. 抓取 -->
    <MoveArmToPose goal="@{grasp_pose}"/>
</Sequence>
```

---

## 8. 代码改动清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `include/lgw_bt_planner/is_detection_confirmed_node.hpp` | **新增** | 两个类声明(`IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed`) |
| `src/is_detection_confirmed_node.cpp` | **新增** | 两个类实现(共享同一份判断逻辑的代码骨架,各自硬编码类名) |
| `src/bt_plugin_registration.cpp` | **修改** | 加 2 行 `BT::registerNode<IsHeadDetectionConfirmed>(factory, "IsHeadDetectionConfirmed");` 类似调用 |
| `CMakeLists.txt` | **修改** | 把 `is_detection_confirmed_node.cpp` 加到 `add_executable` / 依赖列表;确保 install `bt_plugins/` 路径生效(参考现有 `move_arm_to_pose_action_node.cpp` 同位置) |
| `bt_trees/detection_condition_test.xml` | **新增** | §7.1 最小 demo XML,用于冒烟测试 |
| `launch/lgw_bt_planner.launch.py` | **修改** | (a) `plugins` 参数 glob 加入新 `.so`;(b) 在 bt_runner 的 ROS 参数里声明两个节点实例并配置 `topic_name`(详细格式在实现阶段定);(c) `behavior_trees` 参数新增 `detection_condition_test.xml` 入口 |

### 不修改的文件

- `bt_runner_node.cpp`(RosNodeParams 通过 BT.ROS2 标准机制传,无需改 runner)
- 现有 `move_arm_to_*_action_node.cpp` / `set_pose_stamped_node.cpp` / `pose_string_parser.cpp` / `named_target_loader.cpp`
- 现有 3 个 demo XML(`pose_via_set_pose_stamped.xml` / `go_to_target_and_back.xml` / `pick_and_place_demo.xml`)

### 新增包依赖

- `yoloe_interfaces`(本工作空间外,但已在 host `~/yoloe_deploy/ros2_ws/` 和容器内 `/opt/yoloe_transfer/humble_ws/` 都已构建)
- `behaviortree_ros2`(已是本包依赖,见 `bt_runner_node.cpp` 现有 `#include`)
- `rclcpp` / `geometry_msgs` / `std_msgs`(已有)

`package.xml` 加一行 `<depend>yoloe_interfaces</depend>`,确保 colcon build 时能找到 msg header。

---

## 9. 测试策略

### 9.1 单元测试

**不在容器内跑 `colcon test`**(`bt-tech-summary.md §8` 说明本容器缺 `ament_cmake_test`)。改跑 build/ 下 gtest 二进制。

测试用例(独立 `.cpp`,不依赖 ROS runtime):
- `test_is_detection_confirmed_logic.cpp`:把判断逻辑提取为纯函数,输入各种 `ObservationResult` 组合,验证返回值:
  - `last_msg == nullopt` → FAILURE
  - `status == NO_TARGET` → FAILURE
  - `status == CONFIRMED` 但 `detections` 空 → FAILURE
  - `status == CONFIRMED` + 有目标 + age 1.0s + max_age 2.5s → SUCCESS
  - `status == CONFIRMED` + 有目标 + age 3.0s + max_age 2.5s → FAILURE
  - `status == CONFIRMED` + 有目标 + age 0s + max_age 0s → SUCCESS(age 0 ≤ 0)

**这部分逻辑可独立测试的前提**:把 `onTick` 中的判断逻辑抽取成自由函数 `evaluate_detection(const std::optional<ObservationResult>&, double max_age_sec, rclcpp::Time now)`。这是 BT 节点里少见的"可单元测试"的代码,值得做。

### 9.2 集成测试

**冒烟测试**:`detection_condition_test.xml` + 手动 mock publisher。

测试用 mock 节点(`ros2 topic pub --once`)发布:
- 一条 CONFIRMED + 1 个 detection → 节点返回 SUCCESS,Sequence 推进
- 一条 NO_TARGET → 节点返回 FAILURE,Sequence 返回 FAILURE
- 一条 ERROR → 节点返回 FAILURE,Sequence 返回 FAILURE

完整流程(配合真机话题)见 `docs/detection.txt` §十二 监控程序。

### 9.3 Groot2 验证

启动 `bt_runner` 时 Groot2 自动 attach ZMQ publisher,可视化观察:
- 节点 tick 频率(10Hz)
- `last_observation` OutputPort 值(`SUCCESS` 时有内容)
- `max_age_sec` InputPort 值
- FAILURE 时 OutputPort 为空

---

## 10. 已知约束与注意事项

1. **`max_age_sec` 默认 2.5s 来自 `detection.txt §十一`**。BT 调用方可通过 XML 端口覆盖。
2. **节点 halt 后 STOPPED,黑板值不清除**(BT 标准行为)。如果将来跨子树场景需要,XML 加 `ClearBlackboard` 节点(无需改 C++)。
3. **每个节点一个独立 subscription**(BT.ROS2 `RosTopicSubNode` 行为)。两个条件节点订阅不同话题,无重复订阅问题。
4. **`RosNodeParams` 注入话题名**:launch 层配在 bt_runner 的 ROS 参数里,本 spec 不固化具体 YAML。
5. **真机话题不存在时**(YOLOE 没启动):节点一直 FAILURE,不会崩,符合 BT 容错要求。
6. **mock + 真机同启禁忌不适用本节点**:本节点是只读 subscriber,无副作用;CLAUDE.md 提到的 `/move_action` 多源竞争禁忌针对 MoveIt 动作节点,与本节点无关。
7. **`yoloe_interfaces::msg::ObservationResult::CONFIRMED` 等常量**:使用消息常量,不用数字 1/2/5(detection.txt §九 明确要求)。

---

## 11. 范围外(后续 spec)

| 任务 | spec 范围 |
|---|---|
| 目标选择 / 多目标排序 | 后续 spec(节点族:`SelectTargetFromObservations`) |
| TF2 转换(camera optical → base_link) | 后续 spec(`TransformObservationToBaseLink`) |
| 抓取位姿生成 | 后续 spec(`GenerateGraspPose`) |
| 底盘导航(Nav2 接入) | 后续 spec(`NavigateToApproachPose`) |
| 路径记录 / 回放 | 后续 spec |
| 整棵 `mission_grotto_walker.xml` 编排 | 后续 spec,在所有节点族就绪后 |

---

## 12. 验收标准

- [ ] 两个节点在 `colcon build` 中无 warning 通过
- [ ] gtest 单元测试 6 条 case 全部通过
- [ ] `detection_condition_test.xml` 在真机或 mock 下,SUCCESS / FAILURE 两条路径分别演示成功
- [ ] Groot2 可观察到节点状态、`max_age_sec` 值、`last_observation` 写入
- [ ] launch `lgw_bt_planner.launch.py` 一键启动正常,不破坏现有 3 个 demo
- [ ] 现有 3 个 demo XML(`pose_via_set_pose_stamped.xml` / `go_to_target_and_back.xml` / `pick_and_place_demo.xml`)继续能跑(回归测试)
