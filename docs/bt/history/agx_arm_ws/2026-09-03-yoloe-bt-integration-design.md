# YOLOE 感知 → BT 抓取集成设计

| 项 | 值 |
|---|---|
| 文档日期 | 2026-09-03 |
| 范围 | 新建 `l4t/agx_arm_ws/src/agx_arm_perception/`;BT XML;launch |
| 目的 | 把 yoloe 两阶段感知输出接入 Piper 机械臂 BT 树,完成"相机1 发现 → 移到观测位姿 → 相机2 二次识别 → 抓取"全链路 |
| 上位架构标准 | `l4t/go2_bt_ws/docs/bt-design-standard.md` + `l4t/agx_arm_ws/docs/superpowers/specs/2026-08-30-piper-bt-planner-design.md` |
| 现状依赖 | `l4t/agx_arm_ws/src/piper_bt_planner/bt_trees/pose_via_set_pose_stamped.xml` (观测位姿源); `/home/unitree/my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json` (TF 静态变换源); `/opt/yoloe_transfer/humble_ws/src/yoloe_humble_orchestration/yoloe_humble_orchestration/mock_camera2_coordinator.py` (真机链路触发器,**不动**) |
| 状态 | 设计中 (brainstorming 拍板,待用户 review) |
| 关联硬件 | Piper 6-DOF 臂 + agx_gripper 夹爪 + 相机1 (机器狗头部) + 相机2 (机械臂端) |

---

## 1. 概览

### 1.1 问题

yoloe 两阶段链路(`/opt/yoloe_transfer/humble_ws`)目前是自驱动的:

```
相机1 yoloe 检测 → ObservationRequest → mock 延时 1s → ObservationCommand.START → 相机2 yoloe → ObservationResult
```

链路本身跑得通,但**没有任何节点把 `ObservationResult`(相机2 彩色光学帧下的目标坐标)转换成 `base_link` 下的抓取位姿**,也没法让 BT 树在合适的时机介入抓取流程。

### 1.2 解决方案

按 `bt-design-standard.md` 的三层架构(BT XML / BT Action Node 薄壳 / ROS Action Server),把 yoloe 链路当作**只读事件源**,BT 端只**消费** `ObservationRequest` 和 `ObservationResult`,不发任何 topic。

```
Layer 1: BT XML (perceive_to_grasp.xml) — 7 节点 Sequence
         ↓
Layer 2: 3 个 BT Action Node 薄壳 (WaitForStage1Request / WaitForStage2Result / ComputePickPose)
         ↓ action 调用
Layer 3: 3 个 ROS Action Server (订阅 yoloe topic + TF buffer)
         ↓ 只读
         yoloe two_stage_node (existing) + mock_coordinator (existing, 不动)
```

### 1.3 范围 (最小可用集)

- 1 个新包 `agx_arm_perception`
- 3 个 ROS action server (`wait_stage1_request_server` / `wait_stage2_result_server` / `compute_pick_pose_server`)
- 3 个 BT action node 薄壳
- 3 个 action msg 定义 (`.action`)
- 1 个 BT XML (`perceive_to_grasp.xml`)
- 1 个 launch (`perception_pipeline.launch.py`)
- 1 个 TF 静态变换 launch (`tf_static.launch.py`, 读 calibration JSON 发 `tcp_link → camera_arm_color_optical_frame`)
- 15 个新文件,1 个新包,**不动** yoloe / mock / 现有 BT 节点 / 现有法兰盘 → tcp_link TF (`display.launch.py`)

---

## 2. 设计约束 (已拍板)

| # | 决策点 | 选择 |
|---|---|---|
| 1 | BT 与 yoloe 集成边界 | **BT 只消费**(订阅 Request/Result),不发 topic |
| 2 | 两阶段触发方式 | **保持现状**(mock + ObservationCommand.START 自驱动) |
| 3 | 机械臂到位时序 | **不上闸门**,Sequence 顺序靠 BT 节点保证;Result 不合格时 ComputePickPose 重拍 |
| 4 | 观测位姿来源 | **写死**(复用 `pose_via_set_pose_stamped.xml` 中 2026-09-02T23:30 实测值) |
| 5 | detection 选择策略 | `detections[0]` + `position_valid` 检查 |
| 6 | 运行目标 | **真机** (mock 保留是为不破坏相机2 yoloe 推理逻辑) |
| 7 | TF 转换位置 | 独立 ROS 节点 `compute_pick_pose_server` action server |
| 8 | action 接口 | 最少接口: `goal=timeout_sec`, `feedback=等待时长`, `result=success/fail + payload` |
| 9 | 包位置 | **新建 `agx_arm_perception`** |
| 10 | 抓取姿态 | **末端 z 轴朝下** (quaternion (1,0,0,0));detection 无 orientation 字段, 单目深度也无法估算, 类别查表留 out of scope |
| 11 | 抓取流程 | **top-down approach**: pre-grasp (上方 0.1m) → grasp (中心 z - size.z/2) → close → lift (上方 0.1m) → home |
| 12 | 抓取前夹爪状态 | **`pre-grasp` 前显式张开** (`target_width=0.085`);`grasp` 之后闭合 (`target_width=0.020`)。均留余量避免机械极限。**不放 `observ_pose` 之后张开**: 该处夹爪默认张开 (上电复位), BT 启动前已确认 |
| 13 | BT 外层控制 | **裸 Sequence + BT runner tick loop** — `bt_runner` 在 `while (rclcpp::ok())` 中 10Hz 持续 tick, Sequence 每次 tick 都从 `WaitForStage1Request` 重新开始; 上次抓取完成后下次 tick 自动等下一个目标。 **无需 Repeat / Loop 包装节点**。 空超时由 WaitForStage1Request 短超时 (3s) + 外层 kill BT runner 控制 |

---

## 3. 架构

### 3.1 BT 树 (`piper_bt_planner/bt_trees/perceive_to_grasp.xml`)

**循环机制** (依赖 BT runner 自然 tick loop, 详见 `bt_runner_node.cpp:139-141`): `bt_runner` 在 `while (rclcpp::ok())` 中以 10Hz 持续调用 `tree_.tickOnce()`。 Sequence 节点每次 tick 都从第一个 child (`WaitForStage1Request`) 重新开始, 上一次抓取完成后下次 tick 自动进入"等下一个目标"。 无需 Repeat / Loop 包装节点。

**空超时退出**: WaitForStage1Request 设短超时 (3s), 无目标时 3s 后 Sequence FAILURE → tree FAILURE → runner 继续 tick → 又从 WaitForStage1Request 开始 (进入"等待-失败-重试"循环, 直到节点被杀)。 外层 kill BT runner 或重启脚本即可停止。

```xml
<root BTCPP_format="4" main_tree_to_execute="PerceiveToGrasp">
  <BehaviorTree ID="PerceiveToGrasp">
    <Sequence name="main">
      <WaitForStage1Request   timeout_sec="3.0"
                              result_key="stage1_request"/>
      <SetPoseStamped        output_key="observ_pose"
                              value="world;0.2984353524805887;0.031647254555282334;0.1057279150037916;-0.013794928041476204;0.9083855022939685;0.03656195389544479;0.4163036184202731"/>
      <MoveArmToPose         goal="{observ_pose}"/>
      <WaitForStage2Result   timeout_sec="5.0"
                              result_key="stage2_result"/>
      <ComputePickPose       timeout_sec="10.0"
                              stage2_result_key="stage2_result"
                              pre_grasp_key="pre_grasp_pose"
                              grasp_key="grasp_pose"
                              lift_key="lift_pose"/>
      <ControlGripper        target_width="0.085"/>          <!-- 张开 (范围 0~0.1, 留余量避免机械极限) -->
      <MoveArmToPose         goal="{pre_grasp_pose}"/>
      <MoveArmToPose         goal="{grasp_pose}"/>
      <ControlGripper        target_width="0.020"/>           <!-- 闭合 (范围 0~0.1, 留余量避免完全闭合) -->
      <MoveArmToPose         goal="{lift_pose}"/>
      <MoveArmToNamed        named_target="home"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**超时参数** (BT XML 写死):
- `timeout_sec="3.0"` (WaitForStage1Request) — 短超时, 无目标时每个 cycle 最多 3 秒, BT runner 10Hz tick 间不浪费 CPU
- `timeout_sec="5.0"` (WaitForStage2Result) — 相机2 处理超时, 单 cycle 内 5 秒
- `timeout_sec="10.0"` (ComputePickPose) — 含最多 3 次重拍, 累计超时 10 秒

### 3.2 数据流

```
[two_stage_node stage1]
    │
    ▼ ObservationRequest (相机1 光学帧)
[WaitForStage1Request action server]
    │ SUCCESS + 缓存 latest_request 写到 blackboard
    ▼
[SetPoseStamped] → blackboard 'observ_pose' (world frame)
    │
    ▼
[MoveArmToPose] → 机械臂移到观测位姿
    │ (mock 已自动回 START,two_stage 跑 stage2)
    ▼ ObservationResult (相机2 彩色光学帧,detections[])
[WaitForStage2Result action server]
    │ SUCCESS + 缓存 latest_result 写到 blackboard
    ▼
[ComputePickPose action server]
    ├─ 读 blackboard 'stage2_result'
    ├─ 取 detections[0] → 检查 position_valid
    │  └─ false → 重发 ObservationRequest(经过 mock → START → stage2 重拍)
    ├─ 查 TF: camera_arm_color_optical_frame → base_link
    └─ 返回 PoseStamped (base_link) 写到 blackboard 'pick_pose'
    │
    ▼
[MoveArmToPose] → 抓取
    │
    ▼
[ControlGripper close]
    │
    ▼
[ComputePickPose action server]
    ├─ 读 blackboard 'stage2_result'
    ├─ 取 detections[0] → 检查 position_valid
    │  └─ false → 重发 ObservationRequest(经过 mock → START → stage2 重拍)
    ├─ 查 TF: camera_arm_color_optical_frame → base_link
    ├─ 计算 3 个 pose (全部 base_link frame, 末端 z 轴朝下):
    │   ├─ pre_grasp = (x=center_m.x, y=center_m.y, z=center_m.z + 0.1, q=(1,0,0,0))
    │   ├─ grasp     = (x=center_m.x, y=center_m.y, z=center_m.z - size_m.z/2, q=(1,0,0,0))
    │   └─ lift      = (x=center_m.x, y=center_m.y, z=center_m.z + 0.1, q=(1,0,0,0))
    └─ 写 blackboard 'pre_grasp_pose' / 'grasp_pose' / 'lift_pose'
    │
    ▼
[ControlGripper target_width=0.085]   ← 抓取前张开
    │
    ▼
[MoveArmToPose pre_grasp_pose]
    │
    ▼
[MoveArmToPose grasp_pose]
    │
    ▼
[ControlGripper target_width=0.020]   ← 闭合抓取
    │
    ▼
[MoveArmToPose lift_pose]
    │
    ▼
[MoveArmToNamed home]
```

### 3.3 时序

| 时刻 | 事件 |
|---|---|
| t=0 | BT 启动,调 WaitForStage1Request |
| t=N | 相机1 检测到目标,two_stage_node 发 Request |
| t=N+0.05 | WaitForStage1Request SUCCESS |
| t=N+0.05 | MoveArmToObservPose 开始 |
| t=N+1.05 | mock 自动回 START(延时 1s) |
| t=N+1.10 | two_stage_node 跑 stage2 |
| t=N+1.5~2.5 | ObservationResult(假设 stage2 耗时 0.4~1.4s) |
| t=N+2~4 | MoveArmToObservPose 完成(机械臂到位耗时 3~5s) |
| t=N+4 | WaitForStage2Result 实际等到的是 t=N+1.5~2.5 的 Result(机械臂**还没到位**) |

**风险**: Result 可能在 MoveArmToPose 完成前就出现。ComputePickPose 拿到的 `detections[0].center_m` 是机械臂未到位时的相机2 图像,坐标可能不准确。

**缓解**: ComputePickPose 内部检测 `position_valid=false` 或 `size_within_limit=false` 时,主动重发 ObservationRequest 触发 stage2 重拍。重拍上限 N 次(默认 3),超时 FAILURE。

### 3.4 设计依据验证(Q1/Q2 已源码核对)

设计已经按 spec §3.1–3.3 拍板,但下面两个关键点曾有歧义,通过读源码确认行为,作为 implementer / reviewer 的设计依据。

**(a) Q1: mock 是否只在相机1 检测到目标后才让相机2 跑 YOLOE?——是,且为硬门控**

证据 1,`/opt/yoloe_transfer/ros2_ws/src/yoloe_perception/yoloe_perception/two_stage_node.py`(权威容器路径,**spec 路径不一致以这个为准**):

- 行为锚点 `on_stage1_rgbd 入口` (≈ line 309):函数开头 `if not self.workflow.can_create_request(...): stage1_hits = 0; return`,stage1 每帧都被回调但**仅在冷却允许时**才跑 stage1_predictor。
- 行为锚点 `stage1_hits 累计行` (≈ line 345):`self.stage1_hits = self.stage1_hits + 1 if eligible else 0` + 后续 `< consecutive_hits` 早返回,必须连续 `consecutive_hits` 帧命中才发 `ObservationRequest`(单帧不发)。
- 行为锚点 `on_stage2_rgbd 函数头` (≈ line 402–404) 有**硬门控**:
  ```python
  def on_stage2_rgbd(self, color_msg, depth_msg):
      if self.workflow.state != WorkflowState.STAGE2_ACTIVE:
          return
  ```
  即相机2 的 RGB-D 订阅即使收到帧,只要 state 不是 STAGE2_ACTIVE,**直接 return,不跑 stage2 YOLOE**。
- 行为锚点 `on_command.START 分支` (≈ line 380–395):**唯一**把 state 推到 STAGE2_ACTIVE 的路径(调 `self.workflow.start_stage2(...)`)。

> **注意**: 行号会随 source 演化漂移,行为锚点(函数/分支名)是稳定锚;两者都给,implementer / reviewer 任意一处都能定位。

证据 2,Bench mock `/opt/yoloe_transfer/humble_ws/src/yoloe_humble_orchestration/yoloe_humble_orchestration/mock_camera2_coordinator.py`:

- 订阅 `/perception/two_stage/observation_request` → 排队 `delay_sec`(默认 1s)→ 发布 `ObservationCommand.START` 到 `/perception/two_stage/observation_command`。
- 真机替换此 mock 为"动臂到观测位姿 → 到位后发 START",`two_stage_node.py` 一行不用改。
- **BT 不参与此链路**:本 spec 全部 server 只**消费** `ObservationRequest` / `ObservationResult`,不发任何 topic(spec §1.2 / §2 反复强调的设计原则)。

**(b) Q2: TF 怎么"自动"做坐标转换?——不是自动,而是调用 TF2 API**

完整 TF 链:

| 链段 | 帧对 | 来源 | 类型 |
|---|---|---|---|
| 1 | `base_link → ... → link6 → tcp_link` | `robot_state_publisher` 自动发布(URDF + `joint_states` 驱动) | 动态(随机械臂运动) |
| 2 | `tcp_link → camera_arm_color_optical_frame` | `tf_static.launch.py` (Task 3) 读 `2026-09-02_08-34-16_calibration_tcp.json` → `static_transform_publisher` | **静态(launch 启动后固定)**——已确认 URDF `piper_l_with_gripper_description.xacro` 里**没有**此 frame,必须靠本任务 launch 注入 |
| 3 | `camera_arm_color_optical_frame → camera_arm_link → camera_arm_color_frame` | realsense2_camera 驱动自动发布(`humble_ws/two_realsense.launch.py` 已 Include) | 动态 |

`ComputePickPose` 调用模式 (`tf2_ros::Buffer::lookupTransform` in C++):

```cpp
geometry_msgs::TransformStamped t;
t = tf_buffer_->lookupTransform(
    "base_link",
    "camera_arm_color_optical_frame",
    tf2::TimePointZero,           // latest available,避免严格时间戳等待
    tf2::Duration(std::chrono::milliseconds(500))
);
geometry_msgs::Point p_base;
tf2::doTransform(detection.center_m, p_base, t);
```

**实现期注意**:
- TF2 缓冲启动时为空,`lookupTransform` 第一次调用大概率抛 `tf2::TransformException`。Action server 内做 try/except + 重试,或在启动后 sleep ~0.5s 等缓冲填充。
- **不要 transform orientation**:`YoloeDetection3D` 没有 orientation 字段,只有 position;姿态固定 z-down (quat=1,0,0,0),TF 转换只动 position。
- **Task 3 必须先于 3 个 server 起来**——如果 `tf_static.launch.py` 没启动或 calibration JSON 路径错,链段 2 缺失,`lookupTransform` 报 "could not find transform"。

---

## 4. 组件

### 4.1 WaitForStage1Request Action Server

**文件**: `agx_arm_perception/src/wait_stage1_request_server.cpp`

**接口** (`agx_arm_perception/action/WaitStage1Request.action`):
```
# goal
float32 timeout_sec
---
# result
bool success
string failure_reason
YoloeDetection3DArray last_request
---
# feedback
float32 elapsed_sec
```

**行为**:
- 订阅 `/perception/two_stage/observation_request`
- 收到任意一条 Request → SUCCESS,`last_request = msg.candidates`
- 超过 `timeout_sec` 仍未收到 → FAILURE, `failure_reason = "timeout"`

### 4.2 WaitForStage2Result Action Server

**文件**: `agx_arm_perception/src/wait_stage2_result_server.cpp`

**接口** (`agx_arm_perception/action/WaitStage2Result.action`):
```
# goal
float32 timeout_sec
---
# result
bool success
string failure_reason
yoloe_interfaces/msg/ObservationResult last_result
---
# feedback
float32 elapsed_sec
```

**行为**:
- 订阅 `/perception/two_stage/observation_result`
- 收到任意一条 Result → SUCCESS
- `result.status == TIMEOUT / ERROR / CANCELLED` → 仍 SUCCESS(因为 Result 本身就是个状态),由 ComputePickPose 决定后续
- `result.status == NO_TARGET / CONFIRMED` → SUCCESS
- 超过 `timeout_sec` → FAILURE

### 4.3 ComputePickPose Action Server

**文件**: `agx_arm_perception/src/compute_pick_pose_server.cpp`

**接口** (`agx_arm_perception/action/ComputePickPose.action`):
```
# goal
float32 timeout_sec
int32 max_retries          # default 3
float32 pre_grasp_offset_z # default 0.1 (m), pre-grasp 在 grasp 上方距离
float32 lift_offset_z      # default 0.1 (m), lift 在 grasp 上方距离
---
# result
bool success
string failure_reason
geometry_msgs/PoseStamped pre_grasp_pose  # in base_link, 末端 z 朝下
geometry_msgs/PoseStamped grasp_pose      # in base_link, 末端 z 朝下
geometry_msgs/PoseStamped lift_pose       # in base_link, 末端 z 朝下
---
# feedback
float32 elapsed_sec
int32 retry_count
```

**姿态约束**(重要事实): `YoloeDetection3D.msg` 没有 orientation 字段,**只输出位置 (`center_m`)+ 尺寸 (`size_m`)**。 单目深度无法估算物体朝向, 因此抓取姿态统一用**末端 z 轴朝下** (RPY = (π, 0, 0), quaternion = (1, 0, 0, 0))。 按类别查预设姿态表等扩展留 out of scope。

**行为**:
1. 收到 goal → 记录 `start_time`
2. 订阅 `/perception/two_stage/observation_result` (action 期间持续)
3. 取最新 Result 的 `detections[0]`
4. 检查 `position_valid == true`(否则**重试**:发 ObservationRequest 触发 stage2 重拍)
5. 查 TF: `tf_buffer.lookup_transform("base_link", "camera_arm_color_optical_frame", now, timeout=0.5s)`
6. 把 `detections[0].center_m` 应用变换,得到 base_link 下的点 `p_base`
7. 构造 3 个 PoseStamped(全部 `frame_id="base_link"`, `orientation = quaternion(1, 0, 0, 0)` 即 RPY=(π,0,0)):
   - `pre_grasp_pose`: position = (p_base.x, p_base.y, p_base.z + pre_grasp_offset_z)
   - `grasp_pose`: position = (p_base.x, p_base.y, p_base.z - size_m.z/2)
   - `lift_pose`: position = (p_base.x, p_base.y, p_base.z + lift_offset_z)
8. 返回 SUCCESS, 3 个 pose 写入 blackboard

**重试逻辑**:
```cpp
for (int i = 0; i < max_retries; ++i) {
    publish(ObservationRequest{header.frame_id=相机1_光学帧, candidates=[上次Result的candidates]});
    wait_for_new_result();
    if (新Result.detections[0].position_valid) break;
}
if (i >= max_retries) return FAILURE;
```

**注**: 重试发的 ObservationRequest 需要满足 yoloe 协议(带 request_id, candidates 等),且会被 mock 自动接住 → START → stage2 重拍。 mock 内部 `workflow.can_create_request` 控制 cooldown,如果 cooldown 没结束,mock 会忽略重试 Request,导致 stage2 不再触发。 ComputePickPose 必须容忍这种情况:发 Request 后等 N 秒看有没有新 Result,没有就下一次重试(累计等待时间不超 `timeout_sec`)。

### 4.4 BT Action Node 薄壳

3 个文件,统一模式(继承 `BT::RosActionNode`):

- `bt_nodes/wait_stage1_request_bt_node.cpp` — 读 `timeout_sec` 写 goal;成功时把 `last_request` 写到 blackboard
- `bt_nodes/wait_stage2_result_bt_node.cpp` — 同上
- `bt_nodes/compute_pick_pose_bt_node.cpp` — 读 `stage2_result_key` + `timeout_sec` + `max_retries` + `pre_grasp_offset_z` + `lift_offset_z`;成功时把 3 个 PoseStamped 分别写到 `pre_grasp_key` / `grasp_key` / `lift_key` 三个 blackboard 字段

注册到 BT XML 走 `piper_bt_planner/bt_plugins.xml` 或新建 `agx_arm_perception/bt_plugins.xml`。

---

## 5. 文件改动清单

### 新增 (14 个文件 + 1 个新包)

| 路径 | 内容 |
|---|---|
| `agx_arm_perception/package.xml` | ROS2 包定义, 依赖 `rclcpp`, `rclcpp_action`, `yoloe_interfaces`, `tf2_ros`, `geometry_msgs`, `behaviortree_cpp`, `bt_ros2` |
| `agx_arm_perception/CMakeLists.txt` | 编译 3 个 server + 3 个 bt_node |
| `agx_arm_perception/action/WaitStage1Request.action` | action 接口 |
| `agx_arm_perception/action/WaitStage2Result.action` | action 接口 |
| `agx_arm_perception/action/ComputePickPose.action` | action 接口 |
| `agx_arm_perception/src/wait_stage1_request_server.cpp` | ROS 节点 |
| `agx_arm_perception/src/wait_stage2_result_server.cpp` | ROS 节点 |
| `agx_arm_perception/src/compute_pick_pose_server.cpp` | ROS 节点 |
| `agx_arm_perception/src/bt_nodes/wait_stage1_request_bt_node.cpp` | BT action node |
| `agx_arm_perception/src/bt_nodes/wait_stage2_result_bt_node.cpp` | BT action node |
| `agx_arm_perception/src/bt_nodes/compute_pick_pose_bt_node.cpp` | BT action node |
| `agx_arm_perception/launch/perception_pipeline.launch.py` | 启动 3 个 server |
| `agx_arm_perception/launch/tf_static.launch.py` | 读 calibration JSON, 启动 static_transform_publisher |
| `agx_arm_perception/bt_plugins.xml` | BT 插件注册 |
| `piper_bt_planner/bt_trees/perceive_to_grasp.xml` | 主 BT 树 |

### 不动 (existing)

- `/opt/yoloe_transfer/humble_ws/src/yoloe_humble_orchestration/yoloe_humble_orchestration/mock_camera2_coordinator.py`
- `/opt/yoloe_transfer/ros2_ws/src/yoloe_perception/yoloe_perception/two_stage_node.py`
- `piper_bt_planner/bt_trees/pose_via_set_pose_stamped.xml` (参考其 pose 值)
- 现有 `MoveArmToPose` / `ControlGripper` / `MoveArmToNamed` / `SetPoseStamped` BT 节点
- 现有 `agx_arm_description/launch/display.launch.py` 的法兰盘 → tcp_link TF (用 tcp_offset 参数)

### TF 静态变换 (本期实现, 依赖就绪)

**文件**: `agx_arm_perception/launch/tf_static.launch.py`

**输入**: `/home/unitree/my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json`
- `position`: `[-0.07596965491001527, -0.000518235624476887, -0.0815325830105699]`
- `orientation`: `[-0.12200793766567013, 0.1227687120895532, -0.6831942485578522, 0.7094276039286281]`
- (该 JSON 已叠加 tcp_offset=[0,0,0.12,...], 等价于 `tcp_link → camera_arm_color_optical_frame` 直接变换)

**行为**: launch 启动时 `json.load` 解析, 取 `position` / `orientation`, 调 `static_transform_publisher`:
- `--frame-id tcp_link`
- `--child-frame-id camera_arm_color_optical_frame`
- `--x/y/z` ← position
- `--qx/qy/qz/qw` ← orientation

**launch 参数** (可覆盖默认 JSON 路径):
- `calibration_file`: 默认 `/home/unitree/my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json`
- `frame_id`: 默认 `tcp_link`
- `child_frame_id`: 默认 `camera_arm_color_optical_frame`

**集成到 perception_pipeline.launch.py**: 主 launch 内 `IncludeLaunchDescription(PythonLaunchDescriptionSource(tf_static.launch.py))`, 与 3 个 server 一起启动。

---

## 6. 测试策略

> 用户要求:**TDD,每走一步都需要测试验证,但是仅做必要的验证**。

### 6.1 测试原则

1. **集成测试覆盖后,删掉冗余单元测试**(per `feedback_minimize_tests.md`)
2. **不为新库单独写独立 smoke test**
3. **能合并就合并,不另开测试文件**
4. **只在行为有不确定性的地方加测试**

### 6.2 测试层级 (仅必要)

| 层 | 测试 | 必要性 |
|---|---|---|
| 单元 | action goal/feedback/result 序列化 | 不测(RosActionNode 标准行为) |
| 单元 | TF 查询 (`lookup_transform`) 边界 | 仅在 ComputePickPose 单测里覆盖**空 buffer / timeout / target 不存在** 三种情况 |
| 集成 | `tf_static.launch.py` 启动后 `tf2_echo tcp_link camera_arm_color_optical_frame` 验证 TF 数值正确 | **必要** (如果 TF 不对, ComputePickPose 算的 pose 全错) |
| 集成 | `perceive_to_grasp.xml` 端到端跑通(启动全部节点 → 两阶段 yoloe + mock → BT SUCCESS) | **必要** (这是 spec 的核心验收条件) |
| 集成 | Result 不合格时 ComputePickPose 重拍路径 | **必要** (验证重试逻辑,确认 mock / stage2 / Request 能再走一轮) |
| 集成 | BT 节点 blackboard 字段正确(`stage2_result_key` / `output_key`) | 集成测试覆盖,单测不重复 |

### 6.3 验收条件 (DoD)

0. **集成测试 0 (TF 启动验证)**: 启动 `tf_static.launch.py` 后 `ros2 run tf2_ros tf2_echo tcp_link camera_arm_color_optical_frame` 验证数值与 `_tcp.json` 中 position/orientation 一致 (允许四元数符号相反, ROS TF 内部会归一化)
1. **集成测试 1 (端到端正常路径)**: 启动 mock + two_stage_node + `tf_static.launch.py` + 3 个 action server + BT 树 → 手动 publish 一条 `ObservationRequest`(模拟相机1检测到目标) → BT 跑完 → 夹爪 close → 回 home
2. **集成测试 2 (重拍路径)**: 启动同上,第一次 publish Request 让 stage2 模拟返回 `position_valid=false`, 第二次 publish Request 让 stage2 返回有效 → ComputePickPose 重发后第二次 SUCCESS → BT 跑完
3. **集成测试 3 (超时路径)**: 启动 mock + server + BT 树 → **不 publish 任何 Request** → WaitForStage1Request 3s 超时 FAILURE → BT Sequence FAILURE (符合预期)

### 6.4 不测的内容

- action msg 字段类型 / 序列化(标准 ROS2 行为,RosActionNode 处理)
- BT 节点继承 BT::RosActionNode 的 tick 状态机(标准 BT.ROS2 行为)
- TF lookup 在正常情况下的成功路径(集成测试已覆盖)
- mock / two_stage_node 内部逻辑(不是本 spec 范围)

---

## 7. 风险与缓解

| 风险 | 缓解 |
|---|---|
| stage2 Result 在 MoveArmToObservPose 完成前到达 | ComputePickPose 强制 position_valid 检查 + 重拍 |
| mock cooldown 不允许重发 Request | ComputePickPose 触发重试时检查 mock 的 `workflow.can_create_request` 信号;如果 cooldown 没结束, 等待 |
| TF lookup 超时 (`camera_arm_color_optical_frame → base_link`) | ComputePickPose 内 0.5s timeout + FAILURE |
| yoloe_interfaces 在 Humble 容器 / Foxy host 跨 distro 通信 | BT 在 Humble 跑, mock 和 two_stage_node 在 Humble 容器; 不跨 distro |
| `compute_pick_pose_server` 发的 Request 被 mock 拒收 | 测试 2 必须验证此路径, 失败时调整 mock 或重试机制 |
| **detection 没 orientation, 抓取姿态固定 z 朝下** | 本期接受; 按类别查姿态表 (yml) 留 out of scope (见 §8) |
| pre-grasp / grasp / lift z 偏移写死 (0.1m) | goal 参数化 `pre_grasp_offset_z` / `lift_offset_z`, BT 节点可调 |

---

## 8. 后续 (out of scope)

- 观测位姿改为动态计算(从相机1 detection 推算, 而非写死)
- ComputePickPose 增加多 detection 选择策略(目前 detections[0])
- 按 class_name 查预设抓取姿态表(目前固定 z 朝下)
- 与 Go2 (机器狗) BT 树协同(从 Go2 走到目标附近, 启动本 BT 树)
- 真机无 mock 路径(等真机协调器硬件就绪)

---

## 9. 决策日志

| 时间 | 决策 | 理由 |
|---|---|---|
| 2026-09-03 | BT 只消费,不主动发 topic | 符合 bt-design-standard.md |
| 2026-09-03 | 观测位姿写死 | 已有 pose_via_set_pose_stamped.xml 实测值, 台架/真机复用 |
| 2026-09-03 | 新建 agx_arm_perception 包 | 职责清晰,不动现有包 |
| 2026-09-03 | ComputePickPose 强制 position_valid + 重拍 | 缓解 stage2 提前到达的时序风险 |
| 2026-09-03 | mock 不动 | 真机链路依赖 mock 启动 stage2, 不能破坏 |
| 2026-09-03 | 测试仅必要 (4 个集成测试, TF 启动验证是其中之一) | per feedback_minimize_tests.md |
| 2026-09-03 | TF 静态变换 (tcp_link → camera_arm_color_optical_frame) 本期实现 | 现有代码库没有;ComputePickPose 必须依赖此 TF 才能算正确 pose;通过 `tf_static.launch.py` 读 calibration JSON 发 static TF |
