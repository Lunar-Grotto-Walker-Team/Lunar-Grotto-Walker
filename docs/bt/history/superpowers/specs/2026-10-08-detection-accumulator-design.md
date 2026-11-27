# Detection Accumulator Service Design

**Status**: Draft (待用户评审 + 后续实现)
**Date**: 2026-10-08
**Author**: Claude (brainstorming session)
**Workspace**: 新建 `l4t/detection_accumulator_ws/` + 修改 `l4t/lg_walker_ws/`
**Goal**: 把 YOLOE 检测结果从 BT 节点(直接订阅 topic + 时间新鲜度)迁出,改由独立 ROS 节点订阅并按"连续 N 帧 + first-in-range + per-call freshness"策略累加,BT 通过 service 拉取确认目标。

---

## 1. Background

### 1.1 现存问题

`l4t/lg_walker_ws/src/lgw_bt_planner/` 当前的 YOLOE→BT 链路:

```
YOLOE dual_camera_node
  └─ /perception/two_stage/camera_head/result    (ObservationResult)
  └─ /perception/two_stage/camera_arm/result     (ObservationResult)
       │
       ▼
IsHeadDetectionConfirmed (BT::RosTopicSubNode 派生)
  └─ evaluate_detection(opt_msg, max_age_sec=0.3, now)
       ├─ 条件 1: 至少收到 1 条消息
       ├─ 条件 2: status == CONFIRMED
       ├─ 条件 3: detections 非空
       └─ 条件 4: (now - msg.header.stamp) ≤ 0.3s
IsArmDetectionConfirmed (同上)
       │
       ▼
ComputeLookatPose / ComputeArmTargetWorld (读 {last_head_obs} / {last_arm_obs})
```

**3 个问题**:

1. **"看到目标"是单帧时间新鲜度,不是多帧一致性**:任何一帧 CONFIRMED + 0.3s 内都算"看到",YOLOE 偶发单帧误识别会直接 pass-through。
2. **BT 内做 freshness + status 解析 + 多源管理**,每加一种 debounce 策略都要改 BT 节点 + evaluate_detection.h 签名 + XML,改动面跨 ≥3 文件。
3. **多 detection 场景语义糊**:detections[] 多目标时只看 `front()`(最高 conf),即使该目标超出工作距离也强行取它;近处 conf 稍低的目标被丢。

### 1.2 改动诉求(用户原始描述)

> 现在我不想直接将yoloe接入BT节点,而是接入一个ros服务节点,该节点一直接收yoloe推理结果并统计,统计条件为检测结果z方向上距离小于阈值,当连续多帧检测到目标时,通过话题向BT节点发送服务请求,BT节点接收后向该节点发送确认信息,该节点状态重置,BT节点在黑板中记录待处理目标,处理结束后在黑板中清除该目标,当目标信息存在时不触发新的检测结果。

诉求拆解:
- 累加器:独立 ROS 节点,持续订阅 YOLOE,按"连续 N 帧 + Z 距离阈值"统计
- BT:通过 service 拉取候选目标(非订阅 topic 推)
- BT 消费完后"确认",累加器 reset 状态
- BT 黑板记录"待处理目标",存在期间不触发新结果
- "不触发"语义 = "服务请求时间与上次得到检测结果的时间过久时,该结果不接受"(用户后续澄清,见 Decision Log)

---

## 2. Goal / Non-goals

### 2.1 Goal

1. 新建独立 ROS 包 `detection_accumulator`,独立 colcon workspace `l4t/detection_accumulator_ws/`。
2. 累加器节点订阅 `/perception/two_stage/camera_{head,arm}/result`,提供 2 个 service:`/detection_accumulator/{head,arm}/get_target`。
3. 累加器内部按"连续 N 帧 first-in-range"累加,达到阈值后 ARMED 等待 BT 拉取;BT 拉取后 reset。
4. 累加器在 ARMED 状态下做 **per-call freshness check**:"服务请求时间 - 上次得到检测结果时间"超阈值则拒绝(返 has_target=false),状态 reset 到 COUNTING。
5. BT 侧新增 1 个 RosServiceNode 派生类 `GetTargetFromAccumulator`,XML 里 head/arm 各 1 个实例(同 class,不同 `service_name`)。
6. 替换 `head_arm_independent_lookat.xml` / `head_to_arm_verify_grasp.xml` 里的 `IsHead/ArmDetectionConfirmed` 调用点,下游 `ComputeLookatPose` / `ComputeArmTargetWorld` 不动。
7. 累加器启动通过独立 launch 文件,集成进现有 `start_lgw_*.sh`(与 BT 同一 `docker exec self_l4t` 命令链)。

### 2.2 Non-goals

1. 不动 YOLOE 本身代码(`yoloe_try/`, `yoloe_deploy/`),只读它发的 topic。
2. 不改 `ComputeLookatPose` / `ComputeArmTargetWorld` / `MoveArmToPose` 任何逻辑,只换上游检测节点。
3. 不引入 instance_id 去重逻辑(用户最终选择 per-call freshness 而非 instance_id,见 Decision Log §12.4)。
4. 不在累加器里做投影到 base_link 的数学(只看 camera optical frame z 阈值,简单)。
5. 不跨 workspace 集成到 `yoloe_humble_orchestration` 或其它 orchestration 包;只服务 `lgw_bt_planner`。
6. 不替换当前 `yoloe_interfaces::msg::ObservationResult` 消息类型,直接复用。
7. 不实现"按 conf 排序 + 多目标并发 tracking",first-in-range + 单目标 ARMED 即可。

---

## 3. Design Constraints

### 3.1 BT.ROS2 库约束

- `BT::RosServiceNode<ServiceT>` 是 abstract base,**必须派生**实现 `setRequest()` 和 `onResponseReceived()` 两个 pure virtual。XML-only 配置不可行,必须写 1 个 C++ 类。
- `BT::RosServiceNode` 在累加器侧不存在 server 基类,BT 侧只有 client 角色。累加器用 `rclcpp::Service<ServiceT>` 直接提供 server,不需要走 BT.ROS2。
- 同一 BT 节点类可通过 XML `service_name` 端口传不同 service 名,head/arm 用同一 C++ 类即可。
- 现有 `IsHead/ArmDetectionConfirmed` 节点可以保留不删(本 spec 不删,后续 cleanup 可选),新 XML 改用 `GetTargetFromAccumulator` 即可。

### 3.2 部署约束

- 累加器在 `self_l4t` 容器内运行(同 BT),编辑在 host (`/home/unitree/my_ws/...`),build / run 走 `docker exec self_l4t`。
- 服务注册名 / topic 名走 ROS params,默认 `/detection_accumulator/head/get_target` / `/detection_accumulator/arm/get_target` + `/perception/two_stage/camera_{head,arm}/result`。
- 累加器状态机在单节点内,使用 rclcpp 默认 executor(可配)。`std::mutex` 保护 head/arm 状态结构,避免 sub 线程与 service 线程 race。
- 不依赖 `MoveArmToPose` 反馈/TCP 链路,与 arm 控制系统解耦。

### 3.3 真机约束

- 累加器是**纯计算节点**,不动硬件,不接 Unitree SDK,不写 L1 LiDAR topic。
- 累加器 crash → BT 端 `RosServiceNode::onFailure` 返回 FAILURE,与现状 IsHeadDetectionConfirmed 失败语义一致(走 Fallback+ForceSuccess 跳过 step)。

---

## 4. Architecture

```
[ YOLOE dual_camera_node ]
   │
   │ /perception/two_stage/camera_head/result  (existing)
   │ /perception/two_stage/camera_arm/result   (existing)
   ▼
[ detection_accumulator_node ]  ← 新包,独立进程
   │  订阅头/臂两个 topic
   │  内部状态:每个 source 一份 {state, counter, current_target,
   │             last_target_stamp_}
   │  ROS params: consecutive_n, z_threshold_m, freshness_threshold_sec,
   │             head_topic, arm_topic, head_service_name, arm_service_name
   │
   │  提供 2 个 service(各自 handler):
   │    /detection_accumulator/head/get_target
   │    /detection_accumulator/arm/get_target
   ▼
[ BT (lgw_bt_planner) ]
   │  新增 1 个 BT 节点类: GetTargetFromAccumulator (RosServiceNode 派生)
   │  XML 2 个实例(同 class, service_name 不同):
   │    head:
   │      <GetTargetFromAccumulator name="..."
   │        service_name="/detection_accumulator/head/get_target"
   │        last_observation="{last_head_obs}"/>
   │    arm:
   │      <GetTargetFromAccumulator name="..."
   │        service_name="/detection_accumulator/arm/get_target"
   │        last_observation="{last_arm_obs}"/>
   │  替换 head_arm_independent_lookat.xml / head_to_arm_verify_grasp.xml
   │  里的 IsHead/ArmDetectionConfirmed
   ▼
[ 下游节点 ComputeLookatPose / ComputeArmTargetWorld ]   ← 不动
   │  继续读 {last_head_obs} / {last_arm_obs}
   ▼
[ MoveArmToPose 等 ]
```

---

## 5. Components

### 5.1 新包 `detection_accumulator` (workspace: `l4t/detection_accumulator_ws/`)

```
l4t/detection_accumulator_ws/
├── src/
│   └── detection_accumulator/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── msg/                                  # (不需要, 复用 yoloe_interfaces)
│       ├── srv/
│       │   └── GetTarget.srv
│       ├── include/
│       │   └── detection_accumulator/
│       │       └── detection_accumulator_node.hpp
│       ├── src/
│       │   ├── detection_accumulator_node.cpp    # 主节点
│       │   ├── first_in_range.cpp                # pick first detection with z<thr
│       │   └── first_in_range.hpp
│       ├── launch/
│       │   └── detection_accumulator.launch.py
│       └── test/
│           ├── test_first_in_range.cpp           # 单元测试
│           └── test_state_machine.cpp            # 状态机 mock 测试
```

依赖:
- `rclcpp`
- `yoloe_interfaces`(已存在,`l4t/yoloe_deploy/ros2_ws` 或 `yoloe_try/humble_ws` 编译产出,需在 detection_accumulator_ws 装好后 source 进来)
- `std_srvs` (不需要, 用自定义 srv)

### 5.2 修改 `l4t/lg_walker_ws/`

新增文件:
- `src/lgw_bt_planner/include/lgw_bt_planner/get_target_from_accumulator_bt_node.hpp`
- `src/lgw_bt_planner/src/get_target_from_accumulator_bt_node.cpp`

修改文件:
- `src/lgw_bt_planner/src/bt_plugin_registration.cpp` — 新增 1 行 `factory.registerNodeType<...>("GetTargetFromAccumulator", params);`
- `src/lgw_bt_planner/bt_trees/head_arm_independent_lookat.xml` — 2 个 `IsHead/ArmDetectionConfirmed` 替换为 `GetTargetFromAccumulator`
- `src/lgw_bt_planner/bt_trees/head_to_arm_verify_grasp.xml` — 同上
- `src/lgw_bt_planner/CMakeLists.txt` — 新增 `get_target_from_accumulator_bt_node.cpp` 进 `add_executable` 列表
- `package.xml` — 加 `detection_accumulator` 依赖(<build_depend> + <exec_depend>)
- `scripts/start_lgw_head_arm_independent_lookat.sh` / `start_lgw_head_to_arm_verify_grasp.sh` — `docker exec self_l4t ... ros2 launch detection_accumulator detection_accumulator.launch.py` 加到 BT 启动前

---

## 6. Data Flow

### 6.1 Custom Service Definition

```srv
# detection_accumulator/srv/GetTarget.srv
#
# Request: 空
# BT 主动轮询,无 request payload。
# 服务端语义:ARMED 且 fresh → 返 target + reset;其他 → 返 has_target=false。

---

# Response
bool has_target
yoloe_interfaces/ObservationResult target
string status_text
```

### 6.2 累加器 State Machine(per source: head / arm 各自独立)

```
                            first-in-range frame            any get_target call
   COUNTING ──────────────────────── (counter++) ───▶ ARMED ─────────────────▶ COUNTING
   counter < N                                          holds current_target_
   no armed target                                      + last_target_stamp_

   (a) if (counter == N): state := ARMED, last_target_stamp_ = frame.header.stamp
   (b) per-call freshness: if (now - last_target_stamp_) > freshness_threshold_sec_:
       reject (return has_target=false, state := COUNTING, counter := 0)
   (c) fresh: return target, state := COUNTING, counter := 0
```

转移规则(完整):

**on_msg(src, msg)** (YOLOE 帧回调):
1. `picked = pick_first_in_range(*msg, z_threshold_m)`
2. 若 `picked.has_value()`:
   - `current_target_ = *msg`(整个 ObservationResult)
   - `last_target_stamp_ = rclcpp::Time(msg->header.stamp)`
   - `counter++`
   - 若 `counter >= consecutive_n && state == COUNTING`:
     - `state = ARMED`
3. 否则(无 in-range 检测):
   - `counter = 0`
4. 注:ARMED 状态下新帧到达,`last_target_stamp_` 仍更新(累加器保留"最新"为目标),但 `state` 不变。

**on_get_target(src, req, res)** (service handler):
1. lock `state_[src].mutex`
2. 若 `state == ARMED`:
   - `age = now - last_target_stamp_`
   - 若 `age.seconds() > freshness_threshold_sec_`:
     - `res->has_target = false`
     - `res->target = ObservationResult{}`
     - `res->status_text = "armed but stale (age=Xs > threshold=Ys)"`
     - `state = COUNTING; counter = 0`
     - return
   - 否则:
     - `res->has_target = true`
     - `res->target = current_target_`
     - `res->status_text = "armed, delivering"`
     - `state = COUNTING; counter = 0`
     - return
3. 否则(`state == COUNTING`):
   - `res->has_target = false`
   - `res->target = ObservationResult{}`
   - `res->status_text = "counting(counter=N)"`

### 6.3 first_in_range 选目标算法

```cpp
std::optional<YoloeDetection3D> pick_first_in_range(
  const ObservationResult& msg, double z_threshold_m)
{
  // detections[] 假设按 conf 降序(由 YOLOE 内部维护)
  for (const auto& det : msg.detections) {
    if (det.center_m.z < z_threshold_m) {  // camera optical frame z, forward
      return det;
    }
  }
  return std::nullopt;
}
```

**关键语义**:若最高 conf 目标超出 Z 阈值但次高 conf 目标在范围内,选次高。YOLOE 帧间 conf 排序稳定即可。

### 6.4 BT 节点 `GetTargetFromAccumulator` 行为

```cpp
class GetTargetFromAccumulator
  : public BT::RosServiceNode<detection_accumulator::srv::GetTarget>
{
  // providedPorts:
  //   service_name: string (来自 providedBasicPorts, XML 配)
  //   last_observation: OutputPort<ObservationResult>

  bool setRequest(Request::SharedPtr&) override { return true; }  // request 空

  BT::NodeStatus onResponseReceived(
    const Response::SharedPtr& response) override
  {
    if (response->has_target) {
      setOutput("last_observation", response->target);  // BT.CPP 内部绑黑板
      return BT::NodeStatus::SUCCESS;
    }
    // has_target=false: 清黑板(OutputPort 重映射的 key)
    auto it = config().output_ports.find("last_observation");
    if (it != config().output_ports.end() && !it->second.empty()) {
      config().blackboard->set<ObservationResult>(
        it->second, ObservationResult{});
    }
    return BT::NodeStatus::FAILURE;
  }
};
```

---

## 7. Lifecycle / Launch

### 7.1 累加器 Launch File

`detection_accumulator/launch/detection_accumulator.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection_accumulator',
            executable='detection_accumulator_node',
            name='detection_accumulator',
            output='screen',
            parameters=[{
                'consecutive_n': 3,
                'z_threshold_m': 1.5,
                'freshness_threshold_sec': 1.0,
                'head_topic': '/perception/two_stage/camera_head/result',
                'arm_topic':  '/perception/two_stage/camera_arm/result',
                'head_service_name': '/detection_accumulator/head/get_target',
                'arm_service_name':  '/detection_accumulator/arm/get_target',
            }],
        ),
    ])
```

### 7.2 启动脚本改动

`scripts/start_lgw_head_arm_independent_lookat.sh`(示例)新增一行,在 BT 启动命令前:

```bash
# 启动累加器(后台)
docker exec -w /my_ws/l4t/detection_accumulator_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && \
   (nohup ros2 launch detection_accumulator detection_accumulator.launch.py \
    > /tmp/lgw_accumulator.log 2>&1 &)"

# 短暂 sleep 等服务就绪
sleep 2

# 验证 service 可达
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && \
   ros2 service list | grep detection_accumulator" || {
    echo "ERROR: detection_accumulator service not available"
    exit 1
}

# 启动 BT runner(原有命令)
docker exec ... bt_runner ...
```

同样改 `start_lgw_head_to_arm_verify_grasp.sh`。

### 7.3 累加器独立可调试

`ros2 service call /detection_accumulator/head/get_target detection_accumulator/srv/GetTarget` 可独立验证 service,无需 BT。

---

## 8. Error Handling

| 场景 | 累加器行为 | BT 节点行为 |
|---|---|---|
| 累加器未启动 | service 不存在 | `RosServiceNode::onFailure(SERVICE_UNREACHABLE)` → FAILURE → Fallback+ForceSuccess 跳过 |
| 累加器 crash | 同上 | 同上 |
| 累加器 ARMED 但 BT 调用太晚(age > threshold) | 返 has_target=false,reset | FAILURE → 跳过 step |
| YOLOE 停止发 topic | counter 冻结在当前值,last_target_stamp_ 不变 | BT 调用时累加器可能仍在 ARMED 但 age 越来越大,被 freshness check 拒绝;或累加器在 COUNTING,返 false |
| YOLOE 重启 / 切新 session | 累加器状态独立,继续计数;旧 current_target 保留到 N 帧新目标出现或新帧覆盖 | 同上 |
| 多个 BT 同时调 service(不应发生,但理论) | service handler 加 mutex 串行化;两者独立 | BT 端 SyncActionNode 串行,实际不会 |
| msg.header.stamp 异常(epoch 0 / 未来时间) | 累加器 first frame 视为 stale → 拒绝 | per-call freshness 立即 fail |

---

## 9. Testing Strategy

### 9.1 累加器单元测试

`test_first_in_range.cpp`:
- 输入:固定 `ObservationResult` 带 N 个 detections(不同 z 值,不同 conf 序)
- 验证:返回的 detection 索引 + z 阈值边界(empty / single / multiple / front-out-of-range)

`test_state_machine.cpp`(用 mock service 调用 + 模拟 YOLOE 帧):
- 场景 1:counter 不到 N,get_target 返 has_target=false
- 场景 2:counter 到 N → ARMED,get_target 返 has_target=true + 状态 reset
- 场景 3:ARMED 后 1.5s 才 get_target(threshold=1.0s)→ 拒绝,状态 reset
- 场景 4:连续帧中一帧 out-of-range → counter 归零
- 场景 5:first detection out-of-range, second in-range → 选 second,counter 正常累加
- 场景 6:YOLOE 帧间 last_target_stamp_ 持续更新,即使 ARMED 状态不变

### 9.2 BT 节点单元测试

`test_get_target_from_accumulator_bt_node.cpp`(可选,本容器无 ament_cmake_test,跑 gtest 二进制):
- 场景 1:response.has_target=true → OutputPort 写入 + SUCCESS
- 场景 2:response.has_target=false → 黑板键清空 + FAILURE
- 场景 3:service unreachable → FAILURE

### 9.3 集成测试(回放 + 真机)

回放测试:
- 启动累加器(默认参数)
- 启动 YOLOE 回放(用 yoloe_try 现有回放脚本,或录制 bag 回放)
- 启动 BT 用 `head_arm_independent_lookat.xml`
- 验证:每次 get_target 返 has_target=true 的条件;per-call freshness 拒绝的日志;黑板 last_head_obs/last_arm_obs 内容

真机测试:
- 替换原 `start_lgw_*.sh` 中的 IsHead/ArmDetectionConfirmed XML 后跑一轮
- 比对:BT 整体行为应与原 IsHeadDetectionConfirmed 类似,但:
  - 单帧误识别不会 trigger
  - BT 间隔 >1s 不再被喂同 target
  - log 会有 "ARMED / delivering / REJECT stale" 字样

---

## 10. Performance Budget(预计)

| 操作 | 预计开销 |
|---|---|
| YOLOE 帧到达 → first_in_range + counter | O(N) per detection,N<10,**< 10us / frame** |
| get_target service 调用 → 状态机 + freshness check | O(1),**< 1us** |
| 累加器 ROS node 内存 | 2 个 SourceState struct,**< 1 KB** |
| YOLOE topic QoS history | 默认 5(depth),**几十 KB** |
| 启动累加器 → 服务就绪 | node init + 2 service + 2 sub,~**1-2s** |

性能瓶颈在 YOLOE 本身(已知 CPU bound),不在累加器。

---

## 11. Risks + Open Questions

### 11.1 Risks

| Risk | 缓解 |
|---|---|
| YOLOE detections[] 帧间 conf 序不稳定 | first_in_range 仍按当前帧顺序选,与 YOLOE 内部保证一致;若不稳定用户改用按 z 升序选 |
| `rclcpp::Time(msg->header.stamp)` 在 dual_camera_node 里用的是 realsense capture time(确认过,见 `dual_camera_node.py:867-871 copy_header`),跨容器 / 跨设备时钟漂移可能让 freshness 误判 | 累加器和 BT 在同一容器同源时钟,目前无问题;真机有问题再考虑用 `node->now()` 替代 |
| 累加器 service 长时间不被调用,累加器空跑 | 无副作用,资源占用可忽略;不算 risk |
| 累加器 vs BT 启动顺序 | start_lgw_*.sh 先起累加器,`ros2 service list` 验证后再启 BT;若累加器晚于 BT,BT 端 service 不可达,fallback 跳过 step(BT 不挂) |
| `detection_accumulator` 包是新建,在 `l4t/detection_accumulator_ws/` 单独 build,需保证 source 后能 link 到 `yoloe_interfaces` | `l4t_setup.sh` 已 source `yoloe_deploy` 安装路径;若失败,改 `colcon build --packages-select detection_accumulator` 时显式 `--cmake-args -Dyoloe_interfaces_DIR=...` 或在 setup.sh 加 `AMENT_PREFIX_PATH` |
| mutex 粒度:每个 source 独立 mutex,简单但无法保证跨 source 一致性(本设计不需要) | 接受 |
| BT OutputPort "last_observation" 写黑板时,若黑板键已被下游节点读到 stale 值,直到下次 SUCCESS 才会更新 | 与现状 IsHeadDetectionConfirmed 语义一致;可接受 |

### 11.2 Open Questions

- 是否在 launch 里默认 `freshness_threshold_sec=0.3`(沿用 max_age_sec)还是 `1.0`(放宽容错)?— spec 默认 1.0,具体在 launch 文件和 ROS param 默认值上可调。
- 是否需要在累加器里加一个 `/detection_accumulator/diagnostics` topic 输出当前 counter / state / age?便于调参观察。**v1 不加**,后续有需要再补。
- `IsHead/ArmDetectionConfirmed` + `evaluate_detection` 是否保留(向后兼容)?— 本 spec 不删,后续 cleanup 可选。

---

## 12. Decision Log

### 12.1 服务接口:BT 是 service client,累加器是 server

- 候选:topic 双向 / ROS service / Action
- 决策:**2 个 ROS service**(head / arm 各自 1 个),BT 用 `RosServiceNode` 调
- 理由:BT.ROS2 只提供 `RosServiceNode<ServiceT>` client 角色,无 server 基类;topic 双向更绕,action 模式过重
- 时间:2026-10-08 brainstorming

### 12.2 BT 节点:1 个 C++ 类 + 2 个 XML 实例

- 候选:每 source 各 1 个 class(沿用 IsHead/ArmDetectionConfirmed 模式) / 1 class 2 实例
- 决策:**1 class + 2 XML 实例**
- 理由:`RosServiceNode` 的 `service_name` 是运行时端口,无需模板参数化;1 class 减少重复
- 时间:2026-10-08 brainstorming

### 12.3 重触策略:per-call freshness(非 cooldown,非 instance_id)

- 候选 A:持续可领,与现状 IsHeadDetectionConfirmed 一致
- 候选 B:Cooldown,交付后 N 秒内不再 ARMED
- 候选 C:Instance ID 去重,同 instance_id 不重复
- 候选 D:Per-call freshness — 服务请求时间与上次检测结果时间过久时拒绝
- 决策:**候选 D**
- 理由:用户明确要求"服务请求时间与上次得到检测结果的时间过久时,该结果不接受";语义上等价于把 `max_age_sec` 从 BT 节点搬到累加器
- 时间:2026-10-08 brainstorming

### 12.4 包结构:新建独立 workspace

- 候选:合并到 lgw_bt_planner(同进程) / 新建 detection_accumulator 独立 workspace
- 决策:**新建独立 workspace `l4t/detection_accumulator_ws/`**
- 理由:累加器可独立 `ros2 service call` 调试;生命周期解耦;未来可能加更多 source 也不挤 lgw_bt_planner
- 时间:2026-10-08 brainstorming

### 12.5 多检测 + Z 阈值:first-in-range

- 候选:strict front-only / first-in-range(按 conf 序选第一个 z<thr) / 每检测独立跟踪
- 决策:**first-in-range**
- 理由:用户原意"看到目标",不强制最高 conf;若最高 conf 目标超出工作距离,选次高
- 时间:2026-10-08 brainstorming

### 12.6 黑板清除时机:下次 get_target 调用时清

- 候选:只写不清 / 下次 get_target 返 false 时清 / 单独 Ack 节点显式清
- 决策:**下次 get_target 返 false 时,BT 节点主动清空 OutputPort 映射的黑板键**
- 理由:无显式 ack 通道,清黑板与"重新领"是同一动作;实现简单
- 时间:2026-10-08 brainstorming

---

## 13. References

- `l4t/lg_walker_ws/src/lgw_bt_planner/src/is_detection_confirmed_node.cpp` — 当前 BT 节点实现
- `l4t/lg_walker_ws/src/lgw_bt_planner/src/evaluate_detection.cpp` — 当前 freshness 检查
- `l4t/lg_walker_ws/src/lgw_bt_planner/bt_trees/head_arm_independent_lookat.xml` — 待修改 XML
- `l4t/lg_walker_ws/src/lgw_bt_planner/bt_trees/head_to_arm_verify_grasp.xml` — 待修改 XML
- `l4t/bt_ros2_ws/install/behaviortree_ros2/include/behaviortree_ros2/bt_service_node.hpp` — BT::RosServiceNode 基类
- `/home/unitree/yoloe_try/humble_ws/src/yoloe_interfaces/msg/ObservationResult.msg` — 输入消息
- `/home/unitree/yoloe_try/humble_ws/src/yoloe_interfaces/msg/YoloeDetection3D.msg` — 单 detection 字段
- `yoloe_deploy/ros2_ws/src/yoloe_perception/yoloe_perception/dual_camera_node.py:867-871` — `header.stamp = realsense color capture time`(`copy_header` 行为)
- `l4t/lg_walker_ws/docs/superpowers/specs/2026-10-06-d435-gst-pipeline-design.md` — 同目录 spec 格式参考

---

## 14. 后续(实现期)

本 spec **仅记录设计,待用户后续 trigger 实现**。实现期走 `superpowers:writing-plans` skill 出详细实现计划,再走 TDD 实施。

预计实现顺序(用户后续触发时用):
1. `detection_accumulator_ws` 工作空间 init + `GetTarget.srv` + `package.xml` / `CMakeLists.txt`
2. `detection_accumulator_node` + `first_in_range` 单元测试
3. 累加器 launch + start script 集成
4. `GetTargetFromAccumulator` BT 节点 + 单元测试
5. XML 替换 + bt_plugin_registration
6. 集成测试(回放 + 真机 smoke)
