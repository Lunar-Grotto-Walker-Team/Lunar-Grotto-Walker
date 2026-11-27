# Body Pose Bridge — Design

**Status**: Draft (待用户评审)
**Date**: 2026-10-10
**Author**: Claude (design session with user; follow-up to go2_posture_demo 失败排查 2026-10-10)
**Workspace**: 修改 `l4t/lg_walker_ws/`
**Goal**: 把 `SetBodyPoseNode`(当前是 `BT::StatefulActionNode`,内部订阅 lf/sportmodestate + 50Hz 验证 + 三连发命令)拆成两层 —— **持久化 rclcpp 桥接节点**(`body_pose_bridge`)持有 timer 与持续 publish,BT 节点只发一次性 RPC。这解决 Go2 firmware Euler(1007) 命令**瞬时性**(1s 内被 balance controller 拉回中立)导致的"狗不保持目标姿态"问题。

---

## 1. Background

### 1.1 现状

`src/lgw_bt_planner/bt_trees/go2_posture_demo.xml` 当前路径:

```
StandUp → SetBodyPose(tilt 30°) → Sleep 2s → SetBodyPose(recenter) → StandDown
```

`SetBodyPoseNode`(src/set_body_pose_node.cpp,~300 行 `BT::StatefulActionNode`)的当前实现:

- onStart: publish 三连发 `Euler(1007) → BodyHeight(1013) → BalanceStand(1002)`,启动 deadline
- onRunning: 每 tick 读 `lf/sportmodestate`,5 个连续 tick rpy 在 tolerance 内 → SUCCESS;超 5s → FAILURE
- onHalted: publish `StopMove(1003)`

**两个真机验证发现的问题:**

1. **Euler 命令瞬时性**(2026-10-10 验证):Go2 firmware 把 Euler(1007) 视作"瞬时姿态调整指令"而非"目标姿态保持指令"。即使 BT 通过 `Euler → BodyHeight → BalanceStand` 三连发正确顺序让 firmware 在 BalanceStand 时 latch 当前 rpy,firmware 内部 balance controller 仍会在 ~1s 内把姿态拉回 (0, 0, 0)。验证日志:`t=3.4s pitch=-0.355`(响应正确),`t=3.9s pitch=-0.042`(被拉回),`t=5s` 5 个连续 tick 永远拿不到 → BT FAILURE。
2. **Pitch 符号约定反转**(2026-10-10 验证):XML 写 `pitch="-0.5236"`(标准右手法则 nose-down),实机 dog 后仰(nose-up)。Go2 firmware 的 pitch 符号与标准右手法则相反 —— `pitch>0` = nose down,`pitch<0` = nose up。这条与本 spec 无关,但需要同步修。

**当前 SetBodyPose 失败的根因是 #1**:`BT::StatefulActionNode` 一次性 publish + 短超时验证,在硬件需要"持续保持"的语义下**结构性不匹配** —— BT 节点不该扮演 controller 的角色(也是 CLAUDE.md 已知反模式第一条)。

### 1.2 解决方案候选

| 方案 | 描述 | 取舍 |
|---|---|---|
| α:BT 节点内 50Hz republish | 改 `set_body_pose_node.cpp`,创建 `rclcpp::WallTimer`,50Hz republish 三连发 | 触发 CLAUDE.md 反模式(`rclcpp::Rate` / `WallRate`);BT 节点作为决策节点,持 timer 语义不对 |
| β:独立桥接节点 + service(set-and-hold,本 spec) | 新包 `body_pose_bridge` 提供 `/set_body_pose` service,内部 rclcpp::Node 持有 50Hz timer 直到新调用;BT 节点改为 `RosServiceNode` 客户端 | **已选** —— 跟 `go2_twist_bridge` 拓扑对称;bridge 跟 BT 解耦,可单测;复用 ROS service 模式;无需 duration_sec,语义清晰"调一次 = 持续 hold 直到下一次" |
| γ:切换 LowCmd | 跳过 Sport API,直接 `/lowcmd` 控制 12 关节角 | 完全绕过 balance controller,但失去 IMU 反馈保护;需要 gait state machine;复杂度爆炸,不在本 spec 范围 |

### 1.3 为什么选 β(独立桥接节点)

参考已存在的 `go2-nav2-amcl/src/base/go2_twist_bridge/src/twist_bridge.cpp`(~57 行,纯 rclcpp::Node + cmd_vel subscriber → /api/sport/request publisher)。它解决"nav2 发 cmd_vel 高频但 Go2 私有 API" 的桥接问题。Body pose 的需求结构上完全相同,只是**节奏源从 nav2 移到 bridge 自己的 timer**(因为 BT 不持续发)。

---

## 2. Goal / Non-goals

### 2.1 Goal

1. 新建 `l4t/lg_walker_ws/src/body_pose_bridge/` 包,提供 rclcpp::Node 进程 `body_pose_bridge_node`。
2. 定义 ROS 2 service `SetBodyPose.srv`(5 字段,见 §4.1),bridge 作为 service server。
3. Bridge 内部:收到 service 调用 → 50Hz timer 启动 → 每 tick 三连发 `Euler → BodyHeight → BalanceStand` → 验证 `lf/sportmodestate.rpy` 在 tolerance 内稳定 `kStableFrames` tick → 回 SUCCESS → timer **不停**(持续 hold)。新 service 调用替换目标,不创建新 timer。
4. `SetBodyPose` 请求携带 `bool clear=true` 时,bridge 取消 timer 并发一次 `Euler(0,0,0) + BodyHeight(0) + BalanceStand` 收尾,返回 SUCCESS(给后续 StandDown / Move 让路)。
5. 现有 `src/lgw_bt_planner/src/set_body_pose_node.cpp` 由 `BT::StatefulActionNode` 改写为 `BT::RosServiceNode<SetBodyPose>` 客户端,~80 行。
6. `go2_posture_demo.xml` 在 recenter 节点后插入 `<SetBodyPose clear="true"/>`;同时把 `pitch="-0.5236"` 改为 `pitch="+0.5236"`(实机符号修正)。
7. `bt_plugin_registration.cpp` 注册名 `SetBodyPose` 不变(已注册)。
8. gtest 覆盖 bridge 状态机 + BT 节点 2 个返回路径。
9. 真机回归:`go2_posture_demo.xml` 完整跑通 —— dog 前倾 30° hold 2s → 自动回正 → 趴下。

### 2.2 Non-goals

1. **不迁** StandUp / StandDown / BalanceStand / SetBodyHeight 到 bridge —— 它们是**瞬时触发**(Go2 firmware 收到 1004/1005 后会持续保持站立/趴下状态,不需要外部 republish),现有 BT 节点已经能正确处理。YAGNI:只为"持续保持"这一新问题建桥。
2. **不优化** lf/sportmodestate 订阅器实现(沿用 `SetBodyPoseNode` 现有的 `rclcpp::SensorDataQoS` + mutex 缓存)。
3. **不引入** duration_sec 参数 —— 用户明确拒绝,语义改成"调一次 = 持续 hold 直到新调用"。
4. **不发** `StopMove(1003)` 作为 clear 收尾 —— 1003 会让 dog 从 BalanceStand 退到普通站立,可能反而失稳。Clear 路径发 1 个 `Euler(0,0,0) + BodyHeight(0) + BalanceStand` 即可。
5. **不重写** `go2_twist_bridge` —— 那个项目独立维护,与本 spec 无功能重叠。
6. **不引入** ROS 2 lifecycle / managed node —— bridge 启动后常驻,故障靠 watchdog 重启。
7. **不实现** 与 `twist_bridge` 的协商 / 互斥 —— 如果未来需要 body_pose + cmd_vel 共存,需要额外 spec。

---

## 3. Design Constraints

### 3.1 三层架构(参考 2026-10-09 spec §3.1)

| 层 | 类型 | ROS 依赖 | 单元测试路径 |
|---|---|---|---|
| 持久化桥接节点 | `rclcpp::Node` + `rclcpp::ServiceServer` + `rclcpp::WallTimer` | 全套(`rclcpp`, `unitree_api`, `unitree_go`) | gtest 集成测试(真 service server + 临时 publisher) |
| ROS service 接口 | `srv/SetBodyPose.srv` | 仅 `builtin_interfaces` | 由 rosidl 生成 |
| BT 薄壳 | `BT::RosServiceNode<lgw_bt_planner::srv::SetBodyPose>` | BT.ROS2 | gtest mock service |

### 3.2 关键调用约束(2026-10-10 lesson)

`unitree_sdk2/example/go2/go2_sport_client.cpp:63-67` 注释:`balance_stand (controlled by dBodyHeight + rpy)` —— Go2 firmware state machine 在收到 `BalanceStand(1002)` 时 latch 最近一次 `Euler(1007)` + `BodyHeight(1013)` 作为初始姿态。

**因此三连发的 order 是 hard constraint**:

```
Euler(1007) → BodyHeight(1013) → BalanceStand(1002)
```

任何颠倒都会被 firmware 吞掉。本 spec 所有 publish 路径都要遵循这个顺序,bridge 顶层注释必须显式标注(同 2026-10-09 spec 的"call sequence 文档化"原则)。

### 3.3 Bridge 状态机

```
            service received       verifypassed (5 ticks)        new service received
              (verify)               (continue)            timer reset & timer
                                                                         │
IDLE  ───────────────────────→  HOLDING  ──────────┐      ┌─────────────│────────────┐
  ↑                                  │            │      │             │            │
  │                            clear=true  ┌─────┴──┐   │             │            │
  │                                  ▼      │        │   │             │            │
  │                              CLEARING  │        │   │             │            │
  │                                  │      ▼        ▼   ▼             ▼            │
  │                                  │   publish timer cleared         │            │
  │                                  │   send final sequence           │            │
  │                                  │   return SUCCESS                │            │
  │                                  │                                  │            │
  └──────────────────────────────────┴──────────────────────────────────┘
                                  (timer cancelled)
```

**状态说明**:

- **IDLE**:timer 不存在,目标 rpy = (0, 0, 0),body_h_offset = 0,最近一次 publish 是 BalanceStand(1002) 无 rpy(初始默认)
- **HOLDING**:timer 存活 50Hz,目标 = 当前 service 调用值,每 tick 三连发,verification 计数到 5 → return SUCCESS 但**仍 stay in HOLDING**
- **CLEARING**:timer 取消,发一次三连发 rpy=0,body_h=0,BalStand → return SUCCESS → 立即转 IDLE

**关键不变量**:HOLDING 期间 timer 持续 50Hz publish,无论 verification 是否过。这是 1.1 #1 的根因对策。

---

## 4. Design

### 4.1 Service 接口 `srv/SetBodyPose.srv`

```
# request
float64 roll         # 目标 roll(rad),标准右手法则
float64 pitch        # 目标 pitch(rad),Go2 实机 >0 = nose down
float64 yaw          # 目标 yaw(rad)
float64 body_h       # BodyHeight 相对偏移(m),正=drop 负=raise
bool clear           # true = 取消 hold 并回正,此时 rpy/body_h 字段被忽略
---
# response
bool success
string message       # 小写英文短语,失败原因
```

**`clear=true` 语义**:

| `clear` | rpy/body_h | 行为 |
|---|---|---|
| false(默认) | 任意 | 进入 HOLDING,50Hz republish 当前调用值,验证到达返回 SUCCESS,持续 hold |
| true | **忽略** | 进入 CLEARING,timer 取消,发一次三连发(0,0,0,0),返回 SUCCESS,转 IDLE |

**`message` 字段约定**(失败枚举):
- `"duration_must_be_positive"`(预留,当前 caller 不需要;若启用 `hold_duration_sec` 则用)
- `"service_timeout"`
- `"imu_state_not_received"`
- `"clear_already_idle"`(冗余 clear 时,info 级,仍 SUCCESS)
- `"ok"`(SUCCESS 时 message 也填这个便于日志聚合)

### 4.2 Bridge 节点 `body_pose_bridge::BodyPoseBridgeNode`

**文件结构**:
```
src/lgw_walker_ws_feature_detect_and_move/src/body_pose_bridge/
├── CMakeLists.txt              # ament_cmake + rclcpp + unitree_api + unitree_go
├── package.xml                 # 名字 = body_pose_bridge,buildtool = ament_cmake
├── include/lgw_body_pose_bridge/
│   └── body_pose_bridge_node.hpp
├── src/
│   └── body_pose_bridge_node.cpp
└── launch/
    └── body_pose_bridge.launch.py
```

**成员**:

```cpp
class BodyPoseBridgeNode : public rclcpp::Node
{
public:
  explicit BodyPoseBridgeNode(const rclcpp::NodeOptions& opts);

  // service handler
  void handle_set_body_pose(
    const std::shared_ptr<SetBodyPose::Request> req,
    std::shared_ptr<SetBodyPose::Response> res);

private:
  // 50Hz publish callback:publish 三连发 + verify stable_count + notify cv_
  void publish_target_pose();

  // clear 收尾:发一次 rpy=0 + body_h=0 + BalStand
  void send_final_balanced_pose();

  // state subscriber callback
  void state_cb(const unitree_go::msg::SportModeState::SharedPtr msg);

  // ---- members ----
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;

  rclcpp::Service<SetBodyPose>::SharedPtr service_server_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;  // 隔离 timer 与 service
  rclcpp::WallTimer::SharedPtr publish_timer_;

  std::mutex state_mutex_;
  unitree_go::msg::SportModeState latest_state_;
  bool state_received_{false};

  std::mutex target_mutex_;       // 保护 target 与 timer 生命周期
  float target_roll_{0.0f};
  float target_pitch_{0.0f};
  float target_yaw_{0.0f};
  float target_body_h_offset_{0.0f};  // 正=drop,负=raise
  float tolerance_rpy_{0.05f};
  float tolerance_body_h_{0.02f};
  int stable_count_{0};             // 连续验证 tick 数

  enum class State { IDLE, HOLDING, CLEARING };
  State state_{State::IDLE};
  bool state_was_idle_on_entry_{false};  // clear 路径记录,用于 message

  std::condition_variable cv_;     // handler 等待 verification 时阻塞用
};
```

**关键函数**:

**`handle_set_body_pose(req, res)`**(同步 handler,等 verification 成功 / clear 完成才 return)。理由:
- service handler 阻塞 0.5-5s 是 OK 的(rclcpp::Service 不卡死,只是 service executor 线程被占);本场景 service 调用频率极低(BT 一次只调一次),不构成吞吐问题
- BT.ROS2 的 `RosServiceNode` 默认就是 sync 等 future,跟这个模型契合
- 避免异步 + future/promise 的复杂度 + rclcpp::Service "晚回包"路径的边界处理

Handler 实现 sketch:

```cpp
void handle_set_body_pose(req, res) {
  std::unique_lock<std::mutex> lock(target_mutex_);
  if (req->clear) {
    // clear 路径:取消 timer,发一次三连发 rpy=0 + body_h=0 + BalStand,转 IDLE
    if (state_ != State::IDLE) {
      publish_timer_.reset();  // 取消 50Hz republish
      send_final_balanced_pose();
    }
    state_ = State::IDLE;
    res->success = true;
    res->message = (state_was_idle_on_entry_) ? "clear_already_idle" : "ok";
    return;
  }

  // 更新目标(无论当前是 IDLE 还是 HOLDING,目标都要换)
  update_target_locked(req);

  if (state_ == State::IDLE) {
    // IDLE → HOLDING:启动 50Hz timer
    state_ = State::HOLDING;
    stable_count_ = 0;
    publish_timer_ = create_wall_timer(20ms,
        [this]{ publish_target_pose(); },
        timer_cb_group_);
  } else {
    // HOLDING → HOLDING(live update):目标已换,重置 verification 计数
    // 不重启 timer(还在跑,只是 publish_target_pose() 读的是新 target)
    stable_count_ = 0;
  }

  // 等 verification 达 kStableFrames(5 tick)或超时(5s)
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  cv_.wait_until(lock, deadline, [this]{
    return stable_count_ >= kStableFrames;
  });
  res->success = (stable_count_ >= kStableFrames);
  res->message = res->success ? "ok" : "service_timeout";
  // ⚠️ 注意:这里不主动 cancel timer,验证后 bridge 继续 HOLDING(§3.3 状态机)
}
```

**支持的状态转换**(handler 视角):

| 调用 | state 进入 | 退出 | 后续 |
|---|---|---|---|
| `clear=true, state=IDLE` | IDLE → IDLE | message="clear_already_idle" | 下次 set 走 IDF → HOLDING |
| `clear=true, state=HOLDING` | HOLDING → IDLE(timer 取消) | message="ok" | 下次 set 走 IDF → HOLDING(timer 重启) |
| `set(pitch=X), state=IDLE` | IDLE → HOLDING(timer 启动,stable=0) | 等 verify,message="ok"/"service_timeout" | 仍在 HOLDING,新 set 走 live update |
| `set(pitch=X), state=HOLDING` | HOLDING → HOLDING(target 换,stable 重置) | 等 verify,message="ok"/"service_timeout" | 仍在 HOLDING,新 set 走 live update |

**关键不变量**:handler 用 `std::condition_variable::wait_until` 限时等,5s 内 `stable_count_` 不到 5 必返回 —— 避免 BT halt 时 handler 永久阻塞。

**`publish_target_pose()` sketch**(每 20ms tick):

```cpp
void publish_target_pose() {
  // 1. publish 三连发(Euler → BodyHeight → BalStand)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    publish_euler(target_roll_, target_pitch_, target_yaw_);    // api_id=1007
    publish_body_height(target_body_h_offset_);                // api_id=1013
    publish_balance_stand();                                    // api_id=1002
  }

  // 2. 读 latest_state_,算 diff
  unitree_go::msg::SportModeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) return;  // 还没收到 state,verification 跳过
    state = latest_state_;
  }

  bool at_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    at_target = is_within_tolerance(state, target_*, tolerance_*);
    if (at_target) {
      if (++stable_count_ >= kStableFrames) {
        cv_.notify_all();  // ← 唤醒 handler 继续
      }
    } else {
      stable_count_ = 0;  // 出 tolerance,重置
    }
  }
}
```

**关键**:`cv_.notify_all()` 在 `stable_count_ >= 5` 时调用,持 `target_mutex_` 的 timer 与持同一 mutex 的 handler(在 wait_until 中释放锁等待)互不饿死。

## 4.2.1 完整状态转换汇总

Handler 视角下,4 类调用 × 2 类 state 进入 = 9 种转换路径(其中 1 类无转换):

| # | 调用 | state 进入 | 内部动作 | 退出 state | response |
|---|---|---|---|---|---|
| 1 | clear=true | IDLE | (nothing, message=clear_already_idle) | IDLE | success=true |
| 2 | clear=true | HOLDING | timer.reset(); send_final_balanced_pose() | IDLE | success=true, message=ok |
| 3 | set(pitch=X) | IDLE | state=HOLDING; stable=0; create timer; wait verify | HOLDING | success=?/timeout |
| 4 | set(pitch=X) | HOLDING | target 更新; stable=0; 不重启 timer; wait verify | HOLDING | success=?/timeout |

**用户原问 "clear 后还能不能 set":第 1 行之后接第 3 行,完全支持**。set 返回后 bridge 仍 HOLDING,如果 BT 想再换目标走第 4 行(live update);想完全收尾走第 2 行(clear)。整套循环:`set → set(live update) → set(live update) → clear → set → clear → ...` 全部支持。

### 4.3 BT 节点改造

**当前**:`set_body_pose_node.cpp` ~300 行 `BT::StatefulActionNode`,内部订阅 + 发布 + timer + verification。

**目标**:`~80 行 `BT::RosServiceNode<lgw_bt_planner::srv::SetBodyPose>` 客户端,只负责:
1. 构造 request(从 BT port 读 rpy/body_h/clear/tolerance_rpy/tolerance_body_h,塞入 req)
2. async_send_request → spin_until_future_complete(...)
3. response 回来后判断 success,返回 BT NodeStatus::SUCCESS / FAILURE

**端口**(全部保留,**`clear` 是新增的,默认 false**):

| Port | Type | Default | 用途 |
|---|---|---|---|
| `roll` | double | 0.0 | 目标 roll |
| `pitch` | double | 0.0 | 目标 pitch |
| `yaw` | double | 0.0 | 目标 yaw |
| `body_height` | double | 0.0 | 相对偏移 |
| `clear` | bool | false | **新增** —— true 时桥接 cancel hold |
| `tolerance_rpy` | double | 0.05 | 透传给 service,bridge 内部用 |
| `tolerance_body_h` | double | 0.02 | 透传给 service,bridge 内部用 |

**XML 不需要改端口声明**(BT.ROS2 自动 introspect 节点 providedPorts)。XML 实例需要改:
- `pitch="-0.5236"` → `pitch="+0.5236"`(实机符号)
- 在 recenter 后插 `<SetBodyPose clear="true"/>`(无 rpy 字段,因为 clear=true 时被忽略)

**`clear` 默认 false**:BT 现有调用不传 `clear` 走默认,与原行为一致。

### 4.4 launch / lifecycle

- `body_pose_bridge/launch/body_pose_bridge.launch.py` 启动 bridge 节点(`single_node_launch`)。
- **不**自动在 `lgw_bt_planner` 的总 launch 里加。理由:bridge 不是所有 BT tree 都需要(只有用 SetBodyPose 的 tree 才要),由调用方显式启动,失败时单独报错。
- 具体由 `go2_posture_demo.xml` 的 launch wrapper 加 bridge 启动前置项。

### 4.5 真机行为约定(写进 bridge doc 顶头注释)

1. **call sequence**:`Euler(1007) → BodyHeight(1013) → BalanceStand(1002)`,颠倒会被 firmware 吞。
2. **持续保持**:每次 Bridge 进入 HOLDING,timer 持续 50Hz publish 直到收到新 service 调用。**绝不停**。
3. **clear**:只发 1 次三连发(0,0,0,0)+ BalStand,**不发** (避免 1003 触发普通站立)。
4. **StandDown 兼容性**:bridge 在 HOLDING 期间会**锁 Go2 在 BalanceStand + 自定义 rpy**,此时调 StandUp/StandDown/Move **都失效**。XML 流程必须在 StandDown 之前先发 `<SetBodyPose clear="true"/>`。
5. **符号约定**:pitch 右手法则。Go2 实机 `pitch>0` = nose down,与标准约定相反(已在 2026-10-10 真机验证)。XML 实例必须按实机符号写。

---

## 5. Error Handling

| 故障 | 现象 | 处理 |
|---|---|---|
| Bridge 未启动 | service call 失败(`unavailable`) | BT 节点 1s timeout 返回 FAILURE |
| Service handler 内 verification 永远不达标 | cv_.wait 触发 5s timeout | service response `success=false, message="service_timeout"`,BT FAILURE |
| Service handler 内 verification 达标但 dog 仍漂 | rpy 反复进出 tolerance | timer 持续,verification 连续 5 tick 达标 → response success,BT SUCCESS;之后仍 HOLDING |
| 多次连续 clear=true | 第二次 clear 时 state 已 IDLE | 跳过 sent_final_balanced_pose()(no-op),直接 `success=true, message="clear_already_idle"` |
| BT 在 service pending 中 halt(BT tick 被抢占) | bridge service handler 阻塞中 | rclcpp::Service 默认超时 10s,client 端 timeout=5s。Service handler cv_.wait 用 5s timeout,handler 不会卡死 |
| 真机通信断(cyclonedds 丢 / Go2 firmware 卡) | lf/sportmodestate 4-5s 无新 msg | timer 持续 50Hz publish 不停(等它恢复),不影响 service handler(verification 看 stable_count_,超时走 timeout) |
| Bridge 进程被 kill | timer 消失,dog 失去外部控制 | Go2 firmware 自然接管 balance(漂回中立);后续 StandUp/StandDown 恢复响应。这是 fail-safe 行为 |
| Bridge 启动顺序晚于 BT 调用 | 第一个 service 调用 unavailable | BT 节点 1s 内 3 次 retry(沿用现有 `RetryUntilSuccessful num_attempts=3`) |

**Bridge 进程重启**(重要):Bridge 重启 = timer 消失 = dog 失去 rpy 持续保持。如果 BT 仍处于 SetBodyPose 节点 BLOCKING 在 service future,重启后 client 收到 `unavailable`,BT FAILURE → 进入 RetryUntilSuccessful 恢复路径。这是合理的 fail-over 行为,本 spec 不做额外处理。

---

## 6. Testing

### 6.1 单元测试 (gtest,本容器直接跑二进制)

| Test | 内容 | 通过判据 |
|---|---|---|
| `body_pose_bridge_state_machine_test` | mock publisher(写到位数后回写 lf/sportmodestate)、调 service `SetBodyPose{pitch=+0.5236}`、推进虚拟时钟 | service handler 在 ~5*20ms = 100ms 内回 `success=true, message="ok"`;3-call sequence 顺序正确(Euler→BodyHeight→BalStand);handler 返回后 timer 仍存活(状态在 HOLDING) |
| `body_pose_bridge_clear_test` | 先发 `SetBodyPose{pitch=+0.5236}`、再发 `SetBodyPose{clear=true}` | 第二次 service handler 立即回 `success=true`;3-call sequence 是 `(0,0,0,0) + BalStand`,不是 tilt |
| `body_pose_bridge_timeout_test` | mock publisher 不回写 lf/sportmodestate | service handler 5s timeout 回 `success=false, message="service_timeout"` |
| `set_body_pose_action_node_test` | mock service client,BT::BehaviorTreeFactory 装载新 SetBodyPoseNode | rpy 输入正确传到 request;response.success → BT SUCCESS;response.success=false → BT FAILURE;clear=true → request.clear=true(透传) |

### 6.2 集成测试 (gtest,需要 rclcpp::init)

| Test | 内容 | 通过判据 |
|---|---|---|
| `body_pose_bridge_integration_test` | 起真 `BodyPoseBridgeNode` + 临时 node 当 client,记录 /api/sport/request 序列 | 50Hz 频率、3-call 顺序、clear 触发时机符合 §4.5 |

### 6.3 真机回归(`go2_posture_demo.xml`)

| 测试 | 内容 | 通过判据 |
|---|---|---|
| **回归 1**:完整 demo 跑一遍 | stand-up + tilt + sleep 2s + recenter + clear + stand-down | dog 完整跑通,前倾 30° 方向正确(nose-down,不是后仰),2s 后回正,最后趴下 |
| **回归 2**:去掉 `<SetBodyPose clear="true"/>` | 仅跑 stand-up + tilt + sleep + recenter,然后直接 stand-down | recenter 后 bridge 仍在 HOLDING(target=0,持续 50Hz BalStand),StandDown 发出的 1005 被覆盖,**dog 不会趴下** —— 这条 case 验证 clear 节点的必要性 |
| **回归 3**:只发 `<SetBodyPose clear="true"/>`,不发前导 | bridge 已经在 IDLE,clear_already_idle | service response `success=true, message="clear_already_idle"`,dog 无变化 |

**回归 2 的关键意义**:确认 §4.5 #4 的"StandDown 兼容性"警告 —— bridge HOLDING 期间 StandDown 失效。

### 6.4 Launch 验证

| 测试 | 内容 | 通过判据 |
|---|---|---|
| 启动 `body_pose_bridge.launch.py` 不带 BT | bridge 单独跑 | `ros2 node list` 可见 `body_pose_bridge_node`;`ros2 service list` 可见 `/set_body_pose`;`ros2 topic info /api/sport/request` 显示 1 个 publisher |
| 启动 `body_pose_bridge.launch.py` + `bt_pilot` | bridge 与 BT 共跑 | service 可被 BT 调用,无端口冲突 |

---

## 7. Open Items / Parallel Work

### 7.1 不在本 spec,但同步要做的硬修正

**XML pitch 符号翻转**:`go2_posture_demo.xml` 当前 `pitch="-0.5236"` 改为 `pitch="+0.5236"`(Go2 实机 `pitch>0` = nose down)。

**时序**:本 spec 实施后做一次回归即可同时验证符号 + bridge;如果赶进度,**可以现在就改 + 真机 30s 验证 dog 前倾方向** → 这条独立于本 spec,不推迟 spec 评审。

### 7.2 暂不在本 spec,留作未来

1. **bridge 与 twist_bridge 协商**:如果未来出现"边走边调姿态"场景(如 nav2 导航时根据环境自适应降高),需要重新设计 pub 频次 + 互斥锁。本 spec 不涉及。
2. **bridge 自动 recovery**:bridge 进程被 kill 后的 fail-over。本 spec 接受 dog 漂回中立的 fail-safe 行为。
3. **可视化 topic**:当前 bridge 不发 `/body_pose_state` 等状态 topic;如果未来需要 RViz/MVUI 可视化当前实际姿态,新增 topic。
4. **`hold_duration_sec` 字段**:用户已明确拒绝(`不需要这一项参数`)。如果未来出现"hold 有限时间后自动回正"需求,加字段即可,与本 spec 设计兼容。
5. **StandUp / StandDown 迁桥**:YAGNI,本 spec 拒绝。如果未来 nav2 调起 StandUp 失败率提高,再议。

### 7.3 风险

1. **rclcpp::Service 同步 handler 阻塞 timer_cb_group**:隔离 callback group 已设计,但需在 launch 中验证 timer 与 service handler 不会互相饿死(rclcpp executor 默认 Reentrant 在单线程 MultiThreaded 下)。
2. **service handler 5s timeout 与 tolerance**:真机 rpy 可能在 5s 内不达标(imu_state 反馈延迟、Go2 内部控制环慢),如果 5s 不够需要可配。
3. **CV (std::condition_variable) 在多 callback group 的唤醒 ——** 已经在 §4.2 锁定:timer 触发 publish_target_pose → 检查 stable_count_ → 达标时 `cv_.notify_all()`,service handler `cv_.wait` 返回。timer 在 timer_cb_group,service 在 service_cb_group,notify 是 thread-safe 的(std::condition_variable 标准保证)。

---

## 8. Acceptance Criteria

本 spec 通过条件(用户 review 后):
1. spec 文件已经 commit 到 git(本仓 `l4t/lg_walker_ws/docs/superpowers/specs/`)。
2. 用户 review 完 spec,**显式说"approved"**。
3. writing-plans skill 已经出实施计划,用户 approve plan。
4. 实施完成,§6.1-6.3 全部测试通过。
5. `go2_posture_demo.xml` 真机端到端跑通(狗前倾 30° 方向正确,hold 2s,回正,清桥,趴下)。

---

## 9. References

- `unitree_sdk2/example/go2/go2_sport_client.cpp:63-67` —— `balance_stand (controlled by dBodyHeight + rpy)` 注释,call sequence 约束的源头
- `l4t/go2-nav2-amcl/src/base/go2_twist_bridge/src/twist_bridge.cpp` —— 同模式参考实现
- `docs/superpowers/specs/2026-10-09-select-arm-observation-pose-spec.md` —— 同仓库 spec 格式参考
- 2026-10-10 验证日志(详见 `feedback_go2_euler_call_sequence.md`、`feedback_go2_euler_1007_unresponsive.md` 等 memory)
- CLAUDE.md 第 14-18 行已知反模式 —— BT 节点内禁止 `rclcpp::Rate` / `WallRate` 自建循环