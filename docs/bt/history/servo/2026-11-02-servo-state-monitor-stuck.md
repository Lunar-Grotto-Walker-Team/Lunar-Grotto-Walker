# Servo 状态监视器疑似冻结在启动值——调查记录与下一步探针

- 日期:2026-11-02(调查自 2026-10-31 起)
- 状态:**已定位根因并修复**(2026-11-02;§4 的"StateMonitor 恒为零"假设被 §9 推翻)
- 根因一句话:moveit_servo 的 `last_sent_command_` 只在 `ServoCalcs::start()` 时按当时状态播种,
  而本栈在开机(臂在零位)时启动 Servo,takeover 只 unpause,于是首条零指令复播了开机时的全零缓存
- 影响功能:`lgw_servo_node` 的 takeover 前置摆位(plan 2026-10-31)
- 相关:`docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md` §3.8

---

## 1. 现象

真机接管(`/lgw/servo/takeover`)时,节点先用 MoveGroup 把臂摆进"固定姿态族",再 unpause Servo。
摆位成功(节点复判 `max_joint_delta = 0.018` 通过),但 **Servo unpause 后发出的第一条轨迹是全零**,
真臂被从族内姿态拽回零位,零位是腕奇异点 → Servo 报 `Very close to a singularity` → 节点 HOLD。

## 2. 实测证据

三种环境、四次录制,**签名完全一致**:

| 环境 | `/control/joint_states` | Servo 输出 | 真臂 |
|---|---|---|---|
| mock,400 ms settle | 族内姿态 | 98 条,全零,3.3 s | (mock)被拽回零 |
| mock,2 s settle | 族内姿态 | 同上 | (mock)被拽回零 |
| 真机,`follow:=true` | 族内姿态 | 98 条全零 | 被拽回零 |
| 真机,`follow:=false` | 族内姿态 | 68 条全零 | 被拽回零 |
| mock,驱动已停止 | 族内姿态 | 67 条全零 | — |

关键对照:**摆位期间 `/control` 的跳变与零指令出现在同一拍**;之后 `/control` 恒定在族内(或衰减到零),
Servo 的输出始终不变。

## 3. 已排除(每条都有实测)

| 假设 | 排除依据 |
|---|---|
| 两个发布者抢 `/control` | `ros2 topic info -v`:发布者只有 `joint_state_broadcaster` |
| QoS 不兼容 | 发布 RELIABLE,`servo_node_private_*` 订阅 RELIABLE |
| MoveGroup 链路断 | 真臂实测 j2 速度 0.077→0.285 rad/s,位置平滑爬升 |
| 我们节点/状态机的逻辑 | 注入实验直接调 `/servo_node/unpause_servo` + 外部灌零 twist,完全绕开节点,照样复现 |
| `follow` 参数 | true/false 两次结果相同 |
| 真机驱动进程的存在 | 停掉 `agx_arm_ctrl` 后同样复现 |
| Servo 参数配错 | `joint_topic=/control/joint_states`、`publish_period=0.034`、`command_out_topic` 均正确 |
| SRDF 组名/关节名不匹配 | `arm` 组 = chain base_link→link6,joint1..joint6,与消息一致 |
| 摆位与 unpause 间隔太短 | settle 从 400 ms 加到 2 s 无变化 |

## 4. 源码给出的定位

`moveit_servo/src/servo_calcs.cpp`:

```cpp
void ServoCalcs::calculateSingleIteration() {
  ...
  updateJoints();                                    // 每拍都调用(含 paused)
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  ...
  if (wait_for_servo_commands_ || paused_) {
      resetLowPassFilters(original_joint_state_);
      return;                                        // 暂停时不输出
  }
}

void ServoCalcs::updateJoints() {
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->copyJointGroupPositions(joint_model_group_, internal_joint_state_.position);
  current_state_->copyJointGroupVelocities(joint_model_group_, internal_joint_state_.velocity);
  original_joint_state_ = internal_joint_state_;     // ← unpause 后的锚点
}
```

`paused` 期间**仍然每拍刷新**锚点。既然输出恒为零,则只有一种解释:

> **`StateMonitor->getCurrentState()` 恒为零**,即当前状态监视器停留在启动那一刻的值,
> 之后没有再接收/应用 `/control/joint_states` 的更新。

> ⚠️ **该结论已推翻(2026-11-02,见 §9)。** 监视器工作正常;"输出恒为零"的真正来源是
> `calculateSingleIteration()` 里另一条分支——零指令时 `*joint_trajectory = *last_sent_command_`。

Servo 主程序是 `rclcpp::spin(servo_node->get_node_base_interface())`(单线程,默认 callback group),
日志确认它订阅了 `/control/joint_states`。

## 5. 待区分的两条路径

- **A. 订阅未被处理**:该订阅的回调没有被 executor 执行,样本没被 take,reader 停在启动样本。
- **B. 订阅处理了但没写进状态**:`copyJointGroupPositions(joint_model_group_, ...)` 因 `joint_model_group_`
  为空或指向别的模型而静默不写。

## 6. 下一步:独立探针(决定性实验)

**做法**:写一个只依赖 `moveit_core` 的小节点,自己创建 `CurrentStateMonitor`,用与 Servo 相同的
`robot_description` / `robot_description_semantic` 订阅同一个 `/control/joint_states`,每秒打印一次
`getCurrentState()` 里 6 个关节角;在此期间用 MoveGroup 改变 mock 状态。

**判读**:

| 探针表现 | 结论 | 下一步 |
|---|---|---|
| 探针的 monitor 也冻结 | 问题在 topic / monitor 机制,与 Servo 实例无关 | 查 broadcaster 的发布与 monitor 的 `startStateMonitor` 参数 |
| 探针的 monitor 正常跟随 | 问题在 Servo 实例 | 查 Servo 的 executor / callback group / `joint_model_group_`(路径 B) |

**探针骨架**(API 签名需在容器内按 `install/` 下的头文件核对,不要照抄):

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/current_state_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("servo_state_probe");

  // robot_description / robot_description_semantic 由 launch 传入(与 servo_node 同源)
  moveit::core::RobotModelConstPtr model =
    std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description")
      ->getModel();

  auto monitor =
    std::make_shared<moveit::planning_interface::CurrentStateMonitor>(model, node);
  monitor->startStateMonitor("/control/joint_states");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  auto last = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    exec.spin_some();
    if (std::chrono::steady_clock::now() - last > std::chrono::seconds(1)) {
      last = std::chrono::steady_clock::now();
      auto state = monitor->getCurrentState();
      std::vector<double> q;
      state->copyJointGroupPositions(model->getJointModelGroup("arm"), q);
      RCLCPP_INFO(node->get_logger(), "monitor q = [%.3f %.3f %.3f %.3f %.3f %.3f]",
        q[0], q[1], q[2], q[3], q[4], q[5]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
```

**运行方式**(不驱动真机;先停 `agx_arm_ctrl`,再起 mock 栈):

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh; \
  ros2 launch lgw_bt_planner lgw_full.launch.py follow:=false use_bt:=false use_rviz:=false"
# 另一终端
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh; \
  ros2 run servo_state_probe servo_state_probe --ros-args \
    --params-file /tmp/servo_rd.yaml"   # 含 robot_description + semantic
# 第三终端:改变 mock 状态
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh; \
  bash scripts/move_to_joints.sh 0.53 0.89 -0.484 -1.02 0.60 1.25 0.10"
```

## 7. 补充线索(未验证)

今天更早的一次 mock bench(2026-10-31,无真机驱动、同一套 launch)里 Servo **能正常跟踪 50 s**
(`e_ang` 均值 0.048、无奇异)。之后改动过 `servo_node.cpp`(摆位接线)、`servo_node.yaml`(新增参数)
并多次重编。这些都不应影响 Servo 的状态监视器,但**这是唯一尚未对照的差异**。

**如果探针显示 monitor 正常**,建议回头做这次回归对照:用 2026-10-31 的 commit
(`c66af1e`,Task 6 bench)复跑同一注入,确认是"曾经正常、后来被改坏"还是"那次正常另有原因"。

## 8. 当前现场状态

- 真臂停在 home(关节全零),驱动已重启(`speed_percent=80, fast_mode=false`,enable=True);
- ROS 栈已清理,无残留进程;
- 该功能代码在分支 `servo-takeover`,最近提交 `56080f2`(标注为 WIP,未称完成)。

---

## 9. 决定性实验(2026-11-02):监视器正常

### 9.1 独立探针

新增 `src/servo_state_probe`(一次性工具包,`servo_state_probe.launch.py` 用与 Servo
完全相同的 `_moveit_config_builder` 重建 URDF/SRDF)。探针自建 `CurrentStateMonitor`,
同时自建一条普通订阅做对照,每秒打印一次监视器姿态。

mock 栈 + MoveGroup 把臂从 0 挪到 `[0.516 0.881 -0.483 -1.019 0.594 1.232]`:

```
raw    =[ 0.516 0.881 -0.483 -1.019 0.594 1.232 ]
monitor=[ 0.516 0.881 -0.483 -1.019 0.594 1.232 ]
diff   =[ +0.000 ... ]
```

**探针监视器完全跟随 → 文档 §6 判读表的"问题在 Servo 实例"一列。**

运行方式(mock 栈已起):

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && ros2 launch servo_state_probe servo_state_probe.launch.py"
```

### 9.2 Servo 进程内验证

- `ros2 node info /servo_node` 看不到 joint_states 订阅,但
  `/servo_node_private_<addr>` 上有 `/control/joint_states`(QoS RELIABLE depth 1,
  与 broadcaster 兼容)。`PlanningSceneMonitor` 用独立线程 spin 这个私有节点。
- gdb 在该进程里对 `CurrentStateMonitor::jointStateCallback` 下断点,**立即命中**,
  `joint_state->position` 有 7 个值 → 回调确实在跑,不是路径 A。

### 9.3 关键分叉:零指令 vs 非零指令

同一 mock 姿态 `[0.516 0.881 -0.483 -1.019 0.594 1.232]` 下:

| Servo 收到的指令 | Servo 首条输出的 positions |
|---|---|
| 非零 twist(`linear.z=0.01`,带当前 stamp) | `[0.516 0.881 -0.483 -1.018 0.594 1.231]` ✅ 真实姿态 |
| 零 twist(带当前 stamp) | `[0 0 0 0 0 0]` ❌ |

**监视器没问题;全零来自零指令分支。**

> 坑:twist 的 `header.stamp` 必须非零。`twistStampedCB()` 只在
> `stamp != Time(0)` 时更新 `latest_twist_command_stamp_`;stamp=0 的 twist 永远被判
> stale,什么都不会发。`ros2 topic pub` 默认 stamp=0——最初的"Servo 完全没输出"
> 是这个原因,不是 bug。

## 10. 根因

`moveit_servo` `servo_calcs.cpp`:

```cpp
void ServoCalcs::start() {            // 只在 start_servo 时执行
  ...
  planning_scene_monitor_->getStateMonitor()->getCurrentState()
      ->copyJointGroupPositions(joint_model_group_, point.positions);
  last_sent_command_ = std::move(initial_joint_trajectory);   // ← 播种(一次性)
}

void ServoCalcs::calculateSingleIteration() {
  updateJoints();                          // 监视器正常,这块没问题
  ...
  if (have_nonzero_twist_stamped_ && !twist_command_is_stale_) { ... }
  else { *joint_trajectory = *last_sent_command_; }            // ← 零指令复播缓存
  ...
  if (ok_to_publish_ && !paused_) publish(joint_trajectory);
}
```

本栈的生命周期把这条设计用错了位置:

1. 开机时 `lgw_servo_node` 调 `start_servo` → **`last_sent_command_` 播种为当时的臂位姿
   (home,全零)**,随后 pause;
2. takeover 时 MoveGroup 把臂摆进姿态族(监视器同步跟随),但**缓存没动**;
3. `unpause_servo` 后控制律首拍给出零 twist(或任何未产生非零指令的首拍),
   走 else 分支 → 复播全零 → JTC 执行 → 臂被拽回零位 → 腕奇异 → HOLD。

这也解释了 §2 里"摆位期间 `/control` 跳变与零指令出现在同一拍"的错觉:监视器没错,
错的是缓存。

### 10.1 触发条件:为什么 10-31 16:53 的 bench 没触发,21:02 之后才复现

缓存陈旧的缺陷从一开始就在(`c66af1e` 的 `servo_node.cpp` 里同样是"开机 start、takeover
只 unpause"),但它需要一个精确条件才会显形:**unpause 之后的第一条指令恰好是零**。

- `moveit_servo` 的 `isNonZero()` 是**精确零**判断(`utilities.cpp:46`),
  所以只有逐位为零的 twist 才会走 `last_sent_command_` 分支。
- 控制律唯一的精确零输出来自严格对准死区:`e_pos < 0.01 m && e_ang < 0.0349 rad`
  时 `cmd.v = cmd.w = (0,0,0)`(`servo_control_law.cpp:131-145`)。
- `c66af1e`(10-31 16:53)时 takeover 前摆位由 bench 手工做,臂离设定点较远,
  第一拍指令非零 → Servo 走 Cartesian 分支、用**实时监视器**算增量 → 正确。
  那次 mock bench 连跑 50 s 正常。
- `56080f2`(10-31 21:02,pre-position 入 takeover)让节点先把臂摆到姿态族,
  也就是**摆到设定点附近**:一旦残差进入 1 cm / 2° 死区,第一拍就是精确零 twist,
  沉睡的分支被激活。该 commit 的说明本身就是 "WIP: bench shows a mock-side reset"。

2026-11-02 实测复现这个条件:

| 场景 | unpause 后第一拍 twist |
|---|---|
| 静态目标,臂已在姿态族附近(`tcp=0.017m/0.027rad`,死区外) | 非零(`v≈0.005–0.025`) |
| 臂从零位起,由节点自己摆位到 `[0.530 0.890 -0.484 -1.020 0.600 1.249]` | **逐位全零** |
| 跟踪收敛后(在死区内) | 401/401 条逐位全零 |

这也解释了为什么现有 `bench_posture_lookat.sh` **不修复也能过**:它的假目标带 ±15 cm
扫动、残差在死区外;而真机/节点自摆位的几何更容易落进死区。回归必须用
`check_servo_zero_command.py`(见 §11.2),不能只靠 bench。

## 11. 修复与验证

### 11.1 修复(`src/lgw_bt_planner/src/servo_node.cpp`)

`handle_takeover()` 在摆位成功之后、`unpause_servo` 之前,用独立的阻塞客户端
(`reseed_client_`,挂在 `servo_sync_group_`,不复用 boot 路径的异步 `start_client_`)
再调一次 `start_servo`。Servo 此时仍是 paused,`ServoCalcs::start()` 会用摆位后的
当前姿态重新播种 `last_sent_command_`;随后照旧 unpause,状态机语义不变。
失败则 `pause_servo` + RELEASE,绝不让 Servo 在拒绝的 takeover 里继续驱动 JTC。

顺带修了 `CMakeLists.txt` 的导出问题:`lgw_posture` / `lgw_preposition` 作为静态库
被 `lgw_bt_planner_nodes` 链接,却没进 export set,任何一次重新 configure 都会失败。

### 11.2 验证

`bash scripts/bench_posture_lookat.sh 30`(mock,修复后):

```
takeover           success, state=TRACKING
e_ang (rad)        mean 0.048  p95 0.075  max 0.081
e_pos (m)          mean 0.014  p95 0.024  max 0.025
servo status != 0  0.00 s
final q            +0.518 +0.881 -0.467 -0.980 +0.594 +1.213   (留在姿态族,未被拽零)
```

日志中有 `Servo command cache re-seeded from the prepositioned pose`。

> 注意:同一 bench **在没有修复时也能通过**——控制律首拍通常是非零指令,不走
> `last_sent_command_` 分支。bench 覆盖不到本 bug,必须用下面的零指令检查。

新增 `scripts/check_servo_zero_command.py`(mock 或真机均可,零指令=保持,不动臂):

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 scripts/check_servo_zero_command.py \
     --joint-topic /control/joint_states --seconds 2"
```

修复后结果:

```
measured joints        : [0.526, 0.894, -0.499, -1.026, 0.584, 1.231]
first Servo command    : [0.526, 0.894, -0.499, -1.026, 0.584, 1.231]
|cmd - measured|       : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
PASS: the zero twist held the current pose.
```

修复前的同一不变量(手动 A/B,8.2 节场景):pause → 臂在零位时 `start_servo`
(缓存=零)→ MoveGroup 摆位 → unpause → 零 twist → 输出全零;改为摆位后再
`start_servo` → 输出即当前姿态。两者差 0.5–1.2 rad/关节。

### 11.3 遗留

- 真机复测:用 `follow:=true` + 真驱动跑一次 takeover;零指令检查把
  `--joint-topic` 换成 `/feedback/joint_states`。
- ~~`handle_resume`(HOLD→TRACKING)没有再播种缓存~~ **已修(2026-11-03)**:resume 在
  unpause 之前重新 `start_servo`,并用零指令探针验证播种生效;见
  `docs/superpowers/specs/2026-11-03-hold-recovery-design.md` §15 与
  `scripts/check_servo_resume_reseed.py`。
- `src/servo_state_probe/` 是诊断工具包,不是产品依赖,可按需删除。

