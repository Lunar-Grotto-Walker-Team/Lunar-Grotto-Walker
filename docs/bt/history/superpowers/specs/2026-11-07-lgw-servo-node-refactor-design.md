# servo_node 重构设计（2026-11-07）

- 状态：**待评审**
- 前序：`2026-10-31-lgw-servo-node-refactor.md`（已过期，仅保留问题清单与骨架，
  行号/边界一律以本文为准）
- 对象：`src/lgw_bt_planner/src/servo_node.cpp`，**3246 行**（原 spec 写作时 1831 行）

---

## 1. 意图与理由

节点从"一个 900 行的视觉伺服控制器"长成了"整个手臂运行时"：参数加载、ROS 接线、
位姿求解、QP/DLS 选择、HOLD/恢复编排、Servo 生命周期、服务编排全在一个类里。
过去两个月每一次功能迭代（QP、分支滞回、恢复预算、command 域）都要改这个文件，
且每次改完只能靠真机肉眼判断有没有回归。

现在功能面基本稳定，欠账可以还。目标**不是让它变短**，而是让：
- 业务实现（可判定、可证明的部分）脱离 rclcpp，能离线单测；
- 节点退回"接线 + 持有实例"，任何一处改动只有一个"为什么会被改"的理由；
- 每次改一步都有**自动化证据**证明行为没变（本任务明确：不进真机）。

## 2. 目标 / 非目标 / 反目标

**目标**
1. 节点文件只剩接线：订阅、服务、定时器、参数装载、实例持有、状态发布；
2. 每拍业务链路是纯函数：`(输入状态, 参数) -> (twist, 新状态)`，无 rclcpp、无 TF 查询；
3. 时间相关逻辑（超时、冷却、截止期、重试）通过**注入时钟**变得可复现；
4. 参数按 ROS 2 官方做法迁移到 **`generate_parameter_library`**（YAML schema →
   类型化结构体 + 内建校验），非法值在**启动时**失败；
5. "失败 → 停机"只有一条代码路径；
6. 模式用类型表达（`enum class`），新增模式只改一处；
7. 场景级集成测试用 **`launch_testing`** 组织，纯逻辑用 gtest，回放用独立二进制。

**非目标**
- 不改控制算法（QP/DLS、分支策略、阻尼、开环/闭环域选择）；
- 不改 ros2_control 硬件后端（`agx_topic_hardware`）、不改 vendor 补丁；
- 不改 BT 树、不改消息定义、不改话题/服务名；
- 不做 `rclcpp_components` 组件化（可后续单独立项）。

**已评估并明确放弃：`rclcpp_lifecycle::LifecycleNode`**（用户决定，2026-11-07）
本节点形态上接近 lifecycle 节点（可配置/可激活/可停用），但映射不干净：
`takeover` 需要 `track_id` 而 lifecycle 转换不带参数，且四个服务被 BT、脚本和
bench 工具广泛调用。改造成本横跨 BT XML、launch 与全部脚本，收益与本轮目标
（业务解耦 + 可测）无关。**本轮保持现状**，若将来要做，按独立项目立项，届时
四个服务可作为薄适配器转调 lifecycle 转换。

**反目标（技术上都成立、但这次不算成功）**
- 为了"更短"而把逻辑移到另一个大文件里；
- 用真机试跑来代替自动验证；
- 一次性大改，无法逐步回退；
- 改完了但没有任何"改之前"的证据可比对。

## 3. 约束（已由用户决定）

1. **基线先冻结**：当前工作树（22 项改动，`servo_node.cpp` 3246 行 vs HEAD 3052 行）
   先提交为重构基线，重构失败可直接回到该点；
2. **每步验收靠自动化测试**；真机只用于里程碑冒烟（§7 V6），不作为逐步验收手段；
3. **允许顺手修复已知缺陷**：但每一处行为改变必须单独成卡（§6），可追溯、可单独回退；
4. 除 §6 列出的卡片外，行为必须与重构前**逐拍等价**；
5. 每一步独立提交、独立验证、可单独回退。

## 4. 现状（实测）

| 区块 | 规模 |
|---|---|
| `update_tracking_locked` | 1688–2028，**341 行** |
| `declare_and_load_parameters` | 416–740，**325 行**，`declare_parameter` **101 处** |
| `recovery_worker` | 1439–1687，**249 行** |
| `on_timer` | 972–1098，127 行 |
| `maybe_start_recovery_locked` | 1338–1438，101 行 |
| `maybe_preposition` | 2169–2289，121 行 |
| `publish_state_locked` | 2961–3069，109 行 |
| `handle_takeover` / `handle_resume` | 108 / 106 行 |
| 其余 25 个成员函数 | 20–100 行 |
| ROS 接线 | 7 订阅、4 服务、5 callback group、1 个 100 Hz 定时器 |
| 已有纯模块 | `servo_state_machine`、`servo_control_law`、`hold_recovery_decision`、`lookat_*`，**18 个 gtest 二进制** |

问题（沿用并更新旧 spec）：
- `update_tracking_locked` 一个函数干八件事；
- `sm_.handle(TARGET_LOST)` + 清几何源 + `return true` 的样板重复 **7 次**；
- 模式用两个 bool 表达（新增第三种时日志漏改，已发生过）；
- 101 个参数全部裸 `declare_parameter`，无范围、无分组；
- `force_cancel_moves` 一设即"拒绝一切 takeover"，等于死配置；
- 无 lint（`ament_lint_auto` 未接），只写不读的字段无人提醒。

## 5. 目标结构

```
servo_node.cpp              ~250 行   接线：订阅/服务/定时器/参数装载/实例持有
servo_params.{hpp,cpp}       参数迁移到 generate_parameter_library：YAML schema
                             (servo_node_params.yaml) + 生成的结构体 + param_listener
servo_session.{hpp,cpp}      会话：状态机驱动、Servo 启停、takeover/release/pause/resume 编排
tracking_loop.{hpp,cpp}      每拍纯逻辑：源选择 → 位姿求解 → 控制律 → 输出
recovery_orchestrator.{hpp,cpp}  HOLD→RECOVERING 决策 + 恢复 worker（I/O 通过接口注入）
servo_geometry.{hpp,cpp}     TF/FK 查询与位姿域选择（命令域/测量域）
状态发布                     servo_state_reporter.{hpp,cpp}
```

**依赖方向（只允许向下）**

```
servo_node  ──>  servo_session  ──>  tracking_loop ──> lookat_* / control_law / state_machine
     │                │                     
     └──> servo_params └──> recovery_orchestrator ──> hold_recovery_decision
```

纯模块（`lookat/*`、`servo_control_law`、`servo_state_machine`、`hold_recovery_decision`）
**一律不动**，它们是本次重构的参照物。

**关键接缝（降级为参数传递，不引入接口类）**

| 接缝 | 形式 | 用途 |
|---|---|---|
| 时间 | 纯函数多一个 `now_ns` 入参（生产传 `now().nanoseconds()`） | 超时/冷却/截止期/重试可复现 |
| 控制拍输入 | 普通 `TrackingInput` struct 入参（目标快照、odom→base、命令关节、测量关节、Servo 状态） | 控制拍不自己查 TF、不自己读时钟 |
| 摆位 | `std::function` 回调注入（下发 goal、返回结果） | 恢复路径可注入假实现 |
| 定时器 | 保留 `create_wall_timer(10ms)`；回放直接调用纯 tick 并传入步进时间 | 不把手动 tick 入口塞进生产路径 |

原则：**可测性来自"纯函数 + 显式输入"，不来自接口与 mock 框架**。`servo_session`
属于会话层，允许直接依赖 rclcpp（持有 client/executor），不强行纯化。

## 6. 缺陷卡片（允许的行为改变，逐条独立、可单独回退）

| 卡 | 现状 | 改成 | 影响的验证 |
|---|---|---|---|
| D1 | `force_cancel_moves:=true` 时一律拒绝 takeover，提示 "not implemented in v1" | 删除该参数（或启动即报错），不留"一设就瘫痪"的死配置 | 回放差异仅在启动参数集合 |
| D2 | 101 个参数裸加载、无校验 | 迁移到 `generate_parameter_library`：schema 内建类型/范围/枚举校验，跨参数互斥写自定义 validator；非法值**启动失败**并给出参数名。参数名保持不变（现有 yaml 扁平结构兼容） | 参数表快照测试（名字+默认值+范围） |
| D3 | 7 处 "失败 → HOLD" 样板 | 单一 `hold_and_return(reason)` 路径 | 逐条等价，回放应无差异 |
| D4 | `lookat_target_relative_` / `lookat_pinned_posture_` 两个 bool | `enum class LookatMode`，分支收成函数 | 回放应无差异 |
| D5 | 控制拍全程持 `mutex_`（100 Hz） | **暂不改**。仅在 S4 抽出纯逻辑后评估是否可收窄；若改，单独卡片、单独验证 | 若改：压力回放 |
| D6 | 无 lint | 接 `ament_lint_auto`（uncrustify + clang-tidy 最小集） | 零行为风险 |
| D7 | 只写不读的字段 | 由 lint/编译器告警指认后清理；**没有指认就跳过**（评审已核实 `posture_last_status_` 实际在日志里被读，不作为例子） | 零行为风险 |
| D8 | `recovery_worker` 用真实 `sleep_for` + 阻塞调用，deadline 不精确，无法逐拍复现 | 改造成可步进的事件驱动机器 | **本轮不做**，仅记录；S4 只验证决策层 + 场景级 |

## 7. 验证标准（本任务的验收口径）

**V1 单元测试**：每个新类都有 gtest；纯逻辑（不含 rclcpp）覆盖率不低于改造前对应代码的判定分支数（按分支点计数，不追求百分比）。

**V2 回放测试（核心证据）**：
- S0 建"无硬件测试台"：假 TF、假目标流、假关节状态、假 Servo 状态/action，**时间显式传入**；
- 用**重构前**的代码（基线提交的归档副本）跑同一组合成场景，录成 golden trace；
- 每个重构步骤后重放，比对 `twist`、`/lgw/servo/state`、状态迁移序列、服务返回；
- 判据：除 §6 卡片外，**逐拍等值**（浮点按 `1e-9` 比较，时间戳按步进时间对齐）。

**V2b 场景级集成用 `launch_testing`**：起真 `moveit_servo` + mock 硬件（`follow:=false`）
+ 假目标，断言话题/服务/状态；回放二进制只跑纯逻辑，两者不混。

**V3 每步闸门**：`colcon build` 干净 → 18 个既有 gtest 全绿 → 新测试全绿 →
`launch_testing` 场景通过 → 回放 diff 干净（或差异与卡片一一对应）→ 单独提交。
> 前置实测：CLAUDE.md 记录"本容器缺 `ament_cmake_test` 所以 `colcon test` 不可用"，
> 但该包实际存在。S0 第一件事是实测 `colcon test` 能否跑；若仍不可用，
> `launch_testing` 用例改用 `launch_test <file>` 直接执行，并把结论写回 CLAUDE.md。

**V4 参数表快照**：参数名、默认值、范围固化成测试，防止"改了一处默认值没人发现"。

**V5 场景覆盖**：回放至少覆盖 8 个场景——接管、跟踪（目标静止/移动/换支）、HOLD（各 reason）、恢复成功/失败/预算耗尽、release/pause/resume、目标丢失、Servo halt、命令域/测量域两种位姿源。

**V6 真机里程碑冒烟（三个点，用小幅目标变化/摆动，沿用既有测试手法）**：

| 时点 | 动作 | 判据 |
|---|---|---|
| S3 后（控制拍拆完） | 接管 → ±5–14 cm 扫掠 → 静止 → 关栈 | 无 HOLD 抖动、静止段关节 p2p < 5 mrad、关栈不动臂 |
| S5 后（会话拆完） | release / pause / resume 各一次 + 一次恢复 | 状态迁移与重构前一致，服务返回一致 |
| 最终 | 完整 smoke（含目标台阶与平台静止） | 沿用 `docs/dev/2026-11-06-vendor-modifications-and-rollback.md` 的实测表 |

真机只证明"集成没断"，逻辑等价仍以 V1/V2 为准。

## 8. 迁移顺序（每步独立可回退）

| 步 | 内容 | 主要风险 | 回退 |
|---|---|---|---|
| **S0** | 测试台（`launch_testing` 场景 + 回放二进制）+ 时间入参接缝 + 录制 golden trace；**先实测 `colcon test` 可用性** | 接缝本身可能改变时序 | 接缝只加参数，逻辑不动 |
| S1a | `ServoParams` 抽取（纯搬家，参数仍是 `declare_parameter`） | 参数遗漏/默认值漂移 | 用 V4 快照比对 |
| S1b | 迁移到 `generate_parameter_library`（schema + 生成结构体 + 校验）；参数名不变 | 默认值搬迁漂移、动态更新语义 | V4 快照 + 回放逐拍比对 |
| S2 | D3 + D4（样板收敛、模式 enum） | 分支等价性 | 回放逐拍比对 |
| S3 | `tracking_loop` 抽取（`update_tracking_locked` 拆为 pick/solve/apply/publish） | 最大的一块，状态交互多 | 回放 + 单测双保险 |
| S4 | `recovery_orchestrator` 抽取（`recovery_worker` + 恢复决策） | 线程/异步与 MoveGroup 交互 | 注入假 prepositioner 回放 |
| S5 | `servo_session` 抽取（takeover/release/pause/resume + Servo 生命周期） | 服务并发语义 | 服务级回放 |
| S6 | D1 + D6 + D7（死配置、lint、死字段） | 零/低 | 独立提交 |
| S7 | 复核 D5（长锁），必要时单独卡片 | 并发 | 压力回放 |

顺序原则：**先建证据（S0），再搬不改变判断的东西（S1），最后搬最危险的控制拍与恢复（S3/S4）**。

## 9. 评审后已定的实现判断（实施者按此执行，不再讨论）

- **回放边界**：控制拍用纯函数确定性驱动；服务/并发只做场景级集成，用真 `moveit_servo`
  + 假驱动，**不手写 fake 复刻 Servo 语义**。
- **恢复 worker**：保留现有线程与阻塞结构（见 D8，本轮不改）。
- **并发拓扑原样保留**：`servo_sync_group_`(auto_add=false)、局部 `SingleThreadedExecutor`、
  `service_group_` MutuallyExclusive、控制拍全程持 `mutex_`。D5 本轮不做。
- **D3**：`hold_and_return(reason, clear_geometry)` 带标志、逐处等价；saturated 路径不清
  `geometry_source`。"单一失败路径"= 一个函数 + 参数化清理，不是强行归一。
- **D2 拆两步**：S1a 纯搬家、S1b 校验单独提交。校验接受既有哨兵语义
  （`v_max=0` 冻结手臂、`recovery_max_attempts=0` 无限、`recovery_deadline_ms=0` 禁用、
  空 `arm_calibration_path` = 默认 share 路径）。`ServoParams` **只装纯值**；
  `posture_` / `qp_solver_` / `prepositioner_` 的构造与 FAULT 副作用留在节点。
- **状态归属**：新增 `SessionState`（`hold_reason` / `recovery_attempts` / `geometry_source` /
  `session_active`），会话持有并按引用进纯 tick；`recovery_orchestrator` 由节点在 tick 之后、
  **同一把锁内**调用，保持现有调用顺序。
- **模块合并**：`servo_geometry` 并入 S3（TF/FK 查询就是 tick 的输入），
  `servo_state_reporter` 并入 S5；不单列步骤。
- **lint 只报告不阻断**，范围限新增/重构文件；存量 3246 行不重排、存量告警本轮不清。
- **D1**：删除参数，并同步删 `config/servo_node.yaml` 中的 `force_cancel_moves: false`。
- **基线**：当前工作树先提交为 `chore(lgw): freeze pre-refactor baseline`（已提交为
  `3001f71`，分支 `refactor/servo-node-split`），golden trace 由该提交的代码产生；
  S0 的等价性通过"归档副本 vs 接缝版"双跑同一场景证明。
- **参数框架**：迁移到 `generate_parameter_library`（容器已装）。参数名保持不变；
  默认值由代码搬到 YAML schema，用 V4 快照逐一钉住；跨参数互斥（如
  `tcp_pose_domain` 与阻尼启用条件）写自定义 validator。
- **测试框架**：场景级用 `launch_testing`，纯逻辑用 gtest，回放为独立二进制。
- **LifecycleNode**：本轮不做（§2 已记录理由），四个服务保持不变。

### 9.1 D5（长锁）复核结论 — 2026-11-07，Task 8 step 4c

**结论：本轮不改，理由与将来的前置条件记录在此。**

现状（读 `tick()` / `update_tracking_locked` 得出，非推测）：整个控制拍持
`mutex_`，包括 `lookup_tcp_locked()`（先 `can_transform(0)` 探测，不会阻塞等
超时）、`select_source()`、位姿族求解 `solver_->solve()`，直到
`publish_twist_locked()` 返回。服务回调（takeover / release / pause / resume）
抢同一把锁，所以一拍的长度就是服务的最坏响应延迟。

"锁内拷贝输入、锁外计算"在本拍上**不可直接套用**，有三个具体障碍：

1. **求解器不是无状态的**。`StandoffState` / `ff_*` / `last_sent_joints_` /
   `posture_solve_ms_*` 都按拍推进，算在锁外就要为它们再引入一把锁，等于把
   一把长锁换成两把短锁加一次一致性论证。
2. **发布必须与状态机同步**。锁外算完再回来发布，中间可能插进 `release`
   或 `pause`；那正是"release 之后仍发出了一个 twist"的一类缺陷。要么发布前
   重新校验并重算，要么接受一次 TOCTOU——两者都比现状差。
3. **收益没有量化**。求解耗时已经由节点自带仪表
   （`posture_solve_ms_sum_/max_`，每 500 拍打一条 `[posture] solve mean …
   max … ms`）。在拿到这个数之前改并发拓扑，是在用一次安全路径的重排换取
   一个未测量的延迟。

**已补上的测量**（2026-11-07，mock 全栈 `bench_posture_lookat.sh`，40 s，
`lookat_mode:=pinned_posture`）：`[posture] solve mean 0.08–0.09 ms max
0.50–0.93 ms`。拍预算是 10 ms，即求解占单拍 **<1 %（mean）/ <10 %（max）**。
这个量级下拆锁的收益不可见，D5 维持不改。

**将来的前置条件**（满足任意一条再谈）：真机上 `[posture] solve` 的 mean 或
max 超过一拍预算（10 ms），或日志里出现服务调用被控制拍拖住的证据。届时优先
方向是缩短单拍工作（求解器降频 / 增量热启动），不是拆锁。

## 10. 留给实施者的判断

- 新文件的目录位置（建议 `src/lgw_bt_planner/src/servo/` + `include/lgw_bt_planner/servo/`）；
- `ITrackingInput` 的具体字段切分（以"回放可完全由文件驱动"为准）；
- golden trace 的存储格式（建议 JSON 行，便于 diff）；
- 回放测试是独立可执行文件还是 gtest 用例（建议独立二进制，避免污染 18 个既有测试）；
- 各步骤内部如何再细分提交。

## 10. 明确不做的事

- 不重写控制算法或调参；
- 不改硬件后端与 vendor 补丁；
- 不引入行为树/插件框架；
- 不迁移 `LifecycleNode`（§2 已记录理由）；
- 不在逐步骤上真机（仅三个里程碑冒烟，§7 V6）；
- 不以"行数减少"作为验收指标（S1–S5 完成后预计 ~1500 行，但价值在可测性）。
