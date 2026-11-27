# HOLD 内自恢复设计(按判据调用 MoveIt 摆位)

- 日期:2026-11-03
- 状态:**设计草案**(待评审后进入 writing-plans)
- 关联:
  - `docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md` §5.3 / §7.1 / §7.2 / §7.6 / §7.8 / §7.9
  - `docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md` §3.8(pre-position)
  - `docs/servo/2026-11-02-servo-state-monitor-stuck.md` §10–11(缓存播种与 reseed)
- 前置项:`handle_resume` 的 Servo 缓存 reseed(与 takeover 修复同源,见 §12)

---

## 1. 背景

`lgw_servo_node` 目前在 TRACKING 中因下列任一原因转入 HOLD 并 `pause_servo`:

`target_lost` / `tracker_stale` / `pose_stale` / `move_group_busy` /
`servo_halt`(奇异或碰撞降速持续)/ `saturated`(不可达)/ `too_close` /
`follower_lag`(真机持续跟不上)/ `operator_pause`。

HOLD 之后只能由外部 `/lgw/servo/resume` 回到 TRACKING(且要求新鲜的 `TARGET_OK`
latch)。目标:在**会话内部**、按明确判据,让节点自己完成一次有界恢复,减少外层
行为树的翻转次数;但会话的开始/结束、FAULT 处理、行为级重观察仍归外层。

## 2. 目标 / 非目标

**目标**

1. 对**移动确实有收益**的 HOLD 原因,复用现有 pre-position 栈(MoveGroup)把臂
   移到姿态族,然后恢复 TRACKING。
2. 恢复必须有界:次数、时限、速度、可取消、全程可观测。
3. 恢复结束后 Servo 的 `last_sent_command_` 必须是**移动后的**姿态。
4. 默认行为可回退(参数关闭即等价于现状)。

**非目标**

1. 不做会话自治:不自动 takeover / release,不自动处理 FAULT。
2. 不做行为级重观察(转头、转底盘、重新选目标)——那是外层行为节点的职责。
3. 不修改 `moveit_servo` 本体。
4. 不把 `follower_lag` 当作摆位问题处理(见 §3)。

## 3. 关键结论:`follower_lag` 不移动

### 3.1 数据链

```
MoveIt / Servo → JTC(mock) → /control/joint_states
                              → agx_arm_ctrl::_joint_states_callback
                              → move_j / move_js → CAN → 真机
```

依据:`agx_arm_ctrl_single_node.py:735-748`(把收到的整帧关节位置直接下发)
与 `:790-804`(订阅 `control/joint_states`)。

因此:

- MoveIt **从不直接控制真机**,只控制 JTC/mock;真机速度由驱动与固件
  (`speed_percent`、`move_j`/`move_js`)决定;
- Servo 暂停后 JTC 已经停在最后一条指令上(即"正确位置"),再发一条到同一位置的
  MoveGroup 是零长度轨迹,对真机没有任何新信息;
- 真机滞后时,正确的恢复是**冻结命令等它追上**——HOLD 本身已经做到。

### 3.2 三种"滞后"必须分开

| 现象 | 本质 | 处理 |
|---|---|---|
| 追移动目标,命令持续领先 | 真机物理速度上限 | HOLD 冻结,JTC 停住,真机自然收敛;gap 闭合后 resume |
| 真机卡死(驱动异常/使能丢失/CAN) | 硬件/链路故障 | 任何摆位都无效 → FAULT |
| 命令姿态本身不可达 | 姿态问题(不是 lag) | 归入 `saturated`,MoveGroup 到可达族姿态 |

### 3.3 四条姿态判据不能用来判 lag

`preposition_decision.hpp` 的四条判据(joint_delta 0.15 / |dq5| 1.571 /
limit_margin 0.10 / tcp 0.05 m + 0.15 rad)比较的是"当前姿态 vs 族解",
**没有时间维度**,回答的是"是否在错误的腕分支 / 是否贴限位"。

- 喂 `pose_watch` 的**控制域**(follow:=false → `/control`):JTC 早已在族解上,
  滞后不会让它误报;
- 喂**反馈域**(follow:=true → `/feedback`):滞后会伪装成"偏差过大",导致误判。

结论:恢复判据中的"姿态偏差"必须使用**Servo 积分所在的域**(`measured_positions_`),
并且只用于 `servo_halt`/`saturated`/`too_close`,不用于 `follower_lag`。

### 3.4 现有参数已经体现这一区分

`servo_node.yaml:106-122`:`follow_tolerance_rad: 0.45` +
`follow_timeout_ms: 5000`,即"持续 5 s 落后 > 0.45 rad"才 HOLD;
`servo_node.cpp:995-1005` 的注释同样写了 "A SLOW follower is not an
unreachable target",并要求命令停止仍在恶化才判饱和。

`follower_lag` 的恢复因此定义为:

```
TRACKING --gap>0.45 持续 5s--> HOLD(follower_lag)
HOLD --gap<0.10 连续 300ms--> TRACKING          (命令未变,无需 reseed)
HOLD --3s 内 gap 不闭合--> FAULT / exhausted     (不是慢,是卡死)
```

不调用 MoveGroup。

## 4. 恢复判据表

| HOLD 原因 | 是否移动 | 位姿来源 | 恢复条件 |
|---|---|---|---|
| `servo_halt`(奇异/碰撞降速 >3 s) | ✅ | 当前目标 → `posture_.compute` | 移动并复核成功后 reseed + unpause |
| `saturated`(不可达) | ✅ | 同上;若 `family.ok=false` 不移动 | 同上 |
| `target_lost` / `tracker_stale` | ⚠️ 有条件 | **最后已知** `p_arm_base`,且 `age ≤ recovery_geometry_max_age_ms` | 同上;否则不移动,`exhausted` |
| `follower_lag` | ❌ | — | 等待 gap 闭合 → resume;超时 → FAULT |
| `pose_stale` / TF 缺失 | ❌ | MoveGroup 也没有可用状态 | `kNone`:不消耗预算;pose 流恢复后重新判据 |
| `move_group_busy` | ❌ | 外部正在使用 | 由外部在动作结束时 resume |
| `operator_pause` | ❌ | 人工暂停 | 只能人工 resume |
| `FAULT` | ❌ | — | 需重启节点 |

注意:

- **`too_close` 只是 `ServoState` 里的诊断标志,当前不会触发 HOLD**
  (`servo_node.cpp:992` 只写 `too_close_`,没有状态迁移),因此不进入本表。
- 状态机只有一个事件 `TARGET_LOST`;`hold_reason` 必须由节点在**触发点**记录
  (`!target_ok` / `busy` / `servo_halted` / `!pose_alive` / `not_following` /
  几何异常),不能从状态机事件反推。这是本设计新增的实现要求。

共同前置条件(全部满足才启动):

1. 状态为 HOLD,且 Servo 已 paused;
2. `move_group_busy_locked() == false`;
3. `measured_valid_` 且 `lookup_tcp_locked()` 成功;
4. `recovery_attempts < recovery_max_attempts`;
5. `now - hold_since ≤ recovery_deadline_ms`;
6. 原因在白名单内,且(若需要目标)几何源可用。

## 5. 恢复位姿

复用 `maybe_preposition()` 的求解链,但使用**独立的恢复参数集**:

```
target = select_source(selected_snapshot_ | last_known, ...).p_arm_base
family = posture_.compute(target, R_cam_in_tcp_, t_cam_in_tcp_, posture_params_)
decision = decide_preposition(family.joints, q_now, family.tcp_desired, tcp_now,
                              recovery_decision_params_)
```

- `family.ok == false` 或 `out_of_envelope`:**不移动**,记 `exhausted`;
- `decision.needed == false`:跳过移动,直接进入 reseed + resume(便宜路径);
- `last_known` 只在 `target_lost` 时使用,并受 `recovery_geometry_max_age_ms` 约束;
- 移动目标 = `family.joints`(关节空间 goal,平台运动不影响目标定义)。

## 6. 状态模型

**新增顶层状态 `RECOVERING`**(2026-11-03 决定;替代早先的 "HOLD + phase" 方案)。

它定义为 **HOLD 的主动子状态**:

| | HOLD | RECOVERING |
|---|---|---|
| 控制权 | 本节点 | 本节点 |
| Servo | paused | paused |
| JTC | 保持最后指令 | **MoveGroup 驱动** |
| 本节点发 twist | 否 | 否 |
| MoveGroup goal | 无 | 有且仅有一个 |

两者的根本区别是"JTC 上有没有人在写",这是对外可观察的行为差,值得单独一个状态。

内部子阶段(只用于日志,不进对外契约):

```
RECOVERING.sub = deciding | moving | verifying | reseeding
```

### 6.1 迁移表(新增/修改项)

```
HOLD       + RECOVER_START(判据命中)          -> RECOVERING
RECOVERING + RECOVER_OK + TARGET_OK            -> TRACKING
RECOVERING + RECOVER_OK + !TARGET_OK           -> HOLD(exhausted)
RECOVERING + RECOVER_FAIL / RECOVER_EXHAUSTED  -> HOLD(exhausted)
RECOVERING + TARGET_LOST                       -> RECOVERING(仅清零 latch)
RECOVERING + TARGET_OK                         -> RECOVERING(仅置位 latch)
RECOVERING + PAUSE                             -> RECOVERING(accepted no-op)
RECOVERING + RESUME                            -> rejected("recovery in progress")
RECOVERING + RELEASE                           -> RELEASING(不取消有界移动;worker 不再 reseed/unpause)
RECOVERING + FAULT                             -> FAULT
RECOVERING + TAKEOVER                          -> rejected(非 IDLE)
```

保持不变:`TRACKING/HOLD/PREEMPT + RELEASE -> RELEASING`、
`HOLD + RESUME(after TARGET_OK) -> TRACKING`、`FAULT + RESET -> IDLE`。

### 6.2 RELEASE 为什么不取消正在执行的恢复移动

恢复移动有界(≤8 s,实际约 1 s)、低速(0.10)、终点是已知姿态族。中途取消会把臂停
在 A/B 族之间的任意姿态;让它走完则是确定的族姿态。JTC 是单写者,release 后 BT 的
下一个 MoveGroup goal 会自然抢占,不需要节点主动取消。

安全不变量是**release 后绝不再 unpause Servo**:`recovery_cancel_` 置位,worker 在
move 返回后检查 `sm_.state() != RECOVERING` 走取消出口,不 reseed、不 unpause。
若操作员需要立即停止,用急停,而不是依赖可能停在任意姿态的软件取消。

### 6.3 为什么 RESUME 在 RECOVERING 里被拒绝

恢复期间 JTC 正被 MoveGroup 写。`unpause_servo` 会让 Servo 成为第二个 JTC 写入者
(后到覆盖先到,R1 风险)。外部要打断恢复,唯一安全路径是 `RELEASE`
(停止 servo 会话、不再 unpause;有界移动按 §6.2 走完),而不是 resume。

### 6.4 退出路径完整性

`RECOVERING` 的出口只有四条:TRACKING、HOLD、RELEASING、FAULT,覆盖全部事件;
不存在"卡在 RECOVERING"的路径。`recovery_exhausted=true` 后不再自动尝试,
直到 release + 新的 takeover(或外层在目标可用时显式 resume)。

### 6.5 与 takeover 前置摆位的关系

机制上,`RECOVERING` 和 takeover 的 `maybe_preposition()` 是**同一件事**:
Servo paused,由本节点发起 MoveGroup goal 写 JTC。

v1 决定**不合并**:

| | takeover preposition | HOLD recovery |
|---|---|---|
| 触发 | 新会话准备(`handle_takeover` 内) | 会话内异常(`on_timer`) |
| 时序 | **同步**,阻塞 takeover 服务 | **异步**,worker 线程 |
| 失败语义 | 拒绝 takeover(留在 IDLE) | 回 HOLD / exhausted |
| 对外可见性 | 不需要(`PREEMPT` 已表示"准备中") | 需要(`RECOVERING` + 原因) |
| 语义 | 准备,不是恢复 | 恢复 |

理由:状态应表达"JTC 上有没有人写 + 会话处于哪个阶段",而不是把所有 MoveGroup
调用都叫恢复。把 preposition 也叫 RECOVERING 会让日志和 BT 语义都变得不准确。

**统一不变量(两种建模都成立)**:

- `TRACKING`:JTC 的唯一写者是 Servo;
- `HOLD` / `IDLE`:无人写 JTC;
- `RECOVERING`(以及 `PREEMPT` 内的 preposition):JTC 的唯一写者是本节点的
  `MoveGroupPrepositioner`;
- 外部 MoveGroup(本节点检测到 `move_group_busy`):节点处于 HOLD,且**不得**
  同时发起自己的 goal。

**备选(若将来需要"一个机制一个状态")**:把状态改名为 `REPOSITIONING`
(从 `PREEMPT` 和 `HOLD` 都可进入),用 `motion_reason ∈ {preposition,
hold_recovery}` 区分意图。触发条件:takeover 改为异步、前置摆位需要对外可见时。
在那之前,`PREEMPT` 已经足够表达"准备中,臂可能正在被 MoveGroup 移动"。

## 7. 执行时序与并发

```
on_timer(100 Hz)                     recovery worker thread
  HOLD + 判据命中
    state=RECOVERING, sub=deciding
    snapshot(target, q_now, tcp_now)
    sub=moving ─────────────────────▶ posture_.compute → decide
                                      prepositioner_->move_to(...)
                                        (preposition_in_progress_=true)
  (定时器照常跑:state/看门狗/服务)
    sub=verifying ◀─────────────────  move 返回
    复核 q_now
    sub=reseeding ──────────────────▶ call_trigger_sync(reseed_client_, "start_servo")
    unpause_servo
    → TRACKING / HOLD(exhausted)
```

约束:

- `move_to()` 阻塞,只能在 worker 线程;定时器不得持有 `mutex_` 等待;
- 沿用 `preposition_in_progress_` + `MoveGroupPrepositioner::cancel()`;
- `handle_release()` / TRACKING→HOLD 的取消路径必须能打断 recovery;
- 恢复期间仍要发布 `/lgw/servo/state`(`state=RECOVERING` + `hold_reason` 可见);
- 成功后**必须**先 `start_servo` 再 `unpause`(原因见 §12);
- 移动完成后保留现有 settle 等待,但恢复场景可缩短(`recovery_settle_ms`,默认 200 ms);
- 恢复期间不发布 twist(`update_tracking_locked` 在 HOLD 分支已提前返回)。

## 8. 参数(建议默认)

| 参数 | 默认 | 说明 |
|---|---|---|
| `hold_recovery_enable` | **`true`** | **只控制 HOLD 自动恢复**(自动摆位 / follower 自动 resume)。`false` 时不会自动动臂、不会自动 resume。takeover/resume 的 Servo 缓存 reseed 是无条件修复(2026-11-02),不受此开关影响 |
| ~~`recovery_reasons`~~ | 未实现 | v1 白名单固定在纯模块(`servo_halt`/`saturated`,以及开关控制的 `target_lost`);没有字符串列表参数(2026-11-03 实施决定) |
| `recovery_allow_target_lost` | `false` | 是否用最后已知几何做恢复移动 |
| `recovery_max_attempts` | `2` | 每次 HOLD 会话内 |
| `recovery_deadline_ms` | `4000` | 从进入 HOLD 起算 |
| `recovery_geometry_max_age_ms` | `1500` | 仅 `target_lost` |
| `recovery_velocity_scaling` | `0.10` | 比 takeover 的 0.3 更慢 |
| `recovery_timeout_s` | `8.0` | 单次 MoveGroup goal |
| `recovery_tolerance_rad` | `0.02` | 复核 |
| `recovery_settle_ms` | `200` | 移动后等待 |
| `follow_resume_gap_rad` | `0.10` | `follower_lag` 的恢复阈值 |
| `follow_resume_stable_ms` | `300` | 连续满足才 resume |
| `follow_recovery_timeout_ms` | `3000` | 超时 → FAULT/exhausted |

参数放在 `servo_node.yaml` 的 `lgw_servo_node.ros__parameters` 下,与现有键同级。

## 9. 失败路径

| 事件 | 结果 |
|---|---|
| `posture_.compute` 抛异常 / `family.ok=false` | `RECOVERING -> HOLD`,`recovery_exhausted=true` |
| `move_to` 失败/超时/被取消 | 同上;若被 release 取消则不 unpause |
| 复核 `max_joint_delta > tolerance` | 同上(不带着坏姿态恢复) |
| `start_servo` 失败/超时 | `pause_servo` + `RECOVERING -> FAULT`(不进 TRACKING) |
| 恢复成功但目标仍不可用 | `RECOVERING -> HOLD(exhausted)`;不先进 TRACKING 再掉回来 |
| release 在恢复中 | 不取消移动;Servo paused;RELEASING → IDLE;worker 不再 reseed/unpause;移动走到族姿态 |
| `follower_lag` 恢复超时 | FAULT(驱动/链路问题,不重试移动) |

## 10. 对外契约

`ServoState.msg`:

- `state` 增加取值 `RECOVERING`(IDLE / PREEMPT / TRACKING / HOLD / **RECOVERING** /
  RELEASING / FAULT);
- 新增字段(向后兼容,只加字段):

```
string hold_reason            # 见 §4 表
uint32 recovery_attempts
bool   recovery_exhausted
bool   resumable              # TARGET_OK latch 已置位
builtin_interfaces/Time hold_since
```

`recovery_phase` 不再导出(顶层状态已经表达"正在主动恢复");内部子阶段只写日志。

外层 BT 动作:

- `state ∈ {HOLD, RECOVERING}` 都表示"控制权还在节点、臂不归你",动作继续等待;
- SUCCESS 判据仍由外层决定(建议用 `converged` + 连续稳定时间);
- 动作超时必须 ≥ `recovery_deadline_ms + recovery_timeout_s`;
- `recovery_exhausted=true` 或 `state==FAULT` → 动作 FAILURE。

## 11. 复用与新增

复用:

- `PostureFamily::compute()`(`lookat_posture_family.cpp`)
- `decide_preposition()`(`preposition_decision.cpp`)
- `MoveGroupPrepositioner`(`move_group_prepositioner.cpp`)
- `follow_gap_locked()` / `commanded_advancing_locked()`
- `move_group_busy_locked()`、`preposition_in_progress_`、取消路径
- `call_trigger_sync()` + `reseed_client_`(2026-11-02 新增)

新增:

- 纯逻辑 `hold_recovery_decision.{hpp,cpp}`:输入(原因、次数、时限、几何年龄、
  family 状态)输出(是否移动 / 是否 exhausted),可单测;
- `ServoNode` 的 recovery worker + 内部子阶段;
- 状态机 `ServoState::RECOVERING` 与 §6.1 的迁移;
- **同步更新** `2026-10-26-lgw-servo-takeover-design.md` §7.1 迁移表、
  `test_servo_state_machine.cpp`、`ServoState.msg` 的状态注释;
- yaml 参数与 `ServoState` 新字段。

## 12. 前置项:resume 的 Servo 缓存

`moveit_servo` 的 `last_sent_command_` 只在 `ServoCalcs::start()` 播种;
控制律在死区内发**逐位为零**的 twist(`servo_control_law.cpp:131-145`),
Servo 对零 twist 原样复播该缓存。

因此 **HOLD 内一旦移动过机械臂,resume 必须重新播种**,否则恢复后的首拍零指令
会把臂拽回移动前的姿态。实现与 2026-11-02 的 takeover 修复完全相同:
`unpause` 之前调用一次 `start_servo`(`ServoCalcs::start()` 会 `stop()` + 重新播种,
官方语义允许重复调用,见 spec §7.9 修订)。

该前置项独立于本设计,可先提交、先验证。

## 13. 测试与验证

单元:

- `test_hold_recovery_decision.cpp`:原因白名单、次数/时限、几何年龄、family 不可解、
  `follower_lag` 永不返回"移动"。
- `test_servo_state_machine.cpp` 扩展 §6.1 的迁移:
  HOLD→RECOVERING、RECOVERING→TRACKING/HOLD/RELEASING/FAULT、
  RECOVERING 中 RESUME 被拒、PAUSE 幂等、TARGET_LOST/TARGET_OK 只改 latch。

mock bench(`follow:=false`,驱动必须停):

1. 构造 `servo_halt`:把目标放在当前姿态附近但命令穿越腕奇异,确认触发 HOLD 后节点
   自己摆位并回到 TRACKING;
2. 构造 `target_lost` + 最后已知几何:停掉 fake_track,确认 `recovery_allow_target_lost`
   打开时移动、关闭时不移动;
3. 构造 `follower_lag`:mock 下没有真机滞后,用注入方式(直接调用判据函数)验证
   "不移动 + 等 gap"的分支;
4. release 打断恢复:移动中调 release,确认回到 IDLE、不再 unpause/TRACKING;
   有界移动走完到族姿态是预期行为。

真机(`follow:=true`):

- 先用 `recovery_velocity_scaling=0.10` 验证一次奇异恢复;
- 检查 `/lgw/servo/state` 的 `state=RECOVERING`/`attempts`/`exhausted` 与日志一致;
- 验证恢复后首条 Servo 输出姿态 = 移动后姿态(`check_servo_zero_command.py` 的在线版)。

## 14. 开放问题

1. ~~`hold_recovery_enable` 默认 false 还是 true?~~ **已定:true**(2026-11-03)。
2. `target_lost` 是否值得做"最后已知几何"恢复,还是完全留给外层重观察?
3. RECOVERING 期间收到外部 `takeover` 请求被拒绝(非 IDLE);若将来需要"热切换目标",
   应新增显式事件而不是放宽这条规则。

### 14.1 行走中恢复(本计划不实现,保留扩展点)

恢复时平台可能在行走。当前版本的假设是"平台静止或慢速",不做门限;但接口必须为
将来预留,避免届时改签名:

- `HoldRecoveryInput` 预留 `bool platform_motion_ok{true}`(或 `double platform_speed_mps`);
  节点从未来的 odom/sport 状态输入填充;纯函数 `decide_hold_recovery()` 增加一条
  `!platform_motion_ok -> kNone` 即可,**调用方签名不变**;
- MoveGroup 的碰撞检查使用规划时刻的机器人位姿,行走时进行中会变旧;将来若开启
  行走恢复,需要同时评估"规划/执行期间平台运动"的碰撞语义(可能要求限速或暂停平台);
- 本版本文档必须写明该假设,bench 与真机验证都在平台静止条件下进行。

实现本计划时**不写**这个字段(避免死代码),只保证 `HoldRecoveryInput` 是唯一
需要扩展的结构、`decide_hold_recovery()` 是唯一的判据入口。

## 15. 实施结果与真机验证清单(2026-11-03)

已实现并验证(mock):

- `RECOVERING` 顶层状态与 §6.1 迁移(`test_servo_state_machine` 25 tests);
- 纯判据 `hold_recovery_decision`(`test_hold_recovery_decision` 18 tests);
- `hold_reason` / `recovery_*` / `resumable` 字段;
- `handle_resume` 先 reseed 再 unpause(`check_servo_resume_reseed.py` RED→GREEN);
- RECOVERING worker:姿态族 → MoveGroup → 复核 → reseed → 零指令探针 → unpause;
- `follower_lag` 冻结等待 + 自动 resume(纯函数单测;mock 无法触发);
- `bench_hold_recovery.sh`:A 场景 `HOLD → RECOVERING → HOLD(exhausted)`,关节移动
  0.285 rad;B 场景恢复中 release → IDLE,未进 TRACKING。

启动参数(不是运行时参数,节点在启动时读取一次):

```
hold_recovery_enable:=true
recovery_allow_target_lost:=false
```

### 15.1 真机验证清单

平台必须静止或慢速(§14.1)。引导脚本:
`bash scripts/bench_hold_recovery_real.sh <baseline|target_lost|release|saturated>`
(两次人工确认:takeover 摆位前、恢复移动前;预检驱动/反馈/无其他 tracker 发布者)。

实现注意:**移动后复核使用 `commanded_positions_`(`/control`,MoveGroup 实际驱动的域)**。
`follow:=true` 时 `measured_positions_` 是 `/feedback`(真臂,跟随时间常数 ~1 s),
用它复核会把一次正常移动误判为失败;真臂是否跟上由 following monitor 在 resume 后负责。

1. `agx_arm_ctrl` 运行、臂在 home、`enable=True`;
2. `ros2 launch lgw_bt_planner lgw_full.launch.py follow:=true ... hold_recovery_enable:=true`;
3. 把目标放到腕奇异附近触发 `servo_halt`,或制造 `saturated`;
4. 检查 `/lgw/servo/state`:`HOLD(servo_halt/saturated) → RECOVERING(attempt 1) → TRACKING`;
5. 恢复后立刻验证缓存:复用 `check_servo_zero_command.py`(把 `--joint-topic` 换成
   `/feedback/joint_states`)或观察首条输出 = 摆位后姿态;
6. `follower_lag`:用 `speed_percent` 降低制造持续滞后,确认 gap 闭合后自动 resume;
   若 3 s 不闭合应进 FAULT,而不是继续等;
7. 恢复移动中调 `/lgw/servo/release`:确认回到 IDLE、无 TRACKING、不再 unpause;
   有界移动走完到族姿态是预期行为;
8. 若出现"恢复后首拍跳回旧姿态",按 `docs/servo/2026-11-02-servo-state-monitor-stuck.md`
   §12 检查 reseed 是否真的执行。

### 15.2 真机结果(2026-11-03)

| 场景 | 结果 |
|---|---|
| `baseline`(开关关) | PASS:`TRACKING → HOLD`,无 `RECOVERING`;臂仅跟随冻结 JTC 收敛(<0.10 rad) |
| `target_lost` | PASS:`HOLD → RECOVERING → MoveGroup 摆位 → reseed → 探针 → HOLD(exhausted)`;关节移动 ~0.44 rad |
| `release`(新语义) | PASS:恢复中 release → `RELEASING → IDLE`;有界移动走完到族姿态;无 unpause/TRACKING |
| `resume` 缓存 | PASS:真臂收敛到目标(gap 0.018 rad)后 resume,首条输出与目标差 ≤0.019 rad |
| `saturated` | INCONCLUSIVE:远/高目标仍可被姿态族瞄准,10 s 内一直 TRACKING;该场景在当前 lookat 设计下不可靠 |
| `follower_lag` | 未触发:`speed_percent=20` + 慢扫目标下 gap 未持续 >0.45 rad/5 s;需要更持续的 lead(单向 ramp 或更低速度) |

真机暴露并修复的三处:

1. **pre-settle**:HOLD 后等真臂追上冻结的 JTC 再发恢复 goal(否则 JTC 以
   `allowed_start_tolerance 0.01` 拒绝);
2. **复核域**:移动后用 `commanded_positions_`(/control)而非 `/feedback`;
3. **takeover 失败回 IDLE**:PREEMPT 阶段失败只发 RELEASE 会永久停在 RELEASING
   (review #12),现在补 `pause + PAUSE_OK`。

环境注意:宿主机重启后 `can0` 需重新 `can_activate.sh can0 1000000`;停掉 tracker 时
必须补 `scripts/odom_tf_bridge.py`,否则 takeover 报 `no_base_pose`。
