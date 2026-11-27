# 注视位姿生成重构:固定姿态族(j3/j5)+ j2 俯仰辅助

- 日期:2026-10-30(rev2, 并入 builder review 的 7 个问题);
  2026-11-05 rev3(双档姿态族,见 §10)
- 状态:rev2 已实施;rev3 已实施并通过单测/mock,待真机验证
- 作者:Claude Code(与用户共同评审)
- 上游基线:`2026-10-26-lgw-servo-takeover-design.md` §7.5、
  `2026-10-29-lgw-lookat-standoff-redesign.md`(target_relative,未启用)
- 参考实现:`scripts/lookat_posture_proto.py`(C++ 必须与它逐位对齐)、
  `scripts/lookat_reachability.py`、`scripts/lookat_singularity_sweep.py`、
  `scripts/lookat_envelope.py`

---

## 0. 摘要

现有的两条 lookat 规则(基座相对 `xoy/z_fixed`,目标相对 standoff)都是**先定相机
位置、再解 6 自由度姿态**。2026-10-30 的真机测试暴露出这种做法的根本问题:同一套
参数在目标方位变化时会把腕部推向奇异或推到关节限位,而规则本身对这个后果没有任何
约束力。

本文改成**先定姿态、再反推相机位姿**:

```
钉死  j3 = −0.484 rad        (决定相机的基准半径/高度 = 防撞包络)
钉死  j5 = +0.60  rad        (决定奇异余量:离 0 和 ±1.2217 都远)
联立解 j1 / j4 / j6          (方位 / 俯仰 / 画面水平,三关节耦合,必须一起解)
j2 在外层枚举                (扩展俯仰范围,受"相机高度 ≥ 0.20 m"与 cond ≤ 50 守卫)
输出:由此姿态做 FK 得到的 tcp_link 位姿(接口不变,Servo 侧零改动)
```

代价是**放弃一部分关节灵活性**:相机被约束在一个由姿态参数决定的曲面族上;换来的是
解唯一、连续、不撞限位、不进奇异。

**rev2 的重要修正**:原型跑通后发现,加上 0.10 rad 的限位余量之后,**地面目标
(0.40–1.20 m)可以精确命中(残差 0.00°,cond ≈ 25–28),但墙面目标不再达标**
(见 §5)。初版 §5 的墙面数字来自无余量、无 cond 检查的穷举搜索,已作废。

---

## 1. 问题与证据(2026-10-30 真机)

### 1.1 现有规则把腕部推向限位

平台移动测试(180 s,平台走动约 1 m)的日志:

```
126.6 s  servo_node joint4 close to a position limit. Halting.
130.6 s  servo_node joint5 close to a position limit. Halting.
164.0 s  servo_node joint4 close to a position limit. Halting.
166.9 s  TRACKING -> HOLD (servo_halted=1 ...)
```

`joint4` 从静止时 0.71 rad 一路爬到 1.59 rad(有效上限 1.645 = 1.745 − margin 0.1),
臂被冻结在 HOLD,整段任务中断。

### 1.2 根因是"共面"

现有规则把相机放在与目标**同一方位角**、同一高度上:

```
P_ee = ( xoy · t̂_xy , z_fixed )
```

相机、目标、基座 z 轴三者共面,整条臂在这个竖直平面内工作。实测 cond(J):

| xoy | z | roll | q5 | cond |
|---|---|---|---|---|
| 0.35 | 0.22 | 0° | −0.665 | 19.0 |
| 0.25 | 0.27 | 0° | — | 56.5 |
| 0.20 | 0.30 | 0° | −0.082 | **413.8** |
| 0.20 | 0.30 | +30° | −0.798 | 28.3 |

### 1.3 关节分工(以用户手摆姿态为基准,每关节 +0.1 rad 的响应)

| 关节 | 方位变化 | 俯仰变化 | 相机位移 |
|---|---|---|---|
| j1 | **+5.7°** | 0.0° | 高度/半径不变(绕 z 转) |
| j4 | −0.5° | **−5.3°** | 1.4 cm |
| j5 | +8.7° | −1.2° | 1.3 cm |
| j2 / j3 | +7.7° | +0.7° | 2–3 cm(改变相机半径/高度) |

注意:上表是**局部斜率**。j4 扫满行程会把光轴方位甩出 60°(见 §3.2),所以不能把
"j1 管方位、j4 管俯仰"当成解耦来用。

---

## 2. 目标 / 非目标 / 反目标

### 目标

1. 目标在**地面 0.40 m ~ 1.20 m** 时,光轴精确对准(残差 < 1°),cond < 50。
2. 任何工作姿态下 q5 恒定 = +0.60、j3 恒定,不会漂到限位。
3. 相机位置有硬边界:高度 ≥ 0.20 m、半径 ≤ 0.35 m。
4. 解**唯一且连续**:目标方位连续变化时,相邻两拍期望位姿差 < 5 mm / 2°。
5. 现有 Servo 控制律、状态机、HOLD 语义、`lookat_mode: legacy` 路径**完全不动**。

### 非目标

- 不追求任意目标位置都能看。
- 不做在线优化 / QP;不做 6 自由度 IK。
- 不改变相机标定、不改 Servo 参数、不改 JTC/mock 链路。

### 反目标(即使技术跑通也算失败)

- 为了覆盖更大的角度而让相机钻到机身里。
- 目标超包络时输出非法位姿,或让 Servo 撞关节限位 / 奇异停机。
- 姿态在两解之间来回跳(抖动)。
- 需要有经验的调参才能不撞限位 —— 约束必须由构造保证。

---

## 3. 设计

### 3.1 姿态族定义

```
固定:  j3 = posture_j3_rad      (−0.484)
       j5 = posture_j5_rad      (+0.60)
可变:  j1 ∈ [−2.618, 2.618]     方位(有效性见 §3.5 余量)
       j4 ∈ [−1.745, 1.745]     俯仰
       j6 ∈ [−2.094, 2.094]     画面滚转
       j2 ∈ [posture_j2_min_rad, posture_j2_max_rad] = [0.10, 1.571]
```

**j2 的选择语义(rev2 明确)**:j2 不是自由冗余,也不使用启动时预生成的单一"最小
可行 j2"表。运行时按**固定候选顺序**枚举,取第一个通过全部守卫的姿态:

```
候选顺序 = [上一周期的 j2, posture_j2_nominal_rad, prev±0.15, prev±0.30,
            j2_min, j2_max]        去重、夹到 [j2_min, j2_max]
```

第一个候选由上一周期提供,既保证连续性,也是 DLS 收敛的必要初值(见 §3.2)。
`posture_j2_nominal_rad` 只作为"冷启动/上一周期不可用"时的首选,不是硬性目标。

### 3.2 每周期求解(三关节联立)

记 `cam(q)`、`R(q)` 为整条臂的 FK 输出(相机位置、相机朝向),钉住 j3/j5,
未知量 `x = [j1, j4, j6]`:

```
u      = normalize(p_target − cam(q))          目标方向
x_des  = normalize(z_world × u)                相机 x 轴保持水平(uprightness 分支见 §3.6)
y_des  = u × x_des ,  R_des = [x_des y_des u]
e_axis = cross(a_cur, u)                       3 维,光轴未对准时非零
e_roll = x_cur · y_des                         1 维,图像未摆正时非零
e      = [e_axis ; roll_weight · e_roll]       roll_weight = 0.25(对准优先)
J      = ∂e/∂x   (有限差分,3 次额外 FK)
Δx     = −pinv(J)·e ,  单步截断 0.2 rad
x      ← clamp(x + Δx, ±限位 ∓ posture_joint_margin)
迭代至 |e_axis| < 1e-5 或 25 次上限
```

**为什么必须联立**:j4 扫满行程会把光轴方位甩出 ±60°。先解 j1 再解 j4 会互相破坏。
**为什么必须用上一拍解做初值**:DLS 有局部极小,单初值在墙面目标上会收敛到 j4 的
错误一侧(原型实测残差 44°,多初值后收敛到 6.5°)。

外层对每个 j2 候选跑一次上述解算,校验:

```
aim   = acos(u · a_cur)                   必须 ≤ posture_aim_tol_rad (2°)
cond  = σ_max(J_full6) / σ_min(J_full6)   必须 ≤ posture_cond_max (50)
cam.z ≥ posture_cam_min_height_m (0.20) 且 |cam.xy| ≤ posture_cam_max_radius_m (0.35)
```

第一个全部通过的候选即为本周期结果;若都不通过,取 aim 最小者并在状态里置
`kOutOfEnvelope`,**位姿仍必须满足 §3.5 的守卫**(当初版与守卫冲突时,守卫优先,
必要时退回上一周期的合法位姿)。

### 3.3 为什么是 j1/j4/j6

§1.3:j1 是纯 pan、j4 是纯 tilt;j6 对光轴影响小(±1.9°/0.1rad)但对画面滚转影响最大,
用它保持图像水平,让感知看到的是正图。

### 3.4 计算量

25 次迭代 × 4 次 FK + 6 个 j2 候选 ≈ 600 次 FK/周期。C++ 下 MoveIt `RobotState` FK
约 5–10 µs → 3–6 ms,占 100 Hz 周期的 30–60%,偏高。**实现约束**:

- 内层迭代上限降为 8(以上一拍解为初值时实测 3–4 次即收敛);
- j2 候选按需提前退出(第一个通过即止,典型 1 个);
- 若单周期超时,沿用上一周期的解并置 `kOverrun`,不阻塞控制律。

### 3.5 守卫(硬约束,任何情况下不放松)

| 守卫 | 阈值 | 依据 |
|---|---|---|
| 相机高度 ≥ `posture_cam_min_height_m` | 0.20 m | j2 走满时相机可低到 0.062 m、半径 0.015 m(机身内) |
| 相机半径 ≤ `posture_cam_max_radius_m` | 0.35 m | 实测最大 0.326 m,留余量 |
| 关节距限位 ≥ `posture_joint_margin` | 0.10 rad | 与 Servo `joint_limit_margin` 同值,否则 Servo 判"接近限位"停机 |
| cond(J) ≤ `posture_cond_max` | 50 | = `servo_singularity_lower`,以上 Servo 会减速 |

j1 的有效范围因此是 ±2.518(而非 URDF 的 ±2.618):超出的目标方位被夹住并置
`kGuarded`,不抛异常。

### 3.6 退化与异常(沿用 legacy 契约)

| 情况 | 行为 |
|---|---|
| 目标在基座 z 轴上(`r_xy < 1e-3`)或与相机重合 | **抛异常 → 节点 HOLD**(与 `compute_lookat_setpoint` 一致,绝不把 NaN 送进 Servo) |
| 图像水平解需要 j6 超出限位 | 夹 j6,置 `kGuarded`,允许图像歪(歪多少由解算结果决定,不另设阈值) |
| 所有 j2 候选都不满足守卫 | 退回上一周期的合法位姿,置 `kOutOfEnvelope` |

**uprightness 分支**:沿用 `apply_uprightness_flip` 的约定(cam_y 朝下),与现有
lookat 路径一致,避免同一目标在两套规则间切换时画面翻转 180°。

`posture_roll_limit_rad` 参数**删除**:约束加在关节限位上,不加在图像滚转角上
(两者不是同一个量,j6 的夹值不能等价为图像歪多少)。

### 3.7 运动学链的来源

模块需要 FK:关节角 → `tcp_link` 位姿 → 相机位姿。来源与约束:

- **由 `lgw_full.launch.py` 把 `moveit_config.robot_description`(已含 `tcp_offset`
  与 `arm_type` 的展开结果)作为参数传给 `lgw_servo_node`**;
- 模块启动时用 `kdl_parser` 解析该 URDF,构建链并缓存(新增依赖
  `kdl_parser` / `orocos_kdl`,纯函数单测用同一份 URDF fixture);
- 相机位姿 = `FK(tcp_link) ∘ (R_cam_in_tcp, t_cam_in_tcp)`,与现有
  `compute_tcp_pose` 的手眼链**同源同定义**,保证与 TF 里的 `arm1/tcp_link` 一致。

不硬编码 piper 链:launch 的 `arm_type` / `tcp_offset` 变化时必须静默跟随,否则
期望位姿会带常数偏置。

### 3.8 takeover 内的 pre-position(rev3,改为节点内实现)

稳态数学正确不代表过渡段安全:takeover 时臂的实际姿态可能在另一个腕分支(q5 为负),
DLS 的第一段会穿过 q5≈0 的奇异,正是 §1.1 要消除的故障。

rev2 把这一步放在外部脚本里。rev3 改到**节点内**:接管的职责本来就是"把臂带进可伺服的
状态",把它拆给调用方意味着每个调用方(BT / 人工脚本)都要重新实现一遍判据和摆位,而
`servo_node.cpp` 里已经有现成的阻塞等待模式(`MultiThreadedExecutor(2)` + 不进主执行器
的 callback group + 局部 `SingleThreadedExecutor`)。

#### 触发判据(任一成立 → 摆位)

记 q_family 为模块对当前目标解出的关节角,q_now 为 `pose_watch_topic` 上的实测关节角:

| # | 判据 | 参数 | 默认 |
|---|---|---|---|
| 1 | `max_j |q_now − q_family| > ε_j` | `preposition_joint_delta_rad` | 0.15 |
| 2 | **q5 跨越 0**(`q5_now` 与 `q5_family` 异号)——不是"差多少",而是"在哪一侧" | `preposition_branch_delta_rad`(同侧但过远的兜底) | 1.571 |
| 3 | `min_j(距限位) < ε_l` | `preposition_limit_margin_rad` | 0.10 |
| 4 | `|tcp_now − tcp_family| > ε_d` 或夹角 > ε_a | `preposition_tcp_distance_m` / `_angle_rad` | 0.05 m / 0.15 rad |

#### 流程(全程停在 PREEMPT)

```
try_takeover_locked() 成功(目标已选中,状态 = PREEMPT)
  → 用与控制拍相同的 select_source() 求 p_target(base 系)
  → posture_.compute(p_target) → q_family / tcp_family
  → 按下表判定;不需要摆位 → 直接 unpause
  → 需要摆位:
      **不持 mutex_** 发 MoveGroup joint goal,局部执行器等结果(带超时)
      → 成功:重读 q_now 复判一次;仍不满足 → 拒绝接管(IDLE)
      → 把实测 q_now 写入 previous_,使第一拍热启动
  → unpause_servo → TRACKING
```

#### 失败与打断

| 情况 | 行为 |
|---|---|
| goal 被拒 / abort / 超时 | takeover 返回失败,**状态回 IDLE,不 unpause** |
| 摆完复判仍不满足 | 同上,并在 message 里带四项实测量 |
| PREEMPT 期间收到 `release` | cancel 掉 goal,再走正常 release |
| `lookat_mode != pinned_posture` | 跳过判据,直接接管(没有"族"可摆) |
| `preposition_enable := false` | 跳过判据(真机排障用的 kill switch) |

#### 速度

goal 的 `max_velocity_scaling_factor` 取 `preposition_velocity_scaling`,默认 **0.3**;
**0 表示"不指定"**,由 `agx_arm_moveit/config/joint_limits.yaml` 的
`default_velocity_scaling_factor: 0.1` 决定。

> 注意:`lgw_full.launch.py` 的 `servo_velocity_scaling`(0.8)只作用于
> `moveit_servo.override_velocity_scaling_factor`,**不影响 MoveGroup**;驱动的
> `speed_percent` 只是固件的关节速率百分比,不是规划边界。写 1.0 等于把 MoveIt 的
> 0.1 默认值覆盖掉,且计划速度超过固件能力会让 JTC 因路径容差 abort。

#### 可观测性

`/lgw/servo/state` 增加 `needs_preposition` 与 `max_joint_delta` 两个字段,调试时能直接
看出"刚才是否因为族外姿态被摆位"。

#### 代码结构(为可测性)

新增 **STATIC** 小库 `lgw_posture`,含 `lookat_posture_family.cpp`、
`preposition_decision.cpp`(判据纯函数)、`calibration_json.cpp`(把
`parse_calibration_json` 从 BT 节点文件里搬出)。巨型库 `lgw_bt_planner_nodes` 改为
链接它;单元测试只链小库,避免每次改动重链 5 MB 的 BT 插件库。

---

## 4. 参数(servo_node.yaml)

```yaml
lookat_mode: pinned_posture        # legacy | target_relative | pinned_posture
# rev3 (2026-11-05): two posture profiles; primary first, fallback second.
posture_j3_rad: -0.720             # primary elbow fold
posture_j5_rad: -0.800             # primary wrist bend (forward/level/up)
posture_j3_alt_rad: -0.484         # fallback = the rev2 field-proven posture
posture_j5_alt_rad: 0.600
posture_j2_step_rad: 0.150         # uniform j2 candidate grid after the priority list
posture_j2_nominal_rad: 0.890
posture_j2_min_rad: 0.100          # URDF 下限 0 + Servo joint_limit_margin 0.1
posture_j2_max_rad: 1.571           # 90°
posture_cam_min_height_m: 0.200
posture_cam_max_radius_m: 0.350
posture_joint_margin: 0.100
posture_cond_max: 50.0
posture_aim_tol_rad: 0.035          # ~2°
posture_max_overrun_us: 4000        # 单周期解算超时,超时沿用上一拍
```

`lookat_mode` 默认仍是 `legacy`,新路径需显式打开。

---

## 5. 覆盖能力(原型实测,站立,基座距地 0.37 m)

`scripts/lookat_posture_proto.py` 按 §3.2 实现,含全部守卫:

| 目标 | aim 残差 | cond(J) | 相机高度 | j2 | 结论 |
|---|---|---|---|---|---|
| 地面 0.40 m | **0.00°** | 27.5 | 0.23 | 1.08 | ✅ |
| 地面 0.75 m | **0.00°** | 26.5 | 0.30 | 0.96 | ✅ |
| 地面 1.20 m | **0.00°** | 25.1 | 0.34 | 0.96 | ✅ |
| 墙面 1.0 m / 高 1.2 m | 6.5° | **189.6 ✗** | 0.27 | 0.47 | ❌ cond 超限 |
| 墙面 0.8 m / 高 1.6 m | 9.1° | 36.7 | 0.31 | 0.10 | ⚠️ 偏心 9° |

**结论**:地面 0.40–1.20 m 完全达标;墙面目标需要 j4 顶到 ±1.645(margin 边界),
要么 cond 爆掉、要么偏出中心 6–9°。要真正覆盖高墙,只能放松 `posture_joint_margin`
(0.10→0.05 约换 3° 俯仰,仍不够)或让 j5 参与(回到灵活性问题)。

> **rev3 更新(2026-11-05)**:让 j5 取负值并换 `j3=-0.72` 后,墙面目标
> (0.8 m 前 / 1.23 m 高)已被主档覆盖,地面远距离也不再爆 cond。本节表格是
> rev2 的测量,结论由 §10 取代。

初版 §5 的"墙 1.0/1.2 → 1.0°,墙 0.8/1.6 → 6.4°"是**无余量、无 cond 检查**的
穷举结果,作废。

---

## 6. 与现有代码的关系

| 文件 | 变更 |
|---|---|
| `src/lookat_posture_family.{hpp,cpp}` | **新增**:URDF 解析 + FK + §3.2 解算(纯函数,可单测) |
| `src/servo_node.cpp` | 新增 `lookat_mode: pinned_posture` 分支;pre-position 校验;其余不动 |
| `test/test_lookat_posture_family.cpp` | **新增**单测(见 §7) |
| `config/servo_node.yaml` | 新增 §4 参数 |
| `launch/lgw_full.launch.py` | 给 `lgw_servo_node` 传 `robot_description` |
| `CMakeLists.txt` / `package.xml` | 新增 `kdl_parser`、`orocos_kdl` |
| `scripts/preposition_lookat.sh` | **新增**:takeover 前摆位 |
| `lookat_setpoint.cpp` / `lookat_standoff_setpoint.*` | **不动** |

输出接口不变:`tcp_desired` 结构体一致,控制律、状态机、Servo 参数零改动。

---

## 7. 验证

### 7.1 单元测试(容器内跑 build 下的 gtest)

1. **对准**:地面 0.40 / 0.75 / 1.20 m 三例,aim < 1°。
2. **恒定关节**:任意目标下 j3 == −0.484 且 j5 == 0.600(1e-9)。
3. **守卫**:随机目标 1000 个,相机高度 ≥ 0.20 m、半径 ≤ 0.35 m、所有关节距限位 ≥ 0.10。
4. **连续性**:目标方位 −140°→+140°(步长 1°,受 j1 余量限制),相邻两拍 tcp 位置差
   < 5 mm、姿态角差 < 2°。
5. **限位**:同 3,单独断言。
6. **超包络**:目标放到 2 m 高 → `kOutOfEnvelope`,且位姿仍满足 3/5。
7. **退化**:目标在基座 z 轴上 → 抛异常(与 legacy 一致)。
8. **奇异**:所有输出 cond ≤ 50。

### 7.2 碰撞验证(新增,离线)

新族相机半径只有 0.11–0.24 m(旧规则 0.35),离基座更近。§5 的六个姿态必须放进
**含实际碰撞体的 planning scene**(Go2 机身 box、head 相机 box、夹爪)用 MoveIt 的
`checkSelfCollision` / `checkCollision` 验一遍,作为离线验收项;失败则调整
`posture_cam_min_radius_m` 或 j3。

### 7.3 真机

1. 固定目标 → `preposition_lookat.sh` 摆到族内 → takeover;
2. 记录 `e_ang/e_pos/gap`;验收:全程无 `joint* close to a position limit`、无 HOLD;
3. 平台移动复测(同 2026-10-30 的流程)。

---

## 8. 已知风险

| 风险 | 影响 | 对策 |
|---|---|---|
| 墙面目标覆盖不足 | 偏心 6–9° 或 cond 超限 | 明确列为非目标;需要时另设姿态 |
| 计算量接近 100 Hz 预算 | 单周期超时 | 迭代上限 8、候选提前退出、超时沿用上一拍 |
| FK 链与 TF 不一致 | 期望位姿带常数偏置 | §3.7 用同一 URDF 源;单测比对 TF |
| 过渡段穿奇异 | takeover 瞬间 HOLD | §3.8 pre-position |
| 新姿态贴近机身 | Servo 碰撞 halt | §7.2 离线碰撞验证 |

---

## 9. 待定(需用户确认)

1. **takeover 前置摆位是否可接受**(§3.8):会多一步操作,但能避免过渡段穿奇异。
2. ~~**墙面目标是硬需求还是加分项**~~:rev3 已用主档 `(-0.72, -0.80)` 覆盖,
   见 §10(不再需要放松限位余量)。

---

## 10. rev3(2026-11-05):双档姿态族

### 10.1 动机(实测)

1. **行走时远目标硬停**:2026-11-04 真机,目标被拉到 1.27 m(机身高度)时
   cond 越过 100,Servo `servo_halt` → `HOLD(saturated)`。rev2 的
   `(-0.484, +0.60)` 在 1.2–1.6 m 机身高度 cond 已到 62–193。
2. **j3 是远距离杠杆**:`scripts/lookat_j3j5_sweep.py` 全网格 minimax 显示
   `j3=-0.72` 把最坏 cond 从 193 压到 41。
3. **j5 是俯仰杠杆,rev2 的扫描漏了负半轴**:固定其余关节只转 j5,光轴俯仰从
   +30°(j5=-1.2)扫到 -90°(j5=+0.9),j5=0 是奇异。rev2 只测了 j5≥0.4,
   因此错误地得出"俯仰上限 +15°"。
4. **两组失败区互补**(`scripts/lookat_axis_coverage.py`、
   `lookat_j3j5_sweep.py`):

   | 目标 | 主档 (-0.72, -0.80) | 回退档 (-0.484, +0.60) |
   |---|---|---|
   | 0.5–1.6 m 机身高度 / 地面 ≥1.0 m | cond 13–19 | cond 18–40,1.2–1.6 m 处 58–575 |
   | 抬头到 +45° | 可解(cond 23–48) | 不可解 |
   | 近距离低目标(d ≤ 0.6,地面) | aim 10–26° ✗ | aim 0 ✓ |
   | 远侧方位(±139°) | 不可解 | 可解(rev2 已验证) |

   32 个距离×高度目标**并集 0 失败**;35 个方位×俯仰格并集 91%,
   剩余 3 格经加密种子验证为求解器种子问题(可达)。

### 10.2 设计

- 姿态档位升级为**完整 (j3, j5) 对**,主档 `(-0.720, -0.800)`,
  回退档 `(-0.484, +0.600)`;回退档就是 rev2 的原参数,保留它的全部现场验证。
- 主档优先;每档**均分** `posture_max_overrun_us` 预算,避免主档搜索吃光预算
  把回退档饿死(预算耗尽正是需要回退档的场景)。
- **滞回**:上一拍用的是回退档,则本拍先试回退档;否则先试主档。换档是腕部分支
  切换,不能每拍来回跳。
- j2 候选 = 原优先级列表(上一拍 / nominal / ±0.15 / 上下限)+ **均匀网格**
  (`posture_j2_step_rad=0.15`)。六候选会漏掉高俯仰负方位侧的可达解。
- j4 额外种子扩到 `±1.5, ±1.2, ±0.8`。**必须保留 ±1.2**:删掉会回归 ±139°
  方位(回退档在该方位只从 ±1.2 起解)。

### 10.3 参数(覆盖 §4 的对应行)

见 §4 的 yaml 块(已更新)。

### 10.4 代价(实测)

- mock 栈、500 拍窗口:`[posture] solve mean 0.10 ms max 0.28 ms`,
  与 rev2 的 0.10 / 2.38 ms 持平(静止目标第一候选即命中并提前退出)。
- 最坏情况(主档全搜索后走回退档)约为单档的 2–3×,仍低于 4 ms 预算;
  超预算时沿用上一拍(`overrun`)。

### 10.5 行为变化与已知限制

1. **同档连续性上界变成 j2 网格步长**:0.15 rad ≈ tcp 28 mm。需要更平滑就调小
   `posture_j2_step_rad`,代价 ∝ 1/step。单测已按此上界断言。
2. **换档是一次性姿态跳变**:单测限定整个 260° 扫描 ≤2 次、跳变 < 0.30 m。
   真机行走时必须观察换档瞬间的 Servo 指令;必要时在节点侧加"仅 HOLD 时换档"。
3. **墙面目标从"非目标"变成能力**:0.8 m 前、1.23 m 高(≈+47°)由主档覆盖
   (rev2 为 aim 6.5° / cond 190)。单测已改为正向断言。
4. **相机半径 0.30–0.34**(主档),接近 `posture_cam_max_radius=0.35`;§7.2 的
   碰撞复验尚未重做。
5. 主档 `j5=-0.80` 距可用下限(`-1.2217 + 0.1`)还有 0.32 rad,余量充足。

### 10.6 验证

- 单测 15 个姿态族用例(新增:回退档覆盖近距低目标、主档优先、墙面目标可解、
  超包络仍标记),全套 154 tests / 14 suites 通过。
- mock 端到端:地面 0.75 m 目标 takeover → `TRACKING`,pos 8 mm / ang 0.027 rad,
  preposition 自动选中回退档(`q=[..., -0.484, ..., 0.600, ...]`)。
- **待做**:真机行走验证(重点:换档瞬间的指令跳变、近距低目标的实际画面)、
  碰撞复验。

### 10.7 追加(2026-11-05 晚):相机半径上限 0.35 → 0.40

**动机**:真机行走时目标被拉到 tcp 距离 0.478 m,观察族解不出合法姿态 →
`HOLD(servo_halt, exhausted)`。实验发现近距离的瓶颈是 `posture_cam_min_height_m`
(相机要降下来看近目标),而不是瞄准或 cond。

**半径上限的影响**(C++ 求解器实测,36 格:0.35–1.6 m × 高度 +0.15/0/−0.20/−0.37 × 侧偏 −0.2):

| 配置 | 漏检 | 最坏 cond |
|---|---|---|
| 半径 0.35 | 10/36 | 40.8 |
| **半径 0.40** | **0/36** | — |

半径 0.35 时,一些本来可用、cond 更好的外伸姿态被守卫拒绝,求解器被迫退到折叠姿态
(cond 更差)。放宽到 0.40 后不仅覆盖补齐,最坏 cond 也从 40.8 降到 32.9(由
`P1+P2` 两档、半径 0.40、C++ 求解器测得)。

**第三档(高相机俯视,j3≈-1.29 / j5≈+1.05)评估后放弃**:
- 手工摆姿验证该姿态真实存在(光轴 −85°、cond 12);
- 但 0.20–0.30 m 目标即使有它仍然 aim 15–49°(不可达);
- 0.35–1.6 m 用现有两档已 0 漏检,第三档不带来任何增益,只增加 50% 求解开销。

**仍未覆盖**:0.20–0.30 m(无论机身高度还是地面)。这是相机高度下限与腕部限位共同
决定的硬边界,只能靠任务侧保持距离(≥0.35 m)或降低 `posture_cam_min_height_m`
(需碰撞复验)。

**注意**:Python 原型 `lookat_posture_proto.py` 对侧向目标会低估覆盖——C++ 求解器
用目标方位解析地给 j1 播种,原型没有。调参一律以 C++ 结果为准。
