# Servo 速度衰减调查记录（2026-10-28 ~ 10-30）

- 状态:调查完成度 ~80%,仍有 2 个量化因子待定位
- 相关 spec:`docs/superpowers/specs/2026-10-26-lgw-servo-takeover-design.md`
- 复现脚本:`scripts/record_servo_twist.py`、`scripts/record_platform_motion.py`、
  `scripts/lookat_reachability.py`

---

## 1. 现象

平台移动时机械臂"很慢",跟不上;多次被 HOLD 打断。最初表现为:

- `moveit_servo` 日志持续 `Moving closer to a singularity, decelerating`;
- 跟随监控 `gap = /control − /feedback` 长期贴在 0.15 rad;
- 加大 `v_max`、调整注视点后仍不见好转。

## 2. 定量方法(本次调查的关键)

只测"跟随误差"不够——它只能说明臂落后于 mock,说明不了 mock 是否忠实执行了 twist。
真正有效的是**三方对账**:

```
我们发出的 twist  →  雅可比  →  理论上应产生的关节速度 qdot_exp
                                实际 Servo 输出的关节速度 qdot_obs (/control 微分)
                                →  ratio = |qdot_obs| / |qdot_exp|
```

工具:`scripts/record_servo_twist.py` 同步录 `delta_twist_cmds` + `/control` + `/feedback`;
离线用 `jacobian_condition.py` 的链式雅可比算 `qdot_exp = pinv(J)·twist`。

**实测特征**:ratio 恒为 0.20~0.25,**与 twist 大小无关**,且**所有关节同一个系数**
→ 说明不是限位、不是 singular direction,而是某个作用在 `delta_theta` 全体上的**标量**。

## 3. 已排除的假设(均有一次实测证据)

| 假设 | 证据 | 结论 |
|---|---|---|
| 机械臂本体速度上限 | joint5 用 100% 缩放走 1.23 rad / 1.2 s(≈1.2 rad/s);MoveGroup 峰值 lag 0.20 rad、1.3 s 收敛 | ❌ 臂能跑 1.2+ rad/s |
| 关节限速(模型 5 rad/s) | `qdot_exp` 最大 2.6 rad/s | ❌ 未触及 |
| JTC 执行不忠实 | 按关节名对齐后 servo_rate ≡ ctrl_rate,`|traj−control|` ≈ 0.000~0.01 | ❌ JTC 忠实 |
| 奇异减速 | 运动全程 `/servo_node/status = 0`;日志无 `Very close to a singularity` | ❌ |
| 碰撞接近缩放 | status 0;`check_collisions` 关掉后 ratio 不变 | ❌ |
| IK 近似解(`return_approximate_solution`) | 让 Servo 走雅可比路径(见 §6)后 ratio 不变 | ❌ |
| 固件 `speed_percent` 残留 | 以 `speed_percent:=100` 重启驱动后,静态跟随 0.15 → **0.0155 rad** | ✅ 真实存在但已修复,不是主因 |
| 我们自己的控制律 | twist 实测打满 `|v|=0.45 m/s`、`|w|=1.0 rad/s` | ❌ 指令侧没偷懒 |
| Butterworth 平滑系数 1.5 | 改成允许的最小值 1.01,ratio 0.22 → 0.25(几乎不变) | ❌ 不是主因 |
| 瞬态瞬态效应(滤波器的阶跃衰减) | 用**正弦扫掠**做稳态测量(目标 ±0.15 m @0.2 Hz,模拟匀速行走):ratio **0.18**,比阶跃的 0.38 还低 | ❌ 衰减是**持续**的,与运动形态无关 |
| Butterworth 平滑滤波器 | 写直通插件 `lgw_servo_smoothing` 替换后,注入同一 twist:末端转角 0.048 → 0.052 rad(几乎不变) | ❌ **不是滤波器** |
| Servo 状态源(`joint_topic`) | 用 `servo_joint_domain:=feedback` 做 A/B:mock 域 3.0° vs 真机域 **1.6°** | ❌ 改回真机**更差**(印证 A' 的选择),状态源不是主因 |

附:`ros2 topic hz` 实测 twist 输入 99.4 Hz、轨迹输出 100.0 Hz、`/control` 200 Hz,
排除采样/时序因素。

## 4. 确认存在的两个衰减器

把 `moveit_servo.override_velocity_scaling_factor` 从 0(不覆盖)改成 **1.0**
(直接跳过 Servo 内部的限速缩放):

| 配置 | ratio | 峰值关节速度 |
|---|---|---|
| coeff 1.5, 无覆盖 | 0.22 | 0.54 rad/s |
| coeff 1.01, 无覆盖 | 0.25 | — |
| **coeff 1.01, 覆盖 1.0** | **0.38** | **1.03 rad/s** |

⇒ 两个因子叠加:
- `getVelocityScalingFactor()` ≈ **0.66**;
- 平滑滤波闭环 ≈ **0.58**。

(0.66 × 0.58 ≈ 0.38;再乘 1.5 系数的额外衰减 ≈ 0.22,与首测吻合)

## 5. 机制说明(源码)

### 5.1 限速缩放 —— `enforce_limits.cpp`

```cpp
double getVelocityScalingFactor(group, velocity) {
  double f = 1.0;
  for (joint : active) {
    if (bounds.velocity_bounded_ && velocity(i) != 0.0) {
      bounded = clamp(velocity(i), min_velocity_, max_velocity_);
      f = min(f, bounded / velocity(i));      // ← 任一关节超限 → 全体一起缩
    }
  }
  return f;
}
```

`override_velocity_scaling_factor >= 0.01` 时**完全跳过**这个计算。

### 5.2 平滑滤波 —— `servo_calcs.cpp:749` + `butterworth_filter.cpp`

```cpp
// 每周期先把位置重置成 joint_topic 的值,再加 delta,然后滤波
joint_state.position[i] += delta_theta[i];
smoother_->doSmoothing(joint_state.position);

// 滤波器(coeff=1.5 时 scale=0.4, feedback=-0.5)
y = scale*(x_prev + x - feedback*y_prev);
```

关键在于输入依赖自己的输出(每周期以 joint_topic=上一次指令为基准),闭环后
每周期推进量不再是 delta,而是 `f(coeff)·delta`。coeff 越小平滑越弱(反馈项 = 1−coeff),
允许的最小值是 1.01。

## 6. 期间做的两处代码变更

1. **Servo 走雅可比路径**(`lgw_full.launch.py`):给 `servo_node` 传一份去掉 `arm`
   条目的 `robot_description_kinematics`。原因:IK 路径的 `searchPositionIK` 超时是
   **硬编码 `publish_period/2` = 5 ms**,且 `opts.return_approximate_solution = true`,
   存在静默返回部分解的风险。改后 Servo 打印
   `No kinematics solver instantiated for group 'arm'. Will use inverse Jacobian...`。
   **move_group 不受影响**(仍加载完整 kinematics.yaml)。
2. **`kinematics.yaml` 的 `solve_type: Speed` 已回滚** —— MoveIt 2.5.9 的 KDL 插件
   根本没有这个参数(源码 582 行内无 `solve_type`),当时那版改动是空操作。

## 6.5 稳态复测(2026-10-30 收尾)

目标做正弦扫掠 `x = x0 + 0.15·sin(2π·0.2·t)`,等价于平台以 ~0.19 m/s 来回移动:

| 时刻 | \|w\|cmd | qdot_exp | qdot_obs | ratio |
|---|---|---|---|---|
| 2 s | 0.796 | 2.34 | 0.35 | 0.15 |
| 10 s | 0.724 | 2.77 | 0.52 | 0.19 |
| 20 s | 0.687 | 2.63 | 0.47 | 0.18 |
| 30 s | 0.677 | 2.61 | 0.49 | 0.19 |

**稳态 ratio ≈ 0.18~0.19,比阶跃更低。** 这排除了"滤波器只衰减瞬态"的解释:
衰减是持续的,任何基于频率响应的机制(低通、平滑、积分)都无法解释。

⇒ 剩下的是一个**与频率无关的纯比例因子**,位于
`scaleCartesianCommand`(已确认是 `twist·publish_period`)之后的某处,
或者我们对 `J` / `twist` 的解读与实际送入 Servo 的不一致。

## 6.6 注入实验(绕过我们自己的节点,2026-10-30)

为了排除"我们发的 twist 与 Servo 收到的不是同一份",做了**直接注入**:`lgw_servo_node` 置于
IDLE(完全不发 twist),外部以 100 Hz 直接向 `/servo_node/delta_twist_cmds` 灌
`wz = -0.15 rad/s`,持续 1.5 s(命令转角 0.225 rad / 12.9°),再用 **FK 直接量末端实际转角**。

| 配置 | 末端实际转角 | 比值 |
|---|---|---|
| 默认(Butterworth 1.01 + mock 域) | **2.7°** | 0.21 |
| NoOp 平滑插件 | **3.0°** | 0.23 |
| `servo_joint_domain:=feedback`(真机域) | **1.6°** | 0.12 |

**结论**:衰减与我们的控制律无关(整条链路都没参与),与平滑滤波器无关,与状态源无关。
它稳定地发生在 Servo 内部 `twist → delta_theta` 这一段,且对所有已测参数不敏感。

## 6.7 publish_period 的 A/B(2026-10-30,关键)

把 `publish_period` 从 0.01(100 Hz)改到 0.034(30 Hz,MoveIt 官方示例值),其余不变:

| publish_period | 循环频率 | 末端实际转角 | ratio |
|---|---|---|---|
| 0.01 | 100 Hz | 3.0° | 0.23 |
| 0.034 | 30 Hz | 6.2° | 0.48 |

按 `ratio ≈ 1/(1+τ·f)` 反解,两组得到**同一个 τ ≈ 35 ms**(100 Hz→0.22 vs 实测 0.23;30 Hz→0.49 vs 实测 0.48)。
说明环里有一个约 35 ms 的固定滞后,按"每周期"吃掉固定比例的增量——**循环越快,单位时间被吃得越多**。
这解释了为什么官方示例(30 Hz)没有这个问题,而我们 100 Hz 慢了一半多。

### 真机自身延迟(阶跃实测)

```
control  开始动:t_cmd +  66 ms
feedback 开始动:t_cmd + 349 ms
>>> 真机跟随延迟 = 283 ms
```

**283 ms ≫ 35 ms**,所以这 35 ms 主要不在 CAN/臂内部,而在 Servo↔JTC↔状态回灌这段。
但这也说明:如果按官方架构把 JTC 直接连真机(状态源变成真机反馈,Servo 每拍锚定一个
滞后 283 ms 的状态),**速度会更差**——与 §6.5 的 feedback 域 A/B(0.12 vs 0.23)一致。
**结论:当前 mock 架构在速度上是正确选择,不建议改 ros2_control 硬件插件。**

### 最终配置下的复测(安全配置)

驱动改为 `speed_percent:=20 fast_mode:=false`(因 2026-10-30 末端相机支架损坏事故),
`publish_period=0.034` + Butterworth 1.01 + override 0.8 + mock 域:

| 量 | 值 |
|---|---|
| 指令末端转角 | 2.86° (wz=0.05 rad/s × 1.0 s) |
| 实际末端转角 | 1.54° |
| **ratio** | **0.54** |
| mock vs 真机关节差 | 0.001 rad(真机完全跟上) |

### 安全教训(2026-10-30)

1. **注入实验时 Servo 脱离了 `lgw_servo_node` 的全部保护**(无 w_max、无饱和保护、无跟随监控),
   只受 `override_velocity_scaling_factor=0.8`(允许到 4 rad/s 关节速度)约束;
2. **末端 3° 的转动,关节侧需要 ~0.3 rad**(joint6 +0.32、joint4 −0.29)——"末端小角度"不等于
   "关节小动作",伪逆会把转动分配到腕部大角度上;
3. 以后任何绕过 `lgw_servo_node` 的注入测试,都必须先确认末端附件(相机/支架)的活动空间,
   并把 `override_velocity_scaling_factor` 临时降到 0.2 以下。

## 7. 未解决 / 下一步

1. **`getVelocityScalingFactor` 为什么是 0.66**:模型里 6 个臂关节 `velocity` 都是 5,
   而 `qdot_exp` 只有 2.6,理论上不该缩放。需要确认 MoveIt 实际使用的
   `bounds.min_velocity_ / max_velocity_`,以及 `enforceVelocityLimits` 里
   `joint_state.velocity` 在平滑之后是否出现瞬态尖峰。
2. **平滑滤波的 0.58**:系数已到下限 1.01。彻底去掉需要自写一个直通平滑插件
   (MoveIt 只提供 Butterworth 一个实现)。
3. 稳态行为待验证:滤波器的直流增益是 1,短阶跃会被衰减,但**持续运动**下应逐渐
   接近 1。需要用"斜坡型目标"(而非阶跃)复测平台匀速行走场景。
4. **Phase C 平台速度前馈**仍未实现——对"平台移动追不上"这个真实场景,前馈的收益
   可能大于继续抠这 0.38。

## 8. 当前生效配置(2026-10-30)

| 参数 | 值 | 备注 |
|---|---|---|
| `override_velocity_scaling_factor` | 0.8 | 跳过内部缩放,同时保留 80% 限速保护 |
| `butterworth_filter_coeff` | 1.01 | 允许的最小值 |
| `lookat_mode` | legacy | 新模块仍保留但未启用 |
| `follow_tolerance_rad` / `follow_timeout_ms` | 0.45 / 5000 | 原 0.15/300 会把正常加速瞬态判成故障 |
| Servo 运动学 | 无 IK 求解器(雅可比路径) | 仅 servo_node |
