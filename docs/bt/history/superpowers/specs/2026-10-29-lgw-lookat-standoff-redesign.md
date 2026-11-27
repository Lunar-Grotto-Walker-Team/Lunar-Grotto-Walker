# lookat 站位重构:从"基座相对"改为"目标相对 + 显式约束"

- 日期:2026-10-29
- 状态:设计草案(待评审,未实施)
- 作者:Claude Code(与用户共同评审)
- 上游基线:`2026-10-26-lgw-servo-takeover-design.md` §7.5
- 相关实测工具:`scripts/lookat_reachability.py`、`scripts/record_platform_motion.py`

---

## 0. 摘要

当前 lookat 的相机站位是**相对机械臂基座**定义的:

```
P_ee = ( xoy · t̂_xy ,  z_fixed )        t̂_xy = (t.x, t.y) / |t.xy|
```

本文把它改成**相对目标**定义,并把"可达"和"防撞"从隐式副作用变成**显式约束**:

```
P_cam = p_target − d · v̂                d 在 [d_min, d_max] 内自适应
       约束: r_min ≤ |P_cam| ≤ r_max     (臂展)
             z_cam ≥ z_min_cam            (防撞/防压低)
       不可行 → HOLD,绝不发非法指令
朝向仍为严格的 look-at(+ 可选光学轴 roll)
```

---

## 1. 问题与证据(2026-10-28/29 真机实测)

### 1.1 基座相对站位的五个后果

| # | 后果 | 证据 |
|---|---|---|
| 1 | **位置通道基本不干活** | 平台移动 0.78 m 期间 `position_error_m` 最大仅 **0.0305 m**;因为 `P_ee` 只跟随目标*方向*,不跟随目标平移 |
| 2 | 平台平移被翻译成纯角度误差 | 同一段 `angular_error_rad` 最大 **0.2815 rad**;位移通道(上限更大)闲置 |
| 3 | **`xoy` 是隐形的可达门槛** | 目标 xy 距离 0.30 m + `xoy=0.40` → **IK 无解**(相机被摆到目标之后) |
| 4 | **`z_fixed` 直接是奇异旋钮** | 目标 0.75 m 前方、`xoy=0.35` 时:z=0.20→cond 24、**z=0.25→cond 88**、z=0.35→cond 29。存在一条奇异脊 |
| 5 | 管不住重心 | `z_fixed` 只钉相机点;上一轮 tcp 高 **+23.6 cm**、前伸 **45.6 cm**,平台行走重心不稳 |

### 1.2 已解决的部分(不在本文范围)

- 腕部奇异可通过**绕光学轴的 roll** 规避(已实现 `camera_roll_rad`,30°),光轴偏差实测 0.000000°;
- 控制律的 `odom_scale` 0.5→1.0、`kp_ang` 3→5 已落地。

### 1.3 硬几何边界(实测)

- 满限位内 tcp 最大可达半径:**0.871 m**;
- tcp 可贴近基座的最小距离:0.017 m。

---

## 2. 目标 / 非目标

### 2.1 目标

1. 相机站位改为目标相对,站距 `d` 自适应;
2. 可达性与最低高度作为**显式、每周期可判定**的约束;
3. 位置误差重新具有控制意义(平台移动时 `e_pos` 必须显著增大并驱动臂平移);
4. 保留现有 look-at 朝向语义与 `camera_roll_rad`;
5. 不可达时明确 HOLD 并上报原因,而不是发坏指令。

### 2.2 非目标

- 不改 MoveIt Servo 的 IK/奇异/碰撞处理;
- 不改 BT 侧 lookat 节点(它们不经过 `compute_lookat_setpoint`);
- 不在本阶段做图像伺服(IBVS);
- 不在本阶段做关节空间姿态优化(见 §6 Phase B)。

---

## 3. 设计

### 3.1 三个距离,不可能同时钉死

| 符号 | 含义 | 归谁管 |
|---|---|---|
| **R** = \|p_target − P_base\| | 基座↔目标 | 平台运动,不可控 |
| **r** = \|P_cam − P_base\| | 基座↔相机(臂展) | **约束到 [r_min, r_max]** |
| **d** = \|P_cam − p_target\| | 相机↔目标(站距) | **在 [d_min, d_max] 内自适应** |

相机位于基座→目标连线延长线上时 `r = R − d`,所以两者互相牵制。现行方案钉 `r`(`xoy`),`d` 被动浮动;新方案钉 `r` 的**舒适值**(通过选 `d` 实现),`d` 浮动但有界。

### 3.2 算法(每控制周期)

**Step 1 — 视线方向 `v̂`(目标 → 相机)**

```
v̂_default = normalize(P_cam_now − p_target)      // 保持当前观察方位,连续不跳变
若 |P_cam_now − p_target| < 1e-3 → 用配置的 fallback 方向(默认从上前方俯视)
```

用当前方位而非"基座→目标"方向,是为了避免平台横移时相机绕目标甩动。

**Step 2 — 站距 `d` 的选择(精确解,不用近似)**

令 `Δ = p_target − P_base`,要让 `r = r_pref`:

```
|Δ − d·v̂|² = r_pref²
⇒ d² − 2(Δ·v̂)·d + (|Δ|² − r_pref²) = 0
⇒ d = (Δ·v̂) − sqrt( (Δ·v̂)² − |Δ|² + r_pref² )
```

取减号根 → 相机落在基座与目标之间(自然的选择)。判别式 `< 0` 表示该方位下无法把臂展压到 `r_pref`(目标太偏),此时退化为 `d = clamp(Δ·v̂, d_min, d_max)`。

**Step 3 — 相机位置**

```
P_cam = p_target − d · v̂
```

**Step 4 — 约束投影(保证可行)**

按固定顺序投影,每次投影后重新检查:

```
1) 臂展: 若 |P_cam| > r_max → P_cam 沿基座方向拉回到 r_max
          若 |P_cam| < r_min → 推出到 r_min
2) 高度: P_cam.z < z_min_cam → 抬高到 z_min_cam
3) 重新检查 |P_cam| ≤ r_max(抬高可能又超了)
若仍不可行 → 本周期视为不可达
```

投影会让实际站距偏离 `d`,`ServoState` 需回报 `achieved_standoff_m` 以便观测。

**Step 5 — 朝向**

```
direction = normalize(p_target − P_cam)          // 严格对准(投影后依然严格)
R_cam     = uprightness_flip(build_lookat_orientation(direction, +z)) · Rz(roll)
```

**Step 6 — 手眼 → tcp**:沿用现有 `compute_tcp_pose(P_cam, R_cam, R_cam_in_tcp, t_cam_in_tcp)`。

**Step 7 — 不可达**:见 §5。

### 3.3 新增/变更参数

| 参数 | 建议初值 | 说明 |
|---|---|---|
| `reach_pref_m` (r_pref) | 0.45 | 期望臂展 |
| `reach_min_m` | 0.15 | 臂展下限 |
| `reach_max_m` | 0.75 | 臂展上限(实测极限 0.871,留 0.12 余量)|
| `cam_min_height_m` (z_min_cam) | 0.12 | 相机相对基座的最低高度 |
| `standoff_min_m` / `standoff_max_m` | 0.25 / 0.90 | 站距允许区间(无"首选站距":站距由 `reach_pref_m` 导出) |
| `standoff_rate_limit_mps` | 0.30 | `d` 变化率限幅 |
| `camera_roll_rad` | 0.5236 (30°) | 不变 |
| ~~`xoy_distance_m`~~ | — | **废弃**(由 r_pref + d 选择取代)|
| ~~`z_fixed_m`~~ | — | **废弃**(由 z_min_cam 的高度下限取代)|

废弃参数保留声明但忽略,并打印一条 WARN,避免旧 YAML 静默失效。

### 3.4 坐标系与符号约定

- 全部在 `arm1/base_link` 系;`+z` = 世界"上"(与 `build_lookat_orientation` 的 `world_up` 一致);
- `p_target` 由几何源层给出(odom 层为 `T_base_odom⁻¹ · p_odom`);
- `P_cam_now` = `tcp_now.p + tcp_now.R · t_cam_in_tcp`(现有实现);
- `v̂` 由**目标指向相机**,即相机看目标的方向的反向。

### 3.5 连续性

`d` 经一阶滤波 + 变化率限幅;`v̂` 用当前方位天然连续。当平台运动导致 `d` 触到上下限时,输出应平滑贴边而不是跳变。

---

## 4. 控制律侧(独立、可选)

跟踪滞后的主因是"纯 P 环 + 世界固定目标 + 移动基座"。可加前馈:

```
v_ff = −( v_platform + ω_platform × r_tcp )     在 base_link 系
w_ff = −ω_platform
```

由 `/utlidar/robot_pose` 的时间差分得到平台速度。与本文的站位重构**互不依赖**,可单独评审。

---

## 5. 不可达与降级行为

| 情形 | 行为 |
|---|---|
| 投影后仍违反臂展 **或** 高度约束 | `TARGET_LOST` → HOLD,`ServoState.state=HOLD`,新增 `reason=unreachable` |
| 判别式为负且 `d` 越界 | 同上 |
| 目标在基座 z 轴上(方位退化)| 用 fallback `v̂` |
| 目标与相机重合 | HOLD(沿用现有异常路径)|

新增 `ServoState` 字段:`achieved_standoff_m`、`reach_m`、`reason`(空 = 正常)。

---

## 6. 分阶段

| 阶段 | 内容 | 风险 |
|---|---|---|
| **A**(本文) | 目标相对站位 + 约束投影 + HOLD + 参数化 | 中:改变臂的跟踪行为 |
| B | 关节空间姿态优化(cond + 关节余量 + 质心),在 takeover 前离线选 `d`/`roll` | 低:只影响初始姿态 |
| C | 平台速度前馈(§4) | 中:引入速度源与符号约定 |

---

## 7. 验证计划

**单测(`test_lookat_setpoint`)**

1. `d = d_pref` 时 `|P_cam − p_target|` 精确等于 `d_pref`;
2. 目标很远时 `|P_cam|` 被夹到 `r_max`,且 `achieved_standoff > d_max`;
3. 目标很低时 `P_cam.z == z_min_cam`;
4. 投影后光轴仍严格对准目标(偏差 < 1e-12);
5. 不可行输入(目标在基座内、`r_min > r_max` 等)抛出/返回不可达而非 NaN;
6. `roll` 行为与现有 2 个用例保持。

**真机静态**

- takeover 后 `converged=true`,`cond(J) < 40`,`/servo_node/status` 30 s 内无 DECELERATE/HALT。

**平台移动(核心验收)**

- 世界固定目标,平台前后移动 ≥ 0.3 m;
- **`position_error_m` 必须显著增大(> 0.05 m)**,证明位移通道在工作(现在的 0.03 m 上限正是要修掉的现象);
- 停止后 2 s 内重新 `converged`;
- 全程 `geometry_source` 保持 `odom`,`/control` 与 `/feedback` 关节差 < 0.15 rad。

**边界**

- 令 R 逼近 `r_pref + d_max` 与 `r_pref + d_min` 两端:系统应要么正常跟踪,要么干净地 HOLD,禁止出现非法/奇异指令。

**回归**

- `test_lookat_setpoint`、`test_servo_control_law`、`test_servo_state_machine`、`test_geometry_source`、`test_target_selector` 全绿。

---

## 8. 风险与回滚

| 风险 | 缓解 |
|---|---|
| 站距变化导致目标像面尺度变化 | 下游用 3D 位置,不依赖尺度;`d` 变化率限幅 |
| 投影使实际站距偏离期望 | 回报 `achieved_standoff_m`,超限时告警 |
| 平台运动中 `d` 频繁贴边 → 抖动 | 一阶滤波 + 迟滞(贴边 2 s 才判不可达)|
| 改动影响 BT 路径 | `compute_lookat_setpoint` 仅 Servo 使用;BT 另有实现 |

回滚:参数 `lookat_mode: legacy|target_relative`,默认 `legacy` 直到验收通过。

---

## 9. 已决策项(2026-10-29 用户确认)

1. `r_pref / r_max` = 0.45 / 0.75;
2. `cam_min_height_m` = **0.22**(相机相对臂基座的最低高度);
3. **不引入前馈**(Phase C 暂缓);
4. 不可达时 **停在最后可行点、继续按当前目标对准**(`StandoffStatus::kHeldLast`);
5. `standoff_max_m` = **0.90**:站距上限决定远处目标的可达包线,且必须满足
   `d ≥ cam_min_height_m − p_target.z`;0.65 在站立/地面目标下过紧;
6. 实现放在**独立文件** `lookat_standoff_setpoint.{hpp,cpp}`,由
   `lookat_mode: legacy|target_relative` 切换,`compute_lookat_setpoint` 未改动。

### 9.1 由决策 2 推出的操作包线

相机高度 = `p_target.z + d·sin(仰角) ≥ 0.22`,因此 `d ≥ 0.22 − p_target.z`。
配合 `standoff_max_m = 0.90`,目标相对臂基座的最低高度约为 **−0.68 m**;
再低则约束集为空,会持续走 hold-last。

## 10. 遗留待定项

1. `d` 的滤波时间常数(当前只有变化率上限 0.30 m/s,无低通);
2. Phase B 关节空间姿态优化(cond / 关节余量 / 质心);
3. `ServoState` 增补 `achieved_standoff_m / reach_m / reason` 字段(当前仅日志)。
