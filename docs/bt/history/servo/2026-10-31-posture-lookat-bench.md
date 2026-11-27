# 固定姿态族 lookat — mock 链路实测(2026-10-31)

- 方案:`docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md`(rev2)
- 计划:`docs/superpowers/plans/2026-10-31-lgw-pinned-posture-lookat.md`
- 脚本:`scripts/bench_posture_lookat.sh`、`scripts/measure_posture_bench.py`、
  `scripts/bench_cleanup.sh`
- **不驱动真机**:全程 `follow:=false`(rsp/Servo/move_group 都跑 `/control/joint_states`),
  且 bench 启动前强制检查 `agx_arm_ctrl_single` 进程数为 0。

## 1. 工况

```
目标:          base 系 (0.75, 0, -0.37) m,即站立时地面 0.75 m 前方
目标运动:      x 方向正弦扫掠 ±0.15 m @ 0.2 Hz(等效平台速度 ~0.19 m/s)
时长:          50 s
姿态参数:      j3 = -0.484、j5 = 0.600、j2 ∈ [0.10, 1.571]
基准姿态:      预先用 MoveGroup 把 mock 臂摆到族内解
               q = [0.56, 0.96, -0.48, -1.12, 0.60, 1.38]
```

## 2. 结果

| 指标 | 实测 | 阈值 | 判定 |
|---|---|---|---|
| 状态采样 | **500 / 500 TRACKING** | 无 HOLD | ✅ |
| `servo_node/status ≠ 0` 累计 | **0.00 s** | 0 s | ✅ |
| 解算耗时(均值) | **0.10–0.13 ms** | — | ✅ |
| 解算耗时(最坏) | **0.73–1.06 ms** | p99 < 3 ms | ✅ |
| e_ang | 均值 **0.049**、p95 0.077、max 0.082 rad | 均值 < 0.05 | ✅ |
| e_pos | 均值 **0.015**、max 0.025 m | — | ✅ |
| 相机高度 | **0.302 m** | ≥ 0.20 | ✅ |
| 相机半径 | **0.228 m** | ≤ 0.35 | ✅ |
| FK(tcp) vs TF `arm1/tcp_link` | **0.72 mm** | < 2 mm | ✅ |
| max \|qdot\|(mock) | 1.46 rad/s | — | 参考值 |

解算耗时按每 500 拍打一条日志统计(10 条窗口全部 ≤ 1.06 ms),占 100 Hz 周期
(10 ms)的 1–10%,远低于设计预算。

## 3. 与设计预期的偏差

| 预期(spec) | 实测 | 处理 |
|---|---|---|
| 地面 0.40–1.20 m 精确对准 | 单元测试残差 < 1°,bench 全程 e_ang ≤ 0.082 rad | 一致 |
| 方位覆盖 ±150° | **-150° ~ +122°(不对称)** | 相机光轴相对臂 +x 有 ~78° 固定方位偏置,j1 先在下限一侧撞限位;spec 已按实测修正 |
| 墙面目标 6.4° 偏心 | 单元测试判为 `out_of_envelope` | spec §5 已按含余量的实测修正 |

## 4. 过程中发现并修掉的实现缺陷

1. **j1 初值** — 以 j1 = 0 为种子时,DLS 单次行程上限 8×0.2 = 1.6 rad,横向目标
   (相机方位偏置 ~78°)根本走不到,表现为"±60° 以外全部无解"。改为先做一次 FK
   测出光轴方位,再用 `j1 = 目标方位 − 光轴方位` 作种子。这是连续性测试逼出来的。
2. **DLS 局部极小** — 单种子在 j4 = 0 时会锁到奇异的另一侧(残差 7.7°),增加两个
   j4 备选起点。
3. **坐标链冲突** — `base_frame_`/`tcp_frame_` 带 `arm1/` 前缀,而 URDF 里没有;
   直接用会让 `initialize()` 失败、节点进 FAULT。
4. **测量脚本双重 tcp 偏置** — `lookat_reachability.fk()` 已含 0.12 m tcp 偏置,
   脚本又加了一次,首轮因此报出 120 mm 的"帧不一致"假警报。

## 5. bench 环境注意

- mock 栈没有机器人本体,`odom`/`world` 帧需手动补:
  `world → odom → arm1/base_link`(**必须一条链**;给 `arm1/base_link` 挂两个
  parent 会让 tf2 lookup 随机失败,首轮 takeover 报 `no_base_pose` 就是这个原因)。
- mock 臂启动时在 q = 0(奇异位),Servo 第一拍就会 `Very close to a singularity,
  emergency stop`;必须先 MoveGroup 摆到族内姿态再 takeover。
- 清理必须用 `scripts/bench_cleanup.sh`(独立文件):在内联 `bash -c` 里用
  `pgrep -f` 杀进程会匹配到自己的命令行,杀掉自己后整栈残留。

## 6. 未验证(留到真机阶段)

- 过渡段:真臂从任意姿态进入姿态族的第一段(realtime 与奇异行为)。
- 离线碰撞:新族相机半径 0.11–0.33 m,需在含 Go2 机身/头相机/夹爪的 planning
  scene 下验证。
- 实际跟踪速度:Servo 侧 ~0.5x 衰减(见 2026-10-30 那篇)在 mock 下不存在,
  真机速度会低于本表的等效值。
