# 2026-11-16 感知链路搬 lo / 跨域桥 / 伺服修复记录

本文记录一天内完成的三块工作：把 cameras + YOLOE 搬到 `lo` 上的独立 domain、
用一个 domain bridge 把检测结果送回主栈、以及随之暴露并修掉的几个伺服问题。
最后是检测/跟踪参数的调整与遗留问题。

---

## 1. 为什么要搬到 lo

原来的问题是：arm 相机 1280×720 走 eth10（CycloneDDS 默认 `FragmentSize=1344B`）
时，一帧要切成 ~2060 个分片，跑几分钟就退化——`color_age` 涨到 27 秒，处理速率
从 8.5 Hz 掉到 0.7 Hz。

换成 `lo`（MTU 65536 + `FragmentSize=60000B`）之后，同样条件下：

| 配置 | 结果 |
|---|---|
| eth10, 1280×720 RGB8 单流 ~41 MB/s | 整条流死掉 |
| lo, 两路 1280×720@6 合计 ~40 MB/s | 10 分钟恒定 5.98 Hz（arm），跳帧 6 次 |
| lo, 两路全开（head 15 fps + arm 10 fps）合计 ~82 MB/s | `stale ≈ 0`，无积压 |

结论：lo 把传输天花板抬到至少两倍，且 `color_age` 零告警。

## 2. 切换方式

```bash
bash scripts/start_lgw_yoloe.sh --lo          # cameras + YOLOE 走 lo + domain 7
bash scripts/start_lgw_yoloe.sh --lo-domain=9 # 自定 domain(须与桥的 from_domain 一致)
bash scripts/start_lgw_yoloe.sh               # 不传 = 原来的 domain 0 + eth10
```

配置文件：`config/dds_local_lo.xml`（lo-only、`FragmentSize 60000B`、
`MaxMessageSize 65500B`、socket 缓冲 `max 16MB`）。

**两个运行时前提**（重启会丢，脚本每次启动自动设）：

1. `ip link set lo multicast on` —— lo 默认**没有** MULTICAST 标志。不开的话
   CycloneDDS 退化到随机端口（40103/40531 而非 domain 7 的 9150/9151），
   单播发现对不上。
2. `sysctl -w net.core.wmem_max=67108864` —— 配置里的 `SocketSendBufferSize`
   的 max 不能超过它（默认仅 208 KB）。

**XML 里 socket 缓冲只写 `max`、绝不写 `min`**：写了 `min="1MB"` 而 wmem_max
没被抬高时，CycloneDDS 会因达不到下限返回 `DDS_RETCODE_NOT_ENOUGH_SPACE`，
participant 直接创建失败、进程 abort。

脚本内部还要注意：给容器的 `docker exec` 必须**同时透传 `ROS_DOMAIN_ID` 和
`RMW_IMPLEMENTATION`**（少一个就变成 domain 0 + 默认 fastrtps，找不到节点），
且所有 `ros2 node list` 探测都要加 `--no-daemon`（残留 daemon 会用旧配置回答）。

## 3. 跨域桥（domain 7 → domain 0）

主栈（MoveIt / Servo / tracker / BT）仍在 domain 0 / eth10，需要把结果搬回来。

| 文件 | 作用 |
|---|---|
| `config/perception_bridge.xml` | 双 domain 的 CycloneDDS 配置：domain 0 走 eth10，domain 7 走 lo |
| `config/perception_bridge.yaml` | 桥接清单（4 条话题） |
| `scripts/start_lgw_perception_bridge.sh` | 启停脚本，`--stop` / `--status` |

桥的是：`camera_head/result`、`camera_arm/result`、`state`、**`/tf_static`**。
图像和点云一律不桥（那是 lo 存在的意义）。

### 3.1 `/tf_static` 必须桥（最隐蔽的一条）

tracker 的 `broadcast_camera_mount_tf()` 要先 lookup
`camera_X_link → camera_X_color_optical_frame`，而这是 **realsense 自己发的
内部静态 TF**。相机搬到 domain 7 后 domain 0 查不到 → 相机挂载 TF 永远发不出来
→ `adapt()` 返回空 → **`tracks: []`，但检测话题完全正常**。

症状极具迷惑性：topic 有 8.7 Hz 数据、tracker 也在收帧，就是一个 track 都不出。

桥 `/tf_static` 时 QoS 必须写 `durability: transient_local`（+ `depth: 100`），
否则 latch 丢失、晚启动的订阅者收不到。

### 3.2 验证

```bash
# 普通环境(domain 0)
ros2 topic hz /perception/two_stage/camera_arm/result      # ~8 Hz
ros2 run tf2_ros tf2_echo odom camera_head_color_optical_frame
ros2 topic list | grep -E "image_raw|overlay"              # 应为空(没误桥)
```

## 4. 伺服修复

### 4.1 根因：`verify_servo_seed` 在跨域比较（最重要）

`ServoNode::verify_servo_seed`（`src/servo/servo_recovery.cpp`）拿 **Servo 内部
指令缓存** 和参考位姿比。参考原来取 `measured_positions_`（实测域
`/feedback/joint_states`），而 Servo 的 `moveit_servo.joint_topic` 是
**`/control/joint_states`（指令域）**。

`follow:=true` 时真机臂本来就滞后指令 **0.1–0.56 rad**（实测
`gap=0.459 / 0.564 rad`，跟随监控自身门限就是 0.450）。这段正常滞后被记成
`re-seed did not take (probe pose mismatch)`，一次就打成 FAULT —— 表现为
"扫描交接 / HOLD→TRACKING 恢复随机 FAULT，有时又能成功"（lag 小的那几次）。

**修法**：参考改成指令域，与 recovery 后置校验同一条规则：

```cpp
const std::array<double, kArmJointCount> & reference =
  commanded_valid_ ? commanded_positions_ : expected;
```

验证：改完后在 `gap=0.463 / 0.564 rad` 下连续两次扫描交接全部成功、零 FAULT。

> 这是根因修复，不是放宽阈值。前面把容差 0.10→0.15 只是把偶发变少。

### 4.2 扫描交接：先等臂静止 + 失败重试一次

`src/servo/servo_scan_worker.cpp` 的 `finish_scan_found()`：

- 播种前等臂基本静止（所有关节 |v| < 0.05 rad/s，上限 **150 ms**）
- 播种 + 校验失败重试一次，仍失败才 FAULT

等待上限 150 ms 是因为跨域 bug 修好之后臂动不动已不影响判定，这里只留兜底，
避免拖慢交接。

### 4.3 恢复路径：非阻塞延后 + 有限重试

`src/servo/servo_session.cpp` 的 `resume()`：

- 臂还明显在动（相邻两次测量差 > 0.003 rad）就先跳过，下个 tick 再试，不消耗预算
- 容差 0.10 → **0.15**
- 连续失败 **10 次**才 FAULT；成功后预算清零；每次新 HOLD 重置预算

### 4.4 关节限位硬停

现象：MoveIt Servo 反复打印
`joint4/joint5 close to a position limit. Halting.`，一次运行 16 次。

原因：QP 侧余量够，但真机跟随滞后 ~0.2–0.3 rad 把实际关节推过 Servo 的
halt 线（限位 − `joint_limit_margin`）。直读数值：

| 关节 | URDF 限位 | Servo 硬停线 | 滞后余量 |
|---|---|---|---|
| j4 | ±1.745 | ±1.645 | 0.10 rad |
| j5 | ±1.222 | ±1.122 | 0.10 rad |

调整：

| 参数 | 值 | 说明 |
|---|---|---|
| `moveit_servo.joint_limit_margin` | 0.10 → **0.05** | Servo 否决线往限位方向挪 |
| `qp_joint_margin_rad` | 0.20 → 0.30 → **回到 0.20** | 域 bug 修好后不需要再收紧，姿态解可达范围恢复 |

注意：5 分钟长测里仍有 8 次限位硬停（joint5 ×6、joint4 ×2），恢复规划每次都
要求手腕走 1.2–1.67 rad、落点余量只剩 **0.059–0.144 rad** —— 姿态解本身贴着
限位，这是**尚未解决**的问题。

### 4.5 扫描进入耗时

实测 `SCANNING → circle published` 原来是 **3.8–4.7 s**，拆开看：

| 阶段 | 原因 | 耗时 |
|---|---|---|
| 等平台静止 | `scan_require_base_static: true` | **开车时无限等** |
| 启动延时 | `scan_start_delay_ms: 500` | 0.5–0.7 s |
| 预稳定 | 复用 `recovery_presettle_gap_rad: 0.005`（物理不可达，正常跟随差 0.1–0.5 rad） | **吃满 3 s 超时** |
| 入位 MoveGroup | `scan_velocity_scaling: 0.15` | 0.5–1 s |

改动后降到 **1.2–2.0 s**：

| 参数 | 改前 | 改后 |
|---|---|---|
| `scan_require_base_static` | true | **false** |
| `scan_presettle_gap_rad`（新增，独立于 recovery） | — | **0.15** |
| `scan_velocity_scaling` / `accel` | 0.15 / 0.2 | **1.0 / 1.0** |
| 预稳定实测 | 3 s | **0.28 s** |

> MoveGroup 的"全速"是 `joint_limits.yaml` 的 **5.0 rad/s / 5.0 rad/s²**（纸面
> 值），真机实测上限只有 **~1.2 rad/s**。实测缩放到 1.0 并没有导致 JTC 中止
> （控制器自己限速了），入位反而更快。

### 4.6 扫描周期与启动宽容期

| 参数 | 改前 | 改后 | 理由 |
|---|---|---|---|
| `scan_period_s` | 8.0 | **10.0** | 一圈更慢更稳 |
| `scan_start_delay_ms` | 500 | **1500** | 5 分钟里 17 次 HOLD 有 11 次是"目标闪一下就丢"，扫描又都在 1–2 s 内找回；1.5 s 能过滤瞬时丢帧 |

### 4.7 距离筛选

新增 `target_max_distance_m`（远场门限），与 `target_min_distance_m` 组成
`[0.3, 1.5] m` 的服务区间，`<= 0` 表示不限。

**后来根据实测关闭了远场限制**：1.5 m 会把本来能观察到的目标挡掉。
现在 `target_min_distance_m: 0.30`、`target_max_distance_m: 0.0`（不限）。

顺带修了一个隐藏问题：每 tick 的重选用的 `selector_params_` 每次
`clear_session_locked()` 都重置成**结构体默认值**，改 yaml 只影响 takeover。
现在从参数读。

## 5. 检测/跟踪参数调整

| 参数 | 位置 | 改前 | 改后 | 理由 |
|---|---|---|---|---|
| 相机彩色 | realsense launch | 640×480@15 | **1280×720**（head 15 / arm 10） | 模型 imgsz=1024，"1280 是够用上限" |
| 相机深度 | 同上 | 640×480@15 | **848×480**（head 15 / arm 10） | D435 深度精度甜点；arm 的 848 只有 6/8/10 fps |
| `class_thresholds` / 两阶段阈值 | `~/yoloe_deploy/config/` | 0.30 | **0.25** | 远距离召回率不足 |
| `tracker_stream_timeout_ms` | servo_node.yaml | 500 | **1200** | 容忍检测间隙，别跟着检测节奏抖 |
| tracker `visible_timeout_ms` | lgw_3d_tracker runtime.yaml | 5000 | **8000** | 同上 |

**关键认识：相机帧率 ≠ 推理速率。**

| | 相机实际 | YOLOE 处理 |
|---|---|---|
| head 1280×720@15 | **15.0 fps** | ~5.6 帧/秒 |
| arm 1280×720@10 | 10.0 fps | ~5.6 帧/秒 |

YOLOE 双相机总处理上限约 **11–12 帧/秒**，与传输无关（`stale≈0`）。arm 相机
受 USB 2.0 限制，1280 下最多 ~9–10 fps；深度降到 10 Hz 后彩色从 9.4 涨到 14
（USB 等时调度被让出来了）。

## 6. 实测数据汇总

| 场景 | 结果 |
|---|---|
| lo 传输 | 两路 1280 合计 ~82 MB/s，`stale≈0`，10 分钟不退化 |
| 扫描进入 | **1.2–2.0 s**（原 3.8–4.7 s） |
| 扫描交接 | 跟随差 0.46–0.56 rad 下仍成功；域 bug 前必 FAULT |
| 5 分钟跟踪 | FAULT 0、播种失败 0；但 17 次 HOLD（11 次目标丢失 + 5 次限位硬停），11 次扫描全部 1–2 s 找到目标 |
| TRACKING 时精度 | 角度误差 0.7°、位置误差 1 cm |
| 单元测试 | **347/347 通过**（新增 5 个用例） |

## 7. 遗留问题

1. **j5/j4 限位硬停未除**：恢复规划把手腕安排到距限位 0.06–0.14 rad 的位置，
   臂一追就 halt。方向：`qp_j5_branch_max_rad` 0.95 → 0.85，或调 posture 参考值。
2. **远距离检测召回率低**：1.65 m 处石块 12 秒只有 10 帧检出（~9%），呈
   "两三帧一簇 + 空 1.5–2 s" 形态。已降阈值到 0.25、放宽新鲜度/超时，效果待测。
3. **track ID 翻腾**：同一物体短时间内出现多个 track id，影响目标选择稳定性。
   建议打开 tracker 的 trace 定位。
4. **导航/探索未迁移**：nav/SLAM 仍在 domain 0/eth10。评估结论是稳态流量远低于
   相机的 41 MB/s 风险线；只有 `/aft_pgo_map`、`/global_pgo_map`（几十 MB、1 Hz、
   RELIABLE）需要单独关注，探索用的 RViz 配置本就**不订阅**这两条。
5. **`go2_driver` 的 9 个 service** 无法通过 domain_bridge 桥接；目前无人调用，
   将来要遥操作就得自研 service relay。

## 8. 相关脚本速查

```bash
# 感知(lo / domain 7)
bash scripts/start_lgw_yoloe.sh --lo
# 结果跨域桥
bash scripts/start_lgw_perception_bridge.sh [--stop|--status]
# 主栈(会自动起桥；PERCEPTION_BRIDGE=0 关闭)
bash scripts/start_lgw_chain.sh
# 机械臂驱动
bash scripts/start_lgw_arm.sh --driver-only
```
