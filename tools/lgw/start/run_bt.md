# 起栈 + 运行行为树 —— 命令速查

> 适用：`track_collect_reactive.xml`（TreeID `TrackCollectReactive`）采集任务。
> 记录日期 2026-11-18。所有宿主机命令默认在 `cd /home/unitree/my_ws/l4t/lg_walker_ws` 下执行。

---

## 0. 链路分层（为什么是三条命令）

| 步骤 | 在哪 | 命令 | 起什么 |
|---|---|---|---|
| T1 | 宿主机 | `bash scripts/start_lgw_arm.sh --driver-only` | can0 + `agx_arm_ctrl_single`（**不含** MoveIt） |
| T2 | 宿主机 | `bash scripts/start_lgw_yoloe.sh --lo` | 宿主 Foxy 的 YOLOE 推理循环 + 容器内两路 realsense（含 USB 复位、等首帧） |
| T3 | 宿主机（内部 docker exec） | `bash scripts/start_lgw_chain.sh follow:=true use_bt:=true` | MoveIt + Servo + `lgw_servo_node` + `unified_track_node` + `bt_runner` + `body_pose_bridge`，并自动起 domain 7→0 的感知桥 |
| T4 | 宿主机或容器 | `bash scripts/run_bt.sh` | 发 goal 触发行为树 |

YOLOE 只能在**宿主机 Foxy** 上跑（容器没有 Jetpack r35.5.1 + py3.10 的 TensorRT），所以 T2 不在 `lgw_chain.launch.py` 里 —— 链路是 **3 条启动命令 + 1 条 goal**。

`--lo`：相机与 YOLOE 跑在 `lo` + domain 7（大帧不走 eth10），检测结果由 chain 自动起的桥搬回 domain 0。不加 `--lo` 则在 domain 0，此时 T3 要加 `PERCEPTION_BRIDGE=0` 前缀。

---

## 1. 起栈

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws

# T1：CAN + 机械臂驱动
bash scripts/start_lgw_arm.sh --driver-only

# T2：相机 + YOLOE（另一个终端）
bash scripts/start_lgw_yoloe.sh --lo

# T3：主栈（第三个终端）
bash scripts/start_lgw_chain.sh follow:=true use_bt:=true
```

就绪标志：

- T1：`.run/lgw_arm_driver_*.log` 出现 `Agx_arm feedback is ready, control is now enabled`
- T2：`.run/lgw_yoloe_current.log` 里 head / arm 都有 `CONTINUOUS_RESULT`（约 40–60 s）
- T3：日志里 `Loaded BehaviorTree: track_collect_reactive.xml` + `Starting Action Server: lgw_bt_execution`

---

## 2. 发 goal 之前必查（熟记这三条）

```bash
# (a) 感知桥在跑（lo 模式）
bash scripts/start_lgw_perception_bridge.sh --status

# (b) arm 相机结果真的在流（domain 0）—— 最关键
docker exec self_l4t bash -lc "source /my_ws/l4t/.l4t_env.sh && timeout 10 ros2 topic hz /perception/two_stage/camera_arm/result"

# (c) 底盘位姿（tracker 的 odom 靠它）+ 夹爪反馈（ControlGripper 首 tick 读不到就 FAIL）
docker exec self_l4t bash -lc "source /my_ws/l4t/.l4t_env.sh && timeout 5 ros2 topic hz /utlidar/robot_pose"
docker exec self_l4t bash -lc "source /my_ws/l4t/.l4t_env.sh && timeout 5 ros2 topic hz /feedback/gripper_status"
```

预期：桥 status 在跑；(b) **5–6 Hz**；(c) ~18 Hz 和 >0 Hz。

> (b) 为 0 就别发 goal —— 见 §5「arm 相机断流」。

---

## 3. 运行行为树

```bash
bash scripts/run_bt.sh TrackCollectReactive    # 树 ID 是默认值,可省略
# 等价容器内写法:
# docker exec -it self_l4t /my_ws/l4t/lg_walker_ws/scripts/run_bt.sh TrackCollectReactive
```

- 树是 `Repeat(-1)`，goal 一直 RUNNING，终端别关。
- **Ctrl-C 取消 goal**：日志会出现 `Received request to cancel goal` → `canceling and halting Behavior Tree`。

---

## 4. 任务行为（真机操作必读）

1. **初始化**：开爪(0.1 m) → 臂回 `home` → `StandUp` → `BalanceStand` → `SwitchGait(classic_walk)` → `ServoTakeover`。
2. **接近**：等选中目标到机械臂基座 XoY **≤ 0.5 m**（`WaitUntilTargetNear`）。此阶段需你遥控靠近；Servo 在 TRACKING，臂会跟着目标转。
3. **低头**：`SetBodyPose(pitch=0.5236)`（30°）——**这一步会发 `BalanceStand`，步态被打掉**。
   **⚠️ 低头期间不要用遥控器推底盘**：2026-11-18 就是这样在前倾 30°、无步态的状态下驱动导致摔倒。
4. **验证**：`WaitForArmHits` 3 次命中 / 5 s（arm 相机）。
5. **抓取**：3 次尝试。每次：`ServoTakeover` → 重新验证 → `ServoRelease` → 用当前 tcp 姿态重算位姿 → 预抓取(-100 mm 光轴) → 抓取 → `ControlGripper(max_force=1.0, require_force=true)` → 回预抓取 → 回 home。
   - 失败恢复：重开爪 → 臂回 home → 重试
6. **收尾**：
   - 成功：`SetBodyPose clear` → `SwitchGait(classic_walk)` → `MarkTrack(1)` 消费 → `ServoTakeover`
   - 三轮全失败：**先恢复姿态 + 重发步态**，再 `MarkTrack(2)` 标 ignored 转下一个
   - 相机死：只恢复姿态，**不** mark（不误判"目标不存在"）

---

## 5. 已知坑（都是 2026-11-18 真机踩出来的）

### arm 相机断流（本次任务"一直不采样"的根因）

- 现象：BT 每轮 `arrived` → 低头 → `[WaitForArmHits] fewer than 3 arm hits within 5000 ms` → 恢复姿态 → 重来，永远不抓。
- 真因：arm 那颗 D435 挂在 **USB 2.0**（内核 `usb 1-3.3`），原先配 `1280x720x10 + 848x480x10 ≈ 286 Mbps` 超过 USB 2.0 实用带宽 ~280 Mbps。跑 4.5 分钟后 realsense 报 `Frames didn't arrived within 5 seconds` 且**不会自愈**。
- 处置：`two_realsense.launch.py`（容器内 `/opt/yoloe_transfer/humble_ws/{src,install}/...`）arm 改 **`1280x720x6` + `848x480x6`**（≈172 Mbps）。head 在 USB 3.2 上保持 720p15。
- 复查：
  ```bash
  docker exec self_l4t bash -c "grep -a 'Open profile' /tmp/two_realsense_inside.log | tail -4"
  docker exec self_l4t bash -c "grep -ac 'Frames didn.t arrived' /tmp/two_realsense_inside.log"   # 应为 0
  ```
- 注意：`/opt/yoloe_transfer` **没有 bind mount**，容器重建会丢；`install/` 那份重新 `colcon build` 也会被 `src` 覆盖（两份都改过则安全）。

### `MoveArmToPose` 4 ms 静默 FAILURE

- 现象：`IDLE → RUNNING` 后 4 ms 直接 `FAILURE`，**没有任何 ERROR 行**，3 次抓取全空。
- 真因：`ComputeGraspPosesFromTrack` 用 `setOutput(动态key, ...)` 写 blackboard —— 该 API 只对**本节点声明过的 OutputPort** 有效，动态 key 静默不写 → 下游 `getInput<PoseStamped>("goal")` 拿不到 → `setGoal` 返回 false → `RosActionNode` 无日志 FAILURE。
- 已修：改直写 `config().blackboard->set()` 并读回校验，失败会打 `blackboard write failed`。

### 日志折行把命令劈成两半

- 现象：`usage: ros2 action send_goal ...` + `bash: line 2: btcpp_ros2_interfaces/action/ExecuteTree: No such file`。
- 真因：长命令被折行，双引号内的换行原样进容器 bash，当命令分隔符。
- 规避：用 `scripts/run_bt.sh`（调用侧恒为一行，且宿主机直接跑会自动 `docker exec`）。

### 分不清"打滑/超时"和"没到目标"

- `MoveGroup SUCCEEDED` 不代表真机到位（ros2_control 跑在 FakeSystem 上）；`MoveArmToPose` 已加 `max_pos_err_m=5 mm` / `max_rot_err_rad=0.05` 兜底判据。

---

## 6. 日志位置

| 内容 | 路径 |
|---|---|
| YOLOE + 推理流 | `.run/lgw_yoloe_current.log`（宿主） |
| 相机（realsense） | 容器内 `/tmp/two_realsense_inside.log` |
| arm 驱动 | `.run/lgw_arm_driver_<ts>.log`（宿主） |
| chain launch 输出 | `/tmp/lgw_chain_<ts>.log`（**仅** `start_lgw_chain.sh` 起时） |
| 行为树 / BT 节点 | 容器内 `/root/.ros/log/bt_runner_*.log`（**任何方式起栈都有**） |
| tracker | 容器内 `/root/.ros/log/python3_*_<ts>.log`（`unified_track_node started` 那条） |

需要逐 tick 轨迹时给 tracker 开 trace：

```bash
mkdir -p .run/trace
bash scripts/start_lgw_chain.sh follow:=true use_bt:=true \
  trace_log_dir:=/my_ws/l4t/lg_walker_ws/.run/trace trace_tag:=run1
```

---

## 7. 停栈顺序

1. **Ctrl-C 取消 goal**（跑了树才需要）
2. **T3 Ctrl-C**（脚本会顺带 stop 感知桥）
3. **T2 Ctrl-C**
4. **T1 Ctrl-C**

❌ 禁止 `pkill -f ros` / `killall` / `kill -9 -1`；要手工清理就先 `docker exec self_l4t ps aux` 列 PID，再逐个 `kill -15`。

---

## 8. 环境

一次性 `ros2` 命令一律用缓存（4 ms）而不是 `l4t_setup.sh`（7.3 s）：

```bash
docker exec self_l4t bash -lc "source /my_ws/l4t/.l4t_env.sh && <命令>"
```

**只有 `colcon build` 装了新包**才需要 `bash l4t/lg_walker_ws/scripts/lgw_env.sh --refresh`；重建已有包、容器重启都不需要。
