# 检测 → 3D 跟踪链路：信息流动与计算过程

- 日期：2026-11-13
- 范围：RealSense 相机输入 → YOLOE 推理 → `lgw_3d_tracker` 3D 跟踪输出
- 代码基线：`yoloe_deploy`（2026-11-06 并行推理默认开启）+
  `lgw_walker_ws`（tracker 2026-10-27，servo 2026-11-12 重构后）
- 目的：把"每一帧数据长什么样、在哪个进程/函数里被算什么、在哪一步会丢"
  固定下来，作为调试和后续接 servo/BT 的共同参照。

---

> **2026-11-13 更新：目标注册表 + Servo 搜索跟踪**
>
> §1–§5 描述的是**变更前**的链路（`status` / `contributor_*` / servo tier）。
> 从 2026-11-13 起：
>
> - tracker 统一状态机：`candidate / active / lost / consumed / archived`；
>   只发布 `task_state == "active"` 的目标；丢失目标保留在注册表里供 re-ID。
> - tracker 的 odom KF **每收到一条 `ObservationResult` 就更新一次**，不再
>   30 Hz 节流；发布是"观测驱动 + `publish_heartbeat_hz=25` 心跳"。
> - 发布字段：`position`（odom KF）、`position_arm_base`（**当前时刻**臂基座
>   投影）、`geometry_valid`（能否驱动）、`source`（arm/head/odom）、
>   `last_observation_stamp`。`contributor_*` 已移除。
> - servo **不再做** head/arm/odom tier 选择、`max_age`、`*_scale`：直接消费
>   `position_arm_base`；auto 模式选最近的 active（或指定 id）。
> - 发布列表为空时，servo 进入 `SCANNING`：j1 小摆 ±15°、j2/j3 固定观测姿态、
>   j4/j5 各 ±60° 走 Lissajous 花瓣（12 s 一朵），j6 实时抵消画面滚转；一条
>   轨迹 60 s 直发 JTC，扫完自动续发。发现 active 目标后先定姿停止、再
>   unpause Servo 回 TRACKING。详见 spec §10.5。
> - 任务层用 `/lgw/tracker/mark_track` 标记 consumed/ignored。
>
> 设计：`docs/superpowers/specs/2026-11-13-target-registry-and-servo-scan-design.md`
> 计划：`docs/superpowers/plans/2026-11-13-target-registry-and-servo-scan.md`

---

## 0. 总览

```
D435i(head) / D435(arm)                        [容器 self_l4t]
  color 640x480@15 + aligned_depth + camera_info
        │
        ▼
yoloe_perception/dual_camera_node              [宿主 Foxy]
  配对 → 解码 → YOLOE engine 推理 → 2D 后处理 → mask 反投影
        │  ObservationResult(center_m，相机光学系)
        │  /perception/two_stage/camera_{head,arm}/result
        ▼
lgw_3d_tracker/unified_track_node              [容器 Humble]
  TF 适配 → KF 预测 → 两阶段关联 → KF 更新 → 生命周期/去重
        │  UnifiedTrackArray(position in odom + contributor in arm1/base_link)
        │  /perception/tracks/unified
        ▼
lgw_bt_planner/servo_node
```

整条链的 **identity（track_id）只在 3D tracker 里产生**，检测侧不保留任何
跨帧 ID。

### 0.1 启动顺序（2026-11-15）

链路由**三个启动脚本**组成，按顺序拉起：

| 步骤 | 在哪 | 命令 | 启动的内容 |
|---|---|---|---|
| 1. CAN + 机械臂驱动 | 宿主机 | `bash scripts/start_lgw_arm.sh --driver-only` | can0 + `agx_arm_ctrl_single` |
| 2. YOLOE + 两路相机 | 宿主机 | `bash scripts/start_lgw_yoloe.sh` | 宿主 Foxy 的 `dual_camera_foxy` 推理循环 + 容器内 `two_realsense.launch.py`（含 USB 复位、等首帧结果） |
| 3. MoveIt + Servo + tracker | 宿主机(内部 docker exec) | `bash scripts/start_lgw_chain.sh follow:=true` | `lgw_full.launch.py`(MoveIt/Servo/lgw servo/可选 BT) + `unified_track_node` |

`lgw_chain.launch.py` 只含第 3 步：相机在 `yoloe_humble_orchestration` 的
install 空间、YOLOE 在宿主 Foxy 环境，都由 `start_lgw_yoloe.sh` 管。

**TF 归属**：tracker 发 `odom→base_link`、`base_link→arm1/world`、
`tcp_link→camera_*`；`arm1/world→…→arm1/tcp_link` 由 robot_state_publisher
根据 `/feedback/joint_states` 发（**驱动不在时这条链会静默消失**）。
servo 启动时做一次一致性检查：FK + `arm_calibration.json` 算出的相机位姿
必须与 TF 链一致，不一致会打 `[calib] camera TF disagrees with FK`。

---

## 1. 相机层：硬件 → ROS 话题

两路 RealSense 由容器内的
`yoloe_humble_orchestration/launch/two_realsense.launch.py` 启动。

| 项 | 值 | 说明 |
|---|---|---|
| 分辨率/帧率 | color 与 depth 都是 **640×480×15** | head 原为 1280×720，2026-10-02 降为 640×480 |
| 对齐 | `align_depth.enable=true` | depth 已对齐到 color，像素坐标一一对应 |
| 同步 | `enable_sync=false` | 硬件不同步，软件按 `header.stamp` 配对 |
| 点云 | `pointcloud.enable=false` | 3D 由 YOLOE 节点自己从 mask + depth 算 |
| `initial_reset` | `false` | 避免每次启动多等 ~21 s；USB 清理交给启动脚本的 unbind/bind |

每路的关键话题（见 `yoloe_deploy/config/runtime.yaml` 的 `two_stage.stage1/2`）：

```
/camera_head/camera_head/color/image_raw
/camera_head/camera_head/aligned_depth_to_color/image_raw
/camera_head/camera_head/color/camera_info

/camera_arm/camera_arm/...（同构）
```

YOLOE 节点默认 `image_transport=raw`，以 `qos_profile_sensor_data`
（BEST_EFFORT）订阅 `sensor_msgs/Image`
（`dual_camera_node.py:440-448`）。

---

## 2. YOLOE 节点内部

### 2.1 彩色/深度配对

`on_stage1_color/depth` 各自把最新消息存入 `stage1_latest_color/depth`，
随后调用 `try_stage1_pair()`（`dual_camera_node.py:774-833`）：

1. color、depth 都有才继续；
2. 比较 `header.stamp`，`|Δt| <= sync_slop_sec = 0.20 s` 才成对；
3. 超差时丢弃较旧的一张，等下一帧；
4. 成对后清空 latest，把 `(color_msg, depth_msg)` 写入
   `stage1_latest_pair`，并 `pair_seq += 1`。

**latest-wins，无队列**：每个相机只保留最新一帧对；处理不过来的中间帧会被
覆盖。arm 侧 `try_stage2_pair()` 逻辑相同。

### 2.2 调度与限速

- 主定时器 50 Hz（`scheduler_period_sec=0.02`）；
- **并行模式（当前默认，`parallel_cameras=true`）**：
  head/arm 各一个 worker 线程，各自 `_take_pair()`；每路
  `head_min_period_sec` / `arm_min_period_sec = 0.10 s`（最大 10 Hz）节流；
- **串行模式**：`on_scheduler()` 用 `inference_busy` 互斥，head/arm 轮流取帧；
- 相机源 15 Hz，但单帧推理 ~50–100 ms（含等 GPU），实际每路约
  **8–10 Hz**，两路合计 **~18–19 fps**。

代码位置：`on_scheduler()` `dual_camera_node.py:1213`；
`_parallel_worker()` 约 `:610`；`_take_pair()` 约 `:578`。

### 2.3 解码

`process_frames()`（`dual_camera_node.py:1348-1440`）：

- 彩色：`cv_bridge.imgmsg_to_cv2(..., "bgr8")`；
- 深度：`passthrough` 取原始数组；uint16 / 16UC1 乘
  `depth_scale_m_per_unit=0.001` 得米；float 原样；
- 校验 color/depth 尺寸一致；
- 若该路 `camera_info` 尚未到达，跳过该路。

### 2.4 YOLOE 推理

`MultiPromptPredictor.track_batch()`
（`yoloe_transfer/inference.py:351-366`）：

- 当前 `enable_2d_tracker: false` → 直接回退 `predict_batch()`；
- Ultralytics `model.predict`，参数：
  `imgsz=1024`、`conf=0.15`、`iou=0.65`、`max_det=50`、`batch=1`、
  backend=engine（FP16 TensorRT，VPE 已烘进引擎）；
- 640×480 先 letterbox 到 1024×1024，输出映射回原图坐标；
- 每条结果包含 `boxes.xyxy / cls / conf` 与 `masks.xy`（多边形，原图坐标）。

> 若恢复 `enable_2d_tracker=true`，这里会走 `model.track()` 并填
> `boxes.id`；下游 `instance_id` 才会有非零值。当前 identity 只由 3D
> tracker 负责。

### 2.5 2D 后处理

`_result_to_detections()`（`inference.py:155-190`）：

- `xyxy` 裁剪到 `[0, W-1] × [0, H-1]`；
- `class_id → class_name`（engine 里烘的是 `red_block` / `wood_block` 等）；
- `masks.xy` 用 `cv2.fillPoly` 画成 640×480 的 bool mask；
- 2D `track_id = 0`（2D tracker 关闭）。

`_postprocess_candidates()`（`inference.py:126-140`）：

1. **包含关系去重**：同类且
   `IoM = intersection / min(area) > 0.80`，保留面积更小的框
   （解决"大框 + 内部小框"，2026-10-26 实测小框才是真实目标）；
2. **跨提示组 NMS**：`prompt_merge_iou = 0.5`。

### 2.6 mask + depth → 3D 点（核心计算）

对每条检测调用 `position_from_mask()`
（`yoloe_transfer/geometry.py:86-129`）：

1. 有效像素：
   `mask & isfinite(depth) & 0.15 m <= depth <= 5.0 m`；
2. 有效点数 `< min_valid_depth_points=30` → `position_valid=false`；
3. 取这些像素的深度 `z`，算中位数 `z_med` 与 MAD；
4. 稳健区间：`|z - z_med| <= max(0.03 m, 3 × 1.4826 × MAD)`，
   剔除前景遮挡/边缘离群点；
5. 剔除后仍 < 30 点 → invalid；
6. 用 CameraInfo 的 `fx, fy, cx, cy` 做针孔反投影
   （相机光学系：+X 右、+Y 下、+Z 前）：

   ```
   X = (u - cx) · Z / fx
   Y = (v - cy) · Z / fy
   Z = depth
   ```

7. `center_m = median(XYZ, axis=0)`：逐轴中位数，不是均值；
8. `size_m = percentile(XYZ, 95) - percentile(XYZ, 5)`（逐轴），
   再乘体积得到 `volume_cm3`；
9. 按 `point_stride=4` 抽样、最多 `max_points_per_instance=2000` 点，
   供体积/调试使用。

**输出单位是米，坐标系是相机彩色光学系。**

### 2.7 组装并发布 ObservationResult

`_publish_camera_detections()`（`dual_camera_node.py:1470-1545`）：

1. 过滤：`conf >= trigger_confidence(0.15)`、`require_valid_3d=true`
   （必须 `position_valid`）、尺寸过滤（`size_filter.enabled=false`，
   当前不生效）；
2. 按置信度降序，最多 `max_candidates=3` 条；
3. 每条转成 `YoloeDetection3D`：
   `class_name / confidence / bbox / mask_area_px / position_valid /
   center_m / size_m / 体积字段 / valid_depth_points`；
4. `header.frame_id = camera_info.header.frame_id`
   （`camera_head_color_optical_frame` /
   `camera_arm_color_optical_frame`）；
   `header.stamp` **直接复制彩色图时间戳**；
5. `request_id = 彩色图 stamp 的纳秒值`（不是递增序号）；
6. `status = CONFIRMED`（有候选）或 `NO_TARGET`（无候选）。

发布：

```
/perception/two_stage/camera_head/result   ObservationResult, RELIABLE depth 10
/perception/two_stage/camera_arm/result    ObservationResult, RELIABLE depth 10
```

**此阶段仍是相机光学系坐标，且没有跨帧 identity。**

> 注意：`dual_camera_node.py` 当前是**纯 continuous 模式**。
> `runtime.yaml` 里的 `two_stage.enabled` / `require_camera2_ready` /
> `observation_request` 等字段它并不读取（两阶段逻辑在另一个
> `two_stage_node.py`，当前链路未启动）。不要按 README 的 v5 两阶段
> 描述去期待 `ObservationRequest` / `ObservationCommand`。

---

## 3. 3D tracker 内部

### 3.1 输入适配：相机系 → odom / 臂基座系

`unified_track_node._on_head_detection/_on_arm_detection` 只把消息存入
`_pending_head/_pending_arm`；`_request_pipeline()` 做 **30 ms debounce**，
`_run_pipeline()`（`unified_track_node.py:222-300`）取出两路最新 pending。

> 这里同样是 latest-wins：30 ms 内到达的多条消息会互相覆盖。

对每条消息调用 `InputAdapter.adapt()`（`input_adapter.py:191-240`）。

#### 第一步：算 `T_cam_odom`（`_cam_to_odom`，`input_adapter.py:139-175`）

head 相机（分三段查，避免 tf2 双 parent 歧义）：

```
T_cam_odom = T_base_odom ⊗ T_armbase_base ⊗ T_cam_armbase

T_base_odom    = lookup("odom"  ← "base_link")            # /utlidar/robot_pose
T_armbase_base = lookup("base_link" ← "arm1/base_link")   # tracker 静态安装桥
T_cam_armbase  = lookup("arm1/base_link" ← camera_head_color_optical_frame)
```

arm 相机：

```
T_cam_odom = T_base_odom ⊗ T_tcp_base ⊗ T_cam_tcp

T_tcp_base = lookup("base_link" ← "arm1/tcp_link")        # URDF 关节 TF
T_cam_tcp  = lookup("arm1/tcp_link" ← camera_arm_color_optical_frame)
```

- 每次 lookup 都**先 `can_transform(..., timeout=0)` 探测**，缺帧立即返回，
  避免阻塞执行器（2026-10-23 实测：缺失帧 `lookup_transform(timeout=0.05)`
  平均 62.7 ms，`can_transform(0)` 为 0.0 ms）；
- 按检测 `stamp` 查不到时退回"最新可用"；
- **任何一段失败 → 该相机整批观测直接丢弃（`return []`）**，不是逐条丢。

#### 第二步：算 `T_cam_armbase`（`_cam_to_arm_base`）

- head：`lookup("arm1/base_link" ← camera_head optical)`，纯静态链；
- arm：`lookup("arm1/base_link" ← camera_arm optical)`，含当前关节角
  （eye-in-hand）。

#### 第三步：逐条检测变换

```
p_odom    = R_odom_cam · center_m + t_odom_cam
p_armbase = T_armbase_cam · [center_m, 1]
```

生成 `TrackObservation`：
`class_name / confidence / source_camera / stamp_ms / position_odom /
position_arm_base / bbox_2d / size`。

> `position_arm_base` 只是"这个相机这条检测在臂基座系的坐标"；
> 相机链缺失时它为 `None`，但 odom 位置照旧，轨迹仍可建立。

### 3.2 KF 预测

`Tracker.predict_all()`（`tracker.py:70-75`）对每条非 deleted 轨迹：

```
x_pred = x              # 静态目标模型：没有速度状态
P_pred = P + Q          # Q = sigma_p² · I，sigma_p = 0.05 m/tick
```

这是一个 **3 维位置 KF，不是 6 维匀速模型**。目标假设为世界系静止，
靠每 tick 的过程噪声吸收抖动。

### 3.3 两阶段关联（Hungarian）

`TwoStageAssigner.assign()`（`cost_matrix.py:70-150`）：

- 只与 `CONFIRMED / COASTING` 轨迹关联；
- 阶段一：`conf >= 0.40` 的高置信观测；
- 阶段二：`0.15 <= conf < 0.40` 的低置信观测，只和阶段一未配上的轨迹配；
- 代价：

  ```
  cost[i,j] = ||p_odom_i - x_j||   若 class 相同且距离 < 0.15 m
            = +∞                   否则
  ```

- `scipy.optimize.linear_sum_assignment` 求最小代价一对一匹配；
- 未配上的观测再做一轮 **tentative 贪心匹配**
  （同类 + 15 cm 内，挑最近）。

当前主路径的门控是**欧氏距离 0.15 m**，不是 `KalmanFilter3D.maha_distance()`
（后者已实现但未在 `Tracker` 主路径使用）。

### 3.4 建轨前抑制 / 新建轨迹

`tracker.py:160-215`：

- 仍未匹配的观测，若与任一现有轨迹 **同类且 3D 距离 < 4 cm**，直接丢弃
  （`AssociationReport.suppressed`），不建新轨。用于解决"同一木块被 YOLOE
  拆成顶面/正面两个实例"导致的 id 空转（2026-10-26 实测 p50 3.03 cm）；
- 其余观测新建 `TENTATIVE` 轨迹：

  ```
  state.x = p_odom
  state.P = 0.05 · I
  hits    = 1
  ```

### 3.5 KF 更新

`KalmanFilter3D.update()`（`kf.py:45-80`），P、R 都是对角阵，逐轴标量增益：

```
K_i   = P_ii / (P_ii + R_ii)        # R = sigma_obs² · I，sigma_obs = 0.03 m
x'    = x + K ⊙ (z - x)
P'_ii = (1 - K_i) · P_ii
```

同时：

- `time_since_last_obs_ms = 0`；
- `hits += 1`；
- `last_obs_by_camera[source_camera] = obs`（head/arm 各留一份）；
- 若原状态是 `COASTING` → 回到 `CONFIRMED`（否则轨迹一旦 coasting
  就永久停在 coasting，2026-10-23 真机 bug）。

### 3.6 生命周期

`tick_lifecycle(dt_ms)` → `Track.tick()`（`track.py:100-130`）：

```
tentative: hits >= 3             → confirmed
confirmed: 缺失 >= 300 ms         → coasting
coasting:  缺失 >= 1500 ms        → deleted（不再发布）
time_since_last_obs_ms += dt_ms   # dt = 两次 pipeline 之间的墙钟 ms
```

每 tick 还会跑 `dedupe_tracks()`：同类且 3D 距离 < 4 cm 的多条轨迹只保留
最成熟的一条（confirmed/coasting 优先，其次 hits 多、创建早、id 小）。

### 3.7 输出

`OutputFormatter.format()`（`output_formatter.py:63-160`）把每条非 deleted
轨迹转成 `UnifiedTrack`：

| 字段 | 来源 |
|---|---|
| `track_id / status / age / time_since_last_observation_ms` | Track |
| `class_name / confidence / last_bbox_2d / size_*` | `last_obs` |
| `position` + `position_covariance[9]` | **KF 状态 `x` 与 `P`（odom 系）** |
| `source_cameras` | 该轨迹被哪些相机观测过 |
| `geometry_frame` | `"arm1/base_link"` |
| `last_observation_stamp` | 所有相机里**最新的检测捕获 stamp** |
| `contributor_cameras` | `"head" / "arm"`（按短名排序，稳定输出） |
| `contributor_positions_arm_base` | 每相机最近一次的臂基座系坐标 |
| `contributor_stamps / confidences` | 对应 stamp 与置信度 |

发布：

```
/perception/tracks/unified
  UnifiedTrackArray, RELIABLE depth 10, header.frame_id = "odom"
```

- `header.stamp` = tracker 当前时间；
- `frame_stamp_ms` = 相对启动的 ms；
- `last_observation_stamp` = 真实观测时刻（servo 用它判断新鲜度）。

**输出包含 `tentative` 轨迹**（只是不发布 deleted）。servo 的自动选靶
只接受 `status == "confirmed"`。

---

## 4. 时间戳与坐标系

这是最容易出错、也最值得单独记住的部分：

```
检测 stamp T_cam（彩色图捕获时刻，容器时钟）
   │
   ├─ odom → base_link：动态 TF，stamp = 远端 stamp + min(now - stamp) 的偏移校正
   │    （odom_tf_broadcaster.py：估计 Go2 与 Jetson 的固定时钟差；
   │     远端时钟跳变时清窗重新收敛）
   ├─ base_link → arm1/world：tracker 静态桥（arm_mount_offset，当前单位阵）
   ├─ arm1/world → arm1/base_link → … → arm1/tcp_link：URDF/RSP，关节角动态
   └─ tcp_link → camera_arm_link / arm1/base_link → camera_head_link：
        tracker 静态安装桥（TRANSIENT_LOCAL）
```

- 相机安装桥是静态 TF，任意 stamp 都可查；
- 动态段（odom、关节）按 `T_cam` 插值；检测比最新 pose 新时退回 latest；
- 因此 tracker 输出的 `position` 是"目标在检测时刻的 odom 位置"；
  `last_observation_stamp` 是捕获时刻；
- servo 用 `now - stamp` 判断几何新鲜度：
  `head_max_age_ms = 400`、`arm_max_age_ms = 400`、
  `odom_fallback_max_age_ms = 1500`。

---

## 5. 关键参数速查

### 5.1 检测侧（`yoloe_deploy/config/runtime.yaml`）

| 参数 | 值 | 作用 |
|---|---|---|
| `inference.imgsz` | 1024 | 推理分辨率 |
| `inference.confidence` | 0.15 | Ultralytics 召回阈值 |
| `inference.iou` | 0.65 | NMS IoU |
| `inference.max_det` | 50 | 单帧最大检测数 |
| `inference.enable_2d_tracker` | false | 关闭 2D 跟踪 |
| `inference.containment_dedup_iom` | 0.80 | 包含关系去重 |
| `inference.prompt_merge_iou` | 0.5 | 跨提示组 NMS |
| `camera.width/height/fps` | 640/480/15 | 相机流 |
| `camera.min/max_depth_m` | 0.15 / 5.0 | 有效深度范围 |
| `camera.min_valid_depth_points` | 30 | 3D 解算最少点数 |
| `camera.point_stride` | 4 | 体积抽样步长 |
| `camera.max_points_per_instance` | 2000 | 单目标最大点数 |
| `size_filter.enabled` | false | 体积过滤关闭 |
| `stage1.trigger_confidence` | 0.15 | head 发布阈值 |
| `stage2.confirmation_confidence` | 0.15 | arm 发布阈值 |
| `max_candidates` | 3 | 单路单帧最多发布条数 |
| `require_valid_3d` | true | 无有效 3D 不发布 |
| `parallel_cameras` | true | 两路并行推理（2026-11-06） |
| `sync_slop_sec` | 0.20 s | color/depth 配对容差 |
| `head/arm_min_period_sec` | 0.10 s | 每路最小处理间隔（launch 值） |

### 5.2 跟踪侧（`lgw_3d_tracker/config/runtime.yaml`）

| 参数 | 值 | 作用 |
|---|---|---|
| `process_noise_position` | 0.05 m/tick | KF Q |
| `observation_noise_position` | 0.03 m | KF R |
| `tentative_min_hits` | 3 | 升 confirmed 所需命中 |
| `coast_enter_ms` | 300 | confirmed → coasting 迟滞 |
| `coast_max_age_ms` | 1500 | coastline → deleted |
| `association_distance_gate_m` | 0.15 m | 关联门控 |
| `duplicate_track_distance_m` | 0.04 m | 重复/影子轨迹合并半径 |
| `two_stage.high_conf_threshold` | 0.40 | 阶段一 |
| `two_stage.low_conf_threshold` | 0.15 | 阶段二 |
| `output_min_period_ms` | 30 | pipeline debounce |
| `arm_frame_prefix` | `arm1/` | 机械臂 TF 前缀 |
| `arm_mount_offset` | 单位阵 | **未实测**（见 §7） |

---

## 6. 信息在哪一步会丢

| 步骤 | 丢弃条件 | 影响 |
|---|---|---|
| 相机配对 | color/depth Δt > 0.20 s | 该帧不成对，等下一帧 |
| 调度 | 每路 > 10 Hz 的新帧 | 只处理最新一对，旧帧被覆盖 |
| 3D 解算 | mask 内有效深度点 < 30 | `position_valid=false`，不发布 |
| 发布过滤 | conf < 0.15 / 无有效 3D / 尺寸过滤开启时超限 | 不进 ObservationResult |
| tracker 适配 | 任一段 TF 缺失 | **该相机整批观测丢弃** |
| tracker 关联 | 类别不同 / 距离 > 15 cm | 可能新建轨迹或被抑制 |
| 建轨前抑制 | 同类且距已有轨迹 < 4 cm | 观测被丢弃（重复面/重复框） |
| 生命周期 | 缺失 > 1.5 s | 轨迹删除，不再发布 |

另外两处"覆盖"不是丢弃，但会改变数据节奏：

- YOLOE 每路只保留最新一帧对；
- tracker 每次 pipeline 只消费两路最新 pending，30 ms 内的旧消息被覆盖。

---

## 7. 已知前提与边界

1. **硬依赖 `/utlidar/robot_pose`**：没有 `odom → base_link`，
   `_cam_to_odom` 返回 `None`，tracker 整批丢弃。当前真机实测约 19 Hz。
2. **硬依赖机械臂 TF**：`arm1/base_link`、`arm1/tcp_link` 由 RSP 发布；
   机械臂未启动或 `frame_prefix` 配错时，arm 相机观测会被整批丢弃，
   head 相机仍可用（head 链只依赖 `base_link` 与静态桥）。
3. **`arm_mount_offset` 是单位阵（未实测）**：影响 head 观测的 odom 位置、
   自动选靶的"最近"排序、odom fallback tier；**不影响** servo 的
   head/arm 臂基座 tier（那条链直接查 `arm1/base_link ← camera_optical`）。
4. **`two_stage` 配置与 `dual_camera_node.py` 不一致**：该节点是
   continuous-only，不读 `two_stage.enabled` / `require_camera2_ready`。
5. **`enable_2d_tracker=false`** 时 `instance_id=0`，
   `trajectory_visualizer` 等依赖 2D track_id 的调试工具失效。
6. **head/arm 3D 存在 ~4 cm 系统性偏差**（2026-10-26 遗留，p50 3.99–4.37 cm）。
   不破坏关联（门控 15 cm），但会影响抓取精度。
7. **头臂一致性的前提**：head 安装桥的 parent 是机械臂基座
   `arm1/base_link`（head 标定是 eye-to-hand 产物），不是机器人本体的
   `base_link`。改标定/桥时不要混用。

---

## 8. 观察与验证命令

```bash
# 1) 检测输出频率与内容
ros2 topic hz /perception/two_stage/camera_head/result
ros2 topic echo /perception/two_stage/camera_head/result --once

# 2) 跟踪输出频率与字段
ros2 topic hz /perception/tracks/unified
ros2 topic echo /perception/tracks/unified --once

# 3) TF 链是否连通（servo/tracker 的前提）
ros2 run tf2_ros tf2_echo odom arm1/base_link
ros2 run tf2_ros tf2_echo arm1/base_link camera_head_color_optical_frame
ros2 run tf2_ros tf2_echo arm1/tcp_link camera_arm_color_optical_frame

# 4) 带逐 tick JSONL 的 tracker（离线分析用）
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
 "source /my_ws/l4t/.l4t_env.sh && \
  ros2 launch lgw_3d_tracker unified_track_node.launch.py \
    trace_log_dir:=/my_ws/l4t/lg_walker_ws/.run trace_tag:=_debug"

python3 scripts/analyze_tracker_trace.py \
  l4t/lg_walker_ws/.run/lgw_3d_tracker_<ts>_debug.jsonl
```

---

## 9. 关键文件索引

| 环节 | 文件 | 入口 |
|---|---|---|
| 相机 bringup | `yoloe_deploy/humble_ws/src/yoloe_humble_orchestration/launch/two_realsense.launch.py` | `generate_launch_description` |
| 检测主节点 | `yoloe_deploy/ros2_ws/src/yoloe_perception/yoloe_perception/dual_camera_node.py` | `on_scheduler` / `process_frames` / `_publish_camera_detections` |
| 推理与 2D 后处理 | `yoloe_deploy/yoloe_transfer/inference.py` | `predict_batch` / `track_batch` / `_result_to_detections` |
| 3D 反投影 | `yoloe_deploy/yoloe_transfer/geometry.py` | `position_from_mask` / `size_metrics` |
| 跟踪主节点 | `lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py` | `_run_pipeline` |
| 坐标系适配 | `lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py` | `adapt` / `_cam_to_odom` |
| KF | `.../unified_track/kf.py` | `predict` / `update` |
| 关联 | `.../unified_track/cost_matrix.py` | `TwoStageAssigner.assign` |
| 生命周期 | `.../unified_track/track.py` | `Track.tick` |
| 跟踪协调 | `.../unified_track/tracker.py` | `associate_and_update` / `tick_lifecycle` |
| 输出格式化 | `.../unified_track/output_formatter.py` | `format` |
| 时钟校正 | `.../unified_track/odom_tf_broadcaster.py` | `update` |

---

## 10. 一句话总结

相机输出「像素 + 深度」；YOLOE 把 mask 内有效深度点反投影成**相机系 3D
中位数**，得到 `center_m`；tracker 用 TF 把它变到 **odom**（用于关联/KF）
和 **`arm1/base_link`**（用于伺服），用 **3D 静态 KF + 两阶段 Hungarian +
4 cm 去重**把多相机、多帧的检测归并成稳定 `track_id`，再把 KF 位置和每相机
臂基座坐标一起发布给 servo。整条链的 identity 只在 3D tracker 里产生。
