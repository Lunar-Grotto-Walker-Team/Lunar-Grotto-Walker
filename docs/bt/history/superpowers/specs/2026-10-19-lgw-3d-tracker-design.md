# lgw_3d_tracker: Unified 3D Tracking for lg_walker_ws

> 日期:2026-10-19
> 状态:设计评审(spec 评审中)
> 实施范围:设计文档 `/home/unitree/yoloe_deploy/docs/混合3D跟踪设计.md` 的 **Phase A-C**(核心跟踪,不含 BT demo / 不动 dual_camera_node)
> 工作区:`/home/unitree/my_ws/l4t/lg_walker_ws`(容器内 `/my_ws/l4t/lg_walker_ws`)
> 测试模式:**纯订阅**(只订阅真话题,不提供 mock 发布器,不加单元测试)
> 关联上游 spec: `混合3D跟踪设计.md`(`yoloe_deploy/docs/`)

---

## 0. 设计对照(说明与上游文档的偏离)

本 spec 实现上游 `混合3D跟踪设计.md` 的核心算法,但**位置、坐标系来源、TF 链** 等因为本工作区现状做了调整。**关键差异**(从上游 spec 改写):

| 项 | 上游设计 | 本 spec 实现 | 理由 |
|---|---|---|---|
| 实现位置 | `yoloe_deploy/ros2_ws/src/yoloe_perception/` | `lg_walker_ws/src/lgw_3d_tracker/`(新包) + `lg_walker_ws/src/yoloe_interfaces/`(扩 msg) | `yoloe_deploy` 不在 `l4t_setup.sh` overlay,容器跑不到 |
| `arm_base → odom` 来源 | `/lf/sportmodestate.position` | **`/utlidar/robot_pose`** (SLAM 校正) | `dddmr_go2_ws/task-4` 已验证 SDK `position` 不可靠,且 SLAM 不会让静态 track 漂 |
| 静态 TF parent | `arm_base_link` | **`base_link`** | 跟随 `lgw_bt_planner` 已存约定(本阶段 `arm_base ≡ base_link` 等价) |
| 静态 TF 发布者 | `unified_track_node` 启动期 | 同上 | 项目内无其他节点发 head_cam static TF |
| Tick 模型 | 隐式,每相机话题触发一次 | **每收到任一相机 YoloeDetection3DArray 跑一次完整 pipeline**(空 detection 也跑) | KF 需 `predict()` 让 P 增长以触发 coasting→deleted |
| KF 时间步 | 隐式 dt | **dt=1 / per-tick**(Q 单位 `m/tick`,非 `m/s`) | 静态模型 `A=I`,dt 不影响 x 但影响 P,Q 必须按 tick 率调 |
| 状态字段 velocity | 上游文档 §5.2 提到 6 维但实现选 3 维 | **3 维,不发 velocity 字段**(与上游一致) | 静态目标 v=0,3 维 = 6 维退化态 |
| 时间同步 | 上游隐式,默认 latest pose | **TF lookup at detection.stamp + 动态 odom→base_link TF** | TF2 buffer 原生线性插值,ROS2 标准习语,显式处理 5Hz detection × 17.8Hz SLAM 时间不同步 |

**与上游一致的核心**:算法(KF、两阶段关联、跨相机合并、生命周期、输出格式)、yaml 参数、nuScenes-style 字段对齐、Phase A-C 分阶段边界、关键决策记录。

---

## 1. 目标与边界

### 1.1 目标

按重要性排序:

1. **每个目标具有全局唯一 track_id**(跨相机一致)
2. **track 在统一世界系 `odom` 下报告 3D 位置**(消费者无须知道传感器拓扑)
3. **消费 `/utlidar/robot_pose` SLAM 实时位姿做世界系转换**(17.8 Hz,延迟 < 100ms,足够支撑 5Hz tracker)
4. **不引入新的检测模型**(沿用 `YoloeDetection3DArray` 输出)
5. **降级路径清晰**:任一相机下线时仍能跑(失去跨相机合并能力)

### 1.2 边界(本期不做)

- ❌ BT 节点 `ReadUnifiedTrack`、BT xml demo、行为树迁移(Phase D)
- ❌ `dual_camera_node.py` 关闭 BoT-SORT(老 `observation_result` 链路保留)
- ❌ 真机 `arm_base → base_link` 精细标定(沿用 §3 假设)
- ❌ 视觉一致性(颜色余弦 / ReID)
- ❌ unit test / mock publisher / launch-time launch test

### 1.3 输出

- `lgw_3d_tracker` 包(Python,`ament_python`)
- 2 个新 msg 加到 `yoloe_interfaces`
- `unified_track_node` 主节点 + 9 个子模块(config / calibration / odom_tf_broadcaster / input_adapter / kf / track / cost_matrix / tracker / output_formatter)+ 1 个 launch + 1 个 yaml

---

## 2. 包与文件布局

### 2.1 `yoloe_interfaces/` 扩展(已有包,加 2 个 msg)

`l4t/lg_walker_ws/src/yoloe_interfaces/msg/`:

```
UnifiedTrack.msg            # 新增
UnifiedTrackArray.msg       # 新增
(ObservationResult.msg 等已有不变)
```

**UnifiedTrack.msg**(字段照搬上游 §5.2,去除 velocity):

```text
builtin_interfaces/Time stamp
uint32 frame_stamp_ms

# === identity ===
uint64 track_id
string status                   # tentative | confirmed | coasting | deleted
int32 age
int32 time_since_last_observation_ms

# === 类别与置信度 ===
string class_name
float32 confidence
int32 observation_count
int32 confirmed_hits

# === 3D 状态(odom 系,3 维 KF) ===
geometry_msgs/Point position           # KF 后验 [x,y,z],米
float64[9] position_covariance         # 3x3 行优先,来自 KF P

# === 2D 辅助信息(用于跨相机合并 + 可视化) ===
sensor_msgs/RegionOfInterest last_bbox_2d
string last_source_camera
float32[3] last_bbox_color
string[] source_cameras

# === 主流 MOT 兼容字段(参考 nuScenes sample_annotation) ===
float32 width_m
float32 length_m
float32 height_m
bool size_valid
```

**UnifiedTrackArray.msg**:

```text
std_msgs/Header header                  # frame_id = "odom"
builtin_interfaces/Time wall_stamp
uint32 frame_stamp_ms
uint64 sequence_id
UnifiedTrack[] tracks
```

### 2.2 `lgw_3d_tracker/` 新包

`l4t/lg_walker_ws/src/lgw_3d_tracker/`:

```
package.xml                    # build_depend: yoloe_interfaces, geometry_msgs, sensor_msgs, std_msgs, tf2_ros, tf2_geometry_msgs, nav_msgs, unitree_go (不需要),geometry_msgs
setup.py                       # entry_points: unified_track_node = lgw_3d_tracker.unified_track_node:main
setup.cfg
resource/lgw_3d_tracker
launch/unified_track_node.launch.py
config/runtime.yaml            # unified_track 段
lgw_3d_tracker/
├── __init__.py
├── unified_track_node.py      # main,sub/pub/TF loader 一处管
└── unified_track/
    ├── __init__.py
    ├── config.py              # YAML 加载
    ├── calibration.py         # head_calibration.json 解析 + StaticTransformBroadcaster
    ├── odom_tf_broadcaster.py # /utlidar/robot_pose → 动态 odom→base_link TF
    ├── input_adapter.py       # YoloeDetection3DArray → odom 系观测点(用 TF lookup at det.stamp)
    ├── kf.py                  # 3 维 NumPy KF(predict/update/gate)
    ├── track.py               # Track dataclass + lifecycle 状态机
    ├── cost_matrix.py         # 3D 距离 cost + 两阶段 Hungarian
    ├── tracker.py             # 协调 predict/associate/update/lifecycle
    └── output_formatter.py    # UnifiedTrack[] ← Track[]
```

**依赖(只列关键)**:`numpy`, `scipy`(Hungarian `linear_sum_assignment`),`rclpy`, `tf2_ros`, `tf2_geometry_msgs`, `ament_index_python`(找 share/),`yoloe_interfaces`(msg),`geometry_msgs`, `sensor_msgs`, `std_msgs`, `nav_msgs`(PoseStamped)。

---

## 3. 坐标系统一(本工作区具体化)

### 3.1 选定:`odom` 系(Go2 L1/L2 SLAM 系)

`/utlidar/robot_pose` 是 Go2 L1/L2 SLAM 校正后的位姿,**SLAM 全局一致,不会像轮式里程计那样随时间漂**——这对**静态目标跟踪**至关重要(否则静态 track 在 odom 系下会跟着 odom 漂移)。

**对照**:
- `/utlidar/robot_odom` (150 Hz, Odometry) — 轮式里程计,长时漂
- `/utlidar/robot_pose` (17.8 Hz, PoseStamped) — SLAM 校正,**本期选定**

### 3.2 完整变换链

```
head_cam(相机系)
   ↓ T_head_cam_to_base_link  (head_calibration.json)
base_link
   ↓ T_base_link_to_odom  (/utlidar/robot_pose,17.8 Hz SLAM)
odom
```

```
arm_cam(相机系)
   ↓ T_arm_cam_to_tcp_link  (arm_calibration.json,已叠 12cm tcp_offset)
tcp_link
   ↓ T_tcp_link_to_arm_base  (agx_arm_ctrl 200Hz FK)
arm_base ≡ base_link  (本阶段假设)
   ↓ T_base_link_to_odom  (/utlidar/robot_pose)
odom
```

**arm_base ≡ base_link 假设**:沿用上游设计 §8.2。当前阶段粗略,机械臂在狗背上有几 cm 偏移和小角度倾角,设计阶段按 0 偏移处理。精细抓取阶段(±1cm 落点)重做标定。

### 3.3 来源表

| 变换 | 来源 | 频率 | 状态 |
|---|---|---|---|
| `head_cam → base_link` | `head_calibration.json` `position` + `orientation` | static | ✅ 启动期广播(本节点) |
| `arm_cam → tcp_link` | `arm_calibration.json`(已叠 12cm z offset) | static | ✅ 由 `agx_arm_ctrl` 消费 |
| `tcp_link → arm_base` | `agx_arm_ctrl` 200Hz FK | 200Hz | ✅ 运行期 |
| `base_link → odom` | `/utlidar/robot_pose.pose` | 17.8 Hz | ✅ 本节点订阅 |
| `base_link → arm_base` | **本阶段假设 I** | — | 🟡 假设 |

### 3.4 TF 发布总览

`unified_track_node` 启动期(`rclpy.init` 后立即):
1. 读 `head_calibration.json`,取 `position` 和 `orientation`(xyzw 四元数)
2. 用 `tf2_ros.StaticTransformBroadcaster` 发布 `base_link → camera_head_color_optical_frame`
3. **必须在订阅话题前完成**,否则 TF lookup 早期失败

**运行时**(`/utlidar/robot_pose` callback):
1. 用 `tf2_ros.TransformBroadcaster` 发动态 `odom → base_link`,**stamp 用 msg.header.stamp 原值**(不替换)
2. 频率跟随 `/utlidar/robot_pose` (~17.8Hz)
3. 由 `odom_tf_broadcaster.py` 模块封装

**`arm_calibration.json` 不重复加载**(已有 `agx_arm_ctrl` 消费者)。

---

## 4. 数据流与节点结构

### 4.1 Topic 清单

| 角色 | 话题 | 类型 | QoS | 频率 |
|---|---|---|---|---|
| Sub | `/perception/two_stage/camera_head/result` | `YoloeDetection3DArray` | 默认 RELIABLE | ~5Hz |
| Sub | `/perception/two_stage/camera_arm/result` | `YoloeDetection3DArray` | 默认 RELIABLE | ~5Hz |
| Sub | `/utlidar/robot_pose` | `PoseStamped` | BEST_EFFORT | ~17.8Hz |
| Pub | `/perception/tracks/unified` | `UnifiedTrackArray` | RELIABLE | 5Hz |
| Static TF | `base_link → camera_head_color_optical_frame` | — | — | 启动一次,永久 |
| Dynamic TF | `odom → base_link` | — | — | 17.8Hz,持续 |

### 4.2 单次 tick 流水线

```
任一相机 topic 到达 → (若距上次 batch < 30ms,等待) → 完整 pipeline:

  1. input_adapter.adapt(detections)        # camera 系 → odom 系 3D 点(用 TF lookup at det.stamp)
  2. tracker.predict_all()                  # P += Q
  3. tracker.associate_and_update(obs)      # 两阶段 Hungarian + KF update
  4. tracker.tick_lifecycle()               # confirmed→coasting→deleted,time_since 累加
  5. output_formatter.format(tracks)        # UnifiedTrack[] 字段填充
  6. publish(UnifiedTrackArray)
```

**TF 时间同步**(关键):

- 节点订阅 `/utlidar/robot_pose` 后,callback 里用 `tf2_ros.TransformBroadcaster` 发 `odom → base_link` 动态 TF,**stamp = msg.header.stamp**(原值),不替换
- TF buffer `cache_time = 30s`(配置项),保留过去 30s 的所有 `odom → base_link` pose
- 每条 detection 转 odom 时:`tf_buffer.lookup_transform("odom", camera_frame, time=det.stamp, timeout=Duration(seconds=0.05))`
  - TF2 buffer 在相邻两个 SLAM pose 之间按时间线性插值
  - head_cam 静态 TF 与 odom→base_link 动态 TF 在 lookup_transform 里自动串联,返回完整 `odom → camera_frame` 一次性变换
- **不再单独维护 `sport_pose_cache`**(TF buffer 兼了)

**Tick 触发**:
- 任一相机 topic 到达,**记录到 `_pending_msg_queue`**(head、arm 各保留最新一条),并请求一次 pipeline 调度
- `sport_pose_cache` 更新**不**触发 tick(只更新缓存,供下次 tick 取最新位姿)
- pipeline 调度器实现:topic 回调里 `request_pipeline()` → 若距上次实际执行 ≥ `output_min_period_ms`(默认 30ms)立即跑;否则用一个 `wall_timer` 在剩余时间(剩余 ≤ 30ms)触发,**timer 只设一次**(避免叠加)
- **空 detection 也跑 pipeline**(取 `_pending_msg_queue` 里的最新两条 head/arm → 都可能为空 → `predict_all + tick_lifecycle + format + publish`)。`predict_all + tick_lifecycle` 必须执行,否则 coasting→deleted 时间累计失效

**30ms debounce 目的**:
- head/arm 各 ~5Hz,周期 200ms,不会真正"撞车"
- debounce 30ms 处理:**单次回调里如果有另一路 30ms 内也到了,把两条 batch 一起处理**(同一 odom 系位姿),避免重复关联同一帧
- 配置项 `output_min_period_ms`(yaml 默认 30)

---

## 5. 输入适配器(input_adapter)

### 5.1 每条 detection 转换

```python
for det in msg.detections:
    if not det.position_valid:
        obs_invalid_count += 1
        continue

    # 一步到位:odom → camera_frame 的 TF lookup at detection.stamp
    # TF buffer 自动串联:odom→base_link(动态,SLAM 17.8Hz,按时间插值)+ base_link→camera_frame(静态,head_calibration.json)
    try:
        T_odom_to_cam = tf_buffer.lookup_transform(
            "odom", msg.header.frame_id,
            time=msg.header.stamp,        # 关键:用 detection 的时间戳,TF2 插值
            timeout=Duration(seconds=0.05))
    except tf2_ros.LookupException as e:
        log_warn("TF lookup failed: %s", str(e))
        # 不参与新 track 创建,只参与 update
        continue

    p_camera = Point(det.center_m.x, det.center_m.y, det.center_m.z)
    p_odom = transform_point(T_odom_to_cam, p_camera)

    obs.append(Observation(
        position_odom=p_odom,
        class_name=det.class_name,
        confidence=det.confidence,
        source_camera=msg.header.frame_id,
        bbox_2d=det.bbox,
        size=(det.size_m.x, det.size_m.y, det.size_m.z),
        size_valid=det.size_valid,
        stamp=msg.header.stamp,
    ))
```

**TF lookup 插值原理**:TF buffer 内同时存静态 TF 和动态 TF。
- 静态 TF 永远生效
- 动态 TF 由 `/utlidar/robot_pose` callback 用 `TransformBroadcaster.send_transform(stamped)` 写入,`stamp = msg.header.stamp`(原值)
- `lookup_transform(target, source, time=det.stamp)` 时,TF buffer 在相邻两个 `odom→base_link` pose 之间按 `(det.stamp - prev.stamp) / (next.stamp - prev.stamp)` 线性插值
- 由于 SLAM 输出间隔约 56ms(17.8Hz),detection 之间间隔约 200ms(5Hz),**detection 时刻大概率落在两个 SLAM pose 之间**,TF 插值保证位姿匹配

### 5.2 降级

| 失败模式 | 行为 |
|---|---|
| TF lookup 超时(50ms) | 用 TF buffer 最近一次值(API 默认行为,`lookup_transform` 在 timeout 内取能取到的最近 stamp),`position_valid=False`,**不参与新 track 创建**,只参与现有 track 的 update |
| TF lookup 极端失败(`ExtrapolationException` 因为 cache 过期) | 整个 batch 跳过,ERROR |
| `/utlidar/robot_pose` > 500ms 无更新 | 整个 batch 跳过,ERROR(TF buffer 已无可用插值源) |
| 两个相机 frame_id 都不在 TF 链 | 退出 ERROR,人工检查(常见因 head_calibration.json 加载失败) |

---

## 6. 跟踪算法

### 6.1 KF 状态与运动模型(3 维静态)

```
state x = [px, py, pz]^T       # odom 系,米
A = I3                            # 静态,x_pred = x
Q = σ_p² · I3                     # σ_p = process_noise_position(默认 0.05 m/tick)
H = I3                            # 直接观测位置
R = σ_obs² · I3                   # σ_obs = observation_noise_position(默认 0.03 m,D435i @ ~3m)
```

### 6.2 KF 步骤

```python
def predict(kf):
    kf.x = kf.x                    # 静态,x_pred = x
    kf.P = kf.P + Q                # P_pred = P + Q

def update(kf, z):
    y = z - kf.x                   # 残差
    S = kf.P + R                   # 残差协方差
    K = kf.P / S                   # 3 维标量除,等价矩阵逆
    kf.x = kf.x + K * y
    kf.P = (1.0 - K) * kf.P        # (I3 - K·H)·P

def maha_distance(kf, z):
    y = z - kf.x
    S = kf.P + R
    return float(y @ np.linalg.inv(np.diag(S)) @ y)  # 简化为对角矩阵
```

**关键设计决定**:
- `dt = 1 / tick`,**Q 是 per-tick 不是 per-second**(5Hz 下每 tick Q=0.05² = 0.0025 m²)
- 静态模型 `x` 不漂移,`P` 随 tick 增长(每 tick +Q);1.5s 不更新后 P ≈ R + 5*0.0025 = 0.043 m²,关联门控相应放宽到 ~0.2m

### 6.3 两阶段关联(cost_matrix)

**输入**:N 个新观测, M 个 active track(状态 confirmed / coasting)。

**步骤**:
1. **类别预筛**:cost[i, j] = +∞ if class_name[i] != class_name[j]
2. **距离预筛**:cost[i, j] = +∞ if distance(i, j) > `association_distance_gate_m`(默认 0.15)
3. **阶段一**:high_conf 观测(conf ≥ `high_conf_threshold`, 默认 0.40) ↔ active_tracks
   - cost = 3D 欧氏距离
   - `scipy.optimize.linear_sum_assignment` 求解
   - 配上的观测 → KF update
4. **阶段二**:low_conf 观测(`low_conf_threshold` ≤ conf < `high_conf_threshold`, 默认 0.15-0.40) ↔ **阶段一未配上的** track
   - 同样 Hungarian
   - 配上的观测 → KF update(用于遮挡恢复)
5. **未关联的观测**:创建新 tentative track(track_id 自增)
6. **未关联的 track**:`time_since_last_observation` 累加;`≥ coast_max_age_ms`(1500ms)→ deleted

### 6.4 生命周期状态机

```
                create
                  ↓
              tentative (hits=0,age=0)
                  ↓
   hits ≥ tentative_min_hits (默认 3)
                  ↓
              confirmed (发布给下游)
                  ↓
   time_since_last_observation ≥ 1 (不更新)
                  ↓
              coasting (KF 只 predict, 仍发布)
                  ↓
   time_since_last_observation ≥ coast_max_age_ms (1500ms)
                  ↓
              deleted (本期:不发布,publish 前过滤)
```

**为什么 tentative**:避免假阳性单帧创建大量 track。

**为什么 coasting**:静态目标偶尔被遮挡 1~2 帧(arm 自遮挡),coasting 让 arm 有缓冲去切换视角而不是直接丢失 track。

---

## 7. 输出与 frame_id

### 7.1 输出字段映射

| Track 字段 | UnifiedTrack 字段 | 来源 |
|---|---|---|
| track.id | `track_id` | 自增 |
| track.status | `status` 字面量 | 状态机 |
| track.hits | `observation_count` | 累计 |
| track.confirmed_hits | `confirmed_hits` | 状态机 |
| track.age | `age` | 累计 |
| track.ms_since_update | `time_since_last_observation_ms` | 状态机 |
| track.x[0..2] | `position.{x,y,z}` | KF 后验 |
| track.P.flatten() | `position_covariance[9]` | KF P |
| track.last_class | `class_name` | 上次观测 |
| track.last_conf | `confidence` | 上次观测 |
| track.last_bbox_2d | `last_bbox_2d` | 上次观测 |
| track.last_source_camera | `last_source_camera` | 上次观测 |
| track.color_avg | `last_bbox_color` | 上次观测 |
| track.sources | `source_cameras[]` | 累加集合 |
| track.last_size_m | `width_m/length_m/height_m + size_valid` | 上次观测 |

**不发字段**:velocity(3 维 KF 无);last_bbox_color 本期从 `detection.color` 字段读(若没有则填 [0,0,0])。

### 7.2 数组包装

- `UnifiedTrackArray.header.frame_id = "odom"`
- `UnifiedTrackArray.header.stamp = self.get_clock().now().to_msg()`(处理时刻)
- `sequence_id` 自增 1/tick
- `frame_stamp_ms` 自启动累计 ms(便于 rviz 时间对齐)
- `deleted` 状态的 track **不进**数组(已停发)

---

## 8. yaml 配置(`config/runtime.yaml`)

```yaml
unified_track:
  enabled: true

  # 输入订阅
  head_detections_topic: /perception/two_stage/camera_head/result
  arm_detections_topic:  /perception/two_stage/camera_arm/result
  sport_pose_topic: /utlidar/robot_pose            # 17.8Hz SLAM

  # 输出
  output_topic: /perception/tracks/unified
  output_frame_id: odom
  output_min_period_ms: 30                         # debounce

  # KF 参数(3 维 state,**单位 m/tick,非 m/s**)
  process_noise_position: 0.05                     # Q,5cm per-tick process noise
  observation_noise_position: 0.03                 # R,D435i depth @ 3m

  # 生命周期
  tentative_min_hits: 3
  coast_max_age_ms: 1500

  # 关联参数
  association_distance_gate_m: 0.15                # 3D 距离门控
  appearance_class_must_match: true                # class_name 必须相同
  two_stage:
    enabled: true
    high_conf_threshold: 0.40
    low_conf_threshold: 0.15

  # odom 缓存
  tf_cache_sec: 30.0                              # TF buffer 缓存时长,支持回溯 lookup
  robot_pose_stale_ms: 500                        # 超过此时间无 /utlidar/robot_pose → 跳过 batch

  # 标定 JSON 路径(从 ament_index 解析 lgw_bt_planner share)
  head_calibration_file: ""                        # 空 = 默认 share/.../head_calibration.json
```

`arm_calibration_file` 不需要(已有消费者)。

---

## 9. 错误处理与回退

| 失败 | 行为 |
|---|---|
| `head_calibration.json` 解析失败 | 节点退出非 0,ERROR |
| 任一相机 topic 超时 5s 无消息 | 仅用现有观测跑 pipeline,WARN,不创该来源新 track |
| TF lookup 超时 | 用 TF buffer 最近一次值(API 默认),`position_valid=False`,不参与新 track 创建 |
| `/utlidar/robot_pose` > 500ms 无更新 | 整个 batch 跳过,ERROR |
| KF P 出现负值(数值异常) | 强制 clip 到 `process_noise_position`,WARN |
| dual_camera_node 不在线 | 仍能跑(单相机跟踪),不阻断 |

---

## 10. 测试(纯订阅模式)

按用户选择,**只订阅真话题**,不提供 mock publisher,不写 unit test,不写集成 launch test。

**测试入口**(用户在真机上):
```bash
# 进入容器
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash

# build(一次性)
source /my_ws/l4t/l4t_setup.sh
colcon build --packages-select yoloe_interfaces lgw_3d_tracker

# 启动(假定 dual_camera_node 已运行)
source install/setup.bash
ros2 launch lgw_3d_tracker unified_track_node.launch.py
```

**验证清单**(任何一次能跑通即可,逐项打勾):
1. `ros2 topic list | grep unified` → 看到 `/perception/tracks/unified`
2. `ros2 topic hz /perception/tracks/unified` → 5Hz 左右
3. `ros2 topic echo --once /perception/tracks/unified` → 至少一个 example 输出(包含 `frame_id=odom`, `position.x/y/z`, `track_id`, `class_name`)
4. `ros2 node info /unified_track_node` → 看到 3 个 sub + 1 个 pub + static TF
5. `ros2 run tf2_tools view_frames` → 看到 `base_link → camera_head_color_optical_frame`
6. (额外)`ros2 topic echo /utlidar/robot_pose --once` 确认数据可达

---

## 11. 决策记录

| 决策项 | 选择 | 备选 | 理由 |
|---|---|---|---|
| 包 | **新建 `lgw_3d_tracker` 在 lg_walker_ws/src/** | 并入 lgw_bt_planner / 走 yoloe_deploy | 跟随 ws 约定("src/ 下只放自写包")+ 容器可 build/test |
| Msg 归属 | **加到 yoloe_interfaces** | 新建 lgw_3d_tracker_interfaces | 已有 yoloe_interfaces 是最省事的归口,语义"perception 接口" |
| 坐标系 | **odom (SLAM 系)** | arm_base / world / map / leg_odom | SLAM 全局一致,静态目标不漂 |
| odom 源 | **`/utlidar/robot_pose`**(SLAM,17.8 Hz) | /utlidar/robot_odom (轮式) | 静态目标场景;SLAM 不漂;17.8Hz 够用 |
| static TF parent | **base_link** | arm_base_link | 跟随 lgw_bt_planner 现有约定;本阶段等价 |
| static TF 发布 | **本节点启动期发** | 另起独立节点 | 测试时不启其他节点,需要本节点能独立成 |
| KF 状态维度 | **3 维 (xyz)** | 6 维 / cuboid | 静态目标 v=0,3 维 = 6 维退化态 |
| KF 时间步 | **dt=1 / tick,Q 是 m/tick** | dt=1/sec,Q 是 m/s | 静态模型 dt 不影响 x,实际影响 P,Q 必须按 tick 率调 |
| Tick 模型 | **每收到任一相机 topic 跑一次(含空)** | timer @ 5Hz / 只在非空时跑 | KF 需 predict 让 P 增长;lifecycle 需 tick 累加 time_since |
| Tick debounce | **30ms** | 0ms / 100ms | 把 head/arm 撞车 batch 一次 |
| 时间同步 | **TF lookup at detection.stamp + 动态 odom→base_link TF** | 手写 timestamp 插值 / latest pose | TF2 buffer 原生线性插值,ROS2 标准习语 |
| 关联度规 | 3D 距离 + class 一致 + 两阶段(high+low conf) | Mahalanobis / 3D IoU / ReID | 静态稀疏场景;两阶段处理遮挡 |
| 跨相机合并 | 同 odom 系内 3D 距离门控 0.15m | 各自独立 track / 严格 TF | SLAM 系共享同一未知(arm_base ≡ base_link) |
| 视觉一致性 | 仅 class 一致 | 颜色余弦 / ReID | yoloe 不输出 embedding,ReID 不可靠 |
| 包依赖 | numpy + scipy | filterpy / OpenCV | 减依赖,3 维 KF ~30 行 |
| 输出格式 | 仿 nuScenes + 项目特化(无 velocity) | MOTChallenge 2D-only | 主流 + 3D 能力 |
| BoT-SORT 状态 | **dual_camera_node 内部关闭**(Phase D 后) | 保留 | 本期不动 dual_camera_node |

---

## 12. 文件输出清单

### 新增

```
l4t/lg_walker_ws/src/yoloe_interfaces/msg/UnifiedTrack.msg
l4t/lg_walker_ws/src/yoloe_interfaces/msg/UnifiedTrackArray.msg
l4t/lg_walker_ws/src/lgw_3d_tracker/package.xml
l4t/lg_walker_ws/src/lgw_3d_tracker/setup.py
l4t/lg_walker_ws/src/lgw_3d_tracker/setup.cfg
l4t/lg_walker_ws/src/lgw_3d_tracker/resource/lgw_3d_tracker
l4t/lg_walker_ws/src/lgw_3d_tracker/launch/unified_track_node.launch.py
l4t/lg_walker_ws/src/lgw_3d_tracker/config/runtime.yaml
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/__init__.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/__init__.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/calibration.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/odom_tf_broadcaster.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/kf.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/cost_matrix.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py
l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py
```

### 修改

```
l4t/lg_walker_ws/src/yoloe_interfaces/CMakeLists.txt   # 加 2 个新 msg
```

(其他文件本期不动。)

### 不做的事

- ❌ BT 节点(Phase D)
- ❌ `dual_camera_node` 关闭 BoT-SORT
- ❌ unit test / mock publisher
- ❌ 回滚 `ultralytics/trackers/utils/kalman_filter.py` 5Hz 噪声参数

---

## 13. 风险与缓解

| 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|
| `arm_base ≡ base_link` 假设有 cm 级偏移 | 高(确认存在) | 中 | 视觉伺服/粗抓取不影响;精密抓取时重做标定 |
| SLAM 失锁 → `/utlidar/robot_pose` 跳变 | 中 | 中 | sport_pose_cache 监控 cache_age_ms;> 500ms 跳过 batch |
| `head_calibration.json` 加载失败 | 低 | 高 | 启动期 ERROR 退出,人介入 |
| BoT-SORT 与 unified_track 同时在 | 中 | 中 | 本期不动 dual_camera_node;两路并存到 Phase D |
| TF lookup 失败(head_cam 静态 TF 未发布) | 低 | 高 | 启动期 static TF 已发布,lookup 不应失败 |
| KF P 数值异常 | 低 | 低 | 强制 clip |
| `observation_result` 与 `unified` 两路 BT 同时触发 | 中 | 中 | BT 调用方按目标 ID 选择订阅;本期不动 |

---

## 14. 引用

- 上游设计:`/home/unitree/yoloe_deploy/docs/混合3D跟踪设计.md`
- 本项目内已有约定:
  - `lgw_bt_planner/src/compute_head_target_world.cpp`(head_calibration.json 加载范例,约定 parent=`base_link`)
  - `lgw_bt_planner/src/apply_head_camera_collision_node.cpp`(同样 head_calibration.json 用法)
  - `body_pose_bridge/src/body_pose_bridge_node.cpp:160-161`(`imu_state.rpy` 用法)
- 跨工作区参考:
  - `l4t/nav_ws/dddmr_go2_ws/.superpowers/sdd/2026-10-03-dddmr-mid360/task-4-report.md`(`SportModeState.position` 不可靠,需自己积分;本 spec 改用 SLAM 避开)
  - `l4t/backupL/go2-nav2-amcl-0918-3D/docs/l1_topic_probe_2026-09-18.md`(`/utlidar/robot_pose` 17.8Hz、`/utlidar/robot_odom` 150Hz 真机实测频率)