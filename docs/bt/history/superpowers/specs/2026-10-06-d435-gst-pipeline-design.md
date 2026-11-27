# D435 GStreamer Pipeline Design

**Status**: Draft (待用户评审)
**Date**: 2026-10-06
**Author**: Claude (brainstorming session)
**Workspace**: `l4t/lg_walker_ws/src/d435_gst_pipeline/`
**Goal**: 在 Jetson Orin NX 8 核 + self_l4t 容器内,用 GStreamer (NVJPEG GPU decode) + 自做 depth alignment 构建 D435 双相机 pipeline,**完整等价替换 realsense-ros**(`two_realsense.launch.py`),让 YOLOE dual_camera_node / rgbd_node / two_stage_node 等下游节点零感知切换。

---

## 1. Background

### 1.1 现存问题

- **realsense-ros CPU bound**:`libjpeg-turbo` MJPEG decode @ 1280×720 ~50ms/帧(单核 burst ~91%),触发 JET Orin NX SoC thermal throttling → 所有核降频 → frame jitter → head 1280×720 stream 死亡(~26s)。
- YOLOE 实际在 host Foxy + GPU TensorRT(38ms/帧 推理,不变),但 pre/post CPU(letterbox)被 realsense-ros 抢占 → jitter ±10ms。
- 详细记录:`docs/2026-10-04-camera-stability-debug.md` Path A/C/D/E。

### 1.2 已实施修复

- Path A/C/D:kernel USB force reset + initial_reset + reconnect_timeout + TimerAction 5s 错峰。
- Path E:两相机降到 640×480×15(缓解 MJPEG decode 占用)。
- 但 head 仍偶发不稳定,YOLOE pre/post CPU 抢占未根本解决。

### 1.3 GStreamer 优势(可行性证据,2026-10-05 验证)

- self_l4t 容器实测:GStreamer 1.20.3 + `libnvjpeg.so` + `nvjpegdec` + `nvvidconv` 全部就绪。
- D435 UVC device `/dev/video0..11` 可见,UVC driver 已 bind(head `1-3.3`,arm `2-1`)。
- CUDA runtime 11.4.298 装好。
- 参考实现:`/home/unitree/my_ws/l4t/front_camera_ws/`(Go2 前视相机,UDP H.264 + cv::VideoCapture + cv_bridge 模式)。

---

## 2. Goal / Non-goals

### 2.1 Goal

1. 单一 ROS 2 包 `d435_gst_pipeline`,节点 `d435_gst_pipeline_node`,双相机实例化(head + arm)。
2. 完整输出 realsense-ros 的 6 个 topic/color + depth + aligned_depth_to_color,各配 camera_info,topic 命名 / 编码 / frame_id / QoS 完全对齐。
3. Color decode 由 CPU libjpeg-turbo 迁移到 GPU NVJPEG(0 ms CPU)。
4. Depth 由 `v4l2src` 直接拉 Z16 raw,绕过 librealsense SDK stream pipeline。
5. Depth alignment 自做(cv::remop),坐标系约定跟 realsense SDK 一致,输出 `aligned_depth_to_color/image_raw`。
6. CameraInfo(K, D, R, P)由节点启动时短暂启 librealsense2 SDK 读 device firmware metadata,缓存 + latched。
7. 双节点启动 TimerAction 5s 错峰(head 立即,arm +5s)。
8. Error handling:USB busy / v4l2 fail / read fail 各有退避重连策略(1→2→4→8→16s,5 次后 fatal)。
9. Shutdown 显式 `release()` 两个 cv::VideoCapture,防止 USB 残留 claim。

### 2.2 Non-goals

1. 不实现 realsense-ros 的所有 advanced feature(temporal filter, hole filling, pointcloud 生成, emitter on/off 动态控制)— YOLOE 不消费这些。
2. 不重新标定 D435 intrinsics/extrinsics — 直接读 SDK device firmware metadata。
3. 不支持硬件同步线(MIPI sync pins)— 我们实际不接 sync 线。
4. 不替换深度对齐为 GPU 实现 — v1 用 naive 自写循环,v2 可选迁 `cv::remop`。
5. 不跨 workspace 集成到 `yoloe_humble_orchestration` — 仅作为独立 pipeline,test script 切。
6. 不动 host yoloe_deploy / DAOs(上游 YOLOE 代码)— 假设它订阅 topic 名不变。

---

## 3. Design Constraints

### 3.1 完整对齐 realsense-ros(用户硬约束)

**"彩色图和深度图参考 realsense 实现,确保能够完整替换对齐"**(用户原话)。

具体含义:

| 维度 | realsense-ros 默认 | gst pipeline 必须等价 |
|---|---|---|
| Color image encoding | `bgr8` | `bgr8` |
| Color frame_id | `<ns>_color_optical_frame` | **同** |
| Depth image encoding | `16UC1` (uint16,mm) | **同** |
| Depth frame_id | `<ns>_depth_optical_frame` | **同** |
| Aligned depth encoding | `16UC1` | **同** |
| Aligned depth frame_id | `<ns>_color_optical_frame`(depth reproject 到 color) | **同** |
| Aligned depth size | 同 color (640×480) | **同** |
| CameraInfo K | `[fx, 0, cx, 0, fy, cy, 0, 0, 1]` | **同**(SDK metadata) |
| CameraInfo D | `[k1, k2, p1, p2, k3]` plumb_bob | **同** |
| CameraInfo R | identity(unrectified)| **同** |
| CameraInfo P | K * [I\|0] | **同** |
| QoS image_raw | BEST_EFFORT, depth=1 | **同** |
| QoS camera_info | RELIABLE, depth=1, TRANSIENT_LOCAL | **同** |
| 帧率 | 15 Hz | **同** |
| timestamp | hardware timestamp | `v4l2src do-timestamp=true` |

### 3.2 互斥模式(避免 USB device 冲突)

- gst pipeline 跑时,**realsense-ros 完全 stop**(librealsense SDK 与 v4l2src 同一 USB device 不能同时 claim)。
- run-based test script:`test_yoloe_stability.sh --pipeline gst|reals`。
- 切换顺序:`stop realsense-ros → 等待 3s(等 USB device release) → start gst`。

### 3.3 YOLOE 业务接口稳定性

YOLOE dual_camera_node / rgbd_node / two_stage_node **不感知切换**:
- topic 命名一致(`/camera_head/color/image_raw` 等)
- message 类型一致(sensor_msgs/Image + sensor_msgs/CameraInfo)
- 帧率一致(15Hz)
- depth scale 一致(`depth_scale_m_per_unit = 0.001`)
- aligned depth alignment 颜色一致(自做算法核对一致)

---

## 4. Architecture

### 4.1 包结构

```
/home/unitree/my_ws/l4t/lg_walker_ws/src/d435_gst_pipeline/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── d435_gst_pipeline_node.cpp  # 单 cpp,实例化为 head/arm
├── launch/
│   └── gst_pipeline.launch.py     # 双相机 launch,启动错峰
├── config/
│   └── d435_gst_pipeline.yaml     # 默认参数(width/height/fps/depth_scale)
├── README.md
└── test/
    └── test_gst_pipeline_smoke.sh # pipeline smoke test
```

### 4.2 节点拓扑

```
d435_gst_pipeline_node (实例化为 d435_gst_pipeline_head / d435_gst_pipeline_arm)
├── [STARTING_SDK] 短暂启 SDK 读 metadata(~1.5s)
│   ├── rs2::pipeline.start() claim D435
│   ├── 读 color/depth intrinsics + extrinsics → 构建 CameraInfo
│   └── rs2::pipeline.stop() 释放 USB
│
└── [RUNNING] 主循环 + align thread
    ├── cv::VideoCapture #1 (color): v4l2src + nvjpegdec + nvvidconv + appsink
    ├── cv::VideoCapture #2 (depth): v4l2src + Z16 + appsink
    ├── 主 timer 70ms → read 两路 → publish
    ├── depth align thread 异步 → publish aligned_depth
    └── camera_info publisher (latched + 30s periodic)
```

### 4.3 互斥模式 vs realsense-ros

| 维度 | realsense-ros | gst pipeline |
|---|---|---|
| 启动入口 | `ros2 launch yoloe_humble_orchestration two_realsense.launch.py`(opt workspace) | `ros2 launch d435_gst_pipeline gst_pipeline.launch.py`(lgw workspace) |
| topic 命名 | `/camera_<name>/color/image_raw` | **完全对称** |
| frame_id | `<name>_color_optical_frame` | **完全对称** |
| 颜色编码 | `bgr8` | **同** |
| QoS | `SensorDataQoS()` | **同** |
| USB device | librealsense SDK claim | v4l2 kernel UVC driver |
| 启动顺序 | head 立即,arm TimerAction 5s | **同** |
| 启动源 | `/opt/yoloe_transfer/humble_ws/`(容器内固化)| `/home/unitree/my_ws/l4t/lg_walker_ws/`(host bind-mount)|

### 4.4 Workspace 启动顺序

```bash
# 场景 A: 跑 realsense-ros pipeline
source /opt/yoloe_transfer/humble_ws/install/setup.bash  # opt workspace
ros2 launch yoloe_humble_orchestration two_realsense.launch.py

# 场景 B: 跑 gst pipeline
source /opt/yoloe_transfer/humble_ws/install/setup.bash  # realsense2_camera_msgs msgs
source /my_ws/l4t/lg_walker_ws/install/setup.bash  # d435_gst_pipeline
ros2 launch d435_gst_pipeline gst_pipeline.launch.py
```

(注:容器内 `/my_ws/l4t/lg_walker_ws/` 对应 host `/home/unitree/my_ws/l4t/lg_walker_ws/`)

---

## 5. Components

### 5.1 launch 参数(`launch/gst_pipeline.launch.py`)

```python
DeclareLaunchArgument("camera_name", default_value="camera_head")
DeclareLaunchArgument("color_width", default_value="640")
DeclareLaunchArgument("color_height", default_value="480")
DeclareLaunchArgument("color_framerate", default_value="15")
DeclareLaunchArgument("depth_width", default_value="640")
DeclareLaunchArgument("depth_height", default_value="480")
DeclareLaunchArgument("depth_framerate", default_value="15")
DeclareLaunchArgument("depth_scale_m_per_unit", default_value="0.001")  # m per depth unit
DeclareLaunchArgument("pipeline_start_delay_sec", default_value="0.0")  # 启动错峰
```

### 5.2 Topic 命名 + frame_id(完全对齐 realsense-ros)

| 字段 | 命名约定 | 实例(head) |
|---|---|---|
| namespace | `camera_<name>` | `camera_head` |
| color image topic | `<ns>/color/image_raw` | `camera_head/color/image_raw` |
| depth image topic | `<ns>/depth/image_rect_raw` | `camera_head/depth/image_rect_raw` |
| aligned image topic | `<ns>/aligned_depth_to_color/image_raw` | `camera_head/aligned_depth_to_color/image_raw` |
| color camera_info | `<ns>/color/camera_info` | `camera_head/color/camera_info` |
| depth camera_info | `<ns>/depth/camera_info` | `camera_head/depth/camera_info` |
| aligned camera_info | `<ns>/aligned_depth_to_color/camera_info` | `camera_head/aligned_depth_to_color/camera_info` |
| color frame_id | `<ns>_color_optical_frame` | `camera_head_color_optical_frame` |
| depth frame_id | `<ns>_depth_optical_frame` | `camera_head_depth_optical_frame` |
| aligned frame_id | `<ns>_color_optical_frame` | `camera_head_color_optical_frame`(同 color)|

### 5.3 QoS Profile

| Topic | QoS | 理由 |
|---|---|---|
| `*/image_raw` | `SensorDataQoS()` (BEST_EFFORT, depth=1) | 大消息(2.76 MB @ 640×480 bgr8),丢一帧可接受 |
| `*/camera_info` | `RELIABLE, depth=1, TRANSIENT_LOCAL` | 静态 metadata,latched 让 YOLOE 启动时拿到 |
| `*/aligned_depth_to_color/image_raw` | `SensorDataQoS()` | 同 image_raw |

### 5.4 CameraInfo publishing 策略

- 启动后 **latched 一次**(确保 YOLOE 启动时能拿到)
- 之后每 **30s publish 一次**(TRANSIENT_LOCAL,replaces existing)
- 内容从 SDK 启动时一次性 cache,不动态更新

### 5.5 双相机启动错峰

```python
Node(package="d435_gst_pipeline", executable="d435_gst_pipeline_node",
     name="d435_gst_pipeline_head",
     parameters=[{"camera_name": "head", ...}])  # T+0

TimerAction(period=5.0, actions=[
  Node(package="d435_gst_pipeline", executable="d435_gst_pipeline_node",
       name="d435_gst_pipeline_arm",
       parameters=[{"camera_name": "arm", ...}])  # T+5s
])
```

### 5.6 package.xml

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>OpenCV</depend>
<depend>gstreamer-1.0</depend>
<depend>librealsense2</depend>  <!-- 仅启动时调一次 -->
```

### 5.7 CMakeLists.txt 关键

```cmake
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)
target_link_libraries(d435_gst_pipeline_node
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${REALSENSE2_LIBRARIES}
)
```

---

## 6. Data Flow

### 6.1 节点状态机

```
[INIT]
  │ 创建 ROS publisher / subscriber / timer
  ▼
[STARTING_SDK] (T=0..+1.5s)
  │ rs2::pipeline.start() claim D435 device
  │ 读 color_intrinsics, depth_intrinsics, extrinsics
  │ 构建 CameraInfo 缓存 + latched publish
  │ rs2::pipeline.stop() 释放 USB
  ▼
[STARTING_V4L2] (T=+1.5..+2s)
  │ cv::VideoCapture #1 (color) 创建 + 1 帧预热 read
  │ cv::VideoCapture #2 (depth) 创建 + 1 帧预热 read
  ▼
[RUNNING] (T=+2s onwards)
  │ 主 timer 70ms 触发 → read 两路 → publish
  │ depth align thread 异步跑(reproject → publish aligned_depth)
  ▼
[SHUTDOWN] (ROS shutdown signal)
  │ timer.cancel()
  │ depth align thread.join()
  │ cv::VideoCapture.release() × 2
```

### 6.2 启动序列时序图

```
T=0.0  节点启动,创建 6 个 publisher
T=0.3  rs2::context discovery 找 D435 device
T=0.5  rs2::pipeline.start() ← claim D435 device
T=0.7  profile.get_stream(COLOR).get_intrinsics() → cache
T=0.9  profile.get_stream(DEPTH).get_intrinsics() → cache
T=1.1  profile.get_extrinsics_to() → cache (R, t)
T=1.3  3 个 camera_info build + latched publish
T=1.5  rs2::pipeline.stop() ← 释放 USB
T=1.6  cv::VideoCapture #1 (color) 创建 + 1 帧预热
T=1.8  cv::VideoCapture #2 (depth) 创建 + 1 帧预热
T=2.0  启动 timer 70ms → [RUNNING]
```

**关键不变量**:SDK 与 v4l2src **互斥持 device**(SDK 1.5s → v4l2 永久)。

### 6.3 主循环数据流

```
┌────────────────────────────────────────────────────────────┐
│ 主 timer 70ms (~14.3Hz)                                      │
├────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─── GStreamer #1 (color) ────────────────────────────┐  │
│  │ v4l2src → nvjpegdec → nvvidconv → appsink            │  │
│  │   ↓                                                  │  │
│  │ cv::VideoCapture.read() → cv::Mat BGR (640×480)      │  │
│  │   ↓                                                  │  │
│  │ cv_bridge::CvImage → sensor_msgs/Image               │  │
│  │   ↓                                                  │  │
│  │ publish /<ns>/color/image_raw                         │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── GStreamer #2 (depth) ────────────────────────────┐  │
│  │ v4l2src → appsink (Z16 raw, 无解码)                   │  │
│  │   ↓                                                  │  │
│  │ cv::VideoCapture.read() → cv::Mat mono16              │  │
│  │   ↓                                                  │  │
│  │ cv_bridge::CvImage → sensor_msgs/Image (16UC1)        │  │
│  │   ↓                                                  │  │
│  │ publish /<ns>/depth/image_rect_raw                    │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌─── depth align thread (异步) ───────────────────────┐  │
│  │ subscribe color + depth (message_filters)            │  │
│  │   ↓                                                  │  │
│  │ reproject depth → color pixel grid                    │  │
│  │   ↓                                                  │  │
│  │ publish /<ns>/aligned_depth_to_color/image_raw        │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
└────────────────────────────────────────────────────────────┘
```

### 6.4 Backpressure

| 层 | 策略 |
|---|---|
| GStreamer appsink | `drop=1 max-buffers=1`(只缓存 1 帧)|
| ROS QoS | `BEST_EFFORT, depth=1`(满 queue 丢旧)|
| 主 timer 70ms | ~14.3Hz(略低于 15Hz source,给 producer 余量)|

### 6.5 Shutdown 序列

```
ROS shutdown signal
  ↓
timer.cancel() → 停主循环
  ↓
depth align thread.join()
  ↓
cv::VideoCapture #1.release() → 释放 color USB
cv::VideoCapture #2.release() → 释放 depth USB
  ↓
DDS publisher 清空 queue
  ↓
节点退出
```

**关键**:shutdown **必须** `release()` 两个 cv::VideoCapture,否则 USB 设备残留 claim(下次启动 BUSY) — 这正是 debug doc Path A 解决的问题。

---

## 7. Reference: realsense SDK Depth Alignment(用户硬约束)

**用户约束**: "彩色图和深度图参考 realsense 实现,确保能够完整替换对齐"

### 7.1 realsense SDK 默认行为

realsense-ros 通过 `align_depth_to_color` 节点(setting `align_depth: true`)做深度对齐:
- 输入:`depth_frame_infra_raw` (16UC1,depth sensor frame)
- 输入:`color_frame` (bgr8,color sensor frame)
- 输入:SDK metadata intrinsics/extrinsics
- 输出:`aligned_depth_to_color/image_raw` (16UC1,color frame size)

算法(SDK 内部,`librealsense2/align.cpp`):
```
for each depth pixel (u_d, v_d, z):
  # depth pixel → depth 3D point(depth optical frame)
  P_d = K_d^-1 × [u_d, v_d, 1]^T × z × depth_scale
  
  # depth → color transform(extrinsics)
  P_c = R × P_d + t
  
  # project to color image(若 P_c.z > 0)
  if P_c.z > 0:
    [u_c, v_c, _] = K_c × P_c
    u_c /= P_c.z; v_c /= P_c.z
    
    # 在 aligned_depth[v_c, u_c] 处写 z(align_units 与原 z 单位一致)
    if 0 <= u_c < color_w and 0 <= v_c < color_h:
      aligned_depth[v_c, u_c] = z  # uint16,与输入同单位(默认 mm)
```

### 7.2 gst pipeline alignment 算法(等价的 SDK 实现)

```cpp
// 缓存(节点启动时 SDK 读)
struct CameraMetadata {
  cv::Mat K_color, K_depth;            // 3x3 intrinsics
  cv::Mat K_color_inv, K_depth_inv;     // 预计算 inverse
  cv::Mat extrinsics_R;                 // 3x3 depth → color rotation
  cv::Vec3d extrinsics_t;               // 3x1 depth → color translation (m)
  std::vector<double> D_color, D_depth; // k1...k5 distortion coefficients
  double depth_scale_m;                 // m per depth unit (默认 0.001)
};

// align thread 处理函数
void AlignDepth(const cv::Mat& depth, const CameraMetadata& meta,
                cv::Mat& aligned_depth) {
  aligned_depth = cv::Mat::zeros(meta.color_height, meta.color_width, CV_16UC1);
  
  const double* K_d_inv = meta.K_depth_inv.ptr<double>();
  const double* R = meta.extrinsics_R.ptr<double>();
  const double* t = &meta.extrinsics_t[0];
  const double* K_c = meta.K_color.ptr<double>();
  const double scale = meta.depth_scale_m;
  
  const uint16_t* depth_data = depth.ptr<uint16_t>();
  
  for (int v_d = 0; v_d < depth.rows; v_d++) {
    for (int u_d = 0; u_d < depth.cols; u_d++) {
      uint16_t z_units = depth_data[v_d * depth.cols + u_d];
      if (z_units == 0) continue;  // invalid pixel
      
      double z_m = z_units * scale;
      
      // K_d^-1 × [u_d, v_d, 1]^T
      double x_d = (K_d_inv[0] * u_d + K_d_inv[1] * v_d + K_d_inv[2]) * z_m;
      double y_d = (K_d_inv[3] * u_d + K_d_inv[4] * v_d + K_d_inv[5]) * z_m;
      double z_d = (K_d_inv[6] * u_d + K_d_inv[7] * v_d + K_d_inv[8]) * z_m;
      
      // R × P_d + t
      double x_c = R[0] * x_d + R[1] * y_d + R[2] * z_d + t[0];
      double y_c = R[3] * x_d + R[4] * y_d + R[5] * z_d + t[1];
      double z_c = R[6] * x_d + R[7] * y_d + R[8] * z_d + t[2];
      
      if (z_c <= 0) continue;
      
      // K_c × P_c / z_c
      double u_c = (K_c[0] * x_c + K_c[1] * y_c + K_c[2] * z_c) / z_c;
      double v_c = (K_c[3] * x_c + K_c[4] * y_c + K_c[5] * z_c) / z_c;
      
      int u_c_int = (int)std::round(u_c);
      int v_c_int = (int)std::round(v_c);
      
      if (u_c_int >= 0 && u_c_int < aligned_depth.cols &&
          v_c_int >= 0 && v_c_int < aligned_depth.rows) {
        aligned_depth.at<uint16_t>(v_c_int, u_c_int) = z_units;
      }
    }
  }
}
```

**性能**(per frame):
- 640×480 = 307,200 像素
- per pixel: ~15 ops(矩阵向量乘)
- 总: ~4.6M ops
- Orin NX 单核: ~5-10 ms(naive)

**v1 实施**,**v2 可选迁 `cv::remop`**(SIMD + 预计算 lookup, ~3-5 ms)。

### 7.3 CameraInfo 等价字段

realsense-ros 输出的 `camera_info` 字段映射:

| 字段 | 值 | 来源 |
|---|---|---|
| `header.frame_id` | `"camera_<name>_color_optical_frame"`(或 `_depth_optical_frame`)| 命名约定 |
| `width` | 640 | launch 参数 |
| `height` | 480 | launch 参数 |
| `distortion_model` | `"plumb_bob"` | SDK metadata |
| `D` (5 doubles) | `[k1, k2, p1, p2, k3]` | SDK metadata |
| `K` (9 doubles,row-major) | `[fx, 0, cx, 0, fy, cy, 0, 0, 1]` | SDK metadata |
| `R` (9 doubles,row-major) | `[1,0,0,0,1,0,0,0,1]` (identity,SDK 无 realsence)| hardcoded |
| `P` (12 doubles,row-major) | `[fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]` (K * [I\|0])| computed from K |

aligned_depth camera_info 的 K 用 color 的 K(因为 aligned depth 在 color frame grid 上)。

---

## 8. Error Handling + Recovery

### 8.1 错误表

| 错误 | 处理 |
|---|---|
| `rs2::pipeline.start()` fail(USB busy)| log error + 退避重试(1,2,4,8,16s),5 次后 ROS 节点 fatal |
| `cv::VideoCapture.open()` fail | log error + 同上退避重试 |
| `cap.read()` 返回 false(USB 断开)| log error + release + 重新 open(节点内重连)|
| NVJPEG 解码 fail(JPEG 损坏)| log warning + 跳过当前帧,下一帧继续 |
| aligned_depth 算法退化(camera_info 无效)| log error + 不 publish aligned_depth,color/depth 继续 |
| cv_bridge 异常(mat 类型错)| log error + skip frame |
| `depth_value` overflow 或 `P_c.z <= 0` | 跳过该像素(算法本身容错)|

### 8.2 重连策略

```cpp
constexpr int kMaxReconnectAttempts = 5;
int attempt = 0;
std::chrono::seconds backoff(1);

while (rclcpp::ok()) {
  if (TryInit(...)) break;  // 成功
  attempt++;
  if (attempt >= kMaxReconnectAttempts) {
    RCLCPP_FATAL(get_logger(), "Max reconnect attempts reached, exiting");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_WARN(get_logger(), "Init failed (attempt %d/%d), retrying in %lds",
              attempt, kMaxReconnectAttempts, backoff.count());
  std::this_thread::sleep_for(backoff);
  backoff = std::min(backoff * 2, std::chrono::seconds(16));
}
```

### 8.3 Shutdown 干净退出

`release()` 两个 cv::VideoCapture **必须** 调用,否则下次启动 BUSY — 这是 debug doc Path A 已踩过的坑。

---

## 9. Testing Strategy

### 9.1 Unit Test(集成到 build,可选)

- `test_pipeline_smoke.sh`:
  - 启动 gst_pipeline launch(head + arm)
  - 验证 6 个 topic 存在(`/camera_head/color/image_raw` 等)
  - 验证 camera_info latched(YOLOE 启动时能收到)
  - 验证第一帧 timestamp < T+5s
  - 验证 frame rate > 10 Hz(持续 30s)
- `test_depth_alignment.cpp`(单元):
  - 模拟 K + extrinsics + 合成 depth 输入,验证 aligned 输出坐标 vs z 单位(跟 realsense SDK 算法比对)

### 9.2 集成测试

- 启动 gst pipeline + YOLOE host Foxy dual_camera_node / rgbd_node / two_stage_node:
  - 验证 YOLOE 推理能跑通(从 result topic 看)
  - 验证 two_stage_node 能输出 3D detection
  - 验证跟 realsense-ros 切换时 YOLOE 不感知

### 9.3 真机回归(对比 realsense-ros)

- 同一光照同一场景,分别跑 realsense-ros 和 gst pipeline:
  - **YOLOE 检测 mAP/recall 对比**(应该一致,允许 ±1 LSB JPEG decode 差异)
  - **two_stage_node 3D 位置估计对比**(aligned depth alignment 精度对比)
  - **stream stability**:跑 30 分钟无 missing
  - **CPU/GPU 占用**:跑长稳,验证 gst pipeline CPU 占用 < realsense-ros

### 9.4 关键验证点

| 检查项 | 命令 | 期望 |
|---|---|---|
| 两相机都起来 | `ros2 node list` | `d435_gst_pipeline_head` + `d435_gst_pipeline_arm` |
| 6 topic 都在 | `ros2 topic list` | 见 §5.2 |
| CameraInfo latched | `ros2 topic echo /camera_head/color/camera_info --qos-durability transient_local` | 有输出 |
| 帧率 | `ros2 topic hz /camera_head/color/image_raw` | ~15 Hz |
| aligned_depth frame_id | `ros2 topic echo /camera_head/aligned_depth_to_color/image_raw --field header.frame_id` | `camera_head_color_optical_frame`(同 color)|
| aligned_depth encoding | `... --field encoding` | `16UC1` |
| aligned_depth size | 验证 width/color:width == aligned:width | 一致 |

---

## 10. Performance Budget(预计)

**单相机 @ 640×480×15**:

| 环节 | CPU 开销 | GPU 开销 |
|---|---|---|
| Color decode (NVJPEG) | ~0 ms | ~3-5 ms |
| Color nvvidconv | ~0 ms | ~1-2 ms |
| cv::VideoCapture wrap + cv_bridge | ~3-4 ms | 0 |
| depth cv::VideoCapture wrap + cv_bridge | ~3-4 ms | 0 |
| depth align (naive 自写) | ~5-10 ms | 0 |
| aligned_depth cv_bridge | ~3-4 ms | 0 |
| ROS publish (DDS,小到大)| ~2-3 ms | 0 |
| **单帧合计** | **~17-33 ms** | **~5-7 ms** |

15fps 预算 = 66 ms,**余量 33-49 ms / 帧**。

双相机 × 2:
- CPU 总消耗:34-66 ms / frame
- 单核占比:51-100%(8 核 Orin NX 平均 6.5-12.5%)
- GPU 占用:~10%

**对比 realsense-ros**:
- CPU 单核 ~91% → 51-100%(差不多;但 gst 在 thermal headroom 上好)
- GPU 0% → ~10%(NVJPEG 占用)
- frame jitter ±10ms → ±3ms

---

## 11. Risks + Open Questions

### 11.1 已知风险

| 风险 | 缓解 |
|---|---|
| 节点启动 +1.5s SDK claim 延迟 | 一次性,可接受;后续节点复用 cache |
| `cv::VideoCapture` 在 Jetson 上版本兼容(GStreamer 1.20)| 已验证容器内 1.20.3,可用 |
| `librealsense2` SDK 短暂 claim + v4l2 claim 时序必须正确(否则 BUSY)| 启动序列严格:SDK 1.5s → release → v4l2 |
| Depth align naive 5-10ms vs realsense SDK | 性能等价 vs 许可,后续可迁 `cv::remop` |
| Shutdown USB 残留 claim | 显式 `release()` × 2 |
| 两个 v4l2src 同时 claim 同一 D435 多 subdevice | D435 实际 color / depth 不同 /dev/videoN,不冲突 |
| 真机 temperature / power throttle 仍可能触发 | gst pipeline CPU 占用低于 realsense-ros,throttle headroom 充足 |

### 11.2 开放问题

- **depth align 算法是否跟 SDK byte-for-byte 完全等价?**:naive 自写 vs SDK SIMD 优化版本,在边界(NaN, Inf, z=0)行为可能略有差异。真机回归测试验证。
- **camera_info 在 launch 后启动 YOLOE 时是否 latched**:YOLOE 启动时可能晚于 gst pipeline 几秒,需要 TRANSIENT_LOCAL QoS 已确认。
- **aligned_depth 频率能否跟上 15Hz**:naive 5-10ms × 15Hz = 75-150ms CPU/秒 = 12-25% 单核,可撑。
- **运行时切 pipeline 时 DDS 状态**:realsense-ros 退出后 → gst 启动,YOLOE 端订阅 topic 端会自动重订阅,需要 TRANSIENT_LOCAL QoS 不变。
- **head 1280×720 死亡问题**:本次设计仍是 640×480(head 也降),如果用户后续要升 1280×720,需要再迁测试。

---

## 12. Decision Log

| 决策 | 选择 | 理由 |
|---|---|---|
| 共存模式 | **互斥模式** | USB device 不能并发 claim |
| 包位置 | **lgw workspace** (`l4t/lg_walker_ws/src/d435_gst_pipeline/`)| host bind-mount + CLAUDE.md 工作流 + 跨 workspace 联动 |
| 范围 | **双相机一次到位** | 实现成本 ≈ 单相机 + 20%,一次到位 |
| CameraInfo | **必须输出**(反转原"不输出"决策)| YOLOE rgbd_node 实际订阅 camera_info + depth alignment 需 intrinsics |
| GStreamer 接入方式 | **方案 A: cv::VideoCapture + cv_bridge**(同 front_camera_ws)| ~150 行,5-10% CPU 开销可忽略 |
| depth align | **自做**(naive v1, cv::remop v2 可选)| 完整替代 SDK 算法,业务可接受 |
| Intrinsics / extrinsics 来源 | **节点启动短暂启 SDK 读 device metadata**(原本 poly)| device 实测,精度高,无 yaml 依赖 |
| Depth scale | launch 参数 **0.001 m/unit**(可覆盖)| realsense 默认 |
| 节点命名 | **d435_gst_pipeline_node** | 避开"color" / "realsense" 关键字 |
| Aligned depth encoding | **16UC1**(同 realsense-ros)| 完全对齐 |
| Aligned depth frame_id | **`<ns>_color_optical_frame`**(depth reproject 到 color) | 完全对齐 |
| 启动错峰 | **TimerAction 5s**(head 立即,arm +5s) | 沿用 realsense-ros 模式 |

---

## 13. References

- **当前 debug doc**:`docs/2026-10-04-camera-stability-debug.md` Path A/C/D/E + Appendix A.1
- **CLAUDE.md**:workspace 根目录/工作流 / 容器映射
- **front_camera_ws reference**:`l4t/front_camera_ws/src/go2_front_camera_driver/`(架构骨架 + cv::VideoCapture + cv_bridge pattern)
- **realsense SDK depth alignment**: `librealsense2/align.cpp`(参考 SDK align 算法)
- **现有 calibration**:`l4t/result/`(只含 hand-eye extrinsics,不含 D435 intrinsics — 由 gst pipeline 启动时 SDK 读)
- **YOLOE pipeline 实际订阅**:
  - `yoloe_deploy/ros2_ws/src/yoloe_perception/yoloe_perception/rgbd_node.py`(color_topic + depth_topic + camera_info_topic)
  - `yoloe_deploy/ros2_ws/src/yoloe_perception/yoloe_perception/two_stage_node.py`(stage1/2_color_sub + depth_sub + camera_info_sub)
  - `yoloe_deploy/ros2_ws/src/yoloe_perception/yoloe_perception/dual_camera_node.py`(stage1_latest_depth + stage2_latest_depth)

---

**Review request**: 请用户确认 §3 (设计约束) "完整对齐 realsense-ros" 是否覆盖完整,然后进入 writing-plans skill 出实施计划。