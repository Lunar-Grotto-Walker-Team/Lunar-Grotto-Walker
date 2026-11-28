# go2_front_camera_driver

宇树 Go2 前视单目相机 ROS2 Humble 驱动。

通过 GStreamer + OpenCV 拉取 UDP 组播 `230.1.1.1:1720` 上的 H.264 RTP 流(15 Hz / 1280×720),
发布对齐 realsense 命名约定的 `sensor_msgs/Image` + `sensor_msgs/CameraInfo` 双话题。

---

## 话题

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/camera/frontcamera/image_raw` | `sensor_msgs/Image` | `BEST_EFFORT, depth=1` | bgr8,1280×720,约 15 Hz |
| `/camera/frontcamera/camera_info` | `sensor_msgs/CameraInfo` | `BEST_EFFORT, depth=1` | 与 image 同帧配对发布 |

启动时可通过参数覆盖:
```bash
ros2 launch go2_front_camera_driver front_camera.launch.py \
  image_topic:=/some/other/topic \
  camera_info_topic:=/some/other/info
```

如果要接 fastlivo2(默认期望 `/camera/camera/color/image_raw`),传:
```bash
ros2 launch go2_front_camera_driver front_camera.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

---

## 构建

在 `self_l4t` 容器内:

```bash
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/ros2_ws/install/setup.bash &&
  apt-get install -y --no-install-recommends \
    ros-humble-camera-info-manager \
    gstreamer1.0-tools gstreamer1.0-libav \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly &&
  colcon build --packages-select go2_front_camera_driver --event-handlers console_direct+
"
```

**注意**:首次构建必须装 `ros-humble-camera-info-manager` + GStreamer 一整套。基础镜像 `ros2:humble-22.04` 都不带。
--event-handlers console_direct+ 作用为：编译输出会直接显示在终端里，方便你实时看到每个包的编译进度、警告和报错。如果不加这个参数，colcon 可能只显示汇总信息，或者把输出缓存后统一显示，调试编译错误时不够直观

---

## 运行

```bash
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 launch go2_front_camera_driver front_camera.launch.py
"
```

launch 默认 `multicast_iface=eth10`(Jetson 到 Go2 的物理网口)。如果你的网络布局不一样,传 `multicast_iface:=<iface>` 覆盖。

## 画面显示（调试用）

```bash
# 容器中
source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash 
gst-launch-1.0 -v udpsrc address=230.1.1.1 port=1720 multicast-iface=eth10 \
  ! application/x-rtp, media=video, encoding-name=H264 \
  ! rtph264depay ! h264parse ! avdec_h264 \
  ! videoconvert ! autovideosink  
```

### FAST-LIVO2 / L2 IMU 参考版本

第二个 launch 把 camera optical frame 挂在 L2 LiDAR/IMU 上,xyz/rpy 用 launch 参数(`tf_x`/`tf_y`/`tf_z`/`tf_roll`/`tf_pitch`/`tf_yaw`)。**当前是 scaffold,占位值需 fastcalib 替换后才能上 SLAM**:

```bash
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 launch go2_front_camera_driver front_camera_fastlivo2.launch.py \
    tf_x:=0.xxx tf_y:=0.xxx tf_z:=0.xxx \
    tf_roll:=-1.5708 tf_pitch:=0.xxx tf_yaw:=-1.5708
"
```

### 一键 alias(推荐)

把"改源码 → build → 启动"做成一行:

```bash
cat >> ~/.bashrc << 'EOF'
alias go2cam='docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash && colcon build --packages-select go2_front_camera_driver --event-handlers console_direct+ >/dev/null 2>&1; ros2 launch go2_front_camera_driver front_camera.launch.py"'
EOF
source ~/.bashrc
```

之后 `go2cam` 就启动,源码改了也自动 build 生效。

### 验证

```bash
docker exec self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 topic hz /camera/frontcamera/image_raw &&
  ros2 topic echo --once /camera/frontcamera/camera_info | head -25
"
```

期望:image_raw 约 14-15 Hz,camera_info 里的 K 矩阵 fx≈829, fy≈830, cx≈632, cy≈364(标定后)。

---

## 内参标定

`config/front_camera_720.yaml` 是 **2026-10-02 实测标定结果**（`camera_calibration` 采集 +
`tools/apply_calibration.py` 转换），不是社区 fork 的估算值：

```
K = [828.971, 0,      631.880,
     0,       829.787, 363.942,
     0,       0,       1      ]
D = [-0.355926, 0.101672, -0.003034, -0.003237, 0]      # plumb_bob
```

历史说明：`backup/go2_ros2_sdk/` 里另有一组 `narrow_stereo` 参数（fx≈864、cx≈639），
来自另一条 stereo 链路，与本次实测相差 fx 4.1%、cx/cy 7–9 px，**不能用于本机**。

原始产物（`/tmp/calibrationdata.tar.gz`、`ost.yaml`）当时没有 `docker cp` 备份，
已随容器重启丢失；这份 YAML 是唯一留存物，K/D/R/P 齐全够用，但重标或换分辨率时
没有原始帧与残差可比对。

完整流程见 [`docs/sensors/front_camera_calibration.md`](../../docs/sensors/front_camera_calibration.md)。

快速参考:
```bash
docker exec self_l4t bash -c "
  apt-get install -y --no-install-recommends ros-humble-camera-calibration python3-yaml
"
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  export DISPLAY=:1 &&
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 run camera_calibration cameracalibrator \
    --size 10x9 --square 0.020 --no-service-check \
    image:=/camera/frontcamera/image_raw
"
```

棋盘规格:11×10 格子(10×9 inner corners),边长 20mm。

GUI 操作详见 SOP。

---

## 目录结构

本包在主仓库 `Lunar-Grotto-Walker` 内：

```
Lunar-Grotto-Walker/
├── src/go2_front_camera_driver/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/front_camera.launch.py            启动 driver + URDF 推导 TF (camera→base_link)
│   ├── launch/front_camera_fastlivo2.launch.py  scaffold:TODO 把 camera TF 挂到 L2 IMU,fastcalib 出结果后再用
│   ├── config/front_camera_720.yaml             相机内参(2026-10-02 标定结果)
│   ├── src/go2_front_camera_driver_node.cpp     H.264 RTP → Image/CameraInfo 节点
│   └── README.md                                本文件
├── tools/apply_calibration.py                   ost.yaml → ROS CameraInfo YAML 转换
└── docs/sensors/front_camera_calibration.md     完整内参标定 SOP
```

历史上曾在独立工作区 `l4t/front_camera_ws/` 开发，该目录的 `setup.sh` 与
`build/ install/ log/` 属于工作区脚手架，没有随包入库。

---

## 已知限制 / TODO

| 事项 | 状态 | 影响 |
|---|---|---|
| **相机 TF** `go2_front_camera_optical_frame → base_link` | URDF 推导 xyz=(0.330, 0, 0.040),rpy=(-π/2, 0, -π/2),含标准 optical 旋转 | URDF 三处一致(`go2.urdf` / `multi_go2.urdf` / `go2_with_realsense.urdf`),无实测标注;SLAM 精度要求高时可用 fastcalib 校准 |
| **内参** | 2026-10-02 已标定(`camera_calibration` + `tools/apply_calibration.py`);原始产物已丢 | 换机/换镜头需重标;重标时没有历史残差可比对 |
| **没有 lidar/imu 时间同步** | `header.stamp` 是解码时刻的 wall-clock | FAST-LIVO2 用 `ApproximateTime` 容错,漂移几毫秒 |
| **`front_camera_fastlivo2.launch.py`** | 新建 scaffold,parent=L2,xyz/rpy 留 fastcalib 接口(占位用 URDF 链式推导) | 当前**勿用于真机 SLAM**,等 fastcalib 出结果替换 6 个 `tf_*` 参数 |
| **FAST-LIVO2 mid360 适配** | 不在本包范围 | 改 `l4t/nav_ws/fastlivo2_ws/` 后续任务 |
| **首次构建依赖** | 需要 apt 装 5 个包 | 已固化到 build 命令;固化到 Dockerfile 是后续任务 |

---

## 关键路径速查

| 文件 | 作用 |
|---|---|
| `src/go2_front_camera_driver/src/go2_front_camera_driver_node.cpp` | 主节点,~110 行 |
| `src/go2_front_camera_driver/launch/front_camera.launch.py` | driver 启动 + URDF 推导 TF (camera→base_link) |
| `src/go2_front_camera_driver/launch/front_camera_fastlivo2.launch.py` | driver + scaffold TF (camera→L2 IMU),fastcalib 出结果前勿用 |
| `src/go2_front_camera_driver/config/front_camera_720.yaml` | 相机内参(标定结果) |
| `tools/apply_calibration.py` | ost.yaml → ROS YAML 转换 |
| `docs/sensors/front_camera_calibration.md` | 完整标定 SOP |

---

## 关键借鉴

| 来源 | 借鉴什么 |
|---|---|
| `l4t/autonomy_stack_go2/.../go2_h264_repub.cpp:15` | GStreamer pipeline 字符串 |
| `backup/go2_ros2_sdk/.../publish_camera_data.py` | Image+CameraInfo 同帧配对发布 |
| `backup/go2_ros2_sdk/.../camera_config.py` | CameraInfo yaml 加载机制 |
| `l4t/nav_ws/fastlivo2_ws/.../config/avia.yaml:4` | image topic 命名约定 |
| `l4t/ros2_ws/src/realsense-ros/.../rs_launch.py` | launch 文件组织风格 |
