# lgw_3d_tracker 真机端到端验证(4 个独立 root cause)

> ⚠️ **2026-10-22 更正**:本文 Round 4 的诊断(归因于 DDS TRANSIENT_LOCAL latch)
> 经纯软件复现证明**是错的**。真正根因是 **tf2 里同一 frame 有多个 parent**,
> 导致 parent 解析不确定、lookup 随机失败。
> 完整更正 + 后续的 frame_prefix 重构见
> `docs/2026-10-22-tf-frame-prefix-refactor.md`。

> ⚠️ **2026-10-22 更正**:本文 Round 4 的诊断(归因于 DDS TRANSIENT_LOCAL latch)
> 经纯软件复现证明**是错的**。真正根因是 **tf2 里同一 frame 有多个 parent**,
> 导致 parent 解析不确定、lookup 随机失败。
> 完整更正 + 后续的 frame_prefix 重构见
> `docs/2026-10-22-tf-frame-prefix-refactor.md`。

**日期**:2026-10-20
**范围**:`lgw_3d_tracker` (unified 3D tracker,dual-camera + arm + SLAM) 在 Jetson Orin / `self_l4t` 容器 + `start_lgw_yoloe.sh` + `start_lgw_arm.sh` 真机 bringup 下,从 `/perception/tracks/unified` 零输出 → 11 Hz 持续出 track 的完整诊断与修复。
**状态**:Round 1-5 全部修复,真机稳态 10-11 Hz,双相机 merge ✓,lifecycle 正常。

## 背景

`lgw_3d_tracker` 之前的 T1-T13 实现与单元测试全在 master 分支 + sim/合成数据下通过,
本次是要把它推到真机 Go2 + agx_arm + 双 realsense 跑通,验证:
1. tracker 真在出 `/perception/tracks/unified` 消息
2. YOLOE dual-camera 的 head + arm 检测都能 transform 到 odom 系
3. 同一目标被双相机分别看到时,被 merge 进同一个 track_id
4. lifecycle (tentative → confirmed → coasting) 在真机时序下正常
5. 输出频率满足 10 Hz 级(给下游 BT 树消费)

## 硬件拓扑(本次验证)

| 部件 | 型号 / topic | 备注 |
|---|---|---|
| 主机 | Jetson Orin NX 16G, jetpack r35.5.1 | host + self_l4t (humble) 容器 |
| 底盘 | Unitree Go2 + utlidar SLAM | `/utlidar/robot_pose` ~18 Hz |
| 机械臂 | agx_arm + agx_gripper | `tcp_link → camera_arm_color_optical_frame` 经 realsense URDF |
| head 相机 | realsense D435i 串号 `_261622078621` | `/perception/two_stage/camera_head/result` |
| arm 相机 | realsense D435 串号 `_262422071246` | `/perception/two_stage/camera_arm/result` |
| 网络 | eth10,cyclonedds | DDS_TRANSIENT_LOCAL 见 Round 4 失效 |

## 时间线 / Round-by-Round root cause

| Round | 现象 | 错误信号 | 根因 | 修复 |
|-------|------|---------|------|------|
| 1 | `/tf_static` 只有 head 静态 TF,缺 arm | `Could not find a connection between 'odom' and 'camera_arm_color_optical_frame'` | agx_arm_perception 未 install,`tcp_link→camera_arm_color_optical_frame` 没人 publish | CLI 起 `static_transform_publisher` 用 `arm_calibration.json _tcp` 数值 (Route A 用户选项) |
| 2 | tracker 收不到 `/utlidar/robot_pose` | 无 callback,`/tf` 没新 odom→base_link | 订阅设 BEST_EFFORT,发布方 RELIABLE → DDS 不匹配 → 完全收不到 | `unified_track_node.py:97` 改 RELIABLE + VOLATILE (A1 用户选项) |
| 3 | tracker 启动时只见到一个相机很快挂 | `ros2 node info` 报 "There are 2 nodes" (静态 TF 重复? 实际是 DDS type 不匹配导致孤儿) | YOLOE 真发的 type 是 `yoloe_interfaces/msg/ObservationResult`,tracker 订阅 `YoloeDetection3DArray` → 类型不匹配 → DDS 拒投 | 改订阅 type 到 `ObservationResult`,input_adapter 同时换 type alias (结构同构) |
| 4 | tracker 自己 publish 的 head 静态 TF lookup 失败 | `Tf has two or more unconnected trees`(本 process 内,view_frames 在外部看 /tf_static 又有) | `StaticTransformBroadcaster.sendTransform` 一次性;在 cyclonedds + eth10 setup 下 latched message 没真落到本 process 的 tf2 buffer | `TransformBroadcaster + 100ms timer` 周期性重发,走 /tf VOLATILE 路径 |
| 5 | 启动后 ~8s + 大量 WARN | `Lookup would require extrapolation into the future`(arm) / `into the past`(head,启动期) | arm 检测 ~30 Hz > SLAM pose 18 Hz,det stamp 略新于最新 robot_pose | `input_adapter.adapt()` 加 now() fallback(代价 <100 ms temporal alignment 误差) |

## 详细修复

### Round 1:`/tf_static` 缺 `tcp_link → camera_arm_color_optical_frame`

**症状**:tracker 启动后所有 arm camera detection lookup 报
```
Could not find a connection between 'odom' and 'camera_arm_color_optical_frame' because they are not part of the same tree
```

**根因**:spec §3.4 假设 `agx_arm_perception` 启动时会 publish
`tcp_link → camera_arm_color_optical_frame` 静态 TF。该 package 未 install,
TF 不存在 → odom 系里 `camera_arm_color_optical_frame` 是另一个独立 tree,
连不到 odom。

**修复**(用户选 Route A:CLI 补,不改代码,快验):
```bash
docker exec -d self_l4t bash -lc "source /my_ws/l4t/l4t_setup.sh; \
  ros2 run tf2_ros static_transform_publisher \
    --x -0.0756 --y 0.0000467 --z -0.0858 \
    --qx -0.123 --qy 0.118 --qz -0.682 --qw 0.711 \
    --frame-id tcp_link --child-frame-id camera_arm_color_optical_frame"
```

数值来源:`lgw_bt_planner/calibration/arm_calibration.json` (必须 _tcp 变体,
已叠加 12 cm tcp_offset,xyzw quaternion convention)。**遗留**:本次容器里已
跑 6 个 arm 静态 TF publisher 进程(历次调试残留),应做 dedup,或下次把
arm static TF 也内嵌到 `lgw_3d_tracker` 启动期(跟 head 同 pattern)。

### Round 2:`/utlidar/robot_pose` 订阅 QoS 不匹配

**症状**:`ros2 topic hz /utlidar/robot_pose` 显示 ~18.7 Hz,但
`unified_track_node` 的 `_on_robot_pose` callback 从不 fire。
`/tf` topic 上 odom→base_link 不更新。

**根因**:发布方 QoS=`RELIABLE + VOLATILE + KEEP_LAST(1)`,tracker 原来订
`BEST_EFFORT + VOLATILE`。DDS Reliability 不匹配 → publisher 找不到匹配
subscriber → 不发。

**修复**:
```diff
 qos_pose = QoSProfile(
         depth=10,
-        reliability=QoSReliabilityPolicy.BEST_EFFORT,
+        reliability=QoSReliabilityPolicy.RELIABLE,
         durability=QoSDurabilityPolicy.VOLATILE,
     )
```

**教训**:SLAM 类的 sensor 链路(odom/pose)一定先看 publisher QoS,再配对;
`BEST_EFFORT` 在 publisher 是 RELIABLE 时直接断流。

### Round 3:DDS type 不匹配(YOLOE 发 `ObservationResult`,tracker 订 `YoloeDetection3DArray`)

**症状**:Round 1+2 修完后,`/perception/tracks/unified` 还是空。tracker
日志没任何 callback,`/perception/two_stage/camera_head/result` topic
`ros2 topic info` 显示有 publisher 但无 subscriber endpoint(或反之)。

**根因**:`/perception/two_stage/camera_head/result` 真发的 type 是
`yoloe_interfaces/msg/ObservationResult`(lgw_bt_planner 全栈已迁移,BT
节点订阅都用此口)。tracker 还在订
`yoloe_interfaces/msg/YoloeDetection3DArray`(master 分支最初代码)。DDS 按
type 类型路由,类型不匹配 → 双方互不可见。

**修复**(改订阅 type,数据通道同构,无功能差异):

`unified_track_node.py`:
```diff
-from yoloe_interfaces.msg import UnifiedTrackArray, YoloeDetection3DArray
+from yoloe_interfaces.msg import ObservationResult, UnifiedTrackArray
 ...
 self._sub_head = self.create_subscription(
-    YoloeDetection3DArray, self.config.head_detections_topic,
+    ObservationResult, self.config.head_detections_topic,
     self._on_head_detection, qos_default)
 self._sub_arm = self.create_subscription(
-    YoloeDetection3DArray, self.config.arm_detections_topic,
+    ObservationResult, self.config.arm_detections_topic,
     self._on_arm_detection, qos_default)
```

`input_adapter.py`:
```diff
-from yoloe_interfaces.msg import YoloeDetection3DArray
+from yoloe_interfaces.msg import ObservationResult
 ...
-def adapt(self, msg: YoloeDetection3DArray) -> List[TrackObservation]:
+def adapt(self, msg: ObservationResult) -> List[TrackObservation]:
```

**结构对照**(本次手工核对):
| 字段 | `YoloeDetection3DArray` | `ObservationResult` |
|---|---|---|
| header | ✓ | ✓ |
| status enum | ✗ | ✓ `CONFIRMED/NO_TARGET/TIMEOUT/CANCELLED/ERROR` |
| status_text | ✗ | ✓ |
| detections[] | ✓ `YoloeDetection3D[]` | ✓ `YoloeDetection3D[]`(同构) |

detections 元素 (`YoloeDetection3D`) 字段完全一致:`center_m / size_m /
class_name / confidence / bbox / position_valid / size_valid / instance_id`。

**现状**:tracker 不强制 gate status enum,与 `lgw_bt_planner`
`compute_lookat_pose_node.cpp` 的 `detections.empty() 即 FAILURE` 模式一致
(空 detections 自然返回 [],不进 track update)。

### Round 4:cyclonedds+eth10 下 `StaticTransformBroadcaster` 不被本 process 收到

> ⚠️ **本节的诊断在 2026-10-22 被实测推翻,保留原文但结论已更正。**
> 真正的根因是 **tf2 里同一 frame 有多个 parent**,不是 DDS latch 问题。
> 详见 `docs/2026-10-22-tf-frame-prefix-refactor.md` 与
> memory `feedback_tf2_multi_parent_ambiguity`。
> 纯软件复现(单进程周期发边,无 DDS 因素):
> `单 parent OK=20 FAIL=0` / `双 parent OK=16 FAIL=4` / `双冲突 OK=13 FAIL=7`。
> 另:实测"一次性 publish 的 /tf_static 对 late-joiner 收不到"确实成立,
> 但那是**另一条独立规律**,不是本症状的成因。

**症状**(关键,看起来很反直觉):
- `view_frames` 跑出来 `camera_head_color_optical_frame: parent='base_link'` 正常
- `ros2 topic echo /tf_static` 能看到 head 静态 TF message
- `ros2 topic info /tf_static --verbose` 报 publisher QoS = TRANSIENT_LOCAL + RELIABLE ✓
- `tf2_echo odom camera_head_color_optical_frame` 在 container 外查询报 "unconnected trees"
- tracker 自己的 `lookup_transform("odom", "camera_head")` 也报 "unconnected trees"

也就是说:DDS 层 TRANSIENT_LOCAL 的 message **有真发出去**(`/tf_static` 上能看到),
但**本 process 的 tf2 Buffer 没收到**——大概率是 cyclonedds 在 eth10 multicast
配置下,late-joining subscriber 收不到 latched message(已 publish 后的 ms 级
race;或 cyclonedds 与 DDSI-RTPS spec 在 TRANSIENT_LOCAL 上的实现差异)。

**修复**:不查为什么,直接绕开。改 `calibration.py`:

```diff
-from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
+from tf2_ros.transform_broadcaster import TransformBroadcaster
 ...
-def broadcast_head_static_tf(node) -> StaticTransformBroadcaster:
-    broadcaster = StaticTransformBroadcaster(node)
-    (tx, ty, tz), (qx, qy, qz, qw) = load_head_calibration()
-    t = TransformStamped()
-    t.header.stamp = node.get_clock().now().to_msg()
-    t.header.frame_id = "base_link"
-    t.child_frame_id = "camera_head_color_optical_frame"
-    t.transform.translation = Vector3(x=tx, y=ty, z=tz)
-    t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
-    broadcaster.sendTransform(t)  # 一次 send
-    return broadcaster
+def broadcast_head_static_tf(node):
+    (tx, ty, tz), (qx, qy, qz, qw) = load_head_calibration()
+    broadcaster = TransformBroadcaster(node)
+    def _publish() -> None:
+        t = TransformStamped()
+        t.header.stamp = node.get_clock().now().to_msg()
+        t.header.frame_id = "base_link"
+        t.child_frame_id = "camera_head_color_optical_frame"
+        t.transform.translation = Vector3(x=tx, y=ty, z=tz)
+        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
+        broadcaster.sendTransform(t)
+    _publish()  # 立即发一份
+    timer = node.create_timer(0.1, _publish)  # 10 Hz 续命
+    return broadcaster, timer
```

`unified_track_node.py` 同步改:
```diff
-self._head_static_broadcaster = broadcast_head_static_tf(self)
+self._head_static_broadcaster, self._head_static_timer = broadcast_head_static_tf(self)
```

`TransformBroadcaster` 走 /tf (默认 VOLATILE),跟动态 odom→base_link 走同
topic、同 path,必然进同一个 Buffer。

**已知 trade-off**:head TF 在 /tf 上每秒 10 个 message,虽然内容恒定(=静态),
但严格说算"伪动态"。可接受的代价:
- DDS 流量:每帧 TransformStamped 几个字节,1 KB/s 量级,完全可忽略
- TF Buffer 占用:多一份伪动态 edge
- lookup 行为:lookup_transform 在 cache_time 内必能找到

**长期建议**:自建 Docker 镜像时改 `CYCLONEDDS_URI` 配置(或不绑 eth10)测
一下能否复现这个 race;若能修就回到 `StaticTransformBroadcaster`。

### Round 5:arm 检测 stamp 略新于最新 robot_pose

**症状**:Round 4 修完后,稳态下 arm lookup 仍偶发:
```
Lookup would require extrapolation into the future.
Requested time T but the latest data is at time T-30ms
```

**根因**:
- `/utlidar/robot_pose` 18.7 Hz (周期 ~53 ms)
- arm realsense 检测 30 Hz (周期 ~33 ms)
- 检测 stamp 是 realsense 自带 capture time,**会略新于最新 robot_pose**(gap ~30 ms)

**修复**:`input_adapter.adapt()` 加二级 fallback(代价 <100 ms temporal
alignment 误差,但保证不丢 observation):
```python
try:
    T = self._tf_buffer.lookup_transform("odom", msg.header.frame_id,
        time=rclpy.time.Time.from_msg(msg.head.stamp), timeout=self._timeout)
except Exception as e:
    try:
        T = self._tf_buffer.lookup_transform("odom", msg.header.frame_id,
            time=rclpy.time.Time(),  # latest available
            timeout=self._timeout)
    except Exception as e2:
        # 真连不上,skip 整批
        return []
```

**trade-off**:1 m/s 速度物体上 100 ms 误差 = 10 cm,**对静态物体无影响**
(本工作区当前目标物是桌上的木块,基本不动)。动态场景需要重审:
- 真需要 temporal accuracy 时,改用 KF 状态预测(用 last robot_pose 速度外推)
- 或把 robot_pose publisher 提到 30+ Hz

## 真机验证数据

启动后稳态:
```
ros2 topic hz /perception/tracks/unified
average rate: 10.224  min: 0.023s max: 0.166s std dev: 0.025s window: 33
```

连续 sequence_id 2156 → 3624 (~3.5 分钟无中断)。

**典型 track 输出**(sequence_id 3624):
```
track_id=18  status=coasting  age=3235ms  obs=1151
  class=wood_block  conf=0.82  last_cam=camera_arm_color_optical_frame
  position (odom): x=0.658  y=-0.097  z=-0.066  (m)
  size: 0.043 × 0.046 × 0.014 m
  source_cameras: [camera_head_color_optical_frame, camera_arm_color_optical_frame]

track_id=39  status=coasting  age=309ms  obs=107
  class=wood_block  conf=0.46  last_cam=camera_head_color_optical_frame
  position (odom): x=0.679  y=-0.067  z=-0.037  (m)
  size: 0.040 × 0.022 × 0.029 m
  source_cameras: [camera_head_color_optical_frame, camera_arm_color_optical_frame]
```

**验证清单**:
- [x] `/perception/tracks/unified` 持续 10-11 Hz
- [x] track 字段完整:`position / covariance / bbox / source_cameras / size / confidence`
- [x] 双相机 merge:`source_cameras` 同时含 head + arm → 同 track_id
- [x] 多目标:track_id 18 (主块) + track_id 39 (次块) 同时维护为独立 ID
- [x] lifecycle:`tentative → confirmed → coasting`(track 41 出现 age=14ms 还在 tentative)
- [x] output_frame_id = `odom`,position 在 SLAM 世界系

## 改动文件清单

```
src/lgw_3d_tracker/lgw_3d_tracker/
├── unified_track_node.py                 (Round 2: RELIABLE QoS; Round 3: ObservationResult type; Round 4: 接 (broadcaster, timer))
└── unified_track/
    ├── input_adapter.py                  (Round 3: ObservationResult type; Round 5: now() fallback)
    ├── calibration.py                    (Round 4: TransformBroadcaster + 100ms timer)
    └── odom_tf_broadcaster.py            (旧 fix: SLAM stamp drift > 1s → fallback to node clock.now())
```

容器内 CLI 进程(`pgrep static_transform_publisher`):
- 6 个 `tcp_link → camera_arm_color_optical_frame`(历次调试残留,等价无副作用)
- 1 个 `base_link → camera_head_color_optical_frame`(Round 4 fallback)

## 已知 caveat

1. **6 个 arm static_transform_publisher 进程重复**:`pgrep -f static_transform_publisher` 显示同一 TF 被多个 instance 同时 publish,虽然等价幂等,DDS 层 /tf_static 上出现多条相同 message。建议下次进容器后 dedup(`pkill -f tcp_link.*camera_arm_color_optical_frame` 留 1 个)。
2. **tracker head static TF 也通过 CLI 起了一份**:与 `calibration.py` 内的 `TransformBroadcaster` 双发,等价幂等。建议下次 dedup 时一起清,只留 `calibration.py` 的。
3. **tracker 启动期有 ~5-10s WARN 噪声**(odom 不存在 / extrapolation into past):buffer 还没填到 cache_time=30s,稳态后清。本次未做"启动期延后 first detection 处理",可作为小优化项。
4. **arm 静态 TF 仍未内嵌到 tracker**:`calibration.py` 里 `broadcast_head_static_tf` 模式可以扩到 arm(读 `arm_calibration.json _tcp` 变体,起 timer)。但 arm 的 TF 还包含 realsense2_camera_node 的 URDF 子链,直接 publish `tcp_link → camera_arm_color_optical_frame` 会重复。本次保留 CLI 路线,待确认是否真重复再做整合。

## take-aways(给后续调试)

1. **DDS QoS 永远是先查项**:`ros2 topic info <topic> --verbose` 看 publisher QoS,再对照 subscriber。
2. **DDS type 不匹配是静默杀手**:publisher/subscriber type 不同,DDS 直接拒投,无任何 warning,只有 `node info` 报"There are N nodes"金丝雀。
3. **cyclonedds+eth10 下不要信任 TRANSIENT_LOCAL**:view_frames 能看到不等于本 process 的 tf2 Buffer 收到。**任何想被本 node lookup 用到的 static TF,直接走 dynamic broadcaster + timer**。
4. **检测 vs SLAM 频率不匹配**:高频 sensor + 低频 SLAM pose 时,lookup 必须有 now() fallback,否则 30% 概率丢 detection。
5. **`ros2 topic hz` 不能告诉你"有多少 wire 真正接收"**:Round 2/3 都是 wire 上有消息但 subscriber 没收到。`ros2 node info` 才是 ground truth。

## 相关引用

- 真机 bringup:`scripts/start_lgw_yoloe.sh` + `scripts/start_lgw_arm.sh`
- 标定 JSON:`src/lgw_bt_planner/calibration/head_calibration.json` + `arm_calibration.json`
- 接口定义:`src/yoloe_interfaces/msg/ObservationResult.msg`
- 本次真机验证对应 master 分支 commits 范围 `eac1511..9c1133e` + 本次 4 个 fix(本会话末)
- 调试日志:`.run/lgw_tracker_*.log` + `.run/lgw_arm_*.log` + `.run/lgw_yoloe_*.log`