# lgw_3d_tracker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 `unified_track_node`(Phase A-C),在 `/home/unitree/my_ws/l4t/lg_walker_ws` 内提供跨相机一致的 3D 跟踪,输出 `/perception/tracks/unified` 给下游 BT 消费者。

**Architecture:** 新建 ament_python 包 `lgw_3d_tracker`,订阅 `/perception/two_stage/camera_*/result` (5Hz) 与 `/utlidar/robot_pose` (17.8Hz SLAM),用 TF lookup at detection.stamp 做时间同步,把每条 detection 转到 odom 系,跑 3 维静态 KF + 两阶段 Hungarian 关联 + 跨相机合并,输出 `UnifiedTrackArray`。`unified_track_node` 启动期读 `head_calibration.json` 广播 head_cam 静态 TF,运行期把 `/utlidar/robot_pose` 转成动态 odom→base_link TF。

**Tech Stack:** ROS 2 Humble (Python 3.10, ament_python), NumPy, scipy (linear_sum_assignment), tf2_ros, yoloe_interfaces (扩展 2 个 msg)。

**Spec:** `l4t/lg_walker_ws/docs/superpowers/specs/2026-10-19-lgw-3d-tracker-design.md`
**上游设计参考:** `/home/unitree/yoloe_deploy/docs/混合3D跟踪设计.md`

## Global Constraints

- 工作目录 `/home/unitree/my_ws/l4t/lg_walker_ws` 是 host 路径,容器内对应 `/my_ws/l4t/lg_walker_ws`。
- 所有 build / install / test 命令走 `docker exec self_l4t bash -c "..."`。
- build 前必须 `source /my_ws/l4t/l4t_setup.sh`(暴露 agx_arm_ws / bt_ros2_ws / unitree_ros2 / yoloe_interfaces)。
- build 命令:`docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select <pkg>"`
- 不允许 `pkill -f ros` / `killall` / `kill -9 -1`,容器清理要先 `docker exec self_l4t ps aux` 列 PID,逐个 `kill -15 <pid>`。
- Python 包遵循 `l4t/lg_walker_ws/README.md`:"src/ 下只放自写包",`lgw_3d_tracker` 放 `src/` 下。
- 测试模式:**纯订阅真话题**,不写 mock publisher,不写 unit test(本 plan 用 build + smoke import 验证每个模块)。
- 所有代码用中文 docstring(本工作区风格),但标识符、msg 字段名用英文。
- 提交风格:Conventional Commits,每 task 完成后立即 commit。

## File Structure

### 新增文件

| 路径 | 责任 |
|---|---|
| `src/yoloe_interfaces/msg/UnifiedTrack.msg` | 单 track 字段定义 |
| `src/yoloe_interfaces/msg/UnifiedTrackArray.msg` | track 数组包装 |
| `src/yoloe_interfaces/CMakeLists.txt` (modify) | 加 2 个新 msg |
| `src/lgw_3d_tracker/package.xml` | 包元数据 + 依赖 |
| `src/lgw_3d_tracker/setup.py` | ament_python,entry_point |
| `src/lgw_3d_tracker/setup.cfg` | 安装配置 |
| `src/lgw_3d_tracker/resource/lgw_3d_tracker` | ament 索引 marker |
| `src/lgw_3d_tracker/launch/unified_track_node.launch.py` | 启动入口 |
| `src/lgw_3d_tracker/config/runtime.yaml` | 运行时参数 |
| `src/lgw_3d_tracker/lgw_3d_tracker/__init__.py` | 空 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py` | 主节点 wiring |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/__init__.py` | 空 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py` | YAML 参数加载 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/calibration.py` | head_calibration.json + StaticTransformBroadcaster |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/odom_tf_broadcaster.py` | /utlidar/robot_pose → dynamic odom→base_link TF |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py` | YoloeDetection3DArray → Observation(odom 系) |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/kf.py` | 3 维静态 KF |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py` | Track dataclass + lifecycle 状态机 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/cost_matrix.py` | 3D 距离 + 两阶段 Hungarian |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py` | predict + associate + update + lifecycle 协调 |
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py` | Track[] → UnifiedTrackArray |

### 修改文件

- `src/yoloe_interfaces/CMakeLists.txt`(加 2 个新 msg)

---

## Task 1: 扩展 yoloe_interfaces 加 2 个 msg

**Files:**
- Create: `src/yoloe_interfaces/msg/UnifiedTrack.msg`
- Create: `src/yoloe_interfaces/msg/UnifiedTrackArray.msg`
- Modify: `src/yoloe_interfaces/CMakeLists.txt`

**Interfaces:**
- Produces: msg 类型 `yoloe_interfaces/msg/UnifiedTrack`, `yoloe_interfaces/msg/UnifiedTrackArray`

- [ ] **Step 1: 看现有 CMakeLists.txt 怎么挂 msg**

```bash
cat /home/unitree/my_ws/l4t/lg_walker_ws/src/yoloe_interfaces/CMakeLists.txt
```

期望:看到 `rosidl_generate_interfaces(...)` 列出 `YoloeDetection3D.msg`、`YoloeDetection3DArray.msg`、`ObservationCommand.msg`、`ObservationRequest.msg`、`ObservationResult.msg`。

- [ ] **Step 2: 写 `UnifiedTrack.msg`**

`src/yoloe_interfaces/msg/UnifiedTrack.msg`:

```text
builtin_interfaces/Time stamp
uint32 frame_stamp_ms

# === identity ===
uint64 track_id
string status
int32 age
int32 time_since_last_observation_ms

# === 类别与置信度 ===
string class_name
float32 confidence
int32 observation_count
int32 confirmed_hits

# === 3D 状态(odom 系,3 维 KF) ===
geometry_msgs/Point position
float64[9] position_covariance

# === 2D 辅助信息 ===
sensor_msgs/RegionOfInterest last_bbox_2d
string last_source_camera
float32[3] last_bbox_color
string[] source_cameras

# === nuScenes-style 物理尺寸 ===
float32 width_m
float32 length_m
float32 height_m
bool size_valid
```

- [ ] **Step 3: 写 `UnifiedTrackArray.msg`**

`src/yoloe_interfaces/msg/UnifiedTrackArray.msg`:

```text
std_msgs/Header header
builtin_interfaces/Time wall_stamp
uint32 frame_stamp_ms
uint64 sequence_id
UnifiedTrack[] tracks
```

- [ ] **Step 4: 在 CMakeLists.txt 注册新 msg**

打开 `src/yoloe_interfaces/CMakeLists.txt`,找到 `rosidl_generate_interfaces(${PROJECT_NAME} ...)` 调用,在现有 msg 列表末尾加:

```cmake
  "msg/UnifiedTrack.msg"
  "msg/UnifiedTrackArray.msg"
```

(后面带逗号,跟前面的项保持逗号分隔。)

- [ ] **Step 5: Build 验证**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select yoloe_interfaces"
```

期望:build 成功,无 error。`install/yoloe_interfaces/include/yoloe_interfaces/msg/` 出现 `unified_track.hpp` 等。

- [ ] **Step 6: 验证 Python 类可见**

```bash
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/lg_walker_ws/install/setup.bash && \
   python3 -c 'from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray; \
               u = UnifiedTrack(); u.track_id = 42; print(type(u).__name__, u.track_id)'"
```

期望:打印 `UnifiedTrack 42`,无 import error。

- [ ] **Step 7: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/yoloe_interfaces/msg/UnifiedTrack.msg \
          src/yoloe_interfaces/msg/UnifiedTrackArray.msg \
          src/yoloe_interfaces/CMakeLists.txt && \
  git commit -m "feat(yoloe_interfaces): add UnifiedTrack + UnifiedTrackArray msg"
```

---

## Task 2: 创建 lgw_3d_tracker 包骨架

**Files:**
- Create: `src/lgw_3d_tracker/package.xml`
- Create: `src/lgw_3d_tracker/setup.py`
- Create: `src/lgw_3d_tracker/setup.cfg`
- Create: `src/lgw_3d_tracker/resource/lgw_3d_tracker`(空文件)
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/__init__.py`(空)
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/__init__.py`(空)

**Interfaces:**
- 无前置依赖
- 后续 task 在 `lgw_3d_tracker.unified_track` 子模块下添加文件

- [ ] **Step 1: 创建 `package.xml`**

`src/lgw_3d_tracker/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lgw_3d_tracker</name>
  <version>0.1.0</version>
  <description>Unified 3D tracking across head + arm cameras. Phase A-C of mixed 3D tracking design.</description>
  <maintainer email="unitree@localhost.localdomain">project team</maintainer>
  <license>Proprietary</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>nav_msgs</depend>

  <depend>yoloe_interfaces</depend>

  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: 创建 `setup.cfg`**

`src/lgw_3d_tracker/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/lgw_3d_tracker
[install]
install_scripts=$base/lib/lgw_3d_tracker
```

- [ ] **Step 3: 创建 `setup.py`**

`src/lgw_3d_tracker/setup.py`:

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lgw_3d_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='project team',
    maintainer_email='unitree@localhost.localdomain',
    description='Unified 3D tracking across cameras (Phase A-C).',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_track_node = lgw_3d_tracker.unified_track_node:main',
        ],
    },
)
```

- [ ] **Step 4: 创建 resource marker 和空 `__init__.py`**

```bash
mkdir -p /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker/resource
touch /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker/resource/lgw_3d_tracker

mkdir -p /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track
touch /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/__init__.py
touch /home/unitree/my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker/lgw_3d_tracker/unified_track/__init__.py
```

- [ ] **Step 5: Build 验证包结构**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select lgw_3d_tracker"
```

期望:build 成功。

- [ ] **Step 6: 验证 install 路径**

```bash
docker exec self_l4t bash -c \
  "ls /my_ws/l4t/lg_walker_ws/install/lgw_3d_tracker/lib/python3.10/site-packages/lgw_3d_tracker/"
```

期望:出现 `unified_track/` 子目录(可能为空,说明 `find_packages` 找到了)。

- [ ] **Step 7: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/ && \
  git commit -m "feat(lgw_3d_tracker): scaffold ament_python package"
```

---

## Task 3: KF 模块(kf.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/kf.py`

**Interfaces:**
- Consumes: 无
- Produces: `class KalmanState3D`: dataclass-like with `x: np.ndarray (3,)`, `P: np.ndarray (3,3)`
- Produces: `class KalmanFilter3D`: `__init__(q: float, r: float)`, `predict(state)`, `update(state, z)`, `maha_distance(state, z) -> float`

- [ ] **Step 1: 写 `kf.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/kf.py`:

```python
"""3 维静态卡尔曼滤波器。

state x = [px, py, pz] (odom 系,米)
A = I3    (静态目标 x_pred = x)
Q = sigma_p^2 * I3    (per-tick process noise)
H = I3    (直接观测位置)
R = sigma_obs^2 * I3 (observation noise)

为什么 dt = 1 / tick:静态模型 A = I 下,x_pred = x 与 dt 无关;
P_pred = P + Q,Q 必须是"每 tick 增量"才稳定。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class KalmanState3D:
    """3 维 KF 状态。"""
    x: np.ndarray  # (3,) 位置向量
    P: np.ndarray  # (3, 3) 协方差


class KalmanFilter3D:
    """3 维静态 KF,3×3 矩阵,纯 NumPy。

    Args:
        sigma_p: per-tick 过程噪声标准差 (m/tick),典型 0.05。
        sigma_obs: 观测噪声标准差 (m),典型 0.03(D435i @ ~3m)。
    """

    def __init__(self, sigma_p: float, sigma_obs: float):
        if sigma_p <= 0 or sigma_obs <= 0:
            raise ValueError("sigma_p 和 sigma_obs 必须 > 0")
        self._Q = (sigma_p ** 2) * np.eye(3)
        self._R = (sigma_obs ** 2) * np.eye(3)

    def predict(self, state: KalmanState3D) -> KalmanState3D:
        """静态 x_pred = x;P_pred = P + Q。

        Returns:
          新的 KalmanState(不修改入参)。
        """
        return KalmanState3D(x=state.x.copy(), P=state.P + self._Q)

    def update(self, state: KalmanState3D, z: np.ndarray) -> KalmanState3D:
        """KF update,3×3 标量除。

        Args:
          state: 当前状态
          z: (3,) 观测向量

        Returns:
          更新后的新 KalmanState。
        """
        if z.shape != (3,):
            raise ValueError(f"z must be shape (3,), got {z.shape}")

        y = z - state.x
        S = state.P + self._R
        # 3×3 标量除,等价 K = P @ S^{-1},但 S 是对角矩阵,直接除即可
        K = state.P / S  # numpy broadcast 元素级
        new_x = state.x + K * y
        new_P = (1.0 - K) * state.P
        return KalmanState3D(x=new_x, P=new_P)

    def maha_distance(self, state: KalmanState3D, z: np.ndarray) -> float:
        """3 维 Mahalanobis 距离(对角 S 简化)。

        用于关联门控,数值 = sqrt((z-x)^T S^{-1} (z-x))。
        """
        if z.shape != (3,):
            raise ValueError(f"z must be shape (3,), got {z.shape}")
        y = z - state.x
        S_diag = np.diag(state.P + self._R)  # (3,)
        # 避免除零
        if np.any(S_diag <= 0):
            return float("inf")
        return float(np.sqrt(np.sum(y ** 2 / S_diag)))
```

- [ ] **Step 2: 写 smoke 验证脚本**

把下面的命令直接粘到终端(临时一次性 import 检查,**不要 commit**):

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import numpy as np
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D, KalmanState3D

# 1. 静态预测:x 不变,P 增长
kf = KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03)
s = KalmanState3D(x=np.array([0.0, 0.0, 0.0]), P=np.eye(3) * 0.001)
s_pred = kf.predict(s)
assert np.allclose(s_pred.x, s.x), \"predict 改变了 x\"
assert np.allclose(s_pred.P, s.P + (0.05**2) * np.eye(3)), \"P 增量错了\"

# 2. update:观测带噪声,后验拉向观测
s_upd = kf.update(s, z=np.array([0.10, 0.0, 0.0]))
assert abs(s_upd.x[0] - 0.10) < 0.001, f\"x 应接近 0.10, 实际 {s_upd.x[0]}\"

# 3. maha_distance:0 残差距离为 0
d = kf.maha_distance(s, z=np.array([0.0, 0.0, 0.0]))
assert d == 0.0, f\"零残差应距离为 0, 实际 {d}\"

print(\"KF smoke OK\")
'"
```

期望:最后一行打印 `KF smoke OK`,无 assertion error。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/kf.py && \
  git commit -m "feat(agx_3d_tracker): add 3D static Kalman filter"
```

---

## Task 4: Track lifecycle(track.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py`

**Interfaces:**
- Consumes: `KalmanState3D` from kf.py
- Produces: `enum TrackStatus {TENTATIVE, CONFIRMED, COASTING, DELETED}`
- Produces: `class Track`: fields and lifecycle methods (below)

- [ ] **Step 1: 写 `track.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py`:

```python
"""Track dataclass + lifecycle 状态机。

状态机:tentative → confirmed → coasting → deleted。
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D


class TrackStatus(str, Enum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    COASTING = "coasting"
    DELETED = "deleted"  # 不发布,仅作为内部状态


@dataclass
class TrackObservation:
    """单次观测,用于跨 tick 累积。"""
    class_name: str
    confidence: float
    source_camera: str
    stamp_ms: int  # detection.header.stamp 转为相对 ms
    position_odom: np.ndarray  # (3,)
    bbox_2d: Optional[object] = None  # sensor_msgs/RegionOfInterest
    color: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.0, 0.0, 0.0)  # (w, l, h)
    size_valid: bool = False


@dataclass
class Track:
    """单个 track 实例。

    Lifecycle 字段:
      - hits: 总观测次数(含 tentative 期)
      - confirmed_hits: 升 confirmed 后命中次数
      - time_since_last_obs_ms: 距上次观测 ms
      - age: 自创建以来 tick 数
      - last_obs: 最近一次观测(供 output_formatter 用)
    """
    id: int
    created_ms: int  # 创建时刻(相对启动 ms)
    status: TrackStatus = TrackStatus.TENTATIVE
    state: KalmanState3D = field(default_factory=lambda: KalmanState3D(
        x=np.zeros(3), P=np.eye(3) * 0.05
    ))
    hits: int = 0
    confirmed_hits: int = 0
    time_since_last_obs_ms: int = 0
    age: int = 0
    last_obs: Optional[TrackObservation] = None
    source_cameras: List[str] = field(default_factory=list)

    def apply_observation(self, obs: TrackObservation, kf_update_state) -> None:
        """关联到本 track,做 KF update 并更新 lifecycle 计数。

        Args:
          obs: 关联上的观测
          kf_update_state: callable(KalmanState3D, np.ndarray) -> KalmanState3D
        """
        self.state = kf_update_state(self.state, obs.position_odom)
        self.last_obs = obs
        self.hits += 1
        self.time_since_last_obs_ms = 0
        self.age += 1
        if obs.source_camera not in self.source_cameras:
            self.source_cameras.append(obs.source_camera)

    def tick(self, dt_ms: int, tentative_min_hits: int, coast_max_age_ms: int) -> None:
        """每帧 tick:累计 time_since,推进状态机。

        Args:
          dt_ms: 距上次 tick 的 ms(用于累加 time_since)
          tentative_min_hits: 升 confirmed 所需命中数
          coast_max_age_ms: 升 deleted 阈值
        """
        self.time_since_last_obs_ms += dt_ms
        self.age += 1

        if self.status == TrackStatus.TENTATIVE:
            if self.hits >= tentative_min_hits:
                self.status = TrackStatus.CONFIRMED
                self.confirmed_hits = 0
        elif self.status == TrackStatus.CONFIRMED:
            if self.time_since_last_obs_ms > 0:
                self.status = TrackStatus.COASTING
        elif self.status == TrackStatus.COASTING:
            pass  # 已在 coasting,等超阈值转 deleted

        if self.time_since_last_obs_ms >= coast_max_age_ms:
            self.status = TrackStatus.DELETED
```

- [ ] **Step 2: 写 smoke 验证脚本**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import numpy as np
from lgw_3d_tracker.unified_track.track import Track, TrackStatus, TrackObservation

# identity update:lambda K = I(占位)
def noop_update(s, z):
    return s

# 1. 新建 → tentative
t = Track(id=1, created_ms=0)
assert t.status == TrackStatus.TENTATIVE
assert t.hits == 0

# 2. 3 次命中 → confirmed
for i in range(3):
    obs = TrackObservation(
        class_name=\"red_block\", confidence=0.8,
        source_camera=\"camera_head_color_optical_frame\",
        stamp_ms=i * 100, position_odom=np.array([0.1*i, 0.0, 0.0]))
    t.apply_observation(obs, noop_update)
assert t.status == TrackStatus.CONFIRMED, f\"3 hits 后应 confirmed, 实际 {t.status}\"
assert t.confirmed_hits == 0

# 3. 不再 update,200ms 后 → coasting
t.tick(dt_ms=200, tentative_min_hits=3, coast_max_age_ms=1500)
assert t.status == TrackStatus.COASTING

# 4. 累计到 1500ms → deleted
t.tick(dt_ms=1400, tentative_min_hits=3, coast_max_age_ms=1500)
assert t.status == TrackStatus.DELETED

print(\"Track lifecycle OK\")
'"
```

期望:最后一行打印 `Track lifecycle OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py && \
  git commit -m "feat(lgw_3d_tracker): add Track dataclass with lifecycle state machine"
```

---

## Task 5: Cost matrix + 两阶段 Hungarian(cost_matrix.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/cost_matrix.py`

**Interfaces:**
- Consumes: `np.ndarray` observations (N, 3), `List[Track]`, distance gate, class一致约束
- Produces: `class TwoStageAssigner` with `assign(observations, tracks, kf_distance_gate_fn) -> (obs_to_track: dict[int, int], unmatched_obs: list[int])`

- [ ] **Step 1: 写 `cost_matrix.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/cost_matrix.py`:

```python
"""两阶段关联 cost matrix + Hungarian 匹配。

阶段一:high_conf 观测 vs active tracks
阶段二:low_conf 观测 vs 阶段一未配上的 tracks

约束:
  - class_name 必须一致(否则 cost = +∞)
  - 3D 距离 > gate 则 cost = +∞
"""
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from lgw_3d_tracker.unified_track.track import Track, TrackStatus


# gate_fn(state, z) -> bool,True 表示在门内
GateFn = Callable[[object, np.ndarray], bool]


def _build_cost(
    obs_positions: np.ndarray,  # (N, 3)
    obs_classes: List[str],
    tracks: List[Track],
    gate_fn: GateFn,
) -> np.ndarray:
    """构造成本矩阵 (N, M)。

    cost[i, j] = 3D 欧氏距离,若 class 不匹配或距离超 gate 则 = +∞。
    """
    N = len(obs_positions)
    M = len(tracks)
    INF = 1e9
    cost = np.full((N, M), INF, dtype=np.float64)

    for i in range(N):
        if tracks and i >= 0:
            pass
        for j, t in enumerate(tracks):
            if obs_classes[i] != (t.last_obs.class_name if t.last_obs else ""):
                continue
            if not gate_fn(t.state, obs_positions[i]):
                continue
            d = float(np.linalg.norm(obs_positions[i] - t.state.x))
            if d < INF:
                cost[i, j] = d
    return cost


def _hungarian(cost: np.ndarray) -> List[Tuple[int, int]]:
    """scipy linear_sum_assignment,返回 (obs_idx, track_idx) 配对列表。

    cost = +∞ 的对不会被配(线性分配会自动跳过)。
    """
    if cost.size == 0:
        return []
    row_ind, col_ind = linear_sum_assignment(cost)
    return list(zip(row_ind.tolist(), col_ind.tolist()))


class TwoStageAssigner:
    """两阶段 Hungarian 关联器。

    Args:
      high_conf_th: 高置信度阈值(conf >= 此值进入阶段一)
      low_conf_th: 低置信度阈值(low_conf_th <= conf < high_conf_th 进入阶段二)
      gate_fn: 距离门控 callable(KalmanState3D, z) -> bool
    """

    def __init__(
        self,
        high_conf_th: float,
        low_conf_th: float,
        gate_fn: GateFn,
    ):
        if low_conf_th > high_conf_th:
            raise ValueError("low_conf_th 必须 <= high_conf_th")
        self.high_conf_th = high_conf_th
        self.low_conf_th = low_conf_th
        self.gate_fn = gate_fn

    def assign(
        self,
        obs_positions: np.ndarray,        # (N, 3)
        obs_classes: List[str],           # length N
        obs_confidences: List[float],     # length N
        tracks: List[Track],
    ) -> Tuple[Dict[int, int], List[int]]:
        """两阶段关联。

        Returns:
          obs_to_track: {obs_idx: track_idx}
          unmatched_obs: [obs_idx, ...](未配上任何 track 的观测)
        """
        N = len(obs_positions)
        active_tracks = [t for t in tracks if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)]
        if N == 0 or not active_tracks:
            return {}, list(range(N))

        # 阶段一:high_conf 观测
        high_idx = [i for i in range(N) if obs_confidences[i] >= self.high_conf_th]
        low_idx = [i for i in range(N) if self.low_conf_th <= obs_confidences[i] < self.high_conf_th]

        obs_to_track: Dict[int, int] = {}
        matched_track_idx = set()

        if high_idx:
            high_pos = obs_positions[high_idx]
            high_cls = [obs_classes[i] for i in high_idx]
            cost = _build_cost(high_pos, high_cls, active_tracks, self.gate_fn)
            pairs = _hungarian(cost)
            for i_local, j in pairs:
                if cost[i_local, j] >= 1e9:
                    continue
                obs_to_track[high_idx[i_local]] = j
                matched_track_idx.add(j)

        # 阶段二:low_conf 观测 ↔ 阶段一未配上的 tracks
        if low_idx:
            low_pos = obs_positions[low_idx]
            low_cls = [obs_classes[i] for i in low_idx]
            unmatched_tracks_idx = [j for j in range(len(active_tracks)) if j not in matched_track_idx]
            if unmatched_tracks_idx:
                # 仅构建未配上的 tracks 子集的成本
                sub_tracks = [active_tracks[j] for j in unmatched_tracks_idx]
                cost = _build_cost(low_pos, low_cls, sub_tracks, self.gate_fn)
                pairs = _hungarian(cost)
                # 注意:cost 矩阵的列对应 sub_tracks(局部 0..len-1),要映射回 active_tracks 索引
                for i_local, j_local in pairs:
                    if cost[i_local, j_local] >= 1e9:
                        continue
                    real_j = unmatched_tracks_idx[j_local]
                    obs_to_track[low_idx[i_local]] = real_j

        unmatched_obs = [i for i in range(N) if i not in obs_to_track]
        return obs_to_track, unmatched_obs
```

- [ ] **Step 2: 写 smoke 验证脚本**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import numpy as np
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.track import Track, TrackStatus, TrackObservation
from lgw_3d_tracker.unified_track.kf import KalmanState3D

# 3 个 active tracks
tracks = []
for tid, pos in [(1, np.array([1.0, 0.0, 0.0])),
                  (2, np.array([2.0, 0.0, 0.0])),
                  (3, np.array([0.0, 1.0, 0.0]))]:
    t = Track(id=tid, created_ms=0, hits=3, confirmed_hits=2,
              status=TrackStatus.CONFIRMED,
              state=KalmanState3D(x=pos.copy(), P=np.eye(3)*0.001))
    t.last_obs = TrackObservation(class_name=\"red_block\", confidence=0.8,
                                   source_camera=\"c1\", stamp_ms=0,
                                   position_odom=pos)
    tracks.append(t)

# 2 个观测:近 obs1(高 conf)配 track 1;近 obs2(低 conf)配 track 2;一个杂乱的(高 conf)无配
obs_pos = np.array([
    [1.05, 0.0, 0.0],   # 接近 track 1
    [2.10, 0.0, 0.0],   # 接近 track 2
    [10.0, 10.0, 10.0], # 远离任何 track
])
obs_cls = [\"red_block\", \"red_block\", \"red_block\"]
obs_conf = [0.8, 0.2, 0.8]  # 第 2 个低 conf

def gate(state, z):
    return np.linalg.norm(z - state.x) < 0.5  # 0.5m 门控

assigner = TwoStageAssigner(high_conf_th=0.4, low_conf_th=0.15, gate_fn=gate)
matches, unmatched = assigner.assign(obs_pos, obs_cls, obs_conf, tracks)

# 期望:obs0 -> track 1(idx 0),obs1 -> track 2(idx 1),obs2 unmatched
assert matches.get(0) == 0, f\"obs0 应配 track 1, 实际 {matches}\"
assert matches.get(1) == 1, f\"obs1 应配 track 2, 实际 {matches}\"
assert 2 in unmatched, f\"obs2 应 unmatched, 实际 unmatched={unmatched}\"
print(\"TwoStageAssigner OK\")
'"
```

期望:最后一行打印 `TwoStageAssigner OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/cost_matrix.py && \
  git commit -m "feat(lgw_3d_tracker): add two-stage Hungarian cost matrix"
```

---

## Task 6: Tracker 协调器(tracker.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py`

**Interfaces:**
- Consumes: `KalmanFilter3D`, `TwoStageAssigner`, `Track`
- Produces: `class Tracker`: `predict()`, `associate_and_update(obs)`, `tick_lifecycle(dt_ms) -> List[Track]` (返回所有 active tracks,已过滤 deleted)

- [ ] **Step 1: 写 `tracker.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py`:

```python
"""Tracker 协调器:predict + associate + update + lifecycle。

每 tick 调用顺序:
  1. predict_all()                  所有 active tracks P += Q
  2. associate_and_update(obs_list) 两阶段 Hungarian + KF update
  3. tick_lifecycle(dt_ms)          状态推进 + 返回 active tracks
"""
from typing import List, Optional, Tuple

import numpy as np

from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D, KalmanState3D
from lgw_3d_tracker.unified_track.track import Track, TrackObservation, TrackStatus


class Tracker:
    """3D 跟踪主循环。

    Args:
      kf: 3 维 KF(已经设好 sigma_p / sigma_obs)
      assigner: 两阶段关联器
      tentative_min_hits: 升 confirmed 所需命中数
      coast_max_age_ms: 升 deleted 阈值
      distance_gate_m: 3D 距离门控(米)
    """

    def __init__(
        self,
        kf: KalmanFilter3D,
        assigner: TwoStageAssigner,
        tentative_min_hits: int,
        coast_max_age_ms: int,
        distance_gate_m: float,
    ):
        self.kf = kf
        self.assigner = assigner
        self.tentative_min_hits = tentative_min_hits
        self.coast_max_age_ms = coast_max_age_ms
        self.distance_gate_m = distance_gate_m
        self._next_id = 1
        self._tracks: List[Track] = []
        self._now_ms: int = 0

    def set_now_ms(self, now_ms: int) -> None:
        """外部注入当前相对 ms(供 tick_lifecycle 使用)。"""
        self._now_ms = now_ms

    def predict_all(self) -> None:
        """所有 non-deleted tracks 做 predict(P += Q)。"""
        for t in self._tracks:
            if t.status == TrackStatus.DELETED:
                continue
            t.state = self.kf.predict(t.state)

    def associate_and_update(
        self, observations: List[TrackObservation]
    ) -> None:
        """关联并 KF update,未配上的观测创建新 tentative track。

        Args:
          observations: 本 tick 累积的观测列表(可空)
        """
        if not observations:
            return

        # 收集 active tracks 用于关联
        active = [t for t in self._tracks if t.status in
                  (TrackStatus.CONFIRMED, TrackStatus.COASTING)]
        # 也允许 tentative 关联,设计 §6.3 阶段一主关联确认 + 阶段二恢复
        tentative = [t for t in self._tracks if t.status == TrackStatus.TENTATIVE]
        all_active = active + tentative  # tentative 也参与关联(确认后才 confirmed)

        obs_pos = np.array([o.position_odom for o in observations])
        obs_cls = [o.class_name for o in observations]
        obs_conf = [o.confidence for o in observations]

        # 用 distance_gate 直接构造一个匿名 gate_fn
        def gate_fn(state: KalmanState3D, z: np.ndarray) -> bool:
            return np.linalg.norm(z - state.x) < self.distance_gate_m

        # 临时给 assigner 注入 gate(简单做法:重建 assigner 一次)
        self.assigner.gate_fn = gate_fn
        matches, unmatched_obs = self.assigner.assign(
            obs_pos, obs_cls, obs_conf, all_active
        )

        # 把 active+tentative 索引合并回 self._tracks 索引
        track_idx_map = {}
        for j, t in enumerate(all_active):
            track_idx_map[j] = self._tracks.index(t)

        for obs_idx, j in matches.items():
            real_track_idx = track_idx_map[j]
            t = self._tracks[real_track_idx]
            t.apply_observation(observations[obs_idx], self.kf.update)

        # 未关联观测 → 新 tentative track
        for i in unmatched_obs:
            obs = observations[i]
            new_t = Track(
                id=self._next_id,
                created_ms=self._now_ms,
                state=KalmanState3D(x=obs.position_odom.copy(), P=np.eye(3) * 0.05),
                hits=1,
                time_since_last_obs_ms=0,
                age=1,
                last_obs=obs,
            )
            new_t.source_cameras = [obs.source_camera]
            self._next_id += 1
            self._tracks.append(new_t)

    def tick_lifecycle(self, dt_ms: int) -> List[Track]:
        """推进所有 track 的状态机,返回 active tracks(已过滤 DELETED)。"""
        for t in self._tracks:
            t.tick(dt_ms, self.tentative_min_hits, self.coast_max_age_ms)
        # 清理 deleted 状态 track
        self._tracks = [t for t in self._tracks if t.status != TrackStatus.DELETED]
        # 返回所有非 deleted(active + 仍 tentative + 仍 coasting)
        return [t for t in self._tracks if t.status != TrackStatus.DELETED]

    @property
    def tracks(self) -> List[Track]:
        """当前所有非 deleted tracks。"""
        return [t for t in self._tracks if t.status != TrackStatus.DELETED]
```

- [ ] **Step 2: 写 smoke 验证脚本**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import numpy as np
from lgw_3d_tracker.unified_track.tracker import Tracker
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.track import TrackObservation

kf = KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03)
def gate(state, z): return np.linalg.norm(z - state.x) < 0.5
assigner = TwoStageAssigner(high_conf_th=0.4, low_conf_th=0.15, gate_fn=gate)

tr = Tracker(kf=kf, assigner=assigner,
             tentative_min_hits=3, coast_max_age_ms=1500,
             distance_gate_m=0.5)
tr.set_now_ms(0)

# 第 1 tick:1 个观测,创建 tentative
obs1 = TrackObservation(class_name=\"red_block\", confidence=0.8,
                        source_camera=\"cam_head\", stamp_ms=0,
                        position_odom=np.array([0.0, 0.0, 0.0]))
tr.predict_all()
tr.associate_and_update([obs1])
active = tr.tick_lifecycle(dt_ms=100)
assert len(active) == 1, f\"应有 1 track, 实际 {len(active)}\"
assert active[0].status.name == \"TENTATIVE\"

# 第 2-4 tick:同位置再 3 次,升 confirmed
for i in range(2, 5):
    tr.set_now_ms(i * 100)
    tr.predict_all()
    obs = TrackObservation(class_name=\"red_block\", confidence=0.8,
                           source_camera=\"cam_head\", stamp_ms=i*100,
                           position_odom=np.array([0.0, 0.0, 0.0]))
    tr.associate_and_update([obs])
    active = tr.tick_lifecycle(dt_ms=100)

assert len(active) == 1
assert active[0].status.name == \"CONFIRMED\", f\"3 hits 后应 confirmed, 实际 {active[0].status}\"

# 不再观测,几个 tick 后 → coasting → deleted
for i in range(5, 30):
    tr.set_now_ms(i * 100)
    tr.predict_all()
    tr.associate_and_update([])  # 空
    active = tr.tick_lifecycle(dt_ms=100)

assert len(active) == 0, f\"1.5s 无观测后应全 deleted, 实际 {len(active)}\"
print(\"Tracker end-to-end OK\")
'"
```

期望:最后一行打印 `Tracker end-to-end OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py && \
  git commit -m "feat(lgw_3d_tracker): add Tracker orchestrator"
```

---

## Task 7: Config loader(config.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py`

**Interfaces:**
- Produces: `class UnifiedTrackConfig`: dataclass with all yaml fields + `from_yaml(path) -> UnifiedTrackConfig`

- [ ] **Step 1: 写 `config.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py`:

```python
"""YAML 配置加载器。

读 lgw_3d_tracker/config/runtime.yaml 的 `unified_track` 段,
返回 dataclass 实例。
"""
from dataclasses import dataclass, field
from typing import List

import yaml


@dataclass
class TwoStageConfig:
    enabled: bool = True
    high_conf_threshold: float = 0.40
    low_conf_threshold: float = 0.15


@dataclass
class UnifiedTrackConfig:
    enabled: bool = True
    head_detections_topic: str = "/perception/two_stage/camera_head/result"
    arm_detections_topic: str = "/perception/two_stage/camera_arm/result"
    sport_pose_topic: str = "/utlidar/robot_pose"
    output_topic: str = "/perception/tracks/unified"
    output_frame_id: str = "odom"
    output_min_period_ms: int = 30

    process_noise_position: float = 0.05
    observation_noise_position: float = 0.03

    tentative_min_hits: int = 3
    coast_max_age_ms: int = 1500

    association_distance_gate_m: float = 0.15
    appearance_class_must_match: bool = True
    two_stage: TwoStageConfig = field(default_factory=TwoStageConfig)

    tf_cache_sec: float = 30.0
    robot_pose_stale_ms: int = 500

    head_calibration_file: str = ""  # 空 = 默认 share/.../head_calibration.json

    @classmethod
    def from_yaml(cls, path: str) -> "UnifiedTrackConfig":
        """从 yaml 加载。文件不存在或无 unified_track 段时返回默认值。"""
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return cls()
        section = raw.get("unified_track", {}) or {}
        # 嵌套 two_stage
        ts = section.get("two_stage", {}) or {}
        return cls(
            enabled=section.get("enabled", True),
            head_detections_topic=section.get(
                "head_detections_topic", cls.head_detections_topic),
            arm_detections_topic=section.get(
                "arm_detections_topic", cls.arm_detections_topic),
            sport_pose_topic=section.get(
                "sport_pose_topic", cls.sport_pose_topic),
            output_topic=section.get("output_topic", cls.output_topic),
            output_frame_id=section.get(
                "output_frame_id", cls.output_frame_id),
            output_min_period_ms=section.get(
                "output_min_period_ms", cls.output_min_period_ms),
            process_noise_position=section.get(
                "process_noise_position", cls.process_noise_position),
            observation_noise_position=section.get(
                "observation_noise_position", cls.observation_noise_position),
            tentative_min_hits=section.get(
                "tentative_min_hits", cls.tentative_min_hits),
            coast_max_age_ms=section.get(
                "coast_max_age_ms", cls.coast_max_age_ms),
            association_distance_gate_m=section.get(
                "association_distance_gate_m", cls.association_distance_gate_m),
            appearance_class_must_match=section.get(
                "appearance_class_must_match", cls.appearance_class_must_match),
            two_stage=TwoStageConfig(
                enabled=ts.get("enabled", True),
                high_conf_threshold=ts.get(
                    "high_conf_threshold", TwoStageConfig.high_conf_threshold),
                low_conf_threshold=ts.get(
                    "low_conf_threshold", TwoStageConfig.low_conf_threshold),
            ),
            tf_cache_sec=section.get("tf_cache_sec", cls.tf_cache_sec),
            robot_pose_stale_ms=section.get(
                "robot_pose_stale_ms", cls.robot_pose_stale_ms),
            head_calibration_file=section.get(
                "head_calibration_file", cls.head_calibration_file),
        )
```

- [ ] **Step 2: 写 smoke 验证脚本**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig

# 1. 默认值
c1 = UnifiedTrackConfig()
assert c1.enabled is True
assert c1.process_noise_position == 0.05
assert c1.tentative_min_hits == 3

# 2. 解析 yaml(用一个不存在的文件验证容错)
import tempfile, os
yaml_text = \"\"\"
unified_track:
  enabled: true
  process_noise_position: 0.08
  tentative_min_hits: 5
  two_stage:
    high_conf_threshold: 0.5
    low_conf_threshold: 0.1
\"\"\"
with tempfile.NamedTemporaryFile(mode=\"w\", suffix=\".yaml\", delete=False) as f:
    f.write(yaml_text)
    p = f.name
c2 = UnifiedTrackConfig.from_yaml(p)
assert c2.process_noise_position == 0.08
assert c2.tentative_min_hits == 5
assert c2.two_stage.high_conf_threshold == 0.5
os.unlink(p)
print(\"Config loader OK\")
'"
```

期望:打印 `Config loader OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py && \
  git commit -m "feat(lgw_3d_tracker): add YAML config loader"
```

---

## Task 8: Calibration + Static TF(calibration.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/calibration.py`

**Interfaces:**
- Produces: `class HeadCalibration`:静态文件发布为 TF

- [ ] **Step 1: 写 `calibration.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/calibration.py`:

```python
"""head_calibration.json 解析 + StaticTransformBroadcaster。

启动期调用:从 lgw_bt_planner share 目录加载 head_calibration.json,
广播 base_link → camera_head_color_optical_frame 的静态 TF。
"""
import json
import os
from typing import Optional

import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _default_head_calib_path() -> str:
    """从 lgw_bt_planner share 目录解析 head_calibration.json。"""
    share = get_package_share_directory("lgw_bt_planner")
    return os.path.join(share, "calibration", "head_calibration.json")


def load_head_calibration(path: Optional[str] = None) -> tuple:
    """读 head_calibration.json,返回 (translation, quaternion_xyzw)。

    Args:
      path: 自定义路径,空字符串或 None 表示用 lgw_bt_planner share 默认值。

    Returns:
      ((tx, ty, tz), (qx, qy, qz, qw))
    """
    if not path:
        path = _default_head_calib_path()
    with open(path) as f:
        data = json.load(f)
    pos = data["position"]
    q = data["orientation"]
    return (float(pos[0]), float(pos[1]), float(pos[2])), (
        float(q[0]), float(q[1]), float(q[2]), float(q[3]),
    )


def broadcast_head_static_tf(node: rclpy.node.Node) -> StaticTransformBroadcaster:
    """读 head_calibration.json 并广播 base_link → camera_head_color_optical_frame。

    Args:
      node: 用于构造 broadcaster 的 ROS 节点

    Returns:
      broadcaster(已 send 过一次,后续无需再调用)
    """
    broadcaster = StaticTransformBroadcaster(node)
    (tx, ty, tz), (qx, qy, qz, qw) = load_head_calibration()

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = "base_link"
    t.child_frame_id = "camera_head_color_optical_frame"
    t.transform.translation = Vector3(x=tx, y=ty, z=tz)
    t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    broadcaster.sendTransform(t)
    node.get_logger().info(
        f"Published static TF base_link → camera_head_color_optical_frame "
        f"(t=({tx:.3f}, {ty:.3f}, {tz:.3f}))"
    )
    return broadcaster
```

- [ ] **Step 2: 写 smoke 验证**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
from lgw_3d_tracker.unified_track.calibration import load_head_calibration
t, q = load_head_calibration()
print(f\"translation: {t}\")
print(f\"quaternion (xyzw): {q}\")
assert len(t) == 3 and len(q) == 4
print(\"Calibration loader OK\")
'"
```

期望:打印 `Calibration loader OK` + translation 和 quaternion 数值。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/calibration.py && \
  git commit -m "feat(lgw_3d_tracker): add head_calibration.json loader + static TF broadcaster"
```

---

## Task 9: Odom TF broadcaster(odom_tf_broadcaster.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/odom_tf_broadcaster.py`

**Interfaces:**
- Produces: `class OdomTFBroadcaster`: `init`, `update(msg: PoseStamped)`, `close()`

- [ ] **Step 1: 写 `odom_tf_broadcaster.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/odom_tf_broadcaster.py`:

```python
"""/utlidar/robot_pose → 动态 odom→base_link TF。

每次收到 PoseStamped,广播一次动态 TF,**stamp 用 msg.header.stamp 原值**(不替换)。
TF buffer 会保留这些历史位姿,供 detection.timestamp 的 lookup_transform 插值。
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Vector3
from tf2_ros.transform_broadcaster import TransformBroadcaster


class OdomTFBroadcaster:
    """订阅 /utlidar/robot_pose 后调用 update() 即可广播动态 TF。"""

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._tf_broadcaster = TransformBroadcaster(node)

    def update(self, msg: PoseStamped) -> None:
        """把 PoseStamped 转成 odom → base_link TF 发出。

        Args:
          msg: PoseStamped,header.stamp 应为 SLAM 输出时刻
        """
        t = TransformStamped()
        t.header.stamp = msg.header.stamp        # 保留原值
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation = Vector3(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
        )
        t.transform.rotation = Quaternion(
            x=float(msg.pose.orientation.x),
            y=float(msg.pose.orientation.y),
            z=float(msg.pose.orientation.z),
            w=float(msg.pose.orientation.w),
        )
        self._tf_broadcaster.sendTransform(t)
```

- [ ] **Step 2: smoke 验证(单元,不需要 ROS 运行)**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import inspect
from lgw_3d_tracker.unified_track.odom_tf_broadcaster import OdomTFBroadcaster
# 仅验证 import + 源码检查(避免启 ROS)
src = inspect.getsource(OdomTFBroadcaster)
assert \"TransformBroadcaster\" in src
assert \"sendTransform\" in src
assert \"header.stamp\" in src
print(\"OdomTFBroadcaster interface OK\")
'"
```

期望:打印 `OdomTFBroadcaster interface OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/odom_tf_broadcaster.py && \
  git commit -m "feat(lgw_3d_tracker): add dynamic odom→base_link TF broadcaster"
```

---

## Task 10: Input adapter(input_adapter.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py`

**Interfaces:**
- Produces: `class InputAdapter`: `adapt(yoloe_detection_array_msg) -> List[TrackObservation]`

- [ ] **Step 1: 写 `input_adapter.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py`:

```python
"""YoloeDetection3DArray → List[TrackObservation](odom 系)。

每条 detection 通过 tf2_ros.Buffer.lookup_transform("odom", camera_frame, time=det.stamp)
拿到 TF(TF2 buffer 自动串联 head_calibration static TF 和动态 odom→base_link TF,
并在 SLAM pose 之间插值),把 center_m 从相机系转到 odom 系。
"""
from typing import List

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point
from yoloe_interfaces.msg import YoloeDetection3DArray

from lgw_3d_tracker.unified_track.track import TrackObservation


def _to_point_msg(xyz: np.ndarray) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


class InputAdapter:
    """把 YoloeDetection3DArray 转 odom 系观测列表。

    Args:
      tf_buffer: tf2_ros.Buffer 实例(由调用方提供)
      logger: ROS logger(用于 WARN/ERROR)
      lookup_timeout_sec: TF lookup 超时阈值(秒),默认 0.05
    """

    def __init__(self, tf_buffer: Buffer, logger, lookup_timeout_sec: float = 0.05):
        self._tf_buffer = tf_buffer
        self._logger = logger
        self._timeout = DurationMsg()
        # DurationMsg.sec/nanosec 类型为 int
        self._timeout.sec = int(lookup_timeout_sec)
        self._timeout.nanosec = int((lookup_timeout_sec - int(lookup_timeout_sec)) * 1e9)

    def adapt(self, msg: YoloeDetection3DArray) -> List[TrackObservation]:
        """对 msg.detections 做逐条 TF 转换,返回 TrackObservation 列表。

        Args:
          msg: YoloeDetection3DArray,header.frame_id 是相机光学系

        Returns:
          有效观测列表(position_valid=True 且 TF lookup 成功)
        """
        observations: List[TrackObservation] = []
        # 一次 lookup,所有 detections 复用(同 frame_id 同 time)
        try:
            T_odom_to_cam: TransformStamped = self._tf_buffer.lookup_transform(
                "odom", msg.header.frame_id,
                time=rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=self._timeout,
            )
        except Exception as e:
            self._logger.warn(
                f"TF lookup odom→{msg.header.frame_id} at {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"failed: {e}; skip batch"
            )
            return []

        cam_to_odom = T_odom_to_cam  # not directly transformable; we use do_transform_point
        for det in msg.detections:
            if not det.position_valid:
                continue
            try:
                p_camera = Point(
                    x=float(det.center_m.x),
                    y=float(det.center_m.y),
                    z=float(det.center_m.z),
                )
                p_odom = do_transform_point(p_camera, cam_to_odom)
            except Exception as e:
                self._logger.warn(f"transform_point failed: {e}; skip this det")
                continue

            stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000

            observations.append(TrackObservation(
                class_name=str(det.class_name),
                confidence=float(det.confidence),
                source_camera=str(msg.header.frame_id),
                stamp_ms=stamp_ms,
                position_odom=np.array([p_odom.x, p_odom.y, p_odom.z]),
                bbox_2d=det.bbox,
                color=(0.0, 0.0, 0.0),  # YoloeDetection3D 无 color 字段
                size=(float(det.size_m.x), float(det.size_m.y), float(det.size_m.z)),
                size_valid=bool(det.size_valid),
            ))
        return observations
```

- [ ] **Step 2: smoke 验证(仅 import)**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import inspect
from lgw_3d_tracker.unified_track.input_adapter import InputAdapter
src = inspect.getsource(InputAdapter)
assert \"lookup_transform\" in src
assert \"do_transform_point\" in src
print(\"InputAdapter interface OK\")
'"
```

期望:打印 `InputAdapter interface OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/input_adapter.py && \
  git commit -m "feat(lgw_3d_tracker): add input adapter (camera→odom via TF)"
```

---

## Task 11: Output formatter(output_formatter.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py`

**Interfaces:**
- Produces: `class OutputFormatter`: `format(tracks, sequence_id, frame_stamp_ms, now) -> UnifiedTrackArray`

- [ ] **Step 1: 写 `output_formatter.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py`:

```python
"""Track[] → UnifiedTrackArray msg 格式化。

frame_id = config.output_frame_id(默认 "odom")。
deleted 状态不发布(已经在 Tracker.tick_lifecycle 内过滤)。
"""
from typing import List

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray

from lgw_3d_tracker.unified_track.track import Track, TrackStatus


class OutputFormatter:
    """Track 列表 → UnifiedTrackArray 格式化器。"""

    def __init__(self, frame_id: str):
        self._frame_id = frame_id

    def format(
        self,
        tracks: List[Track],
        sequence_id: int,
        frame_stamp_ms: int,
        now: TimeMsg,
    ) -> UnifiedTrackArray:
        msg = UnifiedTrackArray()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.wall_stamp = now
        msg.frame_stamp_ms = frame_stamp_ms
        msg.sequence_id = sequence_id
        msg.tracks = [self._track_to_msg(t) for t in tracks]
        return msg

    def _track_to_msg(self, t: Track) -> UnifiedTrack:
        m = UnifiedTrack()
        # stamp 与 wall_stamp 一致(简化:track 没有自己单独的 stamp)
        # 严格说应用 last_obs.stamp_ms 转回 Time,但本设计简化用 header.stamp
        # 已删除字段:velocity(3 维 KF 无)
        m.track_id = t.id
        m.status = t.status.value if isinstance(t.status, TrackStatus) else str(t.status)
        m.age = t.age
        m.time_since_last_observation_ms = int(t.time_since_last_obs_ms)

        if t.last_obs is not None:
            m.class_name = str(t.last_obs.class_name)
            m.confidence = float(t.last_obs.confidence)
            if t.last_obs.bbox_2d is not None:
                m.last_bbox_2d = t.last_obs.bbox_2d
            m.last_source_camera = str(t.last_obs.source_camera)
            col = t.last_obs.color
            m.last_bbox_color = [float(col[0]), float(col[1]), float(col[2])]
            sz = t.last_obs.size
            m.width_m = float(sz[0])
            m.length_m = float(sz[1])
            m.height_m = float(sz[2])
            m.size_valid = bool(t.last_obs.size_valid)

        m.observation_count = int(t.hits)
        m.confirmed_hits = int(t.confirmed_hits)

        # KF state
        m.position.x = float(t.state.x[0])
        m.position.y = float(t.state.x[1])
        m.position.z = float(t.state.x[2])
        # 3x3 行优先 flatten
        m.position_covariance = [float(v) for v in t.state.P.flatten()]

        m.source_cameras = list(t.source_cameras)
        return m
```

- [ ] **Step 2: smoke 验证(构造一个 fake Track,验证 msg 字段)**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   python3 -c '
import numpy as np
from builtin_interfaces.msg import Time
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.track import Track, TrackStatus, TrackObservation
from lgw_3d_tracker.unified_track.kf import KalmanState3D

# 构造一个 track
t = Track(id=42, created_ms=0, status=TrackStatus.CONFIRMED,
          hits=5, confirmed_hits=3,
          state=KalmanState3D(x=np.array([1.0, 2.0, 3.0]), P=np.eye(3)*0.01),
          time_since_last_obs_ms=100, age=10)
t.last_obs = TrackObservation(class_name=\"red_block\", confidence=0.9,
                              source_camera=\"cam_head\", stamp_ms=0,
                              position_odom=np.array([1.0, 2.0, 3.0]),
                              bbox_2d=None, color=(1.0, 0.0, 0.0),
                              size=(0.05, 0.05, 0.05), size_valid=True)
t.source_cameras = [\"cam_head\", \"cam_arm\"]

fmt = OutputFormatter(frame_id=\"odom\")
now = Time(sec=100, nanosec=0)
msg = fmt.format([t], sequence_id=1, frame_stamp_ms=1000, now=now)

assert msg.header.frame_id == \"odom\"
assert msg.sequence_id == 1
assert len(msg.tracks) == 1
m = msg.tracks[0]
assert m.track_id == 42
assert m.status == \"confirmed\"
assert abs(m.position.x - 1.0) < 1e-6
assert m.confidence == 0.9
assert m.width_m == 0.05
assert m.last_bbox_color == [1.0, 0.0, 0.0]
assert \"cam_head\" in m.source_cameras and \"cam_arm\" in m.source_cameras
print(\"OutputFormatter OK\")
'"
```

期望:打印 `OutputFormatter OK`。

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py && \
  git commit -m "feat(lgw_3d_tracker): add output formatter (Track → UnifiedTrackArray)"
```

---

## Task 12: 主节点 + launch + yaml(unified_track_node.py)

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py`
- Create: `src/lgw_3d_tracker/launch/unified_track_node.launch.py`
- Create: `src/lgw_3d_tracker/config/runtime.yaml`

**Interfaces:**
- 主节点:订阅 3 个 topic(2 路检测 + robot_pose),发 1 个 topic,启动期广播静态 TF,运行期广播动态 TF
- launch:接受 yaml_path 参数,启动节点
- yaml:`unified_track` 段

- [ ] **Step 1: 写 `runtime.yaml`**

`src/lgw_3d_tracker/config/runtime.yaml`:

```yaml
unified_track:
  enabled: true

  # 输入订阅
  head_detections_topic: /perception/two_stage/camera_head/result
  arm_detections_topic:  /perception/two_stage/camera_arm/result
  sport_pose_topic: /utlidar/robot_pose

  # 输出
  output_topic: /perception/tracks/unified
  output_frame_id: odom
  output_min_period_ms: 30

  # KF 参数(3 维 state,**单位 m/tick,非 m/s**)
  process_noise_position: 0.05
  observation_noise_position: 0.03

  # 生命周期
  tentative_min_hits: 3
  coast_max_age_ms: 1500

  # 关联参数
  association_distance_gate_m: 0.15
  appearance_class_must_match: true
  two_stage:
    enabled: true
    high_conf_threshold: 0.40
    low_conf_threshold: 0.15

  # odom 缓存
  tf_cache_sec: 30.0
  robot_pose_stale_ms: 500

  # 标定 JSON 路径(空 = 默认 share/.../head_calibration.json)
  head_calibration_file: ""
```

- [ ] **Step 2: 写主节点 `unified_track_node.py`**

`src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py`:

```python
"""unified_track_node 主节点。

订阅:
  - /perception/two_stage/camera_head/result (YoloeDetection3DArray)
  - /perception/two_stage/camera_arm/result  (YoloeDetection3DArray)
  - /utlidar/robot_pose                      (PoseStamped, SLAM)

发布:
  - /perception/tracks/unified                (UnifiedTrackArray, frame_id=odom)

启动期:
  - 读 head_calibration.json,广播 base_link → camera_head_color_optical_frame 静态 TF
运行期:
  - /utlidar/robot_pose callback 广播动态 odom → base_link TF
  - 任一相机 topic 到达触发 pipeline(30ms debounce)
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
)
from tf2_ros import TransformListener
from yoloe_interfaces.msg import YoloeDetection3DArray

from lgw_3d_tracker.unified_track.calibration import broadcast_head_static_tf
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.input_adapter import InputAdapter
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.odom_tf_broadcaster import OdomTFBroadcaster
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.tracker import Tracker


class UnifiedTrackNode(Node):
    def __init__(self):
        super().__init__("unified_track_node")

        # 声明参数
        self.declare_parameter("yaml_config", "")
        yaml_path = self.get_parameter("yaml_config").get_parameter_value().string_value
        if yaml_path:
            self.config = UnifiedTrackConfig.from_yaml(yaml_path)
        else:
            self.config = UnifiedTrackConfig()

        self.get_logger().info(
            f"unified_track_node started; head={self.config.head_detections_topic}, "
            f"arm={self.config.arm_detections_topic}, "
            f"pose={self.config.sport_pose_topic}, "
            f"out={self.config.output_topic}"
        )

        # TF:静态 + 动态 + buffer
        self._head_static_broadcaster = broadcast_head_static_tf(self)
        self._tf_buffer = rclpy.duration.Duration(seconds=self.config.tf_cache_sec)
        # 注意:tf2_ros.Buffer 需要 cache_time 参数;rclpy.duration 转 seconds
        from tf2_ros.buffer import Buffer
        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=self.config.tf_cache_sec))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_tf_broadcaster = OdomTFBroadcaster(self)

        # 算法组件
        kf = KalmanFilter3D(
            sigma_p=self.config.process_noise_position,
            sigma_obs=self.config.observation_noise_position,
        )
        # gate_fn 在 tracker 里被覆盖,这里先用 placeholder
        assigner = TwoStageAssigner(
            high_conf_th=self.config.two_stage.high_conf_threshold,
            low_conf_th=self.config.two_stage.low_conf_threshold,
            gate_fn=lambda s, z: True,
        )
        self._tracker = Tracker(
            kf=kf,
            assigner=assigner,
            tentative_min_hits=self.config.tentative_min_hits,
            coast_max_age_ms=self.config.coast_max_age_ms,
            distance_gate_m=self.config.association_distance_gate_m,
        )
        self._input_adapter = InputAdapter(
            self._tf_buffer, self.get_logger(), lookup_timeout_sec=0.05,
        )
        self._output_formatter = OutputFormatter(frame_id=self.config.output_frame_id)

        # 订阅 / 发布
        qos_default = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_pose = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._sub_head = self.create_subscription(
            YoloeDetection3DArray, self.config.head_detections_topic,
            self._on_head_detection, qos_default)
        self._sub_arm = self.create_subscription(
            YoloeDetection3DArray, self.config.arm_detections_topic,
            self._on_arm_detection, qos_default)
        self._sub_pose = self.create_subscription(
            PoseStamped, self.config.sport_pose_topic,
            self._on_robot_pose, qos_pose)

        self._pub = self.create_publisher(
            UnifiedTrackArray if False else __import__(
                "yoloe_interfaces.msg", fromlist=["UnifiedTrackArray"]
            ).UnifiedTrackArray,
            self.config.output_topic, qos_default)

        # pipeline 调度状态
        self._pending_head: YoloeDetection3DArray = None  # type: ignore
        self._pending_arm: YoloeDetection3DArray = None  # type: ignore
        self._last_run_ns: int = 0
        self._last_detection_ns: int = 0
        self._pending_timer = None
        self._start_time_ns = time.monotonic_ns()
        self._sequence_id = 0

    def _on_robot_pose(self, msg: PoseStamped):
        """运行期 odom→base_link TF 广播。"""
        self._odom_tf_broadcaster.update(msg)
        # 检查 stale:若超过 stale_ms 没新 pose,清空缓存触发 warning
        # TF buffer 自己会处理 lookup 失败,这里仅记录 cache age

    def _on_head_detection(self, msg: YoloeDetection3DArray):
        self._pending_head = msg
        self._request_pipeline()

    def _on_arm_detection(self, msg: YoloeDetection3DArray):
        self._pending_arm = msg
        self._request_pipeline()

    def _request_pipeline(self):
        """调度 pipeline,带 30ms debounce。"""
        now_ns = time.monotonic_ns()
        elapsed_ms = (now_ns - self._last_run_ns) / 1e6
        if elapsed_ms >= self.config.output_min_period_ms:
            self._run_pipeline()
        elif self._pending_timer is None:
            # 起一个一次性 wall_timer
            remaining_ms = self.config.output_min_period_ms - elapsed_ms
            self._pending_timer = self.create_timer(
                remaining_ms / 1000.0, self._on_timer_fire)

    def _on_timer_fire(self):
        if self._pending_timer is not None:
            self._pending_timer.cancel()
            self.destroy_timer(self._pending_timer)
            self._pending_timer = None
        self._run_pipeline()

    def _run_pipeline(self):
        """完整一次 pipeline:input → predict → associate → lifecycle → format → publish。"""
        now_ns = time.monotonic_ns()
        self._last_run_ns = now_ns

        # 收集最新 head + arm(任一可能为 None 表示还没收到)
        all_obs = []
        for msg in (self._pending_head, self._pending_arm):
            if msg is None:
                continue
            all_obs.extend(self._input_adapter.adapt(msg))
        # 清空 pending
        self._pending_head = None
        self._pending_arm = None

        # dt 估计:用 last_run 到 now 的差
        # 注意:first tick 时 last_run 可能是 0,dt 会很大 → 用 max
        if self._last_detection_ns == 0:
            dt_ms = 0
        else:
            dt_ms = max(0, (now_ns - self._last_detection_ns) // 1_000_000)
        self._last_detection_ns = now_ns

        # set tracker 时间
        frame_stamp_ms = (now_ns - self._start_time_ns) // 1_000_000
        self._tracker.set_now_ms(frame_stamp_ms)

        # 流水线
        self._tracker.predict_all()
        self._tracker.associate_and_update(all_obs)
        active_tracks = self._tracker.tick_lifecycle(dt_ms)

        # 输出
        now_msg = self.get_clock().now().to_msg()
        self._sequence_id += 1
        msg_out = self._output_formatter.format(
            tracks=active_tracks,
            sequence_id=self._sequence_id,
            frame_stamp_ms=frame_stamp_ms,
            now=now_msg,
        )
        self._pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

注意:文件顶部还需要 import `PoseStamped` 和 `UnifiedTrackArray`,补:

```python
from geometry_msgs.msg import PoseStamped
from yoloe_interfaces.msg import UnifiedTrackArray
```

把这两行加到主节点文件顶部的 import 段(替换那段 `__import__` 黑魔法)。

- [ ] **Step 3: 写 launch 文件**

`src/lgw_3d_tracker/launch/unified_track_node.launch.py`:

```python
"""Launch unified_track_node with default YAML config."""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lgw_3d_tracker")
    yaml_path = os.path.join(pkg_share, "config", "runtime.yaml")

    return LaunchDescription([
        Node(
            package="lgw_3d_tracker",
            executable="unified_track_node",
            name="unified_track_node",
            output="screen",
            parameters=[{"yaml_config": yaml_path}],
        ),
    ])
```

- [ ] **Step 4: Build**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select lgw_3d_tracker"
```

期望:build 成功,无 error。

- [ ] **Step 5: 验证 entry_point 可用**

```bash
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/lg_walker_ws/install/setup.bash && \
   ros2 pkg executables lgw_3d_tracker"
```

期望:打印 `/home/lg/walker_ws/install/.../lib/python3.10/site-packages/lgw_3d_tracker/unified_track_node`,无 error。

- [ ] **Step 6: 启动节点(不连接话题,验证启动不崩)**

让节点在前台跑 3 秒,如果启动日志含 "Published static TF base_link → camera_head_color_optical_frame" 则 OK;然后 kill -15。

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
   timeout 3 ros2 launch lgw_3d_tracker unified_track_node.launch.py 2>&1 | head -50" || true
```

期望:看到
```
[INFO] [unified_track_node]: Published static TF base_link → camera_head_color_optical_frame
[INFO] [unified_track_node]: unified_track_node started; ...
```
无 traceback。

- [ ] **Step 7: Commit**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py \
          src/lgw_3d_tracker/launch/unified_track_node.launch.py \
          src/lgw_3d_tracker/config/runtime.yaml && \
  git commit -m "feat(lgw_3d_tracker): wire main node + launch + yaml config"
```

---

## Task 13: 全量 build 验证 + 终态检查

**Files:** 无新增,只验证

**Interfaces:** 整个包链路 build + launch 验证

- [ ] **Step 1: 全量 build(包含 yoloe_interfaces + lgw_3d_tracker)**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && colcon build --packages-select yoloe_interfaces lgw_3d_tracker"
```

期望:build 成功,**无 CMake / Python error**。

- [ ] **Step 2: 检查 install 路径**

```bash
docker exec self_l4t bash -c \
  "ls /my_ws/l4t/lg_walker_ws/install/lgw_3d_tracker/lib/python3.10/site-packages/lgw_3d_tracker/unified_track/ && \
   ls /my_ws/l4t/lg_walker_ws/install/lgw_3d_tracker/share/lgw_3d_tracker/"
```

期望:第一个 `ls` 看到 9 个 `.py` 文件(`__init__.py`、`calibration.py`、`config.py`、`cost_matrix.py`、`input_adapter.py`、`kf.py`、`odom_tf_broadcaster.py`、`output_formatter.py`、`track.py`、`tracker.py`)。
第二个 `ls` 看到 `launch/` 和 `config/` 子目录。

- [ ] **Step 3: 检查 install 路径下 msg**

```bash
docker exec self_l4t bash -c \
  "ls /my_ws/l4t/lg_walker_ws/install/yoloe_interfaces/include/yoloe_interfaces/msg/ | grep -i unified"
```

期望:看到 `unified_track.hpp`, `unified_track_array.hpp`, `detail/unified_track__struct.hpp` 等。

- [ ] **Step 4: Final ros2 node info smoke**

(实际真机验证需要 dual_camera_node + livox/utlidar 在线,本 step 仅在容器内验证 entry_point)

```bash
docker exec self_l4t bash -c \
  "source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/lg_walker_ws/install/setup.bash && \
   ros2 pkg executables lgw_3d_tracker && \
   python3 -c 'from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray; print(\"msg OK\")'"
```

期望:看到 entry_point 路径 + `msg OK`。

- [ ] **Step 5: Commit 任何清理(如果有)**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git status && git diff --stat
```

如果有未 commit 的修改(通常是 build artifacts),用 `.gitignore` 过滤(已存在的 `lg_walker_ws/.gitignore` 应已覆盖 build/ install/ log/),手动确认无源码改动后**不**commit。

- [ ] **Step 6: 提交最终清理 commit(如改了 .gitignore)**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws && \
  git add -A && git diff --cached --stat && \
  git commit -m "chore(lgw_3d_tracker): final build verification" || true
```

---

## Self-Review Checklist

执行完成后自检:

1. **Spec coverage**:
   - §0 偏离表:✅ (全在文档)
   - §1 目标 + 边界:✅ Task 1-13
   - §2 包布局:✅ Task 2(setup.py / package.xml),Task 12(主节点)
   - §3 坐标系 + 静态 TF:✅ Task 8(静态),Task 9(动态)
   - §4 Topic 清单:✅ Task 12
   - §5 input_adapter TF lookup at det.stamp:✅ Task 10
   - §6 KF + 两阶段关联 + 生命周期:✅ Task 3(KF), Task 4(track), Task 5(cost_matrix), Task 6(tracker)
   - §7 output_formatter 字段映射:✅ Task 11
   - §8 yaml:✅ Task 12 (Step 1)
   - §9 错误处理:在各模块 error handling 体现
   - §10 测试:本 plan 用 smoke 替代(用户选择)
   - §11 决策:在 spec 里
   - §12 文件清单:✅ 全 tasks 创建
   - §13 风险:在 spec 里

2. **Placeholder scan**: 无 TBD/TODO。

3. **Type consistency**:
   - `TrackObservation.position_odom: np.ndarray (3,)` 在 Task 4 定义,Task 6(tracker)和 Task 10(input_adapter)都用这个签名。✅
   - `Tracker.associate_and_update(observations: List[TrackObservation])` 签名 Task 6 定义,Task 12 调用时传 `List[TrackObservation]`。✅
   - `OutputFormatter.format(tracks, sequence_id, frame_stamp_ms, now)` Task 11 定义,Task 12 调用时传 `active_tracks`, `self._sequence_id`, `frame_stamp_ms`, `now_msg`。✅
   - `UnifiedTrack` / `UnifiedTrackArray` 在 Task 1 定义,Task 11/12 使用。✅

4. **DRY / YAGNI**:
   - 算法层(kf/track/cost_matrix/tracker)是纯 NumPy,与 ROS 解耦,便于未来复用。✅
   - ROS 依赖(input_adapter/output_formatter/odom_tf_broadcaster/calibration)各自单一职责,不需要抽取 base class。✅
   - 没有引入未用到的依赖(没有 ReID、没有 filterpy)。✅

---

## Execution

Plan complete and saved to `docs/superpowers/plans/2026-10-19-lgw-3d-tracker.md`. Two execution options:

1. **Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration
2. **Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?