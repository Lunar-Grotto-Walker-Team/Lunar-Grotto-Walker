"""ObservationResult → List[TrackObservation](odom 系)。

Ruling R6 (2026-10-27):相机几何**全部走 TF**,adapter 不再持有标定矩阵。
节点启动时已把两份标定 JSON 合成为安装桥发布到 TF
(`arm1/base_link → camera_head_link` / `arm1/tcp_link → camera_arm_link`,
见 unified_track_node.py),adapter 直接查这些桥即可避免"矩阵实现与桥实现
各算一遍、可能漂移"。

odom 系(仍分两段,lookup 上界是 base_link):

    head: T_cam_odom = T_base_odom ⊗ T_armbase_base ⊗ T_cam_armbase
    arm : T_cam_odom = T_base_odom ⊗ T_tcp_base      ⊗ T_cam_tcp

  T_base_odom    : tf2 lookup "odom" → "base_link"(机器人本体,唯一 parent=odom)
  T_armbase_base : tf2 lookup "base_link" → "<prefix>base_link"(机械臂基座)
  T_tcp_base     : tf2 lookup "base_link" → "<prefix>tcp_link"
                    (机械臂链,lookup 的上界是 base_link,不会越过它的 parent)
  T_cam_armbase  : tf2 lookup "<prefix>base_link" → camera_head_color_optical_frame
  T_cam_tcp      : tf2 lookup "<prefix>tcp_link"  → camera_arm_color_optical_frame

⚠️ head 相机链的父帧是**机械臂基座**(`<prefix>base_link`),不是机器人本体
的 `base_link`。head 桥的 parent 由节点按 head_calibration.json 的语义设置,
adapter 只按这个帧名查。

为什么不整链 lookup:机械臂 URDF 的 world→base_link fixed joint 与机器人本体的
odom→base_link 抢同一个 child(base_link)的 parent 位置,tf2 解析不确定 →
"Tf has two or more unconnected trees" 随机失败(实测 25%~35%)。分段后每段
lookup 的上界都明确,不再有歧义。

Fix Round 3 (2026-10-19 真机验证): YOLOE 实际发出 ObservationResult(带
status/status_text 的包装),不是 YoloeDetection3DArray。两者结构上共用
YoloeDetection3D[] detections + std_msgs/Header,所以只换 type alias,数据
通道不变。tracker 上下文不再依赖 status enum(空 detections 自然 fail,
与 lgw_bt_planner compute_lookat_pose_node 的"detections.empty() 即 FAILURE"
判断模式一致)。

另外每条观测额外带一份**机械臂基座系**坐标(伺服几何,不含 odom 段),
同样是**单条 TF lookup**,不查 odom、不用标定矩阵:

    head:  p_armbase = lookup(<prefix>base_link ← camera_head_optical) ⊗ p_cam
    arm :  p_armbase = lookup(<prefix>base_link ← camera_arm_optical)  ⊗ p_cam

该相机链在 TF 里不可用时 `position_arm_base = None`(odom 位置照旧)。
"""
from typing import List, Optional

import numpy as np
import rclpy
from tf2_ros.buffer import Buffer

from yoloe_interfaces.msg import ObservationResult

from lgw_3d_tracker.unified_track.calibration import transform_to_matrix
from lgw_3d_tracker.unified_track.track import TrackObservation

HEAD_CAM_FRAME = "camera_head_color_optical_frame"
ARM_CAM_FRAME = "camera_arm_color_optical_frame"


class InputAdapter:
    """把 ObservationResult 转 odom 系观测列表。

    Args:
      tf_buffer: tf2_ros.Buffer 实例(由调用方提供)
      logger: ROS logger(用于 WARN/ERROR)
      lookup_timeout_sec: TF lookup 超时阈值(秒),默认 0.01(19 Hz 位姿半周期
        才 26 ms,10 ms 足够;过大会在回调里阻塞执行器)
      arm_base_frame: 机械臂基座在 TF 里的帧名(带 frame_prefix,如
        "arm1/base_link")
      arm_tcp_frame: 机械臂 tcp_link 在 TF 里的帧名(带 frame_prefix,如
        "arm1/tcp_link")。机械臂启用 frame_prefix 后必须用带前缀的名字。
    """

    def __init__(self, tf_buffer: Buffer, logger, lookup_timeout_sec: float = 0.01,
                 arm_base_frame: str = "base_link",
                 arm_tcp_frame: str = "tcp_link"):
        self._tf_buffer = tf_buffer
        self._logger = logger
        # lookup_transform 的 timeout 形参类型为 rclpy.duration.Duration(C 扩展),
        # 不是 builtin_interfaces.msg.Duration(can_transform 内部 Time + timeout 会
        # 抛 TypeError)。
        self._timeout = rclpy.duration.Duration(seconds=lookup_timeout_sec)
        self._arm_base_frame = arm_base_frame
        self._arm_tcp_frame = arm_tcp_frame

    # ------------------------------------------------------------------
    # TF 分段查询
    # ------------------------------------------------------------------
    def _lookup(self, target: str, source: str, stamp) -> Optional[np.ndarray]:
        """查 target→source 的 4x4 矩阵(把 source 系下的点变到 target 系)。

        先按 detection 的 stamp 查(可与 SLAM pose 插值),失败再退到
        "最新可用"。检测频率(~30Hz)高于 SLAM pose(~19Hz)时,det.stamp 常常
        比 buffer 里最新的 odom 位姿还新几十毫秒,直查会报
        "extrapolation into the future" → 必须有 fallback,否则会丢一半观测。

        **所有路径都先做零超时探测**(can_transform),不再直接
        `lookup_transform(timeout=...)`。原因(2026-10-23 真机实测):

          缺失帧 lookup_transform(timeout=0.05) → 平均 62.7 ms
          缺失帧 can_transform(timeout=0)       → 平均  0.0 ms
          存在帧 lookup_transform(timeout=0.05) → 平均  0.2 ms

        `_cam_to_odom` 每次要走 2~3 段,某一段的帧不存在时(机械臂未启动、
        frame_prefix 配错),旧写法会空等 62.7 ms × 2(stamp + latest)× 2 相机,
        回调直接阻塞上百毫秒 → 同线程执行器被饿死 → `odom→base_link` 从
        19 Hz 掉到 ~4 Hz(实测),track 随之掉到 0。改成零超时探测后,
        "缺帧"这种确定性失败只花 ~0.01 ms,不再拖垮整条流水线。
        """
        zero = rclpy.duration.Duration(seconds=0.0)
        latest = rclpy.time.Time()

        # 1) 确定性失败:帧对根本不在 TF 图里 → 立即返回
        if not self._tf_buffer.can_transform(target, source, latest, timeout=zero):
            self._logger.debug(
                f"TF frames {target}←{source} not connected; skip batch")
            return None

        # 2) 按检测 stamp 查(能否插值取决于该时刻是否有数据)
        t_stamp = rclpy.time.Time.from_msg(stamp)
        if self._tf_buffer.can_transform(target, source, t_stamp, timeout=zero):
            T = self._tf_buffer.lookup_transform(
                target, source, time=t_stamp, timeout=zero)
            return transform_to_matrix(T)

        # 3) 该时刻还没有数据(检测比 SLAM 新)→ 退到最新可用。
        #    保留一个短超时:listener 在独立线程,数据可能刚好在路上。
        try:
            T = self._tf_buffer.lookup_transform(
                target, source, time=latest, timeout=self._timeout)
            return transform_to_matrix(T)
        except Exception as e:
            self._logger.warn(
                f"TF lookup {target}←{source} failed at stamp and latest: "
                f"{e}; skip batch"
            )
            return None

    def _cam_to_odom(self, frame: str, stamp) -> Optional[np.ndarray]:
        """算 T(camera → odom),4x4。失败返回 None。

        两段 TF,相机段直接查节点发布的安装桥(不再用标定矩阵)。
        """
        T_base_odom = self._lookup("odom", "base_link", stamp)
        if T_base_odom is None:
            return None

        if ARM_CAM_FRAME in frame or "camera_arm" in frame:
            T_tcp_base = self._lookup("base_link", self._arm_tcp_frame, stamp)
            if T_tcp_base is None:
                return None
            T_cam_tcp = self._lookup(self._arm_tcp_frame, ARM_CAM_FRAME, stamp)
            if T_cam_tcp is None:
                return None
            return T_base_odom @ T_tcp_base @ T_cam_tcp

        if HEAD_CAM_FRAME in frame or "camera_head" in frame:
            # head 相机桥的 parent 是**机械臂基座**,不是机器人本体 base_link
            T_armbase_base = self._lookup("base_link", self._arm_base_frame, stamp)
            if T_armbase_base is None:
                return None
            T_cam_armbase = self._lookup(
                self._arm_base_frame, HEAD_CAM_FRAME, stamp)
            if T_cam_armbase is None:
                return None
            return T_base_odom @ T_armbase_base @ T_cam_armbase

        self._logger.warn(
            f"未知的相机 frame_id '{frame}',skip batch"
        )
        return None

    def _cam_to_arm_base(self, frame: str, stamp) -> Optional[np.ndarray]:
        """算 T(camera → 机械臂基座),4x4;TF 链不可用时返回 None。

        **单条 TF lookup**(Ruling R6):安装桥已经把标定合成为
        `<arm_base> ← camera_{head,arm}_color_optical_frame`,这里直接查,
        不再用标定矩阵、也不经过 odom。
        """
        if ARM_CAM_FRAME in frame or "camera_arm" in frame:
            optical = ARM_CAM_FRAME
        elif HEAD_CAM_FRAME in frame or "camera_head" in frame:
            optical = HEAD_CAM_FRAME
        else:
            return None
        return self._lookup(self._arm_base_frame, optical, stamp)

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def adapt(self, msg: ObservationResult) -> List[TrackObservation]:
        """对 msg.detections 做逐条变换,返回 TrackObservation 列表。

        Args:
          msg: ObservationResult,header.frame_id 是相机光学系,status/status_text
            仅供参考(本函数不强制 gate,空 detections 自然返回 [])

        Returns:
          有效观测列表(position_valid=True 且 TF 链齐备)
        """
        observations: List[TrackObservation] = []

        frame = str(msg.header.frame_id)
        T_cam_odom = self._cam_to_odom(frame, msg.header.stamp)
        if T_cam_odom is None:
            return []

        R = T_cam_odom[:3, :3]
        t = T_cam_odom[:3, 3]
        # 臂基座链:head 纯标定/arm 查 TF;缺失时该批次 position_arm_base=None
        # (odom 位置照旧发布,不影响 KF)。
        T_cam_armbase = self._cam_to_arm_base(frame, msg.header.stamp)

        stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000

        for det in msg.detections:
            if not det.position_valid:
                continue
            p_cam = np.array([
                float(det.center_m.x), float(det.center_m.y), float(det.center_m.z),
            ])
            p_odom = R @ p_cam + t
            if T_cam_armbase is not None:
                p_armbase = (T_cam_armbase @ np.append(p_cam, 1.0))[:3]
            else:
                p_armbase = None

            observations.append(TrackObservation(
                class_name=str(det.class_name),
                confidence=float(det.confidence),
                source_camera=frame,
                stamp_ms=stamp_ms,
                position_odom=p_odom,
                position_arm_base=p_armbase,
                bbox_2d=det.bbox,
                color=(0.0, 0.0, 0.0),  # YoloeDetection3D 无 color 字段
                size=(float(det.size_m.x), float(det.size_m.y), float(det.size_m.z)),
                size_valid=bool(det.size_valid),
            ))
        return observations
