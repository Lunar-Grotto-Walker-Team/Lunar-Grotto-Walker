"""unified_track_node — target registry for the servo stack.

Subscribes:
  - /perception/two_stage/camera_head/result (ObservationResult)
  - /perception/two_stage/camera_arm/result  (ObservationResult)
  - /utlidar/robot_pose                      (PoseStamped, SLAM)

Publishes:
  - /perception/tracks/unified  (UnifiedTrackArray, frame_id=odom)

State updates run on every ObservationResult (no debounce).  Publishing is
event-driven on each update plus a heartbeat that keeps the arm-base
projection current and the servo watchdog alive.
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
)
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from yoloe_interfaces.msg import ObservationResult, UnifiedTrackArray
from yoloe_interfaces.srv import MarkTrack

from lgw_3d_tracker.unified_track.calibration import (
    broadcast_arm_mount_tf, broadcast_camera_mount_tf, calibration_matrix,
    transform_to_matrix,
)
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.input_adapter import InputAdapter
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.odom_tf_broadcaster import OdomTFBroadcaster
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.selection import SelectionParams
from lgw_3d_tracker.unified_track.trace_logger import TraceLogger, cam_label
from lgw_3d_tracker.unified_track.tracker import AssociationReport, Tracker


class UnifiedTrackNode(Node):
    def __init__(self):
        super().__init__("unified_track_node")

        self.declare_parameter("yaml_config", "")
        yaml_path = self.get_parameter("yaml_config").get_parameter_value().string_value
        if yaml_path:
            self.config = UnifiedTrackConfig.from_yaml(yaml_path)
        else:
            self.config = UnifiedTrackConfig()

        self.declare_parameter("trace_log_dir", "")
        self.declare_parameter("trace_tag", "")
        self.declare_parameter("trace_summary_period_sec", 5.0)
        trace_dir = self.get_parameter("trace_log_dir").value
        trace_tag = self.get_parameter("trace_tag").value
        trace_period = float(
            self.get_parameter("trace_summary_period_sec").value)
        self._trace = None
        if trace_dir:
            self._trace = TraceLogger(
                trace_dir, tag=trace_tag, summary_period_sec=trace_period,
                sink=lambda s: self.get_logger().info(s),
            )
            self._trace.log_event(
                "start",
                yaml_config=yaml_path,
                gate_m=self.config.association_distance_gate_m,
                output_topic=self.config.output_topic,
                jsonl_path=self._trace.jsonl_path,
            )
            self.get_logger().info(
                f"trace logging ON: {self._trace.jsonl_path}")

        self.get_logger().info(
            f"unified_track_node started; head={self.config.head_detections_topic}, "
            f"arm={self.config.arm_detections_topic}, "
            f"pose={self.config.sport_pose_topic}, "
            f"out={self.config.output_topic}"
        )

        # TF buffer + static mount bridges (order matters: buffer first).
        self._tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=self.config.tf_cache_sec)
        )
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_tf_broadcaster = OdomTFBroadcaster(self)

        self._arm_mount_broadcaster = broadcast_arm_mount_tf(
            self,
            frame_prefix=self.config.arm_frame_prefix,
            offset_xyz=self.config.arm_mount_offset[:3],
            offset_rpy=self.config.arm_mount_offset[3:6],
        )

        self._head_mount_broadcaster, self._head_mount_timer = broadcast_camera_mount_tf(
            self, self._tf_buffer,
            cam_to_parent=calibration_matrix(
                "head", self.config.head_calibration_file),
            parent_frame=f"{self.config.arm_frame_prefix}base_link",
            link_frame="camera_head_link",
            optical_frame="camera_head_color_optical_frame",
        )
        self._arm_cam_mount_broadcaster, self._arm_cam_mount_timer = \
            broadcast_camera_mount_tf(
                self, self._tf_buffer,
                cam_to_parent=calibration_matrix("arm"),
                parent_frame=f"{self.config.arm_frame_prefix}tcp_link",
                link_frame="camera_arm_link",
                optical_frame="camera_arm_color_optical_frame",
            )

        # Algorithm components.
        kf = KalmanFilter3D(
            sigma_p=self.config.process_noise_position,
            sigma_obs=self.config.observation_noise_position,
        )
        assigner = TwoStageAssigner(
            high_conf_th=self.config.two_stage.high_conf_threshold,
            low_conf_th=self.config.two_stage.low_conf_threshold,
            gate_fn=lambda s, z: True,
        )
        self._tracker = Tracker(
            kf=kf,
            assigner=assigner,
            candidate_min_hits=self.config.candidate_min_hits,
            candidate_min_age_ms=self.config.candidate_min_age_ms,
            candidate_ttl_ms=self.config.candidate_ttl_ms,
            active_ttl_ms=self.config.active_ttl_ms,
            consumed_ttl_ms=self.config.consumed_ttl_ms,
            distance_gate_m=self.config.association_distance_gate_m,
            duplicate_track_distance_m=self.config.duplicate_track_distance_m,
            max_registry_size=self.config.max_registry_size,
            consumed_suppress_enable=self.config.consumed_suppress_enable,
            consumed_suppress_radius_m=self.config.consumed_suppress_radius_m,
            consumed_suppress_ttl_ms=self.config.consumed_suppress_ttl_ms,
            arm_window_ms=self.config.arm_window_ms,
            predict_freeze_after_ms=self.config.predict_freeze_after_ms,
            max_track_distance_m=self.config.max_track_distance_m,
            base_exclusion_radius_m=self.config.base_exclusion_radius_m,
            selected_max_unseen_ms=self.config.selected_max_unseen_ms,
            selection=SelectionParams(
                class_filter=self.config.select_class_filter,
                min_distance_m=self.config.select_min_distance_m,
                max_distance_m=self.config.select_max_distance_m,
                fresh_ms=self.config.selection_fresh_ms,
            ),
        )
        self._input_adapter = InputAdapter(
            self._tf_buffer, self.get_logger(), lookup_timeout_sec=0.01,
            arm_base_frame=f"{self.config.arm_frame_prefix}base_link",
            arm_tcp_frame=f"{self.config.arm_frame_prefix}tcp_link",
        )
        self._output_formatter = OutputFormatter(
            frame_id=self.config.output_frame_id,
            arm_base_frame=f"{self.config.arm_frame_prefix}base_link",
        )

        qos_default = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_pose = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._sub_head = self.create_subscription(
            ObservationResult, self.config.head_detections_topic,
            self._on_head_detection, qos_default)
        self._sub_arm = self.create_subscription(
            ObservationResult, self.config.arm_detections_topic,
            self._on_arm_detection, qos_default)
        self._sub_pose = self.create_subscription(
            PoseStamped, self.config.sport_pose_topic,
            self._on_robot_pose, qos_pose)

        self._pub = self.create_publisher(
            UnifiedTrackArray,
            self.config.output_topic, qos_default)

        self._mark_srv = self.create_service(
            MarkTrack, "/lgw/tracker/mark_track", self._on_mark_track)

        self._start_time_ns = time.monotonic_ns()
        self._last_update_ms = 0
        self._sequence_id = 0
        heartbeat_hz = float(self.config.publish_heartbeat_hz)
        if heartbeat_hz < 2.0:
            self.get_logger().warn(
                f"publish_heartbeat_hz={heartbeat_hz} is below 2 Hz; clamped "
                "to 2 Hz so the servo stream watchdog (500 ms) cannot fire")
            heartbeat_hz = 2.0
        self._heartbeat_timer = self.create_timer(
            1.0 / heartbeat_hz, self._heartbeat)

    # ------------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------------
    def _on_robot_pose(self, msg: PoseStamped):
        self._odom_tf_broadcaster.update(msg)

    def _on_head_detection(self, msg: ObservationResult):
        self._process_observation(msg, "head")

    def _on_arm_detection(self, msg: ObservationResult):
        self._process_observation(msg, "arm")

    def _process_observation(self, msg: ObservationResult, cam: str):
        now_ms = self.get_clock().now().nanoseconds // 1_000_000
        frame_stamp_ms = (time.monotonic_ns() - self._start_time_ns) // 1_000_000
        dt_ms = 0 if self._last_update_ms == 0 else max(
            0, now_ms - self._last_update_ms)

        observations = self._input_adapter.adapt(msg)
        trace_input = {
            "cam": cam,
            "frame_id": str(msg.header.frame_id),
            "dets": len(msg.detections),
            "adapted": len(observations),
        }

        self._tracker.set_now_ms(now_ms)
        arm_base_odom = self._lookup_arm_base_odom()
        self._tracker.set_arm_base_position(arm_base_odom)
        self._tracker.predict_all()
        report = self._tracker.associate_and_update(observations)
        active = self._tracker.tick_lifecycle(dt_ms)
        self._last_update_ms = now_ms

        self._publish_active(active, frame_stamp_ms)
        if self._trace is not None:
            self._log_trace(
                frame_stamp_ms, dt_ms, [trace_input], observations, report,
                active)

    def _on_mark_track(self, request, response):
        ok = self._tracker.mark_track(int(request.track_id), int(request.action))
        if ok:
            # Publish immediately so a following takeover sees the new
            # selected_track_id without waiting for the next heartbeat.
            self._publish_active()
        if ok and int(request.action) == 6:
            center = self._tracker.base_center_odom
            if center is not None:
                self.get_logger().info(
                    f"base circle pinned at odom x={center[0]:.2f} "
                    f"y={center[1]:.2f} "
                    f"(radius={self.config.base_exclusion_radius_m:.2f} m)")
        response.success = bool(ok)
        response.message = (
            f"track {request.track_id} action {request.action} "
            f"{'accepted' if ok else 'rejected'}")
        return response

    # ------------------------------------------------------------------
    # output
    # ------------------------------------------------------------------
    def _lookup_arm_base_odom(self):
        """Arm base origin in odom, or None when it cannot be trusted.

        Selection ranks targets by XoY distance to this origin.  A missing or
        stale transform returns None, which makes the tracker keep its current
        selection instead of ranking with a bad prior.
        """
        target = self.config.output_frame_id
        source = f"{self.config.arm_frame_prefix}base_link"
        zero = rclpy.duration.Duration(seconds=0.0)
        latest = rclpy.time.Time()
        if not self._tf_buffer.can_transform(target, source, latest, timeout=zero):
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                target, source, time=latest, timeout=zero)
        except Exception as exc:  # noqa: BLE001 - a missing TF is not fatal
            self.get_logger().debug(f"odom←{source} lookup failed: {exc}")
            return None
        stamp = transform.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            now_ns = self.get_clock().now().nanoseconds
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if (now_ns - stamp_ns) // 1_000_000 > self.config.robot_pose_stale_ms:
                return None
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def _lookup_odom_base(self):
        zero = rclpy.duration.Duration(seconds=0.0)
        latest = rclpy.time.Time()
        if not self._tf_buffer.can_transform(
                self.config.output_frame_id, "base_link", latest, timeout=zero):
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self.config.output_frame_id, "base_link",
                time=latest, timeout=zero)
            return transform_to_matrix(transform)
        except Exception as exc:  # noqa: BLE001 - a missing TF is not fatal
            self.get_logger().debug(f"odom←base_link lookup failed: {exc}")
            return None

    def _publish_active(self, active=None, frame_stamp_ms=None):
        now = self.get_clock().now()
        if frame_stamp_ms is None:
            frame_stamp_ms = (time.monotonic_ns() - self._start_time_ns) // 1_000_000
        if active is None:
            active = self._tracker.active_tracks()
        T_base_odom = self._lookup_odom_base()
        self._sequence_id += 1
        msg = self._output_formatter.format(
            tracks=active,
            sequence_id=self._sequence_id,
            frame_stamp_ms=frame_stamp_ms,
            now=now.to_msg(),
            T_base_odom=T_base_odom,
            observation_max_age_ms=self.config.observation_max_age_ms,
            source_priority=self.config.source_priority,
            selected_track_id=self._tracker.selected_track_id,
            arm_window_ms=self.config.arm_window_ms,
        )
        self._pub.publish(msg)

    def _heartbeat(self):
        # Advance the lifecycle even when no observation arrived. Without
        # this, a dead detection pipeline leaves the tracker publishing its
        # last active tracks forever and the servo watchdog (message arrival)
        # never fires.
        now_ms = self.get_clock().now().nanoseconds // 1_000_000
        dt_ms = 0 if self._last_update_ms == 0 else max(
            0, now_ms - self._last_update_ms)
        self._tracker.set_now_ms(now_ms)
        arm_base_odom = self._lookup_arm_base_odom()
        self._tracker.set_arm_base_position(arm_base_odom)
        self._tracker.tick_lifecycle(dt_ms)
        self._last_update_ms = now_ms
        self._publish_active()

    # ------------------------------------------------------------------
    # trace
    # ------------------------------------------------------------------
    def _log_trace(self, frame_stamp_ms: int, dt_ms: int, inputs, observations,
                   report: AssociationReport, tracks) -> None:
        try:
            self._trace.log_tick({
                "seq": int(self._sequence_id),
                "t_ms": int(frame_stamp_ms),
                "dt_ms": int(dt_ms),
                "inputs": inputs,
                "obs": [{
                    "i": i,
                    "cam": cam_label(o.source_camera),
                    "frame_id": o.source_camera,
                    "cls": o.class_name,
                    "conf": round(float(o.confidence), 3),
                    "x": round(float(o.position_odom[0]), 4),
                    "y": round(float(o.position_odom[1]), 4),
                    "z": round(float(o.position_odom[2]), 4),
                    "stamp_ms": int(o.stamp_ms),
                } for i, o in enumerate(observations)],
                "matches": report.matches,
                "tentative_matches": report.tentative_matches,
                "reid_matches": report.reid_matches,
                "created": report.created,
                "unmatched": report.unmatched,
                "suppressed": report.suppressed,
                # 2026-11-18: 建轨门限丢弃的观测(arm_base_distance / odom_origin)
                "rejected": report.rejected,
                "dedupe_removed": self._tracker.last_dedupe_removed,
                "tracks": [self._track_snapshot(t) for t in tracks],
            })
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_trace_error_logged", False):
                self._trace_error_logged = True
                self.get_logger().warn(f"trace logging failed: {e}")

    @staticmethod
    def _track_snapshot(t) -> dict:
        obs = t.last_obs
        return {
            "id": int(t.id),
            "task_state": t.task_state.value,
            "hits": int(t.hits),
            "cams": [cam_label(c) for c in t.source_cameras],
            "last_cam": cam_label(obs.source_camera) if obs else "",
            "cls": obs.class_name if obs else "",
            "conf": round(float(obs.confidence), 3) if obs else 0.0,
            "x": round(float(t.state.x[0]), 4),
            "y": round(float(t.state.x[1]), 4),
            "z": round(float(t.state.x[2]), 4),
            "tsi_ms": int(t.time_since_last_obs_ms),
        }

    def shutdown(self) -> None:
        if self._trace is not None:
            self._trace.log_event("stop", ticks=self._trace._ticks)
            self._trace.close()
            self._trace = None


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
