"""Track[] -> UnifiedTrackArray formatter.

Publishes only the tracker's `active` tracks.  `position` is the odom KF
state; `position_arm_base` is the highest-priority fresh observation
(arm > head) projected through the current TF, or the last known KF state
when neither camera is fresh.  `contributor_*` is gone: the servo consumes
one published point, and the per-camera detail lives in the trace log.
"""
from typing import List, Optional, Sequence

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray

from lgw_3d_tracker.unified_track.geometry_projector import (
    choose_source, project_odom_to_arm_base)
from lgw_3d_tracker.unified_track.track import Track


def _ms_to_time(stamp_ms: int) -> TimeMsg:
    ms = int(stamp_ms)
    return TimeMsg(sec=ms // 1000, nanosec=(ms % 1000) * 1_000_000)


def _to_point(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


class OutputFormatter:
    def __init__(self, frame_id: str, arm_base_frame: str = "arm1/base_link"):
        self._frame_id = frame_id
        self._arm_base_frame = arm_base_frame

    def format(
        self,
        tracks: List[Track],
        sequence_id: int,
        frame_stamp_ms: int,
        now: TimeMsg,
        T_base_odom: Optional[object] = None,
        observation_max_age_ms: int = 500,
        source_priority: Sequence[str] = ("arm", "head"),
        selected_track_id: int = 0,
        arm_window_ms: int = 3000,
    ) -> UnifiedTrackArray:
        msg = UnifiedTrackArray()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.wall_stamp = now
        msg.frame_stamp_ms = frame_stamp_ms
        msg.sequence_id = sequence_id
        msg.selected_track_id = int(selected_track_id)
        msg.arm_window_ms = int(arm_window_ms)
        now_ms = int(now.sec) * 1000 + int(now.nanosec) // 1_000_000
        msg.tracks = [
            self._track_to_msg(
                t, now_ms, T_base_odom, int(observation_max_age_ms),
                list(source_priority), int(arm_window_ms))
            for t in tracks
        ]
        return msg

    def _track_to_msg(
        self,
        t: Track,
        now_ms: int,
        T_base_odom: Optional[object],
        observation_max_age_ms: int,
        source_priority: List[str],
        arm_window_ms: int,
    ) -> UnifiedTrack:
        m = UnifiedTrack()
        m.track_id = int(t.id)
        m.task_state = t.task_state.value
        m.age = int(t.age)
        m.time_since_last_observation_ms = int(t.time_since_last_obs_ms)
        m.arm_hits_in_window = int(
            t.count_recent_arm_hits(now_ms, arm_window_ms))

        source, chosen = choose_source(
            t, now_ms, source_priority, observation_max_age_ms)
        if chosen is not None:
            p_source_odom = chosen.position_odom
            last_seen_ms = int(chosen.stamp_ms)
            diagnostic_obs = chosen
        else:
            p_source_odom = t.state.x
            last_seen_ms = int(t.last_seen_ms)
            diagnostic_obs = t.last_obs

        if diagnostic_obs is not None:
            m.class_name = str(diagnostic_obs.class_name)
            m.confidence = float(diagnostic_obs.confidence)
            if diagnostic_obs.bbox_2d is not None:
                m.last_bbox_2d = diagnostic_obs.bbox_2d
            m.last_source_camera = str(diagnostic_obs.source_camera)
            col = diagnostic_obs.color
            m.last_bbox_color = [float(col[0]), float(col[1]), float(col[2])]
            sz = diagnostic_obs.size
            m.width_m = float(sz[0])
            m.length_m = float(sz[1])
            m.height_m = float(sz[2])
            m.size_valid = bool(diagnostic_obs.size_valid)

        m.observation_count = int(t.hits)
        m.confirmed_hits = int(t.confirmed_hits)

        # odom KF state
        m.position.x = float(t.state.x[0])
        m.position.y = float(t.state.x[1])
        m.position.z = float(t.state.x[2])
        m.position_covariance = [float(v) for v in t.state.P.flatten()]

        m.source_cameras = list(t.source_cameras)
        m.geometry_frame = self._arm_base_frame
        m.source = source

        if T_base_odom is not None:
            p_arm = project_odom_to_arm_base(p_source_odom, T_base_odom)
            m.position_arm_base = _to_point(p_arm)
            m.geometry_valid = True
            t.last_arm_base = p_arm
        else:
            m.geometry_valid = False
            if t.last_arm_base is not None:
                m.position_arm_base = _to_point(t.last_arm_base)

        if last_seen_ms > 0:
            m.last_observation_stamp = _ms_to_time(last_seen_ms)
        return m
