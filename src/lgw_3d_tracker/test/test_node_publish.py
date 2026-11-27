"""Published contract: only active tracks, live arm-base point, source."""
import numpy as np
from builtin_interfaces.msg import Time

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.track import (
    Track, TrackObservation, TrackState)


def test_publishes_only_active_with_arm_base():
    t = Track(
        id=7, created_ms=0, first_seen_ms=0,
        state=KalmanState3D(np.array([1.0, 0.0, 1.0]), np.eye(3)))
    t.task_state = TrackState.ACTIVE
    t.last_seen_ms = 1000
    t.last_obs_by_camera["camera_arm_color_optical_frame"] = TrackObservation(
        "sample", 0.8, "camera_arm_color_optical_frame", 1000,
        np.array([1.0, 0.0, 1.0]))

    T = np.eye(4)
    T[0, 3] = 1.0
    fmt = OutputFormatter("odom", "arm1/base_link")
    arr = fmt.format(
        [t], 1, 1000, Time(sec=1), T, 500, ["arm", "head"])

    assert len(arr.tracks) == 1
    m = arr.tracks[0]
    assert m.task_state == "active"
    assert m.source == "arm"
    assert m.geometry_valid is True
    assert np.isclose(m.position_arm_base.x, 0.0)
    assert np.isclose(m.position_arm_base.z, 1.0)


def test_geometry_invalid_keeps_last_arm_base():
    t = Track(
        id=8, created_ms=0, first_seen_ms=0,
        state=KalmanState3D(np.array([1.0, 0.0, 1.0]), np.eye(3)))
    t.task_state = TrackState.ACTIVE
    t.last_seen_ms = 1000
    t.last_arm_base = np.array([0.2, 0.0, 0.4])
    t.last_obs_by_camera["camera_head_color_optical_frame"] = TrackObservation(
        "sample", 0.8, "camera_head_color_optical_frame", 1000,
        np.array([1.0, 0.0, 1.0]))

    fmt = OutputFormatter("odom", "arm1/base_link")
    arr = fmt.format([t], 2, 1000, Time(sec=1), None, 500, ["arm", "head"])
    m = arr.tracks[0]
    assert m.geometry_valid is False
    assert np.isclose(m.position_arm_base.x, 0.2)
    assert np.isclose(m.position_arm_base.z, 0.4)


def test_publishes_selection_and_arm_hits():
    t = Track(
        id=7, created_ms=0, first_seen_ms=0,
        state=KalmanState3D(np.zeros(3), np.eye(3) * 0.05))
    t.task_state = TrackState.ACTIVE
    t.record_arm_hit(1000, 3000)
    t.record_arm_hit(1100, 3000)

    fmt = OutputFormatter("odom", "arm1/base_link")
    arr = fmt.format([t], 1, 0, Time(sec=2, nanosec=0), None, 500,
                     ["arm", "head"], selected_track_id=7, arm_window_ms=3000)

    assert arr.selected_track_id == 7
    assert arr.arm_window_ms == 3000
    assert arr.tracks[0].arm_hits_in_window == 2


def test_publishes_selection_and_arm_hits():
    t = Track(
        id=7, created_ms=0, first_seen_ms=0,
        state=KalmanState3D(np.zeros(3), np.eye(3) * 0.05))
    t.task_state = TrackState.ACTIVE
    t.record_arm_hit(1000, 3000)
    t.record_arm_hit(1100, 3000)

    fmt = OutputFormatter("odom", "arm1/base_link")
    arr = fmt.format([t], 1, 0, Time(sec=2, nanosec=0), None, 500,
                     ["arm", "head"], selected_track_id=7, arm_window_ms=3000)

    assert arr.selected_track_id == 7
    assert arr.arm_window_ms == 3000
    assert arr.tracks[0].arm_hits_in_window == 2
