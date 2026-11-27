"""TraceLogger / 关联明细回归测试。

跑法(容器内):
  source /my_ws/l4t/l4t_setup.sh
  source /my_ws/l4t/lg_walker_ws/install/setup.bash
  python3 -m pytest src/lgw_3d_tracker/test/test_trace_logging.py -q
"""
import json

import numpy as np

from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.trace_logger import TraceLogger, cam_label
from lgw_3d_tracker.unified_track.track import TrackObservation, TrackState
from lgw_3d_tracker.unified_track.tracker import Tracker

HEAD = "camera_head_color_optical_frame"
ARM = "camera_arm_color_optical_frame"


def _tracker() -> Tracker:
    kf = KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03)
    assigner = TwoStageAssigner(
        high_conf_th=0.4, low_conf_th=0.15, gate_fn=lambda s, z: True)
    return Tracker(
        kf=kf, assigner=assigner,
        candidate_min_hits=3, candidate_min_age_ms=0,
        candidate_ttl_ms=2000, active_ttl_ms=15000,
        consumed_ttl_ms=60000, distance_gate_m=0.15,
        duplicate_track_distance_m=0.04,
        max_registry_size=100, consumed_suppress_enable=True,
        arm_window_ms=3000, predict_freeze_after_ms=1000)


def _obs(cam: str, x: float, y: float = 0.0, z: float = 0.0,
         conf: float = 0.8) -> TrackObservation:
    return TrackObservation(
        class_name="wood_block", confidence=conf, source_camera=cam,
        stamp_ms=0, position_odom=np.array([x, y, z], dtype=float))


def _confirm_track(tr: Tracker):
    """喂 3 帧同位置 head 观测,把唯一的 track 升到 confirmed。"""
    for _ in range(3):
        tr.predict_all()
        tr.associate_and_update([_obs(HEAD, 0.0)])
        tr.tick_lifecycle(100)
    tracks = tr.tracks
    assert len(tracks) == 1
    assert tracks[0].task_state == TrackState.ACTIVE
    return tracks[0]


def test_cam_label():
    assert cam_label(HEAD) == "head"
    assert cam_label(ARM) == "arm"
    assert cam_label("") == "unknown"


def test_trace_logger_writes_tick_and_summary(tmp_path):
    lines = []
    tl = TraceLogger(str(tmp_path), tag="_t", summary_period_sec=0.0,
                     sink=lines.append)
    tl.log_event("start", foo=1)
    tl.log_tick({
        "seq": 7, "t_ms": 1000, "dt_ms": 100,
        "inputs": [{"cam": "head", "dets": 1, "adapted": 1}],
        "obs": [{"i": 0, "cam": "head", "x": 0.1, "y": 0.2, "z": 0.3}],
        "matches": [{"obs": 0, "track_id": 3, "dist": 0.01}],
        "tentative_matches": [], "created": [], "unmatched": [],
        "tracks": [
            {"id": 3, "task_state": "active", "hits": 9,
             "cams": ["head", "arm"], "x": 0.1, "y": 0.2, "z": 0.3},
            {"id": 4, "task_state": "candidate", "hits": 1,
             "cams": ["arm"], "x": 0.9, "y": 0.0, "z": 0.0},
        ],
    })
    tl.close()

    recs = [json.loads(l) for l in open(tl.jsonl_path, encoding="utf-8")]
    assert recs[0]["event"] == "start"
    assert recs[1]["seq"] == 7
    assert recs[1]["matches"][0]["track_id"] == 3

    summary = open(tl.summary_path, encoding="utf-8").read()
    assert "cross=1" in summary
    assert "single=1" in summary
    assert "id=3 cams=head+arm" in summary
    assert lines and "cross=1" in lines[-1]


def test_association_report_marks_cross_camera_same_track():
    """arm 观测能配到 head 已建立的 confirmed track(同轨迹合并)。"""
    tr = _tracker()
    tr.set_now_ms(0)
    t = _confirm_track(tr)

    tr.predict_all()
    report = tr.associate_and_update([_obs(ARM, 0.02)])
    tr.tick_lifecycle(100)

    assert [m["track_id"] for m in report.matches] == [t.id]
    assert report.created == []
    assert ARM in tr.tracks[0].source_cameras


def test_stale_active_remains_active_and_rematches():
    tr = _tracker()
    tr.set_now_ms(0)
    t = _confirm_track(tr)

    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)
    assert t.task_state == TrackState.ACTIVE

    tr.set_now_ms(6500)
    tr.predict_all()
    tr.associate_and_update([_obs(ARM, 0.01)])
    tr.tick_lifecycle(100)
    assert t.task_state == TrackState.ACTIVE


def test_two_cameras_same_tick_duplicate_is_suppressed():
    """同一 tick 内两相机看到同一目标:Hungarian 一对一只能配一条,
    第二条若在被关联轨迹的去重半径内 → 丢弃,不新建轨迹。
    """
    tr = _tracker()
    tr.set_now_ms(0)
    t = _confirm_track(tr)

    tr.predict_all()
    report = tr.associate_and_update(
        [_obs(HEAD, 0.00), _obs(ARM, 0.02)])
    tr.tick_lifecycle(100)

    assert len(report.matches) == 1
    assert report.created == []
    assert len(report.suppressed) == 1
    assert report.suppressed[0]["track_id"] == t.id
