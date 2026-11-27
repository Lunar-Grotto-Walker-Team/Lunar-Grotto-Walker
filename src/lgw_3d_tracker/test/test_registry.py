"""Global registry: lost retention, re-ID, consumed, capacity (2026-11-13)."""
import numpy as np

from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.track import TrackObservation, TrackState
from lgw_3d_tracker.unified_track.tracker import Tracker

HEAD = "camera_head_color_optical_frame"
ARM = "camera_arm_color_optical_frame"


def _tracker():
    return Tracker(
        kf=KalmanFilter3D(0.05, 0.03),
        assigner=TwoStageAssigner(0.4, 0.15, lambda s, z: True),
        candidate_min_hits=3, candidate_min_age_ms=0,
        candidate_ttl_ms=2000, active_ttl_ms=15000,
        consumed_ttl_ms=60000, distance_gate_m=0.15,
        duplicate_track_distance_m=0.04,
        max_registry_size=100, consumed_suppress_enable=True,
        arm_window_ms=3000, predict_freeze_after_ms=1000)


def _obs(x, cam="camera_arm_color_optical_frame", cls="sample"):
    return TrackObservation(cls, 0.8, cam, 0, np.array([x, 0.0, 1.0]))


def _feed(tr, x=0.0, n=3):
    for _ in range(n):
        tr.set_now_ms(0)
        tr.predict_all()
        tr.associate_and_update([_obs(x)])
        tr.tick_lifecycle(100)


def test_stale_active_is_retained_and_rematched():
    tr = _tracker()
    _feed(tr)
    tid = tr.active_tracks()[0].id
    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)
    assert [t.id for t in tr.active_tracks()] == [tid]

    tr.set_now_ms(7000)
    tr.predict_all()
    tr.associate_and_update([_obs(0.05)])
    tr.tick_lifecycle(100)
    assert [t.id for t in tr.active_tracks()] == [tid]


def test_far_object_creates_new_candidate():
    tr = _tracker()
    _feed(tr)
    old_id = tr.all_tracks()[0].id
    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)

    tr.set_now_ms(7000)
    tr.predict_all()
    tr.associate_and_update([_obs(1.0)])   # 1 m away, outside 0.15 m gate
    tr.tick_lifecycle(100)

    ids = [t.id for t in tr.all_tracks()]
    assert old_id in ids
    assert any(i != old_id for i in ids)


def test_mark_consumed_removes_from_active_and_blocks_reid():
    tr = _tracker()
    _feed(tr)
    tid = tr.active_tracks()[0].id
    assert tr.mark_track(tid, 1) is True
    assert tr.active_tracks() == []

    tr.set_now_ms(7000)
    tr.predict_all()
    tr.associate_and_update([_obs(0.0)])
    tr.tick_lifecycle(100)
    assert all(t.id != tid for t in tr.active_tracks())


def _tracker_gated(max_d=5.0, base_r=2.5, base_center=None):
    tr = _tracker()
    tr.max_track_distance_m = max_d
    tr.base_exclusion_radius_m = base_r
    if base_center is not None:
        tr.set_base_center(np.asarray(base_center, dtype=float))
    return tr


def test_far_observation_from_arm_base_is_dropped():
    """距机械臂基座 > 5 m 的观测不建轨(2026-11-18)。"""
    tr = _tracker_gated()
    tr.set_arm_base_position(np.array([0.0, 0.0, 0.0]))
    tr.set_now_ms(0)
    rep = tr.associate_and_update([_obs(6.0)])   # [6, 0, 1] -> 6.08 m
    assert rep.rejected and rep.rejected[0]["reason"] == "arm_base_distance"
    assert tr.all_tracks() == []


def test_observation_inside_base_circle_is_dropped():
    """与基地圆心 XoY 距离 < 2.5 m 的观测不建轨(2026-11-18)。

    基地圆心 = 任务启动时机器人的 odom 位置, 不是 odom 原点。
    """
    tr = _tracker_gated(base_center=[0.0, 0.0, 0.0])
    tr.set_now_ms(0)
    rep = tr.associate_and_update([_obs(1.0)])   # [1, 0, 1] -> XoY 1.0 m
    assert rep.rejected and rep.rejected[0]["reason"] == "base_circle"
    assert tr.all_tracks() == []


def test_base_circle_follows_pinned_center_not_odom_origin():
    """圆心钉在 (10, 0, 0) 时, 原点附近的观测不受屏蔽。"""
    tr = _tracker_gated(base_center=[10.0, 0.0, 0.0])
    # 不设 arm base -> 5 m 门限关闭, 只验基地屏蔽圈
    tr.set_now_ms(0)
    tr.associate_and_update([_obs(1.0)])          # 离基地圆心 XoY 9 m -> 保留
    assert tr.all_tracks()


def test_observation_outside_both_gates_still_creates_track():
    tr = _tracker_gated(base_center=[0.0, 0.0, 0.0])
    tr.set_arm_base_position(np.array([0.0, 0.0, 0.0]))
    _feed(tr, x=3.0)     # XoY 3 m > 2.5, 距基座 3.16 m < 5
    assert tr.all_tracks()
    assert tr.active_tracks()


def test_mark_pin_base_center_uses_current_arm_base():
    """action=6: 把基地圆心钉在当前机械臂基座 odom 位置(2026-11-18)。"""
    tr = _tracker_gated(base_r=2.5)
    assert tr.base_center_odom is None
    # 还没拿到臂基座位姿 -> 拒绝
    assert tr.mark_track(0, 6) is False
    assert tr.base_center_odom is None

    tr.set_arm_base_position(np.array([10.0, 2.0, 0.5]))
    assert tr.mark_track(0, 6) is True
    np.testing.assert_allclose(tr.base_center_odom, [10.0, 2.0, 0.5])

    # 钉住后: 圆心 1 m 内的观测被丢, 9 m 外的保留
    tr.set_now_ms(0)
    rep = tr.associate_and_update([_obs(9.0)])   # XoY 与圆心差 1 m
    assert rep.rejected and rep.rejected[0]["reason"] == "base_circle"


def test_mark_clear_arm_hits_keeps_state_and_selection():
    """action=5 只清命中历史; 不消费、不清选择(2026-11-18)。"""
    tr = _tracker()
    _feed(tr)
    t = tr.active_tracks()[0]
    t.arm_hit_stamps_ms.clear()          # _feed() 已经记过命中, 先归零
    t.record_arm_hit(1000, 3000)
    t.record_arm_hit(2000, 3000)
    assert t.count_recent_arm_hits(2000, 3000) == 2

    assert tr.mark_track(t.id, 5) is True
    assert t.count_recent_arm_hits(2000, 3000) == 0
    assert t.task_state.value == "active"
    assert any(x.id == t.id for x in tr.active_tracks())


def test_registry_capacity_evicts_oldest_lost_then_candidate():
    tr = _tracker()
    tr.max_registry_size = 2
    for x in (0.0, 1.0, 2.0):
        tr.set_now_ms(0)
        tr.predict_all()
        tr.associate_and_update([_obs(x)])
        tr.tick_lifecycle(100)
    assert len(tr.all_tracks()) <= 2


def test_wrong_class_does_not_match():
    tr = _tracker()
    _feed(tr)
    old_id = tr.active_tracks()[0].id
    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)

    tr.set_now_ms(7000)
    tr.predict_all()
    report = tr.associate_and_update([_obs(0.05, cls="other")])
    tr.tick_lifecycle(100)

    assert [m["track_id"] for m in report.matches] == []
    assert any(t.id != old_id for t in tr.all_tracks())


def test_active_archives_after_active_ttl():
    tr = _tracker()
    _feed(tr)
    old_id = tr.all_tracks()[0].id
    tr.set_now_ms(15001)            # past active_ttl_ms=15000
    tr.tick_lifecycle(100)

    assert all(t.id != old_id for t in tr.all_tracks())


def test_two_cameras_same_target_single_active():
    """Focus 1: both cameras update one track; no duplicate candidate."""
    tr = _tracker()
    for _ in range(3):
        tr.set_now_ms(0)
        tr.predict_all()
        tr.associate_and_update([_obs(0.0, cam=ARM)])
        tr.tick_lifecycle(100)
        tr.predict_all()
        tr.associate_and_update([_obs(0.02, cam=HEAD)])
        tr.tick_lifecycle(100)

    active = tr.active_tracks()
    assert len(active) == 1
    assert active[0].hits == 6
