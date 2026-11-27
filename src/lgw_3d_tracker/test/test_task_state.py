"""Tracker task-state lifecycle (2026-11-17: lost retired, 15 s rolling TTL)."""
import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.track import Track, TrackState

TICK = dict(candidate_min_hits=3, candidate_min_age_ms=0, candidate_ttl_ms=2000,
            active_ttl_ms=15000, consumed_ttl_ms=60000)


def test_selected_track_is_exempt_from_active_ttl():
    """2026-11-19: 被选中的轨迹不因 active TTL 归档(id 不能在任务中途滚动)。"""
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(0, 10, **TICK)                 # 0 ms 升级为 ACTIVE
    t.last_seen_ms = 30000                # 一直在被观测
    t.tick(30000, 10, is_selected=True, selected_max_unseen_ms=60000, **TICK)
    assert t.task_state == TrackState.ACTIVE

    # 未选中时同样的时间点会归档
    t.tick(30001, 10, is_selected=False, selected_max_unseen_ms=60000, **TICK)
    assert t.task_state == TrackState.ARCHIVED


def test_selected_exemption_is_bounded_by_unseen_cap():
    """豁免有上限: 选中后长时间没被观测仍然退役。"""
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(0, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE

    # last_seen 停在 1000, 现在 62000 -> 连续 61 s 没观测(阈值取严格大于)
    t.last_seen_ms = 1000
    t.tick(62000, 10, is_selected=True, selected_max_unseen_ms=60000, **TICK)
    assert t.task_state == TrackState.ARCHIVED


def make_track(**kw):
    defaults = dict(id=1, created_ms=0, first_seen_ms=0, last_seen_ms=0,
                    state=KalmanState3D(x=np.zeros(3), P=np.eye(3) * 0.05))
    defaults.update(kw)
    return Track(**defaults)


def test_states_have_no_lost():
    assert [s.value for s in TrackState] == ["candidate", "active", "consumed", "archived"]


def test_candidate_promotes_at_three_hits():
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(100, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE


def test_candidate_archives_after_ttl():
    t = make_track(task_state=TrackState.CANDIDATE, hits=1, created_ms=0)
    t.tick(2001, 10, **TICK)
    assert t.task_state == TrackState.ARCHIVED


def test_active_archives_15s_after_promotion():
    # 2026-11-18: 活跃期从升级时刻起算, 新观测不再续命。
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(0, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE
    assert t.active_since_ms == 0
    t.tick(15000, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE
    t.tick(15001, 10, **TICK)
    assert t.task_state == TrackState.ARCHIVED


def test_fresh_observations_do_not_extend_active_life():
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(0, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE
    # 观测一直很新, 但 active_since 不动 -> 到点仍然失活
    for now in (5000, 10000, 14000):
        t.last_seen_ms = now
        t.tick(now, 10, **TICK)
        assert t.task_state == TrackState.ACTIVE
    t.last_seen_ms = 16000
    t.tick(16000, 10, **TICK)
    assert t.task_state == TrackState.ARCHIVED


def test_active_retirement_never_marks_ignored():
    """失活只到 ARCHIVED; 不写 consumed_reason, 不进 CONSUMED。"""
    t = make_track(task_state=TrackState.CANDIDATE, hits=3)
    t.tick(0, 10, **TICK)
    t.tick(16000, 10, **TICK)
    assert t.task_state == TrackState.ARCHIVED
    assert t.consumed_reason == ""


def test_consumed_archives_from_consumed_since_not_last_seen():
    t = make_track(task_state=TrackState.CONSUMED, last_seen_ms=0,
                   consumed_since_ms=50000)
    t.tick(100000, 10, **TICK)   # consumed 后 50 s
    assert t.task_state == TrackState.CONSUMED
    t.tick(110001, 10, **TICK)   # consumed 后 60+ s
    assert t.task_state == TrackState.ARCHIVED


def test_arm_hits_prune_outside_window():
    t = make_track()
    t.record_arm_hit(1000, 3000)
    t.record_arm_hit(2000, 3000)
    assert t.count_recent_arm_hits(2500, 3000) == 2
    t.record_arm_hit(5000, 3000)
    assert t.count_recent_arm_hits(5000, 3000) == 2


def test_arm_hits_are_independent_of_last_seen():
    t = make_track(task_state=TrackState.ACTIVE, last_seen_ms=0)
    t.record_arm_hit(9000, 3000)
    assert t.count_recent_arm_hits(9000, 3000) == 1
    assert t.time_since_last_obs_ms == 0
