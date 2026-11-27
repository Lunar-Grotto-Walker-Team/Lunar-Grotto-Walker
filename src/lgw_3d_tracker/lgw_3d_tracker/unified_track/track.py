"""Track dataclass + task-state machine (2026-11-17: lost retired).

States: candidate -> active -> archived, plus terminal consumed.  `candidate`
is the false-positive gate; `active` is the only published state and lives
exactly `active_ttl_ms` after it was promoted; `consumed` is set by the task
layer and never auto-revives.  Visibility is a field
(`time_since_last_obs_ms`), not a state.

2026-11-18: the active lifetime is measured from the promotion timestamp
(`active_since_ms`), NOT from the last observation.  Fresh observations no
longer extend it — a track is retired 15 s after it became active no matter
how often it is seen.  Retirement sets ARCHIVED, never CONSUMED, so the
object is not suppressed: if it is still being detected, a brand-new track
(new id) is created and becomes selectable again.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D


class TrackState(str, Enum):
    CANDIDATE = "candidate"
    ACTIVE = "active"
    CONSUMED = "consumed"
    ARCHIVED = "archived"


@dataclass
class TrackObservation:
    """One detection observation, accumulated across ticks."""
    class_name: str
    confidence: float
    source_camera: str
    stamp_ms: int  # detection.header.stamp epoch ms
    position_odom: np.ndarray  # (3,)
    # (3,) arm base coordinates; None when that camera's TF chain is missing.
    position_arm_base: Optional[np.ndarray] = None
    bbox_2d: Optional[object] = None  # sensor_msgs/RegionOfInterest
    color: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.0, 0.0, 0.0)  # (w, l, h)
    size_valid: bool = False


@dataclass
class Track:
    """One registered target.

    `task_state` is the single lifecycle/task state.  `state` remains the
    Kalman state (odom frame); the two names are deliberately different.
    `last_seen_ms` / `first_seen_ms` / `created_ms` are wall-clock epoch ms.
    """
    id: int
    created_ms: int
    task_state: TrackState = TrackState.CANDIDATE
    first_seen_ms: int = 0
    last_seen_ms: int = 0
    state: KalmanState3D = field(default_factory=lambda: KalmanState3D(
        x=np.zeros(3), P=np.eye(3) * 0.05
    ))
    hits: int = 0
    confirmed_hits: int = 0
    time_since_last_obs_ms: int = 0
    age: int = 0
    last_obs: Optional[TrackObservation] = None
    last_obs_by_camera: Dict[str, TrackObservation] = field(default_factory=dict)
    source_cameras: List[str] = field(default_factory=list)
    # Last valid arm-base projection, kept for TF dropouts.
    last_arm_base: Optional[np.ndarray] = None
    consumed_reason: str = ""
    # Wall-clock ms when the task layer set consumed/ignored.  Suppression and
    # consumed retention count from here, never from last_seen_ms.
    consumed_since_ms: int = 0
    # Wall-clock ms of the candidate -> active promotion.  Active lifetime is
    # measured from here (2026-11-18), not from last_seen_ms: new observations
    # do not extend it.
    active_since_ms: int = 0
    # Wall-clock ms of each arm-camera hit, pruned to the verify window.
    arm_hit_stamps_ms: List[int] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.last_obs is not None:
            self.last_obs_by_camera.setdefault(
                self.last_obs.source_camera, self.last_obs)

    def apply_observation(self, obs: TrackObservation, kf_update_state) -> None:
        """Associate one observation: KF update + observation bookkeeping.

        The tracker sets `last_seen_ms` after calling this method; the Track
        itself has no clock.
        """
        self.state = kf_update_state(self.state, obs.position_odom)
        self.last_obs = obs
        self.last_obs_by_camera[obs.source_camera] = obs
        self.hits += 1
        self.time_since_last_obs_ms = 0
        if self.task_state == TrackState.ACTIVE:
            self.confirmed_hits += 1
        if obs.source_camera not in self.source_cameras:
            self.source_cameras.append(obs.source_camera)
        if obs.position_arm_base is not None:
            self.last_arm_base = np.asarray(obs.position_arm_base, dtype=float)

    def refresh(self, task_state: TrackState, now_ms: int) -> None:
        """Externally set the task state (consumed/ignored/reactivate/re-ID)."""
        self.task_state = task_state
        self.last_seen_ms = now_ms

    def record_arm_hit(self, now_ms: int, window_ms: int) -> None:
        """Record one arm-camera hit and drop entries outside the window."""
        self.arm_hit_stamps_ms.append(int(now_ms))
        self._prune_arm_hits(now_ms, window_ms)

    def count_recent_arm_hits(self, now_ms: int, window_ms: int) -> int:
        """Arm-camera hits at or after ``now_ms - window_ms``."""
        self._prune_arm_hits(now_ms, window_ms)
        return len(self.arm_hit_stamps_ms)

    def _prune_arm_hits(self, now_ms: int, window_ms: int) -> None:
        cutoff = int(now_ms) - int(window_ms)
        if self.arm_hit_stamps_ms:
            self.arm_hit_stamps_ms = [
                stamp for stamp in self.arm_hit_stamps_ms if stamp >= cutoff]

    def tick(
        self,
        now_ms: int,
        dt_ms: int,
        candidate_min_hits: int,
        candidate_min_age_ms: int,
        candidate_ttl_ms: int,
        active_ttl_ms: int,
        consumed_ttl_ms: int,
        is_selected: bool = False,
        selected_max_unseen_ms: int = 0,
    ) -> None:
        """Advance the state machine once.  Times are wall-clock epoch ms.

        `is_selected` marks the track the task layer is working on
        (2026-11-19): a selected track is EXEMPT from the active TTL so its id
        cannot roll over mid-task, which is what made {track_id} in the BT go
        stale.  The exemption is bounded by `selected_max_unseen_ms`: once the
        target has not been observed for that long it retires anyway, so a
        target that was taken away cannot live forever.
        """
        self.time_since_last_obs_ms += dt_ms
        self.age += 1

        if self.task_state == TrackState.CANDIDATE:
            if (self.hits >= candidate_min_hits
                    and now_ms - self.first_seen_ms >= candidate_min_age_ms):
                self.task_state = TrackState.ACTIVE
                self.active_since_ms = now_ms
                self.confirmed_hits = 0
            elif now_ms - self.created_ms > candidate_ttl_ms:
                self.task_state = TrackState.ARCHIVED
        elif self.task_state == TrackState.ACTIVE:
            # 2026-11-18: 从"升级为 ACTIVE 的时刻"起算, 新观测不再续命。
            # 失活只到 ARCHIVED, 不进 CONSUMED —— 不会被 consumed_suppress
            # 当成 ignore 抑制, 物体仍被检测到时会以新 id 重新建轨。
            # 2026-11-19: 但"被选中的目标"豁免这条, 否则抓取中途 id 滚动,
            # BT 里的 {track_id} 就成了悬空值。
            if not is_selected and now_ms - self.active_since_ms > active_ttl_ms:
                self.task_state = TrackState.ARCHIVED
            elif (selected_max_unseen_ms > 0
                    and now_ms - self.last_seen_ms > selected_max_unseen_ms):
                # 豁免不是无限的: 长时间看不到就退役。
                self.task_state = TrackState.ARCHIVED
        elif self.task_state == TrackState.CONSUMED:
            if now_ms - self.consumed_since_ms > consumed_ttl_ms:
                self.task_state = TrackState.ARCHIVED
        # ARCHIVED never auto-transitions.
