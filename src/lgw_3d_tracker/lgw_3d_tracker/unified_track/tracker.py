"""Tracker coordinator: predict + associate + update + lifecycle.

Global registry keyed by stable track id.  `candidate` is the false-positive
gate; `active` is published and lives until `active_ttl_ms` since its last
observation; `consumed` is set by the task layer (terminal); `archived` is
dropped.  Invisible active tracks stay in the association pool, so a
re-observation keeps the id; a displacement beyond the gate is a new target.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.geometry_projector import short_camera_name
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D, KalmanState3D
from lgw_3d_tracker.unified_track.selection import SelectionParams, select_target
from lgw_3d_tracker.unified_track.track import Track, TrackObservation, TrackState


@dataclass
class AssociationReport:
    """Per-tick association detail, for trace/diagnostics only."""
    matches: List[dict] = field(default_factory=list)
    tentative_matches: List[dict] = field(default_factory=list)
    reid_matches: List[dict] = field(default_factory=list)
    created: List[dict] = field(default_factory=list)
    unmatched: List[int] = field(default_factory=list)
    suppressed: List[dict] = field(default_factory=list)
    # 2026-11-18: 建轨门限丢弃的观测(距机械臂基座过远 / 落在 odom 原点屏蔽圈)。
    rejected: List[dict] = field(default_factory=list)


class Tracker:
    """3D tracking loop with a retained global registry."""

    def __init__(
        self,
        kf: KalmanFilter3D,
        assigner: TwoStageAssigner,
        candidate_min_hits: int = 3,
        candidate_min_age_ms: int = 0,
        candidate_ttl_ms: int = 2000,
        active_ttl_ms: int = 15000,
        consumed_ttl_ms: int = 60000,
        distance_gate_m: float = 0.15,
        duplicate_track_distance_m: float = 0.04,
        max_registry_size: int = 100,
        consumed_suppress_enable: bool = True,
        consumed_suppress_radius_m: float = 0.15,
        consumed_suppress_ttl_ms: int = 10000,
        arm_window_ms: int = 3000,
        predict_freeze_after_ms: int = 1000,
        # 建轨门限; <=0 = 关闭。真正生效的值来自 runtime.yaml
        # (config.py 默认 5.0 / 2.5), 这里默认关闭以保持单元测试的旧行为。
        max_track_distance_m: float = 0.0,
        base_exclusion_radius_m: float = 0.0,
        # 选中轨迹的 active TTL 豁免上限(ms): 选中后多久没被观测就退役。
        # <=0 = 不设上限(不建议)。
        selected_max_unseen_ms: int = 60000,
        selection: SelectionParams = SelectionParams(),
    ):
        self.kf = kf
        self.assigner = assigner
        self.candidate_min_hits = candidate_min_hits
        self.candidate_min_age_ms = candidate_min_age_ms
        self.candidate_ttl_ms = candidate_ttl_ms
        self.active_ttl_ms = active_ttl_ms
        self.consumed_ttl_ms = consumed_ttl_ms
        self.distance_gate_m = distance_gate_m
        self.duplicate_track_distance_m = duplicate_track_distance_m
        self.max_registry_size = max_registry_size
        self.consumed_suppress_enable = consumed_suppress_enable
        self.consumed_suppress_radius_m = consumed_suppress_radius_m
        self.consumed_suppress_ttl_ms = consumed_suppress_ttl_ms
        self.arm_window_ms = arm_window_ms
        self.predict_freeze_after_ms = predict_freeze_after_ms
        self.max_track_distance_m = max_track_distance_m
        self.base_exclusion_radius_m = base_exclusion_radius_m
        self.selected_max_unseen_ms = selected_max_unseen_ms
        # 基地圆心(odom, 固定): 节点启动那一刻机器人所在位置, 由 node 设置。
        self._base_center_odom: Optional[np.ndarray] = None
        self.selection = selection
        self._arm_base_position: Optional[np.ndarray] = None
        self._selected_track_id = 0

        self.last_dedupe_removed: List[dict] = []
        self._next_id = 1
        self._tracks: List[Track] = []
        self._now_ms: int = 0

    def set_now_ms(self, now_ms: int) -> None:
        self._now_ms = now_ms

    def set_arm_base_position(self, p_odom: Optional[np.ndarray]) -> None:
        """Arm base origin in odom; None means "cannot rank, keep selection"."""
        self._arm_base_position = (
            None if p_odom is None else np.asarray(p_odom, dtype=float).copy())

    @property
    def base_center_odom(self) -> Optional[np.ndarray]:
        """当前基地圆心(odom 副本); None = 尚未钉住, 屏蔽圈不生效。"""
        return (None if self._base_center_odom is None
                else self._base_center_odom.copy())

    def set_base_center(self, p_odom: Optional[np.ndarray]) -> None:
        """固定"基地圆心"(odom 系): 启动那一刻机器人所在位置。

        2026-11-18: 与 odom 原点不同 —— 原点是上电点, 会随里程计漂移;
        基地是任务起点, 采集完的目标堆放处。只在还没有圆心时设置(或用
        force=True 重新钉), 避免运行中被反复覆盖。
        """
        if p_odom is None:
            return
        self._base_center_odom = np.asarray(p_odom, dtype=float).copy()

    def _gate_observations(
        self, observations: List[TrackObservation]
    ) -> Tuple[List[TrackObservation], List[dict]]:
        """2026-11-18 两道建轨门限(在关联/建轨之前生效)。

        * 与机械臂基座的三维距离 > max_track_distance_m -> 丢弃。
          臂够不着, 建轨只会给选择器喂一个不可达目标
          (真机现象: 抓取时 move_group 报 GOAL_STATE_INVALID)。
        * 与"基地圆心"的 XoY 距离 < base_exclusion_radius_m -> 丢弃。
          基地圆心是任务启动那一刻机器人所在位置(odom 系)的快照, 不是
          odom 原点(上电点会随里程计漂移, 基地是任务起点)。用来屏蔽基地里
          已采集、堆放着的目标, 避免被重复识别。

        只作用于"这一帧的观测"; 已存在的轨迹不受影响, 继续按自身 TTL 退役。
        max_track_distance_m <= 0 或暂时拿不到 arm base 时该条不限;
        base_exclusion_radius_m <= 0 或基地圆心尚未钉住时屏蔽圈关闭。
        """
        kept: List[TrackObservation] = []
        rejected: List[dict] = []
        for i, o in enumerate(observations):
            p = np.asarray(o.position_odom, dtype=float)
            if (self.max_track_distance_m > 0.0
                    and self._arm_base_position is not None):
                d = float(np.linalg.norm(p - self._arm_base_position))
                if d > self.max_track_distance_m:
                    rejected.append({
                        "obs": i, "reason": "arm_base_distance",
                        "class": o.class_name, "dist": round(d, 3)})
                    continue
            if (self.base_exclusion_radius_m > 0.0
                    and self._base_center_odom is not None):
                d0 = float(np.linalg.norm(p[:2] - self._base_center_odom[:2]))
                if d0 < self.base_exclusion_radius_m:
                    rejected.append({
                        "obs": i, "reason": "base_circle",
                        "class": o.class_name, "dist": round(d0, 3)})
                    continue
            kept.append(o)
        return kept, rejected

    @property
    def selected_track_id(self) -> int:
        """The mission target; 0 means none.  Changes only on TTL/consume/
        ignore/request — never because another target became nearer."""
        return self._selected_track_id

    def predict_all(self) -> None:
        """Predict fresh active/candidate tracks; stale ones stay frozen."""
        for t in self._tracks:
            if (t.task_state in (TrackState.CANDIDATE, TrackState.ACTIVE)
                    and t.time_since_last_obs_ms <= self.predict_freeze_after_ms):
                t.state = self.kf.predict(t.state)

    def associate_and_update(
        self, observations: List[TrackObservation]
    ) -> AssociationReport:
        report = AssociationReport()
        if not observations:
            return report

        # 2026-11-18 建轨门限: 在关联/建轨之前把不合格的观测丢掉。
        observations, report.rejected = self._gate_observations(observations)
        if not observations:
            return report

        active = [t for t in self._tracks if t.task_state == TrackState.ACTIVE]
        candidate = [t for t in self._tracks if t.task_state == TrackState.CANDIDATE]
        consumed = [t for t in self._tracks if t.task_state == TrackState.CONSUMED]

        obs_pos = np.array([o.position_odom for o in observations])
        obs_cls = [o.class_name for o in observations]
        obs_conf = [o.confidence for o in observations]

        def gate_fn(state: KalmanState3D, z: np.ndarray) -> bool:
            return np.linalg.norm(z - state.x) < self.distance_gate_m

        self.assigner.gate_fn = gate_fn
        matches, unmatched_obs = self.assigner.assign(
            obs_pos, obs_cls, obs_conf, active)
        track_idx_map = {j: self._tracks.index(t) for j, t in enumerate(active)}

        for obs_idx, j in matches.items():
            t = self._tracks[track_idx_map[j]]
            dist = float(np.linalg.norm(
                observations[obs_idx].position_odom - t.state.x))
            report.matches.append(
                {"obs": obs_idx, "track_id": t.id, "dist": round(dist, 4)})
            self._apply_match(t, observations[obs_idx])

        # Candidate second pass: class + distance gate, greedy nearest.
        if candidate and unmatched_obs:
            still_unmatched = []
            for i in unmatched_obs:
                obs = observations[i]
                best_j = -1
                best_d = self.distance_gate_m
                for tj, t in enumerate(candidate):
                    if t.last_obs is None or t.last_obs.class_name != obs.class_name:
                        continue
                    d = float(np.linalg.norm(obs.position_odom - t.state.x))
                    if d < best_d:
                        best_d = d
                        best_j = tj
                if best_j >= 0:
                    t = candidate[best_j]
                    report.tentative_matches.append(
                        {"obs": i, "track_id": t.id, "dist": round(best_d, 4)})
                    self._apply_match(t, obs)
                else:
                    still_unmatched.append(i)
            unmatched_obs = still_unmatched

        # Duplicate-face suppression for unmatched observations.
        if unmatched_obs:
            still_unmatched = []
            for i in unmatched_obs:
                obs = observations[i]
                nearest = None
                for t in self._tracks:
                    if t.task_state in (TrackState.CONSUMED, TrackState.ARCHIVED):
                        continue
                    if t.last_obs is None or t.last_obs.class_name != obs.class_name:
                        continue
                    d = float(np.linalg.norm(obs.position_odom - t.state.x))
                    if d < self.duplicate_track_distance_m:
                        nearest = (t, d)
                        break
                if nearest is None:
                    still_unmatched.append(i)
                else:
                    report.suppressed.append({
                        "obs": i, "track_id": nearest[0].id,
                        "dist": round(nearest[1], 4)})
            unmatched_obs = still_unmatched

        # Optional consumed-location suppression (v1 off).
        if self.consumed_suppress_enable and consumed and unmatched_obs:
            still_unmatched = []
            for i in unmatched_obs:
                obs = observations[i]
                hit = None
                for t in consumed:
                    if t.last_obs is None or t.last_obs.class_name != obs.class_name:
                        continue
                    if self._now_ms - t.consumed_since_ms > self.consumed_suppress_ttl_ms:
                        continue
                    d = float(np.linalg.norm(obs.position_odom - t.state.x))
                    if d < self.consumed_suppress_radius_m:
                        hit = (t, d)
                        break
                if hit is None:
                    still_unmatched.append(i)
                else:
                    report.suppressed.append({
                        "obs": i, "track_id": hit[0].id,
                        "dist": round(hit[1], 4)})
            unmatched_obs = still_unmatched

        # New candidate.
        for i in unmatched_obs:
            obs = observations[i]
            new_t = Track(
                id=self._next_id,
                created_ms=self._now_ms,
                task_state=TrackState.CANDIDATE,
                first_seen_ms=self._now_ms,
                last_seen_ms=self._now_ms,
                state=KalmanState3D(
                    x=obs.position_odom.copy(), P=np.eye(3) * 0.05),
                hits=1,
                time_since_last_obs_ms=0,
                age=1,
                last_obs=obs,
            )
            new_t.source_cameras = [obs.source_camera]
            if obs.position_arm_base is not None:
                new_t.last_arm_base = np.asarray(obs.position_arm_base, dtype=float)
            report.created.append({"obs": i, "track_id": new_t.id})
            self._next_id += 1
            self._tracks.append(new_t)

        report.unmatched = list(unmatched_obs)
        return report

    def _apply_match(self, t: Track, obs: TrackObservation) -> None:
        """Apply one associated observation; reset a stale KF to the measurement."""
        if t.time_since_last_obs_ms > self.predict_freeze_after_ms:
            new_state = KalmanState3D(
                x=obs.position_odom.copy(), P=np.eye(3) * 0.05)
            t.apply_observation(obs, lambda _state, _z: new_state)
        else:
            t.apply_observation(obs, self.kf.update)
        if short_camera_name(obs.source_camera) == "arm":
            t.record_arm_hit(self._now_ms, self.arm_window_ms)
        t.last_seen_ms = self._now_ms

    def tick_lifecycle(self, dt_ms: int) -> List[Track]:
        """Advance every track, drop archived, dedupe, enforce capacity."""
        self.last_dedupe_removed = []
        for t in self._tracks:
            t.tick(
                self._now_ms, dt_ms, self.candidate_min_hits,
                self.candidate_min_age_ms, self.candidate_ttl_ms,
                self.active_ttl_ms, self.consumed_ttl_ms,
                # 选中豁免: 任务正在操作的那个目标不参与 active TTL 归档,
                # 只受"长时间无观测"上限约束(2026-11-19)。
                is_selected=(t.id == self._selected_track_id),
                selected_max_unseen_ms=self.selected_max_unseen_ms)
        self._tracks = [
            t for t in self._tracks if t.task_state != TrackState.ARCHIVED]
        self.last_dedupe_removed = self.dedupe_tracks()
        self._enforce_capacity()
        self._maintain_selection()
        self._select_if_needed()
        return self.active_tracks()

    def _maintain_selection(self) -> None:
        if self._selected_track_id == 0:
            return
        for t in self._tracks:
            if t.id == self._selected_track_id:
                if t.task_state == TrackState.ACTIVE:
                    return
                break
        self._selected_track_id = 0

    def _select_if_needed(self) -> None:
        if self._selected_track_id != 0:
            return
        self._selected_track_id = select_target(
            self._tracks, self._now_ms, self._arm_base_position, self.selection)

    def _enforce_capacity(self) -> None:
        while len(self._tracks) > self.max_registry_size:
            victims = [
                t for t in self._tracks
                if t.task_state == TrackState.ACTIVE
                and t.id != self._selected_track_id
                and t.time_since_last_obs_ms > self.predict_freeze_after_ms]
            if not victims:
                victims = [
                    t for t in self._tracks if t.task_state == TrackState.CANDIDATE]
            if not victims:
                return  # never evict a fresh active or consumed target
            victim = min(victims, key=lambda t: (t.last_seen_ms, t.id))
            self._tracks.remove(victim)

    def dedupe_tracks(self) -> List[dict]:
        """Drop same-class tracks whose 3D positions nearly coincide."""
        if len(self._tracks) < 2:
            return []

        def maturity(t: Track):
            selected = t.id == self._selected_track_id
            established = t.task_state == TrackState.ACTIVE
            return (0 if selected else 1, 0 if established else 1,
                    -t.hits, t.created_ms, t.id)

        removed: List[dict] = []
        survivors: List[Track] = []
        for t in sorted(self._tracks, key=maturity):
            dup_kept = None
            for s in survivors:
                if (s.last_obs is None or t.last_obs is None
                        or s.last_obs.class_name != t.last_obs.class_name):
                    continue
                d = float(np.linalg.norm(s.state.x - t.state.x))
                if d < self.duplicate_track_distance_m:
                    dup_kept = (s, d)
                    break
            if dup_kept is None:
                survivors.append(t)
            else:
                kept, d = dup_kept
                removed.append({
                    "removed_id": int(t.id),
                    "kept_id": int(kept.id),
                    "dist": round(d, 4),
                    "removed_status": t.task_state.value,
                    "removed_hits": int(t.hits),
                })
                t.task_state = TrackState.ARCHIVED
                if self._selected_track_id == t.id:
                    self._selected_track_id = kept.id

        if removed:
            self._tracks = [
                t for t in self._tracks if t.task_state != TrackState.ARCHIVED]
        return removed

    def mark_track(self, track_id: int, action: int) -> bool:
        """1/2=CONSUMED, 4=request (id 0 releases), 5=clear arm hits.

        2026-11-18: action 5 只清空该轨迹的 arm 相机命中历史, 状态不变;
        action 6 把基地屏蔽圈圆心钉在当前机械臂基座 odom 位置(不需要 track_id)。
        BT 在"低头 25° 完成"之后调用它, 这样 WaitForArmHits 统计到的命中
        一定是低头之后的 —— 否则低头过程中的旧命中会立刻把验证"通过"掉
        (真机实测: 低头命令发出 97 ms 后 target verified, arm_hits=5)。
        """
        if action == 4 and track_id == 0:
            self._selected_track_id = 0
            return True
        if action == 6:
            # 基地屏蔽圈圆心 = 调用这一刻机器人的 odom 位置。
            # 行为树在初始化时调用(任务起点), 不依赖 tracker 节点的启动时刻。
            if self._arm_base_position is None:
                return False
            self.set_base_center(self._arm_base_position)
            return True
        for t in self._tracks:
            if t.id != track_id:
                continue
            if action in (1, 2):
                t.task_state = TrackState.CONSUMED
                t.consumed_reason = "consumed" if action == 1 else "ignored"
                t.consumed_since_ms = self._now_ms
                if self._selected_track_id == track_id:
                    self._selected_track_id = 0
                return True
            if action == 4:
                if t.task_state != TrackState.ACTIVE:
                    return False
                self._selected_track_id = track_id
                return True
            if action == 5:
                # 只清命中历史, 不改状态、不改 selection。
                t.arm_hit_stamps_ms.clear()
                return True
            return False
        return False

    def active_tracks(self) -> List[Track]:
        return [t for t in self._tracks if t.task_state == TrackState.ACTIVE]

    def all_tracks(self) -> List[Track]:
        return list(self._tracks)

    @property
    def tracks(self) -> List[Track]:
        return list(self._tracks)
