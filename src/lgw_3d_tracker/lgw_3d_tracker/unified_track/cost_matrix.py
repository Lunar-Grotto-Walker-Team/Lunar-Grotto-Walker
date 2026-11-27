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

from lgw_3d_tracker.unified_track.track import Track, TrackState


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
        active_tracks = [t for t in tracks if t.task_state == TrackState.ACTIVE]
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