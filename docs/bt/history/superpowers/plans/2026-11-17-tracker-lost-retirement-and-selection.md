# Tracker lost 退役、40 s TTL 与单所有者选择 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `lgw_3d_tracker` 的生命周期简化为 candidate/active/consumed/archived（40 s 滚动 TTL），由 tracker 单点维护 `selected_track_id`，servo 只执行该 id，并把 arm 相机命中计数随消息发布。

**Architecture:** 消息层新增 3 个字段；tracker 删除 lost 与 re-ID 通道，新增纯函数选择模块与 arm 命中窗口；servo 删除自己的 nearest/requested 选择器，只按 `selected_track_id` 找轨道。三层同批升级，消息字段是唯一跨语言契约。

**Tech Stack:** Python 3.10 / rclpy / numpy / pytest（tracker）；C++17 / rclcpp / gtest（servo）；ROS 2 Humble / CycloneDDS。

**Spec:** `docs/superpowers/specs/2026-11-17-tracker-lost-retirement-and-selection-design.md`

## Global Constraints

- `active_ttl_ms = 40000`：距**最后一次观测**滚动计时；`>` 比较。
- `association_distance_gate_m = 0.15`：主关联与 candidate 二次关联共用。
- 选择距离带：`select_min_distance_m = 0.20`，`select_max_distance_m = 0.0`（0 = 不限），基准是 `odom ← arm1/base_link` 的 **XoY 平面距离**。
- `selection_fresh_ms = 1000`，`predict_freeze_after_ms = 1000`。
- `arm_verify_window_ms = 3000`；判据由 BT 用 `arm_hits_in_window >= 3`（BT 不在本计划范围）。
- `consumed_ttl_ms = 60000`，`consumed_suppress_radius_m = 0.15`，`consumed_suppress_ttl_ms = 10000`，`consumed_suppress_enable = true`。
- `candidate_min_hits = 3`，`candidate_ttl_ms = 2000`，`duplicate_track_distance_m = 0.04`，`publish_heartbeat_hz = 25`。
- `select_class_filter` 默认 `""`（= 允许所有 class）；部署值 `wood_block`。
- 消息字段：`UnifiedTrackArray.selected_track_id`（uint64）、`UnifiedTrackArray.arm_window_ms`（uint32）、`UnifiedTrack.arm_hits_in_window`（int32）。
- 执行约束：TDD 循环内**不整包编译**；Python 直接跑 pytest，C++ 只编被测 target。阶段收尾才做完整 `colcon build` + 全量测试。
- 跑 `colcon test` 前先停机械臂链并 `/control_enable false`（否则机械臂会被带到全零位，j3 可能触发固件保护）。
- 所有容器命令在 `self_l4t` 容器内执行，先 `source /my_ws/l4t/.l4t_env.sh`。
- `.msg` 变更无法只编 target，属于 Phase Gate A。

## Review Focus

1. **选中轨道被 40 s TTL 归档** → `selected_track_id` 必须归 0；下一 tick 可立刻选中新目标（BT 处理 id 跳变）。归 Task 3/5。
2. **selected 轨道被 dedupe 合并** → 不得留下悬空 id；选择改指向 `kept_id`。归 Task 5。
3. **arm 基座 TF 缺失/陈旧** → 不进行新选择，保持原 `selected_track_id`。归 Task 5/6。
4. **重新出现位移 > 0.15 m** → 按新目标处理（新建 candidate），旧 active 自然到期。归 Task 3。
5. **ignore 前目标已 >10 s 未观测** → 抑制仍须从 `consumed_since_ms` 起算，否则一 ignore 就失效。归 Task 2/3。

---

## Task 1: 消息契约（yoloe_interfaces）

**Files:**
- Modify: `src/yoloe_interfaces/msg/UnifiedTrackArray.msg`
- Modify: `src/yoloe_interfaces/msg/UnifiedTrack.msg`
- Test: `src/lgw_3d_tracker/test/test_messages.py`（新建）

**Interfaces:**
- Produces: `UnifiedTrackArray.selected_track_id: uint64`、`UnifiedTrackArray.arm_window_ms: uint32`、`UnifiedTrack.arm_hits_in_window: int32`（后续所有任务依赖）。

- [ ] **Step 1: 写失败测试**

```python
from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray


def test_new_fields_default_to_zero():
    arr = UnifiedTrackArray()
    track = UnifiedTrack()
    assert arr.selected_track_id == 0
    assert arr.arm_window_ms == 0
    assert track.arm_hits_in_window == 0


def test_new_fields_are_settable():
    arr = UnifiedTrackArray()
    arr.selected_track_id = 7
    arr.arm_window_ms = 3000
    track = UnifiedTrack()
    track.arm_hits_in_window = 3
    assert (arr.selected_track_id, arr.arm_window_ms, track.arm_hits_in_window) == (7, 3000, 3)
```

- [ ] **Step 2: 跑测试确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_messages.py -q"
```
Expected: FAIL — `AttributeError: selected_track_id`。

- [ ] **Step 3: 加字段**

`UnifiedTrackArray.msg` 末尾加：

```
uint64 selected_track_id   # 0 = 无任务目标
uint32 arm_window_ms       # arm_hits_in_window 的窗口长度
```

`UnifiedTrack.msg` 的 `observation timing` 段后加：

```
int32 arm_hits_in_window   # 最近 arm_window_ms 内 arm 相机的命中次数
```

- [ ] **Step 4: Phase Gate A — 重建接口包（唯一允许的整包构建）**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select yoloe_interfaces"
```

- [ ] **Step 5: 跑测试确认通过**

同 Step 2 命令。Expected: `2 passed`。

- [ ] **Step 6: 提交**

```bash
git add src/yoloe_interfaces/msg/UnifiedTrackArray.msg src/yoloe_interfaces/msg/UnifiedTrack.msg src/lgw_3d_tracker/test/test_messages.py
git commit -m "feat(msg): add selected_track_id, arm_window_ms, arm_hits_in_window

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 2: Track 生命周期与 arm 命中窗口

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py`
- Test: `src/lgw_3d_tracker/test/test_task_state.py`

**Interfaces:**
- Consumes: 无。
- Produces:
  - `TrackState` 仅含 `CANDIDATE / ACTIVE / CONSUMED / ARCHIVED`。
  - `Track.consumed_since_ms: int = 0`、`Track.arm_hit_stamps_ms: list[int]`。
  - `Track.tick(now_ms, dt_ms, candidate_min_hits, candidate_min_age_ms, candidate_ttl_ms, active_ttl_ms, consumed_ttl_ms) -> None`。
  - `Track.record_arm_hit(now_ms: int, window_ms: int) -> None`。
  - `Track.count_recent_arm_hits(now_ms: int, window_ms: int) -> int`。

- [ ] **Step 1: 重写 `test_task_state.py` 为失败测试**

```python
import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.track import Track, TrackState

TICK = dict(candidate_min_hits=3, candidate_min_age_ms=0, candidate_ttl_ms=2000,
            active_ttl_ms=40000, consumed_ttl_ms=60000)


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


def test_active_archives_after_40s_without_observation():
    t = make_track(task_state=TrackState.ACTIVE, last_seen_ms=0)
    t.tick(40000, 10, **TICK)
    assert t.task_state == TrackState.ACTIVE
    t.tick(40001, 10, **TICK)
    assert t.task_state == TrackState.ARCHIVED


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
```

- [ ] **Step 2: 跑测试确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_task_state.py -q"
```
Expected: FAIL — `TrackState` 仍有 `LOST`，`tick` 不接受新参数。

- [ ] **Step 3: 改 `track.py`**

- 从 `TrackState` 删除 `LOST`。
- `Track` 增加 `consumed_since_ms: int = 0`、`arm_hit_stamps_ms: List[int] = field(default_factory=list)`。
- `apply_observation()` 删除 `LOST → ACTIVE` 分支。
- `tick()` 新签名（见 Interfaces）；ACTIVE 分支改为
  `if now_ms - self.last_seen_ms > active_ttl_ms: self.task_state = ARCHIVED`；
  CONSUMED 分支改为 `if now_ms - self.consumed_since_ms > consumed_ttl_ms: ARCHIVED`；
  CANDIDATE 分支保持不变。
- 增加 `record_arm_hit()` 与 `count_recent_arm_hits()`：用 `now_ms - window_ms` 剪枝，保留 `stamp >= now_ms - window_ms` 的条目。`record_arm_hit` 只追加并剪枝，不触碰 `last_seen_ms` / `time_since_last_obs_ms`。

- [ ] **Step 4: 跑测试确认通过**

同 Step 2。Expected: `7 passed`（按实际用例数）。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py src/lgw_3d_tracker/test/test_task_state.py
git commit -m "feat(tracker): retire lost state, add 40s active TTL and arm hit window

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 3: Tracker 关联与生命周期

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py`
- Test: `src/lgw_3d_tracker/test/test_tracker_lifecycle.py`（新建）
- Modify: `src/lgw_3d_tracker/test/test_tracker_dedupe.py`、`test_registry.py`、`test_trace_logging.py`（构造参数同步）

**Interfaces:**
- Consumes: Task 2 的 `Track.tick` 新签名、`Track.record_arm_hit`。
- Produces:
  - `Tracker.__init__(..., active_ttl_ms=40000, consumed_ttl_ms=60000, distance_gate_m=0.15, duplicate_track_distance_m=0.04, max_registry_size=100, consumed_suppress_enable=True, consumed_suppress_radius_m=0.15, consumed_suppress_ttl_ms=10000, arm_window_ms=3000, predict_freeze_after_ms=1000)`；删除 `visible_timeout_ms / lost_ttl_ms / reid_distance_m / reid_reinit_distance_m`。
  - `Tracker.mark_track(track_id: int, action: int) -> bool`：1/2 = consume/ignore（设置 `consumed_since_ms`），3 = 返回 False，其他 = False。
  - `Tracker.tick_lifecycle(dt_ms) -> List[Track]`（选择集成在 Task 5）。

- [ ] **Step 1: 写失败测试 `test_tracker_lifecycle.py`**

```python
import numpy as np

from lgw_3d_tracker.unified_track.track import TrackObservation, TrackState
from lgw_3d_tracker.unified_track.tracker import Tracker
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner


def make_tracker(**kw):
    params = dict(candidate_min_hits=3, candidate_min_age_ms=0, candidate_ttl_ms=2000,
                  active_ttl_ms=40000, consumed_ttl_ms=60000, distance_gate_m=0.15,
                  duplicate_track_distance_m=0.04, max_registry_size=100,
                  consumed_suppress_enable=True, consumed_suppress_radius_m=0.15,
                  consumed_suppress_ttl_ms=10000, arm_window_ms=3000,
                  predict_freeze_after_ms=1000)
    params.update(kw)
    return Tracker(kf=KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03),
                   assigner=TwoStageAssigner(0.45, 0.30, lambda s, z: True), **params)


def obs(x, cls="wood_block", cam="camera_head_color_optical_frame", conf=0.9):
    return TrackObservation(class_name=cls, confidence=conf, source_camera=cam,
                            stamp_ms=0, position_odom=np.array([x, 0.0, 0.0]))


def test_stale_active_match_keeps_id_and_resets_kf():
    tr = make_tracker()
    tr.set_now_ms(0)
    tr.associate_and_update([obs(0.0)])
    tr.associate_and_update([obs(0.0)])
    tr.associate_and_update([obs(0.0)])          # 3 hits -> active
    tr.tick_lifecycle(0)
    assert len(tr.active_tracks()) == 1
    track_id = tr.active_tracks()[0].id

    tr.set_now_ms(5000)
    tr.tick_lifecycle(5000)                        # 5 s 无观测，冻结
    report = tr.associate_and_update([obs(0.10)])  # 0.10 < 0.15
    assert [m["track_id"] for m in report.matches] == [track_id]
    t = tr.active_tracks()[0]
    assert np.allclose(t.state.x, [0.10, 0.0, 0.0])
    assert np.allclose(np.diag(t.state.P), [0.05, 0.05, 0.05])


def test_observation_beyond_gate_creates_candidate():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
    tr.tick_lifecycle(0)
    tr.set_now_ms(5000)
    report = tr.associate_and_update([obs(0.20)])  # > 0.15
    assert report.matches == []
    assert [c["obs"] for c in report.created] == [0]


def test_predict_freezes_when_stale():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
    tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    p_before = t.state.P.copy()
    tr.set_now_ms(5000)
    tr.tick_lifecycle(5000)
    tr.predict_all()
    assert np.allclose(t.state.P, p_before)


def test_mark_track_consume_sets_consumed_since():
    tr = make_tracker()
    tr.set_now_ms(1000)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
        tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    assert tr.mark_track(t.id, 1)
    assert t.task_state == TrackState.CONSUMED
    assert t.consumed_since_ms == 1000
    assert tr.mark_track(t.id, 3) is False


def test_consumed_suppress_uses_consumed_since():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
        tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    tr.set_now_ms(20000)                 # last_seen 仍是 0（20 s 前）
    assert tr.mark_track(t.id, 2)        # ignore 发生在 20000
    tr.set_now_ms(21000)                 # consumed_since 后仅 1 s
    report = tr.associate_and_update([obs(0.0)])
    assert report.created == []          # 仍被 consumed_suppress 压住
```

- [ ] **Step 2: 跑测试确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_tracker_lifecycle.py -q"
```
Expected: FAIL — 构造器不接受新参数 / `mark_track(3)` 仍成功。

- [ ] **Step 3: 改 `tracker.py`**

- 构造器按 Interfaces 增删参数；保存 `active_ttl_ms`、`consumed_ttl_ms`、`predict_freeze_after_ms`、`arm_window_ms`。
- `predict_all()`：仅当 `t.time_since_last_obs_ms <= predict_freeze_after_ms` 且状态为 candidate/active 时 predict。
- `associate_and_update()`：删除 lost 池与整个 re-ID 块；主关联与 candidate 二轮统一走 `_apply_match(t, obs)`：
  - `stale = t.time_since_last_obs_ms > self.predict_freeze_after_ms`
  - `stale` → `KalmanState3D(x=obs.position_odom.copy(), P=np.eye(3) * 0.05)` 替代 kf.update；
  - 否则 `self.kf.update`；
  - 若 `short_camera_name(obs.source_camera) == "arm"` → `t.record_arm_hit(self._now_ms, self.arm_window_ms)`；
  - `t.last_seen_ms = self._now_ms`。
- `tick_lifecycle()`：调用 `t.tick(...)` 时传 `now_ms, dt_ms` 与 5 个阈值参数（无 visible/lost）。
- consumed 抑制分支的 TTL 判断改为 `self._now_ms - t.consumed_since_ms > consumed_suppress_ttl_ms`（不再用 `last_seen_ms`）。
- `_enforce_capacity()`：先淘汰 `time_since_last_obs_ms > predict_freeze_after_ms` 的 active（按 `last_seen_ms` 最老优先），再淘汰 candidate；Task 5 再加 selected 保护。
- `mark_track()`：1/2 → CONSUMED + reason + `consumed_since_ms = self._now_ms`；3 → False；其他 → False。
- `_track_snapshot` 不在本任务。

- [ ] **Step 4: 同步既有测试的构造参数**

`test_registry.py`、`test_tracker_dedupe.py`、`test_trace_logging.py`：把 `visible_timeout_ms`、`lost_ttl_ms`、`reid_distance_m`、`reid_reinit_distance_m` 换成 `active_ttl_ms=40000`、`consumed_ttl_ms=60000`、`predict_freeze_after_ms=1000`、`arm_window_ms=3000`；删除依赖 `TrackState.LOST` 的断言。跑：

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/ -q"
```
Expected: 全部 PASS（`test_config.py` 除外，Task 6 才改）。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_3d_tracker/
git commit -m "refactor(tracker): remove lost/reid pools, 0.15 gate, stale KF reset

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 4: 纯选择模块

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/selection.py`
- Test: `src/lgw_3d_tracker/test/test_selection.py`

**Interfaces:**
- Consumes: `Track`（Task 2）。
- Produces:
  - `@dataclass(frozen=True) SelectionParams(class_filter: str = "", min_distance_m: float = 0.20, max_distance_m: float = 0.0, fresh_ms: int = 1000)`。
  - `select_target(tracks: Sequence[Track], now_ms: int, p_arm_base_odom: Optional[np.ndarray], params: SelectionParams) -> int`，返回 `track_id` 或 0。

- [ ] **Step 1: 写失败测试 `test_selection.py`**

```python
import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.selection import SelectionParams, select_target
from lgw_3d_tracker.unified_track.track import Track, TrackState


def make_track(tid, xyz, cls="wood_block", state=TrackState.ACTIVE, age_ms=0):
    t = Track(id=tid, created_ms=0, first_seen_ms=0, last_seen_ms=0,
              task_state=state, hits=3,
              state=KalmanState3D(x=np.array(xyz, dtype=float), P=np.eye(3) * 0.05))
    t.time_since_last_obs_ms = age_ms
    return t


BASE = np.array([0.0, 0.0, 0.3])
P = SelectionParams()


def test_picks_nearest_by_xoy_distance():
    tracks = [make_track(1, (1.0, 0.0, 0.0)), make_track(2, (0.0, 0.5, 2.0))]
    assert select_target(tracks, 0, BASE, P) == 2   # XoY 决定，z 不参与


def test_ignores_stale_tracks():
    tracks = [make_track(1, (0.5, 0.0, 0.0), age_ms=1500)]
    assert select_target(tracks, 2000, BASE, P) == 0


def test_respects_class_filter():
    tracks = [make_track(1, (0.5, 0.0, 0.0), cls="red_block")]
    assert select_target(tracks, 0, BASE, SelectionParams(class_filter="wood_block")) == 0


def test_respects_distance_band():
    tracks = [make_track(1, (0.10, 0.0, 0.0)),
              make_track(2, (2.0, 0.0, 0.0))]
    assert select_target(tracks, 0, BASE, SelectionParams(min_distance_m=0.20)) == 2
    assert select_target(tracks, 0, BASE,
                         SelectionParams(min_distance_m=0.20, max_distance_m=1.0)) == 0


def test_ignores_non_active_and_no_base_pose():
    tracks = [make_track(1, (0.5, 0.0, 0.0), state=TrackState.CANDIDATE)]
    assert select_target(tracks, 0, BASE, P) == 0
    assert select_target([make_track(2, (0.5, 0.0, 0.0))], 0, None, P) == 0
```

- [ ] **Step 2: 跑测试确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_selection.py -q"
```
Expected: FAIL — `ModuleNotFoundError: selection`。

- [ ] **Step 3: 实现 `selection.py`**

只做纯计算：遍历 `task_state == ACTIVE`、类别匹配、`now_ms - last_seen_ms <= fresh_ms`、`hypot(dx, dy)` 在 `[min, max]`（`max <= 0` 表示不限）的轨道，返回距离最小者的 id；无候选返回 0。使用 `t.state.x` 与 `p_arm_base_odom` 的前两维。

- [ ] **Step 4: 跑测试确认通过**

同 Step 2。Expected: `5 passed`。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/selection.py src/lgw_3d_tracker/test/test_selection.py
git commit -m "feat(tracker): add pure XoY target selection module

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 5: Tracker 选择集成与一致性

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py`
- Test: `src/lgw_3d_tracker/test/test_selection_integration.py`（新建）
- Modify: `src/lgw_3d_tracker/test/test_tracker_dedupe.py`（selected 保护）

**Interfaces:**
- Consumes: Task 4 的 `SelectionParams`、`select_target`。
- Produces:
  - `Tracker.set_arm_base_position(p_odom: Optional[np.ndarray]) -> None`
  - `Tracker.selected_track_id -> int`（property）
  - `Tracker.mark_track(track_id, action)` 增加 action 4：`id=0` 清空选择；`id!=0` 仅当目标为 active 时选中，否则 False。

- [ ] **Step 1: 写失败测试 `test_selection_integration.py`**

```python
import numpy as np
from test_tracker_lifecycle import make_tracker, obs   # 复用夹具


def seed_active(tr, x=0.5):
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(x)])
    tr.tick_lifecycle(0)
    return tr.active_tracks()[0]


def test_selects_when_none_and_base_pose_available():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id


def test_no_selection_without_base_pose():
    tr = make_tracker()
    seed_active(tr)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == 0


def test_lock_survives_closer_target():
    tr = make_tracker()
    t = seed_active(tr, 0.8)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id
    for _ in range(3):
        tr.associate_and_update([obs(0.3)])
        tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id


def test_selection_cleared_on_consume():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.mark_track(t.id, 2)
    assert tr.selected_track_id == 0


def test_request_zero_releases_and_request_consumed_fails():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.mark_track(0, 4)
    assert tr.selected_track_id == 0
    tr.mark_track(t.id, 1)
    assert tr.mark_track(t.id, 4) is False


def test_selected_track_archived_clears_selection():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    tr.set_now_ms(40001)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == 0


def test_dedupe_keeps_selection_pointing_to_existing_track():
    from lgw_3d_tracker.unified_track.kf import KalmanState3D
    from lgw_3d_tracker.unified_track.track import Track

    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    # 同位置、同类的第二条轨道，迫使 dedupe 合并掉一条
    dup = Track(id=99, created_ms=-1000, first_seen_ms=-1000, last_seen_ms=0,
                task_state=TrackState.ACTIVE, hits=10,
                state=KalmanState3D(x=t.state.x.copy(), P=np.eye(3) * 0.05),
                last_obs=t.last_obs)
    tr._tracks.append(dup)
    removed = tr.dedupe_tracks()
    assert removed, "expected dedupe to remove one track"
    assert tr.selected_track_id in {x.id for x in tr.tracks}
    assert tr.selected_track_id in (t.id, 99)
```

- [ ] **Step 2: 跑测试确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_selection_integration.py -q"
```
Expected: FAIL — 无 `set_arm_base_position` / `selected_track_id`。

- [ ] **Step 3: 集成**

- 构造器增加 `selection: SelectionParams = SelectionParams()` 与 `self._arm_base_position: Optional[np.ndarray] = None`、`self._selected_track_id = 0`。
- `set_arm_base_position()` 保存副本。
- property `selected_track_id` 返回 `self._selected_track_id`。
- `tick_lifecycle()` 顺序：tick → 删 archived → `dedupe_tracks()` → `_enforce_capacity()` → `_maintain_selection()` → `select`（仅当 `_selected_track_id == 0` 且 `_arm_base_position is not None`）→ 返回 active。
- `_maintain_selection()`：若 selected 不为 0 且对应轨道不存在或非 ACTIVE → 置 0。
- `dedupe_tracks()`：maturity key 改为 `(0 if t.id == self._selected_track_id else 1, 0 if active else 1, -hits, created_ms, id)`；删除条目时若 `removed_id == selected` → `selected = kept_id`。
- `_enforce_capacity()`：淘汰候选排序时排除 `id == selected`。
- `mark_track()`：1/2 时若 `id == selected` → 清 0 + `consumed_since_ms`；action 4 按 Interfaces 实现。

- [ ] **Step 4: 跑全量 Python 测试**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/ -q"
```
Expected: 除 `test_config.py`（Task 6）外全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add src/lgw_3d_tracker/
git commit -m "feat(tracker): own selected_track_id with lock and dedupe consistency

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 6: 节点、格式化与配置

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py`
- Modify: `src/lgw_3d_tracker/config/runtime.yaml`
- Test: `src/lgw_3d_tracker/test/test_config.py`、`test_node_publish.py`

**Interfaces:**
- Consumes: Task 5 的 `Tracker.selected_track_id`、`set_arm_base_position`。
- Produces:
  - `UnifiedTrackConfig` 新字段（见 Global Constraints）；删除 `visible_timeout_ms / lost_ttl_ms / reid_distance_m / reid_reinit_distance_m`。
  - `OutputFormatter.format(..., selected_track_id: int = 0, arm_window_ms: int = 3000)`，每条 track 写 `arm_hits_in_window`。
  - `UnifiedTrackNode._lookup_arm_base_odom() -> Optional[np.ndarray]`。

- [ ] **Step 1: 更新 `test_config.py` 为失败测试**

```python
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig


def test_new_defaults():
    c = UnifiedTrackConfig()
    assert c.active_ttl_ms == 40000
    assert c.consumed_ttl_ms == 60000
    assert c.association_distance_gate_m == 0.15
    assert c.predict_freeze_after_ms == 1000
    assert c.selection_fresh_ms == 1000
    assert c.arm_window_ms == 3000
    assert c.select_min_distance_m == 0.20
    assert c.select_max_distance_m == 0.0
    assert c.select_class_filter == ""
    assert c.consumed_suppress_enable is True


def test_legacy_keys_are_ignored(tmp_path):
    p = tmp_path / "runtime.yaml"
    p.write_text("unified_track:\n  lost_ttl_ms: 999\n  reid_distance_m: 0.3\n")
    c = UnifiedTrackConfig.from_yaml(str(p))
    assert c.active_ttl_ms == 40000
```

- [ ] **Step 2: 运行确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cd /my_ws/l4t/lg_walker_ws/src/lgw_3d_tracker && \
   PYTHONPATH=. python3 -m pytest test/test_config.py -q"
```
Expected: FAIL — `active_ttl_ms` 不存在。

- [ ] **Step 3: 改 `config.py`**

删旧字段、加新字段；`from_yaml()` 读取新键；把 `visible_timeout_ms / lost_ttl_ms / reid_distance_m / reid_reinit_distance_m` 加入已废弃键的 `warnings.warn` 列表并忽略。`consumed_suppress_enable` 默认改 `True`。

- [ ] **Step 4: 改 `output_formatter.py` 与新增测试断言**

`format()` 设置 `msg.selected_track_id`、`msg.arm_window_ms`；`_track_to_msg()` 设置 `m.arm_hits_in_window = t.count_recent_arm_hits(now_ms, arm_window_ms)`。在 `test_node_publish.py` 增加：

```python
import numpy as np
from builtin_interfaces.msg import Time

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.track import Track, TrackState


def test_formatter_publishes_selection_and_arm_hits():
    t = Track(id=7, created_ms=0, first_seen_ms=0, last_seen_ms=0,
              task_state=TrackState.ACTIVE, hits=3,
              state=KalmanState3D(x=np.zeros(3), P=np.eye(3) * 0.05))
    t.record_arm_hit(1000, 3000)
    t.record_arm_hit(1100, 3000)
    fmt = OutputFormatter(frame_id="odom", arm_base_frame="arm1/base_link")
    arr = fmt.format(tracks=[t], sequence_id=1, frame_stamp_ms=0,
                     now=Time(sec=2, nanosec=0), T_base_odom=None,
                     selected_track_id=7, arm_window_ms=3000)
    assert arr.selected_track_id == 7
    assert arr.arm_window_ms == 3000
    assert arr.tracks[0].arm_hits_in_window == 2
```

- [ ] **Step 5: 改 `unified_track_node.py`**

- 新增 `_lookup_arm_base_odom()`：`can_transform("odom", f"{arm_frame_prefix}base_link", latest, timeout=0)` 零超时探测；成功则 `lookup_transform` 并返回 translation 的 numpy 拷贝；若 `header.stamp` 非零且 `now - stamp > robot_pose_stale_ms` → 返回 None；失败返回 None。
- 更新 `Tracker(...)` 构造调用：删除 `visible_timeout_ms / lost_ttl_ms / reid_distance_m / reid_reinit_distance_m`，改传 `active_ttl_ms=self.config.active_ttl_ms`、`consumed_ttl_ms=self.config.consumed_ttl_ms`、`arm_window_ms=self.config.arm_window_ms`、`predict_freeze_after_ms=self.config.predict_freeze_after_ms`；并传
  `selection=SelectionParams(class_filter=self.config.select_class_filter, min_distance_m=self.config.select_min_distance_m, max_distance_m=self.config.select_max_distance_m, fresh_ms=self.config.selection_fresh_ms)`。
- `_process_observation()` 与 `_heartbeat()`：在 `tick_lifecycle` 前调用 `self._tracker.set_arm_base_position(self._lookup_arm_base_odom())`。
- `_publish_active()`：传 `selected_track_id=self._tracker.selected_track_id`、`arm_window_ms=self.config.arm_window_ms`。
- `_track_snapshot()`：增加 `"arm_hits": t.count_recent_arm_hits(self._now?)` — 用 `now_ms` 参数传入或省略；trace 字段非强制，若实现则用 `self._last_update_ms`。

- [ ] **Step 6: 更新 `runtime.yaml`**

删除 `visible_timeout_ms`、`lost_ttl_ms`、`reid_distance_m`、`reid_reinit_distance_m`；写入：

```yaml
  active_ttl_ms: 40000
  association_distance_gate_m: 0.15
  predict_freeze_after_ms: 1000
  selection_fresh_ms: 1000
  arm_window_ms: 3000
  select_class_filter: "wood_block"
  select_min_distance_m: 0.20
  select_max_distance_m: 0.0
  consumed_ttl_ms: 60000
  consumed_suppress_enable: true
```

- [ ] **Step 7: Phase Gate B — tracker 全量构建与测试**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_3d_tracker && \
   cd src/lgw_3d_tracker && PYTHONPATH=. python3 -m pytest test/ -q"
```
Expected: build 成功；全部 pytest PASS。

- [ ] **Step 8: 提交**

```bash
git add src/lgw_3d_tracker/
git commit -m "feat(tracker): publish selected id and arm hit window from node

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 7: Servo 只消费 selected_track_id

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_node.hpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_node.cpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_takeover.cpp`
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_params.hpp`
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`
- Modify: `src/lgw_bt_planner/config/servo_node_parameters.yaml`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`
- Delete: `src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp`、`src/lgw_bt_planner/src/target_selector.cpp`、`src/lgw_bt_planner/tests/test_target_selector.cpp`
- Test: `src/lgw_bt_planner/tests/test_servo_scan_gate.cpp`、`test_param_contract.cpp`

**Interfaces:**
- Consumes: Task 1 的 `UnifiedTrackArray.selected_track_id`。
- Produces: servo 不再有 `TargetSelectorParams / TargetSelectorState / TargetSelection / select_target / fresh_selector_state`；`Takeover.srv` 的 `track_id` / `class_filter` 被忽略。

- [ ] **Step 1: 重写 `test_servo_scan_gate.cpp` 为失败测试**

```cpp
TEST(ServoNodeScanGate, TakeoverWithoutSelectedTargetIsScanOnly)
{
  if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray empty;
  node->set_latest_array_for_test(empty);

  bool scan_only = false;
  EXPECT_TRUE(node->takeover_for_test(0, "", scan_only));
  EXPECT_TRUE(scan_only);
}

TEST(ServoNodeScanGate, TakeoverIgnoresRequestedTrackId)
{
  // selected_track_id == 0 时，即使请求里带非零 track_id / class_filter
  // 也必须走 scan-only（旧代码会按 requested_missing 直接拒绝）。
  if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray empty;
  node->set_latest_array_for_test(empty);

  bool scan_only = false;
  EXPECT_TRUE(node->takeover_for_test(9, "wood_block", scan_only));
  EXPECT_TRUE(scan_only);
}

TEST(ServoNodeScanGate, TakeoverWithSelectedIdMissingFails)
{
  if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 9;
  node->set_latest_array_for_test(arr);

  bool scan_only = false;
  EXPECT_FALSE(node->takeover_for_test(0, "", scan_only));
  EXPECT_FALSE(scan_only);
}

TEST(ServoNodeScanGate, ReselectFollowsSelectedId)
{
  if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 5;
  yoloe_interfaces::msg::UnifiedTrack track;
  track.track_id = 5;
  track.task_state = "active";
  arr.tracks.push_back(track);
  node->set_latest_array_for_test(arr);

  node->reselect_for_test();
  EXPECT_EQ(node->selected_track_id_for_test(), 5u);
}

TEST(ServoNodeScanGate, ReselectClearsWhenSelectedMissing)
{
  if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrackArray arr;
  arr.selected_track_id = 9;
  node->set_latest_array_for_test(arr);

  node->reselect_for_test();
  EXPECT_EQ(node->selected_track_id_for_test(), 0u);
}
```

在 `servo_node.hpp` 的 test hooks 段加：

```cpp
void reselect_for_test()
{
  std::lock_guard<std::mutex> lock(mutex_);
  reselect_locked();
}
uint64_t selected_track_id_for_test() const
{
  return has_selected_snapshot_ ? selected_snapshot_.track_id : 0u;
}
```

- [ ] **Step 2: 只编 target 并运行，确认失败**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cmake --build build/lgw_bt_planner --target test_servo_scan_gate -j4 && \
   ./build/lgw_bt_planner/test_servo_scan_gate"
```
Expected: FAIL — selected id 未被使用。

- [ ] **Step 3: 改 `servo_takeover.cpp`**

- 删除 `params` / `candidates` / `select_target` 块与 `lookup_odom_base_locked()` 调用。
- `request.track_id` / `request.class_filter`：限频 WARN "deprecated and ignored"。
- **检查顺序**：FAULT → 非 IDLE → move_group busy → `selected == 0`（scan-only）→ **selected 存在性**（找不到 → `reason = "selected_missing"`，返回 false）→ `start_state_ != STARTED` → 设置快照。把存在性放在 start 检查之前，测试与诊断都能区分两种失败。
- `scan_only` 条件：`(latest_array_ ? latest_array_->selected_track_id : 0) == 0`。
- selected 非 0 且存在：设置 `selected_snapshot_`、`has_selected_snapshot_`、`tracking_track_id_`，`loop_.reset()`，`arm_saturation_window_locked()`，`sm_.handle(TAKEOVER)`。

- [ ] **Step 4: 改 `servo_node.cpp` / `servo_node.hpp`**

- `reselect_locked()`：读 `latest_array_->selected_track_id`；0 → `has_selected_snapshot_ = false`、`tracking_track_id_ = 0`；非 0 在 tracks 中查找，找不到 → 同样清空并限频 WARN `selected_missing`；找到 → 更新 snapshot，若 track_id 变化则 `loop_.reset()`。
- 删除 `fresh_selector_state()` 定义与声明、`selector_params_`、`selector_state_`、`last_selection_`、`#include "lgw_bt_planner/target_selector.hpp"`。
- `clear_session_locked()`：删除 selector 初始化与 `last_selection_` 重置。
- 删除 `target_min_distance_m_` / `target_max_distance_m_` 成员与赋值（`servo_node.cpp:376-377`、`servo_node.hpp:526-527`）。
- 删除 `flatten_candidates()` 的声明与定义（`TrackCandidate` 随之消失，已无调用者）。
- 更新 `servo_node.hpp` 顶部仍提及 `target_selector` 的注释。

- [ ] **Step 5: 改参数与 CMake**

- `servo_params.hpp`：删除 `target_min_distance_m`、`target_max_distance_m`。
- `servo_node.yaml`：删除 `target_min_distance_m` / `target_max_distance_m` 两行及其注释。
- `servo_node_parameters.yaml`：删除两个声明块。
- `test_param_contract.cpp`：从期望参数名列表中删除 `"target_max_distance_m"`、`"target_min_distance_m"`。
- `CMakeLists.txt`：删除两处 `src/target_selector.cpp`、`test_target_selector` 的 `ament_add_gtest` 块及其 link/dependencies；删除 `target_selector.hpp/.cpp/test_target_selector.cpp` 三个文件。

- [ ] **Step 6: 只编 target 并运行，确认通过**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && cmake --build build/lgw_bt_planner --target test_servo_scan_gate test_param_contract -j4 && \
   ./build/lgw_bt_planner/test_servo_scan_gate && ./build/lgw_bt_planner/test_param_contract"
```
Expected: 两个二进制全部 PASS。

- [ ] **Step 7: 提交**

```bash
git add src/lgw_bt_planner/
git commit -m "refactor(servo): follow selected_track_id, drop local target selector

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Task 8: Phase Gate C 与真机验证

**Files:** 无代码改动。

- [ ] **Step 1: 停链并禁用控制**

按现有流程停机械臂链，确认 `/control_enable false`（避免全零位触发 j3 保护）。

- [ ] **Step 2: 全量构建**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select yoloe_interfaces lgw_3d_tracker lgw_bt_planner"
```
Expected: 三个包构建成功。

- [ ] **Step 3: 全量测试**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon test --packages-select lgw_3d_tracker lgw_bt_planner && \
   colcon test-result --verbose"
```
Expected: 0 failures。

- [ ] **Step 4: 启动链路并做 trace 验证（手动）**

启动感知 + tracker + servo 后确认：

1. `ros2 topic echo /perception/tracks/unified --once` 的 `selected_track_id` 非 0，且 `arm_window_ms == 3000`；
2. 选中目标在视野内移动时 `selected_track_id` 不因旁边出现更近目标而改变；
3. 目标离开视野 40 s 后从列表消失，`selected_track_id` 归 0；
4. `arm_hits_in_window` 在 arm 相机命中时增长、3 s 后回落到 0；
5. 对同一目标连续观测时 trace 的 `matches[].dist` p95 落在 0.15 m 内；若明显超限，记录数据并回报，不擅自改门限。

- [ ] **Step 5: 提交验证记录**

把 trace 结论追加到 spec 的 §15 或新建 `docs/2026-11-17-tracker-selection-verification.md`，然后提交。
