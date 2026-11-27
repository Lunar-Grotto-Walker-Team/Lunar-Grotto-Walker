# Target Registry + Servo Search-and-Track Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tracker becomes the single owner of target state (candidate/active/lost/consumed), publishes the best current arm-base point at observation rate plus a heartbeat; servo becomes a thin follower that scans with MoveIt when the active list is empty.

**Architecture:** Python tracker maintains a global object registry keyed by stable track id, updates its odom KF on every `ObservationResult`, retains lost targets for re-ID, and publishes only `active` targets with `position_arm_base` projected through the current TF. C++ servo consumes a single published arm-base point, removes the head/arm/odom freshness tier machine, selects the nearest active target, and enters a `SCANNING` state (MoveGroup planner takeover, modelled after `RECOVERING`) when no target is available.

**Tech Stack:** ROS 2 Humble, Python 3.10/rclpy/numpy/scipy, C++17/rclcpp/MoveIt Servo/MoveGroup, gtest + pytest.

**Spec:** `docs/superpowers/specs/2026-11-13-target-registry-and-servo-scan-design.md`

## Global Constraints

- Build/test inside the container: `docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c "source /my_ws/l4t/.l4t_env.sh && <command>"`.
- Tracker tests: `python3 -m pytest src/lgw_3d_tracker/test/<file> -q` in the container.
- Servo tests: `colcon build --packages-select lgw_bt_planner && colcon test --packages-select lgw_bt_planner` in the container; single test binary `./build/lgw_bt_planner/test_<name>`.
- Message contract is authoritative: `task_state`, `position`, `position_arm_base`, `geometry_frame`, `geometry_valid`, `source`; `status` and `contributor_*` are removed.
- Tracker updates on every observation; publish is event-driven plus `publish_heartbeat_hz=25`.
- arm source priority > head > last-known; no fusion.
- Only `HOLD + HoldReason::kTargetLost` may enter `SCANNING`; operator pause / follower lag / servo halt / pose stale / tracker stale never scan.
- Scan pauses MoveIt Servo (single JTC writer); j4/j6 from `/feedback/joint_states`; pre-settle before the first scan goal.
- All existing tests must pass after being updated for the new contract; only the tests for code deleted in Task 8 (`test_geometry_source`) are removed.
- Never use `pkill -f ros`; clean container processes by explicit PID.
- Commit messages end with:
  `Co-Authored-By: Claude Code <noreply@anthropic.com>`

## Review Focus

1. **Two cameras observing the same target within a few ms** — each observation must update the KF once (no 30 Hz throttle), but must not create a duplicate candidate or double-count `hits` into an early confirmation. (Task 3/5 tests.)
2. **Lost target re-ID aliasing** — a different object appearing near a lost target's last odom position must not silently reuse its id. Distance/class/TTL gates; ambiguity → new candidate. (Task 3 test.)
3. **Scan safety gate** — an operator pause, dead tracker stream, stale pose, or follower lag must never start arm motion. Only `HOLD + target_lost` + healthy state + stationary base may scan. (Task 9/11 tests.)
4. **Frame validity** — `geometry_frame != "arm1/base_link"` or `geometry_valid == false` must HOLD, never drive. (Task 7/8 test.)
5. **Scan trajectory safety** — j5 must not cross the exact singularity, j4/j6 use measured values, and cancel/timeout must leave Servo paused. (Task 10/11 test.)

---

### Task 0: Branch

**Files:** none (git only)

- [ ] **Step 1: Create the feature branch**

```bash
cd /home/unitree/my_ws/l4t/lg_walker_ws
git checkout -b feature/target-registry-servo-scan
```

- [ ] **Step 2: Confirm the working tree**

Run: `git status --short`
Expected: only the two new docs (`2026-11-13-...`) are untracked.

---

### Task 1: Interface contract — `UnifiedTrack` + `MarkTrack`

**Files:**
- Modify: `src/yoloe_interfaces/msg/UnifiedTrack.msg`
- Create: `src/yoloe_interfaces/srv/MarkTrack.srv`
- Modify: `src/yoloe_interfaces/CMakeLists.txt`
- Test: build + `ros2 interface show`

**Interfaces:**
- Consumes: nothing.
- Produces: `UnifiedTrack` fields listed below; `yoloe_interfaces/srv/MarkTrack` with `uint64 track_id`, `uint8 action`, response `bool success`, `string message`.

- [ ] **Step 1: Replace `msg/UnifiedTrack.msg` with the new contract**

```text
builtin_interfaces/Time stamp
uint32 frame_stamp_ms

uint64 track_id
string task_state              # candidate | active | lost | consumed (wire: active)
string class_name
float32 confidence
int32 age
int32 observation_count
int32 confirmed_hits

# odom KF state
geometry_msgs/Point position
float64[9] position_covariance

# best current geometry (arm base_link), projected at publish time
geometry_msgs/Point position_arm_base
string geometry_frame
bool geometry_valid
string source                  # arm | head | odom

builtin_interfaces/Time last_observation_stamp
int32 time_since_last_observation_ms

sensor_msgs/RegionOfInterest last_bbox_2d
string last_source_camera
float32[3] last_bbox_color
string[] source_cameras
float32 width_m
float32 length_m
float32 height_m
bool size_valid
```

- [ ] **Step 2: Create `srv/MarkTrack.srv`**

```text
uint64 track_id
uint8 action          # 1=CONSUMED, 2=IGNORED, 3=REACTIVATE
---
bool success
string message
```

- [ ] **Step 3: Add the srv to `CMakeLists.txt`**

In `rosidl_generate_interfaces`, add `"srv/MarkTrack.srv"` next to the msg entries. Add
`DEPENDENCIES ...` unchanged (std_msgs/geometry_msgs/sensor_msgs/builtin_interfaces).

- [ ] **Step 4: Build and verify**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select yoloe_interfaces && \
   source install/setup.bash && ros2 interface show yoloe_interfaces/srv/MarkTrack"
```

Expected: build succeeds; the interface prints the two fields and response.

- [ ] **Step 5: Commit**

```bash
git add src/yoloe_interfaces/msg/UnifiedTrack.msg src/yoloe_interfaces/srv/MarkTrack.srv src/yoloe_interfaces/CMakeLists.txt
git commit -m "feat(interfaces): target task state, live arm-base point, MarkTrack srv

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 2: Tracker unified state machine

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py`
- Test: `src/lgw_3d_tracker/test/test_task_state.py` (create)

**Interfaces:**
- Consumes: `TrackObservation`, `KalmanState3D`.
- Produces:
  - `TrackState(str, Enum)` values `CANDIDATE`, `ACTIVE`, `LOST`, `CONSUMED`, `ARCHIVED`.
  - `Track` fields: `task_state: TrackState`, `first_seen_ms: int`, `last_seen_ms: int`.
  - `Track.tick(now_ms: int, dt_ms: int, candidate_min_hits: int, candidate_min_age_ms: int, candidate_ttl_ms: int, visible_timeout_ms: int, lost_ttl_ms: int) -> None`.
  - `Track.refresh(task_state: TrackState, now_ms: int) -> None` (re-ID/consumed helper).

- [ ] **Step 1: Write the failing tests**

Create `test/test_task_state.py`:

```python
import numpy as np
from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.track import (
    Track, TrackObservation, TrackState)


def _obs(cam="camera_arm_color_optical_frame", x=0.0):
    return TrackObservation(
        class_name="sample", confidence=0.8, source_camera=cam,
        stamp_ms=0, position_odom=np.array([x, 0.0, 1.0]))


def _track():
    return Track(id=1, created_ms=0, first_seen_ms=0,
                 state=KalmanState3D(np.zeros(3), np.eye(3) * 0.05))


def test_candidate_needs_hits_and_age():
    t = _track()
    t.apply_observation(_obs(), lambda s, z: s)
    t.apply_observation(_obs(), lambda s, z: s)
    t.tick(100, 50, 3, 200, 2000, 5000, 60000)
    assert t.task_state == TrackState.CANDIDATE
    t.apply_observation(_obs(), lambda s, z: s)
    t.tick(300, 100, 3, 200, 2000, 5000, 60000)
    assert t.task_state == TrackState.ACTIVE


def test_candidate_ttl_archives():
    t = _track()
    t.tick(2001, 100, 3, 0, 2000, 5000, 60000)
    assert t.task_state == TrackState.ARCHIVED


def test_active_to_lost_and_lost_ttl():
    t = _track()
    t.task_state = TrackState.ACTIVE
    t.last_seen_ms = 0
    t.tick(5001, 100, 3, 0, 2000, 5000, 60000)
    assert t.task_state == TrackState.LOST
    t.tick(65002, 100, 3, 0, 2000, 5000, 60000)
    assert t.task_state == TrackState.ARCHIVED


def test_consumed_stays_consumed():
    t = _track()
    t.task_state = TrackState.ACTIVE
    t.refresh(TrackState.CONSUMED, 10)
    t.tick(100000, 100, 3, 0, 2000, 5000, 60000)
    assert t.task_state == TrackState.CONSUMED
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_task_state.py -q"
```

Expected: `ImportError: cannot import name 'TrackState'`.

- [ ] **Step 3: Implement the state machine in `track.py`**

Replace `TrackStatus` with `TrackState`; add `first_seen_ms`/`last_seen_ms`; replace
`tick(...)` with the signature above. Rules:

```
CANDIDATE: hits >= candidate_min_hits and now_ms - first_seen_ms >= candidate_min_age_ms
           -> ACTIVE
           else now_ms - created_ms > candidate_ttl_ms -> ARCHIVED
ACTIVE:    now_ms - last_seen_ms > visible_timeout_ms -> LOST
LOST:      now_ms - last_seen_ms > lost_ttl_ms -> ARCHIVED
CONSUMED/ARCHIVED: no automatic transition
```

`Track.apply_observation` keeps its existing signature; `Tracker.associate_and_update`
sets `t.last_seen_ms = self._now_ms` after the call and resets
`time_since_last_obs_ms`. `refresh(state, now_ms)` sets `task_state` and
`last_seen_ms`.

- [ ] **Step 4: Run to verify it passes**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_task_state.py -q"
```

Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py src/lgw_3d_tracker/test/test_task_state.py
git commit -m "feat(tracker): unified candidate/active/lost/consumed state machine

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 3: Tracker registry, re-ID, consumed, capacity

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py`
- Test: `src/lgw_3d_tracker/test/test_registry.py` (create)

**Interfaces:**
- Consumes: `TrackState`, `Track`, `TrackObservation`, `TwoStageAssigner`.
- Produces:
  - `Tracker.__init__(..., candidate_min_hits, candidate_min_age_ms, candidate_ttl_ms, visible_timeout_ms, lost_ttl_ms, distance_gate_m, duplicate_track_distance_m, reid_distance_m, reid_reinit_distance_m, max_registry_size, consumed_suppress_enable=False, consumed_suppress_radius_m=0.15, consumed_suppress_ttl_ms=10000)`.
  - `Tracker.active_tracks() -> List[Track]`.
  - `Tracker.all_tracks() -> List[Track]`.
  - `Tracker.mark_track(track_id: int, action: int) -> bool` with `1=CONSUMED`, `2=IGNORED`, `3=REACTIVATE`.
  - `Tracker.associate_and_update(observations: List[TrackObservation]) -> AssociationReport` (re-ID second level).
  - `Tracker.tick_lifecycle(dt_ms: int) -> List[Track]` returns active tracks only.

- [ ] **Step 1: Write the failing tests**

Create `test/test_registry.py`:

```python
import numpy as np
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.track import TrackObservation, TrackState
from lgw_3d_tracker.unified_track.tracker import Tracker


def _tracker():
    return Tracker(
        kf=KalmanFilter3D(0.05, 0.03),
        assigner=TwoStageAssigner(0.4, 0.15, lambda s, z: True),
        candidate_min_hits=3, candidate_min_age_ms=0,
        candidate_ttl_ms=2000, visible_timeout_ms=5000,
        lost_ttl_ms=60000, distance_gate_m=0.15,
        duplicate_track_distance_m=0.04,
        reid_distance_m=0.3, reid_reinit_distance_m=0.1,
        max_registry_size=100)


def _obs(x, cam="camera_arm_color_optical_frame", cls="sample"):
    return TrackObservation(cls, 0.8, cam, 0, np.array([x, 0.0, 1.0]))


def _feed(tr, x=0.0, n=3):
    for _ in range(n):
        tr.set_now_ms(0)
        tr.predict_all()
        tr.associate_and_update([_obs(x)])
        tr.tick_lifecycle(100)


def test_lost_track_is_retained_and_reactivated_by_reid():
    tr = _tracker()
    _feed(tr)
    tid = tr.all_tracks()[0].id
    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)
    assert tr.all_tracks()[0].task_state == TrackState.LOST
    assert tr.active_tracks() == []
    tr.set_now_ms(7000)
    tr.predict_all()
    tr.associate_and_update([_obs(0.05)])
    tr.tick_lifecycle(100)
    assert [t.id for t in tr.active_tracks()] == [tid]


def test_reid_gate_rejects_far_object():
    tr = _tracker()
    _feed(tr)
    old_id = tr.all_tracks()[0].id
    tr.set_now_ms(6000)
    tr.tick_lifecycle(100)
    tr.set_now_ms(7000)
    tr.predict_all()
    tr.associate_and_update([_obs(1.0)])   # 1 m away, outside 0.3 m gate
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


def test_registry_capacity_evicts_oldest_lost_then_candidate():
    tr = _tracker()
    tr.max_registry_size = 2
    for x in (0.0, 1.0, 2.0):
        tr.set_now_ms(0)
        tr.predict_all()
        tr.associate_and_update([_obs(x)])
        tr.tick_lifecycle(100)
    assert len(tr.all_tracks()) <= 2
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_registry.py -q"
```

Expected: failures on missing `active_tracks`/`mark_track`/re-ID.

- [ ] **Step 3: Implement the registry**

In `Tracker`:

1. Constructor stores the new fields and `_next_id`, `_tracks`.
2. `associate_and_update`:
   - normal association against `ACTIVE` + `CANDIDATE` (not `LOST`);
   - unmatched observations → **re-ID** against `LOST` tracks with same
     `last_obs.class_name`, `|obs.position_odom - t.state.x| < reid_distance_m`,
     `_now_ms - t.last_seen_ms <= lost_ttl_ms`; choose nearest; if the distance
     `> reid_reinit_distance_m`, reset `t.state = KalmanState3D(obs.position_odom.copy(), np.eye(3)*0.05)`,
     else `kf.update`; set `task_state=ACTIVE`, `last_seen_ms=_now_ms`;
   - remaining → new `Track(task_state=CANDIDATE, first_seen_ms=_now_ms, last_seen_ms=_now_ms)`.
   - `CONSUMED`/`ARCHIVED` never match.
3. `tick_lifecycle`: call `tick(...)` with the new params; drop `ARCHIVED` from
   `_tracks`; enforce `max_registry_size` by evicting oldest `LOST`, then oldest
   `CANDIDATE`; return `active_tracks()`.
4. `mark_track`: `1/2` → `CONSUMED`; `3` → `ACTIVE` + `last_seen_ms=_now_ms`;
   unknown id → false.
5. Consumed suppression (only when `consumed_suppress_enable`): before creating
   a new candidate, skip observations within `consumed_suppress_radius_m` of a
   `CONSUMED` track's last `state.x` whose `last_seen_ms` is within
   `consumed_suppress_ttl_ms`; default off (v1).

- [ ] **Step 4: Run to verify it passes**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_registry.py -q"
```

Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/tracker.py src/lgw_3d_tracker/test/test_registry.py
git commit -m "feat(tracker): registry, lost re-ID, consumed marking, capacity

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 4: Source selection and arm-base projection

**Files:**
- Create: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/geometry_projector.py`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py`
- Test: `src/lgw_3d_tracker/test/test_projection.py` (create)

**Interfaces:**
- Produces:
  - `def short_camera_name(frame: str) -> str` (`"head"`/`"arm"`/raw).
  - `def choose_source(track, now_ms: int, priority: list[str], max_age_ms: int) -> tuple[str, Optional[TrackObservation]]`.
  - `def project_odom_to_arm_base(p_odom: np.ndarray, T_base_odom: np.ndarray) -> np.ndarray`.
  - `OutputFormatter.format(..., T_base_odom: Optional[np.ndarray], observation_max_age_ms: int, source_priority: list[str])`.

- [ ] **Step 1: Write the failing tests**

Create `test/test_projection.py`:

```python
import numpy as np
from lgw_3d_tracker.unified_track.geometry_projector import (
    choose_source, project_odom_to_arm_base)
from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.track import (
    Track, TrackObservation, TrackState)


def _track_with(cams):
    t = Track(id=1, created_ms=0, first_seen_ms=0,
              state=KalmanState3D(np.array([1.0, 2.0, 3.0]), np.eye(3)))
    t.task_state = TrackState.ACTIVE
    for cam, stamp in cams.items():
        t.last_obs_by_camera[cam] = TrackObservation(
            "sample", 0.8, cam, stamp, np.array([1.0, 2.0, 3.0]))
    return t


def test_arm_wins_when_both_fresh():
    t = _track_with({"camera_arm_color_optical_frame": 1000,
                     "camera_head_color_optical_frame": 1000})
    src, obs = choose_source(t, 1000, ["arm", "head"], 500)
    assert src == "arm"
    assert obs.source_camera == "camera_arm_color_optical_frame"


def test_head_used_when_arm_stale():
    t = _track_with({"camera_arm_color_optical_frame": 0,
                     "camera_head_color_optical_frame": 1000})
    src, obs = choose_source(t, 1200, ["arm", "head"], 500)
    assert src == "head"


def test_odom_when_nothing_fresh():
    t = _track_with({"camera_arm_color_optical_frame": 0})
    src, obs = choose_source(t, 2000, ["arm", "head"], 500)
    assert src == "odom"
    assert obs is None


def test_project_uses_current_base_pose():
    T = np.eye(4)
    T[:3, 3] = [1.0, 0.0, 0.0]       # base at (1,0,0) in odom
    p = project_odom_to_arm_base(np.array([2.0, 0.0, 0.0]), T)
    assert np.allclose(p, [1.0, 0.0, 0.0])
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_projection.py -q"
```

Expected: `ModuleNotFoundError: geometry_projector`.

- [ ] **Step 3: Implement**

`geometry_projector.py`:

```python
def short_camera_name(frame: str) -> str:
    lowered = str(frame).lower()
    if "head" in lowered:
        return "head"
    if "arm" in lowered:
        return "arm"
    return str(frame)


def choose_source(track, now_ms, priority, max_age_ms):
    for name in priority:
        for frame, obs in track.last_obs_by_camera.items():
            if short_camera_name(frame) != name:
                continue
            if now_ms - obs.stamp_ms <= max_age_ms:
                return name, obs
    return "odom", None


def project_odom_to_arm_base(p_odom, T_base_odom):
    p = np.append(np.asarray(p_odom, dtype=float), 1.0)
    return (np.linalg.inv(T_base_odom) @ p)[:3]
```

`OutputFormatter.format`: for each `active` track, choose source; if `obs` is not
None use `obs.position_odom` and `obs.stamp_ms`, else `track.state.x` and
`track.last_seen_ms`; `position_arm_base = project_odom_to_arm_base(...)` when
`T_base_odom` is not None, else keep `track.last_arm_base` and set
`geometry_valid=False`. Add `last_arm_base` to `Track` if not present. Remove
`contributor_*` output. Copy the remaining diagnostic fields unchanged:
`position_covariance` (from `track.state.P.flatten()`),
`age`, `observation_count`, `confirmed_hits`, `last_bbox_2d`,
`last_source_camera`, `last_bbox_color`, `source_cameras`, size fields, and
`time_since_last_observation_ms`.

- [ ] **Step 4: Run to verify it passes**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_projection.py -q"
```

Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_3d_tracker/lgw_3d_tracker/unified_track/geometry_projector.py \
  src/lgw_3d_tracker/lgw_3d_tracker/unified_track/output_formatter.py \
  src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py \
  src/lgw_3d_tracker/test/test_projection.py
git commit -m "feat(tracker): arm>head>odom source selection and live arm-base projection

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 5: Tracker config + node pipeline (per-observation update, event publish, heartbeat, MarkTrack)

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/config.py`
- Modify: `src/lgw_3d_tracker/config/runtime.yaml`
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track_node.py`
- Test: `src/lgw_3d_tracker/test/test_config.py` (create), `src/lgw_3d_tracker/test/test_node_publish.py` (create)

**Interfaces:**
- Consumes: `UnifiedTrackArray`, `MarkTrack`, `InputAdapter`, `OutputFormatter`, `Tracker`.
- Produces: `UnifiedTrackConfig` fields from the spec §9.7; node parameters
  `publish_heartbeat_hz`, `mark_track` service.

- [ ] **Step 1: Write the failing config test**

Create `test/test_config.py`:

```python
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig


def test_new_registry_defaults():
    c = UnifiedTrackConfig()
    assert c.candidate_min_hits == 3
    assert c.candidate_ttl_ms == 2000
    assert c.visible_timeout_ms == 5000
    assert c.lost_ttl_ms == 60000
    assert c.observation_max_age_ms == 500
    assert c.publish_heartbeat_hz == 25
    assert c.source_priority == ["arm", "head"]
    assert c.reid_distance_m == 0.3
    assert c.max_registry_size == 100


def test_yaml_overrides(tmp_path):
    p = tmp_path / "runtime.yaml"
    p.write_text("unified_track:\n  visible_timeout_ms: 7000\n")
    assert UnifiedTrackConfig.from_yaml(str(p)).visible_timeout_ms == 7000
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_config.py -q"
```

Expected: `AttributeError: candidate_min_hits`.

- [ ] **Step 3: Implement config**

Add the fields from spec §9.7; keep `association_distance_gate_m`,
`duplicate_track_distance_m`, `process_noise_position`,
`observation_noise_position`, `two_stage`, `arm_frame_prefix`,
`arm_mount_offset`. Remove `coast_max_age_ms` / `coast_enter_ms` (ignore them
in `from_yaml` with a warning if present). Update `config/runtime.yaml` with the
new keys and remove the old ones.

- [ ] **Step 4: Write the failing node test**

Create `test/test_node_publish.py` (pure formatter-level test; no ROS spin):

```python
import numpy as np
from builtin_interfaces.msg import Time
from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.track import (
    Track, TrackObservation, TrackState)


def test_publishes_only_active_with_arm_base():
    t = Track(id=7, created_ms=0, first_seen_ms=0,
              state=KalmanState3D(np.array([1.0, 0.0, 1.0]), np.eye(3)))
    t.task_state = TrackState.ACTIVE
    t.last_seen_ms = 1000
    t.last_obs_by_camera["camera_arm_color_optical_frame"] = TrackObservation(
        "sample", 0.8, "camera_arm_color_optical_frame", 1000,
        np.array([1.0, 0.0, 1.0]))
    T = np.eye(4)
    T[0, 3] = 1.0
    fmt = OutputFormatter("odom", "arm1/base_link")
    arr = fmt.format([t], 1, 1000, Time(sec=1), T, 500, ["arm", "head"])
    assert len(arr.tracks) == 1
    assert arr.tracks[0].task_state == "active"
    assert arr.tracks[0].source == "arm"
    assert arr.tracks[0].geometry_valid is True
    assert np.isclose(arr.tracks[0].position_arm_base.x, 0.0)
```

- [ ] **Step 5: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_node_publish.py -q"
```

Expected: formatter signature/fields mismatch.

- [ ] **Step 6: Rework the node**

In `unified_track_node.py`:

1. Delete the 30 ms debounce (`_request_pipeline`, `_on_timer_fire`,
   `_pending_head/_pending_arm`, `_last_run_ns`).
2. `_on_head_detection` / `_on_arm_detection` call
   `_process_observation(msg)` immediately.
3. `_process_observation(msg)`:
   ```
   now_ms = self.get_clock().now().nanoseconds // 1_000_000
   frame_stamp_ms = (time.monotonic_ns() - _start_time_ns)//1_000_000
   dt_ms = max(0, now_ms - _last_update_ms)
   obs = self._input_adapter.adapt(msg)
   self._tracker.set_now_ms(now_ms)
   self._tracker.predict_all()
   report = self._tracker.associate_and_update(obs)
   active = self._tracker.tick_lifecycle(dt_ms)
   _last_update_ms = now_ms
   self._publish_active()
   trace...
   ```
   **Time base:** `now_ms` must be wall-clock milliseconds so it is comparable
   with `TrackObservation.stamp_ms` (epoch ms) in `choose_source`; do not use
   the monotonic clock for `set_now_ms`. `frame_stamp_ms` stays relative for the
   trace only.
4. `_publish_active()`: look up `odom ← base_link` with a zero-timeout
   `can_transform`; pass `T_base_odom` to the formatter; publish.
5. Add `create_timer(1.0/publish_heartbeat_hz, self._publish_active)`.
6. Add `create_service(MarkTrack, "/lgw/tracker/mark_track", self._on_mark_track)`.
7. Keep the odom TF broadcaster and the static mount bridges unchanged.

- [ ] **Step 7: Run the tracker tests**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test -q"
```

Expected: all pass (update the old lifecycle/dedupe/trace tests to the new
`task_state` API in the same task).

- [ ] **Step 8: Commit**

```bash
git add src/lgw_3d_tracker
git commit -m "feat(tracker): per-observation updates, event publish, heartbeat, MarkTrack

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 6: Tracker trace/scripts adapt

**Files:**
- Modify: `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/trace_logger.py`
- Modify: `scripts/fake_track_publisher.py`
- Modify: `scripts/analyze_tracker_trace.py`
- Test: `src/lgw_3d_tracker/test/test_trace_logging.py` (update)

**Interfaces:**
- Produces: trace JSONL `task_state` field; fake publisher now fills
  `task_state="active"`, `position_arm_base`, `geometry_valid=True`, `source`.

- [ ] **Step 1: Update the trace test**

In `test_trace_logging.py`, replace `status` assertions with `task_state` and
assert each tick writes the new value. Add a fake publisher smoke check:
`scripts/fake_track_publisher.py --help` exits 0.

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_trace_logging.py -q"
```

Expected: old `status` field assertion fails.

- [ ] **Step 3: Implement**

- Trace snapshot: `"task_state": t.task_state.value`.
- `fake_track_publisher.py`: build `UnifiedTrack` with the new fields; publish at
  20 Hz; default `position_arm_base` = `position`; `geometry_valid=True`;
  `task_state="active"`.
- `analyze_tracker_trace.py`: read `task_state` (fall back to `status` for old
  logs).

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test/test_trace_logging.py -q && \
   python3 scripts/fake_track_publisher.py --help >/dev/null"
```

Expected: pass; help exits 0.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_3d_tracker scripts/fake_track_publisher.py scripts/analyze_tracker_trace.py
git commit -m "chore(tracker): trace and tooling for task_state contract

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 7: Servo target selector and snapshot contract

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp`
- Modify: `src/lgw_bt_planner/src/target_selector.cpp`
- Test: `src/lgw_bt_planner/tests/test_target_selector.cpp`

**Interfaces:**
- Produces:
  ```
  struct TrackCandidate { uint64_t track_id; std::string task_state;
                          std::string class_name; tf2::Vector3 p_odom;
                          int64_t last_seen_ns; };
  struct TargetSelectorParams { uint64_t requested_track_id; std::string class_filter; };
  struct TargetSelectorState { uint64_t last_track_id; };
  ```
- Removes `reacquire_on_loss`, `reacquire_radius_m`, `last_known_p_odom`,
  `last_class`, `has_last`.
- `TrackSnapshot` is **not** added here: it would collide with the existing
  struct in `geometry_source.hpp`. Task 8 deletes that header and adds the new
  `TrackSnapshot` to `servo_types.hpp`.

- [ ] **Step 1: Update the selector tests**

In `tests/test_target_selector.cpp`:
- `mk(track_id, task_state, class_name, p_odom)`.
- `RequestedTrackWins`: requested id found even if it is not `active`? The wire
  only carries `active`; keep the requested branch status-agnostic.
- `AutoPicksNearestActive`: a `lost`/`coasting` candidate (use `task_state="lost"`)
  is skipped; nearest `active` wins.
- `MissingRequestedHolds`: no reacquisition.
- Delete any `reacquire` test.

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_target_selector"
```

Expected: compile error (`status` / `reacquire_on_loss`).

- [ ] **Step 3: Implement**

`target_selector.hpp`: new structs/signatures as above. `target_selector.cpp`:

```
requested != 0: find id; if found ok; else reason="requested_missing", ok=false.
requested == 0:
  if (!p_base_in_odom) reason="no_base_pose", ok=false;
  for each c: if class_filter non-empty and c.class_name != filter -> skip;
              if c.task_state != "active" -> skip;
              nearest by |c.p_odom - *p_base_in_odom|;
  none -> reason="no_candidate";
state.last_track_id = hit.track_id;
```

`servo_types.hpp`: add `TrackSnapshot`; add `SCANNING` to `ServoState` and
`state_name` in Task 9; remove `SourceKind`/`SourceSelection` in Task 8.

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_target_selector"
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/target_selector.hpp \
  src/lgw_bt_planner/src/target_selector.cpp \
  src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp \
  src/lgw_bt_planner/tests/test_target_selector.cpp
git commit -m "refactor(servo): task_state selector and thin TrackSnapshot

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 8: Servo consumes the published arm-base point; remove tier logic

**Files:**
- Modify: `src/lgw_bt_planner/src/servo/servo_node.cpp`
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_node.hpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_tick.cpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_recovery.cpp`
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/tracking_loop.hpp`
- Modify: `src/lgw_bt_planner/src/servo/tracking_loop.cpp`
- Delete: `src/lgw_bt_planner/include/lgw_bt_planner/geometry_source.hpp`
- Delete: `src/lgw_bt_planner/src/geometry_source.cpp`
- Delete: `src/lgw_bt_planner/tests/test_geometry_source.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`
- Test: `src/lgw_bt_planner/tests/test_servo_node_tick.cpp`,
  `src/lgw_bt_planner/tests/test_tracking_loop.cpp`,
  `src/lgw_bt_planner/tests/test_servo_node_header_includes.cpp`

**Interfaces:**
- Consumes: `UnifiedTrack::position_arm_base`, `geometry_valid`, `source`,
  `last_observation_stamp`, `task_state`.
- Produces:
  ```cpp
  struct TrackSnapshot {
    uint64_t track_id{0};
    std::string task_state;
    std::string source;
    tf2::Vector3 p_odom{0, 0, 0};
    tf2::Vector3 p_arm_base{0, 0, 0};
    bool geometry_valid{false};
    int64_t last_seen_ns{0};
  };
  ```
  `TrackSnapshot` moves into `servo_types.hpp`; `TrackingInput` gains
  `bool geometry_valid` and loses `int64_t target_stamp_ns`.

- [ ] **Step 1: Write the failing node test**

Add to `tests/test_servo_node_tick.cpp`:

```cpp
TEST(ServoNodeTick, SnapshotUsesPublishedArmBaseAndGeometryValid)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  yoloe_interfaces::msg::UnifiedTrack track;
  track.track_id = 7;
  track.task_state = "active";
  track.geometry_frame = "arm1/base_link";
  track.geometry_valid = true;
  track.source = "arm";
  track.position.x = 1.0;
  track.position_arm_base.x = 0.5;
  track.position_arm_base.z = 0.4;
  track.last_observation_stamp = node->now().to_msg();
  const auto snap = node->make_snapshot_for_test(track);
  EXPECT_EQ(snap.track_id, 7u);
  EXPECT_TRUE(snap.geometry_valid);
  EXPECT_NEAR(snap.p_arm_base.x(), 0.5, 1e-9);
  EXPECT_EQ(snap.source, "arm");
}
```

Add `make_snapshot_for_test` as a public thin wrapper if needed.

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_servo_node_tick"
```

Expected: compile error (missing `position_arm_base` on snapshot / no
`make_snapshot_for_test`).

- [ ] **Step 3: Implement**

- `make_snapshot`: copy `track.task_state`, `track.geometry_valid`,
  `track.source`, `track.position`, `track.position_arm_base`,
  `time_to_ns(track.last_observation_stamp)`.
- `flatten_candidates`: copy `task_state`, `p_odom`, `last_seen_ns`.
- `reselect_locked`: after a successful selection, if the new `track_id`
  differs from `tracking_track_id_`, call `loop_.reset()` before replacing the
  snapshot (target switch must not carry the old feed-forward/control-law
  memory); then assign `tracking_track_id_`.
- `on_track_array`: keep the `geometry_frame` check but make it a hard failure:
  if non-empty and != `base_frame_`, log ERROR and `hold_and_return(HoldReason::kPoseStale)`.
- `update_tracking_locked`: replace `select_source(...)` with
  `const TrackSnapshot & snap = selected_snapshot_;`
  `target_ok = has_selected_snapshot_ && snap.geometry_valid && stream_alive;`
  use `snap.p_arm_base` everywhere (solver, recovery, diagnostics);
  `geometry_source_ = snap.source`.
- `TrackingInput`: remove `target_stamp_ns`; add `bool geometry_valid`.
  `TrackingLoop::step`: replace the stale-stamp check with
  `if (!in.geometry_valid) return kTrackerStale;`; remove `kTargetStaleNs`.
- `servo_recovery.cpp::resolve_recovery_target_locked`: use
  `selected_snapshot_.p_arm_base` and `selected_snapshot_.last_seen_ns`.
- Remove `geometry_source.hpp/.cpp`, its test, and the CMake source/test
  entries; remove `GeometrySourceParams geom` from `ServoParams`; remove
  `SourceKind`/`SourceSelection` from `servo_types.hpp`; remove
  `Contributor`/`select_source` includes.
- Remove `geom_params_`, `source_state_` members.

- [ ] **Step 4: Run the servo tests**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   colcon test --packages-select lgw_bt_planner && colcon test-result --verbose"
```

Expected: 0 failures. Update `test_tracking_loop.cpp` (remove
`target_stamp_ns`, add `geometry_valid`), `test_servo_params.cpp` /
`test_param_contract.cpp` (remove geom params) in this task.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner
git commit -m "refactor(servo): consume published arm-base point; delete geometry tiers

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 9: Servo SCANNING state machine

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp`
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp`
- Modify: `src/lgw_bt_planner/src/servo_state_machine.cpp`
- Test: `src/lgw_bt_planner/tests/test_servo_state_machine.cpp`

**Interfaces:**
- Produces: `ServoState::SCANNING`; events `SCAN_START`, `SCAN_FOUND`,
  `SCAN_TIMEOUT`, `SCAN_FAIL`; `state_name` returns `"SCANNING"`.

- [ ] **Step 1: Add failing tests**

```cpp
TEST(StateMachine, ScanOnlyFromHoldAndReturnsToPreempt)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));   // TRACKING -> HOLD
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  EXPECT_EQ(sm.state(), ServoState::SCANNING);
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_FOUND));
  EXPECT_EQ(sm.state(), ServoState::PREEMPT);
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  EXPECT_EQ(sm.state(), ServoState::TRACKING);
}

TEST(StateMachine, ScanTimeoutGoesHoldAndReleaseLeaves)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(sm.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_TRUE(sm.handle(ServoEvent::TARGET_LOST));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_TIMEOUT));
  EXPECT_EQ(sm.state(), ServoState::HOLD);
  ASSERT_TRUE(sm.handle(ServoEvent::SCAN_START));
  ASSERT_TRUE(sm.handle(ServoEvent::RELEASE));
  EXPECT_EQ(sm.state(), ServoState::RELEASING);
}

TEST(StateMachine, ScanRejectedOutsideHold)
{
  ServoStateMachine sm;
  ASSERT_TRUE(sm.handle(ServoEvent::TAKEOVER));
  EXPECT_FALSE(sm.handle(ServoEvent::SCAN_START));   // PREEMPT
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_servo_state_machine"
```

Expected: compile error (`SCANNING`).

- [ ] **Step 3: Implement**

- `ServoState::SCANNING` and `state_name`.
- `ServoEvent` new values.
- `handle`:
  - `HOLD + SCAN_START` → SCANNING (clear `target_ok_`).
  - `SCANNING + SCAN_FOUND` → PREEMPT.
  - `SCANNING + SCAN_TIMEOUT/SCAN_FAIL` → HOLD.
  - `SCANNING + RELEASE` → RELEASING.
  - `SCANNING + PAUSE` accepted no-op.
  - `SCANNING + TARGET_OK` sets latch, `TARGET_LOST` clears it (stay).
  - `SCANNING + RESUME/TAKEOVER` rejected.
  - `SCANNING + FAULT` → FAULT via the top-level handler.

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_servo_state_machine"
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo_types.hpp \
  src/lgw_bt_planner/include/lgw_bt_planner/servo_state_machine.hpp \
  src/lgw_bt_planner/src/servo_state_machine.cpp \
  src/lgw_bt_planner/tests/test_servo_state_machine.cpp
git commit -m "feat(servo): SCANNING state transitions

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 10: Scan waypoint generator

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_scan.hpp`
- Create: `src/lgw_bt_planner/src/servo/servo_scan.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt`
- Test: `src/lgw_bt_planner/tests/test_servo_scan.cpp`

**Interfaces:**
- Produces:
  ```cpp
  struct ScanParams {
    double j1_min_rad{-1.047}, j1_max_rad{1.047}, j1_step_rad{0.1745};
    double j5_min_rad{-1.0}, j5_max_rad{-0.3};
    double j5_min_rad_pos{0.3}, j5_max_rad_pos{1.0};
    double j5_step_rad{0.2}, j5_gap_rad{0.3};
    double j2_raise_rad{0.0}, j3_raise_rad{0.0};
  };
  std::vector<std::array<double, 6>> make_scan_waypoints(
    const std::array<double, 6> & measured, const ScanParams & p);
  ```

- [ ] **Step 1: Write the failing test**

```cpp
#include "lgw_bt_planner/servo/servo_scan.hpp"
#include <gtest/gtest.h>

TEST(ScanWaypoints, PreservesJ4J6AndRaisesJ2J3)
{
  const std::array<double, 6> q{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  lgw_bt_planner::ScanParams p;
  p.j2_raise_rad = 0.8;
  p.j3_raise_rad = -0.4;
  const auto wps = lgw_bt_planner::make_scan_waypoints(q, p);
  ASSERT_FALSE(wps.empty());
  for (const auto & w : wps) {
    EXPECT_NEAR(w[1], 0.8, 1e-9);
    EXPECT_NEAR(w[2], -0.4, 1e-9);
    EXPECT_NEAR(w[3], 0.4, 1e-9);
    EXPECT_NEAR(w[5], 0.6, 1e-9);
  }
}

TEST(ScanWaypoints, CoversBothJ5BranchesAndAvoidsGap)
{
  const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
  lgw_bt_planner::ScanParams p;
  const auto wps = lgw_bt_planner::make_scan_waypoints(q, p);
  bool neg = false, pos = false;
  for (const auto & w : wps) {
    EXPECT_LE(std::abs(w[4]), 0.3 + 1e-9) << "j5 inside singularity gap";
    if (w[4] <= p.j5_max_rad + 1e-9) neg = true;
    if (w[4] >= p.j5_min_rad_pos - 1e-9) pos = true;
  }
  EXPECT_TRUE(neg);
  EXPECT_TRUE(pos);
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner"
```

Expected: missing header.

- [ ] **Step 3: Implement**

`make_scan_waypoints`: start with the measured pose (so the planner has a
reachable seed), then for each `j1` from `j1_min` to `j1_max` step
`j1_step`, emit for each `j5` in the negative branch then the positive branch:
`{j1, j2_raise, j3_raise, measured[3], j5, measured[5]}`. Clamp j5 to the
branch ranges; never emit `|j5| < j5_gap_rad`.

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_servo_scan"
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_scan.hpp \
  src/lgw_bt_planner/src/servo/servo_scan.cpp \
  src/lgw_bt_planner/tests/test_servo_scan.cpp src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(servo): joint-space scan waypoint generator

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 11: Scan worker and triggering

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_node.hpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_node.cpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_tick.cpp`
- Modify: `src/lgw_bt_planner/src/servo/servo_recovery.cpp` (release/abort hooks)
- Test: `src/lgw_bt_planner/tests/test_servo_node_tick.cpp` (scan gate),
  `src/lgw_bt_planner/tests/test_servo_scan_gate.cpp` (create; gate + state
  transitions). The full MoveGroup worker is exercised by the mock run in
  Task 13; do not build a fake MoveGroup action server here.

**Interfaces:**
- Produces:
  - `void ServoNode::maybe_start_scan_locked(int64_t now_ns, bool target_ok);`
  - `void ServoNode::scan_worker();`
  - `void ServoNode::cancel_scan();`
  - Members: `scan_in_progress_`, `scan_target_found_`, `scan_cancel_`,
    `scan_thread_`, `scan_attempts_`, `scan_exhausted_`, `scan_last_end_ns_`,
    `scan_params_`.

- [ ] **Step 1: Write the failing gate tests**

```cpp
TEST(ServoNodeScanGate, DoesNotScanOnOperatorPause)
{
  // construct node, drive it into HOLD with kOperatorPause, call
  // maybe_start_scan_locked(now, false) twice; assert state stays HOLD and
  // scan_in_progress_for_test() == false.
}

TEST(ServoNodeScanGate, ScansOnTargetLost)
{
  // HOLD + kTargetLost + no target -> after scan_start_delay_ms the state is
  // SCANNING (or scan_in_progress_for_test() true).
}
```

Use the existing `state_for_test()` and add `hold_reason_for_test()` /
`scan_in_progress_for_test()` accessors if missing.

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_servo_node_tick"
```

Expected: missing scan gate.

- [ ] **Step 3: Implement**

1. In `update_tracking_locked` HOLD branch, after the target/TARGET_OK logic:
   ```
   if (hold_reason_ == HoldReason::kTargetLost && !target_ok) {
     maybe_start_scan_locked(now_ns, target_ok);
   }
   ```
2. `maybe_start_scan_locked` gates:
   - `sm_.state() == HOLD && hold_reason_ == kTargetLost`
   - `!scan_in_progress_`, `!scan_exhausted_`
   - `now_ns - scan_last_end_ns_ >= scan_retry_cooldown_s*1e9`
   - `stream_alive`, `pose_seen_`, `measured_complete_locked()`
   - `!scan_require_base_static_ || base_is_static_locked()`
   - `now_ns - hold_since_ns_ >= scan_start_delay_ms*1e6`
   Then `sm_.handle(SCAN_START)`, set `scan_in_progress_`, `scan_target_found_=false`,
   start `scan_thread_`.
3. `scan_worker`:
   - call `pause_servo` (blocking, mutex released);
   - wait for pre-settle (`recovery_presettle_gap_rad`/`_stable_ms`);
   - `make_scan_waypoints(measured_positions(), scan_params_)`;
   - for each waypoint: if `scan_target_found_ || scan_cancel_` break;
     `prepositioner_->move_to(...)`; on failure break; sleep `scan_dwell_ms`;
   - if found: `finish_scan_found()`; else `finish_scan_timeout()`.
4. `finish_scan_found`: start/unpause Servo + verify (reuse the recovery
   helpers), `sm_.handle(SCAN_FOUND)` then `UNPAUSE_OK`; on failure HOLD.
5. `finish_scan_timeout`: `scan_attempts_++`; `sm_.handle(SCAN_TIMEOUT)`;
   after `scan_max_attempts` set `scan_exhausted_`.
6. `on_track_array`: if `state == SCANNING` and the selector finds an active
   target with `geometry_valid` → `scan_target_found_ = true`.
7. `release` sets `scan_cancel_ = true`; join the thread in teardown.
8. **Auto-resume**: in the HOLD branch, if `auto_resume_on_target_` and
   `target_ok` and `hold_reason_ != kOperatorPause`, call the same resume
   sequence as `/lgw/servo/resume` (re-seed + unpause + verify), not a bare
   `sm_.handle(RESUME)`.
9. `base_is_static_locked()`: compare `odom ← base_link` over the last 200 ms;
   static when translation change < 0.02 m and rotation change < 0.02 rad.

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   colcon test --packages-select lgw_bt_planner && colcon test-result --verbose"
```

Expected: 0 failures; the scan gate tests pass. The full worker is verified by
the Task 13 mock run.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner
git commit -m "feat(servo): MoveGroup scan worker with safety gates

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 12: Servo config, launch, parameter contract

**Files:**
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/servo/servo_params.hpp`
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`
- Modify: `src/lgw_bt_planner/launch/lgw_full.launch.py`
- Modify: `src/lgw_bt_planner/tests/test_servo_params.cpp`
- Modify: `src/lgw_bt_planner/tests/test_param_contract.cpp`
- Modify: `src/lgw_bt_planner/tests/test_servo_node_header_includes.cpp`

**Interfaces:**
- Removes `head_max_age_ms`, `arm_max_age_ms`, `odom_fallback_max_age_ms`,
  `upgrade_hysteresis_ticks`, `head_scale`, `arm_scale`, `odom_scale`,
  `reacquire_on_loss`, `reacquire_radius_m`.
- Adds `auto_scan`, `scan_start_delay_ms`, `scan_timeout_s`,
  `scan_max_attempts`, `scan_retry_cooldown_s`, `auto_resume_on_target`,
  `scan_require_base_static`, the scan joint params, and
  `consumed_suppress_*` (tracker side).

- [ ] **Step 1: Update the parameter contract test**

In `test_param_contract.cpp`, assert the new names exist and the removed names
do not. Add a YAML parse check that `servo_node.yaml` contains
`scan_j5_min_rad`, `scan_j5_max_rad_pos`, `scan_require_base_static`.

- [ ] **Step 2: Run to verify it fails**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   ./build/lgw_bt_planner/test_param_contract"
```

Expected: missing scan params.

- [ ] **Step 3: Implement**

- Remove the geom/scale/reacquire fields from `ServoParams` and YAML.
- Add the scan fields with the defaults in spec §10.7.
- Remove the geom launch arguments/transmissions from `lgw_full.launch.py`
  (keep the servo node parameter YAML).
- Set `arm_scale` equivalent: no scale; the arm source is used at 1.0.

- [ ] **Step 4: Run**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner && \
   colcon test --packages-select lgw_bt_planner && colcon test-result --verbose"
```

Expected: 0 failures.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner
git commit -m "chore(servo): scan config, remove tier/reacquire parameters

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 13: Integration — fake tracker, mock end-to-end, docs

**Files:**
- Modify: `scripts/fake_track_publisher.py`
- Create: `scripts/test_target_registry_e2e.py`
- Modify: `docs/2026-11-13-detection-to-tracking-dataflow.md`
- Test: run the mock script and the full test suites.

**Interfaces:**
- Consumes: `UnifiedTrackArray`, `MarkTrack`.
- Produces: a mock run that proves servo selects the nearest active target,
  HOLDs when `geometry_valid=false`, and enters SCANNING when the list is empty.

- [ ] **Step 1: Update the fake publisher**

Publish at 20 Hz: one `active` track at `position=(1,0,0.5)`,
`position_arm_base=(0.4,0,0.3)`, `geometry_valid=True`, `source="arm"`,
`task_state="active"`. Add `--geometry-valid false` and `--empty` flags for
the integration scenarios.

- [ ] **Step 2: Add the mock end-to-end script**

`scripts/test_target_registry_e2e.py`:
- starts a fake `UnifiedTrackArray` publisher (or imports the fake publisher);
- verifies servo `ServoState` track_id/geometry_source after takeover;
- flips `--empty` and checks `state` reaches `HOLD`/`SCANNING`;
- flips `geometry_valid=false` and checks HOLD.
Use the existing `bench_*` script patterns for process start/stop by PID.

- [ ] **Step 3: Run the suites**

```bash
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && python3 -m pytest src/lgw_3d_tracker/test -q && \
   colcon build --packages-select lgw_bt_planner && \
   colcon test --packages-select lgw_bt_planner && colcon test-result --verbose"
```

Expected: tracker all pass; servo 0 failures.

- [ ] **Step 4: Update the dataflow document**

Update `docs/2026-11-13-detection-to-tracking-dataflow.md`:
- detection → tracker remains;
- tracker now publishes `task_state`/`position_arm_base`/`geometry_valid`;
- servo no longer selects camera tiers;
- add a "Servo target selection" and "SCANNING" section;
- add the `MarkTrack` service.

- [ ] **Step 5: Commit**

```bash
git add scripts/fake_track_publisher.py scripts/test_target_registry_e2e.py \
  docs/2026-11-13-detection-to-tracking-dataflow.md
git commit -m "test: mock target-registry/servo scan end-to-end; update dataflow doc

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:** every §8 interface (Task 1), §9 tracker state/registry/
source/projection/publish/service/config (Tasks 2-5), §10 servo selector/
tick/state machine/scan/config (Tasks 7-12), §11 task-layer interface (Task 13
documentation), §13 tests (each task), §17 builder resolutions (Tasks 3, 5, 9,
11, 12). The calibration items (`scan_j2/j3_raise_rad`, `arm_mount_offset`)
are explicit placeholders in §10.7 and Task 12; they are not faked.

**Type consistency:** `TrackState`/`task_state` is used consistently; the
tracker's `Track.state` remains the KF state while `task_state` is the
lifecycle. `TrackSnapshot` is the only servo-side target type after Task 8.
`MarkTrack.action` values `1/2/3` are used in both Task 3 and Task 1.

**Review Focus tests:** Task 3 covers re-ID aliasing and capacity; Task 5/6
cover per-observation updates and publish; Task 9/11 cover scan gating; Task
7/8 cover `geometry_valid`/frame mismatch; Task 10/11 cover scan-j5 and
cancel/timeout.
