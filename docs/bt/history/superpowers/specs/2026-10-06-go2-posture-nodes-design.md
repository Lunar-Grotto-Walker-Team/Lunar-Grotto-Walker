# Go2 Posture BT Nodes — Design Spec

**Date:** 2026-10-06
**Package:** `lgw_bt_planner` (in `l4t/lg_walker_ws`)
**Status:** Draft — awaiting user review
**Author:** claude-code (per brainstorming skill)

## 1. Problem & motivation

`lgw_bt_planner` currently has 6 BT.ROS2 plugin-registered nodes driving the
Piper arm + MoveIt + perception. **It has no nodes that drive the Go2 base** —
without StandUp / BalanceStand / StandDown, the dog cannot be commanded from
the BT tree at all. This is the minimal set needed to (a) close the loop on
real-hardware deployment and (b) serve as the bracketing nodes of any future
pick-and-place mission that uses the mobile base.

A 2026-09-24 implementation attempt left a detailed `work.log` describing the
exact same three nodes, but the source files were never persisted (they were
either not committed or were swept in a later cleanup). The design below is
deliberately close to the work.log record so that prior reasoning and
discoveries are not lost.

## 2. Scope

### In scope (this spec)

- Three BT leaf nodes driving the Go2 base via `/api/sport/request` topic:
  - `StandUp` — go from lying → standing.
  - `BalanceStand` — switch into balance mode (the posture Move requires).
  - `StandDown` — go from standing → lying.
- A BT XML demo tree exercising the round-trip sequence
  (`StandUp → BalanceStand → 2 s settle → StandDown`).
- A top-level ROS2 launch file that brings up `bt_runner` with the demo tree
  discoverable.
- Container build verification + node symbol export verification.

### Out of scope (deferred — matches work.log "不在范围" list)

- `Move` (cmd_vel velocity) BT node and the accompanying `twist_bridge`.
- `Damp` (1001) and `RecoveryStand` (1006) BT nodes.
- A `mock_go2_driver` for lgw — verification is on real hardware.
- Sharing implementation with `go2_bt_ws` — the lgw package remains
  self-contained per its README convention.
- gtest unit tests — work.log did not add any, and the verifier is real
  hardware. (gtest is left as a follow-up in §10.)
- Wiring the demo tree into `pick_and_place_demo.xml` as bracketing nodes.

## 3. Architecture

Three independent BT leaf nodes, each directly inheriting
`BT::StatefulActionNode`. No shared intermediate base class. Constants and
topic-name strings live in a single header file that all three `.cpp` files
`#include`.

```
   BT tick ───►  StandUpNode        ──► /api/sport/request (api_id=1004)
              ──►  BalanceStandNode  ──► /api/sport/request (api_id=1002)
              ──►  StandDownNode     ──► /api/sport/request (api_id=1005)
                              ▲
                              │
                 lf/sportmodestate (subscribed by all three)
```

### Why no shared base class

- The genuinely duplicated code (publisher/subscriber construction,
  `std::mutex`, `latest_state_`, empty `providedPorts()`) is ~30 lines per
  node — not enough to justify an inheritance layer.
- Every existing leaf node in `lgw_bt_planner` inherits directly from a
  BT.ROS2 / BT.CPP base class. Introducing an `SportTopicNodeBase` would be
  the only package-local intermediate class, breaking the convention.
- The future `Move` node (deferred) has a substantially different shape
  (continuous republish of `/cmd_vel`, no client-side threshold,
  duration-based success) and **should not** share a base with
  StandUp / BalanceStand / StandDown. So no plausible future reuse is
  blocked by staying flat.
- The two genuinely shared things — constants and topic-name strings — are
  factored into a header of `inline constexpr` values, which the three
  `.cpp` files `#include`. No inheritance involved.

## 4. Components

### 4.1 `sport_api_constants.hpp` (shared header)

Lives in `include/lgw_bt_planner/sport_api_constants.hpp`. All values are
`inline constexpr` — single source of truth, no duplication across the three
node files:

```cpp
namespace lgw_bt_planner::sport_api {

// Topic names — mirrors go2_bt_ws conventions (note: state topic has no
// leading "/").
inline constexpr const char* kSportRequestTopic   = "/api/sport/request";
inline constexpr const char* kSportModeStateTopic = "lf/sportmodestate";

// Sport API IDs (see
// unitree_ros2/example/src/include/common/ros2_sport_client.h).
inline constexpr int32_t kApiIdStandUp      = 1004;
inline constexpr int32_t kApiIdStandDown    = 1005;
inline constexpr int32_t kApiIdBalanceStand = 1002;
inline constexpr int32_t kApiIdStopMove     = 1003;

// Completion thresholds (from go2_bt_ws/CLAUDE.md node table — verified).
inline constexpr float  kStandHeight    = 0.30f;
inline constexpr float  kLyingHeight    = 0.20f;
inline constexpr int    kStableFrames   = 5;
inline constexpr int8_t kBalanceMode    = 1;

// Per-node timeouts (seconds).
inline constexpr double kStandUpTimeoutSec      = 5.0;
inline constexpr double kStandDownTimeoutSec    = 5.0;
inline constexpr double kBalanceStandTimeoutSec = 2.0;

}  // namespace lgw_bt_planner::sport_api
```

### 4.2 Three independent nodes

| File                                   | Class                  | api_id | Threshold                            | Timeout |
|----------------------------------------|------------------------|--------|--------------------------------------|---------|
| `stand_up_node.hpp` / `.cpp`           | `StandUpNode`          | 1004   | `body_height ≥ 0.30` for 5 frames    | 5.0 s   |
| `balance_stand_node.hpp` / `.cpp`      | `BalanceStandNode`     | 1002   | `state.mode == 1`                    | 2.0 s   |
| `stand_down_node.hpp` / `.cpp`         | `StandDownNode`        | 1005   | `body_height < 0.20` for 5 frames    | 5.0 s   |

Each class:
- inherits `BT::StatefulActionNode` directly,
- owns its own `publisher_`, `subscriber_`, `state_mutex_`,
  `latest_state_`, `stable_count_`, `deadline_`,
- overrides `onStart`, `onRunning`, `onHalted`, and provides a static
  `providedPorts() { return {}; }` (BT.CPP v4 `static_assert` requires
  it even when empty — this is what bit the first build attempt in
  work.log).

Rationale for threshold values (from `go2_bt_ws/CLAUDE.md` node table and
work.log §探活):

- 0.30 m and 0.20 m are the established success thresholds in `go2_bt_ws`;
  re-deriving them here would diverge from the only verified numbers.
- 5 frames of stability rejects one-frame IMU glitches without adding
  perceptible latency (5 × 50 ms = 250 ms at the typical state rate).
- 2 s for BalanceStand is generous: state.mode usually flips within one
  cycle of `/api/sport/request` processing.

### 4.3 BT XML demo tree

`bt_trees/go2_posture_demo.xml`:

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="Go2PostureDemo" BTCPP_format="4">
  <BehaviorTree ID="Go2PostureDemo">
    <Sequence name="posture_round_trip">
      <RetryUntilSuccessful num_attempts="3">
        <StandUp name="stand_up"/>
      </RetryUntilSuccessful>
      <BalanceStand name="balance_stand"/>
      <Sleep name="settle" msec="2000"/>
      <RetryUntilSuccessful num_attempts="3">
        <StandDown name="stand_down"/>
      </RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>
</root>
```

Why `RetryUntilSuccessful`: StandUp/StandDown may fail transiently if the
machine is mid-motion when the BT tick fires. Three retries are a cheap
robustness bump without hiding real faults. StandUp success here means
**all three** nodes succeeded end-to-end.

### 4.4 Launch file

`launch/go2_posture_demo.launch.py` — mirrors the style of
`lgw_bt_planner.launch.py` and `bt_with_lgw_moveit.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lgw_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            parameters=[{
                'plugins': ['lgw_bt_planner/bt_plugins'],
                'behavior_trees': ['lgw_bt_planner/bt_trees'],
                'action_name': 'lgw_bt_execution',
                'tick_frequency': 10,
            }],
        ),
    ])
```

This is functionally identical to `lgw_bt_planner.launch.py`. We keep it as
its own file because:

- The package convention is one launch file per demo tree
  (`bt_with_lgw_moveit.launch.py` already follows this for pick-and-place).
- Users running only the posture demo don't have to read the moveit
  launch chain.

### 4.5 Registration in `bt_plugin_registration.cpp`

Add one block inside `BT_REGISTER_ROS_NODES`:

```cpp
factory.registerNodeType<lgw_bt_planner::StandUpNode>(
  "StandUp", BT::RosNodeParams(nh_shared));
factory.registerNodeType<lgw_bt_planner::BalanceStandNode>(
  "BalanceStand", BT::RosNodeParams(nh_shared));
factory.registerNodeType<lgw_bt_planner::StandDownNode>(
  "StandDown", BT::RosNodeParams(nh_shared));
```

No `default_port_value` because these nodes don't take an `action_name` or
`topic_name` port (publish / subscribe targets are fixed by
`sport_api_constants.hpp`).

## 5. Data flow

### 5.1 Outbound: BT tick → real robot

```
BT tick (BT executor thread)
  → onStart() [first tick only]
      build unitree_api::msg::Request{ api_id = <class-constant> }
      publisher_->publish(request)            // one-shot
      stable_count_ = 0
      deadline_ = now() + <class-timeout>
      return RUNNING

  → onRunning() [subsequent ticks]
      lock state_mutex_; read latest_state_; unlock
      if (threshold_just_met_for_kStableFrames):
        return SUCCESS
      else if (now() > deadline_):
        return FAILURE
      else:
        return RUNNING
```

The single publish in `onStart` is correct: Go2 sport mode commands are
fire-and-forget. The state subscription carries the outcome back.

### 5.2 Inbound: real robot → BT tick

```
sportmodestate publisher (DDS thread)
  → subscriber callback
      lock state_mutex_; latest_state_ = *msg; unlock
      (no wakeup — BT tick polls state on its own schedule)
```

The subscription callback does **not** call `emitWakeUpSignal()`.
BT.CPP will poll on each tick anyway (10 Hz `tick_frequency`), and emitting
wake-ups from a DDS callback risks priority inversion if the BT thread is
busy holding the mutex.

### 5.3 Halting

```
parent node calls halt() (e.g., on FAILURE upstream)
  → onHalted()
      if class is StandUp or StandDown:
        publish unitree_api::msg::Request{ api_id = 1003 }  // StopMove
      else if class is BalanceStand:
        // no-op: BalanceStand has no motion to abort, and forcing
        // StopMove here would knock mode back to 0.
      (no state mutation needed — next tick's onStart resets)
```

This matches work.log's exact onHalted design.

## 6. QoS, threading, mutex model

- **Publisher** QoS on `/api/sport/request`: **RELIABLE, KEEP_LAST(1)**.
  - Top-level `CLAUDE.md` rule "❌ `/api/sport/request` 的 QoS 改为非
    RELIABLE" applies here. RELIABLE is also the safe-by-default choice:
    it is downward-compatible with both RELIABLE and BEST_EFFORT
    subscribers, whereas BEST_EFFORT would be rejected by any
    RELIABLE subscriber on the robot.
  - The fire-and-forget nature of these commands means the publisher
    has nothing to queue: a single publish per node tick, deadline in
    `onRunning`, no replays. KEEP_LAST(1) is the right depth.
- **Subscriber** QoS on `lf/sportmodestate`: **BEST_EFFORT, KEEP_LAST(5)**
  via `rclcpp::SensorDataQoS()`.
  - The state publisher on the real robot is RELIABLE (work.log §探活);
    a BEST_EFFORT subscriber is downward-compatible with it.
  - The state topic is high-frequency (20 Hz) and lossy-impervious
    (every frame carries essentially the same body_height / mode), so
    BEST_EFFORT is the appropriate fit. The top-level `CLAUDE.md`
    rule targets the *command* topic, not state.
- **Threading model**: BT ticks run on the BT thread (the
  `TreeExecutionServer` worker). Subscriber callbacks run on whatever
  executor the `bt_runner` node is registered with (the
  `MultiThreadedExecutor` in `bt_runner_node.cpp` with 0 worker threads +
  250 ms `spin_some` — known deadlock workaround documented there).
- **Mutex**: `std::mutex state_mutex_` guards `latest_state_`. The publish
  path holds no lock because publishing only touches `publisher_`, which
  is thread-safe inside rclcpp.

## 7. Error handling

| Failure mode                              | Detection                                     | Result         |
|-------------------------------------------|-----------------------------------------------|----------------|
| Real hardware unreachable (`/api/sport/request` no listener) | `publisher_->publish` returns no error at rclcpp layer; we can't observe directly. | Silent — leaves the dog in prior state. Acceptable for fire-and-forget sport commands. |
| `lf/sportmodestate` silent for 5 s (cable unplug / process dead) | `onRunning` deadline check                    | FAILURE        |
| `body_height` stuck between thresholds (mechanical fault)     | 5-frame stability check never reaches `kStableFrames` within 5 s | FAILURE (after timeout) |
| BT node halted mid-motion (parent failure)                    | BT.CPP `halt()`                               | `onHalted` fires; StandUp/StandDown publish StopMove (1003) |
| `RetryUntilSuccessful` exhausted (3 strikes)                  | RetryUntilSuccessful internal counter         | FAILURE to outer Sequence |

No new error handling surfaces beyond what BT.CPP already provides; the
nodes return SUCCESS / FAILURE / RUNNING per `StatefulActionNode` contract.

## 8. Testing plan

### 8.1 Container-side verification (this PR)

- `colcon build --packages-select lgw_bt_planner` succeeds.
- `nm -D install/lgw_bt_planner/lib/liblgw_bt_planner_nodes.so | grep
  BT_RegisterRosNodeFromPlugin` finds the plugin symbol.
- `nm -D` finds `StandUpNode::onStart`, `…onRunning`, `…onHalted`,
  `…providedPorts`, plus the same for `BalanceStandNode` and
  `StandDownNode`.
- `ros2 launch lgw_bt_planner go2_posture_demo.launch.py` reports:
  - `[INFO] Loaded ROS Plugin: liblgw_bt_planner_nodes.so`
  - `[INFO] Loaded BehaviorTree: go2_posture_demo.xml`
  - No new errors vs. the existing 6 BT trees remaining loadable.
- No new build warnings involving the new files.

### 8.2 Hardware-side verification (user-driven, out of PR scope)

These match work.log's "留给用户/下一轮" list verbatim:

- Round-trip on real hardware: dog stands up, holds balance 2 s, lies down.
- Validate `body_height` thresholds ≥0.30 / <0.20 actually fire SUCCESS on
  the real dog (vs. mock).
- Validate `state.mode == 1` actually flips after BalanceStand.
- Force-fail by yanking eth10 or wedging body_height; verify 5 s timeout
  returns FAILURE without hanging.
- Force `onHalted` (e.g., via ReactiveSequence abort); verify StopMove
  (1003) appears on `/api/sport/request`.
- ≥20 round-trip cycles; check for leaks / crashes.

## 9. Files touched

### New

| Path                                                                       | Purpose                          |
|----------------------------------------------------------------------------|----------------------------------|
| `src/lgw_bt_planner/include/lgw_bt_planner/sport_api_constants.hpp`        | Shared constants + topic names   |
| `src/lgw_bt_planner/include/lgw_bt_planner/stand_up_node.hpp`              | StandUpNode declaration          |
| `src/lgw_bt_planner/include/lgw_bt_planner/balance_stand_node.hpp`         | BalanceStandNode declaration     |
| `src/lgw_bt_planner/include/lgw_bt_planner/stand_down_node.hpp`            | StandDownNode declaration        |
| `src/lgw_bt_planner/src/stand_up_node.cpp`                                 | StandUpNode implementation       |
| `src/lgw_bt_planner/src/balance_stand_node.cpp`                            | BalanceStandNode implementation  |
| `src/lgw_bt_planner/src/stand_down_node.cpp`                               | StandDownNode implementation     |
| `src/lgw_bt_planner/bt_trees/go2_posture_demo.xml`                         | Demo BT tree                     |
| `src/lgw_bt_planner/launch/go2_posture_demo.launch.py`                     | Launch wrapper for the demo tree |
| `src/lgw_bt_planner/docs/superpowers/specs/2026-10-06-go2-posture-nodes-design.md` | This spec                |

### Modified

| Path                                                | Change                                                                                       |
|-----------------------------------------------------|----------------------------------------------------------------------------------------------|
| `src/lgw_bt_planner/src/bt_plugin_registration.cpp` | Add 3 `#include`s + 3 `factory.registerNodeType<...>(...)` lines.                            |
| `src/lgw_bt_planner/CMakeLists.txt`                 | Add `find_package(unitree_api)` / `find_package(unitree_go)`; append the 3 new `.cpp`s to `NODE_SOURCES`; extend `ament_target_dependencies` and `ament_export_dependencies`. |
| `src/lgw_bt_planner/package.xml`                    | Add `<depend>unitree_api</depend>` and `<depend>unitree_go</depend>`.                        |

### Untouched (verified, no edit needed)

- `src/lgw_bt_planner/launch/lgw_bt_planner.launch.py` — still works; the
  new demo is auto-discovered by the `behavior_trees` glob.
- `src/lgw_bt_planner/launch/bt_with_lgw_moveit.launch.py` — unchanged.
- `src/lgw_bt_planner/src/bt_runner_node.cpp` — unchanged.

## 10. Follow-ups (deferred — work.log parity)

These match the "不在范围" list in work.log:

- `Move` BT node + `/cmd_vel → /api/sport/request` twist_bridge (mirror
  `go2_bt_ws/src/go2_twist_bridge/src/twist_bridge.cpp`).
- `Damp` (1001) and `RecoveryStand` (1006) BT nodes.
- `mock_go2_driver` for lgw — for in-container dry-run.
- gtest unit tests (the `go2_bt_ws/test_*` pattern).
- Wiring `StandUp → BalanceStand → … → StandDown` as bracketing nodes of
  `pick_and_place_demo.xml`.
- Sharing the implementation with `go2_bt_ws` (a `go2_bt_common` package or
  a sub-fork). Currently lgw package is self-contained per its README.

## 11. Spec self-review

- **Placeholders:** none. All thresholds, timeouts, file paths, QoS choices
  are concrete.
- **Internal consistency:** §4 (components), §5 (data flow), §6 (QoS /
  threading), §7 (error handling), §9 (files) cross-check. The QoS choice
  in §6 is consistent with the top-level `CLAUDE.md` rule (RELIABLE
  publisher on `/api/sport/request`).
- **Scope:** three BT nodes + demo XML + launch file + plugin wiring —
  one PR-sized change. No decomposition needed.
- **Ambiguity:** "fire-and-forget publisher failure is silent" is explicit
  in §7. The "RetryUntilSuccessful" choice is explicit in §4.3. The QoS
  choice is explicit in §6. The "no shared base class" decision is explicit
  in §3.

## 12. Resolved decisions

These were Open Questions in the first draft; resolved in this revision:

1. **QoS** — RELIABLE publisher (default rclcpp publisher QoS =
   `rclcpp::QoS(1)` or `rclcpp::ServicesQoS()` style with `KEEP_LAST(1)`),
   BEST_EFFORT subscriber (`rclcpp::SensorDataQoS()`). Reasoning in §6.
2. **Shell wrapper** — dropped. Only `launch/go2_posture_demo.launch.py`
   ships; users invoke via `ros2 launch`. No new convention established.
3. **Behavior tree ID** — `Go2PostureDemo` confirmed. Matches the
   `PascalCase` style of `PickAndPlaceDemo` / `HeadToArmLookat`.

This spec is now ready for the writing-plans skill.
