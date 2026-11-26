# elevation_traversability_bridge

`grid_map_msgs/GridMap` (post-`elevation_mapping`) -> `nav_msgs/OccupancyGrid`
(Nav2 `costmap_2d`-aligned) bridge.

## What it does

Subscribes `/elevation_map_raw_post` and republishes
`/terrain_traversability`. Per cell, slope (radians) and roughness (meters)
layers are classified against soft / lethal cutoffs and combined as the max
cost dimension. The result is upsampled (default 2x for 0.10 m GridMap ->
0.05 m costmap) and emitted as a Nav2-compatible OccupancyGrid.

### Why `/elevation_map_raw_post` and not `/elevation_map`

`elevation_mapping` publishes **two** GridMaps:

| Topic | Layers | Contains terrain analysis? |
|-------|--------|---------------------------|
| `/elevation_map` | `elevation`, `upper_bound`, `lower_bound`, `color`, `uncertainty_range` | **no** |
| `/elevation_map_raw_post` | raw layers **+** `elevation_inpainted`, `elevation_smooth`, `normal_vectors_x/y/z`, `slope`, `roughness` | **yes** |

The fused map (`/elevation_map`) is a spatial fusion of the raw map and
carries only the elevation plus its confidence bounds; the slope / roughness
layers are produced by the postprocessing filter chain and published on the
raw+post topic. Subscribing the fused map makes every frame fail with
`missing required layer 'slope'` and nothing is ever published downstream.


Cost values used (Nav2 Costmap2D-internal convention):

| Value | Meaning |
|-------|---------|
| 0     | FREE    |
| 85    | UNKNOWN (Nav2 treats as inflated obstacle) |
| 128   | SOFT (warning band, inflation will clip) |
| 254   | LETHAL (impassable) |

NaN in either slope or roughness -> 85 (UNKNOWN). This happens at cells
that elevation_mapping has never observed.

### Never-observed cells (validity mask)

The postprocessing chain's `InpaintFilter` (`cv::inpaint`) fills **every**
hole of the map, so in areas the robot never drove the derived `slope` /
`roughness` layers are finite and look like perfectly flat, free ground.
The only layer that still carries the NaN holes is the raw `elevation`
layer, so the bridge uses it as a **validity mask**: a cell with NaN
elevation is reported as 85 (UNKNOWN) even though its slope/roughness say
"flat".

Without that mask the whole 50 m x 50 m map is classified as traversable,
including the ~99% that was never scanned. If the source GridMap has no
`elevation` layer the bridge falls back to the old rule (NaN in slope /
roughness -> unknown).

## Frame lock-step

The bridge refuses to convert any GridMap whose `header.frame_id` does not
match `required_frame_id` (default `"map"`). On mismatch it logs FATAL and
exits with code 2. Silently emitting an OccupancyGrid with a wrong
frame_id would let a misconfigured TF tree corrupt the Nav2 costmap.

If you run elevation_mapping with a non-default frame (e.g. `odom`), pass
it on launch:

```bash
ros2 launch elevation_traversability_bridge bridge.launch.py \
    terrain_traversability_bridge.required_frame_id:=odom
```

## Lazy publish on processing failure

If `process_gridmap` raises (missing required layer, shape mismatch,
non-integer upsample factor, ...), the bridge logs ERROR and **does not**
publish an empty OccupancyGrid. Nav2 keeps using its last-known costmap,
which is safer than a one-frame blank that would briefly treat all cells
as UNKNOWN / LETHAL.

## Launch

```bash
ros2 launch elevation_traversability_bridge bridge.launch.py
```

Edit `config/bridge_params.yaml` (or pass overrides on the command line) to
retune thresholds without rebuilding.

## Parameters

All under `terrain_traversability_bridge.ros__parameters`:

| Key | Type | `bridge_params.yaml` | Meaning |
|-----|------|---------|---------|
| `input_topic`         | string  | `/elevation_map_raw_post`  | Source GridMap topic (post-processed map) |
| `output_topic`        | string  | `/terrain_traversability`  | Published OccupancyGrid topic |
| `required_frame_id`   | string  | `map`                      | Frame_id the source MUST declare |
| `target_resolution`   | float   | `0.10`                     | Output cell pitch (m) |
| `slope_soft_upper_deg`     | float | `60.0`  | Slope <= this is FREE (degrees) |
| `slope_lethal_upper_deg`   | float | `89.0`  | Slope >  this is LETHAL (effectively off) |
| `roughness_soft_upper_m`   | float | `0.12`  | Roughness <= this is FREE (m) |
| `roughness_lethal_upper_m` | float | `0.35`  | Roughness >  this is LETHAL (m) |
| `height_soft_m`            | float | `0.15`  | Height above local ground: SOFT band starts (m); 0 disables |
| `height_lethal_m`          | float | `0.20`  | Height above local ground: LETHAL (m); 0 disables |
| `height_window_cells`      | int   | `1`     | Window for local ground: 1 = 3x3 = 0.3 m |
| `height_min_neighbors`     | int   | `2`     | Raised cells required in the 3x3 neighbourhood (incl. self) |
| `ramp_release_max_slope_deg` | float | `60.0` | A cell on a ramp this shallow cannot be made LETHAL by roughness/height; 0 disables |
| `clearance_threshold_m`    | float | `0.0`   | Overhead clearance lethal below this (m); 0 disables |

Between soft and lethal is SOFT (128).

The flat-terrain profile (2026-11-19) makes **height above local ground the
primary obstacle test**: slopes are only a near-vertical hint because the
cave entrance has a steep but drivable ramp, and roughness only adds a cost
bump because the surface is rough but drivable. 15 cm is the largest step
the Go2 crosses cleanly; 20 cm is forced only with foot slip that corrupts
mapping, hence 0.15 / 0.20 m.

`ramp_release_max_slope_deg` (2026-11-19 field run) additionally guarantees
that a cell whose slope layer and one-cell height step both read <= 60 deg
cannot be made LETHAL by the roughness/height tests -- it becomes SOFT at
worst. The windowed-minimum height test is orientation dependent (a 55-60 deg
ramp spans 0.22 m diagonally but only 0.11 m along an axis), so the same
entrance ramp read 254 or 0 depending on its heading; on the real cave the
whole down-slope read 254 and the frontier explorer could not find a single
goal with 0.4 m clearance. Walls/steps read near-vertical and stay lethal.

The 30 s frame report also prints the per-rule lethal counts
(`lethal slope=... rough=... height=... -> after_release=... (ramp_released=...)`), so a
future 254 band can be attributed to the rule that produced it without an
offline replay.

## QoS

- **Subscribe** `/elevation_map_raw_post`: BEST_EFFORT, KEEP_LAST depth=1
  (a BEST_EFFORT subscriber is served by elevation_mapping's RELIABLE
  publisher).
- **Publish** `/terrain_traversability`: RELIABLE, KEEP_LAST depth=1,
  TRANSIENT_LOCAL. Late-joining consumers (RViz, Nav2 startup race)
  still receive the most recent costmap.

Note: the bridge is also the **only** subscriber of `/elevation_map_raw_post`
- elevation_mapping skips the whole postprocessing chain when nobody listens
to that topic, so without the bridge no slope/roughness is computed at all.

## Wire-format note (Nav2 costmap_2d)

`nav_msgs/OccupancyGrid.data` is declared `int8[]` in the ROS IDL
(range -128..127), but `costmap_2d` and the on-wire CDR format treat it
as `uint8` (range 0..255). Values 128 / 254 overflow the Python binding's
strict setter. `bridge_processor.process_gridmap` writes to the private
`_data` slot directly so the wire-level bytes survive intact. Read
verification with `np.array(out.data, dtype=np.uint8)` to avoid the
signed-overflow trap.

The consumer must therefore read the bytes as **unsigned**:
`nav2_traversability_plugin::TraversabilityLayer` does exactly that
(`static_cast<uint8_t>(data[index])`) and maps them onto the Nav2
Costmap2D constants:

| Raw byte | Nav2 costmap_2d cost | Meaning |
|----------|----------------------|---------|
| 0        | `FREE_SPACE` (0)     | traversable |
| 85       | *(cell left untouched)* | UNKNOWN - no terrain data, other layers decide |
| 128      | 128 (soft)           | passable but penalized |
| 254      | `LETHAL_OBSTACLE` (254) | impassable |

Reading the bytes as signed `int8` (the naive `int8_t value = data[i]`)
turns 254 into -2 and 128 into -128, so every lethal / soft cell silently
degrades to FREE, while 85 (UNKNOWN) becomes >= 65 and turns into a lethal
obstacle - exactly inverted.

## Layout

```
elevation_traversability_bridge/
├── elevation_traversability_bridge/
│   ├── __init__.py
│   ├── bridge_classifier.py   # pure: ThresholdConfig + classify_cell
│   ├── bridge_config.py       # topic / geometry defaults (single source of truth)
│   ├── bridge_processor.py    # pure: process_gridmap + UpsampleFactor
│   └── bridge_node.py         # rclpy: subscription + publish shell
├── test/
│   ├── test_bridge_classifier.py
│   ├── test_bridge_config.py
│   └── test_bridge_processor.py
├── config/
│   └── bridge_params.yaml
├── launch/
│   └── bridge.launch.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Tests

```bash
cd /home/unitree/my_ws/l4t/nav_ws/elevation_traversability_bridge
docker exec -w /my_ws/l4t/nav_ws/elevation_traversability_bridge self_l4t \
    bash -lc "source /opt/ros/humble/setup.bash && \
              source /my_ws/l4t/nav_ws/elevmap_fastlio/install/setup.bash && \
              python3 -m pytest test/ -v"
```

Expect: 20/20 tests PASS (8 classifier + 3 config + 9 processor). The
bridge node itself is exercised only by launch (no extra pytest cases —
its behavior is I/O wrapper logic and is verified in the launch test).

`test_bridge_config.py` pins the input topic and asserts that the packaged
yaml and the node defaults cannot drift apart — the drift between
`/elevation_map` and `/elevation_map_raw_post` is exactly what broke the
terrain chain.