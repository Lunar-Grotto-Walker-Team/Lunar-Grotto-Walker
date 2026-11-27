"""Pure conversion: grid_map_msgs/GridMap -> nav_msgs/OccupancyGrid.

Frame lock-step: caller must guarantee msg.header.frame_id matches the
Nav2 global frame (typically "map") and msg.info.pose is in that frame.
This module does no TF.

Cost semantics (Nav2 conventions):
    0   = FREE (drive-able)
    128 = SOFT (warning band, inflation will clip)
    254 = LETHAL (impassable)
    85  = UNKNOWN (no data here, treat as inflated obstacle)

NaN slope or roughness at a cell => UNKNOWN (85).
"""
from __future__ import annotations

from copy import deepcopy

import array as array_mod

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from grid_map_msgs.msg import GridMap as GridMapMsg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

from .bridge_classifier import ThresholdConfig


REQUIRED_LAYERS = ("slope", "roughness")

#: Layer used as the *validity mask*: a cell whose elevation is NaN was never
#: observed by the lidar.  The postprocessing chain's InpaintFilter (cv::inpaint)
#: fills **every** hole of the map, so the derived slope / roughness layers are
#: finite (and look like flat ground) even in areas the robot never drove.
#: The raw `elevation` layer is the only one that still carries the NaN holes,
#: so it is what tells "scanned" from "never scanned".
#:
#: If the source GridMap has no such layer the bridge falls back to the old
#: rule (NaN in slope / roughness -> unknown).
VALIDITY_LAYER = "elevation"

#: Optional vertical-clearance layer, published only when the projector runs
#: with ``publish_clearance: true``. Value = metres between the cell's local
#: ground and the lowest overhead point above it, NaN where nothing is
#: overhead (open sky -> always passable). Cells below
#: ``cfg.clearance_threshold_m`` become lethal regardless of how flat the
#: surface is: the dog is ~0.8 m tall, a cave arch narrower than 1.0 m is a
#: wall as far as it is concerned.
CLEARANCE_LAYER = "clearance"

#: Optional per-cell point count published by the projector. Cells with fewer
#: points than ``cfg.min_support_for_lethal`` cannot be lethal: their elevation
#: is a 1-2 point median, and a 0.3 m step between two such cells reads as a
#: 70 deg wall on flat ground.
COUNTS_LAYER = "counts"

# Nav2 cost values.
COST_FREE = 0
COST_SOFT = 128
COST_LETHAL = 254
COST_UNKNOWN = 85


@dataclass(frozen=True)
class UpsampleFactor:
    """How many occupancy_grid cells per grid_map cell along each axis.

    For 0.10 m GridMap -> 0.05 m costmap, factor = 2 (each GridMap cell is
    replicated into a 2x2 block of identical costmap cells).

    Computed at runtime by `compute_upsample_factor()` from a sample msg;
    tests can pass a literal to avoid the round-trip.
    """
    n: int = 2

    def apply(self, costs: np.ndarray) -> np.ndarray:
        """Nearest-neighbor upsample: replicate each cell into an n x n block.

        Output shape is (h*n, w*n). With n=2, a 10x10 input becomes a
        20x20 output where each input cell is duplicated into a 2x2
        quadrant. We use np.kron with an n x n ones matrix, which is
        equivalent to "max over n x n sub-block of duplicates" — so a
        single lethal input cell dominates its 2x2 output block, matching
        Nav2's expectation that lethal cells never get averaged into
        non-lethal territory.
        """
        if self.n == 1:
            return costs
        return np.kron(costs, np.ones((self.n, self.n), dtype=costs.dtype))


def compute_upsample_factor(msg: GridMapMsg, target_resolution: float) -> UpsampleFactor:
    """n = source_resolution / target_resolution, rounded to nearest int.

    Raises ValueError on non-integer ratio or target coarser than source.
    """
    n_float = msg.info.resolution / target_resolution
    n = int(round(n_float))
    if n < 1:
        raise ValueError(
            f"target_resolution {target_resolution} must be <= source "
            f"{msg.info.resolution}"
        )
    if not math.isclose(n_float, n, rel_tol=1e-3):
        raise ValueError(
            f"non-integer upsample factor {n_float} between source "
            f"{msg.info.resolution} and target {target_resolution}"
        )
    return UpsampleFactor(n=n)


def _extract_layer_as_array(msg: GridMapMsg, layer: str) -> np.ndarray:
    """Return the named layer as a 2D float32 array of shape (rows, cols).

    Layout facts, verified against the real grid_map_core (see
    pgo_gridmap_projector/test/gridmap_reference.cpp):

      * grid_map's Eigen matrix is column-major and its buffer index (0, 0)
        is the cell at (max x, max y), not the lower-left corner;
      * `matrixEigenCopyToMultiArrayMessage` writes dim[0].size = outerSize
        = number of columns (Y axis) and dim[1].size = innerSize = number
        of rows (X axis), and copies the raw column-major buffer.

    So the flat buffer reshaped as (dim[0].size, dim[1].size) is the map
    rotated 180 degrees relative to an OccupancyGrid, whose row 0 / col 0
    is the lower-left (min x, min y). Flip both axes to get back the
    world picture: result[row, col] with row increasing along +y and col
    increasing along +x.
    """
    if layer not in msg.layers:
        raise ValueError(
            f"missing required layer '{layer}' from GridMap "
            f"(have: {msg.layers})"
        )
    idx = msg.layers.index(layer)
    flat = np.asarray(msg.data[idx].data, dtype=np.float32)
    dims = msg.data[idx].layout.dim
    if len(dims) >= 2:
        # dim[0] is the fast-varying one (column / Y); dim[1] is the row / X.
        height, width = dims[0].size, dims[1].size
    elif len(dims) == 1:
        height = int(round(msg.info.length_y / msg.info.resolution))
        width = int(round(msg.info.length_x / msg.info.resolution))
    else:
        height = int(round(msg.info.length_y / msg.info.resolution))
        width = int(round(msg.info.length_x / msg.info.resolution))
    if flat.size != height * width:
        # Fallback: trust geometry only.
        height = int(round(msg.info.length_y / msg.info.resolution))
        width = int(round(msg.info.length_x / msg.info.resolution))
    # 180-degree rotation: grid_map's buffer origin is the top-right cell.
    return flat.reshape((height, width), order="C")[::-1, ::-1]


def _classify_dimension(
    matrix: np.ndarray, soft_upper: float, lethal_upper: float,
    cost_free: int, cost_soft: int, cost_lethal: int,
) -> np.ndarray:
    """Vectorized single-dim classify: <= soft => free, soft<.<= lethal => soft,
    > lethal => lethal. NaN values are left untouched (caller handles NaN
    -> unknown).
    """
    out = np.full(matrix.shape, cost_free, dtype=np.int32)
    mask_soft = (matrix > soft_upper) & ~np.isnan(matrix)
    out[mask_soft] = cost_soft
    mask_lethal = (matrix > lethal_upper) & ~np.isnan(matrix)
    out[mask_lethal] = cost_lethal
    return out


def _local_min_window(z: np.ndarray, half_cells: int) -> np.ndarray:
    """NaN-aware minimum over a (2k+1)^2 window (inf where nothing is valid)."""
    filled = np.where(np.isfinite(z), z, np.inf)
    size = 2 * int(half_cells) + 1
    return ndimage.minimum_filter(filled, size=size, mode="constant",
                                  cval=np.inf)


def _local_min_3x3(z: np.ndarray) -> np.ndarray:
    """NaN-aware 3x3 minimum (NaN treated as +inf).

    ``minimum_filter`` with a constant +inf border is exactly the old
    nine-slice loop, but runs in C instead of building nine temporary
    arrays over the grid.
    """
    fill = np.where(np.isfinite(z), z, np.inf).astype(np.float32,
                                                      copy=False)
    return ndimage.minimum_filter(fill, size=3, mode="constant",
                                  cval=np.inf)


def _suppress_flat_small_clusters(
    costs: np.ndarray,
    elevation: np.ndarray,
    slope_rad: np.ndarray,
    cfg: ThresholdConfig,
) -> np.ndarray:
    """Downgrade flat, isolated lethal cells produced by mean-difference roughness.

    A lethal cluster of at most ``small_cluster_max_cells`` cells survives
    only when it carries height evidence: the cell's own elevation is at
    least ``small_cluster_min_height_m`` above the 3x3 local minimum (a thin
    pole, a rock, a step), or its slope exceeds
    ``small_cluster_min_slope_deg``. Flat cells pulled up by a neighbouring
    vertical structure are reset to free.

    This used to be a Python flood fill over every lethal cell (a stacks +
    9-neighbour loop with scalar np.isfinite calls). On bags with many
    lethal cells -- slopes, walls -- that alone saturated the bridge at
    ~100% CPU and capped the chain at 1.5 Hz, while the corridor bag stayed
    at 45%. ``ndimage.label`` does the same 8-connected labelling in C and
    the per-cell decision is vectorised, so the cost is a couple of numpy
    passes over the grid.
    """
    lethal = costs == cfg.cost_lethal
    if not lethal.any() or cfg.small_cluster_max_cells <= 0:
        return costs

    # Same window as the mound rule: a narrow obstacle should look tall against
    # its neighbourhood, and after the windowed-slope change the "steep" test
    # rarely fires, so the height evidence is what keeps real thin obstacles.
    local_min = _local_min_window(elevation, cfg.height_window_cells)
    height_above_min = elevation.astype(np.float64) - local_min
    slope_deg = np.degrees(slope_rad)

    # 8-connected components: the Python version walked all eight
    # neighbours, so the structure must be the full 3x3 block, not the
    # ndimage default cross.
    labels, count = ndimage.label(lethal, structure=np.ones((3, 3), dtype=int))
    if count == 0:
        return costs

    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    small = sizes <= cfg.small_cluster_max_cells
    small[0] = False                      # label 0 is "not lethal"

    elevated = np.isfinite(height_above_min) & (
        height_above_min >= cfg.small_cluster_min_height_m)
    steep = slope_deg > cfg.small_cluster_min_slope_deg
    # NaN slope compares False, exactly like the scalar `slope_deg[r, c] >
    # threshold` in the old code.
    downgrade = lethal & small[labels] & ~(elevated | steep)
    costs[downgrade] = cfg.cost_free
    return costs


def _apply_trail_clear(costs: np.ndarray, trail_xy: np.ndarray | None,
                       half_width_m: float, resolution: float,
                       origin_x: float, origin_y: float,
                       stats: dict | None = None,
                       robot_xy: tuple[float, float] | None = None) -> np.ndarray:
    """Force the cells the robot has driven through to free.

    The robot's own trajectory is evidence: a cell it occupied cannot be an
    obstacle.  This is aimed at the failure where upstream drift/loop closure
    leaves two copies of a wall in /aft_pgo_map, so the corridor the robot
    just came down is filled in and it gets trapped by its own map.  A live
    sensor cannot undo that (the cells DO have points); the trailed cells are
    the one thing both the map and the robot agree on.

    On the RETURN leg the part that matters is the outbound trail, which is
    *ahead* of the robot -- that is why trail_window_s has to span the whole
    excursion, and why this works by splatting the trail into the output grid
    and dilating, instead of a radius query over every cell (a 30 min trail
    covers the whole window, and a per-cell query would cost ~0.5 s/frame).

    UNKNOWN cells are left alone: "I drove here" says nothing about cells the
    map has no data for.
    """
    if trail_xy is None or len(trail_xy) == 0 or half_width_m <= 0.0:
        return costs
    trail = np.asarray(trail_xy, dtype=np.float64).reshape(-1, 2)
    rows, cols = costs.shape
    ix = np.rint((trail[:, 0] - origin_x) / resolution - 0.5).astype(np.int64)
    iy = np.rint((trail[:, 1] - origin_y) / resolution - 0.5).astype(np.int64)
    inside = (ix >= 0) & (ix < cols) & (iy >= 0) & (iy < rows)
    if not inside.any():
        return costs  # the trail is entirely outside this frame's window
    mask = np.zeros((rows, cols), dtype=bool)
    mask[iy[inside], ix[inside]] = True

    # Disc footprint of half_width_m: the robot's body radius around each
    # trail point is space it actually occupied.
    r_cells = max(1, int(round(half_width_m / resolution)))
    yy, xx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
    footprint = (xx * xx + yy * yy) <= r_cells * r_cells + 0.5
    near = ndimage.binary_dilation(mask, structure=footprint)

    hit = near & (costs != COST_UNKNOWN)
    if stats is not None:
        stats["trail_cells"] = int(np.count_nonzero(hit))
        stats["trail_changed"] = int(np.count_nonzero(hit & (costs != 0)))
        if robot_xy is not None:
            # Cells freed away from the robot are the return-leg case: the
            # outbound trail is what lies ahead.  A non-zero counter proves
            # the prior is working ahead of the robot, not just under it.
            rr, cc = np.nonzero(hit & (costs != 0))
            if rr.size:
                dx = origin_x + (cc + 0.5) * resolution - robot_xy[0]
                dy = origin_y + (rr + 0.5) * resolution - robot_xy[1]
                stats["trail_changed_far"] = int(
                    np.count_nonzero(np.hypot(dx, dy) > 2.0))
            else:
                stats["trail_changed_far"] = 0
    costs[hit] = 0
    return costs


def process_gridmap(
    msg: GridMapMsg,
    cfg: ThresholdConfig | None = None,
    target_resolution: float = 0.05,
    trail_xy: np.ndarray | None = None,
    trail_half_width_m: float = 0.0,
    stats: dict | None = None,
    robot_xy: tuple[float, float] | None = None,
) -> OccupancyGrid:
    """Convert a GridMap to a Nav2 costmap-aligned OccupancyGrid.

    - Reads required layers (slope, roughness). Fails loudly if missing.
    - NaN slope or roughness => cell value 85 (Nav2 unknown).
    - NaN elevation (never observed) => cell value 85 as well, even when the
      inpainted slope / roughness look like flat free ground.
    - Per-cell cost = max(slope_cost, roughness_cost) so the more severe
      dimension dominates (a slightly-steep but smooth slope is still
      "soft", but a tiny slope with 20 cm roughness is lethal).
    - Upsamples by integer factor n (max-per-block) to target_resolution.
    - Frame carried verbatim from msg.header.frame_id.
    """
    if cfg is None:
        cfg = ThresholdConfig()

    # 1. Layer validation.
    for layer in REQUIRED_LAYERS:
        if layer not in msg.layers:
            raise ValueError(
                f"missing required layer '{layer}' "
                f"(required: {list(REQUIRED_LAYERS)}, have: {msg.layers})"
            )

    # 2. Extract layers.
    slope = _extract_layer_as_array(msg, "slope")        # radians
    roughness = _extract_layer_as_array(msg, "roughness")  # meters

    if slope.shape != roughness.shape:
        raise ValueError(
            f"slope.shape={slope.shape} != roughness.shape={roughness.shape}"
        )

    height, width = slope.shape

    # 2b. Validity mask: cells that were never observed (NaN elevation).
    validity = None
    if VALIDITY_LAYER in msg.layers:
        validity = np.isfinite(_extract_layer_as_array(msg, VALIDITY_LAYER))
        if validity.shape != slope.shape:
            raise ValueError(
                f"{VALIDITY_LAYER}.shape={validity.shape} != "
                f"slope.shape={slope.shape}"
            )

    # 3. Per-cell classification, max-of-dimensions.
    slope_cost = _classify_dimension(
        slope,
        math.radians(cfg.slope_soft_upper_deg),
        math.radians(cfg.slope_lethal_upper_deg),
        cfg.cost_free, cfg.cost_soft, cfg.cost_lethal,
    )
    rough_cost = _classify_dimension(
        roughness,
        cfg.roughness_soft_upper_m,
        cfg.roughness_lethal_upper_m,
        cfg.cost_free, cfg.cost_soft, cfg.cost_lethal,
    )
    costs = np.maximum(slope_cost, rough_cost)

    # 3a. Height criterion -- the primary obstacle test for flat terrain.
    # A cell raised above its local ground by `height_soft_m` costs extra;
    # `height_lethal_m` makes it impassable. Because the reference is a
    # *windowed minimum* rather than a slope, a sustained ramp never reaches
    # the threshold (with height_window_cells=1 a 3x3 window spans only
    # 0.141 m on a 45 deg slope), while a real step or rock does.
    #
    # The old 3x3 erosion is replaced by a neighbour count: with a 3x3 window
    # a genuine step edge is one cell wide, so erosion erased it, while a
    # lone noisy cell still has a count of 1 and is dropped. The support
    # floor and the frontier exemption below still remove sparse/edge
    # artefacts.
    if ((cfg.height_soft_m > 0.0 or cfg.height_lethal_m > 0.0)
            and VALIDITY_LAYER in msg.layers):
        elev = _extract_layer_as_array(msg, VALIDITY_LAYER)
        if elev.shape == costs.shape:
            filled = np.where(np.isfinite(elev), elev, np.inf)
            size = 2 * cfg.height_window_cells + 1
            local_min = ndimage.minimum_filter(
                filled, size=size, mode="constant", cval=np.inf)
            dh = elev - local_min
            finite = np.isfinite(dh)

            def _raised(threshold: float) -> np.ndarray:
                mask = finite & (dh >= threshold)
                count = ndimage.uniform_filter(
                    mask.astype(np.float64), size=3, mode="constant",
                    cval=0.0) * 9.0
                return mask & (count >= cfg.height_min_neighbors - 1e-9)

            if cfg.height_soft_m > 0.0:
                soft = _raised(cfg.height_soft_m)
                costs[soft] = np.maximum(costs[soft], cfg.cost_soft)
            if cfg.height_lethal_m > 0.0:
                costs[_raised(cfg.height_lethal_m)] = cfg.cost_lethal

    # 3b. Frontier exemption: the newest cells cannot be judged -- they are
    # where the crest shadows and the sparse far returns live. Leave them to
    # the unobserved mask (UNKNOWN) instead of inventing obstacles.
    if cfg.frontier_cells > 0 and VALIDITY_LAYER in msg.layers:
        elev = _extract_layer_as_array(msg, VALIDITY_LAYER)
        if elev.shape == costs.shape:
            observed = np.isfinite(elev)
            if observed.all():
                # Nothing unobserved: no frontier. (distance_transform_edt
                # with no zero element measures to the array border, which
                # would otherwise free every edge cell.)
                dist = None
            else:
                dist = ndimage.distance_transform_edt(observed)
            frontier = (np.zeros_like(observed) if dist is None
                        else observed & (dist <= cfg.frontier_cells))
            costs[frontier & (costs == cfg.cost_lethal)] = cfg.sparse_lethal_cost

    # 3b-bis. Support floor: a lethal verdict from one or two far points is noise,
    # not an obstacle. Downgrade to SOFT so the planner stops detouring around
    # phantom walls on a flat slope while still preferring to go around.
    if (cfg.min_support_for_lethal > 0 and COUNTS_LAYER in msg.layers):
        counts = _extract_layer_as_array(msg, COUNTS_LAYER)
        if counts.shape != costs.shape:
            raise ValueError(
                f"{COUNTS_LAYER}.shape={counts.shape} != "
                f"slope.shape={costs.shape}")
        sparse = np.isfinite(counts) & (counts < cfg.min_support_for_lethal)
        costs[(costs == cfg.cost_lethal) & sparse] = cfg.sparse_lethal_cost

    # 4. NaN handling: any NaN in slope or roughness => 85 (Nav2 unknown).
    nan_mask = np.isnan(slope) | np.isnan(roughness)
    if validity is not None:
        # Never-observed cells are unknown too, even though the inpainted
        # slope / roughness look like perfectly flat, free ground.
        nan_mask |= ~validity
    costs[nan_mask] = COST_UNKNOWN

    # 4b. Remove flat, isolated lethal cells left by the mean-difference
    # roughness at the edge of vertical structures (e.g. a door frame's foot).
    # Cells with their own height evidence (thin poles, rocks, steps) or a
    # near-vertical normal are kept. Requires the inpainted elevation layer;
    # without it the cleanup is skipped (previous behaviour).
    if cfg.small_cluster_max_cells > 0 and "elevation_inpainted" in msg.layers:
        elevation_inp = _extract_layer_as_array(msg, "elevation_inpainted")
        if elevation_inp.shape == costs.shape:
            costs = _suppress_flat_small_clusters(
                costs, elevation_inp, slope, cfg)

    # 4c. Clearance: a cell with less than `clearance_threshold_m` of headroom
    # is impassable even when its surface is perfectly flat -- the costmap has
    # no other way to see an overhang, because the arch's underside is not part
    # of the elevation surface at all. Applied AFTER the small-cluster cleanup
    # so a genuine low arch is not downgraded to free for "looking flat".
    if cfg.clearance_threshold_m > 0.0 and CLEARANCE_LAYER in msg.layers:
        clearance = _extract_layer_as_array(msg, CLEARANCE_LAYER)
        if clearance.shape != costs.shape:
            raise ValueError(
                f"{CLEARANCE_LAYER}.shape={clearance.shape} != "
                f"slope.shape={costs.shape}"
            )
        blocked = np.isfinite(clearance) & (clearance < cfg.clearance_threshold_m)
        if cfg.min_support_for_lethal > 0 and COUNTS_LAYER in msg.layers:
            # Same support floor as the slope/height tests: a cell whose
            # elevation came from one or two far points must not be turned
            # into an obstacle by the clearance test either (measured on
            # slope_20261114_132503: the cave profile's clearance layer alone
            # pushed ~5k cells to 254, with a median support of 6-7 points).
            counts_c = _extract_layer_as_array(msg, COUNTS_LAYER)
            if counts_c.shape == costs.shape:
                blocked &= ~(np.isfinite(counts_c)
                             & (counts_c < cfg.min_support_for_lethal))
        costs[blocked] = cfg.cost_lethal

    # 5. Upsample.
    factor = compute_upsample_factor(msg, target_resolution)
    if height % factor.n != 0 or width % factor.n != 0:
        raise ValueError(
            f"GridMap shape ({height}, {width}) not divisible by upsample "
            f"factor {factor.n}; cannot downsample to target_resolution "
            f"{target_resolution}"
        )
    upsampled = factor.apply(costs)

    # 5b. Free the cells the robot actually drove through.  Applied AFTER the
    # upsample so the mask lands on the published lattice, and after every
    # lethal rule so nothing can re-mark them.
    if trail_half_width_m > 0.0:
        upsampled = _apply_trail_clear(
            upsampled, trail_xy, trail_half_width_m, target_resolution,
            msg.info.pose.position.x - (upsampled.shape[1] * target_resolution) / 2.0,
            msg.info.pose.position.y - (upsampled.shape[0] * target_resolution) / 2.0,
            stats=stats, robot_xy=robot_xy,
        )

    # 6. Build OccupancyGrid.
    out = OccupancyGrid()
    out.header = Header()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = msg.header.frame_id

    out.info = MapMetaData()
    out.info.resolution = target_resolution
    out.info.height = upsampled.shape[0]
    out.info.width = upsampled.shape[1]
    # Origin shift: msg.info.pose is the CENTER of the grid; OccupancyGrid
    # origin is the LOWER-LEFT corner. We mirror msg.info.pose so the
    # geometric center stays aligned (Nav2 cares about center alignment
    # for SLAM/posture-overlay purposes more than it cares about the
    # exact pixel boundary).
    # deepcopy: rclpy messages are shared objects, so assigning the
    # pose and then moving its origin would mutate the INPUT message.
    # That bit the trail test (a second call saw a moved msg origin) and
    # would bite any caller that reuses a message.
    out.info.origin = deepcopy(msg.info.pose)
    out.info.origin.position.x = (
        msg.info.pose.position.x - (upsampled.shape[1] * target_resolution) / 2.0
    )
    out.info.origin.position.y = (
        msg.info.pose.position.y - (upsampled.shape[0] * target_resolution) / 2.0
    )

    # OccupancyGrid.data is `int8[]` per ROS spec and the rclpy binding
    # enforces [-128, 127] on assignment, which would overflow the 128/254
    # costs chosen by Task 2 ThresholdConfig. The on-wire CDR format is
    # actually a uint8 sequence, so we bypass the strict setter by writing
    # to the private `_data` slot directly with a Python list. This keeps
    # the wire-level values intact (Nav2 / costmap_2d consumers accept
    # 0-255 raw bytes; 128/254/85 are Nav2 Costmap2D-internal conventions).
    # int8 bit pattern, but as array('b') rather than a Python list: the
    # list route materialises one Python int per cell (~490k) per frame.
    out._data = array_mod.array(
        "b", upsampled.astype(np.uint8).flatten(order="C").view(np.int8).tobytes())
    return out