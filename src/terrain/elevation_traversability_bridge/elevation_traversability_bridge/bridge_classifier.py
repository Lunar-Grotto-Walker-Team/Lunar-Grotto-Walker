"""Cost classification from elevation_mapping derived layers.

slope_rad:   post-elevation_mapping `acos(normal_vectors_z)`, radians, [0, pi]
             0 = perfectly flat up, pi/2 = vertical wall
roughness_m: post-elevation_mapping `|elevation_inp - elevation_smooth|`, meters
             0 = perfectly smooth, larger = steppable roughness
"""
from __future__ import annotations

import math
from dataclasses import dataclass


# Use dataclass so users can override defaults via launch/yaml.
@dataclass(frozen=True)
class ThresholdConfig:
    # Slope: degrees. Default from spec § 6.4.
    slope_soft_upper_deg: float = 30.0    # <= soft_upper => not lethal
    slope_lethal_upper_deg: float = 35.0   # > lethal_upper => lethal

    # Roughness: meters. Default from spec § 6.4.
    roughness_soft_upper_m: float = 0.05  # <= soft_upper => not lethal
    roughness_lethal_upper_m: float = 0.15  # > lethal_upper => lethal

    # Outputs.
    cost_free: int = 0
    cost_soft: int = 128
    cost_lethal: int = 254

    # Small-cluster cleanup. The mean-difference roughness in the filter chain
    # bleeds a vertical structure's height into the adjacent flat cells, so a
    # door frame's foot can produce 1-3 isolated lethal cells on otherwise
    # flat ground. A lethal cluster of at most `small_cluster_max_cells` is
    # kept only when it carries its own height evidence: its elevation is at
    # least `small_cluster_min_height_m` above the 3x3 local minimum (a thin
    # pole, a rock or a step), or its slope exceeds
    # `small_cluster_min_slope_deg`. Flat artefacts are downgraded to free.
    # 0 disables the cleanup (previous behaviour).
    small_cluster_max_cells: int = 3
    small_cluster_min_height_m: float = 0.15
    small_cluster_min_slope_deg: float = 60.0

    # Clearance layer (optional, projector `publish_clearance`). A cell whose
    # lowest overhead is closer than this to its local ground cannot be driven
    # through -- the dog plus stowed arm is ~0.8 m tall, so 1.0 m is the
    # minimum gap it needs under an arch. Only honoured when the source
    # GridMap actually carries a `clearance` layer; 0 disables the check.
    clearance_threshold_m: float = 1.0

    # Support floor for a lethal cell. A 0.1 m cell holding one or two far,
    # grazing-angle points gets a noisy median: adjacent cells then differ by
    # 0.3-0.4 m and the slope layer reports 60-75 deg on flat ground (measured
    # 2026-11-15 on slope_20261114_132503: 66% of lethal cells sat at the
    # observation frontier, two thirds of them slope-only). Such cells are
    # downgraded to SOFT (128): traversable with a cost bump, so the planner
    # stops detouring around phantom obstacles but is still nudged away.
    # Real obstacles carry dozens of points per cell. 0 disables the rule.
    min_support_for_lethal: float = 8.0

    # What a too-sparse lethal cell becomes. FREE (0) removes the phantom
    # obstacle entirely -- a 1-7 point cell carries no trustworthy height, and
    # fine real obstacles are the local costmap's job. Use 128 to keep a cost
    # bump instead (the planner then still prefers to go around).
    sparse_lethal_cost: int = 0

    # Height criterion (2026-11-15, retuned 2026-11-19 for flat terrain).
    #
    # This is the PRIMARY obstacle test for this deployment: the ground is
    # flat/rough but steep slopes do not occur, and every real obstacle shows
    # up as a height discontinuity.  A cell whose elevation is at least
    # `height_soft_m` above its local ground is passable-but-costly (128);
    # at least `height_lethal_m` above it, impassable (254).  Compared to the
    # slope test this never fires on a sustained gradient, so the steep ramp
    # down into the cave stays free while a 20 cm rock/step does not.
    #
    # 0 disables the corresponding band (`height_soft_m = 0` keeps only the
    # lethal test, `height_lethal_m = 0` disables the whole height criterion).
    height_soft_m: float = 0.0
    height_lethal_m: float = 0.30
    # Window for the local ground under a cell.  2 = 5x5 = 0.5 m.  On a slope
    # the window itself has a drop of (2*k)*res*tan(slope): at 22 deg a 0.5 m
    # window already spans 0.20 m, so a steep ramp would be read as a step.
    # 1 = 3x3 = 0.3 m keeps a 50 deg ramp under the 0.20 m threshold.
    height_window_cells: int = 2
    # A raised cell needs at least this many raised cells in its own 3x3
    # neighbourhood (itself included).  This replaces the old 3x3 erosion:
    # with a 3x3 height window a genuine step edge is only ONE cell wide, so
    # erosion would erase it, while a single noisy cell still has a count
    # of 1 and is dropped.  2 = one neighbour, 3 = two neighbours.
    height_min_neighbors: int = 2

    # Frontier exemption: a cell within this many cells of an UNOBSERVED cell
    # cannot be made lethal by the criteria above. At the edge of a new scan
    # the map is still being built (crest occlusion shadows, sparse far
    # returns); those are filtered instead of turned into obstacles, and the
    # unobserved area beyond already blocks planning. 0 disables.
    frontier_cells: int = 2

    def __post_init__(self) -> None:
        if self.slope_soft_upper_deg < 0:
            raise ValueError("slope_soft_upper_deg must be >= 0")
        if self.slope_lethal_upper_deg <= self.slope_soft_upper_deg:
            raise ValueError("slope_lethal_upper_deg must be > slope_soft_upper_deg")
        if self.roughness_soft_upper_m < 0:
            raise ValueError("roughness_soft_upper_m must be >= 0")
        if self.roughness_lethal_upper_m <= self.roughness_soft_upper_m:
            raise ValueError("roughness_lethal_upper_m must be > roughness_soft_upper_m")
        if self.small_cluster_max_cells < 0:
            raise ValueError("small_cluster_max_cells must be >= 0")
        if self.small_cluster_min_height_m < 0:
            raise ValueError("small_cluster_min_height_m must be >= 0")
        if not 0.0 <= self.small_cluster_min_slope_deg <= 90.0:
            raise ValueError("small_cluster_min_slope_deg must be in [0, 90]")
        if self.clearance_threshold_m < 0:
            raise ValueError("clearance_threshold_m must be >= 0")
        if self.min_support_for_lethal < 0:
            raise ValueError("min_support_for_lethal must be >= 0")
        if not 0 <= self.sparse_lethal_cost <= 254:
            raise ValueError("sparse_lethal_cost must be in [0, 254]")
        if self.height_lethal_m < 0:
            raise ValueError("height_lethal_m must be >= 0")
        if self.height_soft_m < 0:
            raise ValueError("height_soft_m must be >= 0")
        if (self.height_lethal_m > 0
                and self.height_soft_m > self.height_lethal_m):
            raise ValueError("height_soft_m must be <= height_lethal_m")
        if self.height_window_cells < 1:
            raise ValueError("height_window_cells must be >= 1")
        if self.height_min_neighbors < 1:
            raise ValueError("height_min_neighbors must be >= 1")
        if self.frontier_cells < 0:
            raise ValueError("frontier_cells must be >= 0")


def _classify_one(metric: float, soft_upper: float, lethal_upper: float,
                  cost_free: int, cost_soft: int, cost_lethal: int) -> int:
    """Return cost for one threshold-passed dimension."""
    if metric <= soft_upper:
        return cost_free
    if metric <= lethal_upper:
        return cost_soft
    return cost_lethal


def classify_cell(
    slope_rad: float,
    roughness_m: float,
    cfg: ThresholdConfig | None = None,
) -> int:
    """Combine slope and roughness classifications using max-of-dimensions.

    Both dimensions are checked independently, and the more severe cost wins
    (so 30 deg slope AND 5cm roughness both "soft" => still "soft", not pushed
    to lethal; but a 30 deg "soft" slope with 20cm "lethal" roughness => lethal).
    """
    if cfg is None:
        cfg = ThresholdConfig()

    slope_deg = math.degrees(slope_rad)

    slope_cost = _classify_one(
        slope_deg,
        cfg.slope_soft_upper_deg,
        cfg.slope_lethal_upper_deg,
        cfg.cost_free, cfg.cost_soft, cfg.cost_lethal,
    )
    roughness_cost = _classify_one(
        roughness_m,
        cfg.roughness_soft_upper_m,
        cfg.roughness_lethal_upper_m,
        cfg.cost_free, cfg.cost_soft, cfg.cost_lethal,
    )
    return max(slope_cost, roughness_cost)