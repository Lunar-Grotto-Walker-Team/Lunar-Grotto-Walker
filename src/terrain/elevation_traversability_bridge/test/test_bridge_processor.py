"""Tests for elevation_traversability_bridge.bridge_processor.

NOTE on dtype: nav_msgs/OccupancyGrid.data is `int8[]` per the ROS spec,
which holds [-128, 127]. The classifier produces 128/254 (Nav2 Costmap2D
internal convention). These values overflow int8 on read.

The implementation stores values as a Python list via the private `_data`
slot so that 128/254 survive on the wire (ROS2's on-wire format is actually
a uint8 sequence; the Python binding's int8 assertion is overly strict).
For verification, the tests read the list with `dtype=np.uint8` to view
the raw bytes 0-255 without signed overflow.
"""
import math

import numpy as np
from grid_map_msgs.msg import GridMap as GridMapMsg
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension, MultiArrayLayout
from nav_msgs.msg import OccupancyGrid, MapMetaData

from elevation_traversability_bridge.bridge_classifier import ThresholdConfig
from elevation_traversability_bridge.bridge_processor import (
    process_gridmap,
    UpsampleFactor,
)


def _cost_array(out: OccupancyGrid) -> np.ndarray:
    """Read out.data as uint8 so 128/254 survive (int8 would overflow).

    Nav2 costmap values are 0-100 (Nav2 standard), 128/254 (Costmap2D
    internal), and 255 (unknown). The bridge emits 0/85/128/254 in this
    project (chosen by Task 2 ThresholdConfig), so uint8 is the only
    safe read for verification.
    """
    return np.array(out.data, dtype=np.uint8).reshape(out.info.height, out.info.width)


def _make_gridmap(
    layer_data: dict[str, np.ndarray],
    resolution: float = 0.10,
    length_x: float = 5.0,
    length_y: float = 5.0,
    frame_id: str = "map",
    pose_x: float | None = None,
    pose_y: float | None = None,
) -> GridMapMsg:
    """Build a GridMap message exactly the way grid_map_ros does.

    `layer_data` maps layer name -> 2D array of shape (n_y, n_x), indexed
    [row = y from min, col = x from min] — the lower-left-origin world
    picture that an OccupancyGrid uses.

    Serialization is verified against the real grid_map_core (see
    pgo_gridmap_projector/test/gridmap_reference.cpp):
      * getSize() = (length_x/res, length_y/res) = (n_x, n_y);
      * buffer index (0, 0) is the cell at (max x, max y);
      * the layer matrix is column-major;
      * layout.dim[0].size = n_y (Eigen outerSize), dim[1].size = n_x.
    """
    n_y, n_x = next(iter(layer_data.values())).shape
    msg = GridMapMsg()
    msg.info.resolution = resolution
    msg.info.length_x = length_x
    msg.info.length_y = length_y
    msg.info.pose.position.x = length_x / 2.0 if pose_x is None else pose_x
    msg.info.pose.position.y = length_y / 2.0 if pose_y is None else pose_y
    msg.layers = sorted(layer_data.keys())
    msg.basic_layers = ["elevation"] if "elevation" in msg.layers else msg.layers[:1]
    msg.data = []
    for layer in msg.layers:
        arr = layer_data[layer].astype(np.float32)   # [y, x]
        eigen = arr[::-1, ::-1].T                    # [x_from_max, y_from_max]
        flat = eigen.flatten(order="F")              # column-major buffer
        dim0 = MultiArrayDimension(label="column_index", size=n_y,
                                   stride=n_x * n_y)
        dim1 = MultiArrayDimension(label="row_index", size=n_x,
                                   stride=n_x)
        layout = MultiArrayLayout(
            dim=[dim0, dim1],
            data_offset=0,
        )
        flat_msg = Float32MultiArray(
            layout=layout,
            data=flat.tolist(),
        )
        msg.data.append(flat_msg)
    msg.header.frame_id = frame_id
    return msg


def test_process_gridmap_flat_returns_free():
    h, w = 10, 10
    layer = {
        "elevation": np.zeros((h, w), dtype=np.float32),
        "slope": np.zeros((h, w), dtype=np.float32),
        "roughness": np.zeros((h, w), dtype=np.float32),
    }
    msg = _make_gridmap(layer)
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    assert arr.max() <= 0
    assert arr.shape == (20, 20)   # 10x10 upsampled 2x -> 20x20
    assert out.info.resolution == 0.05
    assert out.header.frame_id == "map"


def test_process_gridmap_steep_slope_is_lethal():
    h, w = 10, 10
    elevation = np.zeros((h, w), dtype=np.float32)
    slope = np.full((h, w), math.radians(40.0), dtype=np.float32)
    roughness = np.zeros((h, w), dtype=np.float32)
    msg = _make_gridmap({
        "elevation": elevation,
        "slope": slope,
        "roughness": roughness,
    })
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    assert (arr == 254).all()


def test_process_gridmap_soft_band_has_128():
    h, w = 4, 4
    elevation = np.zeros((h, w), dtype=np.float32)
    slope = np.full((h, w), math.radians(32.0), dtype=np.float32)
    roughness = np.zeros((h, w), dtype=np.float32)
    msg = _make_gridmap({
        "elevation": elevation,
        "slope": slope,
        "roughness": roughness,
    })
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    assert (arr == 128).all()


def test_process_gridmap_nan_layer_cells_become_unknown():
    h, w = 4, 4
    elevation = np.zeros((h, w), dtype=np.float32)
    slope = np.full((h, w), math.radians(0.0), dtype=np.float32)
    slope[1, 1] = math.nan
    roughness = np.full((h, w), 0.0, dtype=np.float32)
    msg = _make_gridmap({
        "elevation": elevation,
        "slope": slope,
        "roughness": roughness,
    })
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    # NaN 区域内必须 == 85(unknown),upsample 后对应一块 2x2 的 area
    # (1, 1) in (4, 4) -> (2..3, 2..3) after 2x upsampling
    block = arr[2:4, 2:4]
    assert (block == 85).all()


def test_process_gridmap_missing_layer_raises():
    h, w = 4, 4
    msg = _make_gridmap({
        "elevation": np.zeros((h, w), dtype=np.float32),
        # 'slope' missing
        "roughness": np.zeros((h, w), dtype=np.float32),
    })
    import pytest
    with pytest.raises(ValueError, match="missing required layer"):
        process_gridmap(msg, ThresholdConfig())


def test_process_gridmap_frame_id_carried():
    h, w = 4, 4
    msg = _make_gridmap({
        "elevation": np.zeros((h, w), dtype=np.float32),
        "slope": np.zeros((h, w), dtype=np.float32),
        "roughness": np.zeros((h, w), dtype=np.float32),
    }, frame_id="map")
    out = process_gridmap(msg, ThresholdConfig())
    assert out.header.frame_id == "map"


def test_upsampling_takes_max_per_subcell():
    """Two adjacent cells: lethal next to free → upsampled block = LETHAL."""
    h, w = 2, 2
    elevation = np.zeros((h, w), dtype=np.float32)
    slope = np.array(
        [
            [0.0, math.radians(40.0)],
            [0.0, 0.0],
        ],
        dtype=np.float32,
    )
    roughness = np.zeros((h, w), dtype=np.float32)
    msg = _make_gridmap({
        "elevation": elevation,
        "slope": slope,
        "roughness": roughness,
    })
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    # 2x2 input -> 4x4 output. (0, 1) cell is lethal, expands to (0:2, 2:4).
    assert arr.shape == (4, 4)
    assert (arr[0:2, 2:4] == 254).all()
    # 同样的 (0, 1) 在 upsample 中 max 传播到邻居? 不,max 严格只在自己 sub-block。
    assert (arr[0:2, 0:2] == 0).all()

def test_process_gridmap_unobserved_elevation_becomes_unknown():
    """elevation 层 NaN = 该格从未被观测过 -> UNKNOWN(85),不能当成可通行。

    后处理链的 InpaintFilter 用 cv::inpaint 会把整张图的 NaN 空洞全填平,
    所以未观测区的 slope / roughness 也是有限的 0(看起来像"平地")。
    只有原始 `elevation` 层(未被 inpaint 污染)还保留着 NaN,必须拿它做
    有效性掩膜,否则整张 50m x 50m 图会被判成处处可通行。
    """
    h, w = 4, 4
    elevation = np.zeros((h, w), dtype=np.float32)
    elevation[0, :] = np.nan           # 第一行从未被观测
    slope = np.zeros((h, w), dtype=np.float32)        # inpaint 填平后的"平地"
    roughness = np.zeros((h, w), dtype=np.float32)
    msg = _make_gridmap({
        "elevation": elevation,
        "slope": slope,
        "roughness": roughness,
    })
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    # 2x 上采样:输入第 0 行 -> 输出第 0-1 行
    assert (arr[0:2, :] == 85).all(), "未观测格必须是 UNKNOWN(85),不能是 FREE(0)"
    assert (arr[2:, :] == 0).all(), "观测过的格仍是可通行"


def test_process_gridmap_without_elevation_layer_falls_back_to_nan_rule():
    """没有 elevation 层时退回旧规则(只看 slope / roughness 的 NaN)。"""
    h, w = 4, 4
    slope = np.zeros((h, w), dtype=np.float32)
    roughness = np.zeros((h, w), dtype=np.float32)
    msg = _make_gridmap({"slope": slope, "roughness": roughness})
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    assert (arr == 0).all()


def test_conversion_preserves_world_position():
    """A lethal cell at world (1.5, 1.0) must stay at (1.5, 1.0).

    Regression for the 180-degree rotation: grid_map's buffer origin is the
    top-right cell while OccupancyGrid's is the bottom-left, so the bridge
    must flip both axes. The old `_make_gridmap()` helper serialized with
    the same wrong assumption as the implementation, which is why this was
    never caught.
    """
    n_y, n_x = 3, 4            # 4 m x 3 m at 1 m/cell, centred on origin
    elevation = np.zeros((n_y, n_x), dtype=np.float32)
    slope = np.zeros((n_y, n_x), dtype=np.float32)
    roughness = np.zeros((n_y, n_x), dtype=np.float32)
    # row 2 -> y = -1.5 + 2.5 = +1.0 ; col 3 -> x = -2.0 + 3.5 = +1.5
    slope[2, 3] = 1.0          # 57 deg -> LETHAL
    msg = _make_gridmap(
        {"elevation": elevation, "slope": slope, "roughness": roughness},
        resolution=1.0, length_x=4.0, length_y=3.0,
        pose_x=0.0, pose_y=0.0)

    out = process_gridmap(msg, ThresholdConfig(), target_resolution=1.0)
    arr = _cost_array(out)
    assert (out.info.height, out.info.width) == (n_y, n_x)
    assert arr[2, 3] == 254, "lethal cell moved: world position not preserved"
    assert arr[0, 0] == 0, "the opposite corner must stay free"


# --- small-cluster cleanup (2026-10-31) -----------------------------------

def _cluster_msg(costs_rough, elevation_inp, slope_deg=0.0,
                 include_inpainted=True):
    """5x5 grid: a lethal mask driven by roughness, flat unless overridden."""
    h, w = costs_rough.shape
    layers = {
        "elevation": np.zeros((h, w), dtype=np.float32),
        "slope": np.full((h, w), math.radians(slope_deg), dtype=np.float32),
        "roughness": costs_rough.astype(np.float32),
    }
    if include_inpainted:
        layers["elevation_inpainted"] = elevation_inp.astype(np.float32)
    return _make_gridmap(layers)


def _cell(arr, r, c):
    """Read the 2x2 upsampled block for source cell (r, c)."""
    return arr[2 * r:2 * r + 2, 2 * c:2 * c + 2]


def test_flat_isolated_lethal_is_suppressed():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[2, 2] = 0.20        # lethal by roughness
    elev = np.zeros((6, 6), dtype=np.float32)
    out = process_gridmap(_cluster_msg(rough, elev), ThresholdConfig())
    arr = _cost_array(out)
    assert arr.max() == 0, "flat isolated lethal cell must be downgraded"
    assert (_cell(arr, 2, 2) == 0).all()


def test_elevated_isolated_lethal_is_kept():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[2, 2] = 0.20
    elev = np.zeros((6, 6), dtype=np.float32)
    elev[2, 2] = 0.40         # a real bump / thin pole
    out = process_gridmap(_cluster_msg(rough, elev), ThresholdConfig())
    arr = _cost_array(out)
    assert (_cell(arr, 2, 2) == 254).all()
    assert arr.sum() == 254 * 4


def test_large_flat_cluster_is_kept():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[1:3, 1:3] = 0.20    # 2x2 = 4 cells > max_cells=3
    elev = np.zeros((6, 6), dtype=np.float32)
    out = process_gridmap(_cluster_msg(rough, elev), ThresholdConfig())
    arr = _cost_array(out)
    assert (_cell(arr, 1, 1) == 254).all()
    assert (_cell(arr, 2, 2) == 254).all()


def test_small_steep_cluster_is_kept():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[2, 2] = 0.20
    elev = np.zeros((6, 6), dtype=np.float32)
    out = process_gridmap(
        _cluster_msg(rough, elev, slope_deg=70.0), ThresholdConfig())
    arr = _cost_array(out)
    assert (_cell(arr, 2, 2) == 254).all()


def test_missing_inpainted_layer_skips_cleanup():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[2, 2] = 0.20
    elev = np.zeros((6, 6), dtype=np.float32)
    msg = _cluster_msg(rough, elev, include_inpainted=False)
    out = process_gridmap(msg, ThresholdConfig())
    arr = _cost_array(out)
    assert (_cell(arr, 2, 2) == 254).all(), \
        "without elevation_inpainted the cleanup must be a no-op"


def test_disabling_cleanup_keeps_flat_lethal():
    rough = np.zeros((6, 6), dtype=np.float32)
    rough[2, 2] = 0.20
    elev = np.zeros((6, 6), dtype=np.float32)
    cfg = ThresholdConfig(small_cluster_max_cells=0)
    out = process_gridmap(_cluster_msg(rough, elev), cfg)
    arr = _cost_array(out)
    assert (_cell(arr, 2, 2) == 254).all()


# --- flat-terrain height criterion (2026-11-19) ---------------------------
def _mound_msg(elev, slope=None):
    h, w = elev.shape
    if slope is None:
        slope = np.zeros((h, w), dtype=np.float32)
    return _make_gridmap({
        "elevation": elev.astype(np.float32),
        "slope": slope.astype(np.float32),
        "roughness": np.zeros((h, w), dtype=np.float32),
        "counts": np.full((h, w), 20.0, dtype=np.float32),
    })


def _height_cfg(**kwargs):
    """The flat-terrain deployment profile (bridge_params.yaml)."""
    defaults = dict(height_soft_m=0.15, height_lethal_m=0.20,
                    height_window_cells=1, height_min_neighbors=2)
    defaults.update(kwargs)
    return ThresholdConfig(**defaults)


def test_one_cell_ridge_is_now_a_real_obstacle():
    """A raised 1-cell line inside fully observed terrain is an obstacle.

    Height is now the primary test. The old 3x3 erosion removed 1-cell lines
    to suppress crest occlusion shadows, but those always border UNOBSERVED
    cells, which the frontier exemption already handles. A 0.4 m ridge backed
    by real points is therefore a thin object (post, kerb), not a seam.
    """
    elev = np.zeros((20, 20))
    elev[10, 3:17] = 0.4
    arr = _cost_array(process_gridmap(_mound_msg(elev)))
    assert (arr[20:22, 6:34] == 254).all()


def test_lone_noisy_cell_is_not_an_obstacle():
    """height_min_neighbors: a single raised cell has count 1 and is dropped."""
    elev = np.zeros((20, 20))
    elev[10, 10] = 0.4
    arr = _cost_array(process_gridmap(_mound_msg(elev)))
    assert (arr == 0).all()


def test_fourteen_cm_bump_is_free():
    """The dog crosses 15 cm cleanly: stay FREE."""
    elev = np.zeros((20, 20))
    elev[8:12, 8:12] = 0.14
    arr = _cost_array(process_gridmap(_mound_msg(elev), _height_cfg()))
    assert (arr[16:24, 16:24] == 0).all()


def test_seventeen_cm_bump_is_soft():
    """15-20 cm: crossable, but the planner should prefer to go around.

    The 3x3 window only sees the bump's rim: the interior of a 4x4 bump
    keeps its own height as local ground (dh = 0), which is fine -- the rim
    encloses it.
    """
    elev = np.zeros((20, 20))
    elev[8:12, 8:12] = 0.17
    arr = _cost_array(process_gridmap(_mound_msg(elev), _height_cfg()))
    assert (arr[16, 16:24] == 128).all()
    assert (arr[16:24, 16] == 128).all()
    assert (arr[18:22, 18:22] == 0).all()


def test_twenty_two_cm_obstacle_is_lethal():
    """20 cm is forced only with foot slip, which corrupts mapping."""
    elev = np.zeros((20, 20))
    elev[8:12, 8:12] = 0.22
    arr = _cost_array(process_gridmap(_mound_msg(elev), _height_cfg()))
    assert (arr[16, 16:24] == 254).all()
    assert (arr[16:24, 16] == 254).all()


def test_steep_ramp_is_not_an_obstacle():
    """The cave entrance ramp must stay drivable.

    Diagonal orientation is the worst case: the 3x3 corner is 1.414 cells
    away, so a 45 deg plane spans 0.141 m across the window -- under both
    thresholds -- while the old 5x5 window would have spanned 0.25 m and
    marked the whole ramp lethal.
    """
    idx = np.arange(20, dtype=np.float64)
    elev = 0.0707 * (idx[:, None] + idx[None, :])
    arr = _cost_array(process_gridmap(_mound_msg(elev), _height_cfg()))
    assert (arr == 0).all()
