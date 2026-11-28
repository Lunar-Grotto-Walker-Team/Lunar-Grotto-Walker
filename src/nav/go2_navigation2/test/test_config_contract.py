import ast
import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def _params():
    with open(os.path.join(PKG, "config", "nav2_params.yaml")) as f:
        return yaml.safe_load(f)


def test_global_costmap_is_a_rolling_terrain_window():
    g = _params()["global_costmap"]["global_costmap"]["ros__parameters"]
    assert g["resolution"] == 0.10
    assert g["rolling_window"] is True
    assert (g["width"], g["height"]) == (60, 60)
    assert "origin_x" not in g and "origin_y" not in g
    assert g["track_unknown_space"] is True
    # 2026-11-19: the L2 near-field obstacle layer is back, and its position
    # in the list is load-bearing: the terrain layer writes the whole grid
    # (FREE included) directly into the master each cycle, so anything the
    # obstacle layer marks must be written AFTER it -- otherwise the next
    # terrain write erases it, and stale marks would never be cleared.
    assert g["plugins"] == [
        "terrain_traversability_layer", "obstacle_layer", "inflation_layer"]
    assert g["terrain_traversability_layer"]["unknown_policy"] == "no_information"
    assert g["terrain_traversability_layer"]["enabled"] is True
    # Only L2 in the global layer: the Livox sources are already represented
    # by the terrain grid, and their wider marks are what smeared after map
    # jumps in experiment A (2026-11-10).
    ol = g["obstacle_layer"]
    assert ol["observation_sources"] == "l2_ground l2_obstacles"
    assert ol["l2_ground"]["topic"] == "/cloud_ground_l2"
    assert ol["l2_obstacles"]["topic"] == "/cloud_obstacles_l2"
    assert ol["l2_ground"]["marking"] is False
    assert ol["l2_obstacles"]["marking"] is True
    # L2 is trusted out to 1.5 m; raytrace must reach at least that far.
    for src in ("l2_ground", "l2_obstacles"):
        assert ol[src]["obstacle_max_range"] == 1.5
        assert ol[src]["raytrace_max_range"] >= ol[src]["obstacle_max_range"]
    assert "obstacles" not in ol          # no Livox point source
    assert "cloud" not in ol


def test_global_planner_may_route_through_unknown():
    # The robot's own cell is UNKNOWN in the terrain map (Mid360S blind zone
    # + 13 deg down-tilt => no ground return within ~1.07 m). With
    # allow_unknown=false NavFn cannot even start, so every goal fails.
    p = _params()["planner_server"]["ros__parameters"]
    assert p["GridBased"]["allow_unknown"] is True


def test_local_costmap_uses_the_ground_filter_sources():
    l = _params()["local_costmap"]["local_costmap"]["ros__parameters"]
    assert l["plugins"] == ["obstacle_layer", "inflation_layer"]
    ol = l["obstacle_layer"]
    assert ol["observation_sources"] == (
        "ground obstacles l2_ground l2_obstacles")
    assert ol["ground"]["topic"] == "/cloud_ground"
    assert ol["obstacles"]["topic"] == "/cloud_obstacles"
    # 2026-11-18: L2 近场两路, 与 Livox 那两路同语义(ground 只清不标)。
    assert ol["l2_ground"]["topic"] == "/cloud_ground_l2"
    assert ol["l2_obstacles"]["topic"] == "/cloud_obstacles_l2"
    assert ol["l2_ground"]["marking"] is False
    assert ol["l2_ground"]["clearing"] is True
    assert ol["l2_obstacles"]["marking"] is True
    assert ol["l2_obstacles"]["clearing"] is True
    # L2 稀疏: 只信任 1.5 m。raytrace 必须 >= obstacle, 否则最后一段永不清。
    for src in ("l2_ground", "l2_obstacles"):
        assert ol[src]["obstacle_max_range"] == 1.5
        assert ol[src]["raytrace_max_range"] >= ol[src]["obstacle_max_range"]
        # 近场清除必须从 0 起(脚下就是盲区)。
        assert ol[src]["raytrace_min_range"] == 0.0
    # Ground points may clear but must never mark; obstacles mark.
    assert ol["ground"]["marking"] is False
    assert ol["ground"]["clearing"] is True
    assert ol["obstacles"]["marking"] is True
    assert "voxel_layer" not in l
    # The old single absolute-height source must be gone.
    assert "cloud" not in ol
    # The local costmap is a display of the robot's immediate surroundings;
    # publishing at 2 Hz made it visibly lag the dog by ~0.5 s in RViz.
    # publish must stay BELOW update: Nav2 publishes from the update loop
    # with a deadline check, so equal rates miss deadlines under jitter
    # (measured 3.2 Hz at 5/5). 10/5 gives a true 5 Hz.
    assert l["update_frequency"] == 10.0
    assert l["publish_frequency"] == 5.0


def test_launch_drops_scan_and_bridge_and_adds_relay():
    path = os.path.join(PKG, "launch", "go2_nav2_slam_explore.launch.py")
    text = open(path).read()
    # The module docstring legitimately explains what was removed (it names
    # the bridge and the old /scan chain), so only assert on executable code.
    doc = ast.get_docstring(ast.parse(text))
    code = text.replace(doc, "") if doc else text
    assert "go2_pointcloud.launch.py" not in code
    assert "cloud_accumulation" not in code
    assert "elevation_traversability_bridge" not in code
    assert "pointlio_cloud_relay" in code
    # Both the relay and the ground filter are started from this launch.
    assert "cloud_to_base" in code
    assert "ground_filter" in code
    # Both map_odom_correction and cloud_to_base load the shared extrinsic.
    assert code.count("pointlio_frames.yaml") >= 2


def test_shared_extrinsic_has_one_source_of_truth():
    with open(os.path.join(PKG, "config", "pointlio_frames.yaml")) as f:
        cfg = yaml.safe_load(f)
    a = cfg["map_odom_correction"]["ros__parameters"]
    b = cfg["cloud_to_base"]["ros__parameters"]
    c = cfg["ground_filter"]["ros__parameters"]
    for k in ("body_to_base_x", "body_to_base_y", "body_to_base_z",
              "body_to_base_roll", "body_to_base_pitch", "body_to_base_yaw"):
        assert a[k] == b[k] == c[k]
    assert c["output_frame"] == "base_footprint"
    assert a["body_frame"] == "aft_mapped"
    assert b["body_frame"] == "aft_mapped"
    assert c["body_frame"] == "aft_mapped"


def test_l2_ground_filter_runs_in_the_odom_frame_with_identity_extrinsic():
    with open(os.path.join(PKG, "config", "pointlio_frames.yaml")) as f:
        cfg = yaml.safe_load(f)
    l2 = cfg["ground_filter_l2"]["ros__parameters"]
    # /utlidar/cloud_deskewed 的 frame_id 就是 odom; 机器人位姿在 odom 里
    # 就是 base_footprint, 所以 body -> base 是恒等。把 Livox 那段的
    # 0.187/pitch13 复制过来会让整片点云平移 0.39 m。
    assert l2["body_frame"] == "base_footprint"
    assert l2["output_frame"] == "base_footprint"
    for k in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert l2[f"body_to_base_{k}"] == 0.0
    assert l2["input_topic"] == "/utlidar/cloud_deskewed"
    # 窄栅格: L2 只信任 1.5 m, 4 m 网格够覆盖 + 邻域填洞。
    assert l2["grid_size"] == 4.0
    assert l2["cell_size"] == 0.3


def test_launch_starts_the_l2_ground_filter_behind_a_flag():
    path = os.path.join(PKG, "launch", "go2_nav2_slam_explore.launch.py")
    code = open(path).read()
    assert "name='ground_filter_l2'" in code
    assert "use_utlidar_l2" in code
    # 默认开(用户 2026-11-18 的验收前提), 但必须能一键关掉。
    assert "default_value='true'" in code
