import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
BT_XML = os.path.join(
    PKG, "behavior_trees", "navigate_to_pose_terrain_recovery.xml")


def _tree():
    return ET.parse(BT_XML).getroot()


def _params():
    with open(os.path.join(PKG, "config", "nav2_params.yaml")) as f:
        return yaml.safe_load(f)


def test_tree_has_single_main_tree():
    root = _tree()
    assert root.tag == "root"
    assert root.attrib["main_tree_to_execute"] == "MainTree"
    trees = root.findall("BehaviorTree")
    assert len(trees) == 1
    assert trees[0].attrib["ID"] == "MainTree"


def test_no_global_costmap_clearing():
    # The global costmap is rebuilt from the upstream /terrain_traversability
    # every cycle. Clearing it only drops the plugin's cached message; the same
    # map returns within <=0.5 s. Worse, the all-UNKNOWN window is reachable by
    # NavFn (allow_unknown: true), so a plan issued in that window can cross
    # cells that were lethal a moment ago. Global clearing belongs to
    # map_odom_correction (clear_costmap_on_switch), not to the BT.
    for node in _tree().iter("ClearEntireCostmap"):
        assert node.attrib["service_name"].startswith(
            "local_costmap/"), node.attrib


def test_recovery_budget_and_action_order():
    recovery = _tree().find(".//RecoveryNode[@name='NavigateRecovery']")
    assert recovery is not None
    assert recovery.attrib["number_of_retries"] == "3"
    round_robin = recovery.find(".//RoundRobin")
    assert round_robin is not None
    actions = [child.tag for child in round_robin]
    # Wait first: give the 2 Hz terrain chain and PGO a chance.
    # Spin second: re-observe. BackUp removed: 6 s in a 1.5 m cave buys little.
    assert actions == ["Wait", "Spin", "ClearEntireCostmap"]
    assert _tree().find(".//BackUp") is None


def test_required_plugins_are_registered():
    plugins = set(
        _params()["bt_navigator"]["ros__parameters"]["plugin_lib_names"])
    required = {
        "nav2_compute_path_to_pose_action_bt_node",
        "nav2_follow_path_action_bt_node",
        "nav2_clear_costmap_service_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_round_robin_node_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "nav2_wait_action_bt_node",
        "nav2_spin_action_bt_node",
    }
    assert required <= plugins


def test_launch_injects_bt_xml_only_into_bt_navigator():
    # The path must be injected as a node-level parameter dict, not through
    # RewrittenYaml param_rewrites: param_rewrites would add the key to every
    # node's ros__parameters section, and rclcpp refuses undeclared parameters
    # at startup for controller/planner/behavior servers.
    with open(os.path.join(PKG, "launch", "nav2_minimal.launch.py")) as f:
        src = f.read()
    assert "default_nav_to_pose_bt_xml" in src
    assert "navigate_to_pose_terrain_recovery.xml" in src
    assert "'bt_xml'" in src
