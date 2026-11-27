"""Launch the elevation_mapping GridMap -> Nav2 traversability bridge.

Single-node launch. Loads thresholds + topics from
config/bridge_params.yaml so the same binary can be retuned per deployment
without rebuilding.

Usage:
    ros2 launch elevation_traversability_bridge bridge.launch.py
    # Override any ros__parameters key on the command line:
    ros2 launch elevation_traversability_bridge bridge.launch.py \\
        terrain_traversability_bridge.required_frame_id:=odom
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("elevation_traversability_bridge")
    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "bridge_params.yaml"]
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description=(
            "YAML file with ros__parameters for the bridge node. "
            "See config/bridge_params.yaml for the schema."
        ),
    )

    bridge_node = Node(
        package="elevation_traversability_bridge",
        executable="bridge_node",
        name="terrain_traversability_bridge",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        bridge_node,
    ])


