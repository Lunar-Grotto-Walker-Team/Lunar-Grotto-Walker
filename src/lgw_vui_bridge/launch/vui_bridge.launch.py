"""Launch vui_bridge_node.

Standalone launcher for the VUI SDK bridge. `go2_posture_demo.launch.py`
includes this file via IncludeLaunchDescription (same pattern as
body_pose_bridge.launch.py — both bridges are independent rclcpp
processes that own third-party SDK / hardware channels; their lifecycles
deliberately outlive a single BT round-trip so service clients can
discover them once and stay connected).

Why a separate launcher:
  - vui_bridge must come up before bt_runner instantiates a
    SetLedBrightnessNode in a fresh tree, otherwise the RosServiceNode's
    internal wait_for_service() will time out and the first tick will
    FAILURE the BT.
  - The `ETH_INTERFACE_NAME` env var override lets operators point the
    bridge at a different subnet without recompiling (e.g. during dog
    re-IP). Pass via:
      ETH_INTERFACE_NAME=eth0 ros2 launch lgw_vui_bridge vui_bridge.launch.py

Usage:
  ros2 launch lgw_vui_bridge vui_bridge.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Allow operators to override the network interface without
        # editing this file; default falls back inside
        # vui_bridge_node.cpp's getNetworkInterface() (which prefers the
        # same-named env var over its compile-time default "eth10").
        DeclareLaunchArgument(
            'network_interface',
            default_value=EnvironmentVariable('ETH_INTERFACE_NAME',
                                              default_value='eth10'),
            description='Cyclone DDS network interface for the SDK '
                        'DomainParticipant. Default "eth10" matches '
                        'l4t/l4t_setup.sh CYCLONEDDS_URI; override via '
                        'ETH_INTERFACE_NAME env for non-default dog subnet.',
        ),
        Node(
            package='lgw_vui_bridge',
            executable='vui_bridge_node',
            name='vui_bridge',
            output='screen',
        ),
    ])
