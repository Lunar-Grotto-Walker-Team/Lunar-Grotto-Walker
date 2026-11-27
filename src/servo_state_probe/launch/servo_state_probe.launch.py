"""Start the state-monitor probe with the *same* URDF/SRDF Servo gets.

The moveit_config is rebuilt with agx_arm_moveit's own builder (the same
module lgw_full.launch.py imports), so the probe's JointModelGroup cannot
disagree with Servo's about arm_type / tcp_offset.

The mock/real MoveIt stack must already be running (lgw_full.launch.py);
this launch starts nothing but the probe.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_AGX_ARM_MOVEIT_LAUNCH_DIR = os.path.join(
    get_package_share_directory('agx_arm_moveit'), 'launch')
if _AGX_ARM_MOVEIT_LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _AGX_ARM_MOVEIT_LAUNCH_DIR)

from _moveit_config_builder import (  # noqa: E402
    ALL_ARM_TYPES,
    ALL_EFFECTOR_TYPES,
    ALL_REVO2_TYPES,
    build_moveit_config,
)


def _probe(context):
    moveit_config = build_moveit_config(context)
    return [
        Node(
            package='servo_state_probe',
            executable='servo_state_probe',
            name='servo_state_probe',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                {
                    'joint_states_topic': LaunchConfiguration(
                        'joint_states_topic'),
                    'joint_group': LaunchConfiguration('joint_group'),
                    'print_period_ms': 1000,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_type', default_value='piper',
                              choices=ALL_ARM_TYPES),
        DeclareLaunchArgument('effector_type', default_value='agx_gripper',
                              choices=ALL_EFFECTOR_TYPES),
        DeclareLaunchArgument('revo2_type', default_value='left',
                              choices=ALL_REVO2_TYPES),
        DeclareLaunchArgument(
            'tcp_offset',
            default_value='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]',
            description='Must match lgw_full.launch.py (DEFAULT_TCP_OFFSET).'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='/control/joint_states',
            description='Same value Servo is configured with.'),
        DeclareLaunchArgument('joint_group', default_value='arm'),
        OpaqueFunction(function=_probe),
    ])
