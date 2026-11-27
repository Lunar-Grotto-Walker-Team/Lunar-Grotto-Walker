"""Top-level LG launch: starts upstream MoveIt demo + head camera collision
applier + lgw_bt_planner stack.

Order:
  1. agx_arm_moveit/launch/demo.launch.py — upstream MoveIt with
     effector_type:=agx_gripper so the xacro <xacro:if effector_type=='agx_gripper'>
     block in agx_arm.srdf.xacro expands natively (gripper group + named states +
     disable_collisions). No SRDF install-dir patching.
  2. apply_head_camera_collision_node — 独立 rclcpp node,启动后读
     calibration/head_calibration.json, 构造 BOX 0.15×0.07×0.07 碰撞
     对象, 调 /apply_planning_scene 同步 apply, 成功后自退。必须在
     demo.launch.py 之后起(MoveIt 必须先 ready),所以 InsertEvent
     策略保证顺序: 与 lgw_bt_planner.launch.py 没有依赖, 让它先于 BT
     runner 启动以确保 arm MoveGroup 路径规划会避开 head 相机。
  3. lgw_bt_planner/launch/lgw_bt_planner.launch.py — bt_runner +
     select_arm_observation_pose_service_node (后者由该 launch 自带,
     不在本文件另起 —— 避免双启动冲突)。

Prerequisites (handled by start_lgw_pose_replay.sh L2 wrapper, NOT by this launch):
  - can0 must be UP @ 1 Mbps.
    Run: bash /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000
  - agx_arm_ctrl hardware driver must already be running.
    Run: ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
           effector_type:=agx_gripper tcp_offset:='...'
This matches the upstream agx_arm_ros README convention — start_single_agx_arm.launch.py
is itself a top-level launch meant to be invoked explicitly, not embedded.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    demo_launch = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'launch', 'demo.launch.py')
    lgw_bt_launch = os.path.join(
        get_package_share_directory('lgw_bt_planner'),
        'launch', 'lgw_bt_planner.launch.py')

    tcp_offset_arg = '[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments={
                'use_rviz': 'false',
                'follow': 'true',
                'tcp_offset': tcp_offset_arg,
                # effector_type:=agx_gripper → move_group's _moveit_config_builder
                # processes agx_arm.srdf.xacro with this mapping, expanding the
                # <xacro:if effector_type=='agx_gripper'> block (gripper group,
                # gripper_open/half/close named states, disable_collisions).
                # Default 'none' would leave those out → MoveArmToNamed
                # gripper_open/close fails. Demo's launch arg defaults to 'none'
                # so we must pass it explicitly.
                'effector_type': 'agx_gripper',
            }.items(),
        ),

        # head camera collision applier — 见文件顶部 step 2。launch action
        # 列表本身按顺序执行,所以这里 demo 之后即保证 MoveIt 已起。
        # output='screen' 让 .run/*.log 接得到 apply 成功/失败日志。
        Node(
            package='lgw_bt_planner',
            executable='apply_head_camera_collision_node',
            name='head_camera_collision',
            output='screen',
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(lgw_bt_launch)),
    ])