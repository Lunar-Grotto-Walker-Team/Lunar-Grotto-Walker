"""只起 go2_transit 一个节点。

nav2 / point_lio / elevmap 由别的 launch 负责 —— 本节点是消费方。

用法（必须自己 source 本工作区的 install，l4t_setup.sh 不含它）：
  ros2 launch go2_transit transit.launch.py
  ros2 launch go2_transit transit.launch.py dry_run:=true
  ros2 launch go2_transit transit.launch.py params_file:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_transit')
    default_params = os.path.join(pkg_share, 'config', 'cave_transit.yaml')

    params_file = LaunchConfiguration('params_file')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='参数文件路径',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='只算路径不动机器狗',
        ),
        Node(
            package='go2_transit',
            executable='transit_node',
            name='transit_node',
            output='screen',
            parameters=[
                params_file,
                # 注意：靠后的 dict 会静默覆盖前面 yaml 里的同名参数。
                # 这里是刻意让命令行 dry_run:= 覆盖 yaml，不是意外。
                {'dry_run': ParameterValue(dry_run, value_type=bool)},
            ],
        ),
    ])
