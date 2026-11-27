"""Launch unified_track_node with default YAML config.

可选 launch 参数(都是字符串,直接透传给同名 ROS 参数):
  trace_log_dir   跟踪日志目录;空 = 关闭逐 tick JSONL 日志
  trace_tag       日志文件名后缀(如 _crosscam)
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lgw_3d_tracker")
    yaml_path = os.path.join(pkg_share, "config", "runtime.yaml")

    trace_log_dir = LaunchConfiguration("trace_log_dir")
    trace_tag = LaunchConfiguration("trace_tag")

    return LaunchDescription([
        DeclareLaunchArgument("trace_log_dir", default_value=""),
        DeclareLaunchArgument("trace_tag", default_value=""),
        Node(
            package="lgw_3d_tracker",
            executable="unified_track_node",
            name="unified_track_node",
            output="screen",
            # The servo cannot drive without the registry this node owns:
            # its death must bring the launch down, not leave a silent stack.
            on_exit=[Shutdown(reason='unified_track_node exited')],
            parameters=[{
                "yaml_config": yaml_path,
                "trace_log_dir": trace_log_dir,
                "trace_tag": trace_tag,
            }],
        ),
    ])
