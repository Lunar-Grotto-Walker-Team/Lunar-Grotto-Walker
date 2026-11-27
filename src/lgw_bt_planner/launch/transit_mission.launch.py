"""TransitMission 精简 launch：只起 transit_node + bt_runner。

**不要直接用 `ros2 launch` 跑这个文件**——bt_runner 要 dlopen go2_transit 的插件
.so，而 go2_transit 装在 nav_ws 里，不在 .l4t_env.sh 的覆盖范围内。用包装脚本：

  bash /my_ws/l4t/lg_walker_ws/scripts/start_transit_mission.sh

它会在一个 shell 里 source /my_ws/l4t/.l4t_nav_env.sh（.l4t_env.sh + A/B/C/D 四个
导航工作区，9 ms），然后再跑本 launch。

参数：
  dry_run:=true   transit_node 只算路径不发动作，用来验证选点不动机器狗
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    transit_share = get_package_share_directory('go2_transit')
    transit_params = os.path.join(transit_share, 'config', 'cave_transit.yaml')

    dry_run = LaunchConfiguration('dry_run')
    action_name = LaunchConfiguration('action_name')
    groot2_port = LaunchConfiguration('groot2_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='只算路径不动机器狗（转发给 transit_node）',
        ),
        # 2026-11-27: 默认就叫 transit_bt_execution，不和机械臂链
        # （lgw_chain.launch.py / lgw_bt_planner.launch.py，默认 lgw_bt_execution）
        # 抢同一个动作名。两套 bt_runner 可以同时在线，send_goal 不会随机
        # 落到没有 go2_transit 插件的那个上。
        # 需要临时改回原名时仍可传 action_name:=lgw_bt_execution，但那种情况下
        # 机械臂链的 bt_runner 必须先停掉。
        DeclareLaunchArgument(
            'action_name',
            default_value='transit_bt_execution',
            description='BT 动作名；默认与机械臂链的 lgw_bt_execution 错开',
        ),
        # BT.CPP 的 Groot2Publisher 会 bind 1667/1668，端口被占时抛异常，
        # 而该构造在 TreeExecutionServer 的 try 里 —— 结果是每次 ExecuteTree
        # 都 abort "Address already in use"。机械臂链的 bt_runner 默认就用 1667，
        # 所以这里默认错开。
        DeclareLaunchArgument(
            'groot2_port',
            default_value='1669',
            description='Groot2 调试端口；与另一个 bt_runner 错开',
        ),

        Node(
            package='go2_transit',
            executable='transit_node',
            name='transit_node',
            output='screen',
            parameters=[
                transit_params,
                # 靠后的 dict 会静默覆盖 yaml 里的同名参数，这里是刻意的：
                # 重试只留树里那一层（RetryUntilSuccessful），避免 3×3 次往返。
                {'max_retries': 0},
                {'dry_run': ParameterValue(dry_run, value_type=bool)},
            ],
        ),

        Node(
            package='lgw_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            output='screen',
            parameters=[{
                # 两个插件库：本包的和 go2_transit 的（后者提供 TransitToLocalPoint）
                'plugins': [
                    'lgw_bt_planner/bt_plugins',
                    'go2_transit/bt_plugins',
                ],
                'behavior_trees': ['lgw_bt_planner/bt_trees'],
                'action_name': action_name,
                'groot2_port': ParameterValue(groot2_port, value_type=int),
                'tick_frequency': 10,
            }],
        ),
    ])
