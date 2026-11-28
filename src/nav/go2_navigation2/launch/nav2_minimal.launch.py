"""
nav2_minimal.launch.py
======================
go2_nav2_slam_explore 专用精简 nav2 启动 (2026-10-23)。

与 nav2_bringup/navigation_launch.py 的差异
-------------------------------------------
去掉(在本栈里从不被调用):
  - smoother_server    : Humble 默认 BT
                         navigate_to_pose_w_replanning_and_recovery.xml 里
                         只有 ComputePathToPose / FollowPath / Spin / BackUp /
                         Wait / ClearEntireCostmap, 没有 SmoothPath。
  - waypoint_follower  : 默认 BT 没有 FollowWaypoints; frontier_explorer 只用
                         NavigateToPose action。
保留(必用):
  - controller_server   : DWB, BT 的 FollowPath
  - planner_server      : Navfn, BT 的 ComputePathToPose/ThroughPoses
  - behavior_server     : BT 恢复分支的 Spin/BackUp/Wait
  - bt_navigator        : 导航入口(frontier 也走它)
  - velocity_smoother   : controller -> cmd_vel_nav -> cmd_vel 链上
  - lifecycle_manager   : 生命周期(autostart)

启动顺序
--------
2026-11-06: **去掉逐批启动**, 全部节点一次性拉起。

分批是 FAST-LIO 时代为了压制启动瞬间的 CPU 尖峰: 同时拉 8 个进程 +
全局 costmap 一次分配几十 MB 会把 8 核打满, livox_ros_driver2 的 IMU 发布
线程被饿死 >1s 会触发 FAST-LIO laserMapping.cpp:333 的一次性 self-sync
误判(IMU 时间戳整体平移 1.15s), 之后 LIO 位姿漂移。

现在上游换成 point_lio (逐帧处理, 不依赖跨帧累积的 IMU 时间戳连续性),
一次性启动没有这个风险; 节点顺序由 lifecycle_manager 的 configure/activate
串行保证, 与 nav2_bringup 上游做法一致。

`start_delay` (默认 3s) 只用于给同一个 launch 里的 go2_driver / cloud relay
留出发 TF 与首帧点云的时间, 不再分批。

注意: lifecycle_nodes 必须和实际启动的节点完全一致 —— 多一个 manager 会一直
等不存在的服务(卡死在 bringup), 少一个那个节点会永远停在 unconfigured。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    start_delay = LaunchConfiguration('start_delay')

    # 与 nav2_bringup/navigation_launch.py 保持一致(去掉 smoother/waypoint)
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('go2_navigation2'), 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # 2026-11-18: 地形链专用恢复树（不清 global costmap，顶层 3 次恢复）。
    # 传空字符串 bt_xml:= 可回退到 nav2_bt_navigator 包内默认树。
    bt_xml_default = os.path.join(
        get_package_share_directory('go2_navigation2'),
        'behavior_trees', 'navigate_to_pose_terrain_recovery.xml')
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml', default_value=bt_xml_default,
        description='Path to the NavigateToPose BT XML; empty string falls '
                    'back to the Nav2 package default')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level')

    declare_start_delay_cmd = DeclareLaunchArgument(
        'start_delay', default_value='3.0',
        description='Seconds to wait before starting the nav2 stack '
                    '(give go2_driver / cloud relay time to publish TF and '
                    'the first cloud frames)')

    def make_node(package, executable, name, extra_remappings=None,
                  extra_parameters=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            # extra_parameters is appended per node so that keys such as
            # default_nav_to_pose_bt_xml never reach nodes that don't declare
            # them (rclcpp rejects undeclared parameters at startup).
            parameters=[configured_params] + (extra_parameters or []),
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + (extra_remappings or []))

    controller_server = make_node(
        'nav2_controller', 'controller_server', 'controller_server',
        [('cmd_vel', 'cmd_vel_nav')])
    planner_server = make_node(
        'nav2_planner', 'planner_server', 'planner_server')
    behavior_server = make_node(
        'nav2_behaviors', 'behavior_server', 'behavior_server')
    bt_navigator = make_node(
        'nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
        extra_parameters=[
            {'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml')}
        ])
    velocity_smoother = make_node(
        'nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    nav2_stack = GroupAction(actions=[
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_bt_xml_cmd,
        declare_log_level_cmd,
        declare_start_delay_cmd,
        TimerAction(period=start_delay, actions=[nav2_stack]),
    ])
