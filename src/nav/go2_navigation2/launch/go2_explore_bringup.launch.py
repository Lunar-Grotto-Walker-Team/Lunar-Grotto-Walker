"""
go2_explore_bringup.launch.py
==============================
一键启动 go2 + SLAM + Nav2 + frontier_exploration 全套。

启动顺序 (实际是并行, 但 frontier_explorer 会等 /map / costmap 上来):
  1. slam_toolbox (在线建图, 发 /map)
  2. go2 本体 (驱动 + TF + 雷达 -> /scan + URDF)
  3. nav2 (planner / controller / costmap / bt_navigator, 由 nav2_bringup 自带 lifecycle_manager 拉起)
  4. frontier_explorer (默认 autostart:=false, 需手动 frontier_exploration_ctl start)
  5. RViz2 (可选, 默认开启)

使用:
  # 标准: 不带 RViz (远程或 headless)
  ros2 launch go2_navigation2 go2_explore_bringup.launch.py rviz:=false

  # 开启自动探索 (默认 false, 建议先 false, 手动触发更安全)
  ros2 launch go2_navigation2 go2_explore_bringup.launch.py autostart:=true

  # 配合 simulator 时间
  ros2 launch go2_navigation2 go2_explore_bringup.launch.py use_sim_time:=true

  # 替换 frontier 参数 (调试时方便)
  ros2 launch go2_navigation2 go2_explore_bringup.launch.py \
      frontier_params_file:=/abs/path/to/your.yaml

然后在另一终端:
  # 触发探索 (默认 autostart:=false 时)
  ros2 run frontier_exploration_ros2 frontier_exploration_ctl start

  # 调试观察器 (可选, 看 frontier 决策过程)
  ros2 launch frontier_exploration_ros2 frontier_debug.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction  # 用于 frontier_explorer 延迟启动, 等 slam_toolbox + nav2 起来
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_nav_pkg = get_package_share_directory("go2_navigation2")
    get_bringup_pkg = get_package_share_directory("nav2_bringup")

    # ---- 启动参数 ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) time when true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Start frontier_explorer 立即开始探索 (建议 false, 手动 frontier_exploration_ctl start)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 (true/false)'
    )

    frontier_params_arg = DeclareLaunchArgument(
        'frontier_params_file',
        default_value=os.path.join(go2_nav_pkg, 'config', 'frontier_go2_params.yaml'),
        description='frontier_explorer 参数 yaml 路径'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    frontier_params_file = LaunchConfiguration('frontier_params_file')

    # ============================================================
    # 1. SLAM Toolbox (在线建图, 发 /map)
    # ============================================================
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
        ),
        launch_arguments=[("use_sim_time", use_sim_time)]
    )

    # ============================================================
    # 2. Go2 本体 + Nav2 (planner/controller/costmap/bt) - 不含 AMCL/map_server
    # ============================================================
    # 2026-10-20: 显式透传 rviz:=false, 内层 launch 不会弹 RViz2,
    # 避免双开 OGL 上下文冲突 → 退出时 SIGSEGV. RViz 由外层独家的 rviz_arg 控制.
    go2_nav2_slam_explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_nav_pkg, "launch", "go2_nav2_slam_explore.launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("rviz", "false"),
            # map->odom 由本文件里的 slam_toolbox 发布;内层再发一条(静态或
            # 动态修正节点)就有两个广播者, tf2 会反复丢其中一条.
            # 2026-10-23: 内层默认已改成动态节点, 这里两个都要关掉.
            ("publish_map_to_odom", "false"),
            ("dynamic_map_to_odom", "false"),
        ]
    )

    # ============================================================
    # 3. Frontier Explorer (延迟 15s, 等 slam_toolbox + nav2 lifecycle 起来)
    # ============================================================
    # autostart 必须用 Python bool 字面量 False/True 强制覆盖 launch_ros 自动生成的
    # temp file (里面硬塞了 'autostart: true' 来自 C++ declare_parameter 默认值, 它在
    # 命令行里排第 2 个 --params-file, 会覆盖我们第 1 个 --params-file YAML 里的
    # 'autostart: false'). 用 LaunchConfiguration 会序列化成字符串, C++ 收到字符串
    # 后 C++ 默认值保持. 用 Python bool 字面量直接序列化成 YAML bool 才生效.
    #
    # 想打开探索: 把 False 改成 True 即可. 临时改 YAML 不会生效 (被 temp 覆盖),
    # 必须改这里或用 frontier_params_file:= 指向一份 autostart=true 的 YAML.
    frontier_explorer_node = Node(
        package='frontier_exploration_ros2',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            frontier_params_file,
            {
                'use_sim_time': use_sim_time,
                'autostart': False,  # ← Python bool 字面量, 强制覆盖 temp file 的 true;改成 False 后,需手动 frontier_exploration_ctl start 才探索
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 错峰: frontier_explorer 依赖 /map + /global_costmap/costmap 都已开始发布.
    # 实测时间: drivers 瞬时 + slam_toolbox 收几个 scan(~3s) + ekf 稳定(~5s) + nav2 7 个 lifecycle 转完(~10s)
    # 留 5s buffer, 15s 总延迟.
    frontier_explorer_delayed = TimerAction(
        period=15.0,
        actions=[frontier_explorer_node]
    )

    # ============================================================
    # 4. RViz2 (可选)
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_bringup_pkg, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        rviz_arg,
        frontier_params_arg,

        slam_toolbox,
        go2_nav2_slam_explore,
        frontier_explorer_delayed,
        rviz_node,
    ])