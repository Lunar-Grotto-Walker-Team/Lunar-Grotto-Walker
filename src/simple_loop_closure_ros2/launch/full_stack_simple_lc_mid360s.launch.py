"""full_stack_simple_lc_mid360s.launch.py

一键启 Go2 + Livox Mid360S 完整建图栈(后端:simple_loop_closure):

  livox_ros_driver2 (msg_MID360s_launch.py)
    -> point_lio (mapping_mid360.launch.py, mid360.yaml, CustomMsg 路径)
    -> simple_loop_closure_node (带话题重映射)
    -> aft_pgo_save_server (Trigger -> /save_req)
    -> 顶层 RViz (单一窗口, 同时显示 LIO + PGO 校正结果)

约定:
  * 顶层声明的参数带 `lio_` 前缀, 避免 IncludeLaunchDescription
    跨作用域 leak 覆盖子 launch 的同名参数。
  * point_lio / simple_loop_closure 各自的 RViz 都关掉, 顶层统一管理。
  * 静态 TF livox_frame -> camera_init 给 Point-LIO 起手位姿。
  * slc_params_file 默认 simple_loop_closure_cave_proxy.yaml（cave-proxy,
    不开腿运动学 z）；leg-z A/B 用 simple_loop_closure_cave_legz.yaml；
    平地走廊/花园显式传 simple_loop_closure.yaml（Go2 相对 z）。

Topic 重映射(在 simple_loop_closure 节点上):
  /odometry       <- /aft_mapped_to_init
  /cloud          <- /cloud_registered
  /pgo_map_cloud  -> /aft_pgo_map
  /pgo_odom       -> /aft_pgo_odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_livox_driver = LaunchConfiguration('use_livox_driver')
    rviz = LaunchConfiguration('show_rviz')
    slc_params = LaunchConfiguration('slc_params_file')
    acc_params = LaunchConfiguration('acc_params_file')
    save_dir = LaunchConfiguration('pgo_save_directory')

    # ---- 1) Livox driver ----
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2', 'msg_MID360s_launch.py',
            ])
        ),
        condition=IfCondition(use_livox_driver),
    )

    # ---- 2) Point-LIO ----
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'launch', 'mapping_mid360.launch.py',
            ])
        ),
        launch_arguments=[
            ('rviz', 'false'),
            # 2026-11-19: 透传给 point_lio 的建图门控, 直到 /start_mapping.
            ('wait_for_start', LaunchConfiguration('wait_for_start')),
        ],
    )

    # ---- 3) SimpleLoopClosure node (带重映射) ----
    slc_node = Node(
        package='simple_loop_closure',
        executable='simple_loop_closure_node',
        name='simple_loop_closure',
        parameters=[slc_params],
        remappings=[
            # 入站
            ('/odometry', '/aft_mapped_to_init'),
            ('/cloud',    '/cloud_registered'),
            # 出站 (回到 /aft_pgo_* 命名约定)
            ('/pgo_map_cloud', '/aft_pgo_map'),
            ('/pgo_odom',      '/aft_pgo_odom'),
            # /vis_pose_graph / /save_req 保持原名
        ],
        output='screen',
    )

    # ---- 4) save wrapper service ----
    # save_node = Node(
    #     package='point_lio_loop_save',
    #     executable='aft_pgo_save_server',
    #     name='aft_pgo_save_server',
    #     parameters=[{'save_directory': save_dir}],
    #     output='screen',
    # )
    # NOTE: aft_pgo_save_server 是计划中的 wrapper,把 service /save_map 翻译成
    # simple_loop_closure 的 /save_req topic。包还没实现,直接用 topic 触发:
    #   ros2 topic pub --once /save_req std_msgs/msg/String "{data: /root/lio_maps/data/}"

    # ---- 4.5) map_accumulator —— 默认不启动 (2026-11-06) ----
    # 本 fork 的 /aft_pgo_map 已经是"用当前 PGO 位姿从全部关键帧重建、并按
    # 0.05 m 体素去重"的整图, accumulator 再做一遍 0.05 m 去重没有任何新增;
    # 而它每秒要把整图重新序列化一遍, 是 /aft_pgo_map 的 RELIABLE 慢消费者:
    #   2.9 GB corridor bag 实测 (scripts/large): 带它时 19.5 MB/帧 -> 0.51 Hz,
    #   去掉后 20.5 MB/帧 -> 0.91~1.00 Hz; A 组 RSS 峰值 908 -> 416 MB。
    # 另外它的 voxel hash 只增不减, 回环后残留旧位姿点, 下游本来就不用
    # /global_pgo_map (见 pgo_gridmap_projector 的 launch 注释)。
    # 需要 /global_pgo_map 时再单独起:
    #   ros2 run map_accumulator map_accumulator_node \
    #     --ros-args --params-file <share>/map_accumulator/config/accumulator.yaml
    # acc_node = Node(
    #     package='map_accumulator',
    #     executable='map_accumulator_node',
    #     name='map_accumulator',
    #     parameters=[acc_params],
    #     output='screen',
    # )

    # ---- 5) 顶层 RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_full_stack',
        arguments=[
            '-d', LaunchConfiguration('rviz_cfg'),
            '--ros-args', '--log-level', 'WARN',
        ],
        condition=IfCondition(rviz),
        output='screen',
        prefix='nice',
    )

    # ---- 6) 静态 TF: livox_frame -> camera_init ----
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_camera_init',
        arguments=[
            '--x', '0.20',
            '--y', '0.00',
            '--z', '0.15',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '0.0',
            '--frame-id', 'camera_init',
            '--child-frame-id', 'livox_frame',
        ],
        condition=IfCondition(use_livox_driver),
        output='screen',
    )

    # ---- 7) 静态 TF: map -> camera_init (identity 别名) ----
    # 给下游 elevation_mapping / frontier_exploration 等用。它们默认 map_frame_id=map,
    # 整栈里没有这个帧会让 lookupTransform 全部 TF2_TIMEOUT。
    # point_lio 的 mid360.yaml 已开 gravity_align=true,所以 camera_init 本身已在重力系下,
    # camera_init -> map 是纯 identity -- 只是把世界帧重命名成下游约定的 /map。
    # 不与 livox_to_camera_init 共生命周期:跑 bag 时 driver 关掉、livox_frame TF 没了,
    # 但 /map 仍然要稳定存在。
    map_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_map',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'camera_init',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_livox_driver', default_value='true',
            description='是否启动 livox_ros_driver2 的 Mid360S 节点;跑 bag 时改为 false'),
        DeclareLaunchArgument(
            'wait_for_start', default_value='false',
            description='point_lio 丢弃 lidar/IMU 直到调用 /start_mapping'
                        '(让 LIO 原点落在摆好的基地原点)'),
        DeclareLaunchArgument(
            'show_rviz', default_value='true',
            description='顶层 RViz 是否打开(改名以避免与子 launch 的 rviz 参数冲突)'),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=PathJoinSubstitution([
                FindPackageShare('simple_loop_closure'),
                'rviz_cfg', 'full_stack_simple_lc_mid360s.rviz',
            ]),
            description='顶层 RViz 配置文件路径'),
        DeclareLaunchArgument(
            'slc_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('simple_loop_closure'),
                'config', 'simple_loop_closure_cave_proxy.yaml',
            ]),
            description='simple_loop_closure 节点参数 yaml 路径'),
        DeclareLaunchArgument(
            'acc_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('map_accumulator'),
                'config', 'accumulator.yaml',
            ]),
            description='map_accumulator 节点参数 yaml 路径'),
        DeclareLaunchArgument(
            'pgo_save_directory',
            default_value=[EnvironmentVariable('HOME'), '/lio_maps/data/'],
            description='/aft_pgo_save_map 落盘根目录(末尾 / 可省略)'),

        livox_launch,
        static_tf_node,
        map_static_tf_node,
        point_lio_launch,
        slc_node,
        # acc_node,  # disabled 2026-11-06: 下游不消费 /global_pgo_map, 见上方说明
        # save_node,  # disabled: package 'point_lio_loop_save' not implemented
        rviz_node,
    ])
