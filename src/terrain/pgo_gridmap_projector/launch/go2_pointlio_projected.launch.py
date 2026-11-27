# go2_pointlio_projected.launch.py
#
# 下游 elevation 链路,给 point_lio 上游用,但**不用** elevation_mapping:
# 用无状态的 projector 把 PGO 校正后的全图投成 GridMap。
#
#   simple_loop_closure (/aft_pgo_map, 1 Hz, PGO 校正后的全图)
#     -> pgo_gridmap_projector      -> /elevation_map_raw
#     -> elevation_fast_postprocessor -> /elevation_map_raw_post
#     -> elevation_traversability_bridge -> /terrain_traversability
#
# 为什么不用 elevation_mapping:
#   它对每个 cell 做时间上的 Kalman 融合,所以回环校正之前写进 cell 的旧值
#   还在,上游的校正传不进去 —— 重访过的地形会出现两份互相错开的副本(鬼影墙)。
#   projector 是无状态的,每一帧都重新栅格化,发布出去的图永远反映当前
#   PGO 解。
#
# 为什么吃 /aft_pgo_map 而不是 /global_pgo_map:
#   map_accumulator 的 voxel hash 是 first-write-wins 且从不 clear,回环之后
#   仍保留校正前的点。/aft_pgo_map 是 simple_loop_closure 用**当前**优化位姿
#   从全部 keyframe 重建的,所以是自洽的。详见 spec 4.4。
#
# 上游 pointlio_ws 另开一个终端启:
#
#   docker exec -w /my_ws/l4t/nav_ws/pointlio_ws self_l4t bash -c \
#     "source /my_ws/l4t/l4t_setup.sh && source install/setup.bash && \
#      ros2 launch simple_loop_closure full_stack_simple_lc_mid360s.launch.py \
#        use_livox_driver:=true show_rviz:=false"
#
# Pre-reqs:
#   - self_l4t 容器在跑。
#   - 上游 pointlio_ws 已启,且:
#       ros2 run tf2_ros tf2_echo map aft_mapped   # 必须出数
#   - grid_map_core / grid_map_ros / grid_map_msgs / grid_map_cv（apt 的
#     ros-humble-grid-map-*）+ 本仓库的 terrain 包已编过。

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

# 压掉父 launch 层的 INFO 噪音。这行在 generate_launch_description() 之前跑,
# 也在任何 Node fork 之前跑,所以子进程继承 WARN(除非自己再 --log-level 覆盖)。
os.environ.setdefault('RCUTILS_LOGGING_SEVERITY', 'WARN')


def generate_launch_description():
    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz',
        default_value='True',
        description='Launch rviz2 with go2_elevmap_fastlio.rviz (elevation-map view).',
    )
    with_bridge_arg = DeclareLaunchArgument(
        'with_bridge',
        default_value='True',
        description='Also start terrain_traversability_bridge (GridMap -> '
                    '/terrain_traversability occupancy grid). Point-lio stack '
                    'satisfies its required_frame_id=map already, so it needs no '
                    'per-upstream override.',
    )
    filter_delay_arg = DeclareLaunchArgument(
        'filter_delay',
        default_value='5.0',
        description='Seconds to wait before starting the filter chain. Only '
                    'guarantees the filter node has subscribed before the '
                    'NEXT /aft_pgo_map frame (upstream runs at 1 Hz), so the '
                    'first few maps are dropped rather than half-processed. '
                    'Not load-bearing: the projector keeps publishing at 1 Hz.',
    )
    slope_smooth_cells_arg = DeclareLaunchArgument(
        'slope_smooth_cells',
        default_value='1',
        description='Box-mean the inpainted elevation with 2N+1 cells before '
                    'computing the normals. 1 = off (slope from the raw '
                    'surface). 3 turns a 1-cell step (a crest occlusion shadow '
                    'bridged by the nearest-neighbour fill) into a ramp while '
                    'keeping multi-cell walls steep.',
    )
    slope_fit_cells_arg = DeclareLaunchArgument(
        'slope_fit_cells',
        default_value='1',
        description='Fit a plane over (2k+1)^2 cells instead of a two-cell '
                    'gradient. Default 1 = 0.3 m window: a single-cell step '
                    'no longer sets the slope, and the window still fits '
                    'inside a 1.5 m corridor. 0 = two-cell gradient (old '
                    'behaviour); 3-5 averages over 0.7-1.1 m (too wide for a '
                    'narrow cave).',
    )
    publish_debug_layers_arg = DeclareLaunchArgument(
        'publish_debug_layers',
        default_value='false',
        description='Also publish elevation_smooth and normal_vectors_* on '
                    '/elevation_map_raw_post. Only the fast postprocessor has '
                    'this switch; RViz needs them, the bridge does not.',
    )

    with_utlidar_fill_arg = DeclareLaunchArgument(
        'with_utlidar_fill',
        default_value='true',
        description='Start utlidar_terrain_fill and let the projector merge '
                    'its ground cloud only into xy cells with no Livox point. '
                    'Set false to restore the pre-fill behavior.',
    )
    fill_topic_arg = DeclareLaunchArgument(
        'fill_topic',
        default_value='/utlidar_terrain_fill',
        description='Ground-only fill cloud (map frame, 1 Hz) from the uLidar.',
    )

    proj_share = get_package_share_directory('pgo_gridmap_projector')
    projector_params_arg = DeclareLaunchArgument(
        'projector_params_file',
        default_value=os.path.join(
            proj_share, 'config', 'projector_params.yaml'),
        description='Projector params file. Override it to run an A/B with a '
                    'different ground_mode/profile without editing the '
                    'installed YAML.',
    )

    # 无状态投影器:订阅 /aft_pgo_map,栅格化成单层 GridMap 发到
    # /elevation_map_raw。
    projector_node = Node(
        package='pgo_gridmap_projector',
        executable='projector_node',
        name='pgo_gridmap_projector',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[
            LaunchConfiguration('projector_params_file'),
            {
                'fill_enabled': ParameterValue(
                    LaunchConfiguration('with_utlidar_fill'),
                    value_type=bool),
                'fill_topic': LaunchConfiguration('fill_topic'),
            },
        ],
    )

    # uLidar 近场地面填充。节点自己在 map<-odom 拿不到时什么都不发,
    # projector 就回到原行为(不会因为 fill 挂掉而整条链失效)。
    fill_share = get_package_share_directory('utlidar_terrain_fill')
    fill_node = Node(
        package='utlidar_terrain_fill',
        executable='utlidar_terrain_fill',
        name='utlidar_terrain_fill',
        output='screen',
        parameters=[os.path.join(
            fill_share, 'config', 'utlidar_fill_params.yaml')],
        respawn=True,
        condition=IfCondition(LaunchConfiguration('with_utlidar_fill')),
    )

    # 2026-11-04: O(N) 后处理节点,替换旧的 grid_map_filters_demo 滤波链。
    # 话题名不变 (/elevation_map_raw -> /elevation_map_raw_post), 所以
    # bridge 零改动。
    #
    # 2026-11-26: 旧的 grid_map_filters_demo 路径已删除 —— 它依赖
    # grid_map_demos，而 apt 没有这个包、源码目录又被 COLCON_IGNORE 掉了，
    # 这条回退路径实际跑不起来。本节点是它的完全替代。
    fast_postprocessor_node = Node(
        package='elevation_fast_postprocessor',
        executable='postprocessor_node',
        name='elevation_fast_postprocessor',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{
            'input_topic': '/elevation_map_raw',
            'output_topic': '/elevation_map_raw_post',
            'publish_debug_layers': ParameterValue(
                LaunchConfiguration('publish_debug_layers'), value_type=bool),
            'slope_smooth_cells': ParameterValue(
                LaunchConfiguration('slope_smooth_cells'), value_type=int),
            'slope_fit_cells': ParameterValue(
                LaunchConfiguration('slope_fit_cells'), value_type=int),
        }],
    )

    # 后处理晚起一点,保证它已经订阅上,不至于把前几帧整个丢掉。
    fast_postprocessor_delayed = TimerAction(
        period=LaunchConfiguration('filter_delay'),
        actions=[fast_postprocessor_node],
    )

    # bridge.launch.py 只接一个 params_file 参数,默认值就是它自己的
    # config/bridge_params.yaml(required_frame_id=map、target_resolution=0.05、
    # 阈值)。point_lio 这一套满足 required_frame_id=map,不需要覆盖。
    #
    # bridge_node.py 是纯被动的:没有 timer,在第一帧 /elevation_map_raw_post
    # 之前就静静地在 spin() 里等着。所以它不需要 TimerAction。
    bridge_launch_path = os.path.join(
        get_package_share_directory('elevation_traversability_bridge'),
        'launch', 'bridge.launch.py',
    )
    bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_path),
        condition=IfCondition(LaunchConfiguration('with_bridge')),
    )

    # rviz 用的还是 elevation_mapping 那份配置(同一个 GridMap 插件、同一批
    # layer 名)。主 display 已经改指向 /elevation_map_raw_post。
    #
    # 两份配置,按 ground_mode 选:
    #   go2_elevmap_fastlio.rviz       band-wide(默认,只改过 topic)-> 原始配色
    #   go2_elevmap_fastlio_cave.rviz  reference + profile=cave -> 按 slope 上色
    # 后者在洞穴里把"墙/坡"的形状显出来,band-wide 里 slope 是同一套数据但
    # 原始配置只画 elevation,不需要换。
    em_share = get_package_share_directory('elevation_mapping')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(em_share, 'rviz2',
                                   'go2_elevmap_fastlio.rviz'),
        description='RViz config for the elevation view. Default is the '
                    'band-wide config; pass '
                    '<em_share>/rviz2/go2_elevmap_fastlio_cave.rviz with '
                    'ground_mode=reference + profile=cave.',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_rviz')),
    )

    return LaunchDescription([
        show_rviz_arg,
        with_bridge_arg,
        filter_delay_arg,
        publish_debug_layers_arg,
        slope_smooth_cells_arg,
        slope_fit_cells_arg,
        with_utlidar_fill_arg,
        fill_topic_arg,
        projector_params_arg,
        rviz_config_arg,
        projector_node,
        fill_node,
        fast_postprocessor_delayed,
        bridge_include,
        rviz_node,
    ])
