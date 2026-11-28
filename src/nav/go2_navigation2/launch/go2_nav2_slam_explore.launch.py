"""
go2_nav2_slam_explore.launch.py
================================
基于 go2_nav2.launch.py 改造, 用于 frontier_exploration 的 SLAM 模式。

差异 (相对于 go2_nav2.launch.py):
  - 移除 map_server 节点 (slam_toolbox 会发 /map, 不需要静态地图加载)
  - 移除 amcl 节点 (在线建图不需要定位)
  - 移除手动 lifecycle_manager (原版只管 map_server+amcl; nav2 lifecycle 由 nav2_bringup 自带 manager 唯一管理)
  - 移除 declare_map / map 启动参数
  - 不再向 nav2_launch 传 map: 启动参数

保留:
  - twist_bridge / go2_driver (狗本体驱动)
  - cloud_to_base (pointlio_cloud_relay: /cloud_registered -> /cloud_base)
  - go2_display_launch (robot_description 发布)
  - nav2_launch (精简版 nav2_minimal.launch.py: planner/controller/
    behavior/bt_navigator/velocity_smoother, 无 smoother_server / waypoint_follower)
  - map_odom_correction (动态 map->odom, 与 point_lio PGO 位姿同源)
  - RViz2

2026-10-30 去掉:
  - 雷达点云 -> /scan 的 include (go2_perception 的单线激光转换):
    不再需要单线激光, local_costmap 障碍层直接吃点云。
  - 地形可通行性 bridge 节点: 桥接移出本 launch, 由上游 terrain 链
    单独提供 /terrain_traversability。

2026-10-31 local 坡地修复:
  local_costmap 的 obstacle_layer 原本吃 /cloud_base, 按 odom 里的绝对
  高度过滤; 缓坡在 1 m 外就会超过 min_obstacle_height 被整片标成 lethal。
  现在改由 pointlio_cloud_relay/ground_filter 按"离局部地面的高度"把点云
  拆成 /cloud_ground (marking:false, clearing:true) 和 /cloud_obstacles
  (marking:true), obstacle_layer 以两个 observation source 消费。
  /cloud_base 保留, 仅作调试/检查用途。

/terrain_traversability 的归属 (2026-10-30):
  由 elevmap_fastlio 工作区的
    pgo_gridmap_projector/launch/go2_pointlio_projected.launch.py
  独占发布 (projector -> filters -> elevation_traversability_bridge,
  其中 with_bridge 默认为 true)。本 launch 绝不能再起第二个 bridge,
  否则同一个话题会有两份 (参数可能还不一致), Nav2 随机收到哪份不可控。
  运行拓扑见 user-log.md 的「point_lio 地形 costmap 运行手册」。

2026-10-23 去掉:
  - lowstate_to_imu + go2_robot_localization(EKF):
    EKF publish_tf=false, /odometry/filtered 全工程无订阅者, /imu 只喂它,
    纯空转(EKF 100Hz). 备注在下方定义处, 需要时可原样加回.

新增:
  - (无) 此处不再加 lifecycle_manager, nav2 全栈生命周期完全交给 nav2_bringup 自带的
    lifecycle_manager_navigation 唯一管理. 加第二个 manager 会互抢 bond, 导致
    CRITICAL FAILURE 把所有节点 shutdown.
  - map -> odom 边 (2026-10-23 改):
      默认由 go2_driver/map_odom_correction 动态发布
        T_map_odom = T_map_body(FAST-LIO) * T_body_base * T_odom_base(driver)^-1
      让 nav2 全局位姿与 elevation_mapping 同源. 旧的静态 map_to_odom_tf 保留但
      默认关闭(dynamic_map_to_odom:=false publish_map_to_odom:=true 可回退).
      原因: 静态单位边要求"狗内部 odom 原点 == 高程图 map 原点", 实机不成立,
      而且两套里程计会各自漂移, 狗会越跑越偏离地图.
      前提: Terminal A(FAST-LIO+elevation_mapping) 必须在跑.
      配 go2_explore_bringup 时两个都要 false, 由 slam_toolbox 负责这条边.

使用:
  ros2 launch go2_navigation2 go2_nav2_slam_explore.launch.py
  # 需要 Terminal A (point_lio) / Terminal B (elevmap 地形链) 先跑,
  # 见 user-log.md §9。

本分支不支持 go2_explore_bringup.launch.py:
  它 include 本 launch, 但内部 slam_toolbox 需要 /scan (本分支已删除该
  话题), 而且它拿不到地形 bridge 的 /terrain_traversability; global
  costmap 只有 terrain 层且 allow_unknown: false, Navfn 会因没有可规划
  空间而失败。前沿探索是本次 spec 的非目标, 不要尝试修通它。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    get_nav2_pkg = get_package_share_directory("go2_navigation2")
    get_bringup_pkg = get_package_share_directory("nav2_bringup")
    go2_description_pkg = get_package_share_directory("go2_description")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_driver_pkg = get_package_share_directory("go2_driver")

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_param_path = LaunchConfiguration(
        'params_file', default=os.path.join(get_nav2_pkg, 'config', 'nav2_params.yaml'))
    # 2026-10-20: standalone 跑 go2_nav2_slam_explore.launch.py (无外层
    # go2_explore_bringup) 时也需要弹 RViz2 看 costmap,加 rviz 启动参数,
    # 默认 true. 配 go2_explore_bringup 一起走时,务必在外层 launch
    # arguments 里把 rviz 显式设为 false,避免双开 OGL 冲突 → 退出时 SIGSEGV.
    # 2026-10-29: 改用本包自己的 rviz 配置(从 nav2_bringup 拷来), 把
    # Global Costmap 的 Alpha 0.3 -> 0.6: 0.3 时整张图太透, unknown 区域
    # 在深色背景上几乎看不清, 和 Local(0.7) 对比也很突兀.
    # 原文件: /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
    rviz_config_dir = os.path.join(get_nav2_pkg, 'rviz', 'go2_nav2_default_view.rviz')
    rviz = LaunchConfiguration('rviz', default='true')

    # 2026-11-18: L2 近场避障开关。L2 只补 Livox 的盲区(约 1.07 m 以内),
    # 出问题时可用 use_utlidar_l2:=false 一键退回只有 Livox 的行为。
    use_utlidar_l2 = LaunchConfiguration('use_utlidar_l2', default='true')

    # ---- map -> odom: 动态修正(默认) / 静态兜底(可选) ----
    # 本 launch 的 TF 图有两条互不相连的链(2026-10-22 实测 view_frames 确认):
    #   树 1 (上游 LIO/elevation_mapping 提供的世界系):
    #       camera_init -[fast_lio 动态]-> body
    #       camera_init -[mapping.launch.py 静态, 重力对齐]-> map
    #   树 2 (狗本体里程计):
    #       odom -[driver.cpp, 来自 /utlidar/robot_pose]-> base_footprint
    #       (2026-11-16: 不再发布 base_footprint -> base_link. base_link 的
    #        parent 归机械臂栈, 导航侧只用 base_footprint.)
    # EKF 的 publish_tf 是 false, 不会补 odom -> base_link.
    # 而 global_costmap.global_frame_id = map, robot_base_frame = base_footprint
    # → 两棵树必须由一条 map -> odom 接起来, 否则 global costmap 一次都不 update.
    #
    # 2026-10-23: 原来发的是一条"单位变换"静态边, 语义是"狗内部 odom 的原点与
    # 高程图 map 的原点重合". 实机证明它不成立 —— 狗站着不动时
    # /utlidar/robot_pose 已经是 (-26.90, -18.87) (狗内部里程计原点早就不在
    # 狗当前位置), nav2 于是把狗摆到 50m×50m 高程图原点 33m 之外, 每秒刷
    #   "Robot is out of bounds of the costmap!"
    #   "Sensor origin at (-27.21, -18.92) is out of map bounds (-25.00, ... )"
    # 而且就算起点恰好重合, 狗内部里程计和 FAST-LIO 是两套独立里程计, 跑起来
    # 也会各自漂移, 狗会越来越偏离地图.
    #
    # 现在默认由 go2_driver/map_odom_correction 以 20Hz 动态发布:
    #   T_map_odom = T_map_body(FAST-LIO) * T_body_base(安装) * T_odom_base(driver)^-1
    # 使 map -> odom -> base_footprint 恒等于 map -> camera_init -> body -> base,
    # 即 nav2 全局位姿与 elevation_mapping 完全同源; local_costmap 仍用 odom 系.
    #
    # 前提(重要): Terminal A(FAST-LIO+elevation_mapping) 必须先起. 否则
    # map -> body 取不到, 本节点只打 WARN 不发布, global_costmap 会因为拿不到
    # map -> base_footprint 而激活失败.
    #
    # 三种互斥组合:
    #   1) 默认: dynamic_map_to_odom:=true, publish_map_to_odom:=false  ← 本管线
    #   2) 无 FAST-LIO/老行为: dynamic_map_to_odom:=false
    #                          publish_map_to_odom:=true (可用 map_to_odom_* 手调)
    #   3) 外部 slam_toolbox 发 map->odom (go2_explore_bringup): 两个都 false
    dynamic_map_to_odom = LaunchConfiguration('dynamic_map_to_odom', default='true')
    # 2026-10-30: 外参 body_to_base_* 统一放到 config/pointlio_frames.yaml,
    # 与 pointlio_cloud_relay/cloud_to_base 共用同一份真值。
    # 语义是 T^base_footprint_body (body -> base), 节点内部按需取逆。
    map_odom_correction = Node(
        package='go2_driver',
        executable='map_odom_correction',
        name='map_odom_correction',
        output='screen',
        parameters=[os.path.join(
            get_nav2_pkg, 'config', 'pointlio_frames.yaml')],
        condition=IfCondition(dynamic_map_to_odom),
    )

    build_map_to_odom = LaunchConfiguration('publish_map_to_odom', default='false')
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x',     LaunchConfiguration('map_to_odom_x', default='0.0'),
            '--y',     LaunchConfiguration('map_to_odom_y', default='0.0'),
            '--z',     LaunchConfiguration('map_to_odom_z', default='0.0'),
            '--roll',  LaunchConfiguration('map_to_odom_roll', default='0.0'),
            '--pitch', LaunchConfiguration('map_to_odom_pitch', default='0.0'),
            '--yaw',   LaunchConfiguration('map_to_odom_yaw', default='0.0'),
            '--frame-id',       'map',
            '--child-frame-id', 'odom',
        ],
        condition=IfCondition(build_map_to_odom),
    )

    # ---- nav2_bringup (planner/controller/bt/costmap/behavior_server 等) ----
    # 注意: 不再传 map: 启动参数, 因为不再用 map_server
    #
    # 重要: 显式传 autostart:=true 给 navigation_launch.py.
    # 外层 go2_explore_bringup.launch.py 自己也声明了 'autostart' (默认 false,
    # 用于控制 frontier_explorer 是否立刻开始探索). 如果不在这里再显式传一次,
    # ROS 2 launch 的作用域规则会让外层 autostart:=false 漏到 nav2_bringup 的
    # lifecycle_manager_navigation.autostart 参数上, 导致 nav2 全栈 7 个节点
    # (planner / controller / bt_navigator / smoother / velocity_smoother /
    # behavior / waypoint_follower + costmap) 永远卡在 unconfigured [1].
    # 这里强制覆盖, 让 lifecycle_manager 始终 true, 而 frontier_explorer 的
    # autostart 仍由外层控制, 互不干扰.
    # 2026-10-23: 改用本包的精简版 nav2_minimal.launch.py:
    #   - 去掉 smoother_server / waypoint_follower (默认 BT 从不调用)
    # 2026-11-06: 上游换成 point_lio 后去掉逐批启动 (逐帧处理, 不受启动
    #   CPU 尖峰造成的 IMU 时间戳 self-sync 影响); 只保留 3s start_delay
    #   给 go2_driver / cloud relay 发 TF 和首帧点云.
    # 需要回退到上游全量节点时, 把下面这个 include 换回
    # nav2_bringup/launch/navigation_launch.py 即可.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_nav2_pkg, "launch", "nav2_minimal.launch.py")),
        launch_arguments=[
            ("params_file", nav2_param_path),
            ("use_sim_time", use_sim_time),
            ("autostart", "true"),
        ]
    )

    # 2026-10-23: 去掉 EKF + lowstate_to_imu.
    # EKF(go2_robot_localization) publish_tf=false, 全工程没有任何节点订阅
    # /odometry/filtered; /imu 的唯一消费者就是它. 两个节点纯空转(EKF 还是
    # 100Hz), 留着只会和 FAST-LIO/livox 抢核. 需要时把下面几行加回来:
    #   IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
    #       go2_core_pkg, "launch", "go2_robot_localization.launch.py")))
    #   + lowstate_to_imu Node

    # 注: 内层 RViz2 已在 2026-10-20 恢复 (standalone 跑时要弹窗口看 costmap).
    # 配 go2_explore_bringup 一起走时务必把 rviz:=false 透传过来,避免双开.

    # ---- 与 driver.launch 一致: cmd_vel -> Unitree API, TF/关节/IMU ----
    twist_bridge = Node(
        package='go2_twist_bridge',
        executable='twist_bridge',
    )
    # 2026-11-16: 不再发布 base_footprint -> base_link。
    # base_link 的 parent 归机械臂栈 (odom -> base_link, = uLidar 原点),
    # 导航侧只用 base_footprint。见 docs/superpowers/specs/
    # 2026-11-16-terrain-nav-realstart-design.md §6。
    go2_driver = Node(
        package='go2_driver',
        executable='driver',
        parameters=[os.path.join(go2_driver_pkg, 'params', 'driver.yaml')],
    )

    # ---- 点云中继: /cloud_registered (aft_mapped) -> /cloud_base (base_footprint) ----
    # local_costmap 的障碍层直接吃这个点云, 不再从 /scan 建 voxel 层。
    # 外参和 map_odom_correction 共用 pointlio_frames.yaml, 单一真值来源。
    cloud_to_base = Node(
        package='pointlio_cloud_relay',
        executable='cloud_to_base',
        name='cloud_to_base',
        output='screen',
        parameters=[os.path.join(
            get_nav2_pkg, 'config', 'pointlio_frames.yaml')],
    )
    # ---- 地形相对高度的地面过滤 (2026-10-31) ----
    # local_costmap 的 obstacle_layer 按 odom 里的绝对高度过滤点云, 缓坡在
    # 1 m 外就会超过 min_obstacle_height 被当成障碍。ground_filter 在重力
    # 对齐的 camera_init 下按"离局部地面的高度"把点云拆成两路:
    #   /cloud_ground    -> obstacle_layer 的 ground source (只清不标)
    #   /cloud_obstacles -> obstacle_layer 的 obstacles source (正常 marking)
    # 参数与 cloud_to_base 共用 pointlio_frames.yaml (ground_filter 段)。
    ground_filter = Node(
        package='pointlio_cloud_relay',
        executable='ground_filter',
        name='ground_filter',
        output='screen',
        parameters=[os.path.join(
            get_nav2_pkg, 'config', 'pointlio_frames.yaml')],
    )
    # 2026-11-18: 同一个 ground_filter 的第二个实例, 处理 Unitree L2。
    # /utlidar/cloud_deskewed 已在 odom(本地图 global_frame), 实测已去畸变、
    # 已分割自身点, 所以不需要外参也不用自滤盒; 仍然要这一层是因为
    # ground_filter 的"离局部地面高度"拆分能避免坡地被绝对 z 判成障碍,
    # 且它把输出改到 base_footprint, 让 nav2 的 raytrace 从机器人(而不是
    # odom 原点)射出。参数见 pointlio_frames.yaml 的 ground_filter_l2 段。
    ground_filter_l2 = Node(
        package='pointlio_cloud_relay',
        executable='ground_filter',
        name='ground_filter_l2',
        output='screen',
        condition=IfCondition(use_utlidar_l2),
        parameters=[os.path.join(
            get_nav2_pkg, 'config', 'pointlio_frames.yaml')],
    )
    # 2026-11-04: /terrain_traversability (0/85/128/254, 0.1 m) ->
    # /exploration_map (0/-1/100, 0.2 m) 给 frontier_exploration_ros2 用。
    # 转换是必须的: 85(未知) 在 frontier 眼里是占据, 而 128/254 在 int8
    # 下是负数会被当成未知 —— 直接喂过去语义完全反了。
    exploration_map_relay = Node(
        package='pointlio_cloud_relay',
        executable='exploration_map_relay',
        name='exploration_map_relay',
        output='screen',
        parameters=[{'input_topic': '/terrain_traversability',
                     'output_topic': '/exploration_map',
                     'output_resolution': 0.2}],
    )
    # ---- robot_description (URDF 可视化) ----
    go2_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_description_pkg, "launch", "display.launch.py")
        ),
        launch_arguments=[("use_joint_state_publisher", "false")]
    )

    # ---- 不在这里再加 lifecycle_manager ----
    # nav2_bringup 的 navigation_launch.py 已经启动了 lifecycle_manager_navigation,
    # 它会把 controller_server / planner_server / bt_navigator / smoother_server /
    # velocity_smoother / behavior_server / waypoint_follower / costmap 等全部 activate.
    # 之前我们手动加过一个 explicit manager, 想"保险"激活节点,
    # 结果两个 manager 抢同一组节点的 bond —— bond 同时只能属于一个 manager,
    # 输的那一边 20s 等不到心跳就会触发 CRITICAL FAILURE, 把所有节点 shutdown 清理掉.
    # 所以这里只让 nav2_bringup 的 manager 独家管理.
    # 如果将来 manager_navigation 没拉起某个节点, 不要在这里加新 manager,
    # 应该:
    #   (1) 改 nav2_bringup/launch/navigation_launch.py 增加该节点到 node_names, 或
    #   (2) 用 explore_recover.sh 中的 ros2 lifecycle set 命令手动 activate.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_nav2_pkg, 'config', 'nav2_params.yaml'),
            description='nav2 参数 yaml'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='启 RViz2 显示 nav2_default_view (true/false). standalone '
                        '时默认 true, 配 go2_explore_bringup 时务必显式传 false.'
        ),
        # map -> odom 来源三选一(互斥):
        #   dynamic: go2_driver/map_odom_correction (默认, 与 FAST-LIO 同源)
        #   static : 老的单位/手调静态边 (publish_map_to_odom:=true)
        #   none   : 外部 slam_toolbox 发 (两个都 false)
        DeclareLaunchArgument(
            'dynamic_map_to_odom',
            default_value='true',
            description='由 go2_driver/map_odom_correction 以 20Hz 动态发布 '
                        'map->odom(与 FAST-LIO 位姿同源). 需要 Terminal A 在跑. '
                        '与 publish_map_to_odom 互斥.'
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom',
            default_value='false',
            description='发布老式静态 map->odom(单位变换/手调). 仅在 '
                        'dynamic_map_to_odom:=false 且没有 slam_toolbox 时用; '
                        '配 go2_explore_bringup (内含 slam_toolbox) 时必须 false.'
        ),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0',
                              description='map->odom 平移 x (m)'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0',
                              description='map->odom 平移 y (m)'),
        DeclareLaunchArgument('map_to_odom_z', default_value='0.0',
                              description='map->odom 平移 z (m)'),
        DeclareLaunchArgument('map_to_odom_roll', default_value='0.0',
                              description='map->odom roll (rad)'),
        DeclareLaunchArgument('map_to_odom_pitch', default_value='0.0',
                              description='map->odom pitch (rad)'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0',
                              description='map->odom yaw (rad)'),
        DeclareLaunchArgument(
            'use_utlidar_l2',
            default_value='true',
            description='起 L2 近场避障 (ground_filter_l2 -> local_costmap 的 '
                        'l2_ground/l2_obstacles 两路源). false 时退回只有 '
                        'Livox 的行为; costmap 里那两路源收不到数据, 静默跳过.'
        ),

        # 狗本体 (2026-10-23: lowstate_to_imu 已去掉, 见上面说明)
        twist_bridge,
        go2_driver,

        # 点云中继 (/cloud_registered -> /cloud_base), 保留作调试/检查用
        cloud_to_base,
        # 地面过滤 (/cloud_registered -> /cloud_obstacles + /cloud_ground),
        # local_costmap 障碍层的真实输入
        ground_filter,
        # L2 近场 (/utlidar/cloud_deskewed -> /cloud_ground_l2 +
        # /cloud_obstacles_l2), 补 Livox 的 1 m 盲区
        ground_filter_l2,
        # 地形图 -> frontier 用的占领栅格 (/exploration_map)
        exploration_map_relay,

        # TF + URDF (2026-10-23: EKF 已去掉, 见上面说明)
        go2_display_launch,
        # 把 camera_init->map 那棵树和 odom->base_footprint 那棵树
        # 接起来, 否则 global_costmap 拿不到 base_footprint 在 map 里的位姿.
        # 默认走动态修正(与 FAST-LIO 同源); 老静态边默认关闭.
        map_odom_correction,
        map_to_odom_tf,

        # nav2 (planner/controller/bt/costmap/behavior)
        nav2_launch,
        # 注意: 不在此处再起 lifecycle_manager,
        #       nav2_bringup 已经带 lifecycle_manager_navigation,
        #       否则会与它抢 bond, 导致 CRITICAL FAILURE.

        # RViz2 (standalone 默认弹, 配外层 bringup 时务必显式 rviz:=false)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ])