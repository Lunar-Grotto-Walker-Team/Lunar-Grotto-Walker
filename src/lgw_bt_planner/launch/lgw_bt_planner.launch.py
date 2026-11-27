"""LG launcher for pick-and-place BT trees.

Loads lgw_bt_planner's plugin (.so) and the BT tree XMLs from this package's
share/. Triggers trees via:
  ros2 action send_goal /lgw_bt_execution btcpp_ros2_interfaces/action/ExecuteTree \
    "{target_tree: <TreeID>}"

Note: gripper_action_server Node was removed (pick_and_place_demo uses
MoveArmToNamed named_target="gripper_open/close" via move_group + SRDF named
states, not the legacy ControlGripper action).

Prerequisites (handled by start_lgw_pose_replay.sh L2 wrapper, NOT by this launch):
  - can0 must be UP @ 1 Mbps.
    Run: bash /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000
  - agx_arm_ctrl hardware driver must already be running (publishes /control + /feedback).
    Run: ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
           effector_type:=agx_gripper tcp_offset:='...'
  - agx_arm_moveit demo.launch.py must already be running (provides /move_action).
This launch brings up the BT runner + select_arm service node + body_pose_bridge
(SetBodyPose BT 节点需要 /set_body_pose service,head_arm_independent_lookat
等用 SetBodyPose 前倾/后倾的 BT 树离不开它);hardware driver + MoveIt 仍
是 L2 责任,对齐上游 agx_arm_ros README 约定。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# bt_runner 的 plugins 里有 go2_transit/bt_plugins(TransitToLocalPoint)。
# RegisterPlugins() 对解析不到的插件目录是**静默跳过**的，缺了它
# transit_mission.xml 会在 "Node not recognized: TransitToLocalPoint" 处解析失败，
# 进而所有引用 <SubTree ID="TransitMission"/> 的树都创建不出来。
#
# go2_transit 现在与本包在同一工作区（同一次 colcon build / 同一个 install），
# 因此直接用包查找即可；不再需要把 nav_ws 的 install 前缀塞进 launch 环境。

def generate_launch_description():
    body_pose_bridge_launch = os.path.join(
        get_package_share_directory('body_pose_bridge'),
        'launch', 'body_pose_bridge.launch.py')

    return LaunchDescription([
        # ---- go2_transit/transit_node(TransitToLocalPoint 的动作服务器) ----
        # 2026-11-20 真机踩坑: 只把插件注册进 bt_runner 是不够的。
        # TransitToLocalPoint::ready() 要求 /go2_transit/transit_to_pose 这个
        # **动作服务器**在线, 而服务器在 transit_node 进程里。以前 transit_node
        # 只由 start_transit_mission.sh(TransitMission 专用 launch)起, 走
        # lgw_chain 跑 TrackCollectReactiveV2 时树能建、插件能加载, 但
        # ready() 永远等不到动作服务器 —— 3 次 × 10 s 后整树 FAILURE, 日志只有
        # 一句笼统的"TF/动作服务器未就绪"。
        # 所以这里跟着 bt_runner 一起把 transit_node 带起来; 那些导航栈没起的
        # 场合(单测 / bench)它只是空转等 goal, 不动机器狗。
        #
        # max_retries 覆盖为 0: 重试只留树里 RetryUntilSuccessful 那一层,
        # 与 transit_mission.launch.py 的约定一致(避免 3×3 次往返)。
        # use_transit:=false 可关掉(例如已有别的 launch 起了 transit_node ——
        # 同名两个节点会让 goal 随机落到其中一个)。
        DeclareLaunchArgument(
            'use_transit', default_value='true',
            choices=['true', 'false'],
            description='start go2_transit/transit_node (TransitToLocalPoint '
                        'needs its action server); set false if another '
                        'launch already runs one'),
        Node(
            package='go2_transit',
            executable='transit_node',
            name='transit_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('go2_transit'),
                    'config', 'cave_transit.yaml'),
                {'max_retries': 0},
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_transit')),
        ),

        # select_arm_observation_pose_service_node — 独立 rclcpp::Node,
        # 暴露 /select_arm_observation_pose 服务给 BT thin shell
        # SelectArmObservationPoseBTNode 调。必须在 MoveIt
        # (agx_arm_moveit/demo.launch.py) 起来之后才能正常处理 IK 调用,
        # 但 launch 顺序由 caller 保证 —— 起本 launch 的脚本 / 手动测试
        # 必须先起 demo。
        # 之所以必须独立成 Node 而非 BT plugin:BT thin shell 是
        # BT::RosServiceNode<S>(plugin 里),它需要 dial 一个
        # ROS 服务,所以服务必须在另一进程里跑。declare 在前,
        # 让 service 在 bt_runner 加载 tree 之前 register 完。
        Node(
            package='lgw_bt_planner',
            executable='select_arm_observation_pose_service_node',
            name='select_arm_observation_pose_service',
            output='screen',
        ),

        # Layer 1 + 2: BT engine + 3 thin shells (MoveArmToNamed,
        # MoveArmToPose, SetPoseStamped). TreeExecutionServer loads all 6
        # BT XMLs from bt_trees/ via the behavior_trees param and
        # discovers the plugin .so via the plugins param.
        Node(
            package='lgw_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            # output='screen' → stdout/stderr 走 launch process,被 start_lgw
            # 脚本的 bash redirect 接住到 .run/lgw_head_to_arm_verify_grasp_bt.log。
            # ROS 2 默认 output='log' 会写到 /root/.ros/log/bt_runner_*.log,
            # start 脚本接不到(2026-10-07 之前 BT runner 日志全部散落在那里)。
            output='screen',
            parameters=[{
                # go2_transit/bt_plugins (2026-11-20): TransitToLocalPoint。
                # track_collect_reactive_v2 的 init 末尾用了 <SubTree
                # ID="TransitMission"/>，不加载这个插件的话
                # transit_mission.xml 会在 "Node not recognized:
                # TransitToLocalPoint" 处解析失败，进而 V2 自己也创建不出来
                # (报 "Can't find tree TransitMission")。
                # 需要 nav_ws 的 go2_transit 已安装并在环境里
                # (.l4t_nav_env.sh)；只注册节点，不 require Nav2 在线。
                'plugins': [
                    'lgw_bt_planner/bt_plugins',
                    'go2_transit/bt_plugins',
                ],
                'behavior_trees': ['lgw_bt_planner/bt_trees'],
                'action_name': 'lgw_bt_execution',
                'tick_frequency': 10,
                # IsHead/ArmDetectionConfirmed subscribe to topics named
                # in detection_condition_test.xml (per-node `topic_name=`
                # InputPort attribute). BT.ROS2 v0.3.0's RosTopicSubNode
                # reads topic_name from the node's BT-config InputPort,
                # NOT from per-instance ROS parameters — see
                # is_detection_confirmed_node.hpp §NOTES for why we keep
                # the topic out of C++.
                # See detection.txt §八 for the canonical topic names.
            }],
        ),

        # body_pose_bridge — 暴露 /set_body_pose service 给 SetBodyPose BT
        # 节点。head_arm_independent_lookat / go2_posture_demo 等用
        # SetBodyPose 控 Go2 机身姿态(rpy / clear HOLDING,2026-10-13
        # v7 移除 body_height)的树都依赖它;不起则 SetBodyPose 立刻
        # FAIL → Sequence fail-fast 截断后续步骤(2026-10-10
        # head_arm_independent_lookat 真机踩坑根因)。同款
        # IncludeLaunchDescription 模式见 go2_posture_demo.launch.py。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(body_pose_bridge_launch)),
    ])