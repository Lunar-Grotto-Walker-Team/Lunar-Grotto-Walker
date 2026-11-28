"""Launch file for go2_front_camera_driver in FAST-LIVO2 L2 configuration.

Differences from front_camera.launch.py:
  - Parent TF frame is the L2 LiDAR / its built-in IMU, NOT base_link.
    fastlivo2 typically uses the lidar frame as the body reference, with the
    lidar's IMU as the inertial source. This launch publishes
    go2_front_camera_optical_frame relative to that body frame.
  - xyz/rpy are derived from the inverse of the lidar->camera extrinsic
    measured by fastcalib_livox (`output/single_calib_result.txt`):
        Rcl (lidar->camera), Pcl (lidar->camera)
        R_c_l = Rcl.T,  P_c_l = -Rcl.T @ Pcl
    RPY extracted from R_c_l via scipy.spatial.transform.Rotation.as_euler('zyx').
  - Override any value at launch time:
        tf_x:=... tf_y:=... tf_z:=... tf_roll:=... tf_pitch:=... tf_yaw:=...

Pre-launch checklist:
  1. Confirm the actual TF frame name published by your livox_ros_driver2.
     Override `livox_parent_frame` if it is not `livox_frame`.
  2. Verify topic names match fastlivo2 yaml (`img_topic`,
     typically /camera/frontcamera/image_raw). Override `image_topic` /
     `camera_info_topic` if needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "multicast_iface",
            default_value="eth10",
            description="Network interface connected to the Go2 (UDP multicast).",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/frontcamera/image_raw",
            description="Published sensor_msgs/Image topic.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/frontcamera/camera_info",
            description="Published sensor_msgs/CameraInfo topic.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="go2_front_camera_optical_frame",
            description="Header frame_id for image and camera_info.",
        ),
        DeclareLaunchArgument(
            "livox_parent_frame",
            default_value="livox_frame",
            description="Parent TF frame for the camera optical frame. "
                        "Should be the L2 lidar/IMU frame actually published by "
                        "your l2_ros2_driver (commonly 'livox_frame').",
        ),
        # Camera -> livox TF (inverse of fastcalib Rcl/Pcl):
        #   Rcl = lidar->camera,  Pcl = lidar->camera translation
        #   R_c_l = Rcl.T,  P_c_l = -Rcl.T @ Pcl, then R_c_l -> RPY via scipy as_euler('zyx')
        # Source: l4t/nav_ws/fastcalib_livox/output/single_calib_result.txt
        DeclareLaunchArgument("tf_x",     default_value="0.10895", description="camera->livox x (m)."),
        DeclareLaunchArgument("tf_y",     default_value="-0.00354", description="camera->livox y (m)."),
        DeclareLaunchArgument("tf_z",     default_value="-0.12256", description="camera->livox z (m)."),
        DeclareLaunchArgument("tf_roll",  default_value="0.01662",  description="camera->livox roll (rad)."),
        DeclareLaunchArgument("tf_pitch", default_value="1.31936",  description="camera->livox pitch (rad)."),
        DeclareLaunchArgument("tf_yaw",   default_value="-1.58950", description="camera->livox yaw (rad)."),
    ]

    driver = Node(
        package="go2_front_camera_driver",
        executable="go2_front_camera_driver_node",
        name="go2_front_camera_driver",
        output="screen",
        parameters=[{
            "multicast_iface":   LaunchConfiguration("multicast_iface"),
            "image_topic":       LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "frame_id":          LaunchConfiguration("frame_id"),
        }],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="go2_front_camera_tf_livox",
        arguments=[
            "--x",     LaunchConfiguration("tf_x"),
            "--y",     LaunchConfiguration("tf_y"),
            "--z",     LaunchConfiguration("tf_z"),
            "--roll",  LaunchConfiguration("tf_roll"),
            "--pitch", LaunchConfiguration("tf_pitch"),
            "--yaw",   LaunchConfiguration("tf_yaw"),
            "--frame-id",       LaunchConfiguration("livox_parent_frame"),
            "--child-frame-id", LaunchConfiguration("frame_id"),
        ],
    )

    return LaunchDescription(args + [driver, camera_tf])
