"""Launch file for go2_front_camera_driver.

Starts the driver node plus a placeholder static_transform_publisher that
publishes go2_front_camera_optical_frame -> base_link. Replace the xyz/rpy
arguments below with the measured Go2 installation pose once known
(see l4t/nav_ws/fastcalib_ws/ for calibration tooling).
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
            description="Network interface connected to the Go2 (used for UDP multicast). "
                        "Default eth10 for the Jetson Go2 eth-to-robot link; override with "
                        "multicast_iface:=<iface> if your setup differs.",
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

    # Static TF: go2_front_camera_optical_frame -> base_link.
    # Source: Unitree URDF chain (backup/go2_ros2_sdk/go2_robot_sdk/urdf/go2.urdf).
    #   base_link -> Head_upper_joint      xyz=(0.285, 0, 0.01), rpy=(0, 0, 0)
    #   Head_upper -> front_camera_joint  xyz=(0.045, 0, 0.03), rpy=(-1.5708, 0, -1.5708)
    # Composed camera pose in base_link:
    #   xyz = (0.285+0.045, 0, 0.01+0.03) = (0.330, 0.0, 0.040)
    #   rpy = (-1.5708, 0, -1.5708)        (Head_upper is identity; same as joint rpy)
    # The (-pi/2, 0, -pi/2) rotation is the standard ROS optical frame convention
    # (Z forward, X right, Y down), so this static_transform_publisher publishes
    # the optical frame directly — same convention as realsense2_camera.
    #
    # TODO: refine with real-machine pitch measurement. URDF says 0 (camera level),
    # but the physical mount on Go2 typically tilts ~15-30 deg downward so the
    # ground in front is visible. Use a level / inclinometer on the camera housing
    # once Go2 is powered on.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="go2_front_camera_tf_urdf",
        arguments=[
            "--x",     "0.330",
            "--y",     "0.0",
            "--z",     "0.040",
            "--roll",  "-1.5708",
            "--pitch", "0.0",
            "--yaw",   "-1.5708",
            "--frame-id",       "base_link",
            "--child-frame-id", "go2_front_camera_optical_frame",
        ],
    )

    return LaunchDescription(args + [driver, camera_tf])
