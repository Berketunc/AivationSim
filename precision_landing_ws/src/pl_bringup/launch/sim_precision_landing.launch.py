"""
Launch file for the precision-landing stack.

PX4 + Gazebo are started separately:
    cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco

Then launch this file:
    ros2 launch pl_bringup sim_precision_landing.launch.py

IMPORTANT — verify Gazebo topic names first:
    gz topic -l   (run while the sim is up)
Gazebo appends _0 to model names at spawn time, so exact names may differ
from the defaults below.  Edit gz_image_topic / gz_camera_info_topic to match.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

# ── Gazebo bridge topic names ──────────────────────────────────────────────────
# Run 'gz topic -l' while the sim is running and confirm these paths.
GZ_IMAGE_TOPIC = (
    '/world/aruco/model/x500_mono_cam_down_0'
    '/link/camera_link/sensor/imager/image'
)
GZ_CAMERA_INFO_TOPIC = (
    '/world/aruco/model/x500_mono_cam_down_0'
    '/link/camera_link/sensor/imager/camera_info'
)


def generate_launch_description():
    bringup_dir = get_package_share_directory('pl_bringup')
    control_params = os.path.join(bringup_dir, 'config', 'control_params.yaml')
    marker_params = os.path.join(bringup_dir, 'config', 'marker.yaml')

    return LaunchDescription([
        LogInfo(msg=(
            '[pl_bringup] Using Gazebo image topic: ' + GZ_IMAGE_TOPIC
        )),

        # Bridge Gazebo compressed image → ROS 2 sensor_msgs/Image
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_image', 'image_bridge', GZ_IMAGE_TOPIC],
            output='screen',
            name='gz_image_bridge',
        ),

        # Bridge Gazebo CameraInfo → ROS 2 sensor_msgs/CameraInfo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'{GZ_CAMERA_INFO_TOPIC}'
                '@sensor_msgs/msg/CameraInfo'
                '[gz.msgs.CameraInfo',
            ],
            output='screen',
            name='gz_camera_info_bridge',
        ),

        # ArUco perception node
        Node(
            package='pl_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            remappings=[
                ('image_raw',   GZ_IMAGE_TOPIC),
                ('camera_info', GZ_CAMERA_INFO_TOPIC),
            ],
            parameters=[marker_params],
        ),

        # Landing controller (includes MAVSDK bridge and state machine)
        Node(
            package='pl_control',
            executable='landing_controller_node',
            name='landing_controller_node',
            output='screen',
            parameters=[control_params],
        ),
    ])
