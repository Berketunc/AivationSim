"""
Launch file for the obstacle-avoidance sensor/pose bridging stack.

PX4 + Gazebo are started separately:
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=warehouse PX4_GZ_MODEL_POSE="-8.5,0,0.2,0,0,0" \\
        make px4_sitl gz_x500_3d_lidar

(-8.5, not -9: see oa_planning's planner_params.yaml `goal` comment for why
spawn/goal need real clearance from the walls, not just from the pillars.)

Then launch this file:
    ros2 launch oa_bringup sim_obstacle_avoidance.launch.py

IMPORTANT — verify Gazebo topic names first:
    gz topic -l   (run while the sim is up)
Gazebo appends _0 to model names at spawn time, so GZ_ODOMETRY_TOPIC below may
need editing if you spawn more than one instance. The LiDAR's point-cloud topic
name is fixed (set explicitly in sim_assets/models/lidar_3d/model.sdf) and does
not depend on the model instance suffix.

Also note: the LiDAR sensor uses lazy publishing, so `gz topic -l` only shows
/scan and /scan/points once something has already subscribed to them at least
once (this launch file's bridge counts as a subscriber, so after it's running
they'll show up normally).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ── Gazebo topic names ─────────────────────────────────────────────────────────
GZ_POINTCLOUD_TOPIC = '/scan/points'
GZ_ODOMETRY_TOPIC = '/model/x500_3d_lidar_0/odometry_with_covariance'
# x500_3d_lidar's mono_cam, repointed downward for ArUco landing (was
# forward-facing, for the now-removed VIO — see model.sdf's comment). Same
# physical camera_link, so these paths are unchanged from when VIO used them.
GZ_IMAGE_TOPIC = '/world/warehouse/model/x500_3d_lidar_0/link/camera_link/sensor/imager/image'
GZ_CAMERA_INFO_TOPIC = (
    '/world/warehouse/model/x500_3d_lidar_0/link/camera_link/sensor/imager/camera_info')

# ── ROS-side topic names the rest of the obstacle-avoidance stack consumes ─────
OA_POINTCLOUD_TOPIC = '/oa/points'
# Research pivot (see MILESTONE2_STATUS.md): VIO (Component 6) has been
# pulled out of the active pipeline entirely, not just demoted — ground
# truth is the only localization source now, matching what Isaac Lab's
# residual-RL training/eval will use (privileged state, standard practice
# for sim-based RL robotics research). The OpenVINS integration itself
# (oa_vio/, scripts/build_openvins.sh) is left on disk, unused, as
# documented appendix/future-work material rather than deleted.
OA_ODOMETRY_TOPIC = '/oa/odom_ground_truth'
OA_IMAGE_TOPIC = '/oa/landing/image_raw'
OA_CAMERA_INFO_TOPIC = '/oa/landing/camera_info'
# Matches path_follower_node's marker_pose_topic/marker_visible_topic defaults.
OA_MARKER_POSE_TOPIC = '/oa/landing/target_pose'
OA_MARKER_VISIBLE_TOPIC = '/oa/landing/is_visible'

# ── TF frame names (same _0 instance-suffix caveat as GZ_ODOMETRY_TOPIC) ───────
# odom_to_tf_node publishes ODOM_FRAME -> BASE_FRAME from OA_ODOMETRY_TOPIC.
# LIDAR_FRAME is the point cloud's header.frame_id (see sim_assets/models/
# lidar_3d), fixed rigidly to BASE_FRAME per the LidarJoint pose in
# x500_3d_lidar/model.sdf — there's no plugin publishing that link, so it's
# a static transform here.
ODOM_FRAME = 'x500_3d_lidar_0/odom'
BASE_FRAME = 'x500_3d_lidar_0/base_footprint'
LIDAR_FRAME = 'x500_3d_lidar_0/lidar_link/lidar_3d'
LIDAR_MOUNT_XYZ = ('0', '0', '0.12')


def generate_launch_description():
    octomap_params = os.path.join(
        get_package_share_directory('oa_bringup'), 'config', 'octomap_params.yaml')
    planner_params = os.path.join(
        get_package_share_directory('oa_planning'), 'config', 'planner_params.yaml')
    control_params = os.path.join(
        get_package_share_directory('oa_control'), 'config', 'control_params.yaml')

    return LaunchDescription([
        # Bridge Gazebo point cloud + ground-truth odometry into ROS 2.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='oa_gz_bridge',
            output='screen',
            arguments=[
                f'{GZ_POINTCLOUD_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'{GZ_ODOMETRY_TOPIC}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
            ],
            remappings=[
                (GZ_POINTCLOUD_TOPIC, OA_POINTCLOUD_TOPIC),
                (GZ_ODOMETRY_TOPIC, OA_ODOMETRY_TOPIC),
            ],
        ),

        # Ground-truth pose -> TF, so downstream mapping/planning nodes can
        # just look up transforms instead of each parsing Odometry directly.
        Node(
            package='oa_bringup',
            executable='odom_to_tf_node',
            name='odom_to_tf_node',
            output='screen',
            # Python's stdout is fully buffered (not line-buffered) when it
            # isn't a tty, which is what launch's own subprocess pipes give
            # it — without this, get_logger().info/error() calls here queue
            # up in that buffer and are lost entirely if the process dies or
            # is SIGINT'd before the buffer fills, instead of reaching
            # launch.log. Same fix applied to every other Python node below.
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{'odom_topic': OA_ODOMETRY_TOPIC}],
        ),

        # Downward camera image needs ros_gz_image specifically (handles the
        # raw/compressed image_transport publishers parameter_bridge doesn't).
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='oa_camera_image_bridge',
            output='screen',
            arguments=[GZ_IMAGE_TOPIC],
            remappings=[(GZ_IMAGE_TOPIC, OA_IMAGE_TOPIC)],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='oa_camera_info_bridge',
            output='screen',
            arguments=[f'{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            remappings=[(GZ_CAMERA_INFO_TOPIC, OA_CAMERA_INFO_TOPIC)],
        ),

        # ArUco marker detection (pl_perception, reused as-is from Milestone
        # 1) off the downward camera — parameters match warehouse.sdf's
        # landing_marker (Tools/simulation/gz/models/arucotag): DICT_4X4_50,
        # id 0, 0.5x0.5m.
        Node(
            package='pl_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'marker_id': 0,
                'marker_size_m': 0.5,
                'aruco_dict': 'DICT_4X4_50',
            }],
            remappings=[
                ('image_raw', OA_IMAGE_TOPIC),
                ('camera_info', OA_CAMERA_INFO_TOPIC),
                ('target_pose', OA_MARKER_POSE_TOPIC),
                ('is_visible', OA_MARKER_VISIBLE_TOPIC),
            ],
        ),

        # Fixed sensor-mount transform (no plugin publishes this one).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=[
                '--x', LIDAR_MOUNT_XYZ[0], '--y', LIDAR_MOUNT_XYZ[1], '--z', LIDAR_MOUNT_XYZ[2],
                '--frame-id', BASE_FRAME,
                '--child-frame-id', LIDAR_FRAME,
            ],
        ),

        # 3D occupancy map from the point cloud + TF.
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[octomap_params],
            remappings=[
                ('cloud_in', OA_POINTCLOUD_TOPIC),
            ],
        ),

        # A* path planning over octomap_server's occupied-cell centers.
        Node(
            package='oa_planning',
            executable='planner_node',
            name='oa_planning_node',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[planner_params, {'odom_topic': OA_ODOMETRY_TOPIC}],
        ),

        # Takes off and drives the planned path via MAVSDK offboard.
        Node(
            package='oa_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[control_params, {'odom_topic': OA_ODOMETRY_TOPIC}],
        ),
    ])
