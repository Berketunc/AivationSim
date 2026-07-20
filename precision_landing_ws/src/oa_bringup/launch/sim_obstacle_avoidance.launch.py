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
GZ_IMAGE_TOPIC = '/world/warehouse/model/x500_3d_lidar_0/link/camera_link/sensor/imager/image'
GZ_CAMERA_INFO_TOPIC = (
    '/world/warehouse/model/x500_3d_lidar_0/link/camera_link/sensor/imager/camera_info')
GZ_IMU_TOPIC = '/world/warehouse/model/x500_3d_lidar_0/link/base_link/sensor/imu_sensor/imu'

# ── ROS-side topic names the rest of the obstacle-avoidance stack consumes ─────
OA_POINTCLOUD_TOPIC = '/oa/points'
# Ground truth is kept around under its own name, for comparison/drift-
# checking against VIO — it's no longer what mapping/planning/control use.
OA_GROUND_TRUTH_ODOM_TOPIC = '/oa/odom_ground_truth'
# This is now published by oa_vio/vio_odom_to_world, not bridged directly
# from Gazebo — but the name is unchanged, so odom_to_tf_node, oa_planning,
# and oa_control all keep working with zero changes: from their point of
# view this is just a different pose source on the same topic.
OA_ODOMETRY_TOPIC = '/oa/odom'
OA_IMAGE_TOPIC = '/oa/vio/image_raw'
OA_CAMERA_INFO_TOPIC = '/oa/vio/camera_info'
OA_IMU_TOPIC = '/oa/vio/imu'
# Must match ov_msckf's `namespace` launch arg + its ROS2Visualizer's fixed
# "odomimu" topic name.
VIO_ODOM_TOPIC = '/ov_msckf/odomimu'
# Published by oa_vio/vio_divergence_watchdog (sim-only — needs ground
# truth); path_follower_node lands immediately on True rather than
# continuing to navigate on a VIO estimate nothing else can validate.
OA_VIO_DIVERGED_TOPIC = '/oa/vio/diverged'

# ── TF frame names (same _0 instance-suffix caveat as GZ_ODOMETRY_TOPIC) ───────
# odom_to_tf_node publishes ODOM_FRAME -> BASE_FRAME from /oa/odom (now VIO,
# see vio_odom_to_world's world_frame_id/body_frame_id parameters below —
# they're set to these same two constants so the TF tree stays connected).
# LIDAR_FRAME is the point cloud's header.frame_id (see sim_assets/models/
# lidar_3d), fixed rigidly to BASE_FRAME per the LidarJoint pose in
# x500_3d_lidar/model.sdf — there's no plugin publishing that link, so it's
# a static transform here.
ODOM_FRAME = 'x500_3d_lidar_0/odom'
BASE_FRAME = 'x500_3d_lidar_0/base_footprint'
LIDAR_FRAME = 'x500_3d_lidar_0/lidar_link/lidar_3d'
LIDAR_MOUNT_XYZ = ('0', '0', '0.12')

# Known spawn pose vio_odom_to_world calibrates its VIO->world correction
# against — must match the PX4_GZ_MODEL_POSE in this file's docstring.
SPAWN_XYZ_YAW = (-8.5, 0.0, 0.2, 0.0)


def generate_launch_description():
    octomap_params = os.path.join(
        get_package_share_directory('oa_bringup'), 'config', 'octomap_params.yaml')
    planner_params = os.path.join(
        get_package_share_directory('oa_planning'), 'config', 'planner_params.yaml')
    control_params = os.path.join(
        get_package_share_directory('oa_control'), 'config', 'control_params.yaml')
    vio_config_path = os.path.join(
        get_package_share_directory('oa_vio'), 'config', 'aviationsim', 'estimator_config.yaml')

    return LaunchDescription([
        # Bridge Gazebo point cloud, ground-truth odometry (kept only for
        # comparison against VIO), and the vehicle IMU into ROS 2.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='oa_gz_bridge',
            output='screen',
            arguments=[
                f'{GZ_POINTCLOUD_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'{GZ_ODOMETRY_TOPIC}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
                f'{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'{GZ_IMU_TOPIC}@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            remappings=[
                (GZ_POINTCLOUD_TOPIC, OA_POINTCLOUD_TOPIC),
                (GZ_ODOMETRY_TOPIC, OA_GROUND_TRUTH_ODOM_TOPIC),
                (GZ_CAMERA_INFO_TOPIC, OA_CAMERA_INFO_TOPIC),
                (GZ_IMU_TOPIC, OA_IMU_TOPIC),
            ],
        ),

        # Camera image needs ros_gz_image specifically (handles the
        # raw/compressed image_transport publishers parameter_bridge doesn't).
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='oa_camera_image_bridge',
            output='screen',
            arguments=[GZ_IMAGE_TOPIC],
            remappings=[(GZ_IMAGE_TOPIC, OA_IMAGE_TOPIC)],
        ),

        # OpenVINS (built separately at ~/open_vins, sourced as an underlay —
        # see README) — mono VIO off the forward-facing camera + vehicle IMU.
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='ov_msckf',
            namespace='ov_msckf',
            output='screen',
            parameters=[
                # ov_core::Printer's PRINT_INFO/PRINT_DEBUG fire from ~489
                # call sites across ov_core/ov_init/ov_msckf, many per-frame
                # (camera, 21Hz) or per-sample (IMU, 250Hz) — at INFO this is
                # a genuine text firehose: real CPU spent formatting/writing
                # it, and it appears to starve ros2 launch's combined-output
                # capture for every other node in the stack (their own
                # get_logger() calls, confirmed working fine in isolation,
                # never showed up in launch.log during a full run).
                # WARNING keeps anything that actually indicates a problem.
                {'verbosity': 'WARNING'},
                {'use_stereo': False},
                {'max_cameras': 1},
                {'config_path': vio_config_path},
                # ov_msckf's own ROS2Visualizer broadcasts a raw "global"->
                # "imu" (and cam calibration) TF on every single IMU
                # callback — i.e. at IMU rate (250Hz here), independent of
                # vio_odom_to_world/odom_to_tf_node below, which already
                # publish the (calibrated, validated) pose this project's TF
                # tree actually uses. Left enabled, this redundant
                # broadcaster floods the log at IMU rate on its own, and
                # floods it with NaN transforms specifically whenever the
                # raw VIO estimate diverges (see MILESTONE2_STATUS.md
                # Component 6) — disabled here since nothing subscribes to
                # its frames.
                {'publish_global_to_imu_tf': False},
                {'publish_calibration_tf': False},
            ],
        ),

        # VIO's pose is in its own arbitrary init-time frame, not Gazebo's
        # world frame — this aligns it once against the known spawn pose.
        Node(
            package='oa_vio',
            executable='vio_odom_to_world',
            name='vio_odom_to_world',
            output='screen',
            # Python's stdout is fully buffered (not line-buffered) when it
            # isn't a tty, which is what launch's own subprocess pipes give
            # it — without this, get_logger().info/error() calls here queue
            # up in that buffer and are lost entirely if the process dies or
            # is SIGINT'd before the buffer fills, instead of reaching
            # launch.log. Same fix applied to every other Python node below.
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'vio_odom_topic': VIO_ODOM_TOPIC,
                'world_odom_topic': OA_ODOMETRY_TOPIC,
                'spawn_x': SPAWN_XYZ_YAW[0],
                'spawn_y': SPAWN_XYZ_YAW[1],
                'spawn_z': SPAWN_XYZ_YAW[2],
                'spawn_yaw': SPAWN_XYZ_YAW[3],
                'world_frame_id': ODOM_FRAME,
                'body_frame_id': BASE_FRAME,
            }],
        ),

        # Compares VIO's corrected pose against Gazebo ground truth (bridged
        # above purely for this) and both logs and publishes when they split
        # apart. MILESTONE2_STATUS.md's Component 6 divergence is
        # progressive (large-but-finite before ever going non-finite), so
        # nothing downstream can tell good pose from bad on its own —
        # path_follower_node below subscribes to OA_VIO_DIVERGED_TOPIC and
        # lands immediately rather than continuing to navigate on it.
        Node(
            package='oa_vio',
            executable='vio_divergence_watchdog',
            name='vio_divergence_watchdog',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{
                'vio_odom_topic': OA_ODOMETRY_TOPIC,
                'truth_odom_topic': OA_GROUND_TRUTH_ODOM_TOPIC,
                'diverged_topic': OA_VIO_DIVERGED_TOPIC,
            }],
        ),

        # VIO pose -> TF, so downstream mapping/planning nodes can just look
        # up transforms instead of each parsing Odometry directly.
        Node(
            package='oa_bringup',
            executable='odom_to_tf_node',
            name='odom_to_tf_node',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[{'odom_topic': OA_ODOMETRY_TOPIC}],
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
            parameters=[planner_params, {
                'odom_topic': OA_ODOMETRY_TOPIC,
                'vio_diverged_topic': OA_VIO_DIVERGED_TOPIC,
            }],
        ),

        # Takes off and drives the planned path via MAVSDK offboard.
        Node(
            package='oa_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[control_params, {
                'odom_topic': OA_ODOMETRY_TOPIC,
                'vio_diverged_topic': OA_VIO_DIVERGED_TOPIC,
            }],
        ),
    ])
