"""
Launch file for the obstacle-avoidance sensor/pose bridging stack.

PX4 + Gazebo are started separately:
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=warehouse PX4_GZ_MODEL_POSE="-9,0,0.2,0,0,0" \\
        make px4_sitl gz_x500_3d_lidar

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

from launch import LaunchDescription
from launch_ros.actions import Node

# ── Gazebo topic names ─────────────────────────────────────────────────────────
GZ_POINTCLOUD_TOPIC = '/scan/points'
GZ_ODOMETRY_TOPIC = '/model/x500_3d_lidar_0/odometry_with_covariance'

# ── ROS-side topic names the rest of the obstacle-avoidance stack consumes ─────
OA_POINTCLOUD_TOPIC = '/oa/points'
OA_ODOMETRY_TOPIC = '/oa/odom'


def generate_launch_description():
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
            parameters=[{'odom_topic': OA_ODOMETRY_TOPIC}],
        ),
    ])
