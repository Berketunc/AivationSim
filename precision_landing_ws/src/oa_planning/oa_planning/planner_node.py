import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from oa_planning import astar
from oa_planning.occupancy_grid import OccupancyGrid3D


class PlannerNode(Node):
    """3D A* planner over octomap_server's occupied-cell centers.

    octomap_server's own wire format (/octomap_binary) is a compressed octree
    serialization with no Python decoder available in this ROS distro (only
    C++ bindings exist via octomap_ros). /octomap_point_cloud_centers already
    gives us that same occupancy decision as a flat point cloud, so we build a
    plain bounded voxel grid from it instead of parsing the octree ourselves.
    """

    def __init__(self):
        super().__init__('oa_planning_node')

        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('origin', [-10.0, -7.0, 0.0])
        self.declare_parameter('size', [20.0, 14.0, 3.5])
        self.declare_parameter('goal', [9.0, 0.0, 1.5])
        self.declare_parameter('replan_period_s', 2.0)
        self.declare_parameter('inflation_radius_m', 0.3)
        self.declare_parameter('cloud_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('odom_topic', '/oa/odom')
        self.declare_parameter('goal_topic', '/oa/goal_pose')
        self.declare_parameter('path_topic', '/oa/path')
        self.declare_parameter('vio_diverged_topic', '/oa/vio/diverged')

        resolution = self.get_parameter('resolution').value
        origin = tuple(self.get_parameter('origin').value)
        size = tuple(self.get_parameter('size').value)
        inflation_radius_m = self.get_parameter('inflation_radius_m').value
        self._grid = OccupancyGrid3D(origin, size, resolution, inflation_radius_m)

        self._goal = tuple(self.get_parameter('goal').value)
        self._current_pos = None
        self._frame_id = None
        # Set by oa_vio/vio_divergence_watchdog (sim-only — needs ground
        # truth). Without this, planning kept running on an increasingly
        # deranged VIO position long after path_follower_node had already
        # given up and landed — harmless (the follower ignores it once
        # stopped) but wasted CPU replanning against nonsense and spammed
        # "No path found" warnings continuously instead of once.
        self._localization_lost = False

        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value, self._on_cloud, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._on_goal, 10)
        self.create_subscription(
            Bool, self.get_parameter('vio_diverged_topic').value, self._on_diverged, 10)

        self._path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 1)

        period = self.get_parameter('replan_period_s').value
        self.create_timer(period, self._plan_once)

    def _on_cloud(self, msg: PointCloud2):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        self._grid.set_occupied_from_points(points)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._current_pos = (p.x, p.y, p.z)
        self._frame_id = msg.header.frame_id

    def _on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        self._goal = (p.x, p.y, p.z)
        self.get_logger().info(f'New goal: {self._goal}')

    def _on_diverged(self, msg: Bool):
        if msg.data and not self._localization_lost:
            self.get_logger().error(
                'VIO localization lost (see vio_divergence_watchdog) — stopping replanning '
                'until it recovers, rather than planning against a position nothing can '
                'validate.')
            empty_path = Path()
            empty_path.header.frame_id = self._frame_id or 'map'
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self._path_pub.publish(empty_path)
        self._localization_lost = msg.data

    def _plan_once(self):
        if self._localization_lost:
            return
        if self._current_pos is None:
            return

        # Expensive (O(occupied cells * offsets)) — do it once per replan,
        # not on every point-cloud callback.
        self._grid.refresh_inflation()

        start_idx = self._grid.world_to_index(self._current_pos)
        goal_idx = self._grid.world_to_index(self._goal)

        path_idx = astar.plan(start_idx, goal_idx, self._grid)

        path_msg = Path()
        path_msg.header.frame_id = self._frame_id or 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if path_idx is None:
            # Publish an EMPTY path rather than just warning and returning.
            # path_follower_node already treats "no poses" as "hold position
            # now" — without this, a failed replan left it blindly executing
            # whatever stale path it had, which could already be running
            # through territory the map has since learned is occupied.
            self.get_logger().warn(
                f'No path found from {self._current_pos} to {self._goal} — '
                f'publishing empty path so the follower stops.')
            self._path_pub.publish(path_msg)
            return

        for idx in path_idx:
            x, y, z = self._grid.index_to_world(idx)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self._path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints')


def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
