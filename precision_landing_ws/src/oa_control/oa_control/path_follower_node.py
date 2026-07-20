#!/usr/bin/env python3
"""
MAVSDK trajectory follower for obstacle-avoidance path planning.

Takes off, then continuously walks the latest oa_planning Path: finds the
nearest waypoint to the current (ground-truth) position, advances through
waypoints as each is reached, and drives body-frame velocity setpoints toward
the current one via the same MavsdkBridge pl_control uses for the precision
landing controller — no ArUco/landing-specific logic in that bridge, so it's
reused as-is rather than duplicated.
"""

import math
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path

from pl_control.mavsdk_bridge import MavsdkBridge


class State(Enum):
    INIT = auto()
    FOLLOW = auto()
    GOAL_REACHED = auto()
    ABORT = auto()


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter('cruise_speed_ms', 0.8)
        self.declare_parameter('waypoint_reached_radius_m', 0.3)
        self.declare_parameter('goal_reached_radius_m', 0.3)
        self.declare_parameter('mavsdk_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('timer_hz', 20.0)
        self.declare_parameter('path_topic', '/oa/path')
        self.declare_parameter('odom_topic', '/oa/odom')

        self.cruise_speed: float = self.get_parameter('cruise_speed_ms').value
        self.waypoint_radius: float = self.get_parameter('waypoint_reached_radius_m').value
        self.goal_radius: float = self.get_parameter('goal_reached_radius_m').value
        self.takeoff_alt: float = self.get_parameter('takeoff_altitude_m').value
        mavsdk_addr: str = self.get_parameter('mavsdk_address').value
        timer_hz: float = self.get_parameter('timer_hz').value

        # ── state ─────────────────────────────────────────────────────────────
        self.state = State.INIT
        self.path: Path | None = None
        self.waypoint_idx = 0
        self.current_pos = None   # (x, y, z)
        self.current_yaw = 0.0

        # ── bridge & subscriptions ────────────────────────────────────────────
        self.bridge = MavsdkBridge(system_address=mavsdk_addr)

        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._path_cb, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)

        # ── startup on background thread (keeps rclpy spin unblocked) ─────────
        threading.Thread(target=self._startup, daemon=True).start()

        self.create_timer(1.0 / timer_hz, self._tick)

        self.get_logger().info('path_follower_node initialized')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        self.path = msg
        # Resume from whichever waypoint is nearest now, rather than snapping
        # back to the start of the path on every replan.
        self.waypoint_idx = self._nearest_waypoint_index(msg)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)
        self.current_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    # ── startup (background thread) ───────────────────────────────────────────

    def _startup(self):
        try:
            self.get_logger().info('Connecting to PX4 via MAVSDK...')
            self.bridge.run(self.bridge.connect(), timeout=30.0)

            self.get_logger().info(f'Connected. Arming and taking off to {self.takeoff_alt}m...')
            self.bridge.run(self.bridge.arm_and_takeoff(self.takeoff_alt), timeout=90.0)

            self.get_logger().info('Altitude reached. Starting offboard velocity loop...')
            self.bridge.start_offboard_loop()
            time.sleep(1.0)  # brief settle before handing off to the follower

            self.state = State.FOLLOW
        except Exception as exc:
            self.get_logger().error(f'Startup failed: {exc}')
            self.state = State.ABORT

    # ── control tick (main thread, ROS timer) ─────────────────────────────────

    def _tick(self):
        if self.state != State.FOLLOW:
            return
        if self.path is None or not self.path.poses or self.current_pos is None:
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            return

        target = self._advance_to_current_waypoint()
        if target is None:
            # Reached the last waypoint: hold position.
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            if self.state != State.GOAL_REACHED:
                self.state = State.GOAL_REACHED
                self.get_logger().info('Goal reached — holding position.')
            return

        cx, cy, cz = self.current_pos
        dx, dy, dz = target[0] - cx, target[1] - cy, target[2] - cz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            return

        speed = min(self.cruise_speed, dist)
        wx, wy, wz = (dx / dist) * speed, (dy / dist) * speed, (dz / dist) * speed

        # World ENU -> body FRD (MAVSDK VelocityBodyYawspeed convention).
        yaw = self.current_yaw
        forward = wx * math.cos(yaw) + wy * math.sin(yaw)
        right = wx * math.sin(yaw) - wy * math.cos(yaw)
        down = -wz

        self.bridge.send_velocity_body(forward, right, down, 0.0)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _advance_to_current_waypoint(self):
        """Return the world-frame (x,y,z) of the waypoint to head toward, or
        None once the last waypoint has been reached."""
        poses = self.path.poses
        cx, cy, cz = self.current_pos

        while self.waypoint_idx < len(poses):
            p = poses[self.waypoint_idx].pose.position
            dist = math.sqrt((p.x - cx) ** 2 + (p.y - cy) ** 2 + (p.z - cz) ** 2)
            is_last = self.waypoint_idx == len(poses) - 1
            radius = self.goal_radius if is_last else self.waypoint_radius
            if dist > radius:
                return (p.x, p.y, p.z)
            if is_last:
                return None
            self.waypoint_idx += 1

        return None

    def _nearest_waypoint_index(self, path: Path) -> int:
        if self.current_pos is None or not path.poses:
            return 0
        cx, cy, cz = self.current_pos
        best_idx, best_dist = 0, math.inf
        for i, pose in enumerate(path.poses):
            p = pose.pose.position
            dist = (p.x - cx) ** 2 + (p.y - cy) ** 2 + (p.z - cz) ** 2
            if dist < best_dist:
                best_dist, best_idx = dist, i
        return best_idx


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
