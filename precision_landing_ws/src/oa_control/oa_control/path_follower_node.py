#!/usr/bin/env python3
"""
MAVSDK trajectory follower for obstacle-avoidance path planning.

Takes off, then continuously walks the latest oa_planning Path in order,
advancing through waypoints as each is reached, and drives body-frame
velocity setpoints toward the current one via the same MavsdkBridge
pl_control uses for the precision landing controller — no ArUco/landing-
specific logic in that bridge, so it's reused as-is rather than duplicated.

Every new Path always starts at (a cell very close to) the current position,
because oa_planning_node always plans from wherever the vehicle currently is
— so a new path is always resumed from waypoint 0, never by searching for
whichever waypoint happens to be spatially nearest. That search used to be
here and was a real bug: the "nearest" waypoint by raw distance can be one
further along the path, on the far side of an obstacle the path was
deliberately routed around, which then had this controller cut straight
through it to reach that "closer" point.

Once the A* goal is reached, this hands off from waypoint-following to
ArUco-marker landing (SEARCH_MARKER -> ALIGN_MARKER -> DESCEND_MARKER),
reusing pl_control.landing_controller_node's PIController and align/descend
approach directly (same airframe, same MavsdkBridge, same marker) — but
embedded in this node's own state machine rather than a second MAVSDK-
connected node, since two nodes can't safely share one offboard control
loop on the same vehicle.
"""

import math
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

from pl_control.landing_controller_node import PIController
from pl_control.mavsdk_bridge import MavsdkBridge


class State(Enum):
    INIT = auto()
    FOLLOW = auto()
    SEARCH_MARKER = auto()
    ALIGN_MARKER = auto()
    DESCEND_MARKER = auto()
    LANDED = auto()
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

        self.declare_parameter('hold_position_kp', 0.5)
        self.declare_parameter('hold_position_max_speed_ms', 0.3)
        self.hold_kp: float = self.get_parameter('hold_position_kp').value
        self.hold_max_speed: float = self.get_parameter('hold_position_max_speed_ms').value

        # ── marker-landing parameters (see module docstring) ────────────────────
        self.declare_parameter('marker_pose_topic', '/oa/landing/target_pose')
        self.declare_parameter('marker_visible_topic', '/oa/landing/is_visible')
        self.declare_parameter('marker_pi_kp', 0.5)
        self.declare_parameter('marker_pi_ki', 0.05)
        self.declare_parameter('marker_pi_integral_max', 0.5)
        self.declare_parameter('marker_horizontal_vel_max', 1.0)
        self.declare_parameter('marker_descend_vel_ms', 0.3)
        self.declare_parameter('marker_hacc_radius_m', 0.20)
        self.declare_parameter('marker_n_frames_aligned', 10)
        self.declare_parameter('marker_final_land_alt_m', 0.5)
        self.declare_parameter('marker_search_speed_ms', 0.2)
        self.declare_parameter('marker_search_step_m', 0.5)
        self.declare_parameter('marker_search_timeout_s', 20.0)
        self.declare_parameter('marker_lost_timeout_s', 2.0)

        marker_pi_kp = self.get_parameter('marker_pi_kp').value
        marker_pi_ki = self.get_parameter('marker_pi_ki').value
        marker_pi_i_max = self.get_parameter('marker_pi_integral_max').value
        marker_v_max = self.get_parameter('marker_horizontal_vel_max').value
        self.marker_pi_x = PIController(marker_pi_kp, marker_pi_ki, marker_pi_i_max, marker_v_max)
        self.marker_pi_y = PIController(marker_pi_kp, marker_pi_ki, marker_pi_i_max, marker_v_max)

        self.marker_descend_vel: float = self.get_parameter('marker_descend_vel_ms').value
        self.marker_hacc_radius_m: float = self.get_parameter('marker_hacc_radius_m').value
        self.marker_n_frames_aligned: int = self.get_parameter('marker_n_frames_aligned').value
        self.marker_final_land_alt_m: float = self.get_parameter('marker_final_land_alt_m').value
        self.marker_search_speed: float = self.get_parameter('marker_search_speed_ms').value
        self.marker_search_step_m: float = self.get_parameter('marker_search_step_m').value
        self.marker_search_timeout_s: float = self.get_parameter('marker_search_timeout_s').value
        self.marker_lost_timeout_s: float = self.get_parameter('marker_lost_timeout_s').value

        # ── state ─────────────────────────────────────────────────────────────
        self.state = State.INIT
        self.path: Path | None = None
        self.waypoint_idx = 0
        self.current_pos = None   # (x, y, z)
        self.current_yaw = 0.0
        self.hold_pos = None      # anchor point while holding (see _hold_position)
        self.last_tick_time = time.monotonic()

        self.marker_pose: PoseStamped | None = None
        self.marker_visible = False
        self.marker_state_entry_time = time.monotonic()
        self.marker_last_seen_time = time.monotonic()
        self.marker_aligned_count = 0

        # ── bridge & subscriptions ────────────────────────────────────────────
        self.bridge = MavsdkBridge(system_address=mavsdk_addr)

        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._path_cb, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('marker_pose_topic').value, self._marker_pose_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('marker_visible_topic').value, self._marker_visible_cb, 10)

        # ── startup on background thread (keeps rclpy spin unblocked) ─────────
        threading.Thread(target=self._startup, daemon=True).start()

        self.create_timer(1.0 / timer_hz, self._tick)

        self.get_logger().info('path_follower_node initialized')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        self.path = msg
        # Every new path starts at (a cell very close to) the current
        # position — see the module docstring for why this is index 0, not
        # a "nearest waypoint" search.
        self.waypoint_idx = 0

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)
        self.current_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _marker_pose_cb(self, msg: PoseStamped):
        self.marker_pose = msg

    def _marker_visible_cb(self, msg: Bool):
        self.marker_visible = msg.data

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
        if self.state in (State.INIT, State.ABORT, State.LANDED):
            return

        now = time.monotonic()
        dt = min(now - self.last_tick_time, 0.5)
        self.last_tick_time = now

        if self.state == State.FOLLOW:
            self._do_follow()
        elif self.state == State.SEARCH_MARKER:
            self._do_marker_search(now)
        elif self.state == State.ALIGN_MARKER:
            self._do_marker_align(now, dt)
        elif self.state == State.DESCEND_MARKER:
            self._do_marker_descend(now, dt)

    def _do_follow(self):
        if self.path is None or not self.path.poses or self.current_pos is None:
            self._hold_position()
            return

        target = self._advance_to_current_waypoint()
        if target is None:
            self.get_logger().info('Goal reached — searching for landing marker.')
            self._enter_marker_search()
            return

        self.hold_pos = None  # actively navigating again; drop any stale anchor
        cx, cy, cz = self.current_pos
        dx, dy, dz = target[0] - cx, target[1] - cy, target[2] - cz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            return

        speed = min(self.cruise_speed, dist)
        wx, wy, wz = (dx / dist) * speed, (dy / dist) * speed, (dz / dist) * speed

        self._send_world_velocity(wx, wy, wz)

    # ── marker landing: SEARCH_MARKER -> ALIGN_MARKER -> DESCEND_MARKER ────────
    #
    # A* always plans to a goal placed exactly at the marker's (x, y) (see
    # planner_params.yaml), so on arrival the marker should already be ~underfoot
    # — SEARCH_MARKER mostly just waits out detector/tracking latency, with a
    # bounded expanding-square scan as a fallback for residual position error
    # rather than the wide-open-ground hunt pl_control's Milestone-1 search does.

    # Body-frame unit vectors for the expanding-square fallback scan: forward,
    # right, back, left.
    _MARKER_SCAN_DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    _MARKER_SEARCH_GRACE_S = 2.0

    def _enter_marker_search(self):
        self.state = State.SEARCH_MARKER
        self.marker_state_entry_time = time.monotonic()
        self.marker_last_seen_time = time.monotonic()
        self.hold_pos = self.current_pos

    def _do_marker_search(self, now: float):
        if self.marker_visible and self.marker_pose is not None:
            self.get_logger().info('Landing marker acquired — aligning.')
            self._enter_marker_align()
            return

        elapsed = now - self.marker_state_entry_time
        if elapsed > self.marker_search_timeout_s:
            self.get_logger().error(
                f'Landing marker not found within {self.marker_search_timeout_s}s '
                'of the goal — holding position rather than landing blind.')
            self._hold_position()
            return

        if elapsed < self._MARKER_SEARCH_GRACE_S:
            self._hold_position()
            return

        step_s = self.marker_search_step_m / self.marker_search_speed
        scan_elapsed = elapsed - self._MARKER_SEARCH_GRACE_S
        leg, t = 0, 0.0
        while True:
            leg_dur = (leg // 2 + 1) * step_s
            if t + leg_dur > scan_elapsed:
                break
            t += leg_dur
            leg += 1

        dx, dy = self._MARKER_SCAN_DIRS[leg % 4]
        self.bridge.send_velocity_body(
            dx * self.marker_search_speed, dy * self.marker_search_speed, 0.0, 0.0)

    def _enter_marker_align(self):
        self.state = State.ALIGN_MARKER
        self.marker_pi_x.reset()
        self.marker_pi_y.reset()
        self.marker_aligned_count = 0

    def _do_marker_align(self, now: float, dt: float):
        if not self._marker_visible_or_revert(now):
            return

        dx = self.marker_pose.pose.position.x
        dy = self.marker_pose.pose.position.y
        error = math.hypot(dx, dy)

        vx = self.marker_pi_x.update(dx, dt)
        vy = self.marker_pi_y.update(dy, dt)
        self.bridge.send_velocity_body(vx, vy, 0.0, 0.0)

        if error < self.marker_hacc_radius_m:
            self.marker_aligned_count += 1
        else:
            self.marker_aligned_count = 0

        if self.marker_aligned_count >= self.marker_n_frames_aligned:
            self.state = State.DESCEND_MARKER
            self.marker_pi_x.reset()
            self.marker_pi_y.reset()

    def _do_marker_descend(self, now: float, dt: float):
        if not self._marker_visible_or_revert(now):
            return

        dx = self.marker_pose.pose.position.x
        dy = self.marker_pose.pose.position.y
        dz = self.marker_pose.pose.position.z   # AGL distance to marker (optical z)
        error = math.hypot(dx, dy)

        vx = self.marker_pi_x.update(dx, dt)
        vy = self.marker_pi_y.update(dy, dt)

        # Only descend when on-target; pause vz if drifting (do NOT reverse).
        if error < self.marker_hacc_radius_m * 2.0:
            vz = min(self.marker_descend_vel, dz * 0.3)
        else:
            vz = 0.0

        self.bridge.send_velocity_body(vx, vy, vz, 0.0)

        if dz < self.marker_final_land_alt_m:
            self.get_logger().info(
                f'Below {self.marker_final_land_alt_m}m — handing off to Action.land().')
            self.state = State.LANDED
            threading.Thread(target=self._land_async, daemon=True).start()

    def _marker_visible_or_revert(self, now: float) -> bool:
        """Return True if the marker is currently visible; otherwise, once
        it's been gone longer than marker_lost_timeout_s, revert to
        SEARCH_MARKER rather than continuing to align/descend on a stale
        pose."""
        if self.marker_visible and self.marker_pose is not None:
            self.marker_last_seen_time = now
            return True
        lost_for = now - self.marker_last_seen_time
        if lost_for > self.marker_lost_timeout_s:
            self.get_logger().warn(f'Landing marker absent {lost_for:.1f}s — reverting to search.')
            self._enter_marker_search()
        return False

    def _land_async(self):
        try:
            self.bridge.run(self.bridge.land(), timeout=60.0)
            self.get_logger().info('Landed successfully.')
        except Exception as exc:
            self.get_logger().error(f'Action.land() failed: {exc}')

    def _hold_position(self):
        """Actively station-keep at a fixed anchor point, rather than just
        sending zero velocity: PX4 doesn't hold altitude perfectly under a
        constant zero-velocity setpoint, and over an extended hold (e.g.
        while the planner repeatedly fails to find a path) that drift is
        enough to settle onto the floor — which then reads as "occupied",
        permanently blocking any further path from being found at all."""
        if self.current_pos is None:
            return
        if self.hold_pos is None:
            self.hold_pos = self.current_pos

        cx, cy, cz = self.current_pos
        hx, hy, hz = self.hold_pos
        ex, ey, ez = hx - cx, hy - cy, hz - cz

        wx = max(-self.hold_max_speed, min(self.hold_max_speed, self.hold_kp * ex))
        wy = max(-self.hold_max_speed, min(self.hold_max_speed, self.hold_kp * ey))
        wz = max(-self.hold_max_speed, min(self.hold_max_speed, self.hold_kp * ez))

        self._send_world_velocity(wx, wy, wz)

    def _send_world_velocity(self, wx: float, wy: float, wz: float):
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
