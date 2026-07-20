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
"""

import math
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

from pl_control.mavsdk_bridge import MavsdkBridge


class State(Enum):
    INIT = auto()
    FOLLOW = auto()
    GOAL_REACHED = auto()
    ABORT = auto()
    # Triggered by oa_vio's divergence watchdog (sim-only — compares VIO
    # against Gazebo ground truth, which doesn't exist on real hardware).
    # MILESTONE2_STATUS.md's Component 6 divergence is large-but-finite
    # before it's ever non-finite, so nothing in this controller's own data
    # can tell good pose from bad — without an external check like the
    # watchdog, the vehicle just keeps confidently navigating on garbage
    # (confirmed: this is what sent it into a wall instead of stopping).
    LOCALIZATION_LOST = auto()


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

        # This controller strafes (moves forward/right relative to current
        # heading) and never yaws — yawspeed is always 0 in the normal
        # FOLLOW state below. That's fine for flight, but it means VIO's
        # dynamic initializer (oa_vio) can never get the rotational
        # diversity it needs from normal flight alone, no matter how long it
        # waits — confirmed empirically (gyroscope change stayed ~0.3deg,
        # miles under its 10deg requirement, throughout an entire climb).
        # This one-time post-takeoff yaw wiggle exists purely to give it
        # that diversity before path-following (and its constant-yaw
        # strafing) begins.
        #
        # MavsdkBridge.send_velocity_body's yawspeed is degrees/second
        # (MAVSDK's VelocityBodyYawspeed.yawspeed_deg_s) — a first pass at
        # this used 0.5 thinking it was rad/s, which produced only ~1.6deg
        # of actual rotation (barely above the ~0.3deg baseline) instead of
        # the intended ~57deg per leg.
        #
        # 30deg/s (the original value) rotates ~60deg over a 2s leg — far
        # more than init_dyn_min_deg's 10deg requirement, and per MILESTONE2
        # Component 6, the leading suspect for the post-init divergence is
        # that fast yaw at that rate produces large per-frame optical flow
        # that KLT can't hold onto against the warehouse's flat, low-texture
        # surfaces (already tracking only 34-37 features against a 37.5
        # default threshold with zero rotation). 12deg/s over the same 2s
        # leg still clears the 10deg gate with margin (24deg/leg) while
        # cutting peak angular rate, and therefore per-frame pixel flow,
        # to <=40% of the original.
        self.declare_parameter('vio_wiggle_enabled', True)
        self.declare_parameter('vio_wiggle_yawspeed_deg_s', 12.0)
        self.declare_parameter('vio_wiggle_leg_duration_s', 2.0)
        # Pure in-place rotation satisfies init_dyn_min_deg's *number* but
        # gives monocular VIO none of the translational parallax it
        # actually needs for a well-conditioned scale/depth estimate —
        # rotation-only motion is a textbook-degenerate case for
        # monocular initialization. That's the leading hypothesis, per
        # MILESTONE2_STATUS.md, for why init kept reporting "success" and
        # then diverging almost immediately regardless of how much the
        # tracker/texture/wiggle-rate tuning improved: none of that gave
        # the initializer real baseline, only a cleaner-looking rotation.
        # This adds genuine lateral (sideways) translation alongside the
        # yaw, still in the same symmetric (+,-2,+) pattern so the vehicle
        # roughly returns to where it started rather than drifting toward
        # a wall — sideways relative to the forward-facing camera gives
        # strong parallax on the features already in view, unlike
        # fore/aft motion which barely displaces bearings to what's ahead.
        self.declare_parameter('vio_wiggle_lateral_speed_ms', 0.4)
        # Settle time between reaching offboard control and starting the
        # wiggle: without this, the wiggle's rotation stacks on top of
        # residual post-takeoff/post-climb translational transients, adding
        # to the total motion the tracker has to survive right when the
        # filter is most fragile (just before init). Let those settle out
        # first so the wiggle is the only motion source.
        self.declare_parameter('vio_wiggle_settle_s', 3.0)

        self.cruise_speed: float = self.get_parameter('cruise_speed_ms').value
        self.waypoint_radius: float = self.get_parameter('waypoint_reached_radius_m').value
        self.goal_radius: float = self.get_parameter('goal_reached_radius_m').value
        self.takeoff_alt: float = self.get_parameter('takeoff_altitude_m').value
        mavsdk_addr: str = self.get_parameter('mavsdk_address').value
        timer_hz: float = self.get_parameter('timer_hz').value
        self.vio_wiggle_enabled: bool = self.get_parameter('vio_wiggle_enabled').value
        self.vio_wiggle_yawspeed: float = self.get_parameter('vio_wiggle_yawspeed_deg_s').value
        self.vio_wiggle_leg_s: float = self.get_parameter('vio_wiggle_leg_duration_s').value
        self.vio_wiggle_lateral: float = self.get_parameter('vio_wiggle_lateral_speed_ms').value
        self.vio_wiggle_settle_s: float = self.get_parameter('vio_wiggle_settle_s').value

        self.declare_parameter('hold_position_kp', 0.5)
        self.declare_parameter('hold_position_max_speed_ms', 0.3)
        self.hold_kp: float = self.get_parameter('hold_position_kp').value
        self.hold_max_speed: float = self.get_parameter('hold_position_max_speed_ms').value

        self.declare_parameter('vio_diverged_topic', '/oa/vio/diverged')

        # ── state ─────────────────────────────────────────────────────────────
        self.state = State.INIT
        self.path: Path | None = None
        self.waypoint_idx = 0
        self.current_pos = None   # (x, y, z)
        self.current_yaw = 0.0
        self.hold_pos = None      # anchor point while holding (see _hold_position)
        self.localization_lost = False

        # ── bridge & subscriptions ────────────────────────────────────────────
        self.bridge = MavsdkBridge(system_address=mavsdk_addr)

        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._path_cb, 1)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('vio_diverged_topic').value, self._diverged_cb, 10)

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

    def _diverged_cb(self, msg: Bool):
        self.localization_lost = msg.data

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

            if self.vio_wiggle_enabled:
                self.get_logger().info(
                    f'Holding still for {self.vio_wiggle_settle_s}s to let post-takeoff '
                    'transients die out before wiggling...')
                time.sleep(self.vio_wiggle_settle_s)
                self._do_vio_wiggle()

            self.state = State.FOLLOW
        except Exception as exc:
            self.get_logger().error(f'Startup failed: {exc}')
            self.state = State.ABORT

    def _do_vio_wiggle(self):
        """Yaw one way then the other while also translating sideways,
        purely so OpenVINS's dynamic initializer sees both the rotational
        diversity its init_dyn_min_deg gate requires AND genuine
        translational parallax before normal (constant-yaw) path-following
        begins. See the vio_wiggle_* parameters' comment for why the
        lateral component was added — pure in-place rotation satisfied the
        gate's number without giving monocular VIO real baseline to work
        with."""
        self.get_logger().info('Wiggling (yaw + lateral translation) to help VIO initialize...')
        leg = self.vio_wiggle_leg_s
        yawspeed = self.vio_wiggle_yawspeed
        lateral = self.vio_wiggle_lateral
        for yaw_speed, right_speed in (
            (yawspeed, lateral), (-2 * yawspeed, -2 * lateral), (yawspeed, lateral)
        ):
            end = time.monotonic() + leg
            while time.monotonic() < end:
                self.bridge.send_velocity_body(0.0, right_speed, 0.0, yaw_speed)
                time.sleep(0.05)
        self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('VIO wiggle done.')

    # ── control tick (main thread, ROS timer) ─────────────────────────────────

    def _tick(self):
        # GOAL_REACHED must keep ticking, not just FOLLOW: _hold_position()
        # is an *active* station-keeping correction (see its docstring —
        # PX4 doesn't hold altitude perfectly under a constant velocity
        # setpoint), and it needs to run every tick, not once. A guard of
        # `!= State.FOLLOW` here would call it exactly once, right as the
        # state transitions to GOAL_REACHED below, then never again — the
        # 20Hz offboard loop just keeps re-sending that one stale setpoint
        # forever, with zero further correction, and the vehicle quietly
        # sinks onto the floor. Confirmed as the actual cause of "reaches
        # the goal, then drifts down and lands on its own."
        if self.state not in (State.FOLLOW, State.GOAL_REACHED):
            return

        if self.localization_lost:
            # Confirmed root cause of "flies into a wall for no reason":
            # VIO diverges to something large-but-finite (not NaN, so
            # vio_odom_to_world's own filter doesn't catch it) well before
            # it ever goes non-finite, and both this controller and
            # oa_planning_node keep confidently navigating on it with no
            # way to tell good pose from bad. oa_vio's divergence watchdog
            # is the only thing that can — trust it and get on the ground
            # under PX4's own controlled Action.land() rather than
            # continuing to steer on bad data.
            self.get_logger().error(
                'VIO localization lost (see vio_divergence_watchdog) — landing now '
                'instead of continuing to navigate on bad data.')
            self.bridge.submit(self.bridge.land())
            self.state = State.LOCALIZATION_LOST
            return

        if self.path is None or not self.path.poses or self.current_pos is None:
            self._hold_position()
            return

        target = self._advance_to_current_waypoint()
        if target is None:
            if self.state != State.GOAL_REACHED:
                self.state = State.GOAL_REACHED
                self.get_logger().info('Goal reached — holding position.')
            self._hold_position()
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
