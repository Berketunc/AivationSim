#!/usr/bin/env python3
"""
GPS-denied precision landing controller.

State machine: SEARCH → ALIGN → DESCEND → FINAL_LAND → LANDED
GPS stays ON at the EKF2 level throughout; the control law here uses only
ArUco-derived pose and the optical-z distance for altitude gating.
"""

import math
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from pl_control.mavsdk_bridge import MavsdkBridge


class State(Enum):
    INIT = auto()
    SEARCH = auto()
    ALIGN = auto()
    DESCEND = auto()
    FINAL_LAND = auto()
    LANDED = auto()
    ABORT = auto()


class PIController:

    def __init__(self, kp: float, ki: float, i_max: float, vel_max: float):
        self.kp = kp
        self.ki = ki
        self.i_max = i_max
        self.vel_max = vel_max
        self._integral = 0.0

    def update(self, error: float, dt: float) -> float:
        self._integral = max(-self.i_max, min(self.i_max, self._integral + error * dt))
        return max(-self.vel_max, min(self.vel_max, self.kp * error + self.ki * self._integral))

    def reset(self):
        self._integral = 0.0


class LandingControllerNode(Node):

    def __init__(self):
        super().__init__('landing_controller_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('pi_kp', 0.5)
        self.declare_parameter('pi_ki', 0.05)
        self.declare_parameter('pi_integral_max', 0.5)
        self.declare_parameter('horizontal_vel_max', 1.0)
        self.declare_parameter('descend_vel', 0.3)
        self.declare_parameter('hacc_radius_m', 0.20)
        self.declare_parameter('n_frames_aligned', 10)
        self.declare_parameter('final_land_alt_m', 0.5)
        self.declare_parameter('takeoff_altitude_m', 5.0)
        self.declare_parameter('search_timeout_s', 30.0)
        self.declare_parameter('lost_timeout_s', 2.0)
        self.declare_parameter('mavsdk_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('timer_hz', 20.0)

        kp = self.get_parameter('pi_kp').value
        ki = self.get_parameter('pi_ki').value
        i_max = self.get_parameter('pi_integral_max').value
        v_max = self.get_parameter('horizontal_vel_max').value

        self.pi_x = PIController(kp, ki, i_max, v_max)
        self.pi_y = PIController(kp, ki, i_max, v_max)

        self.descend_vel: float = self.get_parameter('descend_vel').value
        self.hacc_radius: float = self.get_parameter('hacc_radius_m').value
        self.n_aligned_required: int = self.get_parameter('n_frames_aligned').value
        self.final_land_alt: float = self.get_parameter('final_land_alt_m').value
        self.takeoff_alt: float = self.get_parameter('takeoff_altitude_m').value
        self.search_timeout: float = self.get_parameter('search_timeout_s').value
        self.lost_timeout: float = self.get_parameter('lost_timeout_s').value
        mavsdk_addr: str = self.get_parameter('mavsdk_address').value
        timer_hz: float = self.get_parameter('timer_hz').value

        # ── state ─────────────────────────────────────────────────────────────
        self.state = State.INIT
        self.target_pose: PoseStamped | None = None
        self.is_visible = False
        self.aligned_count = 0
        self.state_entry_time = time.monotonic()
        self.last_seen_time = time.monotonic()
        self.last_tick_time = time.monotonic()

        # ── bridge & subscriptions ────────────────────────────────────────────
        self.bridge = MavsdkBridge(system_address=mavsdk_addr)

        self.create_subscription(PoseStamped, 'target_pose', self._pose_cb, 10)
        self.create_subscription(Bool, 'is_visible', self._visible_cb, 10)

        # ── startup on background thread (keeps rclpy spin unblocked) ─────────
        threading.Thread(target=self._startup, daemon=True).start()

        period = 1.0 / timer_hz
        self.create_timer(period, self._tick)

        self.get_logger().info('landing_controller_node initialized')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        self.target_pose = msg

    def _visible_cb(self, msg: Bool):
        was_visible = self.is_visible
        self.is_visible = msg.data
        if msg.data:
            self.last_seen_time = time.monotonic()
        elif was_visible:
            self.get_logger().info('Marker lost.')

    # ── startup (background thread) ───────────────────────────────────────────

    def _startup(self):
        try:
            self.get_logger().info('Connecting to PX4 via MAVSDK…')
            self.bridge.run(self.bridge.connect(), timeout=30.0)

            self.get_logger().info(f'Connected. Arming and taking off to {self.takeoff_alt}m…')
            self.bridge.run(self.bridge.arm_and_takeoff(self.takeoff_alt), timeout=90.0)

            self.get_logger().info('Altitude reached. Starting offboard velocity loop…')
            self.bridge.start_offboard_loop()
            time.sleep(1.0)  # brief settle before handing off to state machine

            self._enter(State.SEARCH)
        except Exception as exc:
            self.get_logger().error(f'Startup failed: {exc}')
            self.state = State.ABORT

    # ── state-machine tick (20 Hz ROS timer, main thread) ─────────────────────

    def _tick(self):
        if self.state in (
            State.INIT, State.FINAL_LAND, State.LANDED, State.ABORT
        ):
            return

        now = time.monotonic()
        dt = min(now - self.last_tick_time, 0.5)
        self.last_tick_time = now

        if self.state == State.SEARCH:
            self._do_search(now)
        elif self.state == State.ALIGN:
            self._do_align(now, dt)
        elif self.state == State.DESCEND:
            self._do_descend(now, dt)

    # ── per-state handlers ────────────────────────────────────────────────────

    def _do_search(self, now: float):
        self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)

        if self.is_visible and self.target_pose is not None:
            self._enter(State.ALIGN)
            return

        if now - self.state_entry_time > self.search_timeout:
            self.get_logger().warn(
                f'SEARCH timeout ({self.search_timeout}s) — aborting to GPS land.'
            )
            self._abort_land()

    def _do_align(self, now: float, dt: float):
        if not self._marker_visible(now):
            return

        dx = self.target_pose.pose.position.x
        dy = self.target_pose.pose.position.y
        error = math.hypot(dx, dy)

        vx = self.pi_x.update(dx, dt)
        vy = self.pi_y.update(dy, dt)
        self.bridge.send_velocity_body(vx, vy, 0.0, 0.0)

        self.get_logger().debug(
            f'ALIGN  dx={dx:+.3f} dy={dy:+.3f} err={error:.3f}  '
            f'vx={vx:+.3f} vy={vy:+.3f}  aligned={self.aligned_count}'
        )

        if error < self.hacc_radius:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        if self.aligned_count >= self.n_aligned_required:
            self._enter(State.DESCEND)

    def _do_descend(self, now: float, dt: float):
        if not self._marker_visible(now):
            return

        dx = self.target_pose.pose.position.x
        dy = self.target_pose.pose.position.y
        dz = self.target_pose.pose.position.z   # AGL distance to marker (optical z)
        error = math.hypot(dx, dy)

        vx = self.pi_x.update(dx, dt)
        vy = self.pi_y.update(dy, dt)

        # Only descend when on-target; pause vz if drifting (do NOT reverse)
        if error < self.hacc_radius * 2.0:
            vz = min(self.descend_vel, dz * 0.3)
        else:
            vz = 0.0

        self.bridge.send_velocity_body(vx, vy, vz, 0.0)

        self.get_logger().debug(
            f'DESCEND  dx={dx:+.3f} dy={dy:+.3f} dz={dz:.2f}  '
            f'vx={vx:+.3f} vy={vy:+.3f} vz={vz:.3f}'
        )

        if dz < self.final_land_alt:
            self._enter(State.FINAL_LAND)
            threading.Thread(target=self._land_async, daemon=True).start()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _marker_visible(self, now: float) -> bool:
        """Return False and revert to SEARCH if marker has been lost too long."""
        if self.is_visible and self.target_pose is not None:
            return True
        lost_for = now - self.last_seen_time
        if lost_for > self.lost_timeout:
            self.get_logger().warn(f'Marker absent {lost_for:.1f}s — reverting to SEARCH.')
            self._enter(State.SEARCH)
        return False

    def _enter(self, new_state: State):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.pi_x.reset()
        self.pi_y.reset()
        self.aligned_count = 0
        self.state = new_state
        self.state_entry_time = time.monotonic()

    def _land_async(self):
        try:
            self.get_logger().info(
                f'Below {self.final_land_alt}m — handing off to Action.land().'
            )
            self.bridge.run(self.bridge.land(), timeout=60.0)
            self.state = State.LANDED
            self.get_logger().info('Landed successfully.')
        except Exception as exc:
            self.get_logger().error(f'Action.land() failed: {exc}')

    def _abort_land(self):
        self.state = State.ABORT
        threading.Thread(target=self._abort_async, daemon=True).start()

    def _abort_async(self):
        try:
            self.get_logger().warn('Abort: triggering GPS-based land at current position.')
            self.bridge.run(self.bridge.land(), timeout=30.0)
        except Exception as exc:
            self.get_logger().error(f'Abort land failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
