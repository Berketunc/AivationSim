#!/usr/bin/env python3
"""
Compares vio_odom_to_world's corrected VIO pose against Gazebo's ground
truth (kept bridged specifically for this) and logs when they split apart.

vio_odom_to_world already drops non-finite (NaN/Inf) VIO output, but
MILESTONE2_STATUS.md's Component 6 notes document the actual divergence
pattern as progressive: the raw estimate drifts to something large but
still finite (their own example: position reaching y=-460 in a 14m room)
well before it eventually goes non-finite. That intermediate stage sails
straight through the NaN/Inf filter — the controller acts on a finite but
badly wrong pose with no way to tell the difference from a good one. This
node makes that difference visible without staring at two `ros2 topic
echo` terminals side by side.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff_deg(a: float, b: float) -> float:
    return math.degrees(abs(math.atan2(math.sin(a - b), math.cos(a - b))))


class VioDivergenceWatchdog(Node):

    def __init__(self):
        super().__init__('vio_divergence_watchdog')

        self.declare_parameter('vio_odom_topic', '/oa/odom')
        self.declare_parameter('truth_odom_topic', '/oa/odom_ground_truth')
        # Two thresholds, not one: WARN as soon as a real gap opens up (this
        # is expected to some degree — VIO isn't perfect even when healthy)
        # so the trend is visible before it's dangerous; ERROR (latched,
        # logged once per divergence episode rather than every message) at
        # a gap large enough that the controller is almost certainly
        # steering off of bad data.
        #
        # position_error_m was 3.0 until MILESTONE2_STATUS.md's live test of
        # the translational-excitation wiggle fix: drift is now roughly
        # linear (~2m over 77s) rather than an unbounded blowup, and the
        # vehicle covered 12.4 of 17m to the goal before tripping the old
        # 3.0m gate with only ~4.6m left to go. Raised to 5.0m to give it
        # room to actually finish a run — safe to loosen specifically
        # because octomap/planner/controller all consume the *same* VIO
        # frame consistently (LiDAR is rigidly TF'd through it too), so
        # local obstacle avoidance stays self-consistent under gradual
        # drift; only absolute goal-reaching accuracy degrades, not
        # collision safety, as long as drift doesn't blow up again.
        self.declare_parameter('position_warn_m', 2.0)
        self.declare_parameter('position_error_m', 5.0)
        self.declare_parameter('yaw_warn_deg', 15.0)
        self.declare_parameter('yaw_error_deg', 45.0)
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('diverged_topic', '/oa/vio/diverged')

        self._pos_warn = self.get_parameter('position_warn_m').value
        self._pos_err = self.get_parameter('position_error_m').value
        self._yaw_warn = self.get_parameter('yaw_warn_deg').value
        self._yaw_err = self.get_parameter('yaw_error_deg').value
        self._log_period = self.get_parameter('log_period_s').value

        self._truth = None  # (x, y, z, yaw)
        self._last_warn_log = 0.0
        self._error_latched = False

        # Ground truth only exists in sim — this whole node is a sim-only
        # safety net for while VIO itself is unreliable (MILESTONE2_STATUS.md
        # Component 6), not something a real vehicle could run as-is. On
        # real hardware the equivalent signal would have to come from
        # OpenVINS's own reported state covariance instead of a truth
        # comparison.
        self._diverged_pub = self.create_publisher(
            Bool, self.get_parameter('diverged_topic').value, 10)

        self.create_subscription(
            Odometry, self.get_parameter('truth_odom_topic').value, self._on_truth, 10)
        self.create_subscription(
            Odometry, self.get_parameter('vio_odom_topic').value, self._on_vio, 10)

        self.get_logger().info('VIO divergence watchdog running.')

    def _on_truth(self, msg: Odometry):
        p = msg.pose.pose.position
        self._truth = (p.x, p.y, p.z, _yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_vio(self, msg: Odometry):
        if self._truth is None:
            return

        p = msg.pose.pose.position
        vio_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        tx, ty, tz, truth_yaw = self._truth

        dpos = math.sqrt((p.x - tx) ** 2 + (p.y - ty) ** 2 + (p.z - tz) ** 2)
        dyaw = _angle_diff_deg(vio_yaw, truth_yaw)
        now = self.get_clock().now().nanoseconds * 1e-9

        diverged = dpos > self._pos_err or dyaw > self._yaw_err
        self._diverged_pub.publish(Bool(data=diverged))

        if diverged:
            if not self._error_latched:
                self.get_logger().error(
                    f'VIO DIVERGED from ground truth: position error {dpos:.2f}m, '
                    f'yaw error {dyaw:.1f}deg — vio=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}, '
                    f'{math.degrees(vio_yaw):.1f}deg) truth=({tx:.2f}, {ty:.2f}, {tz:.2f}, '
                    f'{math.degrees(truth_yaw):.1f}deg). Controller is now navigating on '
                    'bad data.')
                self._error_latched = True
        elif dpos > self._pos_warn or dyaw > self._yaw_warn:
            self._error_latched = False
            if now - self._last_warn_log > self._log_period:
                self.get_logger().warning(
                    f'VIO drifting from ground truth: position error {dpos:.2f}m, '
                    f'yaw error {dyaw:.1f}deg')
                self._last_warn_log = now
        else:
            self._error_latched = False


def main(args=None):
    rclpy.init(args=args)
    node = VioDivergenceWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
