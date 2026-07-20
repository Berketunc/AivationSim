#!/usr/bin/env python3
"""
Aligns OpenVINS' odometry into this project's world frame.

VIO has no absolute reference: its "global" frame is wherever (and however
oriented) the estimator happened to initialize, not Gazebo's world frame that
sim_assets/worlds/warehouse.sdf and oa_planning's occupancy grid are built in.
Roll/pitch end up close to true world values (both are gravity-aligned, and
OpenVINS observes gravity directly from the IMU) but yaw and position are
whatever OpenVINS's internal convention gives them.

Since the vehicle starts stationary at a known pose (PX4_GZ_MODEL_POSE in
sim_obstacle_avoidance.launch.py's docstring), this node calibrates a fixed
correction transform from the very first VIO pose it sees to that known
starting pose, then applies the same fixed transform to every pose after —
not a continuous re-alignment, just removing VIO's arbitrary initial offset
once. Only pose (position + orientation) is corrected; nothing downstream
(odom_to_tf_node, oa_planning, oa_control) reads velocity/twist from
odometry, so it's passed through uncorrected rather than transformed too.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _quat_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a (x,y,z,w) quaternion, as row-major tuples."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)),
    )


def _matrix_to_quat(m):
    """(x,y,z,w) quaternion from a 3x3 row-major rotation matrix."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = m
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matT(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _matvec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


class VioOdomToWorld(Node):

    def __init__(self):
        super().__init__('vio_odom_to_world')

        self.declare_parameter('vio_odom_topic', '/oa/vio/odomimu')
        self.declare_parameter('world_odom_topic', '/oa/odom')
        # Must match the PX4_GZ_MODEL_POSE this world's launch instructions use.
        self.declare_parameter('spawn_x', -8.5)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.2)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('body_frame_id', 'base_link')

        self._spawn_p = (
            self.get_parameter('spawn_x').value,
            self.get_parameter('spawn_y').value,
            self.get_parameter('spawn_z').value,
        )
        yaw = self.get_parameter('spawn_yaw').value
        self._spawn_R = (
            (math.cos(yaw), -math.sin(yaw), 0.0),
            (math.sin(yaw), math.cos(yaw), 0.0),
            (0.0, 0.0, 1.0),
        )
        self._world_frame = self.get_parameter('world_frame_id').value
        self._body_frame = self.get_parameter('body_frame_id').value

        self._R_correction = None  # set on first VIO message
        self._t_correction = None
        self._warned_nonfinite = False

        self._pub = self.create_publisher(
            Odometry, self.get_parameter('world_odom_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('vio_odom_topic').value, self._on_vio_odom, 10)

    def _on_vio_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p_vio = (p.x, p.y, p.z)

        # The raw VIO estimate can and does go non-finite on divergence (see
        # MILESTONE2_STATUS.md Component 6) — without this check that NaN/inf
        # flows straight through matrix math (silently producing more NaN)
        # and out onto /oa/odom, which odom_to_tf_node then rebroadcasts as
        # TF on every message, at whatever rate OpenVINS publishes (IMU
        # rate) — flooding downstream consumers like octomap_server with a
        # TF_NAN_INPUT error per message instead of failing visibly once.
        # Drop the message here instead: everything downstream already
        # treats "no fresh odom" as hold-position/no-plan, which is the
        # correct behavior while VIO is unusable, not a code path that needs
        # separate handling.
        if not all(math.isfinite(v) for v in (p_vio[0], p_vio[1], p_vio[2], q.x, q.y, q.z, q.w)):
            if not self._warned_nonfinite:
                self.get_logger().error(
                    'VIO odometry is non-finite (estimator has diverged) — dropping '
                    'this and all further non-finite messages until it recovers.')
                self._warned_nonfinite = True
            return
        self._warned_nonfinite = False

        R_vio = _quat_to_matrix(q.x, q.y, q.z, q.w)

        if self._R_correction is None:
            # R_c = R_spawn * R_vio0^T ; t_c = p_spawn - R_c * p_vio0
            self._R_correction = _matmul(self._spawn_R, _matT(R_vio))
            self._t_correction = tuple(
                self._spawn_p[i] - _matvec(self._R_correction, p_vio)[i] for i in range(3)
            )
            self.get_logger().info(
                f'Calibrated VIO->world correction against spawn pose {self._spawn_p}')

        Rc, tc = self._R_correction, self._t_correction
        p_world = tuple(_matvec(Rc, p_vio)[i] + tc[i] for i in range(3))
        R_world = _matmul(Rc, R_vio)
        qx, qy, qz, qw = _matrix_to_quat(R_world)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world_frame
        out.child_frame_id = self._body_frame
        out.pose.pose.position.x = p_world[0]
        out.pose.pose.position.y = p_world[1]
        out.pose.pose.position.z = p_world[2]
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        # Velocity/twist intentionally left zeroed: nothing downstream reads it.
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VioOdomToWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
