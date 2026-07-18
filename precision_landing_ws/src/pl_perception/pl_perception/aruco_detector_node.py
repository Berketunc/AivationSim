#!/usr/bin/env python3

import cv2
import cv2.aruco
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_m', 0.6)
        self.declare_parameter('aruco_dict', 'DICT_5X5_50')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size_m = self.get_parameter('marker_size_m').value
        dict_name = self.get_parameter('aruco_dict').value

        cv_major = int(cv2.__version__.split('.')[0])
        cv_minor = int(cv2.__version__.split('.')[1])
        self.get_logger().info(f'OpenCV {cv2.__version__}')
        if (cv_major, cv_minor) < (4, 7):
            raise RuntimeError(
                f'OpenCV >= 4.7 required for ArucoDetector, found {cv2.__version__}'
            )

        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None

        self.create_subscription(CameraInfo, 'camera_info', self._camera_info_cb, 10)
        self.create_subscription(Image, 'image_raw', self._image_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.visible_pub = self.create_publisher(Bool, 'is_visible', 10)

        self.get_logger().info(
            f'aruco_detector_node ready  marker_id={self.marker_id}  '
            f'size={self.marker_size_m}m  dict={dict_name}'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg: Image):
        visible = Bool()

        if self.camera_matrix is None:
            self.get_logger().warn(
                'No camera_info yet — skipping frame.', throttle_duration_sec=5.0
            )
            visible.data = False
            self.visible_pub.publish(visible)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or self.marker_id not in ids.flatten():
            visible.data = False
            self.visible_pub.publish(visible)
            return

        idx = list(ids.flatten()).index(self.marker_id)
        target_corners = corners[idx]

        # Object points in marker frame: top-left, top-right, bottom-right, bottom-left
        h = self.marker_size_m / 2.0
        obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)

        img_pts = target_corners.reshape(4, 2).astype(np.float64)

        ok, _rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            visible.data = False
            self.visible_pub.publish(visible)
            return

        tvec = tvec.flatten()

        # ── Camera optical → body FRD ──────────────────────────────────────────
        # x500_mono_cam_down mounts mono_cam with pitch=pi/2 (Ry(90 deg)) from base_link.
        # Verified from model.sdf: camera +y_cam = body +y (right); +z_cam = body +x (fwd).
        # OpenCV optical frame: z into scene, x right in image, y down in image.
        #   tvec[0] = x_opt = body right
        #   tvec[1] = y_opt = body backward  -> negate for body forward
        #   tvec[2] = z_opt = AGL distance to marker (always positive)
        dx_body = -tvec[1]  # body forward
        dy_body =  tvec[0]  # body right
        dz_body =  tvec[2]  # body down = AGL distance
        # ─────────────────────────────────────────────────────────────────────

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'body_frd'
        pose.pose.position.x = dx_body
        pose.pose.position.y = dy_body
        pose.pose.position.z = dz_body
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)
        visible.data = True
        self.visible_pub.publish(visible)

        self.get_logger().debug(
            f'marker {self.marker_id}: '
            f'body dx={dx_body:+.3f} dy={dy_body:+.3f} dz={dz_body:.3f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
