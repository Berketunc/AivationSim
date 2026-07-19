import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Republishes a nav_msgs/Odometry pose as a TF transform.

    Uses the frame_id / child_frame_id already carried in the Odometry
    message rather than hardcoding them, so this works unchanged whichever
    pose source (Gazebo ground truth now, VIO later) is bridged in.
    """

    def __init__(self):
        super().__init__('odom_to_tf_node')
        self.declare_parameter('odom_topic', '/oa/odom')
        odom_topic = self.get_parameter('odom_topic').value

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
