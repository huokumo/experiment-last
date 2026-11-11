#!/usr/bin/env python3
"""Bridge the simulator odometry into the TF tree.

``/r1/sim/odometry`` (bridged to ``/odom``) carries ``frame_id=odom`` and
``child_frame_id=base_link`` but a message is not a transform. Nav2 needs the
transform, so this node republishes the same numbers as ``odom -> base_link``.

Where the numbers come from: the planar motion plugin reports the pose the
robot actually has in the simulator, so this is a perfect odometry source.
That is a deliberate choice for a 4-hour lab - the experiment is about the
obstacle cloud, not about drift - and it is stated in the student guide so
nobody mistakes it for a wheel encoder.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('experiment9_odom_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 20)
        self.get_logger().info(f'publishing {self.parent} -> {self.child} from {self.get_parameter("odom_topic").value}')

    def _on_odom(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent
        transform.child_frame_id = self.child
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = None
    try:
        node = OdomToTf()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
