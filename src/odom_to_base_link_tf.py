#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math

class OdomToBaseLinkTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_to_base_link_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to /odom topic to get robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',              # Adjust topic name if your odom topic is different
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'           # parent frame
        t.child_frame_id = 'base_link'       # child frame

        # Get position from odom message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Get orientation quaternion from odom message
        t.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().debug(f'Published odom->base_link transform')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseLinkTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

