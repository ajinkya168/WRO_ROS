#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped
import math


class MapToBaseLinkPose(Node):
    def __init__(self):
        super().__init__('map_to_base_link_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.get_pose)

    def get_pose(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))

            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Convert quaternion to Euler angles to get yaw
            quat = trans.transform.rotation
            (_, _, yaw) = tf_transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w]
            )

            self.get_logger().info(f"Robot Pose -> X: {x:.2f}, Y: {y:.2f}, Yaw: {math.degrees(yaw):.2f}°")

        except Exception as e:
            self.get_logger().warn(f"Could not transform: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = MapToBaseLinkPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

