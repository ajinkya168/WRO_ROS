#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class RVizDummyObstacle(Node):
    def __init__(self):
        super().__init__('rviz_dummy_obstacle')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/dummy_marker', 10)

        self.spawned = False
        self.detected_points = []

        self.marker_id = 0
    



    def scan_callback(self, msg: LaserScan):
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.2 < r < 1.0:  # Threshold to detect an object
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                self.add_marker_if_new(x, y)
            angle += msg.angle_increment

    def add_marker_if_new(self, x, y):
        for (px, py) in self.detected_points:
            if abs(x - px) < 0.1 and abs(y - py) < 0.1:
                return  # Already marked

        self.detected_points.append((x, y))
        self.publish_marker(x, y)

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"  # or "map" if you transform coordinates
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dummy"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Published dummy marker to RViz.")

def main(args=None):
    rclpy.init(args=args)
    node = RVizDummyObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

