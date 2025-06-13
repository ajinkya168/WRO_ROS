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
        self.color_sub = self.create_subscription(String, '/color_direction', self.color_callback, 10)
        self.spawned = False
        self.detected_points = []

        self.marker_id = 0
    import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import math

class ColorBasedMarker(Node):
    def __init__(self):
        super().__init__('color_based_marker')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.color_sub = self.create_subscription(String, '/color_detection', self.color_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/dummy_marker', 10)

        self.last_color = None  # "red" or "green"
        self.color_timeout = self.create_timer(1.0, self.reset_color)  # Reset if too old
        self.detected_points = []
        self.marker_id = 0

    def color_callback(self, msg: String):
        if msg.data.lower() in ["red", "green"]:
            self.last_color = msg.data.lower()
            self.get_logger().info(f"Color detected: {self.last_color}")

    def reset_color(self):
        self.last_color = None  # Expire old detections

    def scan_callback(self, msg: LaserScan):
        if not self.last_color:
            return  # No color seen recently

        angle = msg.angle_min
        for r in msg.ranges:
            if 0.2 < r < 1.0:  # Adjust as needed
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                self.add_marker_if_new(x, y, self.last_color)
                self.last_color = None  # Consume this detection after placing marker
                break  # Only one marker per scan callback
            angle += msg.angle_increment

    def add_marker_if_new(self, x, y, color):
        for (px, py, _) in self.detected_points:
            if abs(x - px) < 0.1 and abs(y - py) < 0.1:
                return  # Already marked

        self.detected_points.append((x, y, color))
        self.publish_marker(x, y, color)

    def publish_marker(self, x, y, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "colored_obstacles"
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

        if color == "red":
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif color == "green":
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Spawned {color} marker at ({x:.2f}, {y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ColorBasedMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




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

