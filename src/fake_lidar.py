#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from std_msgs.msg import Header

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_scan)  # 10Hz
        self.declare_parameter('use_sim_time', True)  # for Gazebo

    def publish_fake_scan(self):
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'  # Set your LiDAR frame name

        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [2.0 + 0.5 * math.sin(i * 0.1) for i in range(num_readings)]
        msg.intensities = [1.0 for _ in range(num_readings)]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

