#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud2
import math

class ColorObstacleInjector(Node):
    def __init__(self):
        super().__init__('color_obstacle_injector')
        self.color = None
        self.latest_pose = None

        self.sub_color = self.create_subscription(String, '/color_direction', self.color_cb, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/marker_pose', self.pose_cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/dummy_obstacles', 10)

    def color_cb(self, msg):
        self.color = msg.data.lower()
        self.publish_obstacle()

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.publish_obstacle()

    def publish_obstacle(self):
        if not self.color or not self.latest_pose:
            return

        direction = 1 if self.color == "red" else -1  # red -> right, green -> left
        offset = 0.5  # meters to side

        angle = self.get_yaw_from_quaternion(self.latest_pose.pose.orientation)
        dx = math.cos(angle)
        dy = math.sin(angle)

        # Perpendicular to heading
        px = self.latest_pose.pose.position.x - dy * offset * direction
        py = self.latest_pose.pose.position.y + dx * offset * direction

        cloud = PointCloud()
        cloud.header = self.latest_pose.header
        pt = Point32(x=px, y=py, z=0.0)
        cloud.points.append(pt)

        self.pub.publish(cloud)
        self.get_logger().info(f"Injected virtual obstacle {self.color} at ({px:.2f}, {py:.2f})")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = ColorObstacleInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

