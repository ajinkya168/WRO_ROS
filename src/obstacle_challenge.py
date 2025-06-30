#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import math


def yaw_to_quaternion(yaw_rad):
    """Convert a yaw (in radians) to a quaternion (z, w only, for 2D)"""
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qz, qw


class MultiGoalWithYaw(Node):
    def __init__(self):
        super().__init__('multi_goal_yaw_client')

        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.color_sub = self.create_subscription(String, '/color_direction', self.color_callback, 10)

        self.current_color = None  # latest seen color ("red", "green", or None)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color_callback(self, msg):
        self.current_color = msg.data.lower()
        self.get_logger().info(f"Color detected: {self.current_color}")

    def send_goal(self, x, y, yaw_deg):
        shift_distance = 0.5  # meters
        yaw_rad = math.radians(yaw_deg)

        # Compute perpendicular shift to yaw
        shift_x, shift_y = 0.0, 0.0

        if self.current_color == 'green':
            shift_x = shift_distance * math.cos(yaw_rad + math.pi / 2)
            shift_y = shift_distance * math.sin(yaw_rad + math.pi / 2)
            self.get_logger().info(f'Shifting LEFT for green block ({shift_x:.2f}, {shift_y:.2f})')

        elif self.current_color == 'red':
            shift_x = shift_distance * math.cos(yaw_rad - math.pi / 2)
            shift_y = shift_distance * math.sin(yaw_rad - math.pi / 2)
            self.get_logger().info(f'Shifting RIGHT for red block ({shift_x:.2f}, {shift_y:.2f})')

        # Apply shift
        x += shift_x
        y += shift_y

        # Prepare goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        qz, qw = yaw_to_quaternion(yaw_rad)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        # Wait for server
        self._client.wait_for_server()
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°')

        send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info('Goal reached.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalWithYaw()

    # 🧭 Replace with your own waypoint sequence (x, y, yaw)
    goals = [
        {'x': -1.0, 'y': -0.15, 'yaw_deg': 270},
        {'x': 0.15, 'y': -1.0, 'yaw_deg': 0},
        {'x': 1.0, 'y': 0.15, 'yaw_deg': 90},
        {'x': -0.15, 'y': 1.0, 'yaw_deg': 180},
        {'x': -1.0, 'y': -0.15, 'yaw_deg': 270},
        {'x': 0.15, 'y': -1.0, 'yaw_deg': 0},
        {'x': 1.0, 'y': 0.15, 'yaw_deg': 90},
        {'x': -0.15, 'y': 1.0, 'yaw_deg': 180},
        {'x': -1.0, 'y': -0.15, 'yaw_deg': 270},
        {'x': 0.15, 'y': -1.0, 'yaw_deg': 0},
        {'x': 1.0, 'y': 0.15, 'yaw_deg': 90},
        {'x': -0.15, 'y': 1.0, 'yaw_deg': 180},
    ]

    for goal in goals:
        success = node.send_goal(goal['x'], goal['y'], goal['yaw_deg'])
        if not success:
            node.get_logger().error('Stopping due to goal failure.')
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

