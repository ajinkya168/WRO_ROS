#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        
    def color_callback(self, msg):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0))

            # Current pose
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Always go forward 1.0 meter
            forward_offset = 1.0
            lateral_offset = 0.0

            if msg.data == "green":
                lateral_offset = 0.5  # forward-right
            elif msg.data == "red":
                lateral_offset = -0.5  # forward-left
            else:
                return  # Ignore

            # Compute new goal offset
            dx = forward_offset * math.cos(yaw) - lateral_offset * math.sin(yaw)
            dy = forward_offset * math.sin(yaw) + lateral_offset * math.cos(yaw)

            target_x = x + dx
            target_y = y + dy

            self.send_goal(target_x, target_y, yaw)

        except TransformException as e:
            self.get_logger().warn(f'TF error: {str(e)}')


    def send_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        yaw_rad = math.radians(yaw_deg)
        qz, qw = yaw_to_quaternion(yaw_rad)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._client.wait_for_server()
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw_deg}°')
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

    # List of goals with yaw in degrees
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

