#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        
        self.linear_axis = 1   # Usually left stick vertical
        self.angular_axis = 3 # Usually right stick horizontal
        self.linear_scale = 0.5
        self.angular_scale = 0.5

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Joystick teleop node started.')

    def joy_callback(self, msg: Joy):
        twist = Twist()
        #self.get_logger().info(f"Axes: {msg.axes}")
        if len(msg.axes) > max(self.linear_axis, self.angular_axis):
            twist.linear.x = self.linear_scale * msg.axes[self.linear_axis]
            twist.angular.z = self.angular_scale * msg.axes[self.angular_axis]
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().warn('Joystick axis index out of range.')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

