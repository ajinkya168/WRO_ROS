#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import ToggleInteractive

class SlamToggleClient(Node):
    def __init__(self):
        super().__init__('slam_toggle_client')
        self.cli = self.create_client(ToggleInteractive, '/slam_toolbox/toggle_interactive_mode')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /slam_toolbox/toggle_interactive_mode service...')

        self.req = ToggleInteractive.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = SlamToggleClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Successfully toggled interactive mode.')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

