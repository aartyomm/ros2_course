#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from turtle_controller_pkg.srv import ChangeBackground

class BackgroundClient(Node):
    def __init__(self):
        super().__init__('background_client')
        self.cli = self.create_client(ChangeBackground, 'change_background')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for change_background service...')

    def send_request(self, r, g, b):
        req = ChangeBackground.Request()
        req.r = r
        req.g = g
        req.b = b
        return self.cli.call_async(req)


def main():
    rclpy.init()
    node = BackgroundClient()

    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)

    future = node.send_request(r, g, b)

    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(result.message)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
