#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtle_controller_pkg.srv import RemoveTurtle

class RemoveClient(Node):
    def __init__(self):
        super().__init__('remove_client')
        self.cli = self.create_client(RemoveTurtle, 'remove_turtle')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for remove_turtle service...')

    def send_request(self):
        req = RemoveTurtle.Request()
        return self.cli.call_async(req)


def main():
    rclpy.init()
    node = RemoveClient()

    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(result.message)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
