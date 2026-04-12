#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.pub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.get_logger().info(f'x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
