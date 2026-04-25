#!/usr/bin/env python3
import rclpy, sys, termios, tty
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info('WASD to move. Q to quit.')
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        key = sys.stdin.read(1)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        while True:
            key = self.get_key()
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            if key == 'w':
                cmd.twist.linear.x = 0.5
            elif key == 's':
                cmd.twist.linear.x = -0.5
            elif key == 'a':
                cmd.twist.angular.z = 1.0
            elif key == 'd':
                cmd.twist.angular.z = -1.0
            elif key == 'q':
                break
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
