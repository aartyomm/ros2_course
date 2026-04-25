#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

def safe_min(ranges, default=3.5):
    valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
    return min(valid) if valid else default

class ObstacleAvoidance(Node):
    SAFE_DIST = 0.5

    def __init__(self):
        super().__init__('obstacle_avoidance')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
    
    def on_scan(self, msg):
        front = safe_min(msg.ranges[0:15] + msg.ranges[345:360])
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        if front > self.SAFE_DIST:
            cmd.twist.linear.x = 0.2
            cmd.twist.angular.z = 0.0
        else:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.5
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
