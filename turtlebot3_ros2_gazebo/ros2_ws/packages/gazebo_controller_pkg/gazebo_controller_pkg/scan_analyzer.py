#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

def safe_min(ranges, default=3.5):
    valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
    return min(valid) if valid else default

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)

    def on_scan(self, msg):
        front = safe_min(msg.ranges[0:15] + msg.ranges[345:360])
        left  = safe_min(msg.ranges[80:100])
        right = safe_min(msg.ranges[260:280])
        self.get_logger().info(f'F={front:.2f} L={left:.2f} R={right:.2f}')
    
def main(args=None):
    rclpy.init(args=args)
    node = ScanAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
