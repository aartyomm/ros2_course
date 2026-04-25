#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_custom_node')

        self.resolution = 0.05
        self.width = 300
        self.height = 300
        self.origin_x = -7.5
        self.origin_y = -7.5

        self.l_occ = 0.85
        self.l_free = -0.4
        self.l_max = 5.0
        self.l_min = -5.0

        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.got_odom = False

        self.subsample = 2
        self.max_range = 1.5

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.create_timer(1.0, self.publish_map)

        self.get_logger().info('SLAM custom node started')


    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self.got_odom = True
    

    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my
    

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells
    

    def scan_callback(self, msg: LaserScan):
        if not self.got_odom:
            return

        rx, ry = self.world_to_map(self.robot_x, self.robot_y)
        if not (0 <= rx < self.width and 0 <= ry < self.height):
            return

        angle = msg.angle_min
        range_max = min(msg.range_max, self.max_range)

        for i, r in enumerate(msg.ranges):
            if i % self.subsample != 0:
                angle += msg.angle_increment
                continue

            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > range_max:
                angle += msg.angle_increment
                continue
            
            hit = r < range_max
            r_eff = min(r, range_max)

            world_angle = self.robot_theta + angle
            ex = self.robot_x + r_eff * math.cos(world_angle)
            ey = self.robot_y + r_eff * math.sin(world_angle)
            mx, my = self.world_to_map(ex, ey)

            mx = max(0, min(self.width - 1, mx))
            my = max(0, min(self.height - 1, my))

            cells = self.bresenham(rx, ry, mx, my)

            for (cx, cy) in cells[:-1]:
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.log_odds[cy, cx] += self.l_free

            if hit:
                self.log_odds[my, mx] += self.l_occ
            else:
                self.log_odds[my, mx] += self.l_free

            angle += msg.angle_increment

        np.clip(self.log_odds, self.l_min, self.l_max, out=self.log_odds)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        known = np.abs(self.log_odds) > 1e-3
        probs = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        data[known] = (probs[known] * 100).astype(np.int8)

        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)
    

def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
