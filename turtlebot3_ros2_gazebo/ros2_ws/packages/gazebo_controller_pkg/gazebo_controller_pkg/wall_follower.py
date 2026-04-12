#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy

def safe_min(ranges, default=5.0):
    valid = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.05]
    return min(valid) if valid else default

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.path_pub = self.create_publisher(Path, '/wall_follower_path', 10)

        self.target_dist = 0.3
        self.search_range = 1.5
        self.front_safe = 0.4
        self.linear_speed = 0.2
        self.kp = 2.0
        self.deadzone = 0.03

        self.min_valid_range = 0.05
        self.default_range = 5.0

        self.front_left_idx_start = 350
        self.front_left_idx_end = 360
        self.front_right_idx_start = 0
        self.front_right_idx_end = 10

        self.right_idx_start = 260
        self.right_idx_end = 280

        self.turn_right_speed = -0.4
        self.turn_left_speed = 0.6
        self.max_angular = 0.5
        self.wall_detect_threshold = 0.15

        self.state = 'find_wall'
        self.path = []

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.path.append((pos.x, pos.y))

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.pose.orientation = msg.pose.pose.orientation

        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def on_scan(self, msg):
        front = safe_min(
            msg.ranges[self.front_left_idx_start:self.front_left_idx_end] +
            msg.ranges[self.front_right_idx_start:self.front_right_idx_end],
            self.default_range
        )
        right = safe_min(
            msg.ranges[self.right_idx_start:self.right_idx_end],
            self.default_range
        )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.state == 'find_wall':
            self.get_logger().info(f"[FIND_WALL] front={front:.3f}, right={right:.3f}")

            if front < self.front_safe:
                self.get_logger().info("[FIND_WALL] Obstacle ahead -> TURN")
                self.state = 'turn'
            
            elif abs(right - self.target_dist) < self.wall_detect_threshold:
                self.get_logger().info("[FIND_WALL] Wall detected at correct distance -> FOLLOW")
                self.state = 'follow'
            
            else:
                self.get_logger().info("[FIND_WALL] Searching wall: moving forward + turning right")
                cmd.twist.linear.x = self.linear_speed
                cmd.twist.angular.z = self.turn_right_speed

        elif self.state == 'turn':
            self.get_logger().info(f"[TURN] front={front:.3f}")

            if front > self.front_safe:
                self.get_logger().info("[TURN] Path is clear -> FOLLOW")
                self.state = 'follow'
            else:
                self.get_logger().info("[TURN] Turning in place (left)")
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_left_speed

        elif self.state == 'follow':
            self.get_logger().info(f"[FOLLOW] front={front:.3f}, right={right:.3f}")

            if right > self.search_range or right < self.min_valid_range:
                self.get_logger().warn("[FOLLOW] Lost wall -> FIND_WALL")
                self.state = 'find_wall'
            
            elif front < self.front_safe:
                self.get_logger().info("[FOLLOW] Obstacle ahead -> TURN")
                self.state = 'turn'
            
            else:
                error = right - self.target_dist
                self.get_logger().info(f"[FOLLOW] error={error:.3f}")

                if abs(error) < self.deadzone:
                    self.get_logger().info("[FOLLOW] Within deadzone -> go straight")
                    cmd.twist.angular.z = 0.0
                else:
                    control = -self.kp * error
                    self.get_logger().info(f"[FOLLOW] Control before clamp={control:.3f}")
                    cmd.twist.angular.z = control

                cmd.twist.angular.z = max(-self.max_angular, min(self.max_angular, cmd.twist.angular.z))
                self.get_logger().info(f"[FOLLOW] Control after clamp={cmd.twist.angular.z:.3f}")

                cmd.twist.linear.x = self.linear_speed
                self.get_logger().info("[FOLLOW] Moving forward")

        self.pub.publish(cmd)

    def save_path(self):
        with open("path.txt", "w") as f:
            for x, y in self.path:
                f.write(f"{x} {y}\n")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_path()

        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = node.get_clock().now().to_msg()
        stop_cmd.header.frame_id = 'base_link'
        node.pub.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
