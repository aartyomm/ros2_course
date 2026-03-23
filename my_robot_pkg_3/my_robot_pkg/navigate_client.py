#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_pkg.action import Navigate

class NavigateClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self.cli = ActionClient(self, Navigate, 'navigate')

    def send_goal(self):
        self.cli.wait_for_server()

        goal = Navigate.Goal()
        goal.target_x = 5.0
        goal.target_y = 3.0

        future = self.cli.send_goal_async(
            goal,
            feedback_callback=self.on_feedback
        )

        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        r = result_future.result()

        self.get_logger().info(
            f'Done! dist={r.result.total_distance:.1f} t={r.result.elapsed_time:.1f}s'
        )

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'pos=({fb.current_x:.1f},{fb.current_y:.1f}) '
            f'remaining={fb.remaining_dist:.1f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigateClient()
    node.send_goal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
