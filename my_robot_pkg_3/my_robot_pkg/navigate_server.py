#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_robot_pkg.action import Navigate

class NavigateServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        ActionServer(
            self, Navigate, 'navigate',
            execute_callback=self.execute,
            goal_callback=lambda goal_request: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT
        )
        self.get_logger().info("server started")

    def execute(self, goal):
        fb = Navigate.Feedback()
        cx, cy = 0.0, 0.0
        tx = goal.request.target_x
        ty = goal.request.target_y
        start = time.time()

        for i in range(1, 11):
            if goal.is_cancel_requested:
                goal.canceled()
                self.get_logger().info('Goal canceled')
                return Navigate.Result()

            cx = tx * i / 10
            cy = ty * i / 10
            fb.current_x, fb.current_y = cx, cy
            fb.remaining_dist = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)
            goal.publish_feedback(fb)
            time.sleep(0.5)

        result = Navigate.Result()
        result.total_distance = math.sqrt(tx ** 2 + ty ** 2)
        result.elapsed_time = time.time() - start
        goal.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigateServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
