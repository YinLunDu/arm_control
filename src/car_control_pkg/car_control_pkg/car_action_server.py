import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from car_control_pkg.car_control_common import BaseCarControlNode

from car_control_pkg.car_nav_controller import NavigationController


class NavigationActionServer(Node):
    def __init__(self, car_control_node):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.car_control_node = car_control_node
        self.nav_controller = NavigationController(self.car_control_node)
        self.get_logger().info("Navigation Action Server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        requested_mode = goal_request.mode
        if requested_mode == "Manual_Nav":
            car_position, _ = self.car_control_node.get_car_position_and_orientation()
            path_points = self.car_control_node.get_path_points(
                include_orientation=True
            )
            if not car_position or not path_points:
                self.get_logger().error("Cannot start navigation: Missing data")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        self.car_control_node.publish_control("STOP")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Navigation action callback"""
        result = NavGoal.Result()

        rate = self.create_rate(10)
        self.nav_controller.reset_index()
        while rclpy.ok():
            # First give executor time to process callbacks
            rate.sleep()
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Navigation canceled by user")
                self.car_control_node.publish_control("STOP")
                result = NavGoal.Result(success=False, message="Navigation canceled")
                goal_handle.canceled()
                break

            nav_result = self.nav_controller.manual_nav()
            if isinstance(nav_result, NavGoal.Result):
                if nav_result.success:
                    self.get_logger().info(
                        f"Navigation completed: {nav_result.message}"
                    )
                    goal_handle.succeed()
                else:
                    self.get_logger().error(f"Navigation failed: {nav_result.message}")
                    goal_handle.abort()
                # Exit the loop and return the result once a final state is reached.
                result = nav_result
                break

            # Publish feedback if navigation is ongoing
            feedback_msg = NavGoal.Feedback()
            feedback_msg.distance_to_goal = float(0.0)
            goal_handle.publish_feedback(feedback_msg)

        return result
