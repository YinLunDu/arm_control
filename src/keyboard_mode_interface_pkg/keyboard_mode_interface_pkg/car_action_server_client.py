from action_interface.action import NavGoal
from keyboard_mode_interface_pkg.base_action_client import BaseActionClient


class CarActionClient(BaseActionClient):
    def __init__(self, node):
        """
        Initializes the CarActionClient.

        Args:
            node: The rclpy.node.Node instance to use for communication.
        """
        super().__init__(
            node=node,
            action_type=NavGoal,
            server_name="nav_action_server",
            client_name="CarActionClient",
        )

    def _create_goal_msg(self, mode):
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode
        return goal_msg

    # Public API methods with descriptive names
    def send_navigation_goal(self, mode):
        """Send a navigation goal with the specified mode"""
        return self.send_goal(mode)

    def cancel_navigation_goal(self):
        """Cancel the current navigation goal if one exists"""
        return self.cancel_goal()
