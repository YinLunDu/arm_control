import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import ArmGoal


class ArmActionServer(Node):
    def __init__(self, arm_commute_node, arm_auto_controller):
        super().__init__("arm_action_server_node")
        self._action_server = ActionServer(
            self,
            ArmGoal,
            "arm_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.arm_commute_node = arm_commute_node
        self.arm_auto_controller = arm_auto_controller
        self.get_logger().info("Arm Action Server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received arm auto request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = ArmGoal.Result()
        mode = goal_handle.request.mode
        self.get_logger().info(f"Executing arm action in mode: {mode}")

        # 選擇對應的自動化方法
        arm_auto_method = self._select_arm_auto_method(mode)
        if arm_auto_method is None:
            self.get_logger().error(f"Unknown mode: {mode}")
            result.success = False
            result.message = f"Unknown mode: {mode}"
            return result

        rate = self.create_rate(10)
        while rclpy.ok():
            rate.sleep()

            # 處理取消請求
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Navigation canceled by user")
                result = ArmGoal.Result(success=False, message="Navigation canceled")
                goal_handle.canceled()
                break

            # 執行自動化方法
            arm_auto_result = arm_auto_method()
            if isinstance(arm_auto_result, ArmGoal.Result):
                if arm_auto_result.success:
                    self.get_logger().info(
                        f"Arm action completed: {arm_auto_result.message}"
                    )
                    goal_handle.succeed()
                else:
                    self.get_logger().error(
                        f"Arm action failed: {arm_auto_result.message}"
                    )
                    goal_handle.abort()
                result = arm_auto_result
                break

            # 發佈反饋
            self._publish_feedback(goal_handle)

        return result

    def _select_arm_auto_method(self, mode: str):
        """
        根據模式選擇對應的 arm_auto_controller 方法
        """
        if mode == "wave":
            return self.arm_auto_controller.arm_wave
        elif mode == "arm_ik_move":
            return self.arm_auto_controller.arm_ik_move
        elif mode == "test":
            return self.arm_auto_controller.test
        else:
            print("no mode")
            return None

    def _check_and_handle_cancel(self, goal_handle, result):
        """
        檢查是否收到取消請求，並處理停止動作與回報
        返回 (canceled: bool, result)
        """
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Arm action canceled by client.")
            goal_handle.canceled()
            result.success = False
            result.message = "Canceled by user"
            return True, result
        return False, result

    def _publish_feedback(self, goal_handle, distance: float = 0.0):
        """
        建立並發佈 Feedback 資訊
        """
        feedback = ArmGoal.Feedback()
        feedback.distance_to_goal = float(distance)
        goal_handle.publish_feedback(feedback)
