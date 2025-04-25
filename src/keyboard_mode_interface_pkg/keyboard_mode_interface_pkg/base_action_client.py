import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import traceback
from keyboard_mode_interface_pkg.action_server_handler import handle_action_result


class BaseActionClient:
    """Base class for action clients with common functionality."""

    def __init__(self, node, action_type, server_name, client_name):
        """
        Initialize the base action client.

        Args:
            node: The ROS2 node
            action_type: The action type to use (e.g., NavGoal, ArmGoal)
            server_name: Name of the action server
            client_name: Human-readable name for this client
        """
        self._node = node
        self._logger = node.get_logger()
        self._server_name = server_name
        self._client_name = client_name

        # Create the action client
        self.action_client = ActionClient(self._node, action_type, server_name)
        self.current_goal_handle = None
        self._get_result_future = None

        self._logger.info(f"{client_name} initialized.")

    def send_goal(self, mode):
        """Send a goal with the specified mode"""
        goal_msg = self._create_goal_msg(mode)

        # Ensure previous goal processing is cleared before sending a new one
        if self.current_goal_handle is not None:
            self._logger.warn(
                "A previous goal might still be active. Cancelling it implicitly is not recommended."
            )
            self.current_goal_handle = None  # Reset handle reference
            if (
                self._get_result_future is not None
                and not self._get_result_future.done()
            ):
                self._get_result_future.cancel()  # Attempt to cancel the future if not done
                self._logger.info("Cancelled previous pending result future.")
            self._get_result_future = None

        # Wait for server with timeout
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self._logger.error(f"{self._server_name} not available!")
            return False

        self._logger.info(f"Sending goal with mode: {mode}")
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def cancel_goal(self):
        """Cancel the current goal if one exists"""
        if self.current_goal_handle is None:
            self._logger.warn("No active goal handle to cancel.")
            return False

        # Check if goal is active - support both approaches
        try:
            # First try using status attribute (newer method)
            current_status = getattr(self.current_goal_handle, "status", None)
            if current_status is not None:
                if current_status not in [
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                ]:
                    self._logger.warn(
                        f"Goal is no longer active (status: {GoalStatus.to_string(current_status)}), cannot cancel."
                    )
                    return False
            # Fall back to is_active property
            elif not self.current_goal_handle.is_active:
                self._logger.warn("Goal is no longer active, cannot cancel.")
                return False
        except AttributeError:
            # If neither approach works, just try to cancel anyway
            pass

        self._logger.info("Requesting goal cancellation...")
        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_result_callback)
            return True
        except Exception as e:
            self._logger.error(f"Error cancelling goal: {e}")
            return False

    def goal_response_callback(self, future):
        """Handles the response after sending a goal."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._logger.info("Goal rejected :(")
                self.current_goal_handle = None
                return

            self._logger.info("Goal accepted :)")
            self.current_goal_handle = goal_handle

            self._get_result_future = self.current_goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        except Exception as e:
            self._logger.error(f"Error in goal response callback: {e}")
            self.current_goal_handle = None

    def cancel_result_callback(self, future):
        """Handles the response after requesting a cancellation."""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                # Handle the UUID properly - it may be a numpy array
                goal_id = cancel_response.goals_canceling[0].goal_id.uuid

                # Convert numpy array to hex string if needed
                if hasattr(goal_id, "hex"):
                    id_str = goal_id.hex()
                else:
                    # Handle numpy array case - convert to bytes then hex
                    import binascii

                    id_bytes = bytes(goal_id)
                    id_str = binascii.hexlify(id_bytes).decode("utf-8")

                self._logger.info(
                    f"Cancellation request acknowledged for goal ID: {id_str}"
                )
            else:
                self._logger.warn(
                    "Cancellation request failed or goal already completed."
                )
        except Exception as e:
            self._logger.error(f"Error processing cancel response: {e}")

    def get_result_callback(self, future):
        """Process the action result, including abort signals."""
        try:
            goal_result = future.result()
            status = goal_result.status
            result = goal_result.result

            handle_action_result(
                node=self._node,
                status=status,
                result=result,
            )

        except Exception as e:
            self._logger.error(f"Error in get_result_callback: {e}")
            self._logger.error(traceback.format_exc())
        finally:
            self.current_goal_handle = None
            self._get_result_future = None

    def feedback_callback(self, feedback_msg):
        """Default feedback callback that can be overridden by derived classes."""
        pass

    def _create_goal_msg(self, mode):
        """Create a goal message with the specified mode.

        Should be implemented by derived classes.
        """
        raise NotImplementedError("Derived classes must implement this method")
