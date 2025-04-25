import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import numpy as np

# can change one index angle
# chnage the angle of all joints
# can increase or decrease a single joint angle


class ArmCummuteNode(Node):
    def __init__(self, arm_params, arm_angle_control):
        super().__init__("arm_commute_node")
        self.arm_angle_control = arm_angle_control
        # Load parameters first
        self.arm_params = arm_params.get_arm_params()

        # Initialize arm parameters
        self.arm_pub = self.create_publisher(
            JointTrajectoryPoint, self.arm_params["global"]["arm_topic"], 10
        )

    def degrees_to_radians(self, degree_positions):
        """Convert a list of positions from degrees to radians using NumPy

        Args:
            degree_positions (list): Joint positions in degrees

        Returns:
            list: Joint positions in radians
        """
        try:
            # Convert to numpy array, then use np.deg2rad for efficient conversion
            positions_array = np.array(degree_positions, dtype=float)
            radian_positions = np.deg2rad(positions_array).tolist()
            return radian_positions
        except (ValueError, TypeError) as e:
            # Fall back to element-by-element conversion if array conversion fails
            self.get_logger().warn(
                f"Could not convert all values at once: {e}, falling back to individual conversion"
            )
            radian_positions = []
            for pos in degree_positions:
                try:
                    radian_positions.append(float(pos) * math.pi / 180.0)
                except (ValueError, TypeError):
                    self.get_logger().error(f"Invalid angle value: {pos}")
                    radian_positions.append(0.0)
            return radian_positions

    def publish_arm_angle(self):
        """Publish the current arm joint angles"""
        joint_positions = self.arm_angle_control.get_arm_angles()
        msg = JointTrajectoryPoint()
        radian_positions = self.degrees_to_radians(joint_positions)
        msg.positions = radian_positions
        msg.velocities = []
        msg.accelerations = []
        msg.effort = []
        msg.time_from_start.sec = 0
        msg.time_from_start.nanosec = 0
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Published angles in radians: {radian_positions}")
