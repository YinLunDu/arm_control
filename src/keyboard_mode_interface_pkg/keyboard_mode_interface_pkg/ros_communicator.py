from rclpy.node import Node
from .model import DeviceDataTypeEnum
from trajectory_msgs.msg import JointTrajectoryPoint

class RosCommunicator(Node):
    def __init__(self):
        super().__init__("RosCommunicator")
        self.publisher_joint_trajectory = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )
    # publish robot arm angle
    def publish_robot_arm_angle(self, angle):
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = angle
        joint_trajectory_point.velocities = [0.0] * len(angle)
        self.publisher_joint_trajectory.publish(joint_trajectory_point)



