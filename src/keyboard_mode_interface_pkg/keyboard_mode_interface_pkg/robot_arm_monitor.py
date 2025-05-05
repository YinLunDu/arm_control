import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from keyboard_mode_interface_pkg.model import DeviceDataTypeEnum

class RobotArmMonitor(Node):
    def __init__(self):
        super().__init__('robot_arm_monitor')
        
        self.subscription = self.create_subscription(
            JointTrajectoryPoint,
            DeviceDataTypeEnum.robot_arm,
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        print('Received Joint Positions: {}'.format(msg.positions))
        print('Received Joint Velocities: {}'.format(msg.velocities))
        print('Received Joint Accelerations: {}'.format(msg.accelerations))
        print('Time from start: {}.{} seconds'.format(
            msg.time_from_start.sec, 
            msg.time_from_start.nanosec
        ))
        print('---')

def main(args=None):  
    rclpy.init(args=args)
    monitor = RobotArmMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
