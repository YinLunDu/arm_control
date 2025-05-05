import threading
import rclpy
from keyboard_mode_interface_pkg.utils.arm_controller import ArmController
from .data_processor import DataProcessor
from .ros_communicator import RosCommunicator
from keyboard_mode_interface_pkg.utils.ik_solver import PybulletRobotController
from keyboard_mode_interface_pkg.utils.MenuApp import ModeApp


def init_ros_node():
    rclpy.init()  # 初始化 ROS2
    node = RosCommunicator() # 建立自定義的 ROS node（負責訂閱/發布/服務等）
    thread = threading.Thread(target=rclpy.spin, args=(node,)) # 用 thread 跑 rclpy.spin
    thread.start()
    return node, thread
"""
初始化 ROS2 環境

建立你的 ROS 節點（RosCommunicator）

用一條獨立的 thread 執行 ROS 通訊（非同步 spin）
"""
def main():
    ros_communicator, ros_thread = init_ros_node()#這行程式碼的作用是呼叫 init_ros_node() 函式，並將它回傳的兩個值分別指定給變數 ros_communicator 和 ros_thread
    data_processor = DataProcessor(ros_communicator)
    ik_solver = PybulletRobotController(end_eff_index=6)
    arm_controller = ArmController(
        ros_communicator, data_processor, ik_solver, num_joints=8
    )
    app = ModeApp(arm_controller)

    try:
        app.main()
    finally:
        rclpy.shutdown() # 結束 ROS2
        ros_thread.join()# 等待 ros spin thread 結束


if __name__ == "__main__":
    main()
