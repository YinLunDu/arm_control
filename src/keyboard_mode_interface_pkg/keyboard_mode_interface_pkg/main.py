
import rclpy
from keyboard_mode_interface_pkg.ros_communicator import RosCommunicator

import threading
from keyboard_mode_interface_pkg.utils.MenuApp import MenuApp

def main():
    rclpy.init()
    ros_manager = RosCommunicator()


    # Create and start ROS spin thread
    ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_manager))
    ros_spin_thread.daemon = True
    ros_spin_thread.start()

    # Run UI in main thread
    app = MenuApp(ros_manager)
    try:
        app.run()
    finally:
        rclpy.shutdown()
        ros_manager.destroy_node()


if __name__ == "__main__":
    main()
