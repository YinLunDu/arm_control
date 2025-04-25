class ModeManager:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager

    def update_mode(self, pressed_key_info):
        title = pressed_key_info.split(":")[-2].replace(" ", "")
        subtitle = pressed_key_info.split(":")[-1]
        if "Control Vehicle" in pressed_key_info:
            car_control_signal = f"{title}:{subtitle}"
            if title == "Manual_Nav":
                if subtitle == "q":
                    self.ros_manager.car_action_client.cancel_navigation_goal()
                else:
                    self.ros_manager.car_action_client.send_navigation_goal(
                        mode="Manual_Nav"
                    )
            else:
                self.ros_manager.publish_car_signal(car_control_signal)

        elif "Manual Arm Control" in pressed_key_info:
            arm_control_signal = f"{title}:{subtitle}"
            self.ros_manager.publish_arm_signal(arm_control_signal)
        elif "Automatic Arm Mode" in pressed_key_info:
            arm_control_signal = f"{title}:{subtitle}"
            if title == "test" or title == "arm_ik_move":
                if subtitle == "q":
                    self.ros_manager.arm_action_client.cancel_arm()
                else:
                    self.ros_manager.arm_action_client.send_arm_mode(mode=title)

        elif "Manual Crane Control" in pressed_key_info:
            pass
        # else:
        #     print("Invalid key pressed")
