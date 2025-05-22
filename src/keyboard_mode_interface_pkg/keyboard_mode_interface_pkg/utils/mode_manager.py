import urwid
from keyboard_mode_interface_pkg.utils.base_mode import BaseMode


class ArmMode(BaseMode):
    submodes = ["0", "1", "2", "3", "4","5", "6"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.arm_controller.manual_control(int(submode), key)

        self.show_submode_screen(
            message=f"Arm Mode: Submode {submode}\nPress 'q' to go back.", on_key=on_key
        )


class AutoArmMode(BaseMode):
    submodes = ["auto_arm_human"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.arm_controller.auto_control(mode=submode, key=key)

        self.show_submode_screen(
            message=f"AutoArm Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key,
        )