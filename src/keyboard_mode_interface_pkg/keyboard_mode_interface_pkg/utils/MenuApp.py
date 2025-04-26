import urwid
import os
from .arm_controller import ArmController  # Import ArmController

class MenuApp:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.arm_controller = ArmController(ros_manager)  # Initialize ArmController


        # 這個 Text 用來顯示動態的「Pressed key: ...」訊息
        self.pressed_key_text = urwid.Text("", align="center")

        # === 定義選單結構 ===
        self.menu_items = {
            "Manual Arm Control": {str(i): None for i in range(7)},
            "Automatic Arm Mode": {"catch": None, "wave": None},  # 沒有子選單
            "Exit": None,
        }

        # === 初始化主選單 ===
        self.menu_stack = []  # 用於儲存 (menu dict, menu title)
        self.current_menu = self.menu_items
        self.current_title = "Main Menu"

        # 顯示標題和底部訊息的元件
        self.header_text = urwid.Text(self.current_title, align="center")
        self.footer_text = urwid.Text("", align="center")

        # 建立主 ListBox
        self.menu = urwid.ListBox(urwid.SimpleFocusListWalker(self.create_menu()))

        # 組合成 Frame
        self.main_frame = urwid.Frame(
            self.menu, header=self.header_text, footer=self.footer_text
        )

    def create_menu(self):
        """根據 self.current_menu 建立動態選單按鈕"""
        menu_widgets = []
        for item in self.current_menu:
            button = urwid.Button(f"{item}")
            urwid.connect_signal(button, "click", self.menu_selected, item)
            menu_widgets.append(urwid.AttrMap(button, None, focus_map="reversed"))
        return menu_widgets

    def menu_selected(self, button, choice):
        """
        點擊選單選項的回呼：
        - 若對應值是 dict，就進入子選單
        - 若對應值是 None(或其他非 dict)，表示執行指令 (或沒有子選單)
        """
        if isinstance(self.current_menu[choice], dict):
            # --- 進入子選單 ---
            self.menu_stack.append((self.current_menu, self.current_title))
            self.current_menu = self.current_menu[choice]
            self.current_title = choice
            self.header_text.set_text(self.current_title)
            self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())

        else:
            # --- 執行指令 / 或者該大標題本身就沒有子選單 ---
            if choice == "Exit":
                raise urwid.ExitMainLoop()  # 直接結束
            elif choice in [str(i) for i in range(6)]:  # 0~5: 關節調整
                self.enter_joint_control(int(choice))
            elif choice == "6":  # 6: 假爪開闔
                self.toggle_gripper()


    def enter_joint_control(self, joint_index):
        """進入關節控制頁面"""
        self.menu_stack.append((self.current_menu, self.current_title))
        self.current_menu = {}
        self.current_title = f"Joint {joint_index} Control"
        self.header_text.set_text(self.current_title)

        self.menu.body = urwid.SimpleFocusListWalker(
            [
                urwid.Text(f"調整關節 {joint_index} 的角度", align="center"),
                urwid.Divider(),
                urwid.Text("按 'i' 增加角度", align="center"),
                urwid.Text("按 'k' 減少角度", align="center"),
                urwid.Text("按 'q' 返回上一頁", align="center"),
            ]
        )

        def handle_joint_input(key):
            if key == "i":
                result = self.arm_controller.move_joint(joint_index, direction=1)
                self.footer_text.set_text(result)
            elif key == "k":
                result = self.arm_controller.move_joint(joint_index, direction=-1)
                self.footer_text.set_text(result)
            elif key == "q":
                self.handle_unhandled_input("q")

        self.main_frame.unhandled_input = handle_joint_input

    def toggle_gripper(self):
        """控制假爪開闔"""
        result = self.arm_controller.execute_action("catch")
        self.footer_text.set_text(result)

    def handle_unhandled_input(self, key):
        """處理未攔截的按鍵，特別是 'q' 用於返回上一層"""
        if key == "q":
            os.system("clear")
            if self.menu_stack:
                # 從堆疊彈回上一層
                prev_menu, prev_title = self.menu_stack.pop()
                self.current_menu = prev_menu
                self.current_title = prev_title
                self.header_text.set_text(self.current_title)
                new_listbox = urwid.ListBox(
                    urwid.SimpleFocusListWalker(self.create_menu())
                )
                self.menu = new_listbox
                self.main_frame.body = new_listbox
                self.main_frame.unhandled_input = self.handle_unhandled_input
            else:
                # 如果沒有上一層可回，就結束程式
                raise urwid.ExitMainLoop()
        elif key == "y":  # 假爪開闔
            result = self.arm_controller.execute_action("catch")
            self.footer_text.set_text(result)



    def run(self):
        """啟動 UI 迴圈"""
        palette = [
            ("reversed", "standout", ""),
            ("header", "white,bold", "dark blue"),
        ]
        loop = urwid.MainLoop(
            widget=self.main_frame,
            palette=palette,
            unhandled_input=self.handle_unhandled_input,
            handle_mouse=False,  # Disable mouse input
        )
        loop.screen.set_terminal_properties(colors=256)
        loop.run()
