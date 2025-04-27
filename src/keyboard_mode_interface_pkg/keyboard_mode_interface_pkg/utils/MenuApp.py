import urwid
from .arm_controller import ArmController

    



class MenuApp:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.arm_controller = ArmController(ros_manager)  # Initialize ArmController
        self.joint_index = None

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
        self.header_text = urwid.Text(self.current_title)
        self.footer_text = urwid.Text("")

        # 建立主 ListBox
        self.menu = urwid.ListBox(urwid.SimpleFocusListWalker(self.create_menu()))

        # 組合成 Frame
        self.main_frame = urwid.Frame(
            self.menu, header=self.header_text, footer=self.footer_text
        )
        self.loop = None  # 添加 MainLoop 引用

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
            elif choice in [str(i) for i in range(7)]:  # 0~6: 關節調整
                self.enter_joint_control(int(choice))



    def enter_joint_control(self, joint_index):
        """進入關節控制頁面"""
        self.joint_index = joint_index
        self.active_joint = joint_index
        self.menu_stack.append((self.current_menu, self.current_title))
        self.current_menu = {}
        self.current_title = f"Joint {joint_index} Control"
        self.header_text.set_text(self.current_title)

        # Create joint control widget
        control_widget = urwid.Pile([
            urwid.Text(f"調整關節 {joint_index} 的角度", align="center"),
            urwid.Divider(),
            urwid.Text("按 'i' 增加角度", align="center"),
            urwid.Text("按 'k' 減少角度", align="center"),
            urwid.Text("按 'q' 返回上一頁", align="center"),
        ])
        
        # Wrap in ListBox
        self.menu = urwid.ListBox(urwid.SimpleListWalker([control_widget]))
        
        # Update frame
        self.main_frame.body = self.menu
        self.main_frame.set_focus('body')

    def toggle_gripper(self):
        """控制假爪開闔"""
        result = self.arm_controller.execute_action("catch")
        self.footer_text.set_text(result)

    def keypress(self, key):
        if(self.joint_index == None):
            return
        if(self.joint_index == 6 and key == 'y'):
                self.toggle_gripper()
        elif key == 'q':
            self.joint_index = None
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
                self.main_frame.unhandled_input = self.keypress
            else:
                raise urwid.ExitMainLoop()
        elif key in ('i', 'k') and self.joint_index != 6:
            direction = 1 if key == 'i' else -1
            try:
                result = self.arm_controller.move_joint(self.joint_index, direction)
                self.footer_text.set_text(result)
            except Exception as e:
                self.footer_text.set_text(f"Error: {str(e)}")
        else:
            if self.loop:
                self.loop.screen.clear()
                self.loop.draw_screen()
                self.footer_text.set_text(f"按鍵 '{key}' 無效，請使用 'q' 返回或 'y' 控制假爪。i direction-1, k direction+1")
            
            
            
        
    def run(self):
        """啟動 UI 迴圈"""
        palette = [
            ("reversed", "standout", ""),
            ("header", "white,bold", "dark blue"),
        ]
        self.loop = urwid.MainLoop(  # 保存 loop 引用
            widget=self.main_frame,
            palette=palette,
            unhandled_input=self.keypress,
        )
        self.loop.screen.set_terminal_properties(colors=256)
        self.loop.run()
