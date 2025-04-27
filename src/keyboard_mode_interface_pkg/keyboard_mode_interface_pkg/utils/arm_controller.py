import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
import time
class ArmController:
    def __init__(self, ros_communicator):
        """
        初始化機械手臂控制器
        
        Args:
            ros_communicator: ROS通信管理器，用於發布機械手臂角度
        """
        self.ros_communicator = ros_communicator
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0, 10.0]
        self.angle_increment = 0.1
        # 預設動作
        self.predefined_actions = {
            "catch": self._catch_action,
        }
    
    def move_joint(self, joint_index, direction):
        """移動指定關節"""
        try:
            if 0 <= joint_index < len(self.current_angles)-2:
                self.current_angles[joint_index] += direction * self.angle_increment
                # 限制角度範圍在 -π 到 π 之間
                self.current_angles[joint_index] = max(min(self.current_angles[joint_index], np.pi), -np.pi)
                # 發布新的角度
                self.publish_angles()
                return f"關節 {joint_index} 移動到 {self.current_angles[joint_index]:.2f} 弧度 \n {self.current_angles}"
            return f"無效的關節索引: {joint_index}"
        except Exception as e:
            return f"移動關節時發生錯誤: {str(e)}"
    
    def publish_angles(self):
        """發布當前機械手臂角度到ROS"""
        
        self.ros_communicator.publish_robot_arm_angle(self.current_angles)
    
    def reset_arm(self):
        """重置機械手臂到初始位置"""
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0, 10.0]
        self.publish_angles()
        return "機械手臂已重置到初始位置"
    
    def execute_action(self, action_name):
        """
        執行預定義動作
        
        Args:
            action_name: 動作名稱
        """
        if action_name in self.predefined_actions:
            return self.predefined_actions[action_name]()
        return f"未知動作: {action_name}"
    
    def _catch_action(self):

        self.current_angles[6]*=(-1)
        self.current_angles[7]*=(-1)
        self.publish_angles()
        # 在實際應用中應該加入延時

        if(self.current_angles[6] > 0):
            return f"{self.current_angles} \n 假爪已關閉"
        else:
            return f"{self.current_angles} \n假爪已打開"

    
    def get_current_state(self):
        """獲取當前機械手臂狀態"""
        state_info = "機械手臂當前角度:\n"
        for i, angle in enumerate(self.current_angles):
            state_info += f"關節 {i}: {angle:.2f} 弧度\n"
        return state_info

