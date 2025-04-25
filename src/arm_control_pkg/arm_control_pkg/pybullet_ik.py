# ############################################
# A Robot controller for kinematics, dynamics
# and control based on pyBullet framework
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

# Input:
# 1. robot_type: specify urdf file initials eg. if urdf file name is 'ur5.urdf', specify 'ur5'
# 2. controllable_joints: joint indices of controllable joints. If not specified, by default all joints indices except first joint (first joint is fixed joint between robot stand and base)
# 3. end-eff_index: specify the joint indices for end-effector link. If not specified, by default the last controllable_joints is considered as end-effector joint
# 4. time_Step: time step for simulation

import pybullet as p
import pybullet_data
import numpy as np
import time
import random
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import math
from scipy.spatial.transform import Rotation as R


class PybulletRobotController:
    def __init__(self, arm_params, arm_angle_control_node):
        self.arm_params = arm_params.get_arm_params()
        self.arm_angle_control_node = arm_angle_control_node
        # self.robot_type = "ur5"
        robot_description_path = get_package_share_directory("robot_description")
        self.urdf_path = os.path.join(robot_description_path, "urdf", "target.urdf")
        self.robot_id = None
        self.num_joints = None
        self.controllable_joints = int(
            self.arm_params["pybullet"]["controllable_joints"]
        )
        self.end_eff_index = int(self.arm_params["pybullet"]["end_eff_index"])
        self.time_step = float(self.arm_params["pybullet"]["time_step"])
        self.previous_ee_position = None
        self.initial_height = float(self.arm_params["pybullet"]["initial_height"])
        self.createWorld(
            GUI=self.arm_params["pybullet"]["gui"],
            view_world=self.arm_params["pybullet"]["view_world"],
        )

        # synchronize the robot with the initial position
        self.set_initial_joint_positions()
        self.draw_end_effector_axes()

    def markPointInFrontOfEndEffector(self, distance=0.3, color=[0, 1, 1]):
        """
        沿著 end-effector 的 local X 軸（前方）延伸指定距離，並在該處畫一個十字標記。

        Args:
            distance (float): 沿 local X 軸延伸的距離（單位：公尺）
            color (list): 標記顏色，預設為青色 [0, 1, 1]
        """
        # 取得末端執行器的狀態
        ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
        position = np.array(ee_state[0])  # 世界座標系下的位置
        orientation = ee_state[1]  # 四元數

        # 將姿態轉換成旋轉矩陣
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # local X 軸（前方）方向向量
        forward_direction = rotation_matrix[:, 0]

        # 計算標記點世界座標
        target_point = position + forward_direction * distance

        # 畫十字標記
        line_length = 0.05
        p.addUserDebugLine(
            target_point - np.array([line_length, 0, 0]),
            target_point + np.array([line_length, 0, 0]),
            color,
            lineWidth=2,
        )
        p.addUserDebugLine(
            target_point - np.array([0, line_length, 0]),
            target_point + np.array([0, line_length, 0]),
            color,
            lineWidth=2,
        )
        p.addUserDebugLine(
            target_point - np.array([0, 0, line_length]),
            target_point + np.array([0, 0, line_length]),
            color,
            lineWidth=2,
        )

    def draw_end_effector_axes(self, axis_length=0.2):
        """
        在 end-effector 當前位置畫出 local XYZ 軸（紅: X, 綠: Y, 藍: Z）

        Args:
            axis_length (float): 每個軸的長度
        """
        print("end : " + str(self.end_eff_index))
        ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
        position = np.array(ee_state[0])
        orientation = ee_state[1]

        # 四元數轉旋轉矩陣
        rot_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # 分別取出 local X, Y, Z 三軸方向向量
        x_axis = rot_matrix[:, 0]  # local X
        y_axis = rot_matrix[:, 1]  # local Y
        z_axis = rot_matrix[:, 2]  # local Z

        # 終點位置
        x_end = position + x_axis * axis_length
        y_end = position + y_axis * axis_length
        z_end = position + z_axis * axis_length

        # 畫三軸
        p.addUserDebugLine(
            position.tolist(), x_end.tolist(), [1, 0, 0], lineWidth=3
        )  # X: 紅
        p.addUserDebugLine(
            position.tolist(), y_end.tolist(), [0, 1, 0], lineWidth=3
        )  # Y: 綠
        p.addUserDebugLine(
            position.tolist(), z_end.tolist(), [0, 0, 1], lineWidth=3
        )  # Z: 藍

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot_id, self.controllable_joints)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def move_end_effector_laterally(self, distance=0.3):
        """
        根據末端執行器目前旋轉姿態，沿其「右手方向（local-side）」平移一定距離。

        Args:
            distance (float): 要平移的距離，正值向右，負值向左。
        """
        # Step 1: 取得當前末端執行器位置與姿態
        ee_state = p.getLinkState(self.robot_id, self.end_eff_index)
        position = np.array(ee_state[0])  # 世界座標
        orientation = ee_state[1]  # 四元數 (x, y, z, w)

        # Step 2: 四元數 → 旋轉矩陣
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)

        # Step 3: 抓出旋轉矩陣的「右手方向」單位向量
        right_direction = rotation_matrix[:, 1]  # 通常 local-Y 是側邊 (右手) 方向

        # Step 4: 根據方向向量計算新的目標位置
        target_position = position + right_direction * distance

        # Step 5: 傳入原姿態，保持手的朝向不變
        target_pose = list(target_position) + list(
            p.getEulerFromQuaternion(orientation)
        )
        joint_angles = self.solveInversePositionKinematics(target_pose)

        if joint_angles:
            print("計算出的關節角度:", joint_angles)
            return joint_angles
        else:
            print("無法計算可行的 IK 解。")

    def generateInterpolatedTrajectory(self, target_position, steps=50):
        """
        Generate joint angle trajectory based on target position.

        Args:
            target_position (list or tuple): A [x, y, z] world coordinate of length 3.
            steps (int): Number of interpolation steps, default is 50.

        Returns:
            list: Joint angles (in angle) corresponding to each interpolation point.
        """
        current_position = self.solveForwardPositonKinematics(self.getJointStates()[0])[
            0:3
        ]

        # 計算每一步的位移向量
        step_vector = (np.array(target_position) - np.array(current_position)) / steps
        self.markTarget(target_position)

        # 用於存儲每一步的關節角度（以弧度表示）
        joint_angles_in_radians = []

        # 逐步靠近目標
        for i in range(steps):
            # 計算當前目標位置
            intermediate_position = np.array(current_position) + (i + 1) * step_vector
            # 計算 IK 解
            joint_angles = self.solveInversePositionKinematics(intermediate_position)

            # 將當前關節角度應用到機器人
            if joint_angles and len(joint_angles) >= len(self.controllable_joints):
                # 直接存儲弧度值
                joint_angles_in_radians.append(
                    joint_angles[: len(self.controllable_joints)]
                )
            else:
                print("無法找到合適的解。")
                break

        return joint_angles_in_radians

    def solveInversePositionKinematics(self, end_eff_pose):
        """
        計算逆向運動學以獲取關節角度，基於給定的末端執行器姿勢。

        Args:
            end_eff_pose (list): 末端執行器的目標位置和姿勢，
                                格式為 [x, y, z, roll, pitch, yaw] (6 個元素) 或 [x, y, z] (3 個元素)。

        Returns:
            list: 對應的關節角度。
        """
        if len(end_eff_pose) == 6:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id,
                self.end_eff_index,
                targetPosition=end_eff_pose[0:3],
                targetOrientation=p.getQuaternionFromEuler(end_eff_pose[3:6]),
            )
        else:
            joint_angles = p.calculateInverseKinematics(
                self.robot_id, self.end_eff_index, targetPosition=end_eff_pose[0:3]
            )

        # 標記末端執行器的位置路徑
        # self.markEndEffectorPath()
        return joint_angles

    def set_initial_joint_positions(self):
        """從配置中讀取初始關節角度並設置"""
        print("設置初始關節角度...")

        # 轉換為弧度
        initial_positions_deg = self.arm_angle_control_node.get_arm_angles()
        print(f"初始關節角度 (角度): {initial_positions_deg}")
        initial_positions_rad = [math.radians(angle) for angle in initial_positions_deg]

        # 設置關節位置
        self.setJointPosition(initial_positions_rad)

    # function for setting joint positions of robot in pybullet
    def setJointPosition(self, position, kp=1.0, kv=1.0):
        # print('Joint position controller')
        zero_vec = [0.0] * len(self.controllable_joints)
        p.setJointMotorControlArray(
            self.robot_id,
            self.controllable_joints,
            p.POSITION_CONTROL,
            targetPositions=position,
            targetVelocities=zero_vec,
            positionGains=[kp] * len(self.controllable_joints),
            velocityGains=[kv] * len(self.controllable_joints),
        )
        for _ in range(100):  # to settle the robot to its position
            p.stepSimulation()

    def markTarget(self, target_position):
        # 使用紅色標記顯示目標位置
        line_length = 0.1  # 調整標記大小
        p.addUserDebugLine(
            [target_position[0] - line_length, target_position[1], target_position[2]],
            [target_position[0] + line_length, target_position[1], target_position[2]],
            [1, 0, 0],  # 紅色
            lineWidth=3,
        )
        p.addUserDebugLine(
            [target_position[0], target_position[1] - line_length, target_position[2]],
            [target_position[0], target_position[1] + line_length, target_position[2]],
            [1, 0, 0],
            lineWidth=3,
        )
        p.addUserDebugLine(
            [target_position[0], target_position[1], target_position[2] - line_length],
            [target_position[0], target_position[1], target_position[2] + line_length],
            [1, 0, 0],
            lineWidth=3,
        )

    def solveForwardPositonKinematics(self, joint_pos):
        # get end-effector link state
        eeState = p.getLinkState(self.robot_id, self.end_eff_index)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
        eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
        return eePose

    # function to initiate pybullet and engine and create world
    def createWorld(self, GUI=True, view_world=False):
        # load pybullet physics engine
        if GUI:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10
        )
        p.setRealTimeSimulation(True)
        p.loadURDF("plane.urdf")
        rotation = R.from_euler("z", 90, degrees=True).as_quat()

        # loading robot into the environment
        # urdf_file = "urdf/" + self.robot_type + ".urdf"
        self.robot_id = p.loadURDF(
            self.urdf_path,
            useFixedBase=True,
            basePosition=[0, 0, self.initial_height],
            baseOrientation=rotation,
        )

        self.num_joints = p.getNumJoints(self.robot_id)  # Joints
        print("#Joints read from pybullet:", self.num_joints)
        if self.controllable_joints is None:
            self.controllable_joints = list(range(1, self.num_joints - 1))
        print("#Controllable Joints:", self.controllable_joints)
        if self.end_eff_index is None:
            self.end_eff_index = self.controllable_joints[-1]
        print("#End-effector:", self.end_eff_index)
        self.num_joints = p.getNumJoints(self.robot_id)
        # print(f"總關節數量: {self.num_joints}")
        self.controllable_joints = list(range(1, self.num_joints - 1))
        # print(f"可控制的關節索引: {self.controllable_joints}")
        # print(f"需要提供的初始位置數量: {len(self.controllable_joints)}")

        if view_world:
            while True:
                p.stepSimulation()
                time.sleep(self.time_step)
