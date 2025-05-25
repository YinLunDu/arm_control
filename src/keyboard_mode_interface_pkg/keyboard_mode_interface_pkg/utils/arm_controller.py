# return 角度一律 radians
import math
import numpy as np
import time

from scipy.spatial.transform import Rotation as R
import numpy as np
import threading

class ArmController:
    def __init__(self, ros_communicator, data_processor, ik_solver, num_joints):
        # 初始化模擬環境與參數
        self.ik_solver = ik_solver
        self.ik_solver.createWorld(GUI=True)
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor

        self.num_joints = num_joints
        self.joint_pos = []
        self.key = 0

        self.world_created = False
        self.is_moving = False
        self.action_in_progress = False
        self.latest_align_coordinates = None
        self.base_link_position = self.ik_solver.get_base_position()

        self.flag = 0
        self._thread_running = False
        self._stop_event = threading.Event()

        self.predefined_actions = {
            "catch": self._catch_action,
        }
 
    def ensure_joint_pos_initialized(self):
        if len(self.joint_pos) < self.num_joints:
            self.joint_pos = [0.0] * self.num_joints
            self.reset_arm()
            self.update_action(self.joint_pos)

    def manual_control(self, index, key):
        # 定義每個軸的最小和最大角度
        joint_limits = [
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 0
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 1
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 2
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 3
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 4   
            {"min_angle": -360.0, "max_angle": 360.0},  # Joint 5   
            {"min_angle": -0.01, "max_angle": 0.01},  # Joint 6
            {"min_angle": -0.01, "max_angle": 0.01},  # Joint 7
        ]

        # 確保初始位置已初始化
        self.ensure_joint_pos_initialized()

        # 獲取當前 joint 的限制
        if 0 <= index < len(joint_limits):
            min_angle = joint_limits[index]["min_angle"]
            max_angle = joint_limits[index]["max_angle"]

            if key == "y" and (index == 6 or index == 7): # 控制夾爪
                self.execute_action("catch")
            elif key == "i" and index not in [6, 7]: # 控制關節
                self.adjust_joint_angle(index, 10, min_angle, max_angle)
            elif key == "k" and index not in [6, 7]: # 控制關節
                self.adjust_joint_angle(index, -10, min_angle, max_angle)
            elif key == "b":
                self.reset_arm()
            elif key == "t":
                self.real_robot_position()
            elif key == "q":
                return True
            else:
                print(f"無效按鍵 '{key}'，請使用 'i', 'k', 'b', 'q'。")
        else:
            print(f"無效索引 {index}，請確認範圍在 0 ~ {len(joint_limits) - 1} 之內。")
        self.update_action(self.joint_pos)

    # try:
    #     object_position_world = self.project_yolo_to_world()
    #     self.ros_communicator.publish_coordinates(object_position_world[0], object_position_world[1], object_position_world[2])
    # except:
    #     pass
    #     # self.ik_solver.setJointPosition(joint_angles)

    #     self.update_action(self.joint_pos)

    # auto control--------------------------------------------------
    def auto_control(self, key=None, mode="auto_arm_control"):
        self.ensure_joint_pos_initialized()
        if self.flag == 0:
            stop_event = threading.Event()
            thread = threading.Thread(target=self.background_task, args=(stop_event,))
        if key == "q":
            if self._thread_running:
                self._stop_event.set()
                self._auto_arm_thread.join()
                self._thread_running = False
            return True
        elif key == "b":  # 重置手臂
            self.reset_arm()
        elif key == "t":
            # 判斷 realposition 和目前目標 joint 前6個最大誤差是否在 1 度以內
            whether_next_pos=True
            real_robot_position = self.data_processor.get_realrobot_position()
            if real_robot_position is not None and len(real_robot_position) >= 6:
                # 取目前 joint_pos 前6個（排除夾爪等多餘項）
                current_joint_pos = self.joint_pos[:6]
                # 計算每個關節的角度誤差（以度為單位）
                errors = [abs(math.degrees(a) - math.degrees(b)) for a, b in zip(current_joint_pos, real_robot_position[:6])]
                max_error = max(errors)
                print(f"最大誤差: {max_error:.2f} 度")
                if max_error <= 1.0:
                    whether_next_pos=True
                else:
                    print("實體與目標 joint 前 6 個最大誤差超過 1 度")
            else:
                print("無法獲取實體機械手臂位置或數量不符")
            if not self._thread_running and whether_next_pos:
                self._stop_event.clear()  # 清除之前的停止狀態
                self._auto_arm_thread = threading.Thread(
                    target = self.background_task,
                    args = (self._stop_event, mode),
                    daemon = True,
                )
                self._auto_arm_thread.start()
                self._thread_running = True
            return False

    def background_task(self, stop_event, mode):
        while not stop_event.is_set():
            if mode == "auto_arm_human":
                self.random_wave()
                self._thread_running = False

    def get_mediapipe_data_coordinates(self):
        mediapipe_coords = self.data_processor.get_processed_mediapipe_data()
        corrected_coords = mediapipe_coords + [0, 0, 24]
        corrected_coords[0] = 0.3
        if corrected_coords[2] < 10:
            corrected_coords[2] = 10
        return mediapipe_coords

    def publish_coordinates(self, x, y, z):
        self.ros_communicator.publish_coordinates(x, y, z)

    def move_to_position(self, object_position_world):
        self.action_in_progress = True
        joint_angles = self.ik_solver.solveInversePositionKinematics(
            object_position_world
        )
        joint_angles = joint_angles[:-1]
        self.joint_pos = list(joint_angles)
        self.update_action(joint_angles)  # 更新動作
        time.sleep(1.0)
        self.action_in_progress = False


    def align_to_target_with_yolo_offset(self, step_size=0.07, tolerance=0.03):
        """
        根據 YOLO 偵測到的偏移量，逐步調整機械手臂的末端位置，使其對準目標物體的中心。

        Args:
            step_size (float): 每次移動的增量距離，默認為 0.05 米。
            tolerance (float): 允許的偏移量容忍範圍，默認為 0.03 米。
        """
        # 獲取物體在相機畫面中的偏移量
        for _ in range(10):
            x_offset, y_offset, _ = (
                self.data_processor.get_processed_yolo_detection_offset()
            )
            depth = self.data_processor.get_processed_yolo_detection_position()[0]
            print(f"depth: {depth}")

            # 檢查偏移量是否已經在允許的容忍範圍內
            if abs(x_offset) <= tolerance and abs(y_offset) <= tolerance:
                print("已對準，停止校正")
                self.update_action(self.joint_pos)
                return True

            # 獲取末端執行器的當前旋轉矩陣
            _, end_effector_rotation_matrix = self.ik_solver.get_current_pose()

            # 根據畫面座標系將 x_offset, y_offset 轉換到夾具的本地座標系
            # x_offset 對應夾具的 Y 軸移動方向，y_offset 對應夾具的 Z 軸移動方向
            move_vector = end_effector_rotation_matrix @ np.array(
                [0, -x_offset, y_offset]
            )

            # 將移動向量歸一化並乘以 step_size 以控制移動步長
            move_vector = move_vector / np.linalg.norm(move_vector) * step_size

            # 移動機械手臂的末端執行器
            self.move_end_effector(
                x_offset=move_vector[0],
                y_offset=move_vector[1],
                z_offset=move_vector[2],
            )

            print("最終對準完成")

    def move_end_effector(self, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        """
        移動機械手臂的末端執行器，基於指定的 x、y、z 偏移量進行移動。

        Args:
            x_offset (float): 沿著 x 軸的移動距離（單位：米）。
            y_offset (float): 沿著 y 軸的移動距離（單位：米）。
            z_offset (float): 沿著 z 軸的移動距離（單位：米）。
        """
        # 獲取末端執行器的當前位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        # 計算目標位置，將偏移量加到當前位置
        target_position_world = np.array(end_effector_position_world) + np.array(
            [x_offset, y_offset, z_offset]
        )

        # 使用 IK 移動到新的目標位置
        self.move_to_position(target_position_world)
        self.latest_align_coordinates = target_position_world
        # 發佈新的目標位置
        self.ros_communicator.publish_coordinates(
            target_position_world[0], target_position_world[1], target_position_world[2]
        )

        # print(f"移動到新位置: x={target_position_world[0]:.4f}, y={target_position_world[1]:.4f}, z={target_position_world[2]:.4f}")

 
    def gradual_move(self, object_position_world):
        print("開始緩慢移動")
        self.action_in_progress = True
        joint_angles_sequence = self.ik_solver.moveTowardsTarget(
            object_position_world, steps=30
        )
        joint_angles_sequence = joint_angles_sequence[:-1]
        for joint_angles in joint_angles_sequence:

            depth = self.data_processor.get_processed_yolo_detection_position()[0]
            detection_status = self.ros_communicator.get_latest_yolo_detection_status()

            if detection_status == None:
                print("no detection")
                if self.align_to_target_with_yolo_offset(tolerance=0.03):
                    break

            if depth < 0.3:
                self.action_in_progress = False
                break
            self.set_all_joint_angles(joint_angles)
            self.update_action(self.joint_pos)
            self.ik_solver.setJointPosition(self.joint_pos)
            time.sleep(0.5)
        self.action_in_progress = False

    def random_wave(self, num_moves=5, steps=180):
        joint_angle_sequences = self.ik_solver.generate_random_target_and_solve_ik()
        for joint_angles in joint_angle_sequences:
            self.ik_solver.setJointPosition(joint_angles)
            self.set_all_joint_angles(joint_angles)
            self.update_action(joint_angles)
            time.sleep(0.01)



    def process_yolo_coordinates(self):
        yolo_coordinates = self.data_processor.get_processed_yolo_detection_position()

        # 檢查 YOLO 偵測座標的維度
        if len(yolo_coordinates) == 2:
            yolo_coordinates = np.append(yolo_coordinates, 0)  # 添加預設 z 值

        # 獲取末端執行器的世界位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        camera_offset_local = np.array([0, 0, 0.1])  # 單位：米
        camera_position_world = (
            np.array(end_effector_position_world)
            + end_effector_rotation_matrix @ camera_offset_local
        )

        # 使用旋轉矩陣轉換到世界坐標
        object_position_world = (
            camera_position_world
            + end_effector_rotation_matrix @ np.array(yolo_coordinates)
        )

        # 獲取基座的世界位置
        base_position_world, _ = self.ik_solver.get_base_pose()

        object_position_base = object_position_world - np.array(base_position_world)
        return object_position_base

    def project_yolo_to_world(self, offset_distance=0.1):
        """
        將 YOLO 偵測到的物體坐標投射到 PyBullet 世界座標系中的末端執行器前方。

        Args:
            yolo_coordinates (list or np.array): YOLO 偵測物體的相機坐標系下的位置 [x, y, z]
            offset_distance (float): 偏移距離，用於將座標投射到末端執行器的前方（單位：米）

        Returns:
            np.array: 投射後的世界座標
        """
        # 檢查 YOLO 偵測座標的維度
        yolo_coordinates = self.data_processor.get_processed_yolo_detection_position()
        if len(yolo_coordinates) == 2:
            yolo_coordinates = np.append(yolo_coordinates, 0)  # 添加預設 z 值

        # 獲取末端執行器的世界位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        # 定義相機的偏移（將 YOLO 偵測到的坐標投射到相機的前方）
        # `yolo_coordinates` 代表了相機坐標系下的物體位置
        object_position_camera = np.array(yolo_coordinates) + np.array(
            [0, 0, offset_distance]
        )

        # 將物體從相機坐標系轉換到世界坐標系
        object_position_world = (
            np.array(end_effector_position_world)
            + end_effector_rotation_matrix @ object_position_camera
        )
        return object_position_world

    def project_yolo_to_world_offset(
        self, offset_distance=0.3, tolerance=0.03, max_iterations=50
    ):
        """
        將 YOLO 偵測到的物體座標投射到 PyBullet 世界坐標系中的末端執行器前方，
        並讓十字中心對準物體，保持距離物體30公分。

        Args:
            offset_distance (float): 指定機械手臂與物體的距離（單位：米），預設為 0.3 米。
            tolerance (float): x 和 y 軸的容忍範圍，預設為 0.03 米。
            max_iterations (int): 最大迭代次數，以防止無限循環。

        Returns:
            np.array: 最終的世界坐標位置。
        """
        for _ in range(max_iterations):
            # 獲取 YOLO 偵測到的物體在相機坐標系中的位置偏移
            yolo_offset = self.data_processor.get_processed_yolo_detection_position()
            x_offset, y_offset, depth_value = (
                yolo_offset[0],
                yolo_offset[1],
                yolo_offset[2],
            )

            # 檢查是否已經在容忍範圍內
            if abs(x_offset) <= tolerance and abs(y_offset) <= tolerance:
                print("已對準，停止校正")
                break

            # 計算物體在相機坐標系下的目標位置，保持 offset_distance 的 Z 軸距離
            target_position_camera = np.array([offset_distance, x_offset, y_offset])

            # 獲取末端執行器的世界位置和旋轉矩陣
            end_effector_position_world, end_effector_rotation_matrix = (
                self.ik_solver.get_current_pose()
            )

            # 將相機坐標系下的目標位置轉換為世界坐標系
            target_position_world = (
                np.array(end_effector_position_world)
                + end_effector_rotation_matrix @ target_position_camera
            )

            # 使用 IK 將末端執行器移動到新的目標位置
            self.move_to_position(target_position_world)

            # 發佈新的目標位置
            self.ros_communicator.publish_coordinates(
                target_position_world[0],
                target_position_world[1],
                target_position_world[2],
            )

            # 小延遲以便系統完成移動
            time.sleep(0.1)

        return target_position_world

    def project_yolo_to_target(
        self, step_size=0.05, target_distance=0.3, tolerance=0.02
    ):
        """
        將 YOLO 偵測的座標投射到距離機械手末端指定距離處，並確保與物體座標向量方向一致，使用線性插值逐步移動。

        Args:
            step_size (float): 每次移動的增量距離，默認為 0.05 米。
            target_distance (float): 目標距離（距離物體的距離），默認為 0.3 米。
            tolerance (float): 允許的最小距離差，用於檢查深度是否接近目標距離，默認為 0.02 米。

        Returns:
            None
        """
        # 獲取 YOLO 偵測到的物體在相機坐標系中的位置
        object_position_world = self.project_yolo_to_world()

        # 計算從末端到物體位置的向量
        end_effector_position_world, _ = self.ik_solver.get_current_pose()
        direction_vector = object_position_world - end_effector_position_world
        current_distance = np.linalg.norm(direction_vector)
        normalized_direction_vector = direction_vector / current_distance

        # 計算目標位置，讓末端保持距離物體 target_distance
        target_position_world = (
            object_position_world - normalized_direction_vector * target_distance
        )

        while current_distance > target_distance + tolerance:
            # 每次移動 step_size 距離，逐步接近目標
            incremental_position = (
                end_effector_position_world + normalized_direction_vector * step_size
            )
            self.move_to_position(incremental_position)

            # 更新當前位置與距離
            end_effector_position_world, _ = self.ik_solver.get_current_pose()
            direction_vector = object_position_world - end_effector_position_world
            current_distance = np.linalg.norm(direction_vector)

            # 發佈當前位置
            self.ros_communicator.publish_coordinates(
                end_effector_position_world[0],
                end_effector_position_world[1],
                end_effector_position_world[2],
            )

            # 檢查是否已達到允許範圍內的距離
            if abs(current_distance - target_distance) <= tolerance:
                print("已達到目標距離範圍內")
                break

            # 短暫延遲以允許手臂移動完成
            time.sleep(0.01)

    def project_yolo_to_world_look_at_target(self, offset_distance=0.1):

        # 獲取 YOLO 偵測到的物體偏移座標
        yolo_coordinates = np.array(
            self.data_processor.get_processed_yolo_detection_position()
        )
        if len(yolo_coordinates) == 2:
            yolo_coordinates = np.append(yolo_coordinates, 0)  # 添加預設 z 值

        # 計算目標物體在相機坐標系下的位置
        target_position_camera = np.array(yolo_coordinates) + np.array(
            [0, 0, offset_distance]
        )

        # 獲取末端執行器的世界位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        # 將相機座標系下的目標位置轉換到世界坐標系
        target_position_world = (
            np.array(end_effector_position_world)
            + end_effector_rotation_matrix @ target_position_camera
        )

        # 計算「相機注視物體」所需的旋轉
        direction_to_target = target_position_world - end_effector_position_world
        direction_to_target /= np.linalg.norm(direction_to_target)  # 歸一化方向向量

        # 確保 z 軸對準目標方向
        z_axis_world = direction_to_target  # 末端執行器 z 軸對準目標
        x_axis_world = np.array([1, 0, 0])  # 假設 x 軸水平對齊
        y_axis_world = np.cross(z_axis_world, x_axis_world)  # 計算 y 軸
        x_axis_world = np.cross(y_axis_world, z_axis_world)  # 重計算 x 軸使其正交

        # 建立旋轉矩陣並轉換為四元數
        rotation_matrix = np.vstack([x_axis_world, y_axis_world, z_axis_world]).T
        target_orientation_quaternion = R.from_matrix(rotation_matrix).as_quat()

        # 使用逆運動學計算末端執行器的位置和朝向
        end_eff_pose = list(target_position_world) + list(target_orientation_quaternion)
        self.move_to_position(end_eff_pose)

        # 返回目標位置和旋轉
        # return target_position_world, target_orientation_quaternion

    def project_yolo_to_world_target(
        self, offset_distance=0.1, step_size=0.01, tolerance=0.05
    ):
        """
        將 YOLO 偵測到的物體坐標投射到 PyBullet 世界坐標系中的末端執行器前方，並逐步移動到目標位置。
        同時將 y 和 z 偏移量控制在容忍範圍內。

        Args:
            offset_distance (float): 偏移距離，用於將坐標投射到末端執行器的前方（單位：米）。
            step_size (float): 每次更新的位置增量。
            tolerance (float): y 和 z 軸偏移的容忍範圍。

        Returns:
            np.array: 最終的世界坐標位置。
        """
        # 獲取 YOLO 偵測到的物體偏移座標
        yolo_coordinates = self.data_processor.get_processed_yolo_detection_position()

        # 檢查 y 和 z 偏移量是否在容忍範圍內
        y_in_range = abs(yolo_coordinates[1]) < tolerance
        z_in_range = abs(yolo_coordinates[2]) < tolerance

        # 如果 y 和 z 都在範圍內，無需移動
        if y_in_range and z_in_range:
            print("目標物已在範圍內，無需移動")
            return np.array(self.ik_solver.get_current_pose()[0])  # 返回當前末端位置

        # 若不在範圍內，則將 y 和 z 偏移量限制在容忍範圍內
        if not y_in_range:
            yolo_coordinates[1] = max(min(yolo_coordinates[1], tolerance), -tolerance)
        if not z_in_range:
            yolo_coordinates[2] = max(min(yolo_coordinates[2], tolerance), -tolerance)

        # 獲取末端執行器的世界位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        # 計算目標相機位置，考慮到 z 軸的 offset_distance
        target_position_camera = np.array(yolo_coordinates) + np.array(
            [0, 0, offset_distance]
        )

        # 計算目標位置的世界坐標
        target_position_world = (
            np.array(end_effector_position_world)
            + end_effector_rotation_matrix @ target_position_camera
        )

        # 計算當前位置與目標位置之間的方向
        direction_vector = target_position_world - np.array(end_effector_position_world)
        distance_to_move = np.linalg.norm(direction_vector)

        # 如果需要移動的距離小於 step_size，直接移動到目標位置
        if distance_to_move <= step_size:
            self.move_to_position(target_position_world)
            self.ros_communicator.publish_coordinates(
                target_position_world[0],
                target_position_world[1],
                target_position_world[2],
            )
            return target_position_world

        # 否則，移動一個增量
        direction_unit_vector = direction_vector / distance_to_move  # 歸一化方向向量
        incremental_position = (
            np.array(end_effector_position_world) + direction_unit_vector * step_size
        )
        self.move_to_position(incremental_position)
        self.ros_communicator.publish_coordinates(
            incremental_position[0], incremental_position[1], incremental_position[2]
        )

    def project_yolo_to_world_fixed_depth(self, offset_distance=0.1):
        """
        將 YOLO 偵測到的物體坐標投射到 PyBullet 世界座標系中的末端執行器前方，
        深度（x 軸）與末端夾具保持一致，y 和 z 偏移量根據 YOLO 偵測的座標進行調整。

        Args:
            offset_distance (float): 偏移距離，用於將坐標投射到末端執行器的前方（單位：米）

        Returns:
            np.array: 投射後的世界座標
        """
        # 獲取 YOLO 偵測的偏移座標
        yolo_coordinates = self.data_processor.get_processed_yolo_detection_position()
        if len(yolo_coordinates) == 2:
            yolo_coordinates = np.append(yolo_coordinates, 0)  # 添加預設 z 值

        # 獲取末端執行器的世界位置和旋轉矩陣
        end_effector_position_world, end_effector_rotation_matrix = (
            self.ik_solver.get_current_pose()
        )

        # 保持 x 軸（深度）與末端夾具一致，並在 y 和 z 軸上添加 YOLO 偵測的偏移
        object_position_camera = np.array(
            [
                0,  # x 軸保持不變
                yolo_coordinates[1],  # y 軸使用 YOLO 偵測到的偏移量
                yolo_coordinates[2]
                + offset_distance,  # z 軸使用 YOLO 偵測到的偏移量 + 前方偏移距離
            ]
        )

        # 將物體從相機坐標系轉換到世界坐標系
        object_position_world = (
            np.array(end_effector_position_world)
            + end_effector_rotation_matrix @ object_position_camera
        )
        # 確保 x 軸深度與末端夾具保持一致
        object_position_world[0] = end_effector_position_world[0]

        return object_position_world

    # 更新電腦裡面手臂角度紀錄
    def set_all_joint_angles(self, angles_degrees):
        """
        Sets all joints to the specified angles in degrees.

        Args:
            angles_degrees (list[float]): A list of angles in degrees for each joint.

        Example:
            >>> self.set_all_joint_angles([30, 45, 90, 60])
        """
        # 確保提供的角度數量與關節數一致
        if len(angles_degrees) != self.num_joints:
            raise ValueError(
                "The number of angles provided does not match the number of joints."
            )

        # 將每個角度設置到對應的關節
        for i, angle in enumerate(angles_degrees):
            self.joint_pos[i] = angle

    def get_joint_angles(self):
        # Return the current joint angles in degrees
        return [round(math.degrees(angle), 2) for angle in self.joint_pos]

    # 更新實體和虛擬
    def update_action(self, joint_pos):
     #   print(f"update_action: {self.get_joint_angles()}")
       # self.ik_solver.setJointPosition(joint_pos)
        numbers =joint_pos[:8]
        
        self.ros_communicator.publish_robot_arm_angle(joint_pos)
      #  joint_pos.append(0)
        if len(joint_pos) < 9:

            joint_pos.append(0)
        self.ik_solver.setJointPosition(joint_pos)
     #   self.ik_solver.markEndEffector()

    def clamp(self, value, min_value, max_value):
        """
        Clamps a value within a specified range.

        Args:
            value (float): The value to be clamped.
            min_value (float): The lower limit of the range.
            max_value (float): The upper limit of the range.

        Returns:
            float: The clamped value.

        Example:
            >>> self.clamp(5, 0, 10)
            5

            >>> self.clamp(15, 0, 10)
            10
        """
        return max(min_value, min(value, max_value))

    def set_joint_position(self, joint_index, target_angle, lower_limit, upper_limit):
        """
        Sets a specific joint to a target angle with specified limits.

        Args:
            joint_index (int): The index of the joint to update.
            target_angle (float): The target angle for the joint (in degrees).
            lower_limit (float): The lower limit for the joint's angle (in degrees).
            upper_limit (float): The upper limit for the joint's angle (in degrees).

        Example:
            Set Joint 0 to 30 degrees, within the range -90 to 90 degrees:
            >>> self.set_joint_position(0, 30, -90, 90)

            Set Joint 1 to -20 degrees, within the range -45 to 45 degrees:
            >>> self.set_joint_position(1, -20, -45, 45)
        """
        self.joint_pos[joint_index] = self.clamp(
            math.radians(target_angle),
            math.radians(lower_limit),
            math.radians(upper_limit),
        )

    def set_multiple_joint_positions(self, joint_configs):
        DEFAULT_LIMITS = (-90, 90)  # 預設角度限制

        for config in joint_configs:
            joint_id = config["joint_id"]
            target_angle = config["angle"]
            # 如果沒有設定 limits，使用預設值
            min_angle, max_angle = config.get("limits", DEFAULT_LIMITS)

            self.set_joint_position(
                joint_index=joint_id,
                target_angle=target_angle,
                lower_limit=min_angle,
                upper_limit=max_angle,
            )

    def set_all_joint_positions(self, angle_degrees):
        angle_radians = math.radians(angle_degrees)
        self.joint_pos = [angle_radians] * self.num_joints

    def reset_arm(self):
        """
        Resets the robotic arm to the default position (all angles set to 0).

        This method resets the positions of all joints to their initial values, and the result can be
        published using the `publish_arm_position()` method.

        Example:
            >>> self.reset_arm()
            >>> self.publish_arm_position()
        """
        joint_configs = [
            {"joint_id": 0,"angle": 0.0,},
            {"joint_id": 1, "angle": 0.0, },
            {"joint_id": 2, "angle": 0.0, },
            {"joint_id": 3, "angle": 0.0, },
            {"joint_id": 4, "angle": 0.0, },
            {"joint_id": 5, "angle": 0.0, },
            {"joint_id": 6, "angle": 0.01, },
            {"joint_id": 7, "angle": -0.01, },
        ]
        init_position = self.data_processor.get_realrobot_position()
        if(init_position != None):
            init_position.extend([0.01])
            init_position.extend([-0.01])
            joint_configs = [
                {"joint_id": 0, "angle": init_position[0], },
                {"joint_id": 1, "angle": init_position[1], },
                {"joint_id": 2, "angle": init_position[2], },
                {"joint_id": 3, "angle": init_position[3], },
                {"joint_id": 4, "angle": init_position[4], },
                {"joint_id": 5, "angle": init_position[5], },
                {"joint_id": 6, "angle": init_position[6], },
                {"joint_id": 7, "angle": init_position[7], },
            ]
        self.set_multiple_joint_positions(joint_configs)

    def set_last_joint_angle(self, target_angle, min_angle=-360.0, max_angle=360.0):
        """
        直接設定機械手臂最後一軸的角度。

        Args:
            target_angle (float): 想要設定的目標角度（以度數為單位）。
            min_angle (float): 最小允許角度（默認為 10 度）。
            max_angle (float): 最大允許角度（默認為 70 度）。
        """
        # 確保 joint_pos 已初始化
        self.ensure_joint_pos_initialized()

        # 限制新角度在 min_angle 和 max_angle 之間
        clamped_angle = max(min(target_angle, max_angle), min_angle)

        # 更新最後一軸的角度，並轉換為 radians
        self.joint_pos[-1] = math.radians(clamped_angle)

        # 更新動作
        self.update_action(self.joint_pos)

    def adjust_joint_angle(self, joint_id, delta_angle, min_angle=-90, max_angle=90):
        """
        Adjusts a joint angle by adding or subtracting from its current position.

        Args:
            joint_id (int): The index of the joint to adjust (0-based index)
            delta_angle (float): The angle to add (positive) or subtract (negative) in degrees
            min_angle (float): Minimum allowed angle in degrees (default: -90)
            max_angle (float): Maximum allowed angle in degrees (default: 90)

        Example:
            # Increase joint 0's angle by 10 degrees with default limits (-90 to 90)
            >>> self.adjust_joint_angle(0, 10)

            # Decrease joint 1's angle by 5 degrees with custom limits
            >>> self.adjust_joint_angle(1, -5, min_angle=-45, max_angle=45)

            # Adjust joint 2 with asymmetric limits
            >>> self.adjust_joint_angle(2, 10, min_angle=0, max_angle=180)
        """
        if joint_id >= self.num_joints:
            return

        # 獲取當前角度（轉換為度數）
        print(f"joint_id: {joint_id}")
        print(f"joint_pos: {len(self.joint_pos)}")
        current_angle = math.degrees(self.joint_pos[joint_id])

        # 計算新角度
        new_angle = current_angle + delta_angle

        # 更新角度
        self.set_joint_position(
            joint_index=joint_id,
            target_angle=new_angle,
            lower_limit=min_angle,
            upper_limit=max_angle,
        )
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

        self.joint_pos[6]*=(-1)
        self.joint_pos[7]*=(-1)
        self.update_action(self.joint_pos)


        if(self.joint_pos[6]> 0):
            return f"{self.joint_pos} \n 假爪已關閉"
        else:
            return f"{self.joint_pos} \n假爪已打開"
    def real_robot_position(self):
        """
        獲取實體機械手臂的當前位置
        """
        # 獲取實體機械手臂的當前位置
        real_robot_position = self.data_processor.get_realrobot_position()
        print(f"real_robot_position: {real_robot_position}")
        if real_robot_position is not None:
            crawl=self.joint_pos[6]
            self.joint_pos = real_robot_position
            self.joint_pos.append(crawl)
            self.joint_pos.append(-crawl)
            return f"實體機械手臂當前位置: {self.joint_pos}"
        else:
            return None

