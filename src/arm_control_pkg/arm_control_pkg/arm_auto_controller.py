from action_interface.action import ArmGoal
import time


class ArmAutoController:
    def __init__(self, arm_params, arm_commute_node, pybulletRobotController):
        self.arm_params = arm_params.get_arm_params()
        self.pybullet_robot_controller = pybulletRobotController
        self.arm_commute_node = arm_commute_node

    def arm_wave(self):
        pass

    def arm_ik_move(self):
        # t = self.pybullet_robot_controller.move_end_effector_laterally()
        # print(t[0:5])
        # self.pybullet_robot_controller.setJointPosition(position=t[0:5])
        t = self.pybullet_robot_controller.markPointInFrontOfEndEffector()
        print(t[0:5])
        self.pybullet_robot_controller.setJointPosition(position=t[0:5])
        return ArmGoal.Result(success=True, message="success")

    def test(self):
        t = self.pybullet_robot_controller.generateInterpolatedTrajectory(
            [0.3, 0.3, 0.3]
        )
        for i in t:
            self.pybullet_robot_controller.setJointPosition(position=i)
            time.sleep(0.1)
        self.pybullet_robot_controller.draw_end_effector_axes()
        return ArmGoal.Result(success=True, message="success")
