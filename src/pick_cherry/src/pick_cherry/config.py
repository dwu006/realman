#This is the config file for the project
import numpy as np
import rospkg
import os.path

class Config:
    def __init__(self):
        # URDF & Pinocchio setup
        rospack = rospkg.RosPack()
        self.URDFPATH = os.path.join(rospack.get_path('pick_cherry'), "urdf/overseas_65_corrected.urdf")
        self.MESH_DIR = "./urdf"
        self.FLOATING_BASE = True
        self.INIT_ARM = [0, -1.75, -0.6, 1.5, 0, 0, 0, 1.75, 0.6, -1.5, 0, 0]   # left arm and right arm
        # self.TRAVERSE_POSE = [-1.43117, -2.21657, 0.418879, 0, 0, 0, 1.43117, 2.21657, -0.418879, 0, 0, 0]
        self.INIT_JCOMMAND = [0, 0, 0, 0, 0, 0, 1, 0.5, 0, 0] + self.INIT_ARM + [0.2, 0, 0]  # arm + platform + finger 2
        self.JOINT_MSG_NAME = [f"l_joint{i}" for i in range(1, 7)] + [f"r_joint{i}" for i in range(1, 7)] + ["platform_joint", "l_finger_joint", "r_finger_joint"]
        self.CAMERA_ROTATION_OFFSET = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2), 0],
                                                [np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                                                [0, 0, 1]])
        self.BASE_ORIENTATION_OFFSET = np.array([0, 0 , np.sin(np.pi/2 / 2), np.cos(np.pi/2 / 2)])  # quaternion for 90 degree rotation around z axis

        # self.HANDEL_PREGRIP_OFFSET = np.array([-0.27, 0.01, -0.06])
        # self.HANDEL_GRIP_OFFSET    = self.HANDEL_PREGRIP_OFFSET +  np.array([0.08, 0.0, 0.0])
        # self.HANDEL_TURN_OFFSET    = self.HANDEL_GRIP_OFFSET + np.array([0.0, +0.02, -0.06])
        # self.HANDEL_PUSH_OFFSET    = self.HANDEL_TURN_OFFSET + np.array([0.04, 0, 0])
        # # 45 degree turn on the x axis
        # self.HANDEL_TURN_ROTATION = np.array([[1, 0, 0],
        #                                       [0, np.cos(np.pi/3), -np.sin(np.pi/3)],
        #                                         [0, np.sin(np.pi/3), np.cos(np.pi/3)]])
        
        # pinocchio parameters
        self.PIN_L_ENDFECCTOR_JOINT_ID = 10
        self.PIN_R_ENDFECCTOR_JOINT_ID = 16
        self.PIN_EPS = 1e-3
        self.PIN_DT = 0.01
        self.PIN_IT_MAX = 1000
        self.PIN_DAMP = 1e-6
        self.PIN_LARM_ROTATION_OFFSET = np.array([
                                            [0,  0,  1],
                                            [0, -1,  0],
                                            [1,  0,  0]])
        self.PIN_RARM_ROTATION_OFFSET = np.array([
                                            [0,  0, 1],
                                            [0, 1,  0],
                                            [-1, 0,  0]])
        self.PIN_ARM_ROTATION_OFFSET = [self.PIN_LARM_ROTATION_OFFSET, self.PIN_RARM_ROTATION_OFFSET]
        self.PIN_GIRPPER_FRAME_NAME = ["l_gripper_base_link", "r_gripper_base_link"]
        self.PIN_BASE_FRAME_NAME = "base_link"
        self.PIN_PLATFORM_FRAME_NAME = "platform_base_link"
        self.PIN_JACOB_JOINT_ID = [[9, 10, 11, 12, 13, 14], [15, 16, 17, 18, 19, 20]] # left and right arm
        self.PIN_Q_TO_JCOMMAND = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 7, 8, 9]   # arm + platform + head joints

        self.BASE_POSE_OFFSET = np.array([0.6, 0.3])


        self.GRIPPER_ROTATION_OFFSET = np.array([[1, 0, 0],
                                 [0, np.cos(np.pi/2), -np.sin(np.pi/2)],
                                 [0, np.sin(np.pi/2), np.cos(np.pi/2)]]) 
        self.MAX_DOORHANDLE_DIST_X = 0.9

                # Joint Limits
        self.JOINT_MIN = np.array([-3.11, -2.27, -2.36, -3.11, -2.23, -6.28])
        self.JOINT_MAX = np.array([3.11, 2.27, 2.36, 3.11, 2.23, 6.28])
        self.RIGHT_ARM_REACH_OFFSET = self.HANDEL_PUSH_OFFSET + np.array([0, -0.5, 0.0])

        if self.FLOATING_BASE:
          self.PIN_JACOB_JOINT_ID = [[9, 10, 11, 12, 13, 14], [15, 16, 17, 18, 19, 20]] # left and right arm
          self.PIN_Q_TO_JCOMMAND = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 7, 8, 9]   # arm + platform + finger 2
        else:
          self.PIN_JACOB_JOINT_ID = [[3, 4, 5, 6, 7, 8], [9, 10, 11, 12, 13, 14]] # left and right arm
          self.PIN_Q_TO_JCOMMAND = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 1, 2]   # arm + platform + finger 2
        # Parameters for Pulling Door
        self.PULL_BASE_OFFSET = np.array([0.56, -0.14, 0.0])

        # Mobile Base Controller Parameters
        self.U_BASE_MIN = np.array([-1.0, -np.pi/3])
        self.U_BASE_MAX = np.array([+1.0, +np.pi/3])
        self.BASE_DT = 0.01
        self.PULL_TURN = np.pi/4

        self.Q = np.diag([5.0, 5.0, 0.0])
        self.R = np.diag([10.0, 0.1])
        self.Qf = np.diag([15.0, 15.0, 3.0])
        self.v_max = 0.3
        self.omega_max = 0.3