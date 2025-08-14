import numpy as np

from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
class RealmanState:
    def __init__(self, config):
        # state buffers x, y, z, rx, ry, rz, rw, platform, head 2, l 6, r 6. Order is made to match the pinocchio model
        # self.initial_base_pose = None
        self.state = np.zeros(22) 



    def update_state(self, state):
        self.state = state

    def update_joint_state(self, jstate_msg):
        # This function is used for Isaac Sim
        self.state[7] = jstate_msg.position[0]
        pos_map = dict(zip(jstate_msg.name, jstate_msg.position))
        for side, base in (('l', 10), ('r', 16)):
            self.state[base:base + 6] = [pos_map[f"{side}_joint{i}"] for i in range(1, 7)]

    def update_platform_state(self, value):
        # Use with Realman Robot. Updates the platform position in the callback function
        self.state[7] = value
        pass

    def update_right_arm_state(self, values):
        # Use with Realman Robot. Updates the platform position in the callback function
        self.state[16:22] = values
        pass

    def update_left_arm_state(self, values):
        # Use with Realman Robot. Updates the platform position in the callback function
        self.state[10:16] = values
        pass

    def update_base_pose(self, msg):
        # if self.initial_base_pose is None:
        #     self.initial_base_pose = values
        #     self.state[:7] = np.array([
        #         0, 0, 0.24,
        #         0, 0, 0, 1.0
        #     ])
        # else:
        #     self.state[:2] = values[:2] - self.initial_base_pose[:2]
        #     angle_diff = values[2] - self.initial_base_pose[2]
        #     w = np.cos(angle_diff / 2.0)
        #     z = np.sin(angle_diff / 2.0)
        #     x = 0.0
        #     y = 0.0
        #     self.state[3:7] = np.array([
        #         x, y, z, w
        #     ])
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = 0.24
        self.state[3] = msg.pose.pose.orientation.x
        self.state[4] = msg.pose.pose.orientation.y
        self.state[5] = msg.pose.pose.orientation.z
        self.state[6] = msg.pose.pose.orientation.w

    def update_head_pose(self, joint_1, joint_2):
        # self.state[8] = joint_1
        # self.state[9] = joint_2
        self.state[8] = 0
        self.state[9] = 0
        pass
