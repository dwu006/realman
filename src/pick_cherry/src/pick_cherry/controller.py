import numpy as np
from numpy.linalg import norm, solve
import pinocchio
# from pinocchio.visualize import MeshcatVisualizer
import crocoddyl
import rospy
from pick_cherry.utils import quat_multiply
import math
from scipy.spatial.transform import Rotation as R

from pink.tasks import FrameTask
from pink import solve_ik, Configuration

from pick_cherry.uni_mpc import MPCController

class Controller:
    def __init__(self, config):
        self.config = config

        # URDF & Pinocchio setup
        self.model, self.collision_model, self.visual_model = pinocchio.buildModelsFromUrdf(
            self.config.URDFPATH, self.config.MESH_DIR, pinocchio.JointModelFreeFlyer()
        )
        self.data = self.model.createData()
        self.tasks = {
            'base': FrameTask(self.config.PIN_BASE_FRAME_NAME, position_cost=1.0, orientation_cost=1.0),
            'r_gripper':FrameTask(self.config.PIN_GIRPPER_FRAME_NAME[1], position_cost=1.0, orientation_cost=1.0),
            'l_gripper':FrameTask(self.config.PIN_GIRPPER_FRAME_NAME[0], position_cost=1.0, orientation_cost=1.0)
        }

        # Useful frames in URDF
        self.base_id = self.model.getFrameId("base_link_underpan")
        self.l_gripper_id = self.model.getFrameId("l_gripper_base_link")
        self.r_gripper_id = self.model.getFrameId("r_gripper_base_link")
        self.headcam_id = self.model.getFrameId("camera_link")

        # initial config & visualizer
        self.q = pinocchio.neutral(self.model)
        pinocchio.forwardKinematics(self.model, self.data, self.q)
        # self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        # self.viz.initViewer(open=True)
        # self.viz.loadViewerModel(color=[1.0, 1.0, 1.0, 1.0])
        # self.viz.displayFrames(True)

        # Base Model for crocoddyl
        self.base_controller = MPCController(
            horizon = 200, dt = self.config.BASE_DT,
            Q = self.config.Q, R = self.config.R, Qf = self.config.Qf,
            v_max = self.config.v_max, omega_max = self.config.omega_max
        )
        
        print(f"model: {self.model}")

        # Pink setup
        self.configuration = Configuration(self.model, self.data, np.array(pinocchio.neutral(self.model)))
        for task in self.tasks.values():
            task.set_target_from_configuration(self.configuration)

    def pink_ik(self, cur_q, base_q, base_p, l_goal_q, l_goal_p, r_goal_q, r_goal_p):
        # Update current configuration (The base keeps going down when using current configuration?)
        # self.configuration.update(cur_q)
            
        # Update tasks with desired poses
        # base_rot_matrix = R.from_quat(base_q).as_matrix()
        # l_rot_matrix = R.from_quat(l_goal_q).as_matrix()
        # r_rot_matrix = R.from_quat(r_goal_q).as_matrix()

        self.tasks['base'].set_target(pinocchio.SE3(base_q, base_p))
        self.tasks['r_gripper'].set_target(pinocchio.SE3(r_goal_q, r_goal_p))
        self.tasks['l_gripper'].set_target(pinocchio.SE3(l_goal_q, l_goal_p))
        print("Tasks updated")
        print("Solving IK")
        velocity = solve_ik(self.configuration, self.tasks.values(), self.config.PIN_DT, solver="daqp")
        print("IK solved")

        self.configuration.integrate_inplace(velocity, self.config.PIN_DT)
        print("Configuration updated")
        # joint_command = [self.configuration.q[i] for i in self.config.PIN_Q_TO_JCOMMAND]
        # joint_command[-3] = 0.3  # platform joint
        return self.configuration.q
    
    def find_ik_pink(self, cur_q, base_q, base_p, l_goal_q, l_goal_p, r_goal_q, r_goal_p):
        # cur_q[3:7] = quat_multiply(cur_q[3:7], self.config.BASE_ORIENTATION_OFFSET)
        configuration = Configuration(self.model, self.data, cur_q)

        base_rot_matrix = R.from_quat(base_q).as_matrix()
        l_rot_matrix = R.from_quat(l_goal_q).as_matrix()
        r_rot_matrix = R.from_quat(r_goal_q).as_matrix()

        self.tasks['base'].set_target(pinocchio.SE3(base_rot_matrix, base_p))
        self.tasks['r_gripper'].set_target(pinocchio.SE3(r_rot_matrix, r_goal_p))
        self.tasks['l_gripper'].set_target(pinocchio.SE3(l_rot_matrix, l_goal_p))
        print("Tasks updated")
        for t in np.arange(0.0, 10, self.config.PIN_DT):
            velocity = solve_ik(configuration, self.tasks.values(), self.config.PIN_DT, solver="quadprog")
            configuration.integrate_inplace(velocity, self.config.PIN_DT)
        print(f"IK Results: {configuration.q.tolist()}")

        r_gripper_frame_id = self.model.getFrameId(self.config.PIN_GIRPPER_FRAME_NAME[1])
        l_gripper_frame_id = self.model.getFrameId(self.config.PIN_GIRPPER_FRAME_NAME[0])
        
        pinocchio.forwardKinematics(self.model, self.data, configuration.q)
        r_gripper_error = pinocchio.log(pinocchio.updateFramePlacement(self.model, self.data, r_gripper_frame_id).actInv(pinocchio.SE3(r_rot_matrix, r_goal_p))).vector
        l_gripper_error = pinocchio.log(pinocchio.updateFramePlacement(self.model, self.data, l_gripper_frame_id).actInv(pinocchio.SE3(l_rot_matrix, l_goal_p))).vector
        
        print(f"Right gripper error: {r_gripper_error}")
        print(f"Left gripper error: {l_gripper_error}")
        return configuration.q
    
    def find_arm_inverse_kinematics(self, curr_state, des_position, des_rot, arm_idx):

        des_rot =  des_rot @ self.config.PIN_ARM_ROTATION_OFFSET[arm_idx]
        frame_id = self.model.getFrameId(self.config.PIN_GIRPPER_FRAME_NAME[arm_idx])
        des_pose = pinocchio.SE3(des_rot, des_position)
        print("finding ik for arm", arm_idx, "with des_pose", des_pose)
        pin_q = curr_state.copy()
        pin_q[3:7] = quat_multiply(pin_q[3:7], self.config.BASE_ORIENTATION_OFFSET)
        # sol_viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        # sol_viz.initViewer(self.viz.viewer)
        # sol_viz.loadViewerModel(rootNodeName="ik_sol_viz" , color=[1.0, 1.0, 1.0, 0.5])
        SUCCESS = False
        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, pin_q)
            oMf = pinocchio.updateFramePlacement(self.model, self.data, frame_id)
            fMd = oMf.actInv(des_pose)
            err = pinocchio.log(fMd).vector
            if norm(err) < self.config.PIN_EPS:
                SUCCESS = True                                                      
                break
            if i >= self.config.PIN_IT_MAX:
                break
            J = pinocchio.computeFrameJacobian(self.model, self.data, pin_q, frame_id)
            J = -np.dot(pinocchio.Jlog6(fMd.inverse()), J)
            J_select = J[:,self.config.PIN_JACOB_JOINT_ID[arm_idx]]
            v_select = -J_select.T.dot(solve(J_select.dot(J_select.T) + self.config.PIN_DAMP * np.eye(6), err))
            v = np.zeros(21)
            v[self.config.PIN_JACOB_JOINT_ID[arm_idx]] = v_select
            pin_q = pinocchio.integrate(self.model, pin_q, v * self.config.PIN_DT)
            for idx, j in enumerate(self.config.PIN_JACOB_JOINT_ID[arm_idx]):
                pin_q[j] = np.clip(pin_q[j], self.config.JOINT_MIN[idx], self.config.JOINT_MAX[idx])
            # sol_viz.display(pin_q)
            if not i % 50:
                print(f"{i}: error = {err.T}")
                
            i += 1
        if SUCCESS:
            rospy.loginfo("IK success")
        else:
            rospy.logerr("IK failed")

        # convert pinocchio q to joint command
        # joint_command = [pin_q[i] for i in self.config.PIN_Q_TO_JCOMMAND]
        # pin_q[10:] = pin_q[10:] % (np.pi*2)
        print(f"\nresult: {pin_q.flatten().tolist()}")
        return pin_q, SUCCESS

    def diff_drive(self, target, init, T=10, dt = 0.01):
        """
        @brief Computes input trajectory for differential drive robot control

        Computes an input trajectory u(t) = [v(t) w(t)] for a differential drive robot
        using a Direct Dynamic Programming (DDP) solver.

        @param target Target pose [x, y, theta] in world frame
        @param init Initial pose in the same format as target
        @param T Time horizon or number of steps (default: 10)
        @param dt Time step in seconds (default: 0.01)

        @return List of control inputs u(t) = [v(t) w(t)] for t = 0, dt, 2*dt, ..., T

        @throws ValueError if the DDP solver fails to find a solution
        """
        e = (init - target).reshape(3, 1)
        problem = crocoddyl.ShootingProblem(e, [ self.base_model ] * T, self.base_model)
        ddp = crocoddyl.SolverDDP(problem)
        if ddp.solve():
            rospy.loginfo("DDP Solver Success")
            return ddp.us
        else:
            raise ValueError("Failed to solve the problem")
        
    def convert_pose_from_camera_to_odom(self, curr_state, pose):
        pin_q = curr_state.copy()
        pin_q[3:7] = quat_multiply(pin_q[3:7], self.config.BASE_ORIENTATION_OFFSET)
        print(f"self.corrected.state {pin_q}")
        
        pinocchio.forwardKinematics(self.model, self.data, pin_q)
        cam_frame_id = self.model.getFrameId("camera_link")
        oMf = pinocchio.updateFramePlacement(self.model, self.data, cam_frame_id)
        # the camera baselink is rotated by 90 degrees around the z axis
        offset = pinocchio.SE3(self.config.CAMERA_ROTATION_OFFSET, np.array([0,0,0]))
        cam_in_world = oMf.act(offset.act(pose))
        return cam_in_world
    
    def compute_frame_pose(self, q, frame_name):
        """
        Computes the end-effector pose for a given joint configuration.
        """
        pinocchio.forwardKinematics(self.model, self.data, q)
        frame_id = self.model.getFrameId(frame_name)
        oMf = pinocchio.updateFramePlacement(self.model, self.data, frame_id)
        return oMf