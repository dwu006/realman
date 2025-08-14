#!/usr/bin/env python3.8

import rospy

import actionlib
from woosh_msgs.msg import MoveBaseAction, MoveBaseGoal
from dual_arm_msgs.msg import MoveJ, GetArmState_Command, ArmState, JointPos, Joint_Current, LiftState, Lift_Height
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PolygonStamped, Polygon, PointStamped
from nav_msgs.msg import Odometry

import numpy as np
from array import array
import pinocchio as pin
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point
from tf2_msgs.msg import TFMessage

from pick_cherry.config import Config
from pick_cherry.realmanState import RealmanState
from pick_cherry.controller import Controller
from pick_cherry.gripper import Gripper
from enum import Enum, auto

from pick_cherry.trajectory import create_waypoint_trajectory, generate_waypoint_trajectory, create_arc_trajectory, create_circular_trajectory

import matplotlib.pyplot as plt

class RobotState(Enum):
    INIT_POSE    = auto()
    DETECT_SCENE = auto()
    DETECT_CHERRY = auto()
    BASE_APPROACH = auto()
    ARM_LIFT_APPROACH  = auto()
    PICK_CHERRY = auto()
    PLACE_CHERRY = auto()

    # add more states here as you go…
    # LIFT   = auto()
    # DONE   = auto()


class RealmanControlNode():
    def __init__(self):
        rospy.init_node('RealmanControlNode')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # publishers
        self.pub_MoveJ_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=1)
        self.pub_MoveJ_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=1)
        self.base_pub = rospy.Publisher('/base_cmd_vel', Twist, queue_size=1)
        self.lift_publisher = rospy.Publisher('/l_arm/rm_driver/Lift_SetHeight', Lift_Height, queue_size=1)
        self.woosh_nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.local_footprint_pub = rospy.Publisher('/move_base/local_costmap/set_footprint', Polygon, queue_size=10)
        self.global_footprint_pub = rospy.Publisher('/move_base/global_costmap/set_footprint', Polygon, queue_size=10)
        self.cmd_vel_goal_pub = rospy.Publisher('/dummy_cmd_vel_goal', Twist, queue_size=1)

        while self.woosh_nav_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print("RealmanControlNode initialized")
        print(self.woosh_nav_pub.get_num_connections())

        self.config = Config()
        self.rm_state = RealmanState(self.config)
        self.rm_controller = Controller(self.config)

         # Manually set the footprint size for move base
        depth = 0.3
        width = 0.35
        points = [
            (-width,  depth),
            ( width,  depth),
            ( width,  -depth),
            (-width,  -depth)
        ]

        polygon = Polygon()
        for x, y in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            polygon.points.append(pt)
        self.local_footprint_pub.publish(polygon)
        self.global_footprint_pub.publish(polygon)
        rospy.loginfo("Footprint published to move_base")

        # self.door_handle_pose = None  # [x, y, z, rx, ry, rz, rw]

        self.state = RobotState.INIT_POSE
        self.state_start_time = rospy.Time.now()
        # self.pregrasp_jcmd = None
        # self.grasp_jcmd = None
        # self.pull_jcmd = None
        # self.turn_jcmd = None
        # self.open_jcmd = None
        # self.pull_base_cmd = None
        # self.push_base_cmd = None
        # self.grip_Handle_pose = None
        self.reference_traj = None
        self.first_entry = True
        # variables for plotting the base MPC test
        self.robot_path = list([])
        self.start_pose = None
        self.goal_pose = None

        self.l_gripper = Gripper(ip='169.254.128.18')
        self.r_gripper = Gripper(ip='169.254.128.19')

        # Topics for requesting feedback
        self.get_r_arm_state_pub = rospy.Publisher('/r_arm/rm_driver/GetArmState_Cmd', GetArmState_Command, queue_size=10)
        self.get_l_arm_state_pub = rospy.Publisher('/l_arm/rm_driver/GetArmState_Cmd', GetArmState_Command, queue_size=10)
        self.get_r_arm_current_pub = rospy.Publisher('/r_arm/rm_driver/GetArmCurrent_Cmd', Empty, queue_size=10)
        self.get_l_arm_current_pub = rospy.Publisher('/l_arm/rm_driver/GetArmCurrent_Cmd', Empty, queue_size=10)
        self.get_lift_state_pub = rospy.Publisher('/l_arm/rm_driver/Lift_GetState', Empty, queue_size=10)
        self.base_vel_pub = rospy.Publisher('/base_cmd_vel', Twist, queue_size=1)

        # subscriptions
        self.handle_sub = rospy.Subscriber('/grip_point', PointStamped, self.grip_point_callback)
        self.l_arm_sub = rospy.Subscriber('/l_arm/joint_states', JointState, self.l_joint_angles)
        self.r_arm_sub = rospy.Subscriber('/r_arm/joint_states', JointState, self.r_joint_angles)
        self.base_pose_sub = rospy.Subscriber('/odom', Odometry, self.base_pose)
        self.lift_state_sub = rospy.Subscriber('/l_arm/rm_driver/LiftState', LiftState, self.update_lift_state)
        # self.head_pose_sub = rospy.Subscriber('/servo_state', Pose2D, self.head_pose)
        
        # start timer
        self.feedback_timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        self.timer = rospy.Timer(rospy.Duration(0.01), self._control_loop)
        self.done_approach_time_stamp = rospy.Time.now() 
        self.approached = False

        # self.pregrasp_pose = None
        # self.grasp_pose = None
        # self.hold_door_left_pose = None
        # self.hold_door_left_reach_pose = None
        # self.hold_door_right_pose = None
        # self.hold_door_right_backward_pose = None
        # self.pull_swing_pose = None
        # self.push_swing_pose = None


    def timer_callback(self, event):
        msg = GetArmState_Command()
        self.get_r_arm_state_pub.publish(msg)
        self.get_l_arm_state_pub.publish(msg)
        msg = Empty()
        self.get_r_arm_current_pub.publish(msg)
        self.get_l_arm_current_pub.publish(msg)
        self.get_lift_state_pub.publish(msg)

    def basePoseCallback(self, msg):
        self.rm_state.update_base_pose(msg)
    
    def convert_lift_state(self, value, real2urdf):
        lb_urdf, up_urdf = 0.9 , 0.08
        lb_real, up_real = 0, 776
        if real2urdf:
            return (value * (up_urdf - lb_urdf) / up_real) + lb_urdf
            # return value * (up_urdf - lb_urdf) / (up_real - lb_real) + lb_urdf
        else:
            return (value - lb_urdf) * (up_real - lb_real) / (up_urdf - lb_urdf)

    def update_lift_state(self, msg):
        value = self.convert_lift_state(msg.height, real2urdf=True)
        self.rm_state.update_platform_state(value)

    def grip_point_callback(self, msg):
        if self.door_handle_pose is None and self.rm_state.state[2] ==0.24:

            pose_in_camera = np.array([msg.point.x, msg.point.y, msg.point.z]) / 1000.0

            # convert to world frame
            self.door_handle_pose = self.rm_controller.convert_pose_from_camera_to_odom(
                self.rm_state.state,
                pose_in_camera
            )
            print(f"pose_in_camera {pose_in_camera}")
            print(f"self.rm_state.state {self.rm_state.state}")
            print(f"door_handle_pose {self.door_handle_pose}")
            print("delay ", (rospy.Time.now()- msg.header.stamp ).to_sec() )

    def head_pose(self, msg):
        self.rm_state.update_head_pose(joint_1 = msg.angle_1, joint_2 = msg.angle_2)
        pass

    def l_joint_angles(self, msg):
        self.rm_state.update_left_arm_state(msg.position)
        # rospy.loginfo(f"Left Arm Joint States Updated: {msg.position}")
        pass

    def r_joint_angles(self, msg):
        self.rm_state.update_right_arm_state(msg.position)
        # rospy.loginfo(f"Left Arm Joint States Updated: {msg.position}")
        pass

    def base_pose(self, msg):
        self.rm_state.update_base_pose(msg)
        pass

    def initPose(self):
        self.sendRosCommand(self.config.INIT_JCOMMAND)

    def _control_loop(self, event):
        # dispatch based on current state
        if self.state == RobotState.INIT_POSE:
            self._handle_init_pose()
        # elif self.state == RobotState.DETECT_HANDLE:
        #     self._handle_detect_handle()
        # elif self.state == RobotState.APPROACH:
        #     self._approach_door()
        # elif self.state == RobotState.PREGRASP:
        #     self._handle_pregrasp()
        # elif self.state == RobotState.GRASP:
        #     self._handle_grasp()
        # elif self.state == RobotState.PULL:
        #     self._handle_pull()
        # elif self.state == RobotState.HOLD_DOOR_LEFT:
        #     self._handle_hold_door_left()
        # elif self.state == RobotState.HOLD_DOOR_RIGHT:
        #     self._handle_hold_door_right()
        # elif self.state == RobotState.PUSH_DOOR:
        #     self._handle_push_door()
        
        # self._approach_door()
        
        # visualize 
        # if self.config.FLOATING_BASE:
        #     self.rm_controller.viz.display(self.rm_state.state)
        # else:
        #     self.rm_controller.viz.display(self.rm_state.state[7:])
    
    def _handle_init_pose(self):
        # keep sending init-pose until 200 ticks have elapsed
        self.sendRosCommand(self.config.INIT_JCOMMAND)

        if (rospy.Time.now() - self.state_start_time).to_nsec() > 500 * 10_000_000:
            # 300 * 0.01s == 3 seconds
            # self._transition_to(RobotState.IKTEST)
            self._transition_to(RobotState.APPROACH)

    def _handle_detect_handle(self):
        if self.grip_Handle_pose is not None:
            rospy.loginfo("Handle detected")
            self._transition_to(RobotState.PREGRASP)

    def plot_approach_trajectory(self):
        """
        Create and save a plot showing the robot's approach trajectory
        """
        if len(self.robot_path) < 2:
            rospy.logwarn("Insufficient trajectory data for plotting")
            return
        
        # Convert to numpy arrays for easier handling
        positions = np.array(self.robot_path)
        x_traj = positions[:, 0]
        y_traj = positions[:, 1]
        theta_traj = positions[:, 2]
        
        # Create figure with subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        # Plot 1: XY trajectory
        ax1.plot(x_traj, y_traj, 'b-', linewidth=2, label='Robot Path', alpha=0.8)
        ax1.plot(self.start_pose[0], self.start_pose[1], 'go', markersize=12, label='Start Position', markeredgecolor='black')
        ax1.plot(self.goal_pose[0], self.goal_pose[1], 'ro', markersize=12, label='Goal Position', markeredgecolor='black')
        ax1.plot(x_traj[-1], y_traj[-1], 'bo', markersize=8, label='Final Position', markeredgecolor='black')
        
        # Add orientation arrows at key points
        n_arrows = min(10, len(positions))
        if n_arrows > 0:
            arrow_indices = np.linspace(0, len(positions)-1, n_arrows, dtype=int)
            for i in arrow_indices:
                if i < len(positions):
                    dx = 0.1 * np.cos(theta_traj[i])
                    dy = 0.1 * np.sin(theta_traj[i])
                    ax1.arrow(x_traj[i], y_traj[i], dx, dy, 
                             head_width=0.03, head_length=0.03, 
                             fc='blue', ec='blue', alpha=0.6)
        
        ax1.set_xlabel('X Position [m]')
        ax1.set_ylabel('Y Position [m]')
        ax1.set_title('Robot Approach Trajectory (XY)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Add distance annotation
        total_distance = 0
        for i in range(1, len(positions)):
            total_distance += np.linalg.norm(positions[i][:2] - positions[i-1][:2])
        
        direct_distance = np.linalg.norm(self.goal_pose[:2] - self.start_pose[:2])
        efficiency = direct_distance/total_distance if total_distance > 0 else 0
        
        ax1.text(0.02, 0.98, f'Total Distance: {total_distance:.3f}m\nDirect Distance: {direct_distance:.3f}m\nEfficiency: {efficiency:.2f}', 
                transform=ax1.transAxes, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Plot 2: Position vs Step (since we don't have timestamps)
        steps = np.arange(len(positions))
        ax2.plot(steps, x_traj, 'r-', linewidth=2, label='X position')
        ax2.plot(steps, y_traj, 'g-', linewidth=2, label='Y position')
        ax2.axhline(y=self.goal_pose[0], color='r', linestyle='--', alpha=0.7, label='Goal X')
        ax2.axhline(y=self.goal_pose[1], color='g', linestyle='--', alpha=0.7, label='Goal Y')
        ax2.set_xlabel('Control Step')
        ax2.set_ylabel('Position [m]')
        ax2.set_title('Position vs Control Steps')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Orientation vs Step
        ax3.plot(steps, theta_traj, 'b-', linewidth=2, label='Robot Heading')
        ax3.axhline(y=self.goal_pose[2], color='b', linestyle='--', alpha=0.7, label='Goal Heading')
        ax3.set_xlabel('Control Step')
        ax3.set_ylabel('Heading [rad]')
        ax3.set_title('Heading vs Control Steps')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Distance to Goal vs Step
        distances_to_goal = []
        for pos in positions:
            dist = np.linalg.norm(pos[:2] - self.goal_pose[:2])
            distances_to_goal.append(dist)
        
        ax4.plot(steps, distances_to_goal, 'm-', linewidth=2, label='Distance to Goal')
        ax4.axhline(y=0.1, color='r', linestyle='--', alpha=0.7, label='Tolerance (0.1m)')
        ax4.set_xlabel('Control Step')
        ax4.set_ylabel('Distance [m]')
        ax4.set_title('Distance to Goal vs Control Steps')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Add overall title with summary
        final_error = distances_to_goal[-1] if distances_to_goal else float('inf')
        num_steps = len(positions)
        
        fig.suptitle(f'Robot Approach Trajectory Analysis\n'
                    f'Steps: {num_steps}, Final Error: {final_error:.3f}m, '
                    f'Path Efficiency: {efficiency:.2f}', 
                    fontsize=14, y=0.95)
        
        plt.tight_layout()
        
        # Save plot with timestamp
        plt.savefig('/home/rm/alphaz_ws/src/door_traverse/log/plot.png', dpi=300, bbox_inches='tight')

        
    # def _approach_door(self):
    #     # grip_pose = np.array([self.door_handle_pose[0], self.door_handle_pose[1], 0.0])
    #     # goal = grip_pose - self.config.PULL_BASE_OFFSET
    #     goal = np.array([1.2, -0.4, -np.pi/2])
    #     goal = np.array([0.0, 0.0, 0.0])
    #     qx, qy, qz, qw = self.rm_state.state[3], self.rm_state.state[4], self.rm_state.state[5], self.rm_state.state[6]
    #     yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    #     current_state = np.array([self.rm_state.state[0], self.rm_state.state[1], yaw])
    #     self.reference_traj = goal * np.ones((self.rm_controller.base_controller.N + 1, 3))
    #     rospy.loginfo(f"Current State: {current_state}, Goal: {goal}")

    #     self.goal_pose = goal.copy()
    #     if self.start_pose is None:
    #         self.start_pose = current_state
    #     self.robot_path.append(current_state.copy())
        
    #     if np.linalg.norm((goal - current_state)[0:2]) > 0.075 or abs(goal[2] -current_state[2]) > 0.1:
    #         control_input, success = self.rm_controller.base_controller.control(current_state, self.reference_traj)
    #         if success:
    #             rospy.loginfo(f"Control Input: {control_input.T}")
    #             self.sendRosCommand(base_command=control_input)
    #         else:
    #             rospy.logwarn("Base controller failed to compute control input.")
    #             self.sendRosCommand(base_command=[0.0, 0.0])
    #     else:
    #         self.sendRosCommand(base_command=[0.0, 0.0])
    #         self._transition_to(RobotState.PREGRASP)
    #         self.plot_approach_trajectory()
    #         self.reference_traj = None

    # def _handle_pregrasp(self):
    #     if self.door_handle_pose is not None and self.pregrasp_pose is None:
    #         base_world_rot = pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix()
    #         # Rotate along x for 90 degrees and then 180 degrees along z
    #         door_handle_rot = base_world_rot @ self.config.HANDLE_PREGRASP_ROTATION_OFFSET
    #         door_handle_pose_des = pin.SE3(door_handle_rot, self.door_handle_pose[:3])
    #         # Transform the handle pose to the local frame with the offset
    #         base_pose_world = pin.SE3(base_world_rot, self.rm_state.state[0:3])
    #         door_handle_pose_des_local = base_pose_world.actInv(door_handle_pose_des)
    #         door_handle_pose_des_local.translation += self.config.HANDLE_PREGRASP_TRANSLATION_OFFSET_LOCAL
    #         door_handle_pose_des = base_pose_world.act(door_handle_pose_des_local)
    #         self.pregrasp_pose = door_handle_pose_des
    #         self.initial_l_hand_pose = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #         self.initial_r_hand_pose = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)
    #         self.state_start_time = rospy.Time.now()

    #     if self.pregrasp_pose is not None:
    #         self.pregrasp_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         self.initial_l_hand_pose.rotation, 
    #                                                         self.initial_l_hand_pose.translation,
    #                                                         self.pregrasp_pose.rotation, 
    #                                                         self.pregrasp_pose.translation)
    #         self.sendRosCommand(self.pregrasp_jcmd)
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 300 * 10_000_000:
    #             self._transition_to(RobotState.GRASP)

    # def _handle_grasp(self):
    #     if self.door_handle_pose is not None and self.grasp_pose is None:
    #         base_world_rot = pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix()
    #         # Rotate along x for 90 degrees and then 180 degrees along z
    #         door_handle_rot = base_world_rot @ self.config.HANDLE_PREGRASP_ROTATION_OFFSET
    #         door_handle_pose_des = pin.SE3(door_handle_rot, self.door_handle_pose[:3])
    #         # Transform the handle pose to the local frame with the offset
    #         base_pose_world = pin.SE3(base_world_rot, self.rm_state.state[0:3])
    #         door_handle_pose_des_local = base_pose_world.actInv(door_handle_pose_des)
    #         door_handle_pose_des_local.translation += self.config.HANDLE_GRASP_TRANSLATION_OFFSET_LOCAL
    #         door_handle_pose_des = base_pose_world.act(door_handle_pose_des_local)
    #         self.grasp_pose = door_handle_pose_des
    #         self.initial_l_hand_pose = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #         self.initial_r_hand_pose = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)
    #         self.state_start_time = rospy.Time.now()

    #     if self.grasp_pose is not None:
    #         self.grasp_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         self.initial_l_hand_pose.rotation, 
    #                                                         self.initial_l_hand_pose.translation,
    #                                                         self.grasp_pose.rotation, 
    #                                                         self.grasp_pose.translation)
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 200 * 10_000_000:
    #             self.r_gripper.close()
                
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 400 * 10_000_000:
    #             self.pull_jcmd = self.grasp_jcmd
    #             self.sendRosCommand(self.grasp_jcmd)
    #             self._transition_to(RobotState.PULL)

    #         self.sendRosCommand(self.grasp_jcmd)
    
    # def _handle_pull(self):
    #     # Keep sending the same joint command as the previous state
    #     if self.pull_base_cmd is None:
    #         backward_speed = -0.2
    #         if backward_speed == 0.0:
    #             self.pull_travel_time = 0.1
    #         else:
    #             self.pull_travel_time = 1.5 / abs(backward_speed)
    #         self.pull_swing_time = 3.0
    #         # swing_time = 1
    #         self.pull_base_cmd = np.array([backward_speed, 0.0])
    #         self.state_start_time = rospy.Time.now()
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)

    #         # Initialize trajectory parameters
    #         self.trajectory_start_time = None
            

    #     if self.pull_base_cmd is not None:
    #         # Swing the arm to the right to open the door
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 100 * self.pull_travel_time * 10_000_000:
    #             if self.pull_swing_pose is None:
    #                 r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #                 self.pull_swing_pose = r_hand.copy()
    #                 self.pull_swing_pose.translation += r_hand.rotation @ np.array([-0.15, 0.0, -0.05])
    #                 self.waypoints = create_waypoint_trajectory(
    #                     r_hand,
    #                     self.pull_swing_pose
    #                 )
    #                 self.trajectory_duration = self.pull_swing_time
    #                 self.trajectory_start_time = rospy.Time.now()

    #             elapsed_time = (rospy.Time.now() - self.trajectory_start_time).to_nsec() / 1e9
    #             current_target_pose = generate_waypoint_trajectory(
    #                 self.waypoints,
    #                 self.trajectory_duration,
    #                 elapsed_time
    #             )
    #             self.pull_swing_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         None, None,
    #                                                         current_target_pose.rotation, 
    #                                                         current_target_pose.translation)
    #             # self.r_gripper.close()
    #             self.pull_base_cmd = np.array([0.0, 0.0])
    #             self.sendRosCommand(self.pull_swing_jcmd)
    #             # self._transition_to(RobotState.HOLD_DOOR_LEFT)
            
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 100 * (self.pull_travel_time + self.pull_swing_time) * 10_000_000:
    #             self._transition_to(RobotState.HOLD_DOOR_LEFT)
            
    #         self.sendRosCommand(base_command=self.pull_base_cmd)

    # def _handle_hold_door_left(self):
    #     if self.first_entry is True:
    #         self.first_entry = False
    #         rospy.loginfo("Entering HOLD_DOOR_LEFT state and fetching initial pose command")
    #         # rospy.loginfo("Initial EE pose: ")
    #         l_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #         r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #         self.initial_l_hand_pose = l_hand
    #         self.initial_r_hand_pose = r_hand
    #         self.initial_r_hand_pose_local = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1], world_frame=False)
    #         # rospy.loginfo(f"Left hand pose: {l_hand}")
    #         # rospy.loginfo(f"Right hand pose: {r_hand}")
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)
    #         self.hold_door_left_approach_time = 0.0
    #         self.hold_door_left_prehold_time = 3.0
    #         self.hold_door_left_reach_time = 3.0
    #         self.state_start_time = rospy.Time.now()

    #     base_cmd = np.array([0.0, 0.0])
        
    #     # Approaching the door
    #     if (rospy.Time.now() - self.state_start_time).to_nsec() < 100 * self.hold_door_left_approach_time * 10_000_000:
    #         self.hold_door_left_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         None, None,
    #                                                         self.initial_r_hand_pose.rotation, 
    #                                                         self.initial_r_hand_pose.translation)
    #         # base_cmd = np.array([0.2, 0.0])
    #         base_cmd = np.array([0.0, 0.0])
    #         # The right arm gripper keeps closing
    #         self.r_gripper.close()

    #     # Preholding the door (position and orientation are hardcoded relative to the right hand)
    #     if 100 * (self.hold_door_left_approach_time) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #         < 100 * (self.hold_door_left_approach_time + self.hold_door_left_prehold_time) * 10_000_000:
    #         if self.hold_door_left_pose is None:
    #             l_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #             r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #             self.initial_l_hand_pose = l_hand
    #             self.initial_r_hand_pose = r_hand
    #             base_world_rot = pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix()
    #             base_pose_world = pin.SE3(base_world_rot, self.rm_state.state[0:3])
    #             r_hand_local = base_pose_world.actInv(r_hand)
    #             l_hand_local = base_pose_world.actInv(l_hand)
    #             l_hand_local.translation = r_hand_local.translation + self.config.HOLD_DOOR_LEFT_TRANSLATION_OFFSET_FROM_R_HAND
    #             self.hold_door_left_pose = base_pose_world.act(l_hand_local)
    #             self.hold_door_left_pose.rotation = self.hold_door_left_pose.rotation @ self.config.HOLD_DOOR_LEFT_ROTATION_OFFSET
    #             # self.rm_controller.update_pink_ik_configuration(self.rm_state.state)

    #         self.hold_door_left_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         self.hold_door_left_pose.rotation, 
    #                                                         self.hold_door_left_pose.translation,
    #                                                         None, None)
    #         base_cmd = np.array([0.0, 0.0])
    #         self.r_gripper.close()

    #     # Reach the door to hold it with an offset from the left hand in the left hand frame
    #     if  100 * (self.hold_door_left_approach_time + self.hold_door_left_prehold_time) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #         < 100 * (self.hold_door_left_approach_time + self.hold_door_left_prehold_time + self.hold_door_left_reach_time) * 10_000_000:
    #         if self.hold_door_left_reach_pose is None:
    #             l_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #             self.hold_door_left_reach_pose = l_hand
    #             self.hold_door_left_reach_pose.translation += l_hand.rotation @ self.config.HOLD_DOOR_LEFT_REACH_TRANSLATION_OFFSET
            
    #         self.hold_door_left_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         self.hold_door_left_reach_pose.rotation, 
    #                                                         self.hold_door_left_reach_pose.translation,
    #                                                         None, None)
    #         base_cmd = np.array([0.0, 0.0])
    #         self.r_gripper.close()
    #         # self._transition_to(RobotState.HOLD_DOOR_RIGHT)

            
    #     self.sendRosCommand(self.hold_door_left_jcmd, base_cmd)

    #     if (rospy.Time.now() - self.state_start_time).to_nsec() >= 100 * (self.hold_door_left_approach_time + self.hold_door_left_prehold_time + self.hold_door_left_reach_time) * 10_000_000:
    #         self.first_entry = True
    #         self._transition_to(RobotState.HOLD_DOOR_RIGHT)

    # def _handle_hold_door_right(self):
    #     if self.first_entry is True:
    #         self.first_entry = False
    #         rospy.loginfo("Entering HOLD_DOOR_RIGHT state and fetching initial pose command")
    #         # rospy.loginfo("Initial EE pose: ")
    #         l_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #         r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #         self.initial_l_hand_pose = l_hand
    #         self.initial_r_hand_pose = r_hand
    #         # rospy.loginfo(f"Left hand pose: {l_hand}")
    #         # rospy.loginfo(f"Right hand pose: {r_hand}")
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)
    #         self.hold_door_right_release_gripper_time = 3.0
    #         self.hold_door_right_backward_arm_time = 2.5
    #         self.hold_door_right_contact_switch = 8.0
    #         self.hold_door_right_put_down_left_arm = 4.0
    #         self.state_start_time = rospy.Time.now()
            
    #         # Initialize trajectory parameters
    #         self.trajectory_start_time = None
    #         self.trajectory_start_time_homing = None
    #         self.initial_r_hand_pose_for_trajectory = None
    #         self.final_r_hand_pose_for_trajectory = None

    #     base_cmd = np.array([0.0, 0.0])

    #     # Fix the joint angles and keep the right gripper open
    #     if (rospy.Time.now() - self.state_start_time).to_nsec() < 100 * (self.hold_door_right_release_gripper_time) * 10_000_000:
    #         self.hold_door_right_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                             pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                             self.rm_state.state[0:3],
    #                                                             None, None,
    #                                                             None, None)
    #         self.r_gripper.open()

    #     # Backward the right arm
    #     if 100 * (self.hold_door_right_release_gripper_time) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #         < 100 * (self.hold_door_right_release_gripper_time + self.hold_door_right_backward_arm_time) * 10_000_000:
    #         if self.hold_door_right_backward_pose is None:
    #             r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #             self.initial_r_hand_pose = r_hand.copy()
    #             self.hold_door_right_backward_pose = r_hand.copy()
    #             self.hold_door_right_backward_pose.translation += r_hand.rotation @ self.config.HOLD_DOOR_RIGHT_BACKWARD_TRANSLATION_OFFSET
    #             self.backward_trajectory_start_time = rospy.Time.now()
    #             self.backward_waypoints = create_waypoint_trajectory(
    #                 self.initial_r_hand_pose, self.hold_door_right_backward_pose
    #             )
            
    #         # Calculate trajectory progress
    #         elapsed_time = (rospy.Time.now() - self.backward_trajectory_start_time).to_nsec() / 1e9

    #         # Generate smooth trajectory through waypoints
    #         current_target_pose = generate_waypoint_trajectory(
    #             self.backward_waypoints,
    #             self.hold_door_right_backward_arm_time,
    #             elapsed_time
    #         )

    #         self.hold_door_right_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         None, None,
    #                                                         current_target_pose.rotation, 
    #                                                         current_target_pose.translation)
    #         self.r_gripper.open()

    #     # Generate and track trajectory for the right arm
    #     if 100 * (self.hold_door_right_release_gripper_time + self.hold_door_right_backward_arm_time) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #         < 100 * (self.hold_door_right_release_gripper_time + self.hold_door_right_backward_arm_time + self.hold_door_right_contact_switch) * 10_000_000:
    #         # Initialize trajectory on first entry to this phase
    #         if self.trajectory_start_time is None:
    #             self.trajectory_start_time = rospy.Time.now()
    #             l_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[0])
    #             r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, self.config.PIN_GIRPPER_FRAME_NAME[1])
    #             self.initial_r_hand_pose_for_trajectory = r_hand.copy()
                
    #             # Create final pose with offset
    #             self.mid_r_hand_pose_for_trajectory = r_hand.copy()
    #             self.mid_r_hand_pose_for_trajectory.rotation = r_hand.rotation @ pin.utils.rpyToMatrix(np.array([0.0, 0.0, -np.pi/2.0]))
    #             self.mid_r_hand_pose_for_trajectory.translation += r_hand.rotation @ np.array([0.2, -0.15, 0.0])
    #             self.final_r_hand_pose_for_trajectory = r_hand.copy()
    #             self.final_r_hand_pose_for_trajectory.rotation = r_hand.rotation @ pin.utils.rpyToMatrix(np.array([0.0, 0.0, -np.pi/2.0])) \
    #                 @ pin.utils.rpyToMatrix(np.array([np.pi/8.0, 0.0, 0.0]))
    #             self.final_r_hand_pose_for_trajectory.translation += r_hand.rotation @ np.array([0.22, -0.15, 0.12])
    #             # self.final_r_hand_pose_for_trajectory = r_hand.copy()
    #             # self.final_r_hand_pose_for_trajectory.rotation = r_hand.rotation @ pin.utils.rpyToMatrix(np.array([0.0, 0.0, -np.pi/2.0]))
    #             # self.final_r_hand_pose_for_trajectory.translation += r_hand.rotation @ np.array([0.16, -0.15, 0.15])
                
    #             # For now, use simple direct trajectory
    #             self.waypoints = create_waypoint_trajectory(
    #                 self.initial_r_hand_pose_for_trajectory,
    #                 self.final_r_hand_pose_for_trajectory,
    #                 intermediate_waypoints=[self.mid_r_hand_pose_for_trajectory]
    #             )
    #             # self.trajectory_duration = 3.0 * len(self.waypoints)
    #             self.trajectory_duration = self.hold_door_right_contact_switch
                
    #             rospy.loginfo(f"Starting waypoint trajectory with {len(self.waypoints)} waypoints")
    #             for i, wp in enumerate(self.waypoints):
    #                 rospy.loginfo(f"Waypoint {i}: {wp.translation}")
            
    #         # Calculate trajectory progress
    #         elapsed_time = (rospy.Time.now() - self.trajectory_start_time).to_nsec() / 1e9
            
    #         # Generate smooth trajectory through waypoints
    #         current_target_pose = generate_waypoint_trajectory(
    #             self.waypoints,
    #             self.trajectory_duration,
    #             elapsed_time
    #         )
            
    #         self.hold_door_right_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                 pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                 self.rm_state.state[0:3],
    #                                                 None, None,
    #                                                 current_target_pose.rotation, 
    #                                                 current_target_pose.translation)
    #         self.r_gripper.open()
    #         base_cmd = np.array([0.0, 0.0])

    #     if 100 * (self.hold_door_right_release_gripper_time + self.hold_door_right_backward_arm_time \
    #         + self.hold_door_right_contact_switch) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #         < 100 * (self.hold_door_right_release_gripper_time + self.hold_door_right_backward_arm_time \
    #             + self.hold_door_right_contact_switch + self.hold_door_right_put_down_left_arm) * 10_000_000:
    #         if self.trajectory_start_time_homing is None:
    #             self.trajectory_start_time_homing = rospy.Time.now()
    #             self.init_current_state = np.array(self.rm_state.state[7:13])

    #         elapsed_time = (rospy.Time.now() - self.trajectory_start_time_homing).to_nsec() / 1e9
    #         phase = elapsed_time / self.hold_door_right_put_down_left_arm            
    #         # Convert lists to numpy arrays for arithmetic operations
            
    #         l_arm_homing = np.array(self.config.INIT_ARM[0:6])
    #         result = (1 - phase) * self.init_current_state + phase * l_arm_homing
    #         # Convert back to list for assignment
    #         self.hold_door_right_jcmd[0:6] = result.tolist()

    #     self.sendRosCommand(self.hold_door_right_jcmd, base_cmd)

    #     if (rospy.Time.now() - self.state_start_time).to_nsec() > 100 * (self.hold_door_right_release_gripper_time \
    #         + self.hold_door_right_backward_arm_time \
    #         + self.hold_door_right_contact_switch + self.hold_door_right_put_down_left_arm) * 10_000_000:
    #         self.first_entry = True
    #         self._transition_to(RobotState.PUSH_DOOR)

    # def _handle_push_door(self):
    #     if self.first_entry == True:
    #         self.first_entry = False
    #         forward_speed = 0.2
    #         if forward_speed == 0.0:
    #             self.push_travel_time = 0.0
    #         else:
    #             self.push_travel_time = 2.0 / abs(forward_speed)
    #         self.push_swing_time = 5.0
    #         # swing_time = 1
    #         self.push_base_cmd = np.array([forward_speed, 0.0])
    #         self.state_start_time = rospy.Time.now()
    #         self.rm_controller.update_pink_ik_configuration(self.rm_state.state)

    #         # Initialize trajectory parameters
    #         self.trajectory_start_time = None
            

    #     if self.push_base_cmd is not None:
    #         # Swing the arm to the right to open the door
    #         if (rospy.Time.now() - self.state_start_time).to_nsec() < 100 * self.push_swing_time * 10_000_000:
    #             if self.push_swing_pose is None:
    #                 base_world_rot = pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix()
    #                 base_pose_world = pin.SE3(base_world_rot, self.rm_state.state[0:3])
    #                 r_hand = self.rm_controller.compute_frame_pose(self.rm_state.state, 
    #                     self.config.PIN_GIRPPER_FRAME_NAME[1]).copy()
    #                 r_hand_local = self.rm_controller.compute_frame_pose(self.rm_state.state, 
    #                     self.config.PIN_GIRPPER_FRAME_NAME[1], world_frame=False).copy()
    #                 r_hand_local.translation += np.array([-0.4, 0.1, 0.0])
    #                 self.push_swing_pose = base_pose_world.act(r_hand_local)
    #                 self.push_swing_pose.rotation = r_hand.rotation @ pin.utils.rpyToMatrix(np.array([np.pi/4.0, 0.0, 0.0]))
    #                 self.waypoints = create_waypoint_trajectory(
    #                     r_hand,
    #                     self.push_swing_pose
    #                 )
    #                 self.trajectory_duration = self.push_swing_time
    #                 self.trajectory_start_time = rospy.Time.now()

    #             elapsed_time = (rospy.Time.now() - self.trajectory_start_time).to_nsec() / 1e9
    #             current_target_pose = generate_waypoint_trajectory(
    #                 self.waypoints,
    #                 self.trajectory_duration,
    #                 elapsed_time
    #             )
    #             self.push_swing_jcmd = self.rm_controller.pink_ik_incremental(self.rm_state.state,
    #                                                         pin.Quaternion(self.rm_state.state[3:7]).normalized().toRotationMatrix(), 
    #                                                         self.rm_state.state[0:3],
    #                                                         None, None,
    #                                                         current_target_pose.rotation, 
    #                                                         current_target_pose.translation)
    #             self.r_gripper.open()
    #             self.push_base_cmd = np.array([0.0, 0.0])
    #             self.sendRosCommand(self.push_swing_jcmd)
    #             # self._transition_to(RobotState.HOLD_DOOR_LEFT)
            
    #         if 100 * (self.push_swing_time) * 10_000_000 <= (rospy.Time.now() - self.state_start_time).to_nsec() \
    #             < 100 * (self.push_swing_time + self.push_travel_time) * 10_000_000:
    #             self.push_swing_jcmd[0:12] = self.config.INIT_ARM
    #             self.push_base_cmd = np.array([0.2, 0.0])
    #             self.sendRosCommand(self.push_swing_jcmd)

    #         if (rospy.Time.now() - self.state_start_time).to_nsec() > 100 * (self.push_swing_time + self.push_travel_time) * 10_000_000:
    #             self.push_base_cmd = np.array([0.0, 0.0])
    #             # self._transition_to(RobotState.HOLD_DOOR_LEFT)
            
    #         self.sendRosCommand(base_command=self.push_base_cmd)


    # def _handle_turn(self):
    #     if self.turn_jcmd is None:
    #         self.turn_jcmd = self.rm_controller.find_arm_inverse_kinematics(
    #             self.rm_state.state,
    #             self.door_handle_pose[:3] + self.config.HANDEL_TURN_OFFSET,
    #             self.config.HANDEL_TURN_ROTATION,
    #             arm_idx=0
    #         )
    #         self.grasp_count = 0
    #         # self.l_gripper.close()
    #         rospy.loginfo("Computed turn IK once")
    #     self.sendRosCommand(self.turn_jcmd)
    #     if (rospy.Time.now() - self.state_start_time).to_nsec() > 300 * 10_000_000:
    #         # 3 seconds
    #         self._transition_to(RobotState.OPENING)
    
    # def _handle_opening(self):
    #     if self.open_jcmd is None:
    #         self.open_jcmd = self.rm_controller.find_arm_inverse_kinematics(
    #             self.rm_state.state,
    #             self.door_handle_pose[:3] + self.config.RIGHRT_ARM_PUSH_POSITION,
    #             np.eye(3),
    #             arm_idx=1
    #         )
    #     base_command = np.array([1.0, 0.0])
    #     self.sendRosCommand(base_command=base_command)
    #     rospy.loginfo("Opening door")
        
    #     if (rospy.Time.now() - self.state_start_time).to_nsec() > 10 * 10_000_000:
    #         self.r_gripper.semi_open()

    #     self.sendRosCommand(self.open_jcmd)


    def _transition_to(self, new_state: RobotState):
        rospy.loginfo(f"→ Transition: {self.state.name} → {new_state.name}")
        self.state = new_state
        self.state_start_time = rospy.Time.now()

    def sendRosCommand(self, joint_command = None, base_command = None):
        if joint_command is not None:
            l_move_j = MoveJ()
            r_move_j = MoveJ()
            l_move_j.joint = joint_command[10:16]
            r_move_j.joint = joint_command[16:22]
            l_move_j.speed = 0.3
            r_move_j.speed = 0.3
            l_move_j.trajectory_connect = 0
            r_move_j.trajectory_connect = 0
            self.pub_MoveJ_l.publish(l_move_j)
            self.pub_MoveJ_r.publish(r_move_j)
            lift_msg = Lift_Height()
            # lift_msg.height = int(776 - joint_command[7] * 1000)
            lift_msg.height = int(self.convert_lift_state(joint_command[7], real2urdf=False))
            # print(f"lift_msg.height: {lift_msg.height}")

            lift_msg.speed = int(50)
            self.lift_publisher.publish(lift_msg)
        if base_command is not None:
            velocity_msgs = Twist()
            velocity_msgs.linear.x   = base_command[0]
            velocity_msgs.angular.z  = base_command[1]
            self.base_pub.publish(velocity_msgs)               

def main():
    node = RealmanControlNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()