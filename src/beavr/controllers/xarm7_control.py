#import rospy
import numpy as np
import time
from copy import deepcopy as copy
from xarm import XArmAPI
from enum import Enum
import math

from beavr.constants import XARM_SCALE_FACTOR
from scipy.spatial.transform import Rotation as R
from beavr.constants import *

class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1

# Wrapper for XArm
class Robot(XArmAPI):
    def __init__(self, ip="192.168.1.197", is_radian=True, simulation_mode=False):
        super(Robot, self).__init__(
            port=ip, is_radian=is_radian, is_tool_coord=False, enable_report=True, report_type="rich")
        if simulation_mode and not self.is_simulation_robot:
            self.set_simulation_robot(on_off=True)
        self.ip = ip

    def clear(self):
        self.clean_error()
        self.clean_warn()
        # self.motion_enable(enable=False)
        self.motion_enable(enable=True)

    def set_mode_and_state(self, mode: RobotControlMode, state: int = 0):
        self.set_mode(mode.value)
        self.set_state(state)

    def reset(self):
        # Clean error
        self.clear()
        print("Slow reset...")
        self.set_mode_and_state(RobotControlMode.CARTESIAN_CONTROL, 0)
        status = self.set_servo_angle(angle=ROBOT_HOME_JS, wait=True, is_radian=True, speed=math.radians(50))
        assert status == 0, "Failed to set robot at home joint position"
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        time.sleep(0.1)



class DexArmControl():
    def __init__(self,ip):
        self.robot = Robot(ip, is_radian=True, simulation_mode=False) 
        self.robot.reset()

    # Controller initializers
    def _init_xarm_control(self):
       
        self.robot.reset()
        
        status, home_pose = self.robot.get_position_aa()
        assert status == 0, "Failed to get robot position"
        home_affine = self.robot_pose_aa_to_affine(home_pose)
        # Initialize timestamp; used to send messages to the robot at a fixed frequency.
        last_sent_msg_ts = time.time()

        # Initialize the environment state action tuple.
   
    def get_arm_pose(self):
        status, home_pose = self.robot.get_position_aa()
        home_affine = self.robot_pose_aa_to_affine(home_pose)
        return home_affine

    def get_arm_position(self):
        code,joint_state = self.robot.get_servo_angle()
        if code != 0:
            raise RuntimeError(f"Failed to get joint states, error code: {code}")
        joint_state = np.array(joint_state[1], dtype=np.float32)
        return joint_state

    def get_arm_velocity(self):
        """
        Returns the velocity of the joints
        
        Returns:
            numpy.ndarray: Array of joint velocities [joint1_velocity, ..., joint7_velocity] in radians/second
            or None if failed to get joint states
        """
        status, states = self.robot.get_joint_states()
        if status != 0:
            print('\033[93m' + f"Warning: Failed to get joint states, error code: {status}" + '\033[0m')
            return None
        
        velocities = np.array(states[1], dtype=np.float32)
        return velocities

    def get_arm_torque(self):
        """ 
        Returns the torque of the joints
        
        Returns:
            numpy.ndarray: Array of joint torques [joint1_torque, ..., joint7_torque] in Nm
            or None if failed to get joint torques
        """
        status, torques = self.robot.get_joints_torque()
        if status != 0:
            print('\033[93m' + f"Warning: Failed to get joint torques, error code: {status}" + '\033[0m')
            return None
        
        torques = np.array(torques, dtype=np.float32)
        return torques
    
    def get_arm_cartesian_coords(self):
        """
            Returns the cartesian coordinates of the arm
            Output Format:
            [x, y, z] (mm), [roll, pitch, yaw] (radians/degrees)
        """
        status, home_pose = self.robot.get_position_aa()
        return home_pose

    def move_arm_joint(self, joint_angles):
        """
        Move robot to specified joint angles
        
        Args:
            joint_angles (list/ndarray): Target joint angles in radians
            
        Returns:
            int: Status code (0 for success)
        """
        status = self.robot.set_servo_angle(
            angle=joint_angles, 
            wait=True, 
            is_radian=True, 
            mvacc=80, 
            speed=10
        )
        if status != 0:
            print('\033[93m' + f"Warning: Failed to move robot, error code: {status}" + '\033[0m')
        return status

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        """
            Input: cartesian_pos: list of [x, y, z] (mm), [roll, pitch, yaw] (radians)
            Output: None
        """
        if self.robot.mode != 1 or self.robot.state != 0:
            self.robot.set_mode(1)
            self.robot.set_state(0)
        cartesian_pos[0:3] = np.array(cartesian_pos[0:3]) * XARM_SCALE_FACTOR
        self.robot.set_servo_cartesian_aa(
                    cartesian_pos, wait=False, relative=False, mvacc=5, speed=1, is_radian=True)

    def arm_control(self, cartesian_pose):
        if self.robot.has_error:
            self.robot.clear()
            self.robot.set_mode_and_state(1)
        self.robot.set_servo_cartesian_aa(
                    cartesian_pose, wait=False, relative=False, mvacc=5, speed=1)
        
    def get_arm_joint_state(self):
        joint_positions = np.array(self.robot.get_servo_angle()[1])
        joint_state = dict(
            position = np.array(joint_positions, dtype=np.float32),
            timestamp = time.time()
        )
        return joint_state
        
    def get_cartesian_state(self):
        status,current_pos=self.robot.get_position_aa()
        cartesian_state = dict(
            position = np.array(current_pos[0:3], dtype=np.float32).flatten(),
            orientation = np.array(current_pos[3:], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return cartesian_state

    def home_arm(self):
        """
        Move the arm to home position using joint angles
        """
        try:
            # Convert ROBOT_HOME_JS to numpy array
            home_joints = np.array(ROBOT_HOME_JS, dtype=np.float32)
            
            if self.robot.mode != 0:
                # Set mode to position control
                self.robot.set_mode(0)
                self.robot.set_state(0)
            
            # Move to home position
            status = self.robot.set_servo_angle(angle=home_joints, wait=True, is_radian=True, mvacc=5, speed=1)
            return status
        except Exception as e:
            print(f"Error in home_arm: {e}")
            return -1

    def home_robot(self):
        """
        Home the robot to the home position defined as a constant in constants.py
        """
        try:
            status = self.home_arm()
            if status != 0:
                print(f"Warning: home_arm returned status {status}")
            return status
        except Exception as e:
            print(f"Error in home_robot: {e}")
            return -1

    def reset_arm(self):
        self.home_arm()

    # Full robot commands
    def move_robot(self, arm_angles):
        """
        Move robot to specified joint angles. This is an alias for move_arm_joint()
        for backward compatibility.
        
        Args:
            arm_angles (list/ndarray): Target joint angles in radians
            
        Returns:
            int: Status code (0 for success)
        """
        # For consistency, use the existing move_arm_joint method
        return self.move_arm_joint(arm_angles)

    def robot_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3]) / XARM_SCALE_FACTOR

        return np.block([[rotation, translation[:, np.newaxis]],
                        [0, 0, 0, 1]])