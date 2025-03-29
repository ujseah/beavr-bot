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
        self._last_mode_check = 0
        self._mode_check_interval = 0.05  # 50ms check interval, no blocking
        self._max_step_distance = 50.0  # Maximum allowed movement in mm (1 cm = 10 mm)
        self._last_position = None  # To track last commanded position

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

    def _ensure_correct_mode(self, desired_mode, desired_state=0):
        """Ensure robot is in the correct mode for the operation without blocking"""
        current_time = time.time()
        
        # Only perform actual mode changes when needed
        if self.robot.mode != desired_mode or self.robot.state != desired_state:
            # If we're switching modes, we need to make sure it happens
            self.robot.set_mode(desired_mode)
            self.robot.set_state(desired_state)
            self._last_mode_check = current_time
            return True
            
        # Periodically check for errors even if mode seems correct
        if current_time - self._last_mode_check > self._mode_check_interval:
            self._last_mode_check = current_time
            if self.robot.has_error:
                self.robot.clear()
                self.robot.set_mode(desired_mode)
                self.robot.set_state(desired_state)
                
        return False
        
    def _exceeds_movement_limit(self, target_pose):
        """
        Check if proposed movement exceeds the maximum allowed distance in a single step.
        If it does, split it into smaller movements.
        
        Args:
            target_pose: Target cartesian pose [x, y, z, roll, pitch, yaw]
            
        Returns:
            tuple: (exceeds_limit, waypoints)
                exceeds_limit (bool): True if movement exceeds limit
                waypoints (list): List of intermediate waypoints if exceeds_limit is True, 
                                 otherwise empty list
        """
        # Always refresh current position to avoid drift issues
        status, current_pose = self.robot.get_position_aa()
        if status != 0:
            # If we can't get current position, play it safe and allow the movement
            print("Warning: Couldn't get current position, allowing movement")
            return False, []
            
        # Use actual robot position as reference, not last commanded position
        current_position = np.array(current_pose[0:3])
        
        # Calculate distance (only for position xyz, not orientation)
        target_position = np.array(target_pose[0:3])
        distance = np.linalg.norm(target_position - current_position)
        
        # Check if distance exceeds limit
        if distance > self._max_step_distance:
            # Generate intermediate waypoints
            waypoints = self._split_large_movement(current_pose, target_pose)
            print(f"\033[93mMovement distance {distance:.2f}mm exceeds {self._max_step_distance}mm limit. "
                  f"Split into {len(waypoints)} segments.\033[0m")
            return True, waypoints
            
        return False, []
        
    def _split_large_movement(self, current_pose, target_pose):
        """
        Split large movements into smaller segments that stay within movement limits
        
        Args:
            current_pose: Current cartesian pose [x, y, z, roll, pitch, yaw]
            target_pose: Target cartesian pose [x, y, z, roll, pitch, yaw]
            
        Returns:
            list: List of intermediate waypoints
        """
        distance = np.linalg.norm(np.array(target_pose[0:3]) - np.array(current_pose[0:3]))
        segments = max(2, int(np.ceil(distance / (self._max_step_distance * 0.8))))
        waypoints = []
        
        current_pos = np.array(current_pose[0:3])
        target_pos = np.array(target_pose[0:3])
        
        current_ori = np.array(current_pose[3:6])
        target_ori = np.array(target_pose[3:6])
        
        for i in range(1, segments + 1):
            t = i / segments
            # Linear interpolation for position
            pos = current_pos * (1-t) + target_pos * t
            # Linear interpolation for orientation (simple approach)
            ori = current_ori * (1-t) + target_ori * t
            
            waypoints.append(np.concatenate([pos, ori]).tolist())
        
        return waypoints

    def arm_control(self, cartesian_pose):
        # Check if movement exceeds safety limit
        exceeds_limit, waypoints = self._exceeds_movement_limit(cartesian_pose)
        
        if exceeds_limit:
            # Execute each waypoint in sequence
            for waypoint in waypoints:
                # Use servo mode (1) for real-time teleoperation
                self._ensure_correct_mode(1, 0)
                self.robot.set_servo_cartesian_aa(
                    waypoint, wait=True, relative=False, mvacc=5, speed=1)
        else:
            # Execute single movement as before
            self._ensure_correct_mode(1, 0)
            self.robot.set_servo_cartesian_aa(
                cartesian_pose, wait=False, relative=False, mvacc=5, speed=1)
        
    def move_arm_cartesian(self, cartesian_pos, duration=3):
        """
        Modified version with improved handling of pitch/yaw rotations
        """
        # Scale position
        scaled_pos = copy(cartesian_pos)
        scaled_pos[0:3] = np.array(cartesian_pos[0:3]) * XARM_SCALE_FACTOR
        
        # Check if movement exceeds safety limit
        exceeds_limit, waypoints = self._exceeds_movement_limit(scaled_pos)
        
        if exceeds_limit:
            # Execute each waypoint in sequence, breaking down rotations
            for waypoint in waypoints:
                # First move position only, keeping current orientation
                pos_only = waypoint.copy()
                curr_ori = self.get_arm_cartesian_coords()[3:6]
                pos_only[3:6] = curr_ori
                
                self._ensure_correct_mode(1, 0)
                self.robot.set_servo_cartesian_aa(
                    pos_only, wait=True, relative=False, mvacc=5, speed=1, is_radian=True)
                
                # Then adjust orientation separately - often more stable
                self._ensure_correct_mode(1, 0)
                self.robot.set_servo_cartesian_aa(
                    waypoint, wait=True, relative=False, mvacc=3, speed=0.5, is_radian=True)
        else:
            # Use servo mode for cartesian movement
            self._ensure_correct_mode(1, 0)
            cartesian_pos[0:3] = np.array(cartesian_pos[0:3]) * XARM_SCALE_FACTOR
            
            # For small movements, we can still break into position then orientation
            # First move position only
            pos_only = cartesian_pos.copy()
            curr_ori = self.get_arm_cartesian_coords()[3:6]
            pos_only[3:6] = curr_ori
            
            self.robot.set_servo_cartesian_aa(
                pos_only, wait=True, relative=False, mvacc=5, speed=1, is_radian=True)
            
            # Then adjust orientation
            self.robot.set_servo_cartesian_aa(
                cartesian_pos, wait=True, relative=False, mvacc=3, speed=0.5, is_radian=True)
    
    def home_arm(self):
        """
        Move the arm to home position using joint angles
        """
        try:
            # Convert ROBOT_HOME_JS to numpy array
            home_joints = np.array(ROBOT_HOME_JS, dtype=np.float32)
            
            # Use position control mode (0) for homing
            self._ensure_correct_mode(0, 0)
            
            # Move to home position
            status = self.robot.set_servo_angle(angle=home_joints, wait=True, 
                                              is_radian=True, mvacc=5, speed=1)
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