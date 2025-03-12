import numpy as np
import zmq
from copy import deepcopy as copy

from openteach.constants import *
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from openteach.utils.vectorops import *
from scipy.spatial.transform import Rotation, Slerp
from .operator import Operator

# Filter for smoothing movements
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        # Position filtering
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        # Orientation filtering using SLERP
        ori_interp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]], axis=0)))
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])

class XArmPyBulletOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        finger_configs,
        stream_configs,
        stream_oculus,
        jointanglepublishport,
        jointanglesubscribeport,
        endeff_publish_port,
        endeffpossubscribeport,
        moving_average_limit,
        allow_rotation=False,
        arm_type='main_arm',
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        control_mode='relative',
    ):
        """Initialize XArm PyBullet Operator.
        
        Args:
            host: Network host
            transformed_keypoints_port: Port for transformed keypoints
            stream_configs: Configuration for VR streaming
            stream_oculus: Whether to stream to Oculus
            endeff_publish_port: Port for end effector publishing
            endeffpossubscribeport: Port for end effector subscription
            robotposesubscribeport: Port for robot pose subscription
            moving_average_limit: Limit for moving average filter
            arm_resolution_port: Port for arm resolution (optional)
            control_mode: Either 'relative' or 'absolute' for position control
        """
        self.notify_component_start('xarm pybullet operator')
        
        # Store parameters
        self._host = host
        self._port = transformed_keypoints_port
        self._stream_oculus = stream_oculus
        self.stream_configs = stream_configs
        self.control_mode = control_mode
        
        # Hand keypoint subscribers
        self._hand_transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host=self._host,
            port=self._port,
            topic='transformed_hand_coords'
        )
        self._arm_transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )

        # Robot state subscribers/publishers
        self.end_eff_position_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=endeffpossubscribeport,
            topic='endeff_coords'
        )
        self.end_eff_position_publisher = ZMQKeypointPublisher(
            host=host,
            port=endeff_publish_port
        )
        self.robot_pose_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=robotposesubscribeport,
            topic='robot_pose'
        )

        # State initialization
        self.arm_teleop_state = ARM_TELEOP_CONT  # Start in CONT state like LiberoSim
        self.gripper_state = 0
        self.pause_flag = 0
        self.gripper_flag = 1
        self.prev_gripper_flag = 0
        self.prev_pause_flag = 0
        self.pause_cnt = 0
        self.gripper_cnt = 0

        # Resolution control subscriber
        if arm_resolution_port:
            self._arm_resolution_subscriber = ZMQKeypointSubscriber(
                host=host,
                port=arm_resolution_port,
                topic='button'
            )

        # Timer and filtering setup
        self._timer = FrequencyTimer(VR_FREQ)
        self._robot = 'xArm_PyBullet'
        self.is_first_frame = True
        self.moving_Average_queue = []
        self.moving_average_limit = moving_average_limit
        self.hand_frames = []

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_arm_keypoint_subscriber(self):
        """Required by ArmOperator - returns arm keypoint subscriber"""
        return self._arm_transformed_keypoint_subscriber

    @property
    def transformed_hand_keypoint_subscriber(self):
        """Required by ArmOperator - returns hand keypoint subscriber"""
        return self._hand_transformed_keypoint_subscriber

    def _get_hand_frame(self):
        """Get hand frame from keypoint subscriber."""
        for i in range(10):
            data = self._arm_transformed_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(4, 3)

    def _turn_frame_to_homo_mat(self, frame):
        """Convert frame to homogeneous transformation matrix."""
        t = frame[0]
        R = frame[1:]
        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1
        return homo_mat

    def _homo2cart(self, homo_mat):
        """Convert homogeneous matrix to cartesian coordinates."""
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(homo_mat[:3, :3]).as_quat()
        return np.concatenate([t, R], axis=0)

    def cart2homo(self, cart):
        """Convert cartesian coordinates to homogeneous matrix."""
        homo = np.zeros((4, 4))
        t = cart[0:3]
        R = Rotation.from_quat(cart[3:]).as_matrix()
        homo[0:3, 3] = t
        homo[:3, :3] = R
        homo[3, :] = np.array([0, 0, 0, 1])
        return homo

    def _get_arm_teleop_state_from_hand_keypoints(self):
        """Get teleop state from hand keypoints."""
        pause_state, pause_status, pause_right = self.get_pause_state_from_hand_keypoints()
        pause_status = np.asanyarray(pause_status).reshape(1)[0]
        # print(f"\nTeleop Debug - State: {pause_state}, Status: {pause_status}, Right: {pause_right}")  # Debug print
        return pause_state, pause_status, pause_right

    def get_pause_state_from_hand_keypoints(self):
        """Get pause state from hand keypoints."""
        transformed_hand_coords = self._hand_transformed_keypoint_subscriber.recv_keypoints()
        if transformed_hand_coords is None:
            return self.arm_teleop_state, False, False
        
        ring_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['ring'][-1]] - transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        middle_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['middle'][-1]] - transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
        thresh = 0.04 
        pause_right = True
        
        if ring_distance < thresh or middle_distance < thresh:
            self.pause_cnt += 1
            if self.pause_cnt == 1:
                self.prev_pause_flag = self.pause_flag
                self.pause_flag = not self.pause_flag
        else:
            self.pause_cnt = 0
        
        pause_state = np.asanyarray(self.pause_flag).reshape(1)[0]
        pause_status = False
        if pause_state != self.prev_pause_flag:
            pause_status = True
        return pause_state, pause_status, pause_right

    def get_gripper_state_from_hand_keypoints(self):
        """Get gripper state from hand keypoints."""
        transformed_hand_coords = self._hand_transformed_keypoint_subscriber.recv_keypoints()
        pinky_distance = np.linalg.norm(
            transformed_hand_coords[OCULUS_JOINTS['pinky'][-1]] - 
            transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]]
        )
        
        thresh = 0.03
        if pinky_distance < thresh:
            self.gripper_cnt += 1
            if self.gripper_cnt == 1:
                self.prev_gripper_flag = self.gripper_flag
                self.gripper_flag = not self.gripper_flag
        else:
            self.gripper_cnt = 0

        gripper_state = np.asanyarray(self.gripper_flag).reshape(1)[0]
        status = False
        if gripper_state != self.prev_gripper_flag:
            status = True
        return gripper_state, status

    def _reset_teleop(self):
        """Reset teleoperation state."""
        print('****** RESETTING TELEOP ****** ')
        self.robot_frame = self.end_eff_position_subscriber.recv_keypoints()
        if self.robot_frame is None:
            print("Warning: No robot frame received")
            return None
        
        # Expect full pose: [x, y, z, qx, qy, qz, qw]
        self.robot_init_H = self.cart2homo(self.robot_frame)

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])

        self.is_first_frame = False
        return first_hand_frame

    def _apply_retargeted_angles(self, log=False):
        """Apply retargeted angles from hand motion to robot."""
        # Check teleop state
        new_arm_teleop_state, pause_status, pause_right = self._get_arm_teleop_state_from_hand_keypoints()
        
        # Update state before checking it
        old_state = self.arm_teleop_state
        self.arm_teleop_state = new_arm_teleop_state
        
        if self.is_first_frame or (old_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            moving_hand_frame = self._reset_teleop()
        else:
            moving_hand_frame = self._get_hand_frame()

        # Get gripper state
        gripper_state, status_change = self.get_gripper_state_from_hand_keypoints()
        if status_change:
            if gripper_state == GRIPPER_OPEN:
                gripper_command = -1
            else:
                gripper_command = 1
        else:
            gripper_command = 0

        if moving_hand_frame is None:
            return

        # Get current robot state
        robot_state = self.robot_pose_subscriber.recv_keypoints()
        if robot_state is not None:
            self.robot_frame = robot_state
            self.robot_moving_H = self.cart2homo(self.robot_frame)
        
        # Transform hand motion to robot commands using LiberoSim's transformations
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)
        H_HI_HH = copy(self.hand_init_H)
        H_HT_HH = copy(self.hand_moving_H)
        H_RI_RH = copy(self.robot_init_H)
        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH

        # Apply coordinate transformations (same as LiberoSim)
        H_R_V = np.array([[0, 0, 1, 0],
                          [0, 1, 0, 0],
                          [-1, 0, 0, 0],
                          [0, 0, 0, 1]])
        H_T_V = np.array([[0, 0, 1, 0],
                          [0, 1, 0, 0],
                          [-1, 0, 0, 0],
                          [0, 0, 0, 1]])

        H_HT_HI_r = (np.linalg.pinv(H_R_V) @ H_HT_HI @ H_R_V)[:3, :3]
        H_HT_HI_t = (np.linalg.pinv(H_T_V) @ H_HT_HI @ H_T_V)[:3, 3]

        relative_affine = np.block([[H_HT_HI_r, H_HT_HI_t.reshape(3, 1)],
                                   [0, 0, 0, 1]])

        # Calculate target pose
        target_translation = H_RI_RH[:3, 3] + relative_affine[:3, 3]
        target_rotation = H_RI_RH[:3, :3] @ relative_affine[:3, :3]
        H_RT_RH = np.block([[target_rotation, target_translation.reshape(-1, 1)],
                            [0, 0, 0, 1]])

        # Get current robot pose
        curr_robot_pose = self.robot_init_H  # Using initial pose as current
        
        # Calculate relative motion (similar to LiberoSim)
        translation_scale = 50.0
        T_curr = curr_robot_pose[:3, 3]
        R_curr = curr_robot_pose[:3, :3]
        
        T_togo = H_RT_RH[:3, 3]
        R_togo = H_RT_RH[:3, :3]
        
        # Calculate relative position and rotation
        rel_pos = (T_togo - T_curr) * translation_scale
        rel_rot = np.linalg.pinv(R_curr) @ R_togo
        rel_axis_angle = Rotation.from_matrix(rel_rot).as_rotvec()
        rel_axis_angle = rel_axis_angle * 5.0  # Scale rotation like LiberoSim
        
        # Create relative motion command
        action = np.concatenate([
            rel_pos,           # Relative position
            rel_axis_angle,    # Relative rotation
            [gripper_command]  # Gripper command
        ])

        # Apply moving average filter
        averaged_action = moving_average(
            action,
            self.moving_Average_queue,
            self.moving_average_limit,
        )

        # Publish command based on teleop state
        if self.arm_teleop_state == ARM_TELEOP_CONT:
            self.end_eff_position_publisher.pub_keypoints(averaged_action, "endeff_coords")
        else:
            zero_action = np.concatenate([np.zeros(6), [gripper_command]])
            self.end_eff_position_publisher.pub_keypoints(zero_action, "endeff_coords")

    def return_real(self):
        """Return whether this is a real robot."""
        return False 