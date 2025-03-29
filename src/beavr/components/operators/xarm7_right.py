import numpy as np
import zmq
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation, Slerp
import time
from typing import Tuple

from beavr.constants import *
from beavr.utils.timer import FrequencyTimer
from beavr.utils.network import ZMQKeypointSubscriber
from beavr.utils.network import EnhancedZMQKeypointPublisher as ZMQKeypointPublisher
from beavr.utils.vectorops import *
from .operator import Operator

from beavr.utils.logger import PoseLogger

# Simple complementary filter with SLERP for orientation (same as Allegro)
class CompStateFilter:
    def __init__(self, init_state, pos_ratio=0.6, ori_ratio=0.8, adaptive=True):
        self.pos_state = init_state[:3]
        self.ori_state = init_state[3:7]
        self.pos_ratio = pos_ratio
        self.ori_ratio = ori_ratio  # Stronger filtering for orientation
        self.adaptive = adaptive
        self.prev_pos = init_state[:3]
        self.velocity = np.zeros(3)
        self.prev_quat = init_state[3:7]

    def __call__(self, next_state):
        # Calculate velocity
        current_pos = next_state[:3]
        self.velocity = current_pos - self.prev_pos
        speed = np.linalg.norm(self.velocity)
        
        # Adaptive filtering for position
        actual_pos_ratio = self.pos_ratio
        if self.adaptive:
            # Increase filtering when movement is small (reduce jitter)
            # Decrease filtering when movement is large (reduce lag)
            threshold = 0.005  # Adjust based on your units
            if speed < threshold:
                actual_pos_ratio = min(0.95, self.pos_ratio + 0.1)  # More filtering for small movements
            else:
                actual_pos_ratio = max(0.7, self.pos_ratio - 0.1 * (speed/threshold))  # Less filtering for large movements
        
        # Apply position filtering
        self.pos_state = self.pos_state * actual_pos_ratio + next_state[:3] * (1 - actual_pos_ratio)
        
        # Always use stronger filtering for orientation (less affected by adaptive)
        # and always use actual ratio slightly higher than position
        actual_ori_ratio = max(actual_pos_ratio, self.ori_ratio)
        
        # Apply orientation filtering
        ori_interp = Slerp([0, 1], Rotation.from_quat([self.ori_state, next_state[3:7]]))
        self.ori_state = ori_interp([1 - actual_ori_ratio])[0].as_quat()
        
        self.prev_pos = current_pos
        self.prev_quat = next_state[3:7]
        
        return np.concatenate([self.pos_state, self.ori_state])

# Add a new class for quaternion-based orientation filtering
class QuaternionFilter:
    """Handles orientation filtering using pure quaternion operations"""
    def __init__(self, init_quat, smoothing=0.85):
        self.current_quat = np.array(init_quat)
        self.smoothing = smoothing
        self.last_update_time = time.time()
        # Angular velocity in axis-angle format (axis * angle)
        self.angular_velocity = np.zeros(3)
        
    def update(self, new_quat, timestamp=None):
        """Apply filtering to new quaternion input"""
        # Ensure quaternions are in the same hemisphere to avoid discontinuities
        if np.dot(self.current_quat, new_quat) < 0:
            new_quat = -new_quat
            
        # Get current time or use provided timestamp
        now = timestamp if timestamp is not None else time.time()
        dt = now - self.last_update_time
        dt = min(0.1, max(0.001, dt))  # Reasonable bounds on dt
        
        # Convert to rotations
        current_rotation = Rotation.from_quat(self.current_quat)
        target_rotation = Rotation.from_quat(new_quat)
        
        # Find the relative rotation from current to target
        relative_rotation = current_rotation.inv() * target_rotation
        
        # Get the angular velocity implied by this change
        rotvec = relative_rotation.as_rotvec()
        instant_velocity = rotvec / dt
        
        # Smooth the angular velocity using exponential filter
        self.angular_velocity = self.smoothing * self.angular_velocity + (1 - self.smoothing) * instant_velocity
        
        # Apply a portion of the smoothed velocity to get new orientation
        delta_rotation = Rotation.from_rotvec(self.angular_velocity * dt)
        new_rotation = current_rotation * delta_rotation
        
        # Update state
        self.current_quat = new_rotation.as_quat()
        self.last_update_time = now
        
        return self.current_quat

class XArm7RightOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        stream_configs,
        stream_oculus,
        endeff_publish_port,
        endeff_subscribe_port,
        moving_average_limit,
        use_filter=True,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        logging_config=None,
    ):
        # Basic initialization
        self.notify_component_start('xarm7 right_operator')
        self._host, self._port = host, transformed_keypoints_port

        # Add finger coordinate subscriber (like Allegro)
        self._hand_transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host,
            port = self._port,
            topic = 'transformed_hand_coords'
        )
        
        self._arm_transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )

        # Optional subscribers (same as Allegro)
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = arm_resolution_port,
            topic = 'button'
        )

        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = teleoperation_reset_port,
            topic = 'pause'
        )

        self.endeff_homo_subscriber = ZMQKeypointSubscriber(
            host = host,
            port =  endeff_subscribe_port,
            topic = 'endeff_homo'
        )

        # ONE publisher for all messages
        self.unified_publisher = ZMQKeypointPublisher(
            host = host,
            port = endeff_publish_port
        )

        self._stream_oculus=stream_oculus
        self.stream_configs=stream_configs

        # State initialization
        self.arm_teleop_state = ARM_TELEOP_STOP
        self.resolution_scale = 1.0
        self.is_first_frame = True
        self._timer = FrequencyTimer(VR_FREQ)
        self._robot = None
        self.real = False

        # Transformation matrices
        self.robot_init_H = None
        self.robot_moving_H = None
        self.hand_init_H = None
        self.hand_moving_H = None
        self.hand_init_t = None

        # Filter setup
        self.use_filter = use_filter
        self.comp_filter = None
        self.quat_filter = None

        # Moving average setup
        self.moving_average_queue = []
        self.moving_average_limit = moving_average_limit
        self.hand_frames = []

        # Separate moving average limits for position and orientation
        self.orientation_average_limit = min(10, moving_average_limit * 2)  # Use more samples for orientation
        self.orientation_queue = []  # Separate queue for orientation samples
        
        # Track previous orientations for stability detection
        self.prev_orientation = None
        self.last_sent_orientation = None
        self.ori_update_counter = 0

        self._stream_oculus = stream_oculus
        self.stream_configs = stream_configs

        # Initialize pose logger based on config
        self.logging_config = logging_config or {"enabled": False}
        self.logging_enabled = self.logging_config.get("enabled", False)
        
        if self.logging_enabled:
            print("Initializing pose logger with config:", self.logging_config)
            self.pose_logger = PoseLogger()
        else:
            self.pose_logger = None

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        """Required by Operator base class."""
        return self._hand_transformed_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._arm_transformed_keypoint_subscriber

    def return_real(self):
        return self.real

    # ------------------------------
    # Frame / Matrix utilities
    # ------------------------------
    def _get_hand_frame(self):
        """Get the hand frame with efficient caching strategy."""
        # Try to get new data (just once to avoid latency)
        data = self._arm_transformed_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        
        if data is None:
            # If no new data, use cached frame immediately
            if hasattr(self, 'last_valid_hand_frame') and self.last_valid_hand_frame is not None:
                return self.last_valid_hand_frame
            return None
        
        # Convert to correct shape
        try:
            frame_data = np.asanyarray(data).reshape(4, 3)  # shape (4,3)
            self.last_valid_hand_frame = frame_data  # Save this valid frame
            return frame_data
        except Exception as e:
            print(f"Error processing hand frame data: {e}")
            if hasattr(self, 'last_valid_hand_frame') and self.last_valid_hand_frame is not None:
                return self.last_valid_hand_frame
            return None

    def _turn_frame_to_homo_mat(self, frame):
        """Convert 4x3 frame to a 4x4 homogeneous transform."""
        t = frame[0]
        R = frame[1:]

        homo_mat = np.eye(4)
        homo_mat[:3, :3] = R.T
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

    def _homo2cart(self, homo_mat):
        # Here we will use the resolution scale to set the translation resolution
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(
            homo_mat[:3, :3]).as_quat()

        cart = np.concatenate(
            [t, R], axis=0
        )

        return cart

    def cart2homo(self, cart):
        """Inverse of _homo2cart."""
        homo = np.eye(4)
        t = cart[:3]
        R = Rotation.from_quat(cart[3:]).as_matrix()
        homo[:3, 3] = t
        homo[:3, :3] = R
        return homo

    def project_to_rotation_matrix(self, R):
        """Adjust a near-rotation matrix to be a valid rotation matrix using SVD."""
        U, _, Vt = np.linalg.svd(R)  # Perform SVD
        R_fixed = U @ Vt  # Reconstruct the rotation matrix

        # Ensure determinant is +1 (no reflection)
        if np.linalg.det(R_fixed) < 0:
            U[:, -1] *= -1  # Flip last column of U
            R_fixed = U @ Vt  # Recalculate R
        return R_fixed
    
    def quat_to_euler_rad(self, qx: float, qy: float, qz: float, qw: float) -> list[float, float, float]:
        """
        Convert quaternion to Euler angles in radians
        
        Args:
            qx, qy, qz, qw: Quaternion components in qx,qy,qz,qw order
            
        Returns:
            List of (roll, pitch, yaw) in radians
        """
        rot = Rotation.from_quat([qx, qy, qz, qw])
        # Use uppercase 'XYZ' for extrinsic rotations around fixed axes
        euler_rad = rot.as_euler('XYZ')  # Get Euler angles in radians, extrinsic rotation due to global ref frame in VR tracking
        return euler_rad

    def _get_resolution_scale_mode(self):
        if not self._arm_resolution_subscriber:
            return 1.0  # default
        data = self._arm_resolution_subscriber.recv_keypoints()
        if data is None:
            return 1.0
        scale = np.asanyarray(data).reshape(1)[0]
        return scale

    def _get_arm_teleop_state(self):
        if not self._arm_teleop_state_subscriber:
            return ARM_TELEOP_CONT  # or STOP
        data = self._arm_teleop_state_subscriber.recv_keypoints()
        if data is None:
            return self.arm_teleop_state
        state = np.asanyarray(data).reshape(1)[0]
        return state

    # ------------------------------
    # Teleop reset logic
    # ------------------------------
    def _reset_teleop(self):
        print('****** RESETTING TELEOP ******')
        self.unified_publisher.pub_keypoints(1, 'reset')
        self.robot_frame = self.endeff_homo_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        while self.robot_frame is None:
            self.unified_publisher.pub_keypoints(1, 'reset')
            self.robot_frame = self.endeff_homo_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            time.sleep(0.01)
        if self.robot_frame is None:
            print("ERROR: No robot frame received.")
            return None
        self.robot_init_H = self.robot_frame
        self.robot_moving_H = copy(self.robot_init_H)

        print("Robot init H:", self.robot_init_H)

        # Get initial hand frame
        first_hand_frame = None
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
            time.sleep(0.01)

        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])

        print("Hand init H:", self.hand_init_H)

        self.is_first_frame = False
        return first_hand_frame

    # ------------------------------
    # Main teleop: transforms
    # ------------------------------

    def _get_finger_coords(self):
        """Get transformed finger coordinates."""
        finger_coords = self._arm_transformed_keypoint_subscriber.recv_keypoints()
        if finger_coords is not None:
            return np.array(finger_coords).reshape(-1, 3)
        return None

    def _fix_quaternion_flips(self, quats):
        """Fix potential quaternion flips by ensuring all are in same hemisphere"""
        if len(quats) <= 1:
            return quats
        
        fixed = [quats[0]]  # First quaternion as reference
        for q in quats[1:]:
            # Calculate dot product with the previous quaternion
            dot = np.sum(fixed[-1] * q)
            # If negative, we're in the opposite hemisphere, so flip
            if dot < 0:
                fixed.append(-q)
            else:
                fixed.append(q)
        return np.array(fixed)

    def _apply_retargeted_angles(self):
        """Apply retargeted angles from hand motion to robot."""
        # Get arm teleop state first
        new_arm_teleop_state = self._get_arm_teleop_state()
        
        # Get resolution scale (like Allegro)
        arm_teleoperation_scale_mode = self._get_resolution_scale_mode()
        if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
            self.resolution_scale = 1
        elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
            self.resolution_scale = 0.6
                    
        # Check for reset or get hand frame
        is_reset = self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT)

        if is_reset:    
            # Reposition initial values for the robot and hand, then get moving hand frame
            moving_hand_frame = self._reset_teleop()
            self.is_first_frame = False
            
            # Initialize CompStateFilter with the first frame
            if self.use_filter and moving_hand_frame is not None:
                cart = self._homo2cart(self.robot_moving_H)
                self.comp_filter = CompStateFilter(
                    init_state=cart,
                    pos_ratio=0.7,  # Slightly stronger position filtering
                    ori_ratio=0.85,  # Strong orientation filtering
                    adaptive=True
                )
        else:
            moving_hand_frame = self._get_hand_frame()
        
        # Update state before None check
        self.arm_teleop_state = new_arm_teleop_state

        if moving_hand_frame is None: 
            return

        # Convert frame to homogeneous matrix
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)
        
        # Transformation begins
        H_HI_HH = copy(self.hand_init_H)
        H_HT_HH = copy(self.hand_moving_H)
        H_RI_RH = copy(self.robot_init_H)

        # These transformation matrices define the mapping between coordinate systems
        # Adjust these if your coordinate systems aren't aligning correctly
        H_R_V = np.array([[ 0,  0,  -1,  0],
                          [ 0,  1,  0,  0],
                          [ -1, 0,  0,  0],
                          [ 0,  0,  0,  1]])

        H_T_V = np.array([[ 0, -1,  0,  0],
                          [ 0,  0, -1,  0],
                          [ -1,  0,  0,  0],
                          [ 0,  0,  0,  1]])
        
        # Calculate transformations
        H_HT_HI = np.linalg.solve(H_HI_HH, H_HT_HH)  # More efficient than pinv for square matrices
        H_HT_HI_r = np.linalg.solve(H_R_V, H_HT_HI @ H_R_V)[:3,:3]
        H_HT_HI_t = np.linalg.solve(H_T_V, H_HT_HI @ H_T_V)[:3,3]
        
        # Make sure rotation part is a valid rotation matrix
        H_HT_HI_r = self.project_to_rotation_matrix(H_HT_HI_r)
        
        relative_affine = np.block([[H_HT_HI_r, H_HT_HI_t.reshape(3,1)], [0, 0, 0, 1]])
        
        # Update robot state
        H_RT_RH = H_RI_RH @ relative_affine
        
        # Ensure we have a valid transformation matrix
        H_RT_RH[:3, :3] = self.project_to_rotation_matrix(H_RT_RH[:3, :3])
        self.robot_moving_H = copy(H_RT_RH)

        # Get cart pose and apply filter
        cart = self._homo2cart(H_RT_RH)
        
        # Apply CompStateFilter (handles both position and orientation)
        if self.use_filter and self.comp_filter is not None:
            cart = self.comp_filter(cart)
        
        # Get euler angles
        position = cart[0:3]
        orientation = cart[3:7]
        roll, pitch, yaw = self.quat_to_euler_rad(orientation[0], orientation[1], orientation[2], orientation[3])
        euler_orientation = [roll, pitch, yaw]

        # Send cartesian coordinates
        self.unified_publisher.pub_keypoints({
            "position": position,
            "orientation": euler_orientation,
            "timestamp": time.time()
        }, "endeff_coords")

        if self.logging_enabled and self.pose_logger:
            try:
                self.pose_logger.log_frame(
                    self.hand_init_H, 
                    self.robot_init_H, 
                    self.hand_moving_H, 
                    H_RT_RH
                )
            except Exception as e:
                print(f"Error logging frame: {e}")

        if time.time() % 5 < 0.2:  # Print every 5 seconds
            euler = self.quat_to_euler_rad(cart[3], cart[4], cart[5], cart[6])
            print(f"\nDiagnostics:")
            print(f"  Hand frame determinant: {np.linalg.det(self.hand_moving_H[:3,:3]):.4f}")
            print(f"  Robot frame determinant: {np.linalg.det(self.robot_moving_H[:3,:3]):.4f}")
            print(f"  Euler angles (deg): Roll={np.degrees(euler[0]):.1f}, Pitch={np.degrees(euler[1]):.1f}, Yaw={np.degrees(euler[2]):.1f}")
            if hasattr(self, '_calibrated') and self._calibrated:
                print(f"  Applied offsets (deg): Roll={np.degrees(self.orientation_offset[0]):.1f}, Pitch={np.degrees(self.orientation_offset[1]):.1f}, Yaw={np.degrees(self.orientation_offset[2]):.1f}")
    
    def moving_average(self, action, queue, limit):
        """Apply moving average filter to action."""
        queue.append(action)
        if len(queue) > limit:
            queue.pop(0)
        return np.mean(queue, axis=0)
    
    def __del__(self):
        self._cleanup()

    def _cleanup(self):
        """Ensure logger saves all data before exit"""
        if self.pose_logger:
            self.pose_logger.close()