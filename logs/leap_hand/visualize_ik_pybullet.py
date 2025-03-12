#!/usr/bin/env python3
import pybullet as p
import numpy as np
import json
import os
import pybullet_data
import time
import zmq
from typing import List, Dict, Tuple, Optional, Union, Any
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime
from enum import Enum, auto


class FingerType(Enum):
    """Enum for finger types."""
    THUMB = auto()
    INDEX = auto()
    MIDDLE = auto()
    RING = auto()


@dataclass
class FingerPosition:
    """Dataclass to store finger position data."""
    pip: np.ndarray
    dip: np.ndarray
    tip: np.ndarray
    finger_type: FingerType
    
    @property
    def is_thumb(self) -> bool:
        return self.finger_type == FingerType.THUMB
    
    @property
    def is_index(self) -> bool:
        return self.finger_type == FingerType.INDEX
    
    @property
    def is_ring(self) -> bool:
        return self.finger_type == FingerType.RING


@dataclass
class HandConfiguration:
    """Dataclass to store hand configuration."""
    joint_angles: List[float]
    target_positions: List[np.ndarray]


class LogHandler:
    """Class for handling log operations."""
    
    @staticmethod
    def load_log(log_file: str) -> Dict[str, Any]:
        """Load log data from a file.
        
        Args:
            log_file: Path to the log file
            
        Returns:
            Log data as a dictionary
        """
        with open(log_file, 'r') as f:
            return json.load(f)
    
    @staticmethod
    def get_frame_data(log_data: Dict[str, Any], frame_number: int) -> Optional[Dict[str, Any]]:
        """Get data for a specific frame from the log.
        
        Args:
            log_data: The loaded log data
            frame_number: Frame number to retrieve
            
        Returns:
            Frame data or None if frame not found
        """
        # Frames are stored under the "frames" key in the JSON structure
        frames = log_data.get("frames", {})
        # Then access the specific frame by its number as a string
        return frames.get(str(frame_number), None)
    
    @staticmethod
    def save_visualization_log(log_data: Dict[str, Any], output_path: str) -> None:
        """Save visualization log data to a JSON file.
        
        Args:
            log_data: Dictionary containing visualization data
            output_path: Path to save the JSON file
        """
        # Ensure the directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Convert numpy arrays to lists for JSON serialization
        serializable_log = {
            "timestamp": log_data["timestamp"],
            "frames": {}
        }
        
        for frame_num, frame_data in log_data["frames"].items():
            serializable_log["frames"][frame_num] = {
                "target_positions": [pos.tolist() if hasattr(pos, 'tolist') else pos for pos in frame_data["target_positions"]],
                "calculated_joint_angles": [float(angle) for angle in frame_data["calculated_joint_angles"]]
            }
        
        # Write to file with pretty formatting
        with open(output_path, 'w') as f:
            json.dump(serializable_log, f, indent=2)
        
        print(f"Visualization log saved to {output_path}")


class CoordinateTransformer:
    """Class for handling coordinate transformations."""
    
    @staticmethod
    def transform_position(
        pos: List[float], 
        scale: float = 1.1, 
        is_ring_finger: bool = False, 
        is_index_finger: bool = False, 
        is_thumb: bool = False
    ) -> List[float]:
        """
        Apply standard transformation to a position for the leap hand.
        
        Args:
            pos: Original position [x, y, z]
            scale: Scale factor to apply
            is_ring_finger: Whether to apply ring finger displacement
            is_index_finger: Whether to apply index finger displacement
            is_thumb: Whether this is a thumb position
            
        Returns:
            Transformed position [x', y', z']
        """
        # Reflect and convert from y-up to z-up
        pos_reflected = [
            -pos[0],  # Negate x to reflect
            -pos[2],  # Negate z (which becomes y in PyBullet)
            pos[1]    # y in log becomes z in PyBullet
        ]
        
        if is_thumb:
            scale = 1.0
        # Rotate 90° around z-axis and scale
        pos_transformed = [
            pos_reflected[1] * scale,           # y becomes x (scaled)
            -pos_reflected[0] * scale,          # -x becomes y (scaled)
            pos_reflected[2] * scale            # z stays z (scaled)
        ]
        
        # Apply special displacement for ring finger if needed
        if is_ring_finger:
            ring_displacement = -0.02
            pos_transformed[1] += ring_displacement
        
        # Apply special displacement for index finger if needed
        if is_index_finger:
            index_displacement = 0.01  # Opposite direction of ring displacement
            pos_transformed[1] += index_displacement
        
        return pos_transformed


class HandPositionCalculator:
    """Class for calculating hand positions."""
    
    @staticmethod
    def compute_hand_pos(frame_data: Dict[str, Any], scale: float = 1.3) -> List[np.ndarray]:
        """
        Build a list of 12 key positions for IK.
        We choose for each finger (thumb, index, middle, ring) a trio of positions.
        Here we use the last three positions.
        The order will be:
          hand_pos[0,1,2] -> thumb (PIP, DIP, fingertip),
          hand_pos[3,4,5] -> index (PIP, DIP, fingertip),
          hand_pos[6,7,8] -> middle (PIP, DIP, fingertip),
          hand_pos[9,10,11] -> ring (PIP, DIP, fingertip).
        
        Args:
            frame_data: Data for the current frame
            scale: Scale factor to apply to positions
            
        Returns:
            List of 12 positions for IK calculation
        """
        finger_positions = frame_data["finger_input_positions"]
        hand_pos = []
        
        finger_types = {
            'thumb': FingerType.THUMB,
            'index': FingerType.INDEX, 
            'middle': FingerType.MIDDLE, 
            'ring': FingerType.RING
        }
        
        for finger, finger_type in finger_types.items():
            positions = finger_positions.get(finger, [])
            if len(positions) >= 3:
                pip_pos = positions[-3]  # Bottom position (PIP joint)
                dip_pos = positions[-2]  # Middle position (DIP joint)
                fingertip_pos = positions[-1]  # Tip position (actual fingertip)
            else:
                raise ValueError(f"Not enough positions for finger {finger}")
            
            # Transform all three positions
            is_ring = finger == 'ring'
            is_index = finger == 'index'
            is_thumb = finger == 'thumb'
            
            transformer = CoordinateTransformer()
            pip_pos_transformed = transformer.transform_position(pip_pos, scale, is_ring, is_index, is_thumb)
            dip_pos_transformed = transformer.transform_position(dip_pos, scale, is_ring, is_index, is_thumb)
            fingertip_pos_transformed = transformer.transform_position(fingertip_pos, scale, is_ring, is_index, is_thumb)
            
            # Add all three transformed positions to the hand_pos list
            hand_pos.append(np.array(pip_pos_transformed, dtype=float))
            hand_pos.append(np.array(dip_pos_transformed, dtype=float))
            hand_pos.append(np.array(fingertip_pos_transformed, dtype=float))
        
        return hand_pos


class VisualMarkerManager:
    """Class for managing visual markers in PyBullet."""
    
    @staticmethod
    def create_visual_marker(position: List[float], color: List[float] = [1, 0, 0, 0.7], size: float = 0.01) -> int:
        """Create a visual sphere marker at the specified position.
        
        Args:
            position: 3D position [x, y, z]
            color: RGBA color [r, g, b, a]
            size: Radius of the sphere
            
        Returns:
            PyBullet object ID for the marker
        """
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=size,
            rgbaColor=color
        )
        marker_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=position
        )
        return marker_id
    
    @staticmethod
    def create_coordinate_frame(position: List[float] = [0, 0, 0], size: float = 0.1) -> None:
        """Create a visual coordinate frame at the specified position.
        
        Args:
            position: Origin position of the frame
            size: Size of the coordinate axes
        """
        # X-axis (red)
        p.addUserDebugLine(position, [position[0] + size, position[1], position[2]], [1, 0, 0], 3)
        # Y-axis (green)
        p.addUserDebugLine(position, [position[0], position[1] + size, position[2]], [0, 1, 0], 3)
        # Z-axis (blue)
        p.addUserDebugLine(position, [position[0], position[1], position[2] + size], [0, 0, 1], 3)
        
        # Add text labels for each axis
        p.addUserDebugText("X", [position[0] + size, position[1], position[2]], [1, 0, 0], 1.5)
        p.addUserDebugText("Y", [position[0], position[1] + size, position[2]], [0, 1, 0], 1.5)
        p.addUserDebugText("Z", [position[0], position[1], position[2] + size], [0, 0, 1], 1.5)
    
    @staticmethod
    def update_visual_marker_position(marker_id: int, position: List[float]) -> int:
        """Update the position of an existing visual marker.
        
        Args:
            marker_id: PyBullet object ID of the marker
            position: New 3D position [x, y, z]
            
        Returns:
            PyBullet object ID of the marker
        """
        p.resetBasePositionAndOrientation(marker_id, position, [0, 0, 0, 1])
        return marker_id


class JointMapUtility:
    """Utility class for hand joint mapping."""
    
    @staticmethod
    def get_letter_to_joint_mapping(hand_id: int) -> Dict[str, int]:
        """Get mapping from letter joints to their indices.
        
        Args:
            hand_id: ID of the hand in PyBullet
            
        Returns:
            Dictionary mapping joint letters to indices
        """
        letter_joints = {}
        for i in range(p.getNumJoints(hand_id)):
            joint_info = p.getJointInfo(hand_id, i)
            joint_name = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
            if joint_name in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']:
                letter_joints[joint_name] = i
        return letter_joints
    
    @staticmethod
    def get_letter_to_ik_index_mapping() -> Dict[str, int]:
        """Get mapping from joint letters to their IK solution indices.
        
        Returns:
            Dictionary mapping joint letters to IK indices
        """
        return {
            'a': 0, 'b': 1, 'c': 2, 'd': 3,      # Index finger
            'e': 4, 'f': 5, 'g': 6, 'h': 7,      # Middle finger
            'i': 8, 'j': 9, 'k': 10, 'l': 11,    # Ring finger
            'm': 12, 'n': 13, 'o': 14, 'p': 15   # Thumb
        }
    
    @staticmethod
    def print_joint_mapping(hand_id: int) -> Dict[str, int]:
        """Print detailed information about joints and return the mapping.
        
        Args:
            hand_id: ID of the hand in PyBullet
            
        Returns:
            Dictionary mapping joint letters to indices
        """
        letter_joints = JointMapUtility.get_letter_to_joint_mapping(hand_id)
        
        # Print joint information
        print("\n=== Joint Mapping Information ===")
        print("Index | Name | Type | Parent Link")
        print("-" * 60)
        
        for i in range(p.getNumJoints(hand_id)):
            joint_info = p.getJointInfo(hand_id, i)
            joint_index = joint_info[0]
            joint_name = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
            joint_type = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"][joint_info[2]]
            parent_link_name = joint_info[12].decode('utf-8') if isinstance(joint_info[12], bytes) else str(joint_info[12])
            
            # Only print detailed info for movable joints
            if joint_info[2] != p.JOINT_FIXED:
                print(f"{joint_index:5d} | {joint_name:4s} | {joint_type:8s} | {parent_link_name}")
        
        # Print finger groupings
        print("\n=== Finger Joint Mapping ===")
        print("Index finger joints:", end=" ")
        for letter in ['a', 'b', 'c', 'd']:
            if letter in letter_joints:
                print(f"{letter}({letter_joints[letter]})", end=" ")
        print()
        
        print("Middle finger joints:", end=" ")
        for letter in ['e', 'f', 'g', 'h']:
            if letter in letter_joints:
                print(f"{letter}({letter_joints[letter]})", end=" ")
        print()
        
        print("Ring finger joints:", end=" ")
        for letter in ['i', 'j', 'k', 'l']:
            if letter in letter_joints:
                print(f"{letter}({letter_joints[letter]})", end=" ")
        print()
        
        print("Thumb joints:", end=" ")
        for letter in ['m', 'n', 'o', 'p']:
            if letter in letter_joints:
                print(f"{letter}({letter_joints[letter]})", end=" ")
        print()
        
        return letter_joints


class InverseKinematicsService:
    """Service class for handling inverse kinematics calculations."""
    
    def __init__(self, urdf_path: str = None):
        """Initialize the IK service."""
        self.urdf_path = urdf_path
        self.zmq_context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        self.connection_established = False
    
    def connect(self, server_address: str = "tcp://localhost:5555") -> bool:
        """Connect to the Pinocchio server.
        
        Args:
            server_address: ZMQ server address
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.zmq_context = zmq.Context()
            self.socket = self.zmq_context.socket(zmq.REQ)
            self.socket.connect(server_address)
            self.connection_established = True
            return True
        except Exception as e:
            print(f"Error connecting to Pinocchio server: {e}")
            self.connection_established = False
            return False
    
    def compute_ik_pinocchio(
        self, 
        hand_id: int, 
        target_positions: List[np.ndarray], 
        end_effector_indices: List[int], 
        current_positions: Optional[List[float]] = None
    ) -> List[float]:
        """
        Compute inverse kinematics using the Pinocchio server.
        
        Args:
            hand_id: PyBullet ID of the hand
            target_positions: List of target positions for each finger joint
            end_effector_indices: List of end effector joint indices
            current_positions: Optional list of current joint positions to use as seed
            
        Returns:
            List of calculated joint angles for the LEAP hand
        """
        # If no current positions are provided, get them from the current state
        if current_positions is None:
            # Only collect positions for movable joints (DOFs)
            movable_joint_indices = []
            for i in range(p.getNumJoints(hand_id)):
                joint_info = p.getJointInfo(hand_id, i)
                if joint_info[2] != p.JOINT_FIXED:  # Skip fixed joints
                    movable_joint_indices.append(i)
            
            current_positions = []
            for i in movable_joint_indices:
                joint_state = p.getJointState(hand_id, i)
                current_positions.append(joint_state[0])
        
        # Check connection
        if not self.connection_established:
            if not self.connect():
                print("Failed to connect to Pinocchio server, falling back to PyBullet IK")
                return self.compute_ik_pybullet(hand_id, target_positions, end_effector_indices, current_positions)
        
        # Prepare data to send
        try:
            data = {
                "target_positions": [pos.tolist() if hasattr(pos, 'tolist') else pos for pos in target_positions],
                "current_positions": current_positions,
                "urdf_path": self.urdf_path
            }
            
            # Send request
            self.socket.send_json(data)
            
            # Get response with timeout
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)
            if poller.poll(1000):  # 1 second timeout
                response = self.socket.recv_json()
                joint_angles = response.get("joint_angles", [])
                
                # Add debugging here
                print("Received joint angles from Pinocchio:", joint_angles)
                
                # Use JointMapUtility instead of duplicating code
                letter_to_joint_idx = JointMapUtility.get_letter_to_joint_mapping(hand_id)
                
                # Apply joint angles to PyBullet model
                print("Applying joint angles to hand model...")
                for letter, joint_idx in letter_to_joint_idx.items():
                    letter_index = ord(letter) - ord('a')
                    if letter_index < len(joint_angles):
                        angle_value = joint_angles[letter_index]
                        print(f"Setting joint {letter} (index {joint_idx}) to {angle_value}")
                        
                        # First reset the joint state directly
                        p.resetJointState(hand_id, joint_idx, angle_value)
                        
                        # Then also apply using position control with strong force
                        p.setJointMotorControl2(
                            bodyIndex=hand_id,
                            jointIndex=joint_idx,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle_value,
                            targetVelocity=0,
                            force=1000,  # Increased force for stronger movement
                            positionGain=1.0,  # Increased gain for faster response
                            velocityGain=1.0,
                        )
                
                return joint_angles
            else:
                print("Timeout waiting for Pinocchio server response")
                return self.compute_ik_pybullet(hand_id, target_positions, end_effector_indices, current_positions)
        except Exception as e:
            print(f"Error communicating with Pinocchio server: {e}")
            print("Falling back to PyBullet IK...")
            return self.compute_ik_pybullet(hand_id, target_positions, end_effector_indices, current_positions)
    
    def compute_ik_pybullet(
        self, 
        hand_id: int, 
        target_positions: List[np.ndarray], 
        end_effector_indices: List[int], 
        current_positions: Optional[List[float]] = None
    ) -> List[float]:
        """
        Compute inverse kinematics for the hand using PIP, DIP, and fingertip positions with PyBullet.
        
        Args:
            hand_id: PyBullet ID of the hand
            target_positions: List of target positions for each finger joint
            end_effector_indices: List of end effector joint indices
            current_positions: Optional list of current joint positions to use as seed
            
        Returns:
            List of calculated joint angles for the LEAP hand with letter-named joints
        """
        # Extract positions from target_positions
        # The order from compute_hand_pos is:
        # [thumb_pip, thumb_dip, thumb_fingertip, 
        #  index_pip, index_dip, index_fingertip, 
        #  middle_pip, middle_dip, middle_fingertip, 
        #  ring_pip, ring_dip, ring_fingertip]
        
        thumb_pip_pos = target_positions[0]
        thumb_dip_pos = target_positions[1]
        thumb_fingertip_pos = target_positions[2]
        
        index_pip_pos = target_positions[3]
        index_dip_pos = target_positions[4]
        index_fingertip_pos = target_positions[5]
        
        middle_pip_pos = target_positions[6]
        middle_dip_pos = target_positions[7]
        middle_fingertip_pos = target_positions[8]
        
        ring_pip_pos = target_positions[9]
        ring_dip_pos = target_positions[10]
        ring_fingertip_pos = target_positions[11]
        
        # PIP joints (proximal interphalangeal - first knuckles)
        pip_indices = [3, 8, 13, 18]
        # DIP joints (distal interphalangeal - middle knuckles)
        dip_indices = [4, 9, 14, 19]
        # Tip head links (visual tip extensions)
        fingertip_indices = [5, 10, 15, 20]
        
        # Combine all indices
        all_end_effector_indices = pip_indices + dip_indices + fingertip_indices
        
        print("Using all finger joints (PIP, DIP, fingertip) as end effectors")
        print(f"PIP joint indices: {pip_indices}")
        print(f"DIP joint indices: {dip_indices}")
        print(f"Fingertip joint indices: {fingertip_indices}")
        
        # Map target positions to all sets of end effectors
        leap_end_effector_pos = [
            # For PIP joints
            index_pip_pos,      # For index PIP joint
            middle_pip_pos,     # For middle PIP joint
            ring_pip_pos,       # For ring PIP joint
            thumb_pip_pos,      # For thumb PIP joint
            
            # For DIP joints
            index_dip_pos,      # For index DIP joint
            middle_dip_pos,     # For middle DIP joint
            ring_dip_pos,       # For ring DIP joint
            thumb_dip_pos,      # For thumb DIP joint
            
            # For fingertip joints
            index_fingertip_pos,  # For index fingertip joint
            middle_fingertip_pos, # For middle fingertip joint
            ring_fingertip_pos,   # For ring fingertip joint
            thumb_fingertip_pos   # For thumb fingertip joint
        ]
        
        # If no current positions are provided, get them from the current state
        if current_positions is None:
            # Only collect positions for movable joints (DOFs)
            movable_joint_indices = []
            for i in range(p.getNumJoints(hand_id)):
                joint_info = p.getJointInfo(hand_id, i)
                if joint_info[2] != p.JOINT_FIXED:  # Skip fixed joints
                    movable_joint_indices.append(i)
            
            current_positions = []
            for i in movable_joint_indices:
                joint_state = p.getJointState(hand_id, i)
                current_positions.append(joint_state[0])
        
        # Calculate IK using all sets of end effectors
        joint_poses = p.calculateInverseKinematics2(
            hand_id,
            all_end_effector_indices,
            leap_end_effector_pos,
            currentPositions=current_positions,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
            jointDamping=[0.1] * len(current_positions)  # Match the number of DOFs
        )
        
        # Use JointMapUtility instead of duplicating code
        letter_to_joint_idx = JointMapUtility.get_letter_to_joint_mapping(hand_id)
        letter_to_ik_idx = JointMapUtility.get_letter_to_ik_index_mapping()
        
        # Update the hand joints
        for letter, joint_idx in letter_to_joint_idx.items():
            if letter in letter_to_ik_idx:
                ik_idx = letter_to_ik_idx[letter]
                if ik_idx < len(joint_poses):
                    p.setJointMotorControl2(
                        bodyIndex=hand_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=joint_poses[ik_idx],
                        targetVelocity=0,
                        force=500,
                        positionGain=0.3,
                        velocityGain=1,
                    )
        
        # Create the output array with 16 joints as in the example
        real_robot_hand_q = [0.0] * 16
        
        # Map the joint angles to the output array
        # The first 16 values of joint_poses correspond to the 16 joints we need
        if len(joint_poses) >= 16:
            real_robot_hand_q[0:4] = joint_poses[0:4]
            real_robot_hand_q[4:8] = joint_poses[4:8]
            real_robot_hand_q[8:12] = joint_poses[8:12]
            real_robot_hand_q[12:16] = joint_poses[12:16]
        
        return [float(i) for i in real_robot_hand_q]


class HandVisualizer:
    """Main class for hand visualization."""
    
    def __init__(self, urdf_path: str):
        """Initialize the hand visualizer.
        
        Args:
            urdf_path: Path to the LEAP hand URDF file
        """
        self.urdf_path = urdf_path
        self.hand_id = None
        self.ik_solver = InverseKinematicsService(urdf_path).compute_ik_pinocchio
    
    def setup_environment(self) -> bool:
        """Set up the PyBullet environment and load the hand model.
        
        Returns:
            True if setup successful, False otherwise
        """
        # Connect to PyBullet
        p.connect(p.GUI)
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(1)  # Enable real-time simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Check if URDF exists
        if not os.path.exists(self.urdf_path):
            print(f"URDF not found at: {self.urdf_path}")
            return False
        
        # Load the hand without rotation
        self.hand_id = p.loadURDF(
            self.urdf_path, 
            basePosition=[0, 0, 0],
            useFixedBase=True
        )
        
        # Apply joint limits for specified indices
        joint_limits = {
            2: (-0.2, 0.2),   # joint 'b' (index finger) - expanded range
            7: (-0.2, 0.2),   # joint 'f' (middle finger)
            12: (-0.2, 0.2),  # joint 'j' (ring finger)
            16: (-0.2, 0.2)   # joint 'n' (thumb)
        }
        
        for joint_index, (lower_limit, upper_limit) in joint_limits.items():
            p.changeDynamics(
                self.hand_id, 
                joint_index, 
                jointLowerLimit=lower_limit, 
                jointUpperLimit=upper_limit
            )
        
        # Initialize hand to open position
        self.initialize_hand_open()
        
        # Print model information
        self.print_hand_info()
        
        # Create coordinate frame
        VisualMarkerManager.create_coordinate_frame()
        
        # Set up camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        return True
    
    def initialize_hand_open(self) -> None:
        """Initialize the hand to an open position."""
        print("\n=== Initializing Hand to Open Position ===")
        # Set all joints to 0 (or slightly open)
        for i in range(p.getNumJoints(self.hand_id)):
            joint_info = p.getJointInfo(self.hand_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:  # Only adjust revolute joints
                # Set to 0 or a small value for a slightly open hand
                p.resetJointState(self.hand_id, i, 0.0)
    
    def print_hand_info(self) -> None:
        """Print information about the hand model."""
        print("\n=== Hand Model Information ===")
        num_joints = p.getNumJoints(self.hand_id)
        print(f"Number of joints: {num_joints}")
        
        # Print joint mapping
        self.print_joint_mapping()
    
    def print_joint_mapping(self) -> Dict[str, int]:
        """Print detailed information about all joints in the hand model and return mapping."""
        return JointMapUtility.print_joint_mapping(self.hand_id)
    
    def visualize_log(self, log_file: str, output_file: Optional[str] = None) -> None:
        """Visualize hand positions from log file.
        
        Args:
            log_file: Path to the log file
            output_file: Optional path to save visualization results
        """

        # Load the log
        log_handler = LogHandler()
        log_data = log_handler.load_log(log_file)
        
        # Get available frames
        if "frames" not in log_data:
            print("No frames found in log.")
            return
        
        available_frames = sorted([int(k) for k in log_data["frames"].keys()])
        print(f"Found {len(available_frames)} frames.")
        
        # Create markers for all finger positions
        finger_markers = {}
        target_markers = []
        marker_manager = VisualMarkerManager()
        
        # Define finger colors
        finger_colors = {
            'thumb': [1, 0, 0, 0.7],    # Red
            'index': [0, 1, 0, 0.7],    # Green
            'middle': [0, 0, 1, 0.7],   # Blue
            'ring': [1, 1, 0, 0.7]      # Yellow
        }
        
        # Define colors for target markers - slightly different from finger colors
        target_colors = {
            'thumb': [0.8, 0, 0, 1.0],    # Darker red
            'index': [0, 0.8, 0, 1.0],    # Darker green
            'middle': [0, 0, 0.8, 1.0],   # Darker blue
            'ring': [0.8, 0.8, 0, 1.0]    # Darker yellow
        }
        
        # Create a consistent scale factor
        scale_factor = 1.1
        
        print("\n=== VISUALIZING HAND POSITIONS AND CALCULATING IK ===")
        
        # Create a dictionary to store visualization results
        visualization_log = {
            "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "frames": {}
        }
        
        # Pre-create target markers (12 positions)
        # Create 3 markers per finger with consistent colors
        for finger_idx, finger_type in enumerate(['thumb', 'index', 'middle', 'ring']):
            # Each finger has 3 markers (PIP, DIP, tip)
            for j in range(3):
                marker_id = marker_manager.create_visual_marker(
                    [0, 0, 0], 
                    target_colors[finger_type], 
                    0.01
                )
                target_markers.append(marker_id)
        
        # Process each frame
        for frame_number in available_frames:
            frame_data = log_handler.get_frame_data(log_data, frame_number)
            if frame_data is None:
                continue

            # Compute target positions for inverse kinematics
            calculator = HandPositionCalculator()
            target_positions = calculator.compute_hand_pos(frame_data, scale_factor)

            # Get all finger positions from the log
            finger_positions = frame_data["finger_input_positions"]
            
            # Update marker positions for target positions
            for i, position in enumerate(target_positions):
                if i < len(target_markers):
                    marker_manager.update_visual_marker_position(target_markers[i], position)
            
            # Create or update markers for all finger positions
            for finger, positions in finger_positions.items():
                if finger not in finger_markers:
                    finger_markers[finger] = []
                    
                # Create new markers if needed
                while len(finger_markers[finger]) < len(positions):
                    marker_id = marker_manager.create_visual_marker([0, 0, 0], finger_colors[finger], 0.01)
                    finger_markers[finger].append(marker_id)
                
                # Update all marker positions
                for i, pos in enumerate(positions):
                    if i < len(finger_markers[finger]):
                        # Apply the same transformation as in compute_hand_pos
                        transformer = CoordinateTransformer()
                        pos_transformed = transformer.transform_position(
                            pos, scale_factor, 
                            finger == 'ring', 
                            finger == 'index', 
                            finger == 'thumb'
                        )
                        marker_manager.update_visual_marker_position(finger_markers[finger][i], pos_transformed)
            
            # PIP joints (proximal interphalangeal - first knuckles)
            pip_indices = [3, 8, 13, 18]
            # DIP joints (distal interphalangeal - middle knuckles)
            dip_indices = [4, 9, 14, 19]
            # Tip head links (visual tip extensions)
            fingertip_indices = [5, 10, 15, 20]
            
            # Combine all indices
            all_end_effector_indices = pip_indices + dip_indices + fingertip_indices
            
            # Calculate joint angles using inverse kinematics
            calculated_joint_angles = self.ik_solver(
                self.hand_id,
                target_positions,
                all_end_effector_indices
            )
            
            # Store results in visualization log
            visualization_log["frames"][str(frame_number)] = {
                "target_positions": target_positions,
                "calculated_joint_angles": calculated_joint_angles
            }
            
            # Wait for user to press Enter to continue to next frame
            input(f"Frame {frame_number}/{available_frames[-1]} - Press Enter to continue...")
        
        # Save visualization results if output path provided
        if output_file:
            log_handler.save_visualization_log(visualization_log, output_file)


def main() -> None:
    """Main function to run the hand visualization."""
    print("=== SCRIPT STARTED ===")
    
    # Define URDF path
    urdf_path = os.path.join(os.getcwd(), "openteach/components/environment/assets/urdf/leap_hand/leap_hand_right.urdf")
    
    # Create and set up the hand visualizer
    visualizer = HandVisualizer(urdf_path)
    if not visualizer.setup_environment():
        return
    
    # Get log file path from user
    log_file = input("Enter path to log file: ")
    log_file = f"logs/{log_file}"
    
    print(f"Opening log file: {log_file}")
    
    if not os.path.exists(log_file):
        print("Log file does not exist")
        return
    
    # Generate output file path based on input log name
    output_dir = "logs/visualization_output"
    os.makedirs(output_dir, exist_ok=True)
    log_basename = os.path.basename(log_file)
    output_file = os.path.join(output_dir, f"vis_{log_basename}")
    
    # Run visualization
    visualizer.visualize_log(log_file, output_file)
    
    print("=== VISUALIZATION COMPLETE ===")
    
    # Keep the simulation running until user exits
    while True:
        time.sleep(0.1)  # Small sleep to prevent high CPU usage
        # Check for exit input
        if input("Press Enter to exit...") != "":
            break


if __name__ == "__main__":
    main()