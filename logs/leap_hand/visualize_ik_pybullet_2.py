#!/usr/bin/env python3

import pybullet as p
import numpy as np
import json
import os
import pybullet_data
import time
from datetime import datetime


def load_log(log_file):
    with open(log_file, 'r') as f:
        return json.load(f)

def get_frame_data(log_data, frame_number):
    # Frames are stored under the "frames" key in the JSON structure
    frames = log_data.get("frames", {})
    # Then access the specific frame by its number as a string
    return frames.get(str(frame_number), None)

def transform_position(pos, scale, is_ring_finger=False):
    """
    Apply transformation to a position with only x-axis reflection.
    
    Args:
        pos: Original position [x, y, z]
        scale: Scale factor to apply
        is_ring_finger: Whether to apply ring finger displacement
        
    Returns:
        Transformed position [x', y', z']
    """
    # Reflect only x-axis and convert from y-up to z-up
    pos_reflected = [
        -pos[0],  # Negate x to reflect
        -pos[2],  # Negate z (which becomes y in PyBullet)
        pos[1]    # y in log becomes z in PyBullet
    ]
    
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
    
    return pos_transformed

def compute_hand_pos(frame_data, scale=1.3):
    """
    Build a list of 8 key positions for IK.
    We choose for each finger (thumb, index, middle, ring) a pair of positions.
    Here we use the last two positions.
    The order will be:
      hand_pos[0,1] -> thumb,
      hand_pos[2,3] -> index,
      hand_pos[4,5] -> middle,
      hand_pos[6,7] -> ring.
    """
    finger_positions = frame_data["finger_input_positions"]
    hand_pos = []
    for finger in ['thumb', 'index', 'middle', 'ring']:
        positions = finger_positions.get(finger, [])
        if len(positions) >= 2:
            middle_pos = positions[-2]
            tip_pos = positions[-1]
            # print(f"Finger {finger} middle pos: {middle_pos}, tip pos: {tip_pos}")
        else:
            raise ValueError(f"Not enough positions for finger {finger}")
        
        # First, reflect the keypoints and convert from y-up to z-up
        middle_pos_transformed = transform_position(middle_pos, scale, finger == 'ring')
        tip_pos_transformed = transform_position(tip_pos, scale, finger == 'ring')
        
        # print(f"Finger {finger} transformed middle pos: {middle_pos_transformed}, transformed tip pos: {tip_pos_transformed}")
        
        hand_pos.append(np.array(middle_pos_transformed, dtype=float))
        hand_pos.append(np.array(tip_pos_transformed, dtype=float))
    return hand_pos

def create_visual_marker(position, color=[1, 0, 0, 0.7], size=0.01):
    """Create a visual sphere marker at the specified position."""
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

def create_coordinate_frame(position=[0, 0, 0], size=0.1):
    """Create a visual coordinate frame at the specified position."""
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

def update_visual_marker_position(marker_id, position):
    """Update the position of an existing visual marker."""
    p.resetBasePositionAndOrientation(marker_id, position, [0, 0, 0, 1])
    return marker_id

def compute_ik(hand_id, target_positions, end_effector_indices):
    """
    Compute inverse kinematics for the hand using both fingertip and middle positions.
    
    Args:
        hand_id: PyBullet ID of the hand
        target_positions: List of target positions for each finger joint
        end_effector_indices: List of end effector joint indices
        
    Returns:
        List of calculated joint angles for the LEAP hand
    """
    # Remove p.stepSimulation() since we're using real-time simulation
    
    # Extract positions from target_positions
    # The order from compute_hand_pos is:
    # [thumb_middle, thumb_tip, index_middle, index_tip, middle_middle, middle_tip, ring_middle, ring_tip]
    thumb_middle_pos = target_positions[0]
    thumb_tip_pos = target_positions[1]
    index_middle_pos = target_positions[2]
    index_tip_pos = target_positions[3]
    middle_middle_pos = target_positions[4]
    middle_tip_pos = target_positions[5]
    ring_middle_pos = target_positions[6]
    ring_tip_pos = target_positions[7]
    
    # Define the end effector indices for middle and tip joints
    # Based on the URDF, these are:
    # Index finger: 2 (PIP/middle), 3 (fingertip)
    # Middle finger: 6 (PIP/middle), 7 (fingertip)
    # Ring finger: 10 (PIP/middle), 11 (fingertip)
    # Thumb: 14 (DIP/middle), 15 (fingertip)
    middle_indices = [3, 7, 11, 15]  # PIP/DIP joints
    tip_indices = [4, 8, 12, 16]     # Fingertip joints
    
    # Combine all end effector indices
    all_indices = middle_indices + tip_indices
    
    print("Using both middle and fingertip joints as end effectors")
    print(f"Middle joint indices: {middle_indices}")
    print(f"Fingertip indices: {tip_indices}")
    
    # Map target positions to all end effectors
    all_target_positions = [
        index_middle_pos,   # Joint 2 - Index PIP
        middle_middle_pos,  # Joint 6 - Middle PIP
        ring_middle_pos,    # Joint 10 - Ring PIP
        thumb_middle_pos,   # Joint 14 - Thumb DIP
        index_tip_pos,      # Joint 3 - Index fingertip
        middle_tip_pos,     # Joint 7 - Middle fingertip
        ring_tip_pos,       # Joint 11 - Ring fingertip
        thumb_tip_pos       # Joint 15 - Thumb fingertip
    ]
    
    # Calculate IK using all end effectors
    try:
        joint_poses = p.calculateInverseKinematics2(
            hand_id,
            all_indices,
            all_target_positions,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
    except Exception as e:
        print(f"IK calculation failed: {e}")
        # Return zeros as a fallback
        return [0.0] * 16
    
    # Create a mapping from joint name to joint index
    joint_name_to_idx = {}
    for i in range(p.getNumJoints(hand_id)):
        joint_info = p.getJointInfo(hand_id, i)
        joint_name = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
        if joint_name in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15']:
            joint_name_to_idx[joint_name] = i
    
    # Update the hand joints
    for joint_name, joint_idx in joint_name_to_idx.items():
        if joint_idx < len(joint_poses):
            p.setJointMotorControl2(
                bodyIndex=hand_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[joint_idx],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )
    
    # Create the output array with 16 joints
    real_robot_hand_q = [0.0] * 16
    
    # Map the joint angles to the output array
    if len(joint_poses) >= 16:
        for i in range(16):
            real_robot_hand_q[i] = joint_poses[i]
    
    return [float(i) for i in real_robot_hand_q]

def save_visualization_log(log_data, output_path):
    """
    Save visualization log data to a JSON file.
    
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

def print_joint_mapping(hand_id):
    """
    Print detailed information about all joints in the hand model,
    including their indices, names, and types.
    
    Args:
        hand_id: PyBullet robot ID
    """
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
    
    # Now print the mapping between letter names and indices
    print("\n=== Letter Joint to Index Mapping ===")
    letter_joints = {}
    for i in range(p.getNumJoints(hand_id)):
        joint_info = p.getJointInfo(hand_id, i)
        joint_name = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
        if joint_name in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p']:
            letter_joints[joint_name] = i
    
    # Sort by letter and print
    for letter in sorted(letter_joints.keys()):
        index = letter_joints[letter]
        joint_info = p.getJointInfo(hand_id, index)
        parent = joint_info[12].decode('utf-8') if isinstance(joint_info[12], bytes) else str(joint_info[12])
        print(f"Joint '{letter}' (index {index}): {parent}")
    
    # Print finger mapping
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

def main():
    print("=== SCRIPT STARTED ===")
    
    # Connect to PyBullet (GUI mode so you can see the hand)
    p.connect(p.GUI)
    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(1)  # Enable real-time simulation
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load the Leap Hand URDF - UPDATED PATH
    urdf_path = os.path.join(os.getcwd(), "openteach/components/environment/assets/urdf/leap_hand/leap_right_urdf/leap_right.urdf")
    if not os.path.exists(urdf_path):
        print("URDF not found at:", urdf_path)
        return
    hand_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
    
    # Initialize the hand in an open position
    print("\n=== Initializing Hand to Open Position ===")
    # Set all joints to 0 (or slightly open)
    for i in range(p.getNumJoints(hand_id)):
        joint_info = p.getJointInfo(hand_id, i)
        if joint_info[2] == p.JOINT_REVOLUTE:  # Only adjust revolute joints
            # Set to 0 or a small value for a slightly open hand
            p.resetJointState(hand_id, i, 0.0)

    
    # Print information about the hand model
    print("\n=== Hand Model Information ===")
    num_joints = p.getNumJoints(hand_id)
    print(f"Number of joints: {num_joints}")

    # Print joint mapping
    print_joint_mapping(hand_id)
    
    # Define all movable joint indices (excluding fixed joints)
    movable_joint_indices = []
    for i in range(p.getNumJoints(hand_id)):
        joint_info = p.getJointInfo(hand_id, i)
        if joint_info[2] == p.JOINT_REVOLUTE:  # Only include REVOLUTE joints
            movable_joint_indices.append(i)
    
    print(f"\nMovable joint indices: {movable_joint_indices}")
    
    # Define end effector joint indices for the leap_right.urdf
    # These will be updated based on the print_joint_mapping output
    # For now, using the joint indices from the URDF (joints 3, 7, 11, 15 for fingertips)
    # and joints before them (2, 6, 10, 14 for middle joints)
    leap_end_effector_indices = [2, 3, 6, 7, 10, 11, 14, 15]
    print("\n=== End Effector Joints ===")
    for i, idx in enumerate(leap_end_effector_indices):
        joint_info = p.getJointInfo(hand_id, idx)
        joint_name = joint_info[1].decode('utf-8') if isinstance(joint_info[1], bytes) else str(joint_info[1])
        finger_name = ["index", "index", "middle", "middle", "ring", "ring", "thumb", "thumb"][i]
        joint_type = ["middle", "tip"][i % 2]
        print(f"{finger_name} {joint_type}: Joint {idx} ({joint_name})")
    
    # Load the log file
    log_file = input("Enter path to log file: ")
    log_file = f"logs/{log_file}"
    
    print(f"Opening log file: {log_file}")
    
    if not os.path.exists(log_file):
        print("Log file does not exist")
        return
    log_data = load_log(log_file)
    
    # Get all available frames
    if "frames" not in log_data:
        print("No frames found in log.")
        return
    
    available_frames = sorted([int(k) for k in log_data["frames"].keys()])
    print(f"Found {len(available_frames)} frames.")
    
    # Create a dictionary to store markers for all finger positions
    finger_markers = {}
    
    # Initialize markers for all finger positions (all positions, not just middle and tip)
    finger_colors = {
        'thumb': [1, 0, 0, 0.7],    # Red
        'index': [0, 1, 0, 0.7],    # Green
        'middle': [0, 0, 1, 0.7],   # Blue
        'ring': [1, 1, 0, 0.7]      # Yellow
    }
    
    # Set up camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=0,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Create a coordinate frame at the origin
    create_coordinate_frame()
    
    # Set a consistent scale factor for all transformations
    scale_factor = 1.1

    print("\n=== VISUALIZING HAND POSITIONS AND CALCULATING IK ===")
    
    # Create a dictionary to store visualization results
    visualization_log = {
        "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "frames": {}
    }
    
    # Process each frame
    for frame_number in available_frames:
        frame_data = get_frame_data(log_data, frame_number)
        if frame_data is None:
            continue
        
        # Get all finger positions from the log
        finger_positions = frame_data["finger_input_positions"]
        
        # Create or update markers for all finger positions
        for finger, positions in finger_positions.items():
            if finger not in finger_markers:
                finger_markers[finger] = []
                
            # Create new markers if needed
            while len(finger_markers[finger]) < len(positions):
                marker_id = create_visual_marker([0, 0, 0], finger_colors[finger], size=0.01)
                finger_markers[finger].append(marker_id)
            
            # Update all marker positions
            for i, pos in enumerate(positions):
                if i < len(finger_markers[finger]):
                    # Apply the same transformation as in compute_hand_pos
                    pos_transformed = transform_position(pos, scale_factor, finger == 'ring')
                    
                    update_visual_marker_position(finger_markers[finger][i], pos_transformed)
        
        # Compute target positions for IK
        target_positions = compute_hand_pos(frame_data, scale=scale_factor)
        
        print(f"\nFrame {frame_number}: Calculating IK for target positions")
        
        # Calculate IK using our function
        calculated_joint_angles = compute_ik(
            hand_id, 
            target_positions, 
            leap_end_effector_indices
        )
        
        print(f"Calculated joint angles: {calculated_joint_angles[:4]} (index), "
              f"{calculated_joint_angles[4:8]} (middle), "
              f"{calculated_joint_angles[8:12]} (ring), "
              f"{calculated_joint_angles[12:16]} (thumb)")
        
        # Store frame data in visualization log
        visualization_log["frames"][str(frame_number)] = {
            "target_positions": target_positions,
            "calculated_joint_angles": calculated_joint_angles
        }
        
        # Wait for user to press Enter to continue to next frame
        input(f"Frame {frame_number}/{available_frames[-1]} - Press Enter to continue...")
    
    print("\n=== VISUALIZATION COMPLETE ===")
    
    # Save visualization log
    output_log_path = f"logs/ik_visualization_output_{visualization_log['timestamp']}.json"
    # save_visualization_log(visualization_log, output_log_path)
    
    # Keep the simulation running until user closes it
    while p.isConnected():
        time.sleep(0.1)

if __name__ == "__main__":
    main()
