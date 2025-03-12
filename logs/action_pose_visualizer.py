import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import glob
import os
from datetime import datetime

def find_matching_log(timestamp_str, log_type="sim"):
    """Find the closest matching log file based on timestamp."""
    # Ensure we're looking in the logs directory
    base_dir = "logs"
    if not os.path.exists(base_dir):
        base_dir = "."  # Fallback to current directory if logs/ doesn't exist
    
    pattern = os.path.join(base_dir, f"{log_type}_log_*.json")
    print(f"Searching for files matching: {pattern}")  # Debug print
    
    log_files = glob.glob(pattern)
    print(f"Found files: {log_files}")  # Debug print
    
    target_time = datetime.strptime(timestamp_str, "%Y%m%d_%H%M%S")
    closest_file = None
    min_diff = float('inf')
    
    for file in log_files:
        try:
            # Extract timestamp from filename (e.g., sim_log_20250131_170340.json)
            timestamp = os.path.basename(file).split('_')[2:4]  # Get both date and time parts
            timestamp = '_'.join(timestamp).split('.')[0]  # Join them and remove .json
            log_time = datetime.strptime(timestamp, "%Y%m%d_%H%M%S")
            
            time_diff = abs((log_time - target_time).total_seconds())
            print(f"File: {file}, Time diff: {time_diff} seconds")  # Debug print
            
            if time_diff < min_diff:
                min_diff = time_diff
                closest_file = file
        except (IndexError, ValueError) as e:
            print(f"Error processing file {file}: {e}")  # Debug print
            continue
    
    if closest_file and min_diff < 60:  # Return None if no file within 1 minute
        print(f"Found matching {log_type} log: {closest_file}")
        return closest_file
    else:
        print(f"No matching {log_type} log found within 1 minute of {timestamp_str}")
        return None

def load_logs(operator_file):
    """Load operator log and find matching simulation log."""
    # Ensure we're looking in the logs directory
    if not operator_file.startswith('logs/'):
        operator_file = os.path.join('logs', operator_file)
    
    print(f"Loading operator file: {operator_file}")  # Debug print
    
    try:
        with open(operator_file, 'r') as f:
            operator_data = json.load(f)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        raise
    
    # Find and load matching sim log
    timestamp = operator_data["session_id"]
    print(f"Looking for sim log matching timestamp: {timestamp}")  # Debug print
    
    sim_file = find_matching_log(timestamp, "sim")
    if sim_file is None:
        raise ValueError(f"No matching sim log found for {timestamp}")
    
    with open(sim_file, 'r') as f:
        sim_data = json.load(f)
    
    return operator_data, sim_data

def matrix_to_euler_angles(rotation_matrix):
    """Convert rotation matrix to euler angles."""
    r = Rotation.from_matrix(rotation_matrix)
    return r.as_euler('xyz', degrees=True)

def plot_data(operator_data, sim_data, series="y", title="Trajectories"):
    """
    Plot pose data comparing end effector, action, and transformed poses.
    
    Args:
        operator_data: Dictionary containing operator log data with transformed poses
        sim_data: Dictionary containing simulation log data with end effector and action poses
        series: If 'y', show time series plots alongside 3D trajectory
        title: Plot title
    """
    # Extract trajectories
    frames = range(min(len(operator_data["frames"]), len(sim_data["frames"])))
    
    # Get transformed poses from operator log
    transformed_poses = []
    for i in frames:
        pose = operator_data["frames"][str(i)].get("transformed_pose")
        if pose:
            # Extract position and convert rotation matrix to quaternion
            pos = [pose[j][3] for j in range(3)]
            rot_matrix = np.array([pose[i][:3] for i in range(3)])
            quat = Rotation.from_matrix(rot_matrix).as_quat()
            transformed_poses.append(np.concatenate([pos, quat]))
    transformed_poses = np.array(transformed_poses)
    
    # Get end effector and action poses from sim log
    ee_trajectory = np.array([sim_data["frames"][str(i)]["end_effector_pose"] for i in frames])
    action_trajectory = np.array([sim_data["frames"][str(i)]["action_pose"] for i in frames])

    # Create plots
    if series == "y":
        fig = plt.figure(figsize=(15, 10))
        gs = fig.add_gridspec(3, 2)
        
        # Position plots
        for i, coord in enumerate(['X', 'Y', 'Z']):
            ax = fig.add_subplot(gs[i, 0])
            ax.plot(ee_trajectory[:, i], label='end effector', color='blue')
            ax.plot(action_trajectory[:, i], label='action', color='red', linestyle='--')
            ax.plot(transformed_poses[:, i], label='transformed', color='green', linestyle=':')
            ax.set_ylabel(coord)
            ax.legend()
        
        # 3D trajectory plot
        ax_3d = fig.add_subplot(gs[:, 1], projection='3d')
    else:
        fig = plt.figure(figsize=(8, 8))
        ax_3d = fig.add_subplot(111, projection='3d')

    # Plot 3D trajectories
    ax_3d.plot(ee_trajectory[:, 0], ee_trajectory[:, 1], ee_trajectory[:, 2],
              marker='o', markersize=2, label='end effector', color='blue')
    ax_3d.plot(action_trajectory[:, 0], action_trajectory[:, 1], action_trajectory[:, 2],
              marker='o', markersize=2, label='action', color='red', linestyle='--')
    ax_3d.plot(transformed_poses[:, 0], transformed_poses[:, 1], transformed_poses[:, 2],
              marker='o', markersize=2, label='transformed', color='green', linestyle=':')

    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.legend()
    ax_3d.set_title('3D Trajectory')

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

def main():
    operator_file = input("Operator log file: ")
    series = input("Series [y/n]: ")
    
    operator_data, sim_data = load_logs(operator_file)
    plot_data(operator_data, sim_data, series=series)

if __name__ == "__main__":
    main() 