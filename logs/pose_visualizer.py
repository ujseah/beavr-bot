import json
import numpy as np
import matplotlib.pyplot as plt
import math


def load_pose_log(log_file):
    log_file = f'logs/{log_file}'
    with open(log_file, 'r') as f:
        return json.load(f)

def compute_pose_distance(pose1, pose2):
    """Compute distance between two poses (position and orientation)"""
    # Position distance (Euclidean)
    pos1 = np.array([pose1[i][3] for i in range(3)])
    pos2 = np.array([pose2[i][3] for i in range(3)])
    pos_dist = np.linalg.norm(pos1 - pos2)
    
    # Orientation distance (angular)
    rot1 = np.array([[pose1[i][j] for j in range(3)] for i in range(3)])
    rot2 = np.array([[pose2[i][j] for j in range(3)] for i in range(3)])
    rel_rot = rot1 @ rot2.T
    angle = np.arccos((np.trace(rel_rot) - 1) / 2)
    
    return pos_dist, angle

def homogeneous_matrix_to_euler_angles(T):
    """Convert homogeneous matrix to euler angles."""
    T = np.array(T)
    assert T.shape == (4, 4), "T must be a 4x4 homogeneous matrix."

    # Extract the rotation matrix
    R = T[:3, :3]

    # Pitch (θ)
    pitch = np.arcsin(-R[2, 0])

    # Yaw (ψ)
    if np.cos(pitch) < 0:
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        yaw = np.arctan2(-R[1, 0], -R[0, 0])

    # Roll (φ)
    if np.cos(pitch) < 0:
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        roll = np.arctan2(-R[2, 1], -R[2, 2])

    return np.array([roll, pitch, yaw])

def extract_trajectories(log_data, use_delta=False):
    """Extract trajectories with optional delta transformation."""
    hand_positions = []
    robot_positions = []
    hand_orientations = []
    robot_orientations = []
    
    # Define transformation matrices
    H_R_V = np.array([[-1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, -1, 0],
                      [0, 0, 0, 1]])
    H_T_V = H_R_V.copy()

    # Process each frame
    for frame in log_data:
        if use_delta:
            # Calculate delta transformations
            delta_t = np.array(frame["current_hand_pose"])[:3, 3] - np.array(frame["init_hand_pose"])[:3, 3]
            delta_r = np.linalg.pinv(np.array(frame['current_hand_pose']))[:3, :3] @ np.array(frame['init_hand_pose'])[:3, :3]
            hand_matrix = np.block([[delta_r, delta_t.reshape(3,1)], [0, 0, 0, 1]])

            delta_t = np.array(frame["transformed_pose"])[:3, 3] - np.array(frame["init_robot_pose"])[:3, 3]
            delta_r = np.linalg.pinv(np.array(frame['transformed_pose'])[:3, :3]) @ np.array(frame['init_robot_pose'])[:3, :3]
            robot_matrix = np.block([[delta_r, delta_t.reshape(3,1)], [0, 0, 0, 1]])
        else:
            hand_matrix = np.array(frame['current_hand_pose'])
            robot_matrix = np.array(frame['transformed_pose'])

        # Extract positions and orientations
        hand_positions.append(hand_matrix[:3, 3])
        robot_positions.append(robot_matrix[:3, 3])
        hand_orientations.append(homogeneous_matrix_to_euler_angles(hand_matrix))
        robot_orientations.append(homogeneous_matrix_to_euler_angles(robot_matrix))

    return (np.array(hand_positions), np.array(robot_positions), 
            np.array(hand_orientations), np.array(robot_orientations))

def analyze_frame_distances(log_data):
    """Analyze distances between consecutive frames"""
    distances = []
    angles = []
    
    for i in range(1, len(log_data)):
        prev_hand = np.array(log_data[i-1]['current_hand_pose'])
        curr_hand = np.array(log_data[i]['current_hand_pose'])
        pos_dist, angle = compute_pose_distance(prev_hand, curr_hand)
        distances.append(pos_dist)
        angles.append(angle)
    
    return np.array(distances), np.array(angles)

def matrix_to_euler_angles(mat: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to Euler XYZ angles."""
    cy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])
    singular = cy < 0.00001
    if not singular:
        roll = math.atan2(mat[2, 1], mat[2, 2])
        pitch = math.atan2(-mat[2, 0], cy)
        yaw = math.atan2(mat[1, 0], mat[0, 0])
    else:
        roll = math.atan2(-mat[1, 2], mat[1, 1])
        pitch = math.atan2(-mat[2, 0], cy)
        yaw = 0
    return np.array([roll, pitch, yaw])

def plot_data(data, delta, series, title):
    # Set marker size
    ms = '1'
    
    """Plot trajectory data with given parameters."""
    frames_data = [data["frames"][str(i)] for i in range(len(data["frames"]))]
    
    init_frame = 0
    end_frame = len(frames_data)

    robot_hm = []
    hand_hm = []

    for frame in frames_data:
        if delta == "y":
            delta_t = np.array(frame["current_hand_pose"])[:3, 3] - np.array(frame["init_hand_pose"])[:3, 3]
            delta_r = np.linalg.pinv(np.array(frame['current_hand_pose']))[:3, :3] @\
                                  np.array(frame['init_hand_pose'])[:3, :3]
            hand_hm.append(np.block([[delta_r, delta_t.reshape(3,1)], [0, 0, 0, 1]]))

            delta_t = np.array(frame["transformed_pose"])[:3, 3] - np.array(frame["init_robot_pose"])[:3, 3]
            delta_r = np.linalg.pinv(np.array(frame['transformed_pose']))[:3, :3] @\
                                  np.array(frame['init_robot_pose'])[:3, :3]
            robot_hm.append(np.block([[delta_r, delta_t.reshape(3,1)], [0, 0, 0, 1]]))
        else:
            hand_hm.append(np.array(frame['current_hand_pose']))
            robot_hm.append(np.array(frame['transformed_pose']))

    robot_homogeneous_matrices = robot_hm[init_frame:end_frame]
    robot_init_point = np.array(frames_data[init_frame]['init_robot_pose'])
    hand_homogeneous_matrices = hand_hm[init_frame:end_frame]
    hand_init_point = np.array(frames_data[init_frame]['init_hand_pose'])

    hand_trajectory_points = np.array([matrix[:3, 3] for matrix in hand_homogeneous_matrices])
    robot_trajectory_points = np.array([matrix[:3, 3] for matrix in robot_homogeneous_matrices])
    hand_euler_angles_points = [matrix_to_euler_angles(hm[:3, :3]) for hm in hand_homogeneous_matrices]
    robot_euler_angles_points = [matrix_to_euler_angles(hm[:3, :3]) for hm in robot_homogeneous_matrices]

    if series == "y":
        # Create a figure with 2 rows: position and orientation
        fig = plt.figure(figsize=(15, 10))
        gs = plt.GridSpec(2, 2, width_ratios=[2, 1], height_ratios=[1, 1])
        
        # Position plots (left)
        axs_pos = [plt.subplot(gs[0, 0])]
        axs_pos.append(plt.subplot(gs[1, 0]))
        
        # 3D plot (right)
        ax_3d = plt.subplot(gs[:, 1], projection='3d')

        x = range(end_frame)
        
        # Define colors for each axis
        axis_colors = {'x': 'red', 'y': 'green', 'z': 'blue'}
        
        # Position series plots
        for axis, label in [('x', 'X'), ('y', 'Y'), ('z', 'Z')]:
            color = axis_colors[axis]
            i = list(axis_colors.keys()).index(axis)
            # Solid line for hand
            axs_pos[0].plot(x, [p[i] for p in hand_trajectory_points], 
                          label=f'{label} hand', color=color, linestyle='-')
            # Dashed line for robot
            axs_pos[0].plot(x, [p[i] for p in robot_trajectory_points], 
                          label=f'{label} robot', color=color, linestyle='--')
        axs_pos[0].set_title(f'{"Δ" if delta == "y" else ""}Position')
        axs_pos[0].legend()
        axs_pos[0].grid(True)

        # Orientation series plots with consistent colors
        orientation_colors = {'roll': 'red', 'pitch': 'green', 'yaw': 'blue'}
        for i, angle in enumerate(['roll', 'pitch', 'yaw']):
            color = orientation_colors[angle]
            # Solid line for hand
            axs_pos[1].plot(x, [angles[i] for angles in hand_euler_angles_points], 
                          label=f'{angle} hand', color=color, linestyle='-')
            # Dashed line for robot
            axs_pos[1].plot(x, [angles[i] for angles in robot_euler_angles_points], 
                          label=f'{angle} robot', color=color, linestyle='--')
        axs_pos[1].set_title(f'{"Δ" if delta == "y" else ""}Orientation')
        axs_pos[1].legend()
        axs_pos[1].grid(True)

        # 3D trajectory plot
        # Solid line and markers for hand (blue)
        ax_3d.plot(hand_trajectory_points[:, 0], hand_trajectory_points[:, 1], 
                  hand_trajectory_points[:, 2], linestyle='-', marker='o', 
                  markersize=ms, color='blue', label='hand')
        # Start point (star marker)
        ax_3d.plot([hand_trajectory_points[0, 0]], [hand_trajectory_points[0, 1]], 
                  [hand_trajectory_points[0, 2]], marker='*', markersize=10, 
                  color='blue', label='hand start')
        # End point (square marker)
        ax_3d.plot([hand_trajectory_points[-1, 0]], [hand_trajectory_points[-1, 1]], 
                  [hand_trajectory_points[-1, 2]], marker='s', markersize=8, 
                  color='blue', label='hand end')

        # Dashed line and markers for robot (red)
        ax_3d.plot(robot_trajectory_points[:, 0], robot_trajectory_points[:, 1], 
                  robot_trajectory_points[:, 2], linestyle='--', marker='o', 
                  markersize=ms, color='red', label='robot')
        # Start point (star marker)
        ax_3d.plot([robot_trajectory_points[0, 0]], [robot_trajectory_points[0, 1]], 
                  [robot_trajectory_points[0, 2]], marker='*', markersize=10, 
                  color='red', label='robot start')
        # End point (square marker)
        ax_3d.plot([robot_trajectory_points[-1, 0]], [robot_trajectory_points[-1, 1]], 
                  [robot_trajectory_points[-1, 2]], marker='s', markersize=8, 
                  color='red', label='robot end')

        # Keep the initial points if delta is "n"
        if delta == "n":
            ax_3d.plot([hand_init_point[0, 3]], [hand_init_point[1, 3]], 
                    [hand_init_point[2, 3]], marker='x', markersize=ms, color='blue')
            ax_3d.plot([robot_init_point[0, 3]], [robot_init_point[1, 3]], 
                    [robot_init_point[2, 3]], marker='x', markersize=ms, color='red')

        ax_3d.legend()
        ax_3d.set_xlabel('X')
        ax_3d.set_ylabel('Y')
        ax_3d.set_zlabel('Z')
        ax_3d.set_title('3D Trajectory')

        plt.suptitle(title)
        plt.tight_layout()
        plt.show()
    else:
        # Just show the 3D trajectory plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Solid line for hand (blue)
        ax.plot(hand_trajectory_points[:, 0], hand_trajectory_points[:, 1], 
               hand_trajectory_points[:, 2], linestyle='-', marker='o', 
               markersize=ms, color='blue', label='hand')
        # Start point (star marker)
        ax.plot([hand_trajectory_points[0, 0]], [hand_trajectory_points[0, 1]], 
                  [hand_trajectory_points[0, 2]], marker='*', markersize=10, 
                  color='blue', label='hand start')
        # End point (square marker)
        ax.plot([hand_trajectory_points[-1, 0]], [hand_trajectory_points[-1, 1]], 
                  [hand_trajectory_points[-1, 2]], marker='s', markersize=8, 
                  color='blue', label='hand end')

        # Dashed line for robot (red)
        ax.plot(robot_trajectory_points[:, 0], robot_trajectory_points[:, 1], 
               robot_trajectory_points[:, 2], linestyle='--', marker='o', 
               markersize=ms, color='red', label='robot')
        # Start point (star marker)
        ax.plot([robot_trajectory_points[0, 0]], [robot_trajectory_points[0, 1]], 
                  [robot_trajectory_points[0, 2]], marker='*', markersize=10, 
                  color='red', label='robot start')
        # End point (square marker)
        ax.plot([robot_trajectory_points[-1, 0]], [robot_trajectory_points[-1, 1]], 
                  [robot_trajectory_points[-1, 2]], marker='s', markersize=8, 
                  color='red', label='robot end')

        # Keep the initial points if delta is "n"
        if delta == "n":
            ax.plot([hand_init_point[0, 3]], [hand_init_point[1, 3]], 
                [hand_init_point[2, 3]], marker='x', markersize=ms, color='blue')
            ax.plot([robot_init_point[0, 3]], [robot_init_point[1, 3]], 
                    [robot_init_point[2, 3]], marker='x', markersize=ms, color='red')

        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title(title)
        plt.show()

def main():
    file_path = input("Log file: ")
    delta = input("Delta [y/n]: ")
    series = input("Series [y/n]: ")

    data = load_pose_log(file_path)

    H_R_V = np.array([[-1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, -1, 0],
                     [0, 0, 0, 1]])
    H_T_V = H_R_V.copy()

    # Visualize original data
    plot_data(data, delta, series, "Original")

    # # Store original transformed poses
    # frames_data = [data["frames"][str(i)] for i in range(len(data["frames"]))]
    # data_transformed_pose_orig = [frame['transformed_pose'] for frame in frames_data]

    # # Transform data
    # last_init_hand = None
    # for i, frame in enumerate(frames_data):
    #     H_HI_HH = np.array(frame['init_hand_pose'])
    #     H_HT_HH = np.array(frame['current_hand_pose'])
    #     H_RI_RH = np.array(frame['init_robot_pose'])

    #     H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH
    #     H_HT_HI_r = (np.linalg.pinv(H_R_V) @ H_HT_HI @ H_R_V)[:3,:3]
    #     H_HT_HI_t = (np.linalg.pinv(H_T_V) @ H_HT_HI @ H_T_V)[:3,3]
    #     relative_affine = np.block([[H_HT_HI_r, H_HT_HI_t.reshape(3,1)], 
    #                               [0, 0, 0, 1]])
    #     H_RT_RH = H_RI_RH @ relative_affine
    #     data["frames"][str(i)]['transformed_pose'] = H_RT_RH.tolist()
        
    #     if last_init_hand is not None and (not np.array_equal(last_init_hand, H_HI_HH)):
    #         print(f'Jump at frame {i}')
    #     last_init_hand = H_HI_HH

    # # Save calculated data
    # calc_file_path = "calc_" + file_path
    # with open(calc_file_path, "w") as file:
    #     json.dump(data, file, indent=4)

    # # Visualize calculated data
    # plot_data(data, delta, series, "Calculated")

    # # Plot original vs calculated
    # for i in range(len(data["frames"])):
    #     data["frames"][str(i)]['init_hand_pose'] = data["frames"][str(i)]['init_robot_pose']
    #     data["frames"][str(i)]['current_hand_pose'] = data_transformed_pose_orig[i]
    # plot_data(data, delta, series, "Original vs Calculated")

if __name__ == "__main__":
    main()