import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
import os
import pybullet as p
import pybullet_data

class LeapHandVisualizer:
    """Visualizer for leap hand log files using PyBullet for kinematics."""
    
    def __init__(self, log_file):
        """Initialize the visualizer with a log file."""
        # Load the log file
        with open(log_file, 'r') as f:
            self.log_data = json.load(f)
        
        self.frames = self.log_data["frames"]
        self.frame_keys = sorted(self.frames.keys(), key=int)
        self.current_frame_idx = 0
        
        # Setup the figure
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.suptitle('Leap Hand Visualization', fontsize=16)
        
        # 3D plot for input finger positions
        self.ax1 = self.fig.add_subplot(131, projection='3d')
        self.ax1.set_title('Input Finger Positions')
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_zlabel('Z')
        
        # 3D plot for transformed hand model
        self.ax3 = self.fig.add_subplot(132, projection='3d')
        self.ax3.set_title('Transformed Hand Model')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        self.ax3.set_zlabel('Z')
        
        # 2D plot for joint angles
        self.ax2 = self.fig.add_subplot(133)
        self.ax2.set_title('Joint Angles')
        self.ax2.set_xlabel('Joint')
        self.ax2.set_ylabel('Angle (radians)')
        
        # Add slider for frame selection
        self.ax_slider = plt.axes([0.2, 0.02, 0.65, 0.03])
        self.slider = Slider(
            self.ax_slider, 'Frame', 0, len(self.frame_keys)-1,
            valinit=0, valstep=1
        )
        self.slider.on_changed(self.update_frame)
        
        # Add buttons for animation control
        self.ax_play = plt.axes([0.05, 0.02, 0.1, 0.03])
        self.btn_play = Button(self.ax_play, 'Play')
        self.btn_play.on_clicked(self.play_animation)
        
        self.ax_stop = plt.axes([0.85, 0.02, 0.1, 0.03])
        self.btn_stop = Button(self.ax_stop, 'Stop')
        self.btn_stop.on_clicked(self.stop_animation)
        
        # Add a button for IK comparison
        self.ax_ik = plt.axes([0.45, 0.05, 0.1, 0.03])
        self.btn_ik = Button(self.ax_ik, 'Compare IK')
        self.btn_ik.on_clicked(self.compare_ik_button_clicked)
        
        # Initialize animation
        self.anim = None
        self.is_playing = False
        
        # Finger colors and names
        self.finger_colors = {
            'thumb': 'red',
            'index': 'green',
            'middle': 'blue',
            'ring': 'purple'
        }
        
        # LEAP hand joint configuration
        self.leap_joint_offsets = {
            'index': 0,
            'middle': 4,
            'ring': 8,
            'thumb': 12
        }
        self.leap_joints_per_finger = 4
        
        # Initialize PyBullet in DIRECT mode (no GUI)
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Get path to Leap Hand URDF - using the exact path you provided
        self.hand_urdf_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "VR_Teleoperation",
            "openteach",
            "components",
            "environment",
            "assets",
            "urdf",
            "leap_hand",
            "leap_hand_right.urdf"
        )
        
        # If the path doesn't exist, try a relative path
        if not os.path.exists(self.hand_urdf_path):
            print(f"URDF not found at {self.hand_urdf_path}, trying relative path...")
            self.hand_urdf_path = os.path.join(
                "VR_Teleoperation",
                "openteach",
                "components",
                "environment",
                "assets",
                "urdf",
                "leap_hand",
                "leap_hand_right.urdf"
            )
        
        # Add the URDF directory to the search path so PyBullet can find the meshes
        p.setAdditionalSearchPath(os.path.dirname(self.hand_urdf_path))
        
        try:
            # Load Leap Hand
            self.robot_id = p.loadURDF(self.hand_urdf_path, 
                                      basePosition=[0, 0, 0],
                                      useFixedBase=True)
            
            # Get joint information
            self.num_joints = p.getNumJoints(self.robot_id)
            print(f"Loaded LEAP hand with {self.num_joints} joints")
            
            # Map joint names to indices and get joint limits
            self.joint_indices = {}
            self.joint_names = {}
            self.joint_limits = {}
            
            print("\nJoint Information:")
            print("-" * 50)
            for i in range(self.num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                self.joint_names[i] = joint_name
                self.joint_indices[joint_name] = i
                self.joint_limits[i] = (joint_info[8], joint_info[9])  # Lower and upper limits
                
                print(f"Joint {i}:")
                print(f"  Name: {joint_name}")
                print(f"  Type: {joint_info[2]}")
                print(f"  Lower limit: {joint_info[8]}")
                print(f"  Upper limit: {joint_info[9]}")
                print("-" * 50)
            
            # Map finger joints
            self.finger_joints = {
                'thumb': [],
                'index': [],
                'middle': [],
                'ring': []
            }
            
            # Identify finger joints based on names
            for i, name in self.joint_names.items():
                if p.getJointInfo(self.robot_id, i)[2] != p.JOINT_FIXED:
                    if name in ['n', 'm', 'o', 'p']:  # Thumb joints
                        self.finger_joints['thumb'].append(i)
                    elif name in ['a', 'b', 'c', 'd']:  # Index joints
                        self.finger_joints['index'].append(i)
                    elif name in ['e', 'f', 'g', 'h']:  # Middle joints
                        self.finger_joints['middle'].append(i)
                    elif name in ['i', 'j', 'k', 'l']:  # Ring joints
                        self.finger_joints['ring'].append(i)
            
            print("\nFinger Joints:")
            for finger, joints in self.finger_joints.items():
                print(f"{finger}: {joints}")
            
            self.pybullet_loaded = True
            
        except Exception as e:
            print(f"Error loading URDF: {e}")
            self.pybullet_loaded = False
        
        # Initial plot
        self.update_plot(0)
    
    def update_plot(self, frame_idx):
        """Update the visualization for the given frame index."""
        # Clear current plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        # Set titles
        self.ax1.set_title('Input Finger Positions (Frame {})'.format(frame_idx))
        self.ax2.set_title('Joint Angles')
        self.ax3.set_title('Transformed Hand Model')
        
        # Get frame data
        frame_key = self.frame_keys[frame_idx]
        frame_data = self.frames[frame_key]
        
        # Plot finger positions
        if 'finger_input_positions' in frame_data:
            finger_positions = frame_data['finger_input_positions']
            
            # Plot each finger's positions
            for finger, positions in finger_positions.items():
                if positions and len(positions) > 0:
                    positions_array = np.array(positions)
                    x = positions_array[:, 0]
                    y = positions_array[:, 1]
                    z = positions_array[:, 2]
                    
                    # Plot the finger line
                    self.ax1.plot(x, y, z, color=self.finger_colors[finger], label=finger, linewidth=2)
                    
                    # Plot the finger points
                    self.ax1.scatter(x, y, z, color=self.finger_colors[finger], s=50)
            
            # Add legend
            self.ax1.legend()
            
            # Draw coordinate axes to help understand orientation
            all_points = []
            for finger, positions in frame_data['finger_input_positions'].items():
                if positions:
                    all_points.extend(positions)
            
            if all_points:
                center = np.mean(np.array(all_points), axis=0)
                
                # Draw coordinate axes
                axis_length = 0.03
                
                # X-axis (red)
                self.ax1.plot([center[0], center[0] + axis_length], 
                             [center[1], center[1]], 
                             [center[2], center[2]], 'r-', linewidth=2)
                self.ax1.text(center[0] + axis_length * 1.1, center[1], center[2], 'X', 
                             color='red', fontsize=10)
                
                # Y-axis (green)
                self.ax1.plot([center[0], center[0]], 
                             [center[1], center[1] + axis_length], 
                             [center[2], center[2]], 'g-', linewidth=2)
                self.ax1.text(center[0], center[1] + axis_length * 1.1, center[2], 'Y', 
                             color='green', fontsize=10)
                
                # Z-axis (blue)
                self.ax1.plot([center[0], center[0]], 
                             [center[1], center[1]], 
                             [center[2], center[2] + axis_length], 'b-', linewidth=2)
                self.ax1.text(center[0], center[1], center[2] + axis_length * 1.1, 'Z', 
                             color='blue', fontsize=10)
        
        # Plot joint angles
        if 'finger_states' in frame_data:
            angles = frame_data['finger_states']
            joint_indices = range(len(angles))
            
            # Plot the joint angles as a bar chart
            self.ax2.bar(joint_indices, angles, color='skyblue')
            
            # Add labels for each finger group
            finger_labels = ['Index', 'Middle', 'Ring', 'Thumb']
            for i, label in enumerate(finger_labels):
                self.ax2.text(i*4 + 1.5, max(angles) * 0.9, label, 
                             ha='center', va='center', fontsize=10)
            
            # Add grid for better readability
            self.ax2.grid(True, linestyle='--', alpha=0.7)
            
            # Set y-axis limits
            self.ax2.set_ylim(-0.5, max(2.0, max(angles) * 1.1))
            
            # Plot transformed hand model using PyBullet
            if self.pybullet_loaded:
                self.plot_transformed_hand_pybullet(angles)
        
        # Update the figure
        self.fig.canvas.draw_idle()
        
        # Set view angle
        self.ax1.view_init(elev=30, azim=120)
        
        # Set equal aspect ratio for better visualization
        self.ax1.set_box_aspect([1, 1, 1])
    
    def plot_transformed_hand_pybullet(self, joint_angles):
        """Plot a 3D model of the hand using PyBullet for forward kinematics."""
        # Set joint angles in PyBullet
        for finger, offset in self.leap_joint_offsets.items():
            for i in range(self.leap_joints_per_finger):
                joint_idx = offset + i
                if joint_idx < len(joint_angles):
                    angle = joint_angles[joint_idx]
                    
                    # Find the corresponding PyBullet joint
                    if i < len(self.finger_joints[finger]):
                        pb_joint_idx = self.finger_joints[finger][i]
                        p.resetJointState(self.robot_id, pb_joint_idx, angle)
        
        # Get link positions for visualization
        link_positions = {}
        for finger, joints in self.finger_joints.items():
            link_positions[finger] = []
            for joint_idx in joints:
                # Get link state
                link_state = p.getLinkState(self.robot_id, joint_idx)
                link_pos = link_state[0]  # Position of the link
                link_positions[finger].append(link_pos)
            
            # Add fingertip position
            if joints:
                last_joint_idx = joints[-1]
                # Get the child link of the last joint
                for i in range(self.num_joints):
                    joint_info = p.getJointInfo(self.robot_id, i)
                    if joint_info[16] == last_joint_idx:  # If parent link is the last joint
                        fingertip_state = p.getLinkState(self.robot_id, i)
                        link_positions[finger].append(fingertip_state[0])
                        break
        
        # Plot each finger
        for finger, positions in link_positions.items():
            if positions:
                positions_array = np.array(positions)
                x = positions_array[:, 0]
                y = positions_array[:, 1]
                z = positions_array[:, 2]
                
                # Plot the finger line
                self.ax3.plot(x, y, z, color=self.finger_colors[finger], label=finger, linewidth=2)
                
                # Plot the finger points
                self.ax3.scatter(x, y, z, color=self.finger_colors[finger], s=50)
        
        # Add palm as a reference point
        palm_pos = p.getLinkState(self.robot_id, 0)[0]  # Base link position
        self.ax3.scatter([palm_pos[0]], [palm_pos[1]], [palm_pos[2]], color='gray', s=100, marker='s')
        
        # Set view angle for better visualization
        self.ax3.view_init(elev=30, azim=-60)
        
        # Set equal aspect ratio
        self.ax3.set_box_aspect([1, 1, 1])
        
        # Add legend
        self.ax3.legend()
    
    def update_frame(self, val):
        """Callback for slider value change."""
        frame_idx = int(val)
        self.current_frame_idx = frame_idx
        self.update_plot(frame_idx)
    
    def animate(self, i):
        """Animation function."""
        if self.is_playing:
            self.current_frame_idx = (self.current_frame_idx + 1) % len(self.frame_keys)
            self.slider.set_val(self.current_frame_idx)
            self.update_plot(self.current_frame_idx)
        return []
    
    def play_animation(self, event):
        """Start the animation."""
        self.is_playing = True
        if self.anim is None:
            self.anim = animation.FuncAnimation(
                self.fig, self.animate, interval=100, blit=True)
    
    def stop_animation(self, event):
        """Stop the animation."""
        self.is_playing = False
    
    def show(self):
        """Show the visualization."""
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        plt.show()
    
    def __del__(self):
        """Clean up PyBullet when the object is deleted."""
        if hasattr(self, 'physics_client'):
            p.disconnect(self.physics_client)

    def compute_inverse_kinematics(self, finger_positions):
        """
        Compute inverse kinematics to find joint angles for given finger positions.
        
        Args:
            finger_positions: Dictionary mapping finger names to target positions
            
        Returns:
            List of joint angles for the LEAP hand
        """
        # Step the simulation to ensure the physics state is updated
        p.stepSimulation()
        
        # Transform coordinates for IK calculation
        transformed_finger_positions = {}
        for finger, positions in finger_positions.items():
            transformed_positions = []
            for pos in positions:
                # Mirror X axis for coordinate system matching
                transformed_pos = [-pos[0], pos[1], pos[2]]
                transformed_positions.append(transformed_pos)
            transformed_finger_positions[finger] = transformed_positions
        
        finger_positions = transformed_finger_positions
        print("Applied coordinate transformation for IK calculation")
        
        # Initialize the joint angles
        real_robot_hand_q = np.zeros(16)
        
        # For each finger, calculate joint angles based on finger shape
        fingers = ['index', 'middle', 'ring', 'thumb']
        
        for finger in fingers:
            if finger in finger_positions and len(finger_positions[finger]) >= 2:
                # Calculate finger direction and curvature
                points = np.array(finger_positions[finger])
                
                # Base to tip vector
                base_to_tip = points[-1] - points[0]
                length = np.linalg.norm(base_to_tip)
                
                # Calculate approximate curvature by comparing straight-line distance to segmented distance
                segment_length = 0
                for i in range(len(points)-1):
                    segment_length += np.linalg.norm(points[i+1] - points[i])
                
                # Curvature ratio: 1.0 means straight, higher values mean more curved
                curvature_ratio = segment_length / max(0.001, length)
                
                # Calculate how much the finger is curled (0 = straight, 1 = fully curled)
                curl_factor = min(1.0, (curvature_ratio - 1.0) * 2.0)
                
                # Calculate the angle between finger and palm plane (approximated as XY plane)
                normalized_direction = base_to_tip / max(0.001, length)
                downward_factor = -normalized_direction[2]  # Z component, negative means pointing down
                inward_factor = normalized_direction[1]     # Y component, positive means pointing inward
                
                print(f"{finger} curl: {curl_factor:.2f}, length: {length:.4f}, " +
                      f"downward: {downward_factor:.2f}, inward: {inward_factor:.2f}")
                
                # REFINED JOINT ANGLE MAPPING WITH OFFSETS
                # The offsets are tuned based on the recorded vs. IK solution comparison
                if finger == 'index':
                    # Add offsets to better match recorded angles
                    joint_indices = [0, 1, 2, 3]
                    # Base angles when finger is straight
                    base_angles = [0.8, 0.7, 0.2, 0.1]  
                    
                    for i, joint_idx in enumerate(joint_indices):
                        curl_weight = 1.5 - (i * 0.3)  # More curl in proximal joints
                        real_robot_hand_q[joint_idx] = base_angles[i] + (curl_factor * 1.5 * curl_weight)
                    
                elif finger == 'middle':
                    joint_indices = [4, 5, 6, 7]
                    # Adjusted base angles based on comparison
                    base_angles = [0.9, 0.7, 0.2, 0.1]  
                    
                    for i, joint_idx in enumerate(joint_indices):
                        curl_weight = 1.5 - (i * 0.3)
                        real_robot_hand_q[joint_idx] = base_angles[i] + (curl_factor * 1.5 * curl_weight)
                    
                elif finger == 'ring':
                    joint_indices = [8, 9, 10, 11]
                    # Adjusted base angles based on comparison
                    base_angles = [0.9, 0.7, 0.2, 0.1]  
                    
                    for i, joint_idx in enumerate(joint_indices):
                        curl_weight = 1.5 - (i * 0.3)
                        real_robot_hand_q[joint_idx] = base_angles[i] + (curl_factor * 1.7 * curl_weight)
                        
                elif finger == 'thumb':
                    joint_indices = [12, 13, 14, 15]
                    # Thumb behaves differently, requiring specific adjustments
                    base_angles = [0.9, 1.0, 0.5, 0.3]  
                    
                    for i, joint_idx in enumerate(joint_indices):
                        if i == 0:  # Opposition movement
                            opposition = 0.5 + inward_factor * 0.8
                            real_robot_hand_q[joint_idx] = base_angles[i] + opposition
                        elif i == 1:  # Side-to-side
                            side_bend = 0.6 + inward_factor * 0.9
                            real_robot_hand_q[joint_idx] = base_angles[i] + side_bend
                        else:  # Curling
                            curl_weight = 1.0 - (i * 0.2)
                            real_robot_hand_q[joint_idx] = base_angles[i] + (curl_factor * curl_weight * 1.2)
        
        # Get the final joint angles
        joint_angles = real_robot_hand_q.tolist()
        
        # Apply joint limits
        limited_joint_angles = self.apply_joint_limits(joint_angles)
        
        return limited_joint_angles

    def apply_joint_limits(self, joint_angles):
        """
        Ensure that joint angles stay within the defined limits.
        
        Args:
            joint_angles: List of joint angles to check
            
        Returns:
            List of joint angles clamped to their limits
        """
        limited_angles = joint_angles.copy()
        
        for i, angle in enumerate(joint_angles):
            # Find the corresponding joint in PyBullet
            pb_joint = None
            for j in range(self.num_joints):
                if p.getJointInfo(self.robot_id, j)[2] != p.JOINT_FIXED:  # Skip fixed joints
                    joint_name = p.getJointInfo(self.robot_id, j)[1].decode('utf-8')
                    # Match the joint to our index
                    if joint_name in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'n', 'm', 'o', 'p']:
                        finger_joint_map = {
                            'a': 0, 'b': 1, 'c': 2, 'd': 3,  # Index
                            'e': 4, 'f': 5, 'g': 6, 'h': 7,  # Middle
                            'i': 8, 'j': 9, 'k': 10, 'l': 11,  # Ring
                            'n': 12, 'm': 13, 'o': 14, 'p': 15  # Thumb
                        }
                        if finger_joint_map.get(joint_name) == i:
                            pb_joint = j
                            break
            
            if pb_joint is not None and pb_joint in self.joint_limits:
                lower_limit, upper_limit = self.joint_limits[pb_joint]
                # If limits are valid (not -inf/inf)
                if lower_limit > -9999 and upper_limit < 9999:
                    limited_angles[i] = max(lower_limit, min(upper_limit, angle))
                    if angle != limited_angles[i]:
                        print(f"Joint {i} angle {angle:.2f} clamped to {limited_angles[i]:.2f} (limits: {lower_limit:.2f}, {upper_limit:.2f})")
        
        return limited_angles

    def compare_ik_fk(self, frame_idx):
        """
        Compare inverse kinematics solution with the recorded joint angles.
        
        Args:
            frame_idx: Index of the frame to compare
            
        Returns:
            IK solution joint angles
        """
        frame_key = self.frame_keys[frame_idx]
        frame_data = self.frames[frame_key]
        
        if 'finger_input_positions' not in frame_data or 'finger_states' not in frame_data:
            print("Missing required data for IK comparison")
            return None
        
        # Get the recorded finger positions and joint angles
        finger_positions = frame_data['finger_input_positions']
        recorded_angles = frame_data['finger_states']
        
        # Compute IK solution
        ik_angles = self.compute_inverse_kinematics(finger_positions)
        
        if ik_angles is None:
            print("Failed to compute IK solution")
            return None
        
        # Create a new figure for comparison
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot the recorded angles and IK solution
        joint_indices = range(len(recorded_angles))
        
        ax1.bar(joint_indices, recorded_angles, width=0.4, label='Recorded Angles', alpha=0.7)
        ax1.bar([i+0.4 for i in joint_indices], ik_angles[:len(recorded_angles)], width=0.4, label='IK Solution', alpha=0.7)
        
        # Add finger group labels
        finger_labels = ['Index', 'Middle', 'Ring', 'Thumb']
        for i, label in enumerate(finger_labels):
            ax1.text(i*4 + 1.5, max(recorded_angles) * 0.9, label, ha='center', va='center', fontsize=10)
        
        ax1.set_title('Comparison of Recorded Angles vs. IK Solution')
        ax1.set_xlabel('Joint Index')
        ax1.set_ylabel('Angle (radians)')
        ax1.legend()
        ax1.grid(True, linestyle='--', alpha=0.7)
        
        # Plot the difference between recorded and IK angles
        differences = [ik - rec for ik, rec in zip(ik_angles[:len(recorded_angles)], recorded_angles)]
        ax2.bar(joint_indices, differences, color='orange')
        ax2.set_title('Difference between IK Solution and Recorded Angles')
        ax2.set_xlabel('Joint Index')
        ax2.set_ylabel('Difference (radians)')
        ax2.grid(True, linestyle='--', alpha=0.7)
        
        # Add a horizontal line at zero
        ax2.axhline(y=0, color='r', linestyle='-', alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Create a separate figure to visualize the hand model with IK solution
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('Hand Model with IK Solution')
        
        # Apply IK solution to the robot
        for i in range(min(len(ik_angles), 16)):
            # Find the corresponding joint in the robot
            for j in range(p.getNumJoints(self.robot_id)):
                joint_info = p.getJointInfo(self.robot_id, j)
                if joint_info[2] == p.JOINT_REVOLUTE:
                    try:
                        joint_num = int(joint_info[1].decode('utf-8'))
                        if joint_num == i:
                            p.resetJointState(self.robot_id, j, ik_angles[i])
                            break
                    except:
                        pass
        
        # Get link positions for visualization
        link_positions = {}
        for finger, joints in self.finger_joints.items():
            link_positions[finger] = []
            for joint_idx in joints:
                # Get link state
                link_state = p.getLinkState(self.robot_id, joint_idx)
                link_pos = link_state[0]  # Position of the link
                link_positions[finger].append(link_pos)
            
            # Add fingertip position
            if joints:
                last_joint_idx = joints[-1]
                # Get the child link of the last joint
                for i in range(self.num_joints):
                    joint_info = p.getJointInfo(self.robot_id, i)
                    if joint_info[16] == last_joint_idx:  # If parent link is the last joint
                        fingertip_state = p.getLinkState(self.robot_id, i)
                        link_positions[finger].append(fingertip_state[0])
                        break
        
        # Plot each finger
        for finger, positions in link_positions.items():
            if positions:
                positions_array = np.array(positions)
                x = positions_array[:, 0]
                y = positions_array[:, 1]
                z = positions_array[:, 2]
                
                # Plot the finger line
                ax.plot(x, y, z, color=self.finger_colors[finger], label=finger, linewidth=2)
                
                # Plot the finger points
                ax.scatter(x, y, z, color=self.finger_colors[finger], s=50)
        
        # Add palm as a reference point
        palm_pos = p.getLinkState(self.robot_id, 0)[0]  # Base link position
        ax.scatter([palm_pos[0]], [palm_pos[1]], [palm_pos[2]], color='gray', s=100, marker='s')
        
        # Set view angle for better visualization
        ax.view_init(elev=30, azim=-60)
        
        # Add legend
        ax.legend()
        
        plt.tight_layout()
        plt.show()
        
        return ik_angles

    def compare_ik_button_clicked(self, event):
        """Handle click on the Compare IK button."""
        ik_angles = self.compare_ik_fk(self.current_frame_idx)
        if ik_angles is not None:
            print("IK Solution:")
            print(ik_angles)

def main():
    # Ask for log file path via input
    log_file = input("Enter the path to the leap hand log file: ")
    
    log_file = f"logs/{log_file}"
    # Check if file exists
    if not os.path.exists(log_file):
        print(f"Error: File {log_file} does not exist")
        return
    
    visualizer = LeapHandVisualizer(log_file)
    visualizer.show()

if __name__ == "__main__":
    main() 