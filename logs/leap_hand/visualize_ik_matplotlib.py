import os
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pybullet as p
import pybullet_data
from matplotlib.widgets import Slider, Button


class HandVisualizer:
    """Visualizer for leap hand IK solutions using PyBullet for kinematics."""
    
    def __init__(self, ik_file_path):
        # Load IK solutions
        self.ik_solutions = self.load_ik_solutions(ik_file_path)
        self.frames = self.ik_solutions["frames"]
        self.frame_keys = sorted([int(f) for f in self.frames.keys()])
        
        # Set up PyBullet
        self.physics_client = self.setup_pybullet()
        
        # Load hand model
        self.hand_id = self.load_hand_model()
        if self.hand_id is None:
            p.disconnect(self.physics_client)
            raise Exception("Failed to load hand model")
        
        # Get movable joint indices
        self.movable_joint_indices = []
        for i in range(p.getNumJoints(self.hand_id)):
            joint_info = p.getJointInfo(self.hand_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.movable_joint_indices.append(i)
        
        # Set up matplotlib figure with subplots
        self.fig = plt.figure(figsize=(15, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize animation state
        self.current_frame_idx = 0
        self.is_playing = False
        self.anim = None
        
        # Set up UI controls
        self.setup_controls()
        
        # Initial plot
        self.update_plot(self.current_frame_idx)
    
    def load_ik_solutions(self, filename):
        """Load IK solutions from a JSON file."""
        with open(filename, 'r') as f:
            return json.load(f)
    
    def setup_pybullet(self):
        """Set up PyBullet in DIRECT mode (no GUI)."""
        physicsClient = p.connect(p.DIRECT)  # Connect to PyBullet in DIRECT mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        return physicsClient
    
    def load_hand_model(self):
        """Load the Leap Hand URDF."""
        urdf_path = os.path.join(os.getcwd(), "openteach/components/environment/assets/urdf/leap_hand/leap_hand_right.urdf")
        if not os.path.exists(urdf_path):
            print("URDF not found at:", urdf_path)
            # Try alternative paths
            urdf_path = os.path.join("VR_Teleoperation", "openteach", "components", "environment", 
                                    "assets", "urdf", "leap_hand", "leap_hand_right.urdf")
            if not os.path.exists(urdf_path):
                print("URDF not found at alternative path either.")
                return None
        
        # Add the URDF directory to the search path so PyBullet can find the meshes
        p.setAdditionalSearchPath(os.path.dirname(urdf_path))
        
        hand_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
        return hand_id
    
    def setup_controls(self):
        """Set up UI controls for the visualization."""
        # Add slider for frame selection
        slider_ax = plt.axes([0.2, 0.02, 0.65, 0.03])
        self.slider = Slider(
            slider_ax, 'Frame', 0, len(self.frame_keys)-1,
            valinit=0, valstep=1
        )
        self.slider.on_changed(self.update_frame)
        
        # Add play/pause buttons
        play_ax = plt.axes([0.1, 0.02, 0.05, 0.03])
        self.play_button = Button(play_ax, 'Play')
        self.play_button.on_clicked(self.play_animation)
        
        pause_ax = plt.axes([0.85, 0.02, 0.05, 0.03])
        self.pause_button = Button(pause_ax, 'Pause')
        self.pause_button.on_clicked(self.stop_animation)
    
    def update_hand_joints(self, joint_angles):
        """Update hand joint angles."""
        # Get all movable joint indices
        movable_joints = []
        for i in range(p.getNumJoints(self.hand_id)):
            joint_info = p.getJointInfo(self.hand_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                movable_joints.append(i)
        
        # Apply joint angles to movable joints
        for i, joint_idx in enumerate(movable_joints):
            if i < len(joint_angles):
                p.resetJointState(self.hand_id, joint_idx, joint_angles[i])
    
    def get_finger_joint_positions(self):
        """Get positions of finger joints grouped by finger."""
        # Get joint information
        num_joints = p.getNumJoints(self.hand_id)
        joint_info = {}
        for i in range(num_joints):
            info = p.getJointInfo(self.hand_id, i)
            joint_info[i] = {
                'name': info[1].decode('utf-8'),
                'parent': info[16],
                'position': p.getLinkState(self.hand_id, i)[0] if i > 0 else p.getBasePositionAndOrientation(self.hand_id)[0]
            }
        
        # Base position
        base_pos = p.getBasePositionAndOrientation(self.hand_id)[0]
        
        # Group joints by finger based on name
        fingers = {
            'palm': [base_pos],
            'index': [],
            'middle': [],
            'ring': [],
            'thumb': []
        }
        
        # Map joints to fingers based on name
        for i in range(num_joints):
            name = joint_info[i]['name'].lower()
            pos = joint_info[i]['position']
            
            if 'index' in name:
                fingers['index'].append((i, pos))
            elif 'middle' in name:
                fingers['middle'].append((i, pos))
            elif 'ring' in name:
                fingers['ring'].append((i, pos))
            elif 'thumb' in name:
                fingers['thumb'].append((i, pos))
            elif i > 0:  # Skip base link
                fingers['palm'].append((i, pos))
        
        # Sort finger joints by their position along the finger
        for finger in ['index', 'middle', 'ring', 'thumb']:
            if fingers[finger]:
                fingers[finger].sort(key=lambda x: x[0])  # Sort by joint index
                fingers[finger] = [pos for _, pos in fingers[finger]]
        
        return fingers
    
    def plot_hand(self, finger_positions, target_positions):
        """Plot the hand with finger positions and target positions."""
        # Clear previous plot
        self.ax.clear()
        
        # Set up the plot
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Hand IK Visualization')
        
        # Set equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])
        
        # Define colors for each finger
        colors = {
            'index': 'blue',
            'middle': 'green',
            'ring': 'red',
            'thumb': 'purple'
        }
        
        # Get all joint positions for each finger
        finger_joint_positions = self.get_finger_joint_chains()
        
        # Plot each finger's joint chain
        for finger, joint_chain in finger_joint_positions.items():
            # Convert joint positions to arrays for plotting
            xs = [pos[0] for pos in joint_chain]
            ys = [pos[1] for pos in joint_chain]
            zs = [pos[2] for pos in joint_chain]
            
            # Plot the joint chain as a line
            self.ax.plot(xs, ys, zs, color=colors[finger], linewidth=2)
            
            # Plot joints as spheres
            for pos in joint_chain:
                self.ax.scatter(pos[0], pos[1], pos[2], color=colors[finger], s=30)
        
        # Plot target positions
        for i, pos in enumerate(target_positions):
            finger_idx = i // 2
            finger_name = ['index', 'middle', 'ring', 'thumb'][finger_idx]
            self.ax.scatter(pos[0], pos[1], pos[2], color='orange', marker='x', s=100)
        
        # Add a coordinate frame at the origin
        self.ax.quiver(0, 0, 0, 0.05, 0, 0, color='r')
        self.ax.quiver(0, 0, 0, 0, 0.05, 0, color='g')
        self.ax.quiver(0, 0, 0, 0, 0, 0.05, color='b')
        
        # Set axis limits
        self.ax.set_xlim([-0.1, 0.1])
        self.ax.set_ylim([-0.1, 0.1])
        self.ax.set_zlim([0, 0.2])
    
    def get_finger_joint_chains(self):
        """Get the positions of all joints in each finger chain."""
        # Define the joint names for each finger based on the URDF
        finger_joint_names = {
            'index': ['a', 'b', 'c', 'd'],  # Index finger joints
            'middle': ['e', 'f', 'g', 'h'], # Middle finger joints
            'ring': ['i', 'j', 'k', 'l'],   # Ring finger joints
            'thumb': ['m', 'n', 'o', 'p']   # Thumb joints
        }
        
        # Map joint names to indices
        joint_name_to_index = {}
        for i in range(p.getNumJoints(self.hand_id)):
            joint_info = p.getJointInfo(self.hand_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_name_to_index[joint_name] = i
        
        # Get the current frame data
        frame_data = self.frames[str(self.frame_keys[self.current_frame_idx])]
        
        # Check if we have calculated_joint_angles or joint_angles
        if "calculated_joint_angles" in frame_data and len(frame_data["calculated_joint_angles"]) > 0:
            # We have calculated joint angles from IK
            print(f"Frame {self.frame_keys[self.current_frame_idx]}: Using calculated joint angles")
            
            # Apply the calculated joint angles to the hand model
            finger_names = ["index", "middle", "ring", "thumb"]
            
            # Reset all joints to zero first
            for i in range(p.getNumJoints(self.hand_id)):
                if p.getJointInfo(self.hand_id, i)[2] == p.JOINT_REVOLUTE:
                    p.resetJointState(self.hand_id, i, 0.0)
            
            # Get the full list of calculated joint angles
            all_joint_angles = frame_data["calculated_joint_angles"]
            
            # Apply each finger's calculated angles
            for finger_idx, finger_name in enumerate(finger_names):
                # Calculate the slice indices for this finger's angles
                joints_per_finger = 4  # Each finger has 4 joints
                start_idx = finger_idx * joints_per_finger
                end_idx = start_idx + joints_per_finger
                
                # Get this finger's calculated angles as a slice
                if start_idx < len(all_joint_angles):
                    finger_angles = all_joint_angles[start_idx:end_idx]
                    
                    # Make sure finger_angles is a list
                    if not isinstance(finger_angles, list):
                        finger_angles = [finger_angles]
                    
                    # Apply angles to this finger's joints
                    for j in range(len(finger_angles)):
                        joint_idx_in_hand = self.movable_joint_indices[start_idx + j]
                        angle = finger_angles[j]
                        p.resetJointState(self.hand_id, joint_idx_in_hand, angle)
        elif "joint_angles" in frame_data:
            # Fallback to the old format if calculated_joint_angles is not available
            print(f"Frame {self.frame_keys[self.current_frame_idx]}: Using joint_angles (fallback)")
            joint_angles = frame_data["joint_angles"]
            
            # Update hand model
            self.update_hand_joints(joint_angles)
        else:
            print(f"Frame {self.frame_keys[self.current_frame_idx]}: No joint angles found, using zeros")
            # No joint angles found, use zeros
            for i in range(p.getNumJoints(self.hand_id)):
                if p.getJointInfo(self.hand_id, i)[2] == p.JOINT_REVOLUTE:
                    p.resetJointState(self.hand_id, i, 0.0)
        
        # Get the base position (palm)
        base_pos, _ = p.getBasePositionAndOrientation(self.hand_id)
        
        # Get joint positions for each finger
        finger_joint_chains = {}
        for finger, joint_names in finger_joint_names.items():
            chain = [base_pos]  # Start with the palm position
            
            # Get the link state for each joint in this finger
            for joint_name in joint_names:
                if joint_name in joint_name_to_index:
                    joint_idx = joint_name_to_index[joint_name]
                    link_state = p.getLinkState(self.hand_id, joint_idx)
                    chain.append(link_state[0])  # Position
            
            # Add fingertip position
            if finger == 'index':
                if 'index_tip' in joint_name_to_index:  # Changed from index_tip_head
                    tip_idx = joint_name_to_index['index_tip']
                    tip_state = p.getLinkState(self.hand_id, tip_idx)
                    chain.append(tip_state[0])
            elif finger == 'middle':
                if 'middle_tip' in joint_name_to_index:  # Changed from middle_tip_head
                    tip_idx = joint_name_to_index['middle_tip']
                    tip_state = p.getLinkState(self.hand_id, tip_idx)
                    chain.append(tip_state[0])
            elif finger == 'ring':
                if 'ring_tip' in joint_name_to_index:  # Changed from ring_tip_head
                    tip_idx = joint_name_to_index['ring_tip']
                    tip_state = p.getLinkState(self.hand_id, tip_idx)
                    chain.append(tip_state[0])
            elif finger == 'thumb':
                if 'thumb_tip' in joint_name_to_index:  # Changed from thumb_tip_head
                    tip_idx = joint_name_to_index['thumb_tip']
                    tip_state = p.getLinkState(self.hand_id, tip_idx)
                    chain.append(tip_state[0])
            
            finger_joint_chains[finger] = chain
        
        return finger_joint_chains
    
    def update_plot(self, frame_idx):
        """Update the plot for the given frame index."""
        self.current_frame_idx = frame_idx
        frame = str(self.frame_keys[frame_idx])
        
        # Get data for this frame
        frame_data = self.frames[frame]
        
        # Check if we have calculated_joint_angles
        if "calculated_joint_angles" in frame_data and len(frame_data["calculated_joint_angles"]) > 0:
            # We have calculated joint angles from IK
            print(f"Frame {frame}: Using calculated joint angles")
            
            # Apply the calculated joint angles to the hand model
            finger_names = ["index", "middle", "ring", "thumb"]
            
            # Reset all joints to zero first
            for i in range(p.getNumJoints(self.hand_id)):
                if p.getJointInfo(self.hand_id, i)[2] == p.JOINT_REVOLUTE:
                    p.resetJointState(self.hand_id, i, 0.0)
            
            # Get the full list of calculated joint angles
            all_joint_angles = frame_data["calculated_joint_angles"]
            
            # Apply each finger's calculated angles
            for finger_idx, finger_name in enumerate(finger_names):
                # Calculate the slice indices for this finger's angles
                joints_per_finger = 4  # Each finger has 4 joints
                start_idx = finger_idx * joints_per_finger
                end_idx = start_idx + joints_per_finger
                
                # Get this finger's calculated angles as a slice
                if start_idx < len(all_joint_angles):
                    finger_angles = all_joint_angles[start_idx:end_idx]
                    
                    # Make sure finger_angles is a list
                    if not isinstance(finger_angles, list):
                        finger_angles = [finger_angles]
                    
                    # Apply angles to this finger's joints
                    for j in range(len(finger_angles)):
                        joint_idx_in_hand = self.movable_joint_indices[start_idx + j]
                        angle = finger_angles[j]
                        p.resetJointState(self.hand_id, joint_idx_in_hand, angle)
        else:
            print(f"Frame {frame}: No calculated joint angles found, using zeros")
            # No joint angles found, use zeros
            for i in range(p.getNumJoints(self.hand_id)):
                if p.getJointInfo(self.hand_id, i)[2] == p.JOINT_REVOLUTE:
                    p.resetJointState(self.hand_id, i, 0.0)
        
        # Get finger positions
        finger_positions = self.get_finger_joint_positions()
        
        # Get target positions
        target_positions = frame_data["target_positions"]
        
        # Plot hand
        self.plot_hand(finger_positions, target_positions)
        
        # Update the figure
        self.fig.canvas.draw_idle()
    
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
            # update_plot is called by the slider callback
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

def main():
    # Get the latest IK solution file
    log_dir = "logs"
    ik_files = [f for f in os.listdir(log_dir) if f.startswith("ik_visualization_output_")]
    if not ik_files:
        print("No IK solution files found.")
        return
    
    latest_file = max(ik_files, key=lambda f: os.path.getmtime(os.path.join(log_dir, f)))
    ik_file_path = os.path.join(log_dir, latest_file)
    print(f"Using IK solution file: {ik_file_path}")
    
    # Create and show the visualizer
    visualizer = HandVisualizer(ik_file_path)
    visualizer.show()

if __name__ == "__main__":
    main()