#!/usr/bin/env python3
"""
Script to visualize recorded joint movements of the LEAP hand in PyBullet
and optionally replay them on the physical hand.
"""
import pybullet as p
import time
import os
import argparse
import sys
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from dataclasses import dataclass
import json

# Import shared components from visualize_ik_pybullet.py
try:
    from logs.leap_hand.visualize_ik_pybullet import (
        LogHandler, VisualMarkerManager, HandVisualizer, CoordinateTransformer, JointMapUtility
    )
except ImportError:
    # Fallback for direct execution
    sys.path.append(os.getcwd())
    from logs.leap_hand.visualize_ik_pybullet import (
        LogHandler, VisualMarkerManager, HandVisualizer, CoordinateTransformer, JointMapUtility
    )

# Import DexArmControl for physical hand control
try:
    from openteach.ros_links.leap_control import DexArmControl
except ImportError:
    print("Warning: DexArmControl not available. Physical hand replay will not be possible.")
    DexArmControl = None


@dataclass
class PlaybackSettings:
    """Settings for joint playback visualization and physical replay."""
    log_file: str
    use_physical_hand: bool = False
    loop_playback: bool = True
    initial_speed: float = 1.0
    delay_between_frames: float = 0.03
    home_on_start: bool = True
    home_on_exit: bool = True
    show_discrepancies: bool = True
    discrepancy_threshold: float = 0.05  # Threshold in radians to highlight discrepancies


class JointPlaybackVisualizer:
    """Class for replaying finger movements from log files in PyBullet and optionally on physical hand."""
    
    def __init__(self, urdf_path: str, settings: PlaybackSettings):
        """Initialize the joint playback visualizer.
        
        Args:
            urdf_path: Path to the LEAP hand URDF file
            settings: Playback settings
        """
        self.urdf_path = urdf_path
        self.settings = settings
        self.hand_id = None
        self.hand_visualizer = HandVisualizer(urdf_path)
        self.log_handler = LogHandler()
        self.physical_hand = None
        self.marker_manager = None
        self.joint_discrepancies = {}  # Store joint discrepancies for visualization
        self.visualization_ids = {}  # Store visualization IDs for updating
        
        # Initialize physical hand if requested
        if settings.use_physical_hand and DexArmControl:
            try:
                print("Initializing physical LEAP hand...")
                self.physical_hand = DexArmControl()
                if settings.home_on_start:
                    print("Homing the hand...")
                    self.physical_hand.home_hand()
                    time.sleep(2)  # Wait for homing to complete
            except Exception as e:
                print(f"Error initializing physical hand: {e}")
                self.physical_hand = None
    
    def __del__(self):
        """Clean up resources when the object is destroyed."""
        if self.physical_hand and self.settings.home_on_exit:
            try:
                print("Homing the hand before exit...")
                self.physical_hand.home_hand()
                time.sleep(2)
                self.physical_hand.close()
            except Exception as e:
                print(f"Error during cleanup: {e}")
    
    def send_to_physical_hand(self, joint_angles: List[float]) -> bool:
        """Send joint angles to the physical LEAP hand.
        
        Args:
            joint_angles: List of joint angles to send
            
        Returns:
            True if successful, False otherwise
        """
        if not self.physical_hand:
            return False
            
        try:
            # Convert to numpy array for DexArmControl
            angles_np = np.array(joint_angles, dtype=np.float32)
            self.physical_hand.move_hand(angles_np)
            return True
        except Exception as e:
            print(f"Error sending joint angles to physical hand: {e}")
            return False
    
    def read_from_physical_hand(self) -> Optional[List[float]]:
        """Read current joint angles from the physical hand.
        
        Returns:
            List of joint angles or None if reading failed
        """
        if not self.physical_hand:
            return None
            
        try:
            # Get current joint positions from physical hand
            joint_state = self.physical_hand.get_hand_state()
            
            # Handle the specific dictionary format {'position': [...], 'velocity': [...], 'effort': [...]}
            if isinstance(joint_state, dict) and 'position' in joint_state:
                positions = joint_state['position']
                
                # Convert to list if it's numpy array or other sequence
                if hasattr(positions, 'tolist'):
                    positions = positions.tolist()
                else:
                    positions = list(positions)
                    
                return positions
            
            # If we couldn't extract positions properly
            print("Warning: Could not extract positions from joint state")
            return None
        except Exception as e:
            print(f"Error reading joint angles from physical hand: {e}")
            return None
    
    def check_joint_discrepancies(self, commanded: List[float], actual: List[float]) -> Dict[int, float]:
        """Compare commanded and actual joint angles to detect discrepancies.
        
        Args:
            commanded: Commanded joint angles
            actual: Actual joint angles from physical hand
            
        Returns:
            Dictionary mapping joint index to discrepancy value (in radians)
        """
        # Ensure both inputs are lists or arrays that support slicing
        commanded_list = list(commanded)
        
        # Check if actual is a dictionary (common format from ROS)
        if isinstance(actual, dict):
            # Try to convert from dictionary to list based on joint indices
            try:
                max_idx = max(int(k) for k in actual.keys() if isinstance(k, (str, int)))
                actual_list = [actual.get(str(i), 0.0) for i in range(max_idx + 1)]
            except ValueError:
                print("Warning: Could not convert joint state dictionary to list")
                return {}
        else:
            # If already a list-like object, convert to standard list
            try:
                actual_list = list(actual)
            except (TypeError, ValueError):
                print(f"Warning: Joint state has unsupported type: {type(actual)}")
                return {}
        
        if len(commanded_list) != len(actual_list):
            print(f"Warning: Commanded ({len(commanded_list)}) and actual ({len(actual_list)}) joint angle counts don't match")
            # Use the smaller length to avoid index errors
            length = min(len(commanded_list), len(actual_list))
            commanded_list = commanded_list[:length]
            actual_list = actual_list[:length]
        
        discrepancies = {}
        for i, (cmd, act) in enumerate(zip(commanded_list, actual_list)):
            diff = abs(cmd - act)
            # Normalize angle difference for revolute joints
            if diff > np.pi:
                diff = 2 * np.pi - diff
            
            # Record if discrepancy is above threshold
            if diff > self.settings.discrepancy_threshold:
                discrepancies[i] = diff
        
        return discrepancies
    
    def visualize_discrepancies(self, letter_to_ik_idx: Dict[str, int], letter_joints: Dict[str, int]) -> None:
        """Visualize joint angle discrepancies in PyBullet.
        
        Args:
            letter_to_ik_idx: Mapping from joint letter to IK index
            letter_joints: Mapping from joint letter to PyBullet joint index
        """
        if not self.joint_discrepancies or not self.settings.show_discrepancies:
            return
        
        # Only display text for significant discrepancies (>5 degrees)
        significant_discrepancies = {k:v for k,v in self.joint_discrepancies.items() 
                                    if v > 0.09}  # ~5 degrees
        
        if not significant_discrepancies:
            return
        
        # Create minimal text showing only significant discrepancies
        discrepancy_text = f"{len(significant_discrepancies)} Joint Discrepancies:\n"
        
        # Map IK indices back to joint letters
        ik_idx_to_letter = {v: k for k, v in letter_to_ik_idx.items()}
        
        # Only visualize the most significant discrepancies (limit to top 3)
        top_discrepancies = sorted(significant_discrepancies.items(), 
                                   key=lambda x: x[1], reverse=True)[:3]
        
        for ik_idx, discrepancy in top_discrepancies:
            joint_letter = ik_idx_to_letter.get(ik_idx, f"Unknown-{ik_idx}")
            discrepancy_text += f"{joint_letter.upper()}: {np.degrees(discrepancy):.1f}° | "
            
            # Highlight only the most significant discrepancies
            if joint_letter in letter_joints:
                joint_idx = letter_joints[joint_letter]
                # Red for high discrepancy
                color = [1, 0, 0, 1]
                p.changeVisualShape(self.hand_id, joint_idx, rgbaColor=color)
        
        # Display simplified text
        if 'discrepancy_text' in self.visualization_ids:
            p.removeUserDebugItem(self.visualization_ids['discrepancy_text'])
        
        self.visualization_ids['discrepancy_text'] = p.addUserDebugText(
            discrepancy_text.strip(' | '),  # Remove trailing separator
            [0, 0, 0.40],
            textColorRGB=[1, 0, 0],  # Red for visibility
            textSize=1.0
        )
    
    def reset_joint_colors(self) -> None:
        """Reset all joint colors to default."""
        for i in range(p.getNumJoints(self.hand_id)):
            p.changeVisualShape(
                self.hand_id,
                i,
                rgbaColor=[0.8, 0.8, 0.8, 1.0]  # Light gray instead of pure white
            )
    
    def replay_finger_movements(self) -> None:
        """Replay finger movements from the log file in PyBullet and optionally on physical hand."""
        print("=== FINGER MOVEMENT REPLAY TOOL ===")
        
        # Load the log file
        log_data = self.log_handler.load_log(self.settings.log_file)
        
        # Setup PyBullet and load the hand model
        if not self.hand_visualizer.setup_environment():
            print(f"Error: Failed to set up environment")
            return
            
        self.hand_id = self.hand_visualizer.hand_id
        self.marker_manager = VisualMarkerManager()
        
        # Print information about joints and get the mapping
        letter_joints = JointMapUtility.get_letter_to_joint_mapping(self.hand_id)
        JointMapUtility.print_joint_mapping(self.hand_id)  # For display
        
        # Get letter to IK index mapping
        letter_to_ik_idx = JointMapUtility.get_letter_to_ik_index_mapping()
        
        # Get all available frames
        if "frames" not in log_data:
            print("No frames found in log.")
            return
        
        available_frames = sorted([int(k) for k in log_data["frames"].keys()])
        print(f"Found {len(available_frames)} frames.")
        
        # Detect joint angle field name in the first available frame
        if available_frames:
            first_frame_data = self.log_handler.get_frame_data(log_data, available_frames[0])
            joint_angle_field = None
            
            # Check if this is a finger_states log or calculated_joint_angles log
            if "finger_states" in first_frame_data:
                joint_angle_field = "finger_states"
                print("Found 'finger_states' field in log data")
            elif "calculated_joint_angles" in first_frame_data:
                joint_angle_field = "calculated_joint_angles"
                print("Found 'calculated_joint_angles' field in log data")
            else:
                # Also check nested fields like 'visualization_data'
                if "visualization_data" in first_frame_data:
                    vis_data = first_frame_data["visualization_data"]
                    if isinstance(vis_data, dict):
                        if "calculated_joint_angles" in vis_data:
                            joint_angle_field = "visualization_data.calculated_joint_angles"
                            print("Found nested 'calculated_joint_angles' field in log data")
            
            if not joint_angle_field:
                print("Warning: Could not find joint angle data in log. The format may not be supported.")
        
        # Add UI controls for PyBullet visualization
        playback_speed = self.settings.initial_speed
        speed_slider_id = p.addUserDebugParameter("Playback Speed", 0.1, 10.0, playback_speed)
        
        # Add a frame slider to directly jump to frames
        frame_slider_id = p.addUserDebugParameter("Frame", 0, len(available_frames)-1, 0)
        
        # Add play/pause button (0 = paused, 1 = playing)
        play_button_id = p.addUserDebugParameter("Play/Pause", 0, 1, 1)
        last_play_button_value = 1  # Start playing
        
        # Add physical hand toggle button
        physical_button_id = None
        if DexArmControl:
            physical_button_id = p.addUserDebugParameter(
                "Send to Physical Hand", 0, 1, 1 if self.settings.use_physical_hand else 0
            )
            last_physical_button_value = 1 if self.settings.use_physical_hand else 0
        
        # Add frameskip slider to reduce processing load
        frameskip_slider_id = p.addUserDebugParameter("Frame Skip", 0, 5, 0)
        frameskip = 0
        
        # Add discrepancy visualization toggle button
        discrepancy_button_id = p.addUserDebugParameter(
            "Show Discrepancies", 0, 1, 0 if self.settings.show_discrepancies else 0  # Default OFF
        )
        last_discrepancy_button_value = 0  # Default OFF
        
        # Add discrepancy threshold slider
        threshold_slider_id = p.addUserDebugParameter("Discrepancy Threshold (deg)", 1, 20, 
                                                      np.degrees(self.settings.discrepancy_threshold))
        last_threshold_value = np.degrees(self.settings.discrepancy_threshold)
        
        # Status variables
        is_playing = True
        current_frame_idx = 0
        last_timestamp = time.time()
        last_frame_slider_value = 0
        
        # Add text display for controls
        controls_text = """
        Controls:
        - Space: Play/Pause
        - Left/Right: Previous/Next Frame
        - Esc: Exit
        - Enter: Step one frame when paused
        """
        p.addUserDebugText(controls_text, [0, 0, 0.30], textColorRGB=[0, 1, 0], textSize=1.2)
        
        try:
            while True:
                # Process key events
                keys = p.getKeyboardEvents()
                
                # Check for exit (Esc key)
                if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
                    print("\nExiting replay...")
                    break
                
                # Toggle play/pause (Spacebar)
                if 32 in keys and keys[32] & p.KEY_WAS_TRIGGERED:
                    is_playing = not is_playing
                    last_timestamp = time.time()  # Reset timer
                    print(f"Playback {'resumed' if is_playing else 'paused'}")
                
                # Next frame (Right arrow or 'n')
                if (124 in keys and keys[124] & p.KEY_WAS_TRIGGERED) or (110 in keys and keys[110] & p.KEY_WAS_TRIGGERED):
                    if not is_playing:
                        current_frame_idx = min(current_frame_idx + 1, len(available_frames) - 1)
                        last_timestamp = time.time()
                        print(f"Manual advance to frame {current_frame_idx}")
                
                # Previous frame (Left arrow or 'p')
                if (123 in keys and keys[123] & p.KEY_WAS_TRIGGERED) or (112 in keys and keys[112] & p.KEY_WAS_TRIGGERED):
                    if not is_playing:
                        current_frame_idx = max(current_frame_idx - 1, 0)
                        last_timestamp = time.time()
                        print(f"Manual back to frame {current_frame_idx}")
                
                # Read UI controls
                playback_speed = p.readUserDebugParameter(speed_slider_id)
                
                # Check play/pause button
                play_button_value = p.readUserDebugParameter(play_button_id)
                if play_button_value != last_play_button_value:
                    is_playing = play_button_value > 0.5
                    last_play_button_value = play_button_value
                    last_timestamp = time.time()
                    print(f"Playback {'resumed' if is_playing else 'paused'}")
                
                # Check frameskip slider
                frameskip = int(p.readUserDebugParameter(frameskip_slider_id))
                
                # Check discrepancy visualization toggle
                discrepancy_button_value = p.readUserDebugParameter(discrepancy_button_id)
                if discrepancy_button_value != last_discrepancy_button_value:
                    self.settings.show_discrepancies = discrepancy_button_value > 0.5
                    last_discrepancy_button_value = discrepancy_button_value
                    if not self.settings.show_discrepancies:
                        self.reset_joint_colors()
                        # Remove discrepancy text if present
                        if 'discrepancy_text' in self.visualization_ids:
                            p.removeUserDebugItem(self.visualization_ids['discrepancy_text'])
                            self.visualization_ids.pop('discrepancy_text')
                
                # Check discrepancy threshold slider
                threshold_deg = p.readUserDebugParameter(threshold_slider_id)
                if abs(threshold_deg - last_threshold_value) > 0.1:  # Avoid tiny changes
                    self.settings.discrepancy_threshold = np.radians(threshold_deg)
                    last_threshold_value = threshold_deg
                    print(f"Discrepancy threshold set to {threshold_deg:.1f}° ({self.settings.discrepancy_threshold:.3f} rad)")
                
                # Check physical hand toggle
                if physical_button_id is not None:
                    physical_button_value = p.readUserDebugParameter(physical_button_id)
                    if physical_button_value != last_physical_button_value:
                        should_use_physical = physical_button_value > 0.5
                        if should_use_physical and not self.physical_hand:
                            try:
                                print("Initializing physical LEAP hand...")
                                self.physical_hand = DexArmControl()
                                if self.settings.home_on_start:
                                    print("Homing the hand...")
                                    self.physical_hand.home_hand()
                                    time.sleep(2)
                            except Exception as e:
                                print(f"Error initializing physical hand: {e}")
                                should_use_physical = False
                        
                        last_physical_button_value = physical_button_value
                        self.settings.use_physical_hand = should_use_physical
                        print(f"Physical hand relay {'enabled' if should_use_physical else 'disabled'}")
                
                # Check frame slider
                frame_slider_value = int(p.readUserDebugParameter(frame_slider_id))
                if frame_slider_value != last_frame_slider_value:
                    last_frame_slider_value = frame_slider_value
                    current_frame_idx = frame_slider_value
                    last_timestamp = time.time()
                
                # Get the current frame
                if is_playing or frame_slider_value != last_frame_slider_value or (13 in keys and keys[13] & p.KEY_WAS_TRIGGERED):
                    if current_frame_idx < len(available_frames):
                        frame_number = available_frames[current_frame_idx]
                        frame_data = self.log_handler.get_frame_data(log_data, frame_number)
                        
                        if frame_data:
                            # Get joint angles based on detected field
                            joint_angles = []
                            
                            # Try to find joint angles in different possible locations
                            if "finger_states" in frame_data:
                                joint_angles = frame_data["finger_states"]
                            
                            # If not found, try calculated_joint_angles
                            if not joint_angles and "calculated_joint_angles" in frame_data:
                                joint_angles = frame_data["calculated_joint_angles"]
                            
                            # If not found, check if it's nested in visualization_data
                            if not joint_angles and "visualization_data" in frame_data:
                                vis_data = frame_data["visualization_data"]
                                if isinstance(vis_data, dict) and "calculated_joint_angles" in vis_data:
                                    joint_angles = vis_data["calculated_joint_angles"]
                            
                            if joint_angles:
                                # Set joint positions in PyBullet
                                for letter, joint_idx in letter_joints.items():
                                    if letter in letter_to_ik_idx:
                                        ik_idx = letter_to_ik_idx[letter]
                                        if ik_idx < len(joint_angles):
                                            p.resetJointState(self.hand_id, joint_idx, joint_angles[ik_idx])
                                
                                # Only do physical hand processing every (frameskip+1) frames
                                if self.settings.use_physical_hand and current_frame_idx % (frameskip+1) == 0:
                                    if self.send_to_physical_hand(joint_angles):
                                        # Read back actual joint positions only if we need them for visualization
                                        if self.settings.show_discrepancies:
                                            actual_joint_angles = self.read_from_physical_hand()
                                            if actual_joint_angles is not None:
                                                self.joint_discrepancies = self.check_joint_discrepancies(
                                                    joint_angles, actual_joint_angles
                                                )
                                                
                                                # Only visualize if significant discrepancies found
                                                if self.joint_discrepancies:
                                                    self.visualize_discrepancies(letter_to_ik_idx, letter_joints)
                            else:
                                print(f"No joint angle data found for frame {frame_number}")
                    
                    # Display current frame info
                    p.addUserDebugText(
                        f"Frame: {current_frame_idx}/{len(available_frames)-1} | Speed: {playback_speed:.1f}x | " + 
                        f"{'PLAYING' if is_playing else 'PAUSED'} | " +
                        f"Physical hand: {'ON' if self.settings.use_physical_hand else 'OFF'} | " +
                        f"Discrepancies: {'SHOWN' if self.settings.show_discrepancies else 'HIDDEN'}", 
                        [0, 0, 0.20], 
                        textColorRGB=[1, 1, 0], 
                        textSize=1.5,
                        replaceItemUniqueId=10000  # Using a fixed ID to update the same text
                    )
                    
                    # Advance to next frame if playing
                    if is_playing:
                        current_time = time.time()
                        time_passed = current_time - last_timestamp
                        
                        # Control frame rate based on playback speed
                        if time_passed >= (1.0 / (30.0 * playback_speed)):  # Aim for 30fps when speed=1.0
                            current_frame_idx += 1
                            last_timestamp = current_time
                            
                            # Loop back to the beginning if requested
                            if current_frame_idx >= len(available_frames):
                                if self.settings.loop_playback:
                                    current_frame_idx = 0
                                    print("Looping back to beginning")
                                else:
                                    current_frame_idx = len(available_frames) - 1
                                    is_playing = False
                                    print("Reached end of playback")
                
                # Step the simulation
                p.stepSimulation()
                time.sleep(0.01)  # Small sleep to prevent maxing out CPU
        
        except KeyboardInterrupt:
            print("\nReplay stopped by user")
        finally:
            # Clean up
            if self.physical_hand and self.settings.home_on_exit:
                try:
                    print("Homing the hand before exit...")
                    self.physical_hand.home_hand()
                    time.sleep(2)
                    self.physical_hand.close()
                    self.physical_hand = None
                except Exception as e:
                    print(f"Error during cleanup: {e}")
            
            # Disconnect from PyBullet
            p.disconnect()


def main() -> None:
    """Main function to run the joint playback visualizer."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Visualize and replay LEAP hand joint movements')
    parser.add_argument('--file', type=str, help='Path to log file (relative to logs/ directory)')
    parser.add_argument('--physical', action='store_true', help='Send joint angles to physical hand')
    parser.add_argument('--no-loop', action='store_true', help="Don't loop playback")
    parser.add_argument('--speed', type=float, default=5.0, help='Initial playback speed')
    parser.add_argument('--delay', type=float, default=0.05, help='Delay between frames for physical hand (seconds)')
    parser.add_argument('--no-home-start', action='store_true', help="Don't home the hand at start")
    parser.add_argument('--no-home-exit', action='store_true', help="Don't home the hand at exit")
    parser.add_argument('--show-discrepancy', action='store_true', help="Don't show joint discrepancies")
    parser.add_argument('--threshold', type=float, default=5.0, 
                       help="Discrepancy threshold in degrees (default: 5.0)")
    args = parser.parse_args()
    
    print("=== JOINT PLAYBACK VISUALIZER ===")
    
    # Find the URDF path
    urdf_path = os.path.join(os.getcwd(), "openteach/components/environment/assets/urdf/leap_hand/leap_hand_right.urdf")
    if not os.path.exists(urdf_path):
        print(f"URDF not found at: {urdf_path}")
        fallback_path = "models/leap_hand/robots/leap_hand.urdf"  # Fallback path
        print(f"Trying fallback path: {fallback_path}")
        urdf_path = fallback_path
    
    # Get log file
    log_file = args.file
    if not log_file:
        log_file = input("Enter path to log file (from logs/ directory): ")
    
    if not log_file.startswith("logs/"):
        log_file = f"logs/{log_file}"
    
    print(f"Opening log file: {log_file}")
    
    if not os.path.exists(log_file):
        print(f"Error: Log file '{log_file}' not found")
        return
    
    # Create playback settings
    settings = PlaybackSettings(
        log_file=log_file,
        use_physical_hand=args.physical,
        loop_playback=not args.no_loop,
        initial_speed=args.speed,
        delay_between_frames=0.0,  # Set to zero to remove delay
        home_on_start=not args.no_home_start,
        home_on_exit=not args.no_home_exit,
        show_discrepancies=False,  # Default to OFF for better performance
        discrepancy_threshold=np.radians(args.threshold)
    )
    
    # Initialize visualizer and replay movements
    visualizer = JointPlaybackVisualizer(urdf_path, settings)
    visualizer.replay_finger_movements()


if __name__ == "__main__":
    main()