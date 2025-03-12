#!/usr/bin/env python3
"""
Script to send pre-calculated IK solutions to the LEAP hand.
This loads joint angles from an IK visualization output file and
sends them to the physical hand using the DexArmControl interface.
"""

import os
import json
import numpy as np
import time
import argparse
from openteach.ros_links.leap_control import DexArmControl

def load_ik_solutions(filename):
    """Load IK solutions from a JSON file."""
    with open(filename, 'r') as f:
        return json.load(f)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Send IK solutions to LEAP hand')
    parser.add_argument('--file', type=str, help='Specific IK solution file to use')
    parser.add_argument('--delay', type=float, default=0.1, help='Delay between frames (seconds)')
    parser.add_argument('--loop', action='store_true', help='Loop the animation')
    args = parser.parse_args()
    
    # Get the IK solution file
    log_dir = "logs"
    if args.file:
        ik_file_path = os.path.join(log_dir, args.file)
        if not os.path.exists(ik_file_path):
            print(f"File not found: {ik_file_path}")
            return
    else:
        # Get the latest IK solution file
        ik_files = [f for f in os.listdir(log_dir) if f.startswith("ik_visualization_output_")]
        if not ik_files:
            print("No IK solution files found.")
            return
        
        latest_file = max(ik_files, key=lambda f: os.path.getmtime(os.path.join(log_dir, f)))
        ik_file_path = os.path.join(log_dir, latest_file)
    
    print(f"Using IK solution file: {ik_file_path}")
    
    # Load IK solutions
    ik_data = load_ik_solutions(ik_file_path)
    frames = ik_data["frames"]
    frame_keys = sorted([int(f) for f in frames.keys()])
    
    # Initialize the hand controller
    print("Initializing hand controller...")
    hand = DexArmControl()
    
    # Home the hand first
    print("Homing the hand...")
    hand.home_hand()
    time.sleep(2)  # Wait for the hand to reach home position
    
    try:
        # Process each frame
        loop_count = 0
        while True:
            if loop_count > 0 and not args.loop:
                break
                
            print(f"Starting playback (loop {loop_count+1})...")
            for frame_idx, frame_key in enumerate(frame_keys):
                frame = str(frame_key)
                frame_data = frames[frame]
                
                # Check if we have calculated joint angles
                if "calculated_joint_angles" in frame_data and len(frame_data["calculated_joint_angles"]) > 0:
                    # Get the joint angles
                    joint_angles = frame_data["calculated_joint_angles"]
                    
                    # Convert to numpy array if it's not already
                    joint_angles = np.array(joint_angles, dtype=np.float32)
                    
                    print(f"Frame {frame_idx+1}/{len(frame_keys)}: Sending joint angles to hand")
                    
                    # Send the joint angles to the hand
                    hand.move_hand(joint_angles)
                    
                    # Wait for the specified delay
                    time.sleep(args.delay)
                else:
                    print(f"Frame {frame_idx+1}/{len(frame_keys)}: No joint angles found, skipping")
            
            loop_count += 1
            
            # If not looping, break after first playback
            if not args.loop:
                break
                
            print("Playback complete. Starting next loop...")
        
        print("Playback complete. Homing the hand...")
        hand.home_hand()
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\nPlayback interrupted by user. Homing the hand...")
        hand.home_hand()
        time.sleep(2)
    
    finally:
        # Clean up
        print("Closing hand controller...")
        hand.close()
        print("Done.")

if __name__ == "__main__":
    main() 