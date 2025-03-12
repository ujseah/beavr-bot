import pybullet as p
import pybullet_data
import time
import numpy as np
import os

def test_leap_hand():
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up simulation environment
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    
    # Get path to Leap Hand URDF
    hand_urdf_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "VR_Teleoperation",
        "beavr",
        "src",
        "components",
        "environment",
        "assets",
        "urdf",
        "leap_hand",
        "leap_hand_right.urdf"
    )
    
    # Add the URDF directory to the search path so PyBullet can find the meshes
    p.setAdditionalSearchPath(os.path.dirname(hand_urdf_path))
    
    # Load Leap Hand
    robot_id = p.loadURDF(hand_urdf_path, 
                         basePosition=[0, 0, 0],
                         useFixedBase=True)
    
    # Print information about joints
    num_joints = p.getNumJoints(robot_id)
    print(f"\nTotal number of joints: {num_joints}")
    print("\nJoint Information:")
    print("-" * 50)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"Joint {i}:")
        print(f"  Name: {joint_info[1].decode('utf-8')}")
        print(f"  Type: {joint_info[2]}")
        print(f"  Lower limit: {joint_info[8]}")
        print(f"  Upper limit: {joint_info[9]}")
        print(f"  Max force: {joint_info[10]}")
        print(f"  Max velocity: {joint_info[11]}")
        print("-" * 50)
    
    # Test moving joints
    while True:
        try:
            # Move joints in a simple pattern
            for joint in range(num_joints):
                # Only move actual joints (not fixed joints)
                if p.getJointInfo(robot_id, joint)[2] != p.JOINT_FIXED:
                    p.setJointMotorControl2(
                        bodyIndex=robot_id,
                        jointIndex=joint,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=np.sin(time.time()) * 0.5
                    )
            
            # Get and print end-effector position (using the thumb tip as reference)
            state = p.getLinkState(robot_id, num_joints-1)
            print(f"\rThumb tip position: {state[0]}", end="")
            
            p.stepSimulation()
            time.sleep(1./240.)  # 240 Hz simulation
            
        except KeyboardInterrupt:
            break
    
    p.disconnect()

if __name__ == "__main__":
    test_leap_hand() 