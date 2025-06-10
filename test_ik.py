from xarm.wrapper import XArmAPI
import numpy as np

# Connect to robot
arm = XArmAPI('192.168.1.197')

# The exact failing position from logs
position = [0.34465954, 0.01285581, 0.30866042]
orientation = [3.1396685, 0.0233298, -0.01143624]

# Test with different scale factors
scale_factors = [1, 10, 100, 1000, 10000]

print("\nTesting position with different scale factors:")
print("Position (original):", position)
print("Orientation:", orientation)
print("================================================")

for scale in scale_factors:
    scaled_pos = [x * scale for x in position]
    test_pose = scaled_pos + orientation
    
    print(f'\nScale factor: {scale}x')
    print(f'Scaled position: {scaled_pos}')
    code, angles = arm.get_inverse_kinematics(test_pose)
    print('Result code:', code)
    if code == 0:
        print('Joint angles:', angles)
    else:
        print('Failed to find solution') 