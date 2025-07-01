# Adding Your Own Robot to BeaVR-Bot

BeaVR-Bot is designed with a modular, embodiment-agnostic architecture that supports any robot with a 6DOF end effector and optional hand/gripper. This guide explains how to integrate new robotic systems.

## System Requirements

### Hardware Requirements
- **6DOF End Effector**: Required for full spatial control (orientation can be disabled for systems with <6DOF)
- **Hand/Gripper**: Optional but recommended for dexterous manipulation tasks
- **Network Connectivity**: For communication between VR system and robot

### Software Requirements
- The robot must be controllable via one of the supported control modes:
  - Joint angle control (servo-level)
  - Cartesian position control  
  - Velocity control

## Integration Steps

### 1. Robot Interface Implementation

Create a new interface class in `src/beavr/interfaces/` by inheriting from the base `Robot` class:

```python
# Example: src/beavr/interfaces/my_robot.py
from beavr.interfaces.robot import Robot

class MyRobotInterface(Robot):
    def __init__(self, ...):
        # Initialize robot-specific communication
        pass
    
    def get_joint_position(self):
        # Return current joint positions
        pass
    
    def send_command(self, command):
        # Send commands to robot
        pass
```

### 2. Controller Implementation

Implement robot-specific control logic in `src/beavr/controllers/`:

```python
# Example: src/beavr/controllers/my_robot_control.py
from beavr.controllers.controller import Controller

class MyRobotController(Controller):
    def __init__(self, robot_interface):
        self.robot = robot_interface
    
    def execute_cartesian_command(self, position, orientation):
        # Convert to robot-specific commands
        pass
    
    def execute_joint_command(self, joint_angles):
        # Send joint commands to robot
        pass
```

### 3. Operator Implementation

Create an operator class in `src/beavr/components/operators/` to handle VR-to-robot retargeting:

```python
# Example: src/beavr/components/operators/my_robot_operator.py
from beavr.components.operators.operator import Operator

class MyRobotOperator(Operator):
    def _apply_retargeted_angles(self):
        # Transform VR keypoints to robot commands
        # Handle coordinate system conversion (Y-up RH → Z-up RH)
        pass
```

### 4. Configuration File

Create a YAML configuration file in `configs/robot/` defining the component pipeline:

```yaml
# Example: configs/robot/my_robot.yaml
robot_name: my_robot

detector:
  _target_: beavr.components.detector.oculus.OculusVRHandDetector
  # VR detector configuration

transforms:
  - _target_: beavr.components.detector.keypoint_transform.TransformHandPositionCoords
    # Coordinate transformation configuration

operators:
  - _target_: beavr.components.operators.my_robot_operator.MyRobotOperator
    # Robot-specific operator configuration

robots:
  - _target_: beavr.interfaces.my_robot.MyRobotInterface
    # Robot interface configuration
```

### 5. Asset Integration

Add robot assets to the `assets/urdf/` directory:
- URDF model file
- Mesh files (if any)
- Material definitions

## Control Mode Implementation

### Position Control
Implement direct cartesian position commands:
```python
def execute_position_control(self, target_position, target_orientation):
    # Send position commands directly to robot
    self.robot.move_to_pose(target_position, target_orientation)
```

### Velocity Control  
Implement velocity-based control:
```python
def execute_velocity_control(self, linear_velocity, angular_velocity):
    # Send velocity commands to robot
    self.robot.set_velocity(linear_velocity, angular_velocity)
```

## Coordinate System Handling

BeaVR-Bot automatically handles coordinate system transformations:
- **VR Input**: Y-up right-hand coordinate system
- **Robot Output**: Z-up right-hand coordinate system
- **Transformation**: Automatic conversion in the operator pipeline

## Testing Integration

1. **Unit Tests**: Create tests in `tests/` directory
2. **Integration Tests**: Verify VR-to-robot communication
3. **Simulation Tests**: Test in simulation environment before real robot

## Configuration Examples

### Single Arm Configuration
For robots with only an arm manipulator, disable hand components in the configuration.

### Bimanual Configuration  
For dual-arm systems, create separate operator instances for each arm.

### Custom DOF Handling
For robots with <6DOF, disable orientation control in the operator configuration.

## Advanced Features

### Simulation Environment
Create corresponding simulation environment by inheriting from `arm_env.py` or `hand_env.py` in `src/beavr/components/environment/`.

### Data Recording
Configure data recording parameters in the robot configuration to capture teleoperation data for policy learning.

### Custom Retargeting
Implement custom motion retargeting algorithms in the operator class for robot-specific kinematics.

For detailed technical information about the system architecture, see [Technical Overview and Workflow](workflow.md).