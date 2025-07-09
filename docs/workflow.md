# BeaVR-Bot Workflow and Technical Overview

## Introduction
This document provides a detailed technical overview of the BeaVR-Bot system architecture, data flow, and component interactions. It is intended for developers and contributors who wish to understand the inner workings of the system and how to extend it with new robotic embodiments.

## Project Structure

BeaVR-Bot follows a modular architecture with the following key directories:

- **`src/beavr/`**: Core teleoperation logic and components
- **`assets/`**: Robot URDF models and mesh files for various embodiments
- **`configs/`**: YAML configuration files for robots, environments, and network settings
- **`docker/`**: Containerized environments for development and deployment
- **`tests/`**: Comprehensive test suite for all components

## Core Architecture

### Component-Based Design

All teleoperation functionality is built around the abstract `Component` class located in `src/beavr/components/component.py`. Each component implements a `stream()` method that handles real-time data processing in the teleoperation pipeline.

### Key Components

#### 1. Detectors (`src/beavr/components/detector/`)
- **Purpose**: Capture and transform VR keypoints to a standardized coordinate system
- **Coordinate System**: Y-up right-hand (VR standard)
- **Key Classes**:
  - `OculusVRHandDetector`: Interfaces with Oculus VR devices
  - `TransformHandPositionCoords`: Transforms raw keypoints to robot coordinate frames
- **Data Flow**: VR device → Raw keypoints → Transformed keypoints (Y-up RH)

#### 2. Operators (`src/beavr/components/operators/`)
- **Purpose**: Transform VR coordinates to robot-specific coordinate systems and perform retargeting
- **Coordinate Transformation**: Y-up RH (VR) → Z-up RH (simulation/robotics standard)
- **Key Functionality**:
  - Implements abstract `Operator` base class
  - Subscribes to transformed keypoints from detectors
  - Performs motion retargeting via `_apply_retargeted_angles()`
  - Manages real-time control loops with timing constraints

#### 3. Environments (`src/beavr/components/environment/`)
- **Purpose**: Simulation environments that mirror real-world operations
- **Supported Simulators**: PyBullet, Isaac Gym, MuJoCo
- **Key Classes**:
  - `arm_env.py`: Base class for arm environments
  - `hand_env.py`: Base class for hand environments
  - Specific implementations for various robot morphologies

#### 4. Recorders (`src/beavr/components/recorders/`)
- **Purpose**: Capture synchronized teleoperation data for policy learning
- **Output Format**: LeRobot-compatible datasets
- **Data Types**:
  - Joint states and cartesian positions
  - Visual data from cameras
  - Robot state information
  - Simulation state snapshots

#### 5. Controllers (`src/beavr/controllers/`)
- **Purpose**: Low-level robot control interfaces
- **Control Modes**:
  - Joint angle control (servo-level)
  - Cartesian position control
  - Velocity control (where supported)
- **Robot-Specific**: Each robot type has dedicated controller implementations

#### 6. Interfaces (`src/beavr/interfaces/`)
- **Purpose**: Abstraction layer between operators and controllers
- **Key Role**: Translates high-level commands to robot-specific control protocols
- **Modular Design**: Single interface file per robot type

## System Workflow

### 1. Initialization Phase
```
Configuration Loading → Component Instantiation → Network Setup → Hardware Verification
```

### 2. Real-Time Teleoperation Loop
```
VR Keypoint Capture → Coordinate Transformation → Motion Retargeting → Robot Control → Data Recording
```

### 3. Data Pipeline
```
Raw VR Data → Detector (Y-up RH) → Operator (Z-up RH) → Interface → Controller → Robot
                                      ↓
                                   Recorder → LeRobot Dataset
```

## Configuration System

### Robot Configuration (`configs/robot/*.yaml`)

Each robot is defined through a YAML configuration that specifies:

- **Detector**: VR interface and keypoint capture settings
- **Transforms**: Coordinate transformation pipeline
- **Operators**: Motion retargeting and control logic
- **Interfaces**: Robot-specific communication protocols
- **Recorders**: Data collection parameters

### Modular Robot Requirements

To add a new robot to BeaVR-Bot, the system requires:

#### Minimum Hardware Requirements:
- **6DOF End Effector**: For full spatial control (orientation can be disabled for <6DOF systems)
- **Hand/Gripper**: Optional but recommended for dexterous manipulation

#### Software Implementation:
1. **Interface Class**: Inherit from base `Robot` class
2. **Controller Class**: Implement robot-specific control protocols
3. **Configuration File**: Define component pipeline in YAML
4. **URDF Assets**: Robot model for simulation environments

### Coordinate System Handling

The system handles multiple coordinate systems:

- **VR Input**: Y-up right-hand coordinate system (standard for VR devices)
- **Robot Output**: Z-up right-hand coordinate system (standard for robotics/simulation)
- **Transformation Pipeline**: Automatic conversion between coordinate systems

### Network Architecture

BeaVR-Bot uses ZeroMQ for inter-component communication:

- **Publisher-Subscriber Pattern**: For real-time data streaming
- **Request-Reply Pattern**: For control commands
- **Configurable Ports**: All communication ports defined in `configs/network.yaml`

## Extending the System

### Adding New Robot Embodiments

1. **Create Interface**: Implement robot-specific communication in `src/beavr/interfaces/`
2. **Implement Controller**: Add control logic in `src/beavr/controllers/`
3. **Configure YAML**: Define component pipeline in `configs/robot/`
4. **Add Assets**: Include URDF and mesh files in `assets/urdf/`
5. **Test Integration**: Verify functionality with test suite

### Simulation Environment Integration

1. **Inherit Base Classes**: Extend `arm_env.py` or `hand_env.py`
2. **Implement Simulation Logic**: Add physics and rendering
3. **Configure Environment**: Update `configs/environment/`
4. **Test Sim-to-Real Transfer**: Verify domain consistency

## Performance Considerations

- **Real-Time Constraints**: System maintains <20ms latency for teleoperation
- **Parallel Processing**: Components run in separate threads/processes
- **Memory Management**: Efficient data structures for high-frequency operations
- **Network Optimization**: ZeroMQ provides low-latency communication

## Development Guidelines

For detailed contribution guidelines and robot integration instructions, see [contributions.md](contributions.md).

The modular architecture ensures that new robot embodiments can be integrated with minimal code changes, requiring only the implementation of robot-specific interfaces while leveraging the existing teleoperation infrastructure. 