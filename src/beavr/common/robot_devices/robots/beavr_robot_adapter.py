import time
import logging
import numpy as np
import zmq
import torch
import json

from beavr.utils.network import ZMQKeypointSubscriber
from beavr.common.robot_devices.robots.utils import Robot
from beavr.common.datasets.utils import Frame
from beavr.controllers.xarm7_control import DexArmControl # For XArm Inverse Kinematics
from beavr.common.robot_devices.cameras.configs import CameraConfig, OpenCVCameraConfig
from beavr.common.robot_devices.cameras.opencv import OpenCVCamera


class BeavrRobotAdapter(Robot):
    """
    Robot adapter that uses ZMQ for robot state and direct OpenCV for cameras.
    """
    robot_type = "beavr_custom_robot"
    features = {
        "observation.state": {
            "shape": (7,),  # XArm has 7 joints
            "dtype": "float32"
        },
        "action": {
            "shape": (7,),  # XArm has 7 joints
            "dtype": "float32"
        }
    }

    def __init__(
        self,
        robot_state_host: str,
        robot_state_port: int,
        robot_state_topic: str,
        cameras: dict[str, CameraConfig],
        actual_state_input_key: str = "joint_states",
        actual_state_payload_key: str = "joint_position",
        command_input_key: str = "commanded_cartesian_state",
        command_payload_key: str = "commanded_cartesian_position",
        use_cartesian_action: bool = False,
        input_angles_are_degrees: bool = True, # Assume XArm7Robot publishes in degrees by default
        xarm_controller: DexArmControl = None
    ):
        """Initialize the robot adapter."""
        # Store configuration
        self.robot_state_host = robot_state_host
        self.robot_state_port = robot_state_port
        self.robot_state_topic = robot_state_topic
        self.actual_state_input_key = actual_state_input_key
        self.actual_state_payload_key = actual_state_payload_key
        self.command_input_key = command_input_key
        self.command_payload_key = command_payload_key
        self.use_cartesian_action = use_cartesian_action
        self.input_angles_are_degrees = input_angles_are_degrees
        self.xarm_controller = xarm_controller

        # Initialize ZMQ subscriber for robot state
        self.state_subscriber = ZMQKeypointSubscriber(
            host=robot_state_host,
            port=robot_state_port,
            topic=robot_state_topic
        )

        # Initialize cameras
        self.cameras = {}
        for name, config in cameras.items():
            if isinstance(config, OpenCVCameraConfig):
                self.cameras[name] = OpenCVCamera(config)
                # Add camera features
                feature_key = f"observation.images.{name}"
                self.features[feature_key] = {
                    "shape": (config.height, config.width, config.channels),
                    "names": ["height", "width", "channels"],
                    "info": None
                }
            else:
                raise ValueError(f"Unsupported camera config type: {type(config)}")

        # State tracking
        self._is_connected = False
        self.last_observation_cache = {"state": None}
        self.last_command_cache = None

        # Performance logging
        self.logs = {}

    @property
    def is_connected(self) -> bool:
        """Return True if the robot is connected."""
        return self._is_connected

    @property
    def camera_features(self) -> dict:
        """Return camera-related features in a structure expected by dataset utilities."""
        cf = {}
        for key, feature_spec in self.features.items():
            if key.startswith("observation.images."):
                cf[key] = feature_spec
        return cf

    @property
    def motor_features(self) -> dict:
        """Return motor-related features in a structure expected by dataset utilities."""
        mf = {}
        if "action" in self.features:
            action_spec = self.features["action"].copy()
            if action_spec.get("shape") is not None:
                action_spec["names"] = [f"action_dim_{i}" for i in range(action_spec["shape"][0])]
            mf["action"] = action_spec

        if "observation.state" in self.features:
            state_spec = self.features["observation.state"].copy()
            if state_spec.get("shape") is not None:
                state_spec["names"] = [f"state_dim_{i}" for i in range(state_spec["shape"][0])]
            mf["observation.state"] = state_spec
        return mf

    @property
    def leader_arms(self) -> dict:
        """Return an empty dict since this robot has no leader arms."""
        return {}

    @property
    def follower_arms(self) -> dict:
        """Return an empty dict since this robot has no follower arms."""
        return {}

    def connect(self):
        """Connect to the robot and cameras."""
        if self._is_connected:
            return

        # Connect cameras
        for name, camera in self.cameras.items():
            try:
                camera.connect()
                logging.info(f"Connected to camera: {name}")
            except Exception as e:
                logging.error(f"Failed to connect to camera {name}: {e}")
                raise

        self._is_connected = True
        logging.info("BeavrRobotAdapter connected successfully")

    def disconnect(self):
        """Disconnect from the robot and cameras."""
        if not self._is_connected:
            return

        # Disconnect cameras
        for name, camera in self.cameras.items():
            try:
                if camera.is_connected:
                    camera.disconnect()
                    logging.info(f"Disconnected from camera: {name}")
            except Exception as e:
                logging.warning(f"Error disconnecting from camera {name}: {e}")

        self._is_connected = False
        logging.info("BeavrRobotAdapter disconnected successfully")

    def _fetch_robot_state(self) -> dict | None:
        """
        Fetch the latest robot state and command from ZMQ.
        Returns a dictionary with 'state' and 'command' keys, or None if no data is available.
        """
        try:
            # Get latest state data (non-blocking)
            data = self.state_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is None:
                return None

            # Parse the data
            if isinstance(data, dict):
                joint_state_dict = data.get(self.actual_state_input_key)
                if joint_state_dict and isinstance(joint_state_dict, dict):
                    state = joint_state_dict.get(self.actual_state_payload_key)
                else:
                    state = None
                command = data.get(self.command_input_key, {}).get(self.command_payload_key)
            else:
                # Try to parse as JSON if it's a string
                try:
                    parsed_data = json.loads(data)
                    joint_state_dict = parsed_data.get(self.actual_state_input_key)
                    if joint_state_dict and isinstance(joint_state_dict, dict):
                        state = joint_state_dict.get(self.actual_state_payload_key)
                    else:
                        state = None
                    command = parsed_data.get(self.command_input_key, {}).get(self.command_payload_key)
                except (json.JSONDecodeError, AttributeError):
                    logging.warning("Failed to parse ZMQ data")
                    return None

            if state is None or command is None:
                logging.warning("Missing state or command in ZMQ data")
                return None

            # Convert to numpy arrays and ensure state is a 7D array
            state = np.array(state, dtype=np.float32)
            if state.ndim == 0:  # If it's a scalar
                logging.warning(f"Received scalar state value: {state}. Expected 7D array.")
                return None

            command = np.array(command, dtype=np.float32)

            # Convert angles if needed
            if self.input_angles_are_degrees:
                state = np.deg2rad(state)
                if not self.use_cartesian_action:
                    command = np.deg2rad(command)

            return {"state": state, "command": command}

        except Exception as e:
            logging.error(f"Error fetching robot state: {e}")
            return None

    def _fetch_image(self, camera_name: str) -> tuple[np.ndarray | None, float]:
        """Fetch an image from the specified camera."""
        try:
            camera = self.cameras.get(camera_name)
            if camera and camera.is_connected:
                return camera.read(), time.time()
        except Exception as e:
            logging.warning(f"Error reading from camera {camera_name}: {e}")
        return None, 0.0

    def _cartesian_to_joint(self, cartesian_pose_mm_rad: np.ndarray) -> np.ndarray | None:
        """Convert a cartesian pose to joint angles using the XArm controller.
        
        Args:
            cartesian_pose_mm_rad: 6D array [x,y,z,rx,ry,rz] where:
                - x,y,z are in millimeters
                - rx,ry,rz are in radians
        """
        if self.xarm_controller is None:
            logging.warning("No XArm controller available for IK")
            return None

        try:
            # Keep the position in millimeters since that's what the XArm API expects
            cartesian_pose_mm_rad = cartesian_pose_mm_rad.copy()
            
            # Get IK solution using get_inverse_kinematics
            code, joint_angles = self.xarm_controller.robot.get_inverse_kinematics(cartesian_pose_mm_rad)
            if code == 0 and joint_angles:
                return np.array(joint_angles, dtype=np.float32)
            else:
                logging.warning(f"IK failed with code {code}")
                return None
                
        except Exception as e:
            logging.error(f"Error in cartesian to joint conversion: {e}")
            return None

    def _update_features_from_observation(self, observation: Frame):
        """Update feature shapes based on actual observation."""
        if "state" in observation:
            state_shape = observation["state"].shape
            if state_shape != self.features["observation.state"]["shape"]:
                self.features["observation.state"]["shape"] = state_shape
                self.features["action"]["shape"] = state_shape

    def capture_observation(self) -> Frame:
        """
        Capture the current robot state and camera images.
        Returns a Frame containing the observation.
        """
        if not self._is_connected:
            raise RuntimeError("Robot is not connected")

        # Get latest robot state
        robot_data = self._fetch_robot_state()
        if robot_data is None:
            # Use cached state if available
            state = self.last_observation_cache.get("state")
            if state is None:
                raise RuntimeError("No robot state available")
        else:
            state = robot_data["state"]
            self.last_observation_cache["state"] = state

        # Ensure state is a proper numpy array
        if not isinstance(state, np.ndarray):
            state = np.array(state, dtype=np.float32)
        elif state.dtype != np.float32:
            state = state.astype(np.float32)

        # Ensure state has the correct shape (should be a 1D array)
        if state.ndim == 0:  # If it's a scalar
            state = state.reshape(1)

        # Prepare observation dictionary with correct prefix
        observation = {"observation.state": state}

        # Add camera images if recording
        for name in self.cameras:
            image, timestamp = self._fetch_image(name)
            if image is not None:
                observation[f"observation.images.{name}"] = image

        # Update features if needed
        self._update_features_from_observation(observation)
        
        return Frame(observation)

    def teleop_step(self, record_data=False) -> tuple[Frame | None, Frame | None]:
        """
        Perform a teleoperation step: get state and action from ZMQ, and optionally capture images.
        """
        # Fetch combined state and command data from ZMQ
        robot_data = self._fetch_robot_state()
        if not robot_data:
            logging.warning("Teleop step failed: No data from ZMQ.")
            return self.capture_observation(), None

        # Get state and print debug info
        state = robot_data["state"]

        observation_data = {"observation.state": state}
        
        # Optionally capture images
        if record_data:
            for name in self.cameras:
                image, _ = self._fetch_image(name)
                if image is not None:
                    observation_data[f"observation.images.{name}"] = image
        
        observation_frame = Frame(observation_data)
        
        # Update cache and features
        self.last_observation_cache["state"] = state
        self._update_features_from_observation(observation_frame)

        # Process the command to get the final action
        command_payload = robot_data["command"]
        action_np = None  # This will hold the final joint angles in radians
        
        if self.use_cartesian_action:
            # Convert Cartesian command to joint action via IK
            # command_payload is a 6D array [x,y,z,rx,ry,rz] from the ZMQ message
            cartesian_command_np = np.array(command_payload, dtype=np.float32)
            if cartesian_command_np.shape == (6,):
                # Scale up positions by 1000x to match working range from tests
                cartesian_command_np[:3] *= 1000
                action_np = self._cartesian_to_joint(cartesian_command_np)
            else:
                logging.warning(f"Teleop: Cartesian command shape mismatch: {cartesian_command_np.shape}")
        else:
            # Directly use joint command, which is assumed to be in degrees from ZMQ
            action_np = np.array(command_payload, dtype=np.float32)
            if self.input_angles_are_degrees:
                action_np = np.deg2rad(action_np)

        if action_np is None:
            logging.warning("Failed to process command into action")
            return observation_frame, None

        # Create action frame
        action_frame = Frame({"action": action_np})
        self.last_command_cache = action_frame

        return observation_frame, action_frame

# Example usage
if __name__ == '__main__':
    from beavr.common.robot_devices.robots.configs import make_robot_config, make_robot_from_config
    from beavr.common.robot_devices.cameras.configs import OpenCVCameraConfig

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("BeavrRobotAdapter")
    
    # Example configuration
    config = make_robot_config(
        "beavr_adapter",
        robot_state_host="localhost",
        robot_state_port=5555,
        robot_state_topic="robot_state",
        cameras={
            "front_cam": OpenCVCameraConfig(
                camera_index=4,
                fps=30,
                width=640,
                height=480,
            )
        },
        use_cartesian_action=True,
        input_angles_are_degrees=True
    )
    
    # Create and use robot
    robot = make_robot_from_config(config)
    robot.connect()
    try:
        observation = robot.capture_observation()
        if observation:
            logger.info("Successfully captured observation")
            logger.debug(f"State shape: {observation.get('observation.state', None)}")
            logger.debug(f"Action shape: {observation.get('action', None)}")
    finally:
        robot.disconnect() 