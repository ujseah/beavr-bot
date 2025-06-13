import time
import logging
import numpy as np
import zmq

from beavr.utils.network import ZMQKeypointSubscriber
from beavr.common.robot_devices.robots.utils import Robot
from beavr.common.datasets.utils import Frame
from beavr.common.robot_devices.cameras.configs import CameraConfig, OpenCVCameraConfig
from beavr.common.robot_devices.cameras.opencv import OpenCVCamera


class MultiRobotAdapter(Robot):
    """
    General robot adapter that can handle any combination of robots (arms, hands, etc.)
    based purely on configuration. No need to create new classes for each combo.
    """
    
    def __init__(
        self,
        robot_configs: list[dict],
        cameras: dict[str, CameraConfig],
        robot_type: str = "multi_robot",
    ):
        """Initialize the multi-robot adapter.
        
        Args:
            robot_configs: List of robot configuration dictionaries, each containing:
                - name: Robot identifier (e.g., "right_xarm7", "leap")
                - host: ZMQ host
                - state_port: Port for state subscription
                - state_topic: Topic for state messages
                - robot_type: Type of robot ("arm" or "hand")
                - observation_key: Key for observation data (e.g., "state", "hand_state")
                - action_key: Key for action data (e.g., "action", "hand_action")
                - joint_count: Number of joints for this robot
                - joint_state_path: Path to joint states in message (e.g., ["joint_states", "joint_position"])
                - command_state_path: Path to command states in message (e.g., ["commanded_cartesian_state", "commanded_cartesian_position"])
            cameras: Dictionary of camera configurations
            robot_type: Overall robot type identifier
        """
        self.robot_type = robot_type
        self.robot_configs = robot_configs
        
        # Create ZMQ subscribers for each robot
        self.robot_subscribers = {}
        self.robot_state_caches = {}
        self.robot_command_caches = {}
        
        for config in robot_configs:
            name = config["name"]
            subscriber = ZMQKeypointSubscriber(
                host=config["host"],
                port=config["state_port"],
                topic=config["state_topic"]
            )
            self.robot_subscribers[name] = subscriber
            self.robot_state_caches[name] = None
            self.robot_command_caches[name] = None

        # Setup cameras
        self.cameras = {}
        for name, camera_config in cameras.items():
            if isinstance(camera_config, OpenCVCameraConfig):
                self.cameras[name] = OpenCVCamera(camera_config)
            else:
                raise ValueError(f"Unsupported camera config type: {type(camera_config)}")

        # Build features dynamically based on robot configs
        self.features = self._build_features()
        
        self._is_connected = False
        self._leader_arms = {}
        self._follower_arms = {}
        self.logs = {}

    def _build_features(self) -> dict:
        """Build the features dictionary dynamically based on robot configurations.
        
        The features are organized such that:
        - All arm joint states come first in the observation array
        - All hand joint states follow the arm states in the observation array
        - All arm actions (6D cartesian) come first in the action array
        - All hand actions follow the arm actions in the action array
        """
        features = {}
        
        # Sort configs to ensure arms come before hands
        arm_configs = [c for c in self.robot_configs if c["robot_type"] == "arm"]
        hand_configs = [c for c in self.robot_configs if c["robot_type"] == "hand"]
        sorted_configs = arm_configs + hand_configs
        
        # Calculate total dimensions
        total_obs_dim = sum(c["joint_count"] for c in sorted_configs)
        total_action_dim = sum(6 if c["robot_type"] == "arm" else c["joint_count"] for c in sorted_configs)
        
        # Build combined observation feature
        obs_names = []
        for config in sorted_configs:
            obs_names.extend([f"{config['name']}_joint_{i}" for i in range(config["joint_count"])])
        
        features["observation.state"] = {
            "shape": (total_obs_dim,),
            "dtype": "float32",
            "names": obs_names,
        }
        
        # Build combined action feature
        action_names = []
        for config in sorted_configs:
            if config["robot_type"] == "arm":
                action_names.extend([f"{config['name']}_{dim}" for dim in ["x", "y", "z", "roll", "pitch", "yaw"]])
            else:
                action_names.extend([f"{config['name']}_cmd_{i}" for i in range(config["joint_count"])])
        
        features["action"] = {
            "shape": (total_action_dim,),
            "dtype": "float32",
            "names": action_names,
        }
        
        # Add camera features
        for name, camera in self.cameras.items():
            if hasattr(camera, 'config'):
                config = camera.config
                features[f"observation.images.{name}"] = {
                    "shape": (config.height, config.width, config.channels),
                    "names": ["height", "width", "channels"],
                    "info": None
                }
        
        # Store the configs order for consistent array building
        self._sorted_configs = sorted_configs
        
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def camera_features(self) -> dict:
        return {k: v for k, v in self.features.items() if k.startswith("observation.images.")}

    @property
    def motor_features(self) -> dict:
        return {k: v for k, v in self.features.items() if not k.startswith("observation.images.")}

    @property
    def leader_arms(self) -> dict:
        return self._leader_arms

    @property
    def follower_arms(self) -> dict:
        return self._follower_arms

    def connect(self):
        """Connect to cameras."""
        if self._is_connected:
            return

        for name, camera in self.cameras.items():
            try:
                camera.connect()
                logging.info(f"Connected to camera: {name}")
                camera.async_read()
                logging.info(f"Started async reading for camera: {name}")
            except Exception as e:
                logging.error(f"Failed to connect to camera {name}: {e}")
                raise

        self._is_connected = True
        logging.info(f"MultiRobotAdapter ({self.robot_type}) connected successfully")

    def disconnect(self):
        """Disconnect from cameras."""
        if not self._is_connected:
            return

        for name, camera in self.cameras.items():
            try:
                if camera.is_connected:
                    camera.disconnect()
                    logging.info(f"Disconnected from camera: {name}")
            except Exception as e:
                logging.warning(f"Error disconnecting from camera {name}: {e}")

        self._is_connected = False
        logging.info(f"MultiRobotAdapter ({self.robot_type}) disconnected successfully")

    def _get_nested_value(self, data: dict, path: list):
        """Helper to get nested dictionary values using a path list."""
        current = data
        for key in path:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None
        return current

    def _fetch_robot_states(self) -> dict:
        """Fetch latest state from all configured robots."""
        states = {}
        
        for config in self.robot_configs:
            name = config["name"]
            subscriber = self.robot_subscribers[name]
            
            try:
                # Drain queue to get latest data
                latest_data = None
                while True:
                    data = subscriber.recv_keypoints(flags=zmq.NOBLOCK)
                    if data is None:
                        break
                    latest_data = data

                if latest_data is not None:
                    self.robot_state_caches[name] = latest_data
                    
                    # Cache command data if available
                    command_data = self._get_nested_value(latest_data, config["command_state_path"])
                    if command_data is not None:
                        self.robot_command_caches[name] = np.array(command_data, dtype=np.float32)

                states[name] = self.robot_state_caches[name]
                
            except Exception as e:
                logging.error(f"Error fetching state for robot {name}: {e}")
                states[name] = self.robot_state_caches[name]
        
        return states

    def _fetch_image(self, camera_name: str) -> tuple[np.ndarray | None, float]:
        """Fetch an image from the specified camera."""
        try:
            camera = self.cameras.get(camera_name)
            if camera and camera.is_connected:
                return camera.async_read(), time.time()
        except Exception as e:
            logging.warning(f"Error reading from camera {camera_name}: {e}")
        return None, 0.0

    def capture_observation(self) -> Frame:
        """Capture current state from all robots and cameras.
        
        Returns a Frame with:
        - A single combined observation array containing all robot states
        - Camera images if available
        """
        if not self._is_connected:
            raise RuntimeError("Robot is not connected")

        robot_states = self._fetch_robot_states()
        observation = {}

        # Build combined state array
        combined_state = []
        
        # Process each robot's state in the sorted order (arms first, then hands)
        for config in self._sorted_configs:
            name = config["name"]
            robot_data = robot_states.get(name)
            
            if robot_data is not None:
                joint_data = self._get_nested_value(robot_data, config["joint_state_path"])
                if joint_data is not None:
                    if not isinstance(joint_data, np.ndarray):
                        joint_data = np.array(joint_data, dtype=np.float32)
                    if joint_data.ndim == 0:
                        joint_data = joint_data.reshape(1)
                    combined_state.extend(joint_data)

        # Convert to numpy array and store
        if combined_state:
            observation["observation.state"] = np.array(combined_state, dtype=np.float32)

        # Add camera images
        for name in self.cameras:
            image, _ = self._fetch_image(name)
            if image is not None:
                observation[f"observation.images.{name}"] = image

        return Frame(observation)

    def teleop_step(self, record_data=False) -> tuple[Frame | None, dict | None]:
        """Acquire one frame of teleoperation data from all robots.
        
        Returns:
        - Frame with combined observation array
        - Dictionary with combined action array
        """
        if not record_data:
            return None, None

        # Get observation frame
        observation_frame = self.capture_observation()

        # Build combined action array
        combined_action = []
        
        # Process each robot's action in the sorted order (arms first, then hands)
        for config in self._sorted_configs:
            name = config["name"]
            command_cache = self.robot_command_caches.get(name)
            
            if command_cache is not None:
                combined_action.extend(command_cache)
            else:
                # If no command cache, use current state as action
                robot_data = self.robot_state_caches.get(name)
                if robot_data is not None:
                    joint_data = self._get_nested_value(robot_data, config["joint_state_path"])
                    if joint_data is not None:
                        if not isinstance(joint_data, np.ndarray):
                            joint_data = np.array(joint_data, dtype=np.float32)
                        if joint_data.ndim == 0:
                            joint_data = joint_data.reshape(1)
                        combined_action.extend(joint_data)

        action_dict = {"action": np.array(combined_action, dtype=np.float32)} if combined_action else None

        if observation_frame is not None:
            return observation_frame, action_dict

        return None, None

    def run_calibration(self):
        """No calibration needed for this adapter."""
        pass

    def send_action(self, action):
        """This adapter doesn't send actions directly."""
        logging.warning("MultiRobotAdapter does not support sending actions directly")
        return action