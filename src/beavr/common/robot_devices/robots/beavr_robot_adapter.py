import time
import logging
import numpy as np
import zmq

from beavr.utils.network import ZMQKeypointSubscriber
from beavr.common.robot_devices.robots.utils import Robot
from beavr.common.datasets.utils import Frame
from beavr.common.robot_devices.cameras.configs import CameraConfig, OpenCVCameraConfig
from beavr.common.robot_devices.cameras.opencv import OpenCVCamera


class BeavrRobotAdapter(Robot):
    """
    Simplified robot adapter for data acquisition. It listens to the state dictionary
    published by XArm7Robot.publish_current_state() which contains:
    - joint_states: Current joint angles
    - xarm_cartesian_states: Current end-effector pose
    - commanded_cartesian_state: Latest command sent to robot
    """
    robot_type = "beavr_custom_robot"
    features = {
        "observation.state": {
            "shape": (7,),  # 7 joint angles in radians
            "dtype": "float32",
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
        },
        "action": {
            "shape": (6,),  # 3D position (m) + 3D euler orientation (rad)
            "dtype": "float32",
            "names": ["x", "y", "z", "roll", "pitch", "yaw"],
        }
    }

    def __init__(
        self,
        robot_state_host: str,
        robot_state_port: int,
        robot_state_topic: str,
        cameras: dict[str, CameraConfig],
    ):
        """Initialize the robot adapter for data acquisition.
        
        Args:
            robot_state_host: Host for ZMQ state dictionary subscription
            robot_state_port: Port for ZMQ state dictionary subscription
            robot_state_topic: Topic for ZMQ state dictionary messages
            cameras: Dictionary of camera configurations
        """
        # Single subscriber for complete state dictionary
        self.state_subscriber = ZMQKeypointSubscriber(
            host=robot_state_host,
            port=robot_state_port,
            topic=robot_state_topic
        )

        self.cameras = {}
        for name, config in cameras.items():
            if isinstance(config, OpenCVCameraConfig):
                self.cameras[name] = OpenCVCamera(config)
                feature_key = f"observation.images.{name}"
                self.features[feature_key] = {
                    "shape": (config.height, config.width, config.channels),
                    "names": ["height", "width", "channels"],
                    "info": None
                }
            else:
                raise ValueError(f"Unsupported camera config type: {type(config)}")

        self._is_connected = False
        self.last_state_cache = None
        self.last_command_cache = None
        # Empty dictionaries for leader/follower arms since this is just an adapter
        self._leader_arms = {}
        self._follower_arms = {}
        # Dictionary to store timing and performance metrics
        self.logs = {}

    @property
    def is_connected(self) -> bool:
        """Return True if the robot is connected."""
        return self._is_connected

    @property
    def camera_features(self) -> dict:
        """Return camera-related features."""
        return {k: v for k, v in self.features.items() if k.startswith("observation.images.")}

    @property
    def motor_features(self) -> dict:
        """Return motor-related features."""
        return {
            "action": self.features["action"],
            "observation.state": self.features["observation.state"]
        }

    @property
    def leader_arms(self) -> dict:
        """Return empty dict since this adapter doesn't have leader arms."""
        return self._leader_arms

    @property
    def follower_arms(self) -> dict:
        """Return empty dict since this adapter doesn't have follower arms."""
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
        logging.info("BeavrRobotAdapter connected successfully")

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
        logging.info("BeavrRobotAdapter disconnected successfully")

    def _fetch_robot_state(self) -> dict | None:
        """
        Fetch the latest state dictionary from ZMQ containing both
        joint states and cartesian command information.
        """
        try:
            # Get latest complete message (drain queue)
            latest_data = None
            while True:
                data = self.state_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
                if data is None:
                    break
                latest_data = data

            if latest_data is None:
                return self.last_state_cache

            # Extract joint state
            joint_states = latest_data.get('joint_states')
            if joint_states is None:
                logging.warning("No joint_states in message")
                return self.last_state_cache

            # Get joint positions in radians
            joint_positions = joint_states.get('joint_position')
            if not isinstance(joint_positions, np.ndarray):
                joint_positions = np.array(joint_positions, dtype=np.float32)

            # Get latest cartesian command
            cartesian_cmd = latest_data.get('commanded_cartesian_state')
            if cartesian_cmd is not None:
                cartesian_position = cartesian_cmd.get('commanded_cartesian_position')
                if cartesian_position is not None and not isinstance(cartesian_position, np.ndarray):
                    cartesian_position = np.array(cartesian_position, dtype=np.float32)
                self.last_command_cache = cartesian_position

            state_dict = {
                "state": joint_positions,
                "timestamp": latest_data.get('timestamp', time.time())
            }
            self.last_state_cache = state_dict
            return state_dict

        except Exception as e:
            logging.error(f"Error fetching robot state: {e}")
            return self.last_state_cache

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
        """Capture the current robot state and camera images."""
        if not self._is_connected:
            raise RuntimeError("Robot is not connected")

        robot_data = self._fetch_robot_state()
        if robot_data is None:
            raise RuntimeError("No robot state available")

        state = robot_data["state"]
        if state.ndim == 0:
            state = state.reshape(1)

        observation = {"observation.state": state}

        for name in self.cameras:
            image, _ = self._fetch_image(name)
            if image is not None:
                observation[f"observation.images.{name}"] = image

        return Frame(observation)

    def teleop_step(self, record_data=False) -> tuple[Frame | None, dict | None]:
        """
        Acquires one frame of teleoperation data for recording.
        Gets both observation (joint states) and action (cartesian command)
        from the single state dictionary published by XArm7Robot.

        Args:
            record_data: Whether to record data. If False, returns None.

        Returns:
            tuple[Frame, dict]: Observation frame and action dictionary if record_data is True,
                              otherwise returns (None, None)
        """
        # Early exit if not recording data
        if not record_data:
            return None, None

        # Fetch state dictionary which includes both joint state and cartesian command
        robot_data = self._fetch_robot_state()
        if robot_data is None:
            return None, None

        # Get observation frame (joint states + camera images)
        observation_frame = self.capture_observation()

        # Get action from cached cartesian command
        if self.last_command_cache is not None:
            action_dict = {'action': self.last_command_cache}
            return observation_frame, action_dict

        return None, None

    def run_calibration(self):
        """No calibration needed for this adapter."""
        pass

    def send_action(self, action):
        """
        This adapter doesn't send actions directly - it only listens to actions 
        sent through the ZMQ stream.
        """
        logging.warning("BeavrRobotAdapter does not support sending actions directly")
        return action