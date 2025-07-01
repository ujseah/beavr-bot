import time
import logging
import numpy as np
import zmq
import torch
from concurrent.futures import ThreadPoolExecutor

from beavr.constants import LEAP_HOME_JS, ROBOT_HOME_JS, ARM_TELEOP_CONT, ARM_TELEOP_STOP, TELEOP_HANDSHAKE_PORT
from beavr.common.robot_devices.robots.utils import Robot
from beavr.common.datasets.utils import Frame
from beavr.common.robot_devices.cameras.configs import CameraConfig, OpenCVCameraConfig
from beavr.common.robot_devices.cameras.opencv import OpenCVCamera

# Network helpers
from beavr.utils.network import (
    ZMQKeypointSubscriber,
    EnhancedZMQKeypointPublisher as ZMQKeypointPublisher,
)
from beavr.utils.handshake import HandshakeClient


class MultiRobotAdapter(Robot):
    """
    General robot adapter that can handle any combination of robots (arms, hands, etc.)
    based purely on configuration. No need to create new classes for each combo.
    """
    
    def __init__(
        self,
        robot_configs: list[dict],
        cameras: dict[str, CameraConfig],
        robot_type: str = "multi_robot_adapter",
        *,
        teleop_port: int = 8089,
        handshake_host: str = "127.0.0.1",
    ):
        """Initialize the multi-robot adapter.
        
        Args:
            robot_configs: List of robot configuration dictionaries, each containing:
                - name: Robot identifier (e.g., "right_xarm7", "leap")
                - host: ZMQ host
                - state_port: Port for state subscription
                - state_topic: Topic for state messages
                - robot_type: Type of robot ("arm" or "hand")
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

        self.home_publishers: dict[str, dict] = {}
        for cfg in robot_configs:
            r_name = cfg["name"]
            home_port = cfg.get("home_subscribe_port")
            if home_port is None:
                # Not fatal – we fall back to the main command publisher which
                # might still work (but risks port conflicts if another PUB is
                # bound there).  Warn the user so they can fix the config.
                logging.warning(
                    "Robot '%s' has no 'home_subscribe_port' – homing signal will reuse the main command port (risk of conflicts)",
                    r_name,
                )
                continue


            self.home_publishers[r_name] = dict(
                publisher=ZMQKeypointPublisher(host=cfg["host"], port=home_port),
                port=home_port,
                topic="home"
            )


        # ------------------------------------------------------------------
        # Ensure PUB sockets are created *immediately* so that SUB sockets on
        # the robot side can establish their TCP connection long before the
        # first HOME signal is sent.  Without this, the very first message
        # can be silently dropped due to the inherent "slow-joiner" behaviour
        # of ZeroMQ PUB/SUB.
        # ------------------------------------------------------------------
        for pub_info in self.home_publishers.values():
            try:
                # Accessing the underlying manager forces socket creation.
                p = pub_info["publisher"]
                p._manager.get_publisher(p._host, p._port)  # pylint: disable=protected-access
            except Exception as e:
                logging.warning(
                    "Failed to pre-bind home publisher socket on tcp://%s:%s: %s",
                    p._host,
                    p._port,
                    e,
                )


        self.command_publishers: dict[str, dict] = {}
        for cfg in robot_configs:
            r_name = cfg["name"]
            r_type = cfg.get("robot_type", "arm")

            # Resolve port & topic based on robot type and config keys
            if r_type == "arm":
                pub_port = cfg.get("endeff_publish_port") or cfg.get("command_pub_port")
                topic = cfg.get("command_topic", "endeff_coords")
            else:  # hand or other
                pub_port = cfg.get("joint_angle_publish_port") or cfg.get("command_pub_port")
                topic = cfg.get("command_topic", "joint_angles")

            # If no port is provided, we cannot create a publisher – warn user.
            if pub_port is None:
                logging.warning(
                    f"No command publish port found in config for robot '{r_name}'. "
                    "This robot will be ignored in send_action()."
                )
                continue


            publisher = ZMQKeypointPublisher(host=cfg["host"], port=pub_port)
            self.command_publishers[r_name] = dict(
                publisher=publisher,
                topic=topic,
                robot_type=r_type,
                port=pub_port,
            )

        # Publisher to toggle reset in the operator
        self.teleop_publisher = ZMQKeypointPublisher(host=cfg["host"], port=teleop_port)

        # Thread pool for asynchronous publishing
        # We keep a small pool (<= #robots) so that PUB sends never block the
        # policy/control loop. Threads are daemonised by default.
        self._publish_pool = ThreadPoolExecutor(max_workers=max(4, len(self.command_publishers)))

        self._last_quaternions: dict[str, np.ndarray] = {}
        self.handshake_host = handshake_host


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
        total_action_dim = sum(7 if c["robot_type"] == "arm" else c["joint_count"] for c in sorted_configs)
        
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
                action_names.extend([f"{config['name']}_{dim}" for dim in ["x", "y", "z", "qx", "qy", "qz", "qw"]])
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

        self.teleop_stop()

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
        missing_robots: list[str] = []
        
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
                    continue  # processed this robot
            # If we reach here we did not manage to append any data for this robot.
            missing_robots.append(name)

            if config["robot_type"] == "arm":
                combined_state.extend(np.array(ROBOT_HOME_JS, dtype=np.float32))
            elif config["robot_type"] == "hand":
                combined_state.extend(np.array(LEAP_HOME_JS, dtype=np.float32))

        # Attach error flags if any
        if missing_robots:
            observation["error.missing_states"] = missing_robots

        # Convert to torch tensor and store
        if combined_state:
            observation["observation.state"] = torch.from_numpy(
                np.array(combined_state, dtype=np.float32)
            )

        # Add camera images
        for name in self.cameras:
            image, _ = self._fetch_image(name)
            if image is not None:
                observation[f"observation.images.{name}"] = torch.from_numpy(image)

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

        # Build combined action array (raw, in native units)
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

        if combined_action:
            action_dict = {"action": np.asarray(combined_action, dtype=np.float32)}
        else:
            action_dict = None

        if observation_frame is not None:
            return observation_frame, action_dict

        return None, None


    def teleop_stop(self):
        """Send a stop signal to all operators to pause teleoperation.
        """
        print(f"[{self.robot_type}] teleop_stop() sent at {time.time():.3f}")
        self.teleop_publisher.pub_keypoints(ARM_TELEOP_STOP, 'pause')

 
        ok = HandshakeClient(host=self.handshake_host, port=TELEOP_HANDSHAKE_PORT).request(timeout=2.0)
        if ok:
            print("Operator acknowledged pause (handshake OK)")
        else:
            print("WARNING: No handshake ACK from operator – proceeding anyway")

        print("Paused teleoperation")

        for _, home_info in self.home_publishers.items():

            home_info["publisher"].pub_keypoints(1, "home")  # 1st attempt (may drop)
            # time.sleep(0.1)  # allow SUB sockets to connect
            # home_info["publisher"].pub_keypoints(1, "home")  # 2nd attempt


    def teleop_resume(self):
        """Send a resume signal to all operators to resume teleoperation.
        """
        print(f"[{self.robot_type}] teleop_resume() sent at {time.time():.3f}")
        self.teleop_publisher.pub_keypoints(ARM_TELEOP_CONT, 'pause')
        print("Resumed teleoperation")


    def run_calibration(self):
        """No calibration needed for this adapter."""
        pass

    def send_action(self, action):
        """Publish a **concatenated** action vector to the corresponding robots.

        The input `action` is assumed to follow the ordering defined in
        `self._sorted_configs` (all arm actions first, then hand actions).

        • Arm action (6-DoF): `[x, y, z, qx, qy, qz, qw]` will be mapped to a
          dictionary `{"position": [...], "orientation": [...], "timestamp": t}`
          and published on topic ``endeff_coords``.
        • Hand action (`n` joints): raw joint position array will be published
          on topic ``joint_angles``.

        Parameters
        ----------
        action : array-like | torch.Tensor
            The concatenated action vector.

        Returns
        -------
        torch.Tensor
            The exact action that has been sent (after possible type/shape
            conversions).
        """
        # Defensive copy & type normalisation --------------------------------
        if isinstance(action, torch.Tensor):
            action_np = action.detach().cpu().flatten().numpy()
        else:
            action_np = np.asarray(action, dtype=np.float32).flatten()

        expected_dim = self.features["action"]["shape"][0]
        if action_np.size != expected_dim:
            raise ValueError(
                f"Action dim mismatch. Expected {expected_dim} values but got {action_np.size}."
            )

        # Slice and publish ---------------------------------------------------
        sent_segments = []
        cursor = 0

        for cfg in self._sorted_configs:
            name = cfg["name"]
            r_type = cfg["robot_type"]
            dim = 7 if r_type == "arm" else cfg["joint_count"]

            segment = action_np[cursor : cursor + dim]
            cursor += dim

            pub_info = self.command_publishers.get(name)
            if pub_info is None:
                logging.debug(f"No publisher configured for robot '{name}'. Skipping action publish.")
                sent_segments.append(segment)
                continue

            # Ensure numpy array dtype float32
            segment_np = np.asarray(segment, dtype=np.float32)

            # Build the payload according to robot type ------------------
            if r_type == "arm":
                pos = segment_np[:3]
                quat = segment_np[3:7]
                # Force positive hemisphere for reproducibility
                if quat[3] < 0:
                    quat = -quat
                self._last_quaternions[name] = quat.copy()

                payload = {
                    "position": pos.tolist(),
                    "orientation": quat.tolist(),  # qx,qy,qz,qw
                    "timestamp": time.time(),
                }
            elif r_type == "hand":
                payload = segment_np
            else:
                logging.error(f"Unsupported robot type: {r_type}")
                sent_segments.append(segment_np)
                continue

            # Submit non-blocking publish --------------------------------
            self._publish_pool.submit(self._publish_async, pub_info, cfg, payload)

            sent_segments.append(segment_np)

        # Concatenate the segments back into a single tensor for return
        return torch.from_numpy(np.concatenate(sent_segments).astype(np.float32))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _publish_async(pub_info: dict, cfg: dict, payload):
        """Publish *payload* on the publisher contained in *pub_info*.

        This is executed in the background thread pool so that network I/O
        (and occasional DNS/network stalls) never block the policy loop.
        """
        try:
            pub_info["publisher"].pub_keypoints(payload, pub_info["topic"])
        except Exception as e:
            logging.error(
                f"Async publish error for robot '{cfg['name']}' on tcp://{cfg['host']}:{pub_info['port']}: {e}"
            )

    def __del__(self):
        # Shutdown thread-pool gracefully
        if hasattr(self, "_publish_pool"):
            self._publish_pool.shutdown(wait=False)
        if hasattr(self, "_last_quaternions"):
            self._last_quaternions.clear()
        if hasattr(self, "teleop_reset_publisher"):
            self.teleop_reset_publisher.stop()
        if hasattr(self, "teleop_pause_publisher") and self.teleop_pause_publisher is not None:
            self.teleop_pause_publisher.stop()