from beavr.controllers.xarm7_control import DexArmControl
from .robot import RobotWrapper
from beavr.utils.network import ZMQKeypointSubscriber
from beavr.utils.network import EnhancedZMQKeypointPublisher as ZMQKeypointPublisher
import numpy as np
import time
import zmq
from beavr.constants import VR_FREQ
import logging

class XArm7Robot(RobotWrapper):
    def __init__(self, host, endeff_subscribe_port, joint_subscribe_port, home_subscribe_port,
                 reset_subscribe_port, robot_ip, is_right_arm=True,
                 # Port for publishing general data (e.g., end effector pose for viz)
                 endeff_publish_port: int = 10009, # Default, check config
                 # Port specifically for publishing the state dictionary for recording
                 state_publish_port: int = 10010, # Default, check config
                 **kwargs): # Allow extra args
        """
        Args:
            host: Network host address
            endeff_subscribe_port: Port for end effector command subscription
            joint_subscribe_port: Port for joint command subscription (if used)
            reset_subscribe_port: Port for reset subscription
            robot_ip: IP address of the XArm robot
            is_right_arm: Whether this is the right arm (True) or left arm (False)
            endeff_publish_port: Port for publishing general end-effector data.
            state_publish_port: Port for publishing the full state dictionary for recording.
        """
        # Ensure required ports are present
        if not endeff_publish_port:
             raise ValueError("XArm7Robot requires an 'endeff_publish_port'")
        if not state_publish_port:
             raise ValueError("XArm7Robot requires a 'state_publish_port'")

        self._controller = DexArmControl(ip=robot_ip)
        self._is_right_arm = is_right_arm
        
        # Use VR_FREQ instead of hardcoded 90Hz
        self._data_frequency = VR_FREQ
        print(f"XArm7Robot '{self.name}' initialized with command frequency: {self._data_frequency}Hz")
        print(f"  Publishing general data on tcp://{host}:{endeff_publish_port}")
        print(f"  Publishing state dictionary on tcp://{host}:{state_publish_port} with topic '{self.name}'")
        
        # Subscribers
        self._cartesian_coords_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = endeff_subscribe_port,
            topic = 'endeff_coords'
        )
        
        self._joint_state_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = joint_subscribe_port,
            topic = 'joint'
        )
        
        # Dedicated RESET subscriber -------------------------------------------------
        self._reset_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = reset_subscribe_port,
            topic = 'reset'
        )

        # Dedicated HOME subscriber --------------------------------------------------
        self._home_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = home_subscribe_port,
            topic = 'home'
        )

        # Optional secondary RESET subscriber on the *home* port for reliability
        self._reset_home_subscriber = None
        if home_subscribe_port != reset_subscribe_port:
            try:
                self._reset_home_subscriber = ZMQKeypointSubscriber(
                    host = host,
                    port = home_subscribe_port,
                    topic = 'reset'
                )
            except Exception as e:
                logging.warning(
                    f"Failed to create secondary reset subscriber on home port {home_subscribe_port}: {e}"
                )

        # Publisher for general data (using endeff_publish_port)
        self._general_publisher = ZMQKeypointPublisher(
            host = host,
            port = endeff_publish_port
        )

        # Publisher specifically for the state dictionary (using state_publish_port)
        self._state_publisher = ZMQKeypointPublisher(
            host = host,
            port = state_publish_port
        )

        # Add caches for received messages
        self._latest_cartesian_coords = None
        self._latest_joint_state = None
        self._latest_cartesian_state_timestamp = 0
        self._latest_joint_state_timestamp = 0
        
        # Recording control
        self._is_recording_enabled = False

        # ------------------------------------------------------------------
        # Flag used to temporarily *pause* consumption of incoming Cartesian
        # commands.  This allows the high-level controller to home the robot
        # and keep it idle (e.g. during dataset reset) even if the teleop
        # operator process continues to publish targets.
        # ------------------------------------------------------------------
        self._teleop_paused = False

        # Add cache for the last valid commanded cartesian position received
        self._latest_commanded_cartesian_position = None
        self._latest_commanded_cartesian_timestamp = 0.0

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'operator_cartesian_states': self.get_cartesian_state_from_operator,
            'xarm_cartesian_states': self.get_robot_actual_cartesian_position,
            'commanded_cartesian_state': self.get_cartesian_commanded_position,
            'joint_angles_rad': self.get_joint_position, # These are the robot's joint angles in radians
        }

    @property
    def name(self):
        return 'right_xarm7' if self._is_right_arm else 'left_xarm7'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_arm_states()
    
    def get_arm_velocity(self):
        return self._controller.get_arm_velocity()

    def get_arm_torque(self):
        return self._controller.get_arm_torque()

    def get_cartesian_state(self):
        cartesian_state=self._controller.get_cartesian_state() 
        return cartesian_state
    
    def get_joint_position(self):
        return self._controller.get_arm_position()
    
    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_coords()

    def reset(self):
        return self._controller._init_xarm_control()
    
    def get_pose(self):
        return self._controller.get_arm_pose()

    # Movement functions
    def home(self):
        return self._controller.home_arm()

    def move(self, input_angles):
        self._controller.move_arm_joint(input_angles)

    def move_coords(self, cartesian_coords, duration=3):
        self._controller.move_arm_cartesian(cartesian_coords, duration=duration)

    def arm_control(self, cartesian_coords):
        self._controller.arm_control(cartesian_coords)

    def move_velocity(self, input_velocity_values, duration):
        pass

    def get_cartesian_state_from_operator(self):
        if self._latest_cartesian_coords is None:
            return None

        cartesian_state_dict = dict(
            cartesian_position = np.array(self._latest_cartesian_coords, dtype=np.float32),
            timestamp = self._latest_cartesian_state_timestamp
        )
        return cartesian_state_dict

    def get_joint_state_from_operator(self):
        if self._latest_joint_state is None:
            return None
        
        joint_state_dict = dict(
            joint_position = np.array(self._latest_joint_state, dtype=np.float32),
            timestamp = self._latest_joint_state_timestamp
        )
        return joint_state_dict

    def get_cartesian_commanded_position(self):
        if self._latest_commanded_cartesian_position is None:
            return None
        return {
            "commanded_cartesian_position": self._latest_commanded_cartesian_position,
            "timestamp": self._latest_commanded_cartesian_timestamp,
        }

    def get_robot_actual_cartesian_position(self):
        cartesian_state=self.get_cartesian_position()
        cartesian_dict = dict(
            cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        return cartesian_dict
    
    def get_robot_actual_joint_position(self):
        joint_state_dict = self._controller.get_arm_joint_state()
        return joint_state_dict
    
    def send_robot_pose(self):
        cartesian_state = self._controller.get_arm_pose()
        self._general_publisher.pub_keypoints(cartesian_state, "endeff_homo")

    def check_reset(self):
        # Check dedicated RESET subscriber first
        reset_bool = self._reset_subscriber.recv_keypoints(flags=zmq.NOBLOCK)

        # Fallback: check optional secondary subscriber on the *home* port
        if reset_bool is None and self._reset_home_subscriber is not None:
            reset_bool = self._reset_home_subscriber.recv_keypoints(flags=zmq.NOBLOCK)

        return reset_bool is not None

    def check_home(self):
        home_bool = self._home_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        return home_bool is not None

    # Modified stream method with automatic recording start after reset
    def stream(self):
        self._controller.home_arm()
        assert self._controller.robot.set_mode_and_state(1, 0), "Failed to enter SERVO-READY"
        
        target_interval = 1.0 / self._data_frequency
        next_frame_time = time.time()
        
        while True:
            current_time = time.time()

            # Only process at the target frequency
            if current_time >= next_frame_time:
                # Calculate next frame time
                next_frame_time = current_time + target_interval

                # ------------------------------------------------------------------
                # 1) Check for *home* or *reset* signals **before** processing
                #    any operator commands so that homing takes precedence.
                # ------------------------------------------------------------------

                if self.check_home():
                    print("Home signal received")
                    # Enter *paused* mode and home the arm.
                    self._controller.home_arm()
                    self._teleop_paused = True
                    print(f"Teleop paused for {self._teleop_paused}")

                    # ------------------------------------------------------------------
                    # Remove *any* additional queued home signals so that we do not
                    # immediately re-enter the paused state once teleop resumes.  This
                    # is important now that the home broadcaster may repeat the message
                    # several times for reliability.
                    # ------------------------------------------------------------------
                    while True:
                        leftover_home = self._home_subscriber.recv_keypoints(zmq.NOBLOCK)
                        if leftover_home is None:
                            break

                    # Flush any queued Cartesian commands so they are not
                    # executed after resuming.
                    while True:
                        leftover = self._cartesian_coords_subscriber.recv_keypoints(zmq.NOBLOCK)
                        if leftover is None:
                            break

                if self.check_reset():
                    # Resume teleoperation: publish current pose so the
                    # operator can re-initialise its kinematic mapping.
                    self.send_robot_pose()
                    self._teleop_paused = False
                    print(f"Teleop resumed for {self._teleop_paused}")
                # ------------------------------------------------------------------
                # 2) Process incoming Cartesian targets **only when** teleop is
                #    active.
                # ------------------------------------------------------------------
                if not self._teleop_paused:
                    msg = self._cartesian_coords_subscriber.recv_keypoints(zmq.NOBLOCK)
                    if msg:
                        self._latest_commanded_cartesian_position = np.concatenate(
                            [np.asarray(msg["position"], dtype=np.float32),
                             np.asarray(msg["orientation"], dtype=np.float32)]
                        )
                        self._latest_commanded_cartesian_timestamp = msg["timestamp"]
                        self.move_coords(self._latest_commanded_cartesian_position)

                # Publish the current state every cycle so that external
                # adapters (e.g. MultiRobotAdapter) receive up-to-date joint
                # information for observation building.
                try:
                    self.publish_current_state()
                except Exception as e:
                    logging.error(f"Failed to publish current state for {self.name}: {e}")

                # Calculate sleep time to maintain consistent frequency
                sleep_time = max(0, next_frame_time - time.time())
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def publish_current_state(self):
        """
        Gathers all state data defined in recorder_functions
        and publishes it as a single dictionary via ZMQ using self.name as topic
        on the state_publish_port.
        """
        current_state_dict = {}
        publish_time = time.time()

        for key, getter_function in self.recorder_functions.items():
            try:
                state_data = getter_function()
                if state_data is not None:
                    current_state_dict[key] = state_data
                    logging.debug(f"Got state for '{key}': {state_data}")
            except Exception as e:
                logging.error(f"Failed to get state for '{key}' on robot '{self.name}': {e}")
                current_state_dict[key] = None

        current_state_dict['timestamp'] = publish_time

        # Log the complete state dictionary
        logging.debug(f"Publishing state dictionary for robot '{self.name}': {current_state_dict}")

        # Publish the state dictionary using the dedicated state publisher and self.name topic
        try:
            self._state_publisher.pub_keypoints(current_state_dict, self.name)
        except Exception as e:
            logging.error(f"Error publishing state dictionary for robot '{self.name}': {e}")

    def __del__(self):
        # Clean up ZMQ sockets
        print(f"Closing ZMQ sockets for {self.name}")
        if hasattr(self, '_cartesian_coords_subscriber'): self._cartesian_coords_subscriber.stop()
        if hasattr(self, '_joint_state_subscriber'): self._joint_state_subscriber.stop()
        if hasattr(self, '_home_subscriber'): self._home_subscriber.stop()
        if hasattr(self, '_reset_subscriber'): self._reset_subscriber.stop()
        if hasattr(self, '_reset_home_subscriber') and self._reset_home_subscriber is not None:
            self._reset_home_subscriber.stop()
        # Close both publishers
        if hasattr(self, '_general_publisher'): self._general_publisher.stop()
        if hasattr(self, '_state_publisher'): self._state_publisher.stop()