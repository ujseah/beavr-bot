from beavr.controllers.leap_control import DexArmControl
from beavr.interfaces.robot import RobotWrapper
import numpy as np
import threading
import time
from queue import Queue
import traceback

from beavr.utils.network import (
    ZMQKeypointSubscriber,
    ZMQPublisherManager,
    cleanup_zmq_resources,
)
import zmq
from beavr.utils.registry import GlobalRegistry

class LeapHandRobot(RobotWrapper):
    def __init__(self, host, joint_angle_subscribe_port, joint_angle_publish_port, reset_subscribe_port, 
                 state_publish_port=10008, home_subscribe_port=10007, simulation_mode=False, **kwargs):
        """Initialize the robot adapter for data acquisition.
        
        Args:
            host: Network host address
            joint_angle_subscribe_port: Port for joint angle command subscription
            joint_angle_publish_port: Port for publishing joint angles
            reset_subscribe_port: Port for reset subscription
            state_publish_port: Port for publishing the full state dictionary for recording
            simulation_mode: Whether to run in simulation mode
            **kwargs: Additional configuration parameters from Hydra
        """
        # Ensure required ports are present
        if not state_publish_port:
            raise ValueError("LeapHandRobot requires a 'state_publish_port'")

        # Store recorder config if provided
        self.recorder_config = kwargs.get('recorder_config', {})
        self.robot_identifier = self.recorder_config.get('robot_identifier', self.name)
        
        if simulation_mode:
            self._controller = None  # Skip hardware initialization in sim mode
        else:
            self._controller = DexArmControl()  # Only init hardware in real mode
        
        self._data_frequency = 60
        self._command_queue = Queue(maxsize=1)
        self._last_command = None
        self._movement_thread = threading.Thread(target=self._execute_movement_loop, daemon=True)
        self._movement_thread.start()

        print(f"LeapHandRobot '{self.name}' initialized with command frequency: {self._data_frequency}Hz")
        print(f"  Publishing state dictionary on tcp://{host}:{state_publish_port} with topic '{self.name}'")

        self._joint_angle_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = joint_angle_subscribe_port,
            topic = 'joint_angles'
        )

        # Centralized publisher manager (new pub/sub structure)
        self._publisher_manager = ZMQPublisherManager.get_instance()
        self._publisher_host = host
        self._joint_angle_publish_port = joint_angle_publish_port
        self._state_publish_port = state_publish_port

        # Pre-bind sockets so that subscribers can connect early
        self._publisher_manager.get_publisher(self._publisher_host, self._joint_angle_publish_port)
        self._publisher_manager.get_publisher(self._publisher_host, self._state_publish_port)

        self._reset_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = reset_subscribe_port,
            topic = 'reset'
        )

        self._home_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = home_subscribe_port,
            topic = 'home'
        )

        # Add recording control
        self._is_recording_enabled = False
        
        # Add caches for data collection
        self._latest_joint_angles = None
        self._latest_joint_angles_timestamp = 0
        self._latest_commanded_angles = None
        self._latest_commanded_angles_timestamp = 0
        
        # Reference to the operator will be set when it's available
        self._operator = None

    @property
    def name(self):
        return 'leap'

    @property
    def recorder_functions(self):
        """Define available recording functions for the robot"""
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_position,
            'transformed_keypoints': self.get_transformed_keypoints,
            'joint_angles_rad': self.get_joint_position,
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # Recording control methods
    def start_recording(self):
        """Enable data recording"""
        self._is_recording_enabled = True
        print(f"Recording started for {self.name}")

    def stop_recording(self):
        """Disable data recording"""
        self._is_recording_enabled = False
        print(f"Recording stopped for {self.name}")

    def is_recording(self):
        """Check if recording is enabled"""
        return self._is_recording_enabled

    # State information functions
    def get_joint_state(self):
        """Get current joint state"""
        return {
            'position': np.array(self._state, dtype=np.float32),
            'timestamp': time.time()
        }

    def get_commanded_joint_state(self):
        return self._state

    def get_joint_position(self):
        return self._controller.get_hand_position()

    def get_joint_velocity(self):
        return self._controller.get_hand_velocity()

    def get_joint_torque(self):
        return self._controller.get_hand_torque()

    def get_commanded_joint_position(self):
        """Get current commanded action"""
        if self._action is None:
            return self.get_joint_state()
        return {
            'position': np.array(self._action, dtype=np.float32),
            'timestamp': time.time()
        }

    # Movement functions
    def home(self):
        self._controller.home_hand()

    def move_coords(self, coords):
        """Required by RobotWrapper but not used for Leap hand"""
        # Leap hand doesn't use coordinate-based control
        pass

    def move(self, desired_angles):
        """Non-blocking move command"""
        try:
            if isinstance(desired_angles, dict):
                desired_angles = desired_angles['position']
            
            # Update command queue (clear old commands)
            while not self._command_queue.empty():
                self._command_queue.get_nowait()
            self._command_queue.put(desired_angles)
            
            # Update cache for recording if enabled
            if self._is_recording_enabled:
                self._latest_commanded_angles = desired_angles
                self._latest_commanded_angles_timestamp = time.time()
                
            self._last_command = desired_angles
        except Exception as e:
            print(f"Error queueing command: {e}")

    def _execute_movement_loop(self):
        """Background thread that executes movements"""
        print("Movement thread started")
        
        while True:
            try:
                if not self._command_queue.empty():
                    target = self._command_queue.get()
                    if not self._controller.move_hand(target):
                        # If move failed, put command back in queue
                        self._command_queue.put(target)
                # Sleep should be relative to the frequency described
                time.sleep(1 / self._data_frequency)
            except Exception as e:
                print(f"Movement error: {e}")
                time.sleep(1 / self._data_frequency)
    
    def check_reset(self):
        """Check if a reset message was received"""
        reset_bool = self._reset_subscriber.recv_keypoints()
        if reset_bool is not None:
            print(f"Received reset signal for {self.name}")
            return True
        return False

    def check_home(self):
        """Check if a home message was received"""
        home_bool = self._home_subscriber.recv_keypoints()
        if home_bool is not None:
            print(f"Received home signal for {self.name}")
            return True
        return False
    
    def stream(self):
        """Stream loop for LeapHand"""
        frame_count = 0
        start_time = time.time()
        last_fps_print = start_time
        
        while True:
            try:
                # Get joint angles with non-blocking receive
                joint_angles = self._joint_angle_subscriber.recv_keypoints()
                self._action = joint_angles
                
                if joint_angles is not None:
                    frame_count += 1
                    current_time = time.time()
                    
                    # Print FPS every 5 seconds
                    if current_time - last_fps_print >= 5.0:
                        fps = frame_count / (current_time - last_fps_print)
                        # print(f"Average FPS over last 5 seconds: {fps:.2f}")
                        frame_count = 0
                        last_fps_print = current_time
                    
                    # Process the command
                    self.move(joint_angles)
                
                # Update the joint state cache if recording is enabled
                if self._is_recording_enabled:
                    current_joint_position = self._controller.get_hand_position()
                    self._latest_joint_angles = current_joint_position
                    self._latest_joint_angles_timestamp = time.time()
                
                # Publish current state regardless of recording state
                self._publisher_manager.publish(
                    self._publisher_host,
                    self._joint_angle_publish_port,
                    'joint_angles',
                    self.get_joint_position(),
                )
                self._state = self.get_joint_position()
                
                # Publish comprehensive state data for recording
                self.publish_current_state()
                
                # Check for reset signal
                if self.check_reset():
                    # Automatically start recording after reset
                    self.start_recording()

                # Check for home signal and send robot to home position
                # if self.check_home():
                #     self.home()
                #     pass
                
                time.sleep(1/self._data_frequency)
                
            except Exception as e:
                print(f"Stream error: {e}")
                traceback.print_exc()
                time.sleep(1/self._data_frequency)

    def _get_operator(self):
        """Get or retrieve the operator reference"""
        if self._operator is None:
            self._operator = GlobalRegistry.get('leap_hand_operator')
        return self._operator
        
    def get_transformed_keypoints(self):
        """Get the transformed keypoints from the operator"""
        if not self._is_recording_enabled:
            return None
            
        operator = self._get_operator()
        if operator is None:
            return None
            
        return operator.get_latest_transformed_keypoints()

    def publish_current_state(self):
        """
        Gathers all state data defined in recorder_functions
        and publishes it as a single dictionary via ZMQ using self.name as topic
        """
        current_state_dict = {}
        publish_time = time.time()

        for key, getter_function in self.recorder_functions.items():
            try:
                state_data = getter_function()
                if state_data is not None:
                    current_state_dict[key] = state_data
            except Exception as e:
                print(f"Failed to get state for '{key}' on robot '{self.name}': {e}")
                current_state_dict[key] = None

        current_state_dict['timestamp'] = publish_time

        # Publish the state dictionary using self.name as topic
        try:
            self._publisher_manager.publish(
                self._publisher_host,
                self._state_publish_port,
                self.name,
                current_state_dict,
            )
        except Exception as e:
            print(f"Error publishing state dictionary for robot '{self.name}': {e}")

    def __del__(self):
        cleanup_zmq_resources()