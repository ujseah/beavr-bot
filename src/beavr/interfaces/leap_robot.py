from beavr.controllers.leap_control import DexArmControl
from beavr.interfaces.robot import RobotWrapper
import numpy as np
import threading
import time
from queue import Queue
import traceback

from beavr.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
import time
import zmq
from beavr.utils.registry import GlobalRegistry

class LeapHandRobot(RobotWrapper):
    def __init__(self, host, joint_angle_subscribe_port, joint_angle_publish_port, reset_subscribe_port):
        self._controller = DexArmControl()
        self._data_frequency = 60
        self._command_queue = Queue(maxsize=1)
        self._last_command = None
        self._movement_thread = threading.Thread(target=self._execute_movement_loop, daemon=True)
        self._movement_thread.start()

        self._joint_angle_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = joint_angle_subscribe_port,
            topic = 'joint_angles'
        )

        self._joint_angle_publisher = ZMQKeypointPublisher(
            host = host,
            port = joint_angle_publish_port
        )

        self._reset_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = reset_subscribe_port,
            topic = 'reset'
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
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_state,
            'transformed_keypoints': self.get_transformed_keypoints
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
        if not self._is_recording_enabled or self._latest_joint_angles is None:
            return None
            
        return {
            'position': self._latest_joint_angles,
            'timestamp': self._latest_joint_angles_timestamp
        }

    def get_commanded_joint_state(self):
        """Get the last commanded joint state"""
        if not self._is_recording_enabled or self._latest_commanded_angles is None:
            return None
            
        return {
            'position': self._latest_commanded_angles,
            'timestamp': self._latest_commanded_angles_timestamp
        }

    def get_joint_position(self):
        return self._controller.get_hand_position()

    def get_joint_velocity(self):
        return self._controller.get_hand_velocity()

    def get_joint_torque(self):
        return self._controller.get_hand_torque()

    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()

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
        reset_bool = self._reset_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        if reset_bool is not None:
            print(f"Received reset signal for {self.name}")
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
                joint_angles = self._joint_angle_subscriber.recv_keypoints(zmq.NOBLOCK)
                
                if joint_angles is not None:
                    frame_count += 1
                    current_time = time.time()
                    
                    # Print FPS every 5 seconds
                    if current_time - last_fps_print >= 5.0:
                        fps = frame_count / (current_time - last_fps_print)
                        print(f"Average FPS over last 5 seconds: {fps:.2f}")
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
                self._joint_angle_publisher.pub_keypoints(self.get_joint_position(), 'joint_angles')
                
                # Check for reset signal
                if self.check_reset():
                    print(f"Reset signal received for {self.name}")
                    # Automatically start recording
                    self.start_recording()
                
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