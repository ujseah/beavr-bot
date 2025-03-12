from openteach.ros_links.leap_control import DexArmControl
from openteach.robot.robot import RobotWrapper
import numpy as np
import threading
import time
from queue import Queue
import traceback

from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
import time
import zmq
class LeapHand(RobotWrapper):
    def __init__(self, host, joint_angle_subscribe_port, joint_angle_publish_port, reset_subscribe_port):
        self._controller = DexArmControl()
        self._data_frequency = 300
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

    @property
    def name(self):
        return 'leap'

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_state
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        """Get current joint state"""
        return self._controller.get_hand_position()

    def get_commanded_joint_state(self):
        commanded_state = self._controller.get_commanded_hand_state()
        print(f"Debug - Commanded state: {commanded_state}")
        return commanded_state

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

    # NOTE: We can just move the code from move to move_coords right?
    def move_coords(self, coords):
        """Required by RobotWrapper but not used for Leap hand"""
        # Leap hand doesn't use coordinate-based control
        # but we need to implement this method
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
                    # print(f"Joint angles received")
                    frame_count += 1
                    current_time = time.time()
                    
                    # Print FPS every 5 seconds
                    if current_time - last_fps_print >= 5.0:
                        fps = frame_count / (current_time - last_fps_print)
                        print(f"Average FPS over last 5 seconds: {fps:.2f}")
                        frame_count = 0
                        last_fps_print = current_time
                    
                    # NOTE: We can add the publish and the reset functionality but not now
                    self.move(joint_angles)
                
                self._joint_angle_publisher.pub_keypoints(self.get_joint_state(), 'joint_angles')
                
                time.sleep(1/self._data_frequency)
                
            except Exception as e:
                print(f"Stream error: {e}")