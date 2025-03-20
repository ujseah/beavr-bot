from beavr.controllers.xarm7_control import DexArmControl 
from .robot import RobotWrapper
from beavr.utils.network import EnhancedZMQKeypointSubscriber as ZMQKeypointSubscriber
from beavr.utils.network import EnhancedZMQKeypointPublisher as ZMQKeypointPublisher
import numpy as np
import time
import zmq

class XArm7Robot(RobotWrapper):
    def __init__(self, host, endeff_subscribe_port, endeff_publish_port, joint_subscribe_port, 
                 reset_subscribe_port, robot_ip, is_right_arm=True):
        """
        Args:
            controller: Robot controller instance (DexArmControl)
            host: Network host address
            endeff_subscribe_port: Port for end effector subscription
            endeff_publish_port: Port for end effector publishing
            joint_subscribe_port: Port for joint state subscription
            reset_subscribe_port: Port for reset subscription
            robot_ip: IP address of the XArm robot
            is_right_arm: Whether this is the right arm (True) or left arm (False)
        """
        self._controller = DexArmControl(ip=robot_ip)
        self._is_right_arm = is_right_arm
        self._data_frequency = 90
        self._cartesian_coords_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = endeff_subscribe_port,
            topic = 'endeff_coords'
        )
        self._cartesian_state_publisher = ZMQKeypointPublisher(
            host = host, 
            port = endeff_publish_port
        )
        self._joint_state_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = joint_subscribe_port,
            topic = 'joint'
        )
        self._reset_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = reset_subscribe_port,
            topic = 'reset'
        )

        # Add caches for received messages
        self._latest_cartesian_coords = None
        self._latest_joint_state = None
        self._latest_cartesian_state_timestamp = 0
        self._latest_joint_state_timestamp = 0
        
        # Recording control
        self._is_recording_enabled = False

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_robot_actual_joint_position,
            'operator_cartesian_states': self.get_cartesian_state_from_operator,
            'xarm_cartesian_states': self.get_robot_actual_cartesian_position,
            'commanded_cartesian_state': self.get_cartesian_commanded_position
        }

    @property
    def name(self):
        return 'right_xarm7' if self._is_right_arm else 'left_xarm7'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_arm_joint_state()
    
    def get_joint_velocity(self):
        return self._controller.get_arm_velocity()

    def get_joint_torque(self):
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
        cartesian_state = self._cartesian_state_subscriber.recv_keypoints()
        cartesian_state_dict= dict(
            commanded_cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        return cartesian_state_dict

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
        self._cartesian_state_publisher.pub_keypoints(cartesian_state, "endeff_homo")

    def check_reset(self):
        reset_bool = self._reset_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        if reset_bool is not None:
            return True
        else:
            return False

    # Modified stream method with automatic recording start after reset
    def stream(self):
        self._controller.home_arm()
        frame_count = 0
        start_time = time.time()
        last_fps_print = start_time
        
        while True:
            recv_coords = self._cartesian_coords_subscriber.recv_keypoints(zmq.NOBLOCK)
            if recv_coords is not None:
                # Always process commands
                frame_count += 1
                current_time = time.time()
                
                # Print FPS every 5 seconds
                if current_time - last_fps_print >= 5.0:
                    fps = frame_count / (current_time - last_fps_print)
                    print(f"Average FPS over last 5 seconds: {fps:.2f}")
                    frame_count = 0  # Reset counter
                    last_fps_print = current_time
                
                # Process the frame
                cartesian_coords = np.concatenate([
                    recv_coords['position'],
                    recv_coords['orientation']
                ])
                self.move_coords(cartesian_coords)
                
                # Always update cache for data collection
                self._latest_cartesian_coords = recv_coords
                self._latest_cartesian_state_timestamp = time.time()
            
            if self.check_reset():
                self.send_robot_pose()
            
            time.sleep(1/self._data_frequency)
