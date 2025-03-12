from beavr.src.ros_links.xarm7_right import DexArmControl 
from .robot import RobotWrapper
from beavr.src.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
import numpy as np
import time
import zmq
class XArm7Right(RobotWrapper):
    def __init__(self, host, endeff_subscribe_port, endeff_publish_port, joint_subscribe_port, reset_subscribe_port, robot_ip):
        """
        Args:
            controller: Robot controller instance (DexArmControl)
            host: Network host address
            endeff_subscribe_port: Port for end effector subscription
            endeff_publish_port: Port for end effector publishing
            joint_subscribe_port: Port for joint state subscription
        """
        self._controller = DexArmControl(ip=robot_ip)
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

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state_from_operator,
            'cartesian_states': self.get_cartesian_state_from_operator,
            'actual_cartesian_states': self.get_robot_actual_cartesian_position,
            'actual_joint_states': self.get_robot_actual_joint_position,
            'commanded_cartesian_state': self.get_cartesian_commanded_position
        }

    @property
    def name(self):
        return 'right_xarm'

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
        cartesian_state = self._cartesian_coords_subscriber.recv_keypoints()
        cartesian_state_dict= dict(
            cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        return cartesian_state_dict
    
    def get_joint_state_from_operator(self):
        joint_state = self._joint_state_subscriber.recv_keypoints()
        joint_state_dict= dict(
            joint_position = np.array(joint_state, dtype=np.float32),
            timestamp = time.time()
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
            print(f"Received data from reset subscriber: {reset_bool}")
            return True
        else:
            return False
    
    def stream(self):
        self._controller.home_arm()
        frame_count = 0
        start_time = time.time()
        last_fps_print = start_time
        
        while True:
            recv_coords = self._cartesian_coords_subscriber.recv_keypoints(zmq.NOBLOCK)
            if recv_coords is not None:
                frame_count += 1
                current_time = time.time()
                
                # Print FPS every 10 seconds
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
                
            if self.check_reset():
                self.send_robot_pose()
            time.sleep(1/self._data_frequency)
