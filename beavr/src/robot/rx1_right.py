from beavr.src.ros_links.rx1_right import RX1RosLink
from .robot import RobotWrapper
from beavr.src.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
import numpy as np
import time
import zmq

class RX1Right(RobotWrapper):
    def __init__(self, host, endeff_subscribe_port, endeff_publish_port, joint_subscribe_port, reset_subscribe_port, robot_ip=None):
        """
        Args:
            host: Network host address
            endeff_subscribe_port: Port for end effector subscription
            endeff_publish_port: Port for end effector publishing
            joint_subscribe_port: Port for joint state subscription
            reset_subscribe_port: Port for reset subscription
            robot_ip: Not used for ROS implementation
        """
        self._controller = RX1RosLink(robot_type='right')
        self._data_frequency = 90
        self.debug = False  # Debug flag that doesn't depend on ROS
        
        # Network subscribers/publishers
        self._cartesian_coords_subscriber = ZMQKeypointSubscriber(
            host=host, 
            port=endeff_subscribe_port,
            topic='endeff_coords'
        )
        self._cartesian_state_publisher = ZMQKeypointPublisher(
            host=host, 
            port=endeff_publish_port
        )
        self._joint_state_subscriber = ZMQKeypointSubscriber(
            host=host, 
            port=joint_subscribe_port,
            topic='joint'
        )
        self._reset_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=reset_subscribe_port,
            topic='reset'
        )

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state_from_operator,
            'cartesian_states': self.get_cartesian_state_from_operator,
            'actual_joint_states': self.get_robot_actual_joint_position,
            'commanded_cartesian_state': self.get_cartesian_commanded_position
        }

    @property
    def name(self):
        return 'right_rx1'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return {
            'position': self._controller.get_robot_position(),
            'velocity': self._controller.get_robot_velocity(),
            'effort': self._controller.get_robot_torque(),
            'timestamp': time.time()
        }
    
    def get_joint_velocity(self):
        return self._controller.get_robot_velocity()

    def get_joint_torque(self):
        return self._controller.get_robot_torque()
    
    def get_joint_position(self):
        return self._controller.get_robot_position()

    def reset(self):
        return self._controller.reset_robot()
    
    # Movement functions
    def home(self):
        return self._controller.home_robot()

    def move(self, input_angles):
        self._controller.move_robot(input_angles)

    def move_coords(self, cartesian_coords, duration=3):
        """
        Move robot to cartesian pose
        Args:
            cartesian_coords: dict with 'position' and 'orientation' keys
        """
        # Format for ROS controller
        if isinstance(cartesian_coords, dict):
            pose = np.concatenate([
                cartesian_coords['position'],
                cartesian_coords['orientation']
            ])
        else:
            pose = cartesian_coords
        
        self._controller.arm_control(pose)

    def move_velocity(self, input_velocity_values, duration):
        # Not implemented in ROS link yet
        pass

    def get_cartesian_state_from_operator(self):
        cartesian_state = self._cartesian_coords_subscriber.recv_keypoints()
        if cartesian_state is None:
            return None
        
        return {
            'cartesian_position': np.array(cartesian_state['position'], dtype=np.float32),
            'cartesian_orientation': np.array(cartesian_state['orientation'], dtype=np.float32),
            'timestamp': cartesian_state['timestamp']
        }
    
    def get_joint_state_from_operator(self):
        joint_state = self._joint_state_subscriber.recv_keypoints()
        return {
            'joint_position': np.array(joint_state, dtype=np.float32),
            'timestamp': time.time()
        }
    
    def get_cartesian_commanded_position(self):
        cartesian_state = self._cartesian_coords_subscriber.recv_keypoints()
        return {
            'commanded_cartesian_position': np.array(cartesian_state, dtype=np.float32),
            'timestamp': time.time()
        }

    def get_robot_actual_joint_position(self):
        return {
            'joint_position': self._controller.get_robot_position(),
            'timestamp': time.time()
        }
    
    def send_robot_pose(self):
        # This might need modification based on how you want to handle poses in ROS
        pass

    def check_reset(self):
        reset_bool = self._reset_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        if reset_bool is not None:
            print(f"Received data from reset subscriber: {reset_bool}")
            return True
        return False
    
    def stream(self):
        self._controller.home_robot()
        frame_count = 0
        start_time = time.time()
        last_fps_print = start_time
        
        while True:
            # Get current robot state and publish it back
            current_state = self.get_joint_state()
            if current_state:
                robot_pose = {
                    'position': self._controller.get_cartesian_position(),
                    'orientation': self._controller.get_cartesian_orientation(),
                    'timestamp': time.time(),
                    'frame': 'right_palm_lower'
                }
                self._cartesian_state_publisher.pub_keypoints(robot_pose)

            recv_coords = self._cartesian_coords_subscriber.recv_keypoints(zmq.NOBLOCK)
            if recv_coords is not None and isinstance(recv_coords, dict):
                if self.debug:
                    print("="*50)
                    print("RX1Right Received ZMQ Data:")
                    print(f"Raw data: {recv_coords}")
                
                frame_count += 1
                current_time = time.time()
                
                if current_time - last_fps_print >= 5.0:
                    fps = frame_count / (current_time - last_fps_print)
                    if self.debug:
                        print(f"Average FPS over last 5 seconds: {fps:.2f}")
                    frame_count = 0
                    last_fps_print = current_time
                
                # Extract position and orientation from the received dictionary
                cartesian_coords = {
                    'position': recv_coords['position'],
                    'orientation': recv_coords['orientation']
                }
                
                if self.debug:
                    print(f"Sending to ROS: {cartesian_coords}")
                self.move_coords(cartesian_coords)
                if self.debug:
                    print("="*50)
                
            if self.check_reset():
                self.send_robot_pose()
            time.sleep(1/self._data_frequency)
