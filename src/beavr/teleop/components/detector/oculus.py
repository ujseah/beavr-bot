import logging
import time

import zmq

from beavr.teleop.components import Component
from beavr.teleop.configs.constants import network, ports, robots
from beavr.teleop.utils.network import ZMQPublisherManager, create_pull_socket
from beavr.teleop.utils.timer import FrequencyTimer

logger = logging.getLogger(__name__)

class OculusVRHandDetector(Component):
    """
    OculusVRHandDetector is a component that detects the hand of the user using the Oculus VR headset.
    """
    def __init__(self, host, oculus_hand_port, oculus_pub_port, button_port, teleop_reset_port):
        """
        Initialize the OculusVRHandDetector component.

        Args:
            host: The host address of the Oculus VR headset.
            oculus_hand_port: The port number of the Oculus VR headset.
            oculus_pub_port: The port number of the Oculus VR headset.
            button_port: The port number of the Oculus VR headset.
            teleop_reset_port: The port number of the Oculus VR headset.
        """
        self.notify_component_start(robots.VR_DETECTOR)
        
        # Socket configuration
        self.host = host
        self.oculus_hand_port = oculus_hand_port
        self.oculus_pub_port = oculus_pub_port
        self.button_port = button_port
        self.teleop_reset_port = teleop_reset_port

        # Initialize sockets
        self.sockets = {
            robots.KEYPOINTS: create_pull_socket(host, self.oculus_hand_port),
            robots.BUTTON: create_pull_socket(host, self.button_port),
            robots.PAUSE: create_pull_socket(host, self.teleop_reset_port)
        }
        
        # Initialize publisher
        self.publisher_manager = ZMQPublisherManager.get_instance()
        
        # Initialize timing
        self.timer = FrequencyTimer(robots.VR_FREQ)
        self.last_received = dict.fromkeys(self.sockets, 0)
        
        # Determine hand side based on port
        if self.oculus_hand_port == network.LEFT_HAND_PORT:
            self.hand_side = robots.LEFT
        elif self.oculus_hand_port == network.RIGHT_HAND_PORT:
            self.hand_side = robots.RIGHT
        else:
            raise ValueError(f"Invalid hand side: {self.oculus_hand_port}")
        
        # logger.info(f"VR detector initialized for {self.hand_side} hand")
        
    def _process_keypoints(self, data):
        """Process raw keypoint data into a list of values."""
        data_str = data.decode().strip()
        is_relative = not data_str.startswith(robots.ABSOLUTE)
        values = [1 if is_relative else 0]
        
        # Parse coordinates (format: <hand>:x,y,z|x,y,z|x,y,z)
        coords = data_str.split(':')[1].strip().split('|')
        for coord in coords:
            values.extend(float(val) for val in coord.split(',')[:3])
            
        return values
        
    def _process_button(self, data):
        """Convert button data to resolution value."""
        return robots.ARM_LOW_RESOLUTION if data == b'Low' else robots.ARM_HIGH_RESOLUTION
        
    def _process_pause(self, data):
        """Convert pause data to teleop status."""
        return robots.ARM_TELEOP_CONT if data == b'Low' else robots.ARM_TELEOP_STOP
        
    def _receive_data(self, socket_name):
        """Receive data from a socket with timeout handling."""
        try:
            data = self.sockets[socket_name].recv()
            self.last_received[socket_name] = time.time()
            return data
        except zmq.Again:
            return None
            
    def stream(self):
        """Main streaming loop for VR hand detection."""
        # logger.info(f"Starting VR detector stream for {self.hand_side} hand")
        consecutive_timeouts = 0
        max_timeouts = 50  # 5 seconds of no data before stopping

        try:
            while True:
                self.timer.start_loop()
                # Receive keypoint data (required)
                keypoint_data = self._receive_data(robots.KEYPOINTS)
                if keypoint_data is None:
                    consecutive_timeouts += 1
                    if consecutive_timeouts >= max_timeouts:
                        logger.warning(f"No data received for {max_timeouts} consecutive attempts. Stopping.")
                        break
                    continue
                
                consecutive_timeouts = 0
                
                # Process and publish keypoints
                keypoints = self._process_keypoints(keypoint_data)
                try:
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=self.hand_side,
                        data=keypoints
                    )
                except Exception as e:
                    logger.error(f"Failed to publish keypoints: {e}")
                
                # Process and publish button state
                if button_data := self._receive_data(robots.BUTTON):
                    button_state = self._process_button(button_data)
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=robots.BUTTON,
                        data=button_state
                    )
                
                # Process and publish pause state
                if pause_data := self._receive_data(robots.PAUSE):
                    pause_state = self._process_pause(pause_data)
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=robots.PAUSE,
                        data=pause_state
                    )
                
                self.timer.end_loop()
                
        except Exception as e:
            logger.error(f"Error in stream loop: {e}")
        finally:
            for name, socket in self.sockets.items():
                socket.close()
                logger.info(f"Closed {name} socket")
            logger.info('Stopped VR hand detection process.')


class BimanualOculusVRHandDetector(Component):
    """Single process handling both left and right hand data."""

    def __init__(
        self,
        host: str,
        right_hand_port: int = network.RIGHT_HAND_PORT,
        left_hand_port: int = network.LEFT_HAND_PORT,
        oculus_pub_port: int = ports.KEYPOINT_STREAM_PORT,
        button_port: int = ports.RESOLUTION_BUTTON_PORT,
        teleop_reset_port: int = ports.TELEOP_RESET_PORT,
    ):
        self.notify_component_start(robots.VR_DETECTOR)

        self.host = host
        self.right_hand_port = right_hand_port
        self.left_hand_port = left_hand_port
        self.oculus_pub_port = oculus_pub_port
        self.button_port = button_port
        self.teleop_reset_port = teleop_reset_port

        self.sockets = {
            'right': create_pull_socket(host, self.right_hand_port),
            'left': create_pull_socket(host, self.left_hand_port),
            robots.BUTTON: create_pull_socket(host, self.button_port),
            robots.PAUSE: create_pull_socket(host, self.teleop_reset_port),
        }

        self.publisher_manager = ZMQPublisherManager.get_instance()
        self.timer = FrequencyTimer(robots.VR_FREQ)

    def _process_keypoints(self, data):
        data_str = data.decode().strip()
        is_relative = not data_str.startswith(robots.ABSOLUTE)
        values = [1 if is_relative else 0]
        coords = data_str.split(':')[1].strip().split('|')
        for coord in coords:
            values.extend(float(val) for val in coord.split(',')[:3])
        return values

    def _process_button(self, data):
        return robots.ARM_LOW_RESOLUTION if data == b'Low' else robots.ARM_HIGH_RESOLUTION

    def _process_pause(self, data):
        return robots.ARM_TELEOP_CONT if data == b'Low' else robots.ARM_TELEOP_STOP

    def _receive_data(self, socket_name):
        try:
            return self.sockets[socket_name].recv(zmq.NOBLOCK)
        except zmq.Again:
            return None

    def stream(self):
        try:
            while True:
                self.timer.start_loop()

                for name, topic in [('right', robots.RIGHT), ('left', robots.LEFT)]:
                    if data := self._receive_data(name):
                        keypoints = self._process_keypoints(data)
                        self.publisher_manager.publish(
                            host=self.host,
                            port=self.oculus_pub_port,
                            topic=topic,
                            data=keypoints,
                        )

                if button_data := self._receive_data(robots.BUTTON):
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=robots.BUTTON,
                        data=self._process_button(button_data),
                    )

                if pause_data := self._receive_data(robots.PAUSE):
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=robots.PAUSE,
                        data=self._process_pause(pause_data),
                    )

                self.timer.end_loop()
        finally:
            for socket in self.sockets.values():
                socket.close()
            logger.info('Stopped bimanual VR hand detection process.')