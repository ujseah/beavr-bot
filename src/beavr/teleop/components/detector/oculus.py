from beavr.teleop.constants import (
    VR_FREQ,
    ARM_LOW_RESOLUTION,
    ARM_HIGH_RESOLUTION,
    ARM_TELEOP_STOP,
    ARM_TELEOP_CONT,
    ABSOLUTE,
    RIGHT,
    LEFT,
    KEYPOINTS,
    BUTTON,
    PAUSE,
    VR_DETECTOR,
    LEFT_HAND_PORT,
    RIGHT_HAND_PORT
)
from beavr.teleop.components import Component
from beavr.teleop.utils.timer import FrequencyTimer
from beavr.teleop.utils.network import create_pull_socket, ZMQPublisherManager
import zmq
import time
import logging


logger = logging.getLogger(__name__)

class OculusVRHandDetector(Component):
    def __init__(self, host, oculus_hand_port, oculus_pub_port, button_port, teleop_reset_port):
        self.notify_component_start(VR_DETECTOR)
        
        # Socket configuration
        self.host = host
        self.oculus_hand_port = oculus_hand_port
        self.oculus_pub_port = oculus_pub_port
        self.button_port = button_port
        self.teleop_reset_port = teleop_reset_port

        # Initialize sockets
        self.sockets = {
            KEYPOINTS: create_pull_socket(host, self.oculus_hand_port),
            BUTTON: create_pull_socket(host, self.button_port),
            PAUSE: create_pull_socket(host, self.teleop_reset_port)
        }
        
        # Initialize publisher
        self.publisher_manager = ZMQPublisherManager.get_instance()
        
        # Initialize timing
        self.timer = FrequencyTimer(VR_FREQ)
        self.last_received = {name: 0 for name in self.sockets}
        
        # Determine hand side based on port
        if self.oculus_hand_port == LEFT_HAND_PORT:
            self.hand_side = LEFT
        elif self.oculus_hand_port == RIGHT_HAND_PORT:
            self.hand_side = RIGHT
        else:
            raise ValueError(f"Invalid hand side: {self.oculus_hand_port}")
        
        logger.info(f"VR detector initialized for {self.hand_side} hand")
        
    def _process_keypoints(self, data):
        """Process raw keypoint data into a list of values."""
        data_str = data.decode().strip()
        is_relative = not data_str.startswith(ABSOLUTE)
        values = [1 if is_relative else 0]
        
        # Parse coordinates (format: <hand>:x,y,z|x,y,z|x,y,z)
        coords = data_str.split(':')[1].strip().split('|')
        for coord in coords:
            values.extend(float(val) for val in coord.split(',')[:3])
            
        return values
        
    def _process_button(self, data):
        """Convert button data to resolution value."""
        return ARM_LOW_RESOLUTION if data == b'Low' else ARM_HIGH_RESOLUTION
        
    def _process_pause(self, data):
        """Convert pause data to teleop status."""
        return ARM_TELEOP_CONT if data == b'Low' else ARM_TELEOP_STOP
        
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
        logger.info(f"Starting VR detector stream for {self.hand_side} hand")
        consecutive_timeouts = 0
        max_timeouts = 50  # 5 seconds of no data before stopping
        
        try:
            while True:
                self.timer.start_loop()
                
                # Receive keypoint data (required)
                keypoint_data = self._receive_data(KEYPOINTS)
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
                if button_data := self._receive_data(BUTTON):
                    button_state = self._process_button(button_data)
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=BUTTON,
                        data=button_state
                    )
                
                # Process and publish pause state
                if pause_data := self._receive_data(PAUSE):
                    pause_state = self._process_pause(pause_data)
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=PAUSE,
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