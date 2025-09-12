import logging
import time

import zmq

from beavr.teleop.common.messaging.publisher import ZMQPublisherManager
from beavr.teleop.common.messaging.utils import create_pull_socket
from beavr.teleop.common.time.timer import FrequencyTimer
from beavr.teleop.components import Component
from beavr.teleop.components.detector.detector_types import (
    ButtonEvent,
    InputFrame,
    SessionCommand,
)
from beavr.teleop.configs.constants import network, ports, robots

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
            self.hand_side = "left"
        elif self.oculus_hand_port == network.RIGHT_HAND_PORT:
            self.hand_side = "right"
        else:
            raise ValueError(f"Invalid hand side: {self.oculus_hand_port}")
                
    def _process_keypoints(self, data):
        """Process raw keypoint data into a list of coordinate values."""
        data_str = data.decode().strip()
        values = []
        
        # Parse coordinates (format: <hand>:x,y,z|x,y,z|x,y,z)
        coords = data_str.split(':')[1].strip().split('|')
        for coord in coords:
            values.extend(float(val) for val in coord.split(',')[:3])
        
        return values

        
    def _receive_data(self, socket_name):
        """Receive data from a socket."""
        try:
            data = self.sockets[socket_name].recv()
            self.last_received[socket_name] = time.time()
            return data
        except zmq.Again:
            return None
            
    def stream(self):
        """Main streaming loop for VR hand detection."""
        consecutive_timeouts = 0
        max_timeouts = 50  # 5 seconds of no data before stopping

        while True:
            self.timer.start_loop()
            #-------------------------------- Receive keypoint data --------------------------------#
            keypoint_data = self._receive_data(robots.KEYPOINTS)
            #---------------------------------------------------------------------------------------#

            if keypoint_data is None:
                consecutive_timeouts += 1
                if consecutive_timeouts >= max_timeouts:
                    logger.warning(f"No data received for {max_timeouts} consecutive attempts. Stopping.")
                    break
                continue
            
            consecutive_timeouts = 0

            # Process and publish keypoints
            keypoints = self._process_keypoints(keypoint_data)
            is_relative = not keypoint_data.decode().strip().startswith(robots.ABSOLUTE)

            #-------------------------------- Publish keypoints --------------------------------#
            self.publisher_manager.publish(
                host=self.host,
                port=self.oculus_pub_port,
                topic=self.hand_side,
                data=InputFrame(
                    timestamp_s=time.time(),
                    hand_side=self.hand_side,
                    keypoints=keypoints,
                    is_relative=is_relative,
                    frame_vectors=None,
                )
            )
            #---------------------------------------------------------------------------------------#

            #-------------------------------- Publish button state --------------------------------#
            # Process and publish button state
            if button_data := self._receive_data(robots.BUTTON):
                self.publisher_manager.publish(
                    host=self.host,
                    port=self.oculus_pub_port,
                    topic=robots.BUTTON,
                    data=ButtonEvent(
                        timestamp_s=time.time(),
                        hand_side=self.hand_side,
                        name=robots.BUTTON,
                        value=robots.ARM_LOW_RESOLUTION if button_data == b'Low' else robots.ARM_HIGH_RESOLUTION,
                    )
                )
            #---------------------------------------------------------------------------------------#

            #-------------------------------- Publish pause state --------------------------------#
            # Process and publish pause state
            if pause_data := self._receive_data(robots.PAUSE):
                self.publisher_manager.publish(
                    host=self.host,
                    port=self.oculus_pub_port,
                    topic=robots.PAUSE,
                    data=SessionCommand(
                        timestamp_s=time.time(),
                        command="resume" if pause_data == b'Low' else "pause",
                    )
                )
            #---------------------------------------------------------------------------------------#
            
            self.timer.end_loop()
            
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
            robots.RIGHT: create_pull_socket(host, self.right_hand_port),
            robots.LEFT: create_pull_socket(host, self.left_hand_port),
            robots.BUTTON: create_pull_socket(host, self.button_port),
            robots.PAUSE: create_pull_socket(host, self.teleop_reset_port),
        }

        self.publisher_manager = ZMQPublisherManager.get_instance()
        self.timer = FrequencyTimer(robots.VR_FREQ)

    def _process_keypoints(self, data):
        data_str = data.decode().strip()
        values = []
        coords = data_str.split(':')[1].strip().split('|')
        for coord in coords:
            values.extend(float(val) for val in coord.split(',')[:3])
        return values

    def _receive_data(self, socket_name):
        try:
            return self.sockets[socket_name].recv(zmq.NOBLOCK)
        except zmq.Again:
            return None

    def stream(self):
        while True:
            self.timer.start_loop()

            for name, topic in [('right', robots.RIGHT), ('left', robots.LEFT)]:
                if data := self._receive_data(name):
                    keypoints = self._process_keypoints(data)
                    is_relative = not data.decode().strip().startswith(robots.ABSOLUTE)
                    self.publisher_manager.publish(
                        host=self.host,
                        port=self.oculus_pub_port,
                        topic=topic,
                        data=InputFrame(
                            timestamp_s=time.time(),
                            hand_side=topic,
                            keypoints=keypoints,
                            is_relative=is_relative,
                            frame_vectors=None,
                        ),
                    )

            if button_data := self._receive_data(robots.BUTTON):
                self.publisher_manager.publish(
                    host=self.host,
                    port=self.oculus_pub_port,
                    topic=robots.BUTTON,
                    data=ButtonEvent(
                        timestamp_s=time.time(),
                        hand_side=topic,
                        name=robots.BUTTON,
                        value=robots.ARM_LOW_RESOLUTION if button_data == b'Low' else robots.ARM_HIGH_RESOLUTION,
                    ),
                )

            if pause_data := self._receive_data(robots.PAUSE):
                self.publisher_manager.publish(
                    host=self.host,
                    port=self.oculus_pub_port,
                    topic=robots.PAUSE,
                    data=SessionCommand(
                        timestamp_s=time.time(),
                        command="resume" if pause_data == b'Low' else "pause",
                    ),
                )

            self.timer.end_loop()
        for socket in self.sockets.values():
            socket.close()
        logger.info('Stopped bimanual VR hand detection process.')
