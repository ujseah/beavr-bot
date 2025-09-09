import logging
import pickle
from typing import Any, Optional

import zmq

from ..publisher import BasePublisher
from ..subscriber import BaseSubscriber
from ..utils import SerializationError

logger = logging.getLogger(__name__)

class ZMQKeypointPublisher(BasePublisher):
    """Publisher for keypoint data using multipart messaging."""
    
    def __init__(self, host: str, port: int):
        super().__init__(host, port, zmq.PUB)

    def pub_keypoints(self, keypoint_array: Any, topic_name: str) -> None:
        """Publish keypoints using multipart messaging.
        
        Args:
            keypoint_array: The keypoint data to publish
            topic_name: The topic to publish to
            
        Raises:
            ConnectionError: If socket operation fails
            SerializationError: If serialization fails
        """
        try:
            buffer = pickle.dumps(keypoint_array, protocol=-1)
            try:
                self._socket.send_multipart([
                    topic_name.encode('utf-8'),
                    buffer
                ], zmq.NOBLOCK)
            except zmq.Again:
                logger.warning(f"High water mark reached for {topic_name}, dropping message")
            except zmq.ZMQError as e:
                raise ConnectionError(f"Failed to send keypoints: {e}") from e
        except Exception as e:
            raise SerializationError(f"Failed to serialize keypoints: {e}") from e

class ZMQKeypointSubscriber(BaseSubscriber):
    """Subscriber for keypoint data using multipart messaging."""
    
    def __init__(self, host: str, port: int, topic: str, context: Optional[zmq.Context] = None):
        super().__init__(host, port, topic, zmq.SUB, context)
        self._last_data: Optional[Any] = None
        self.start()

    def process_message(self, data: Any) -> None:
        """Process received keypoint data.
        
        Args:
            data: The received keypoint data
        """
        self._last_data = data

    def recv_keypoints(self) -> Optional[Any]:
        """Get the latest keypoint data.
        
        Returns:
            The latest keypoint data if available, None otherwise
        """
        data = self._last_data
        if data is not None:
            pass
        return data


class ZMQButtonFeedbackSubscriber(BaseSubscriber):
    """Subscriber for button feedback using multipart messaging."""
    
    def __init__(self, host: str, port: int, context: Optional[zmq.Context] = None):
        super().__init__(host, port, "", zmq.SUB, context)  # Empty topic to receive all messages
        self._last_data: Optional[Any] = None
        self.start()

    def process_message(self, data: Any) -> None:
        """Process received button feedback data.
        
        Args:
            data: The received button feedback data
        """
        self._last_data = data

    def recv_keypoints(self) -> Optional[Any]:
        """Get the latest button feedback data.
        
        Returns:
            The latest button feedback data if available, None otherwise
        """
        return self._last_data