import logging
from typing import Any, Optional

import zmq

from ..subscriber import BaseSubscriber

logger = logging.getLogger(__name__)


class ZMQSubscriber(BaseSubscriber):
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