import logging
from typing import Generic, Optional, Type, TypeVar

import zmq

from ..serialization import Serializer
from ..subscriber import BaseSubscriber

T = TypeVar("T")

logger = logging.getLogger(__name__)


class ZMQSubscriber(BaseSubscriber[T], Generic[T]):
    """Subscriber for keypoint data using multipart messaging.

    Provide ``message_type`` to enable static and runtime typing of payloads.
    """

    def __init__(
        self,
        host: str,
        port: int,
        topic: str,
        context: Optional[zmq.Context] = None,
        message_type: Optional[Type[T]] = None,
        serializer: Optional[Serializer[T]] = None,
    ):
        super().__init__(
            host,
            port,
            topic,
            zmq.SUB,
            context,
            message_type=message_type,
            serializer=serializer,
        )
        self._last_data: Optional[T] = None
        self.start()

    def process_message(self, data: T) -> None:
        """Process received keypoint data.

        Args:
            data: The received keypoint data
        """
        self._last_data = data

    def recv_keypoints(self) -> Optional[T]:
        """Get the latest keypoint data.

        Returns:
            The latest keypoint data if available, None otherwise
        """
        data = self._last_data
        if data is not None:
            pass
        return data


class ZMQButtonFeedbackSubscriber(BaseSubscriber[T], Generic[T]):
    """Subscriber for button/feedback using multipart messaging.

    Optionally supply ``message_type`` for typed payloads.
    """

    def __init__(
        self,
        host: str,
        port: int,
        context: Optional[zmq.Context] = None,
        message_type: Optional[Type[T]] = None,
        serializer: Optional[Serializer[T]] = None,
    ):
        super().__init__(
            host,
            port,
            "",
            zmq.SUB,
            context,
            message_type=message_type,
            serializer=serializer,
        )  # Empty topic to receive all messages
        self._last_data: Optional[T] = None
        self.start()

    def process_message(self, data: T) -> None:
        """Process received button feedback data.

        Args:
            data: The received button feedback data
        """
        self._last_data = data

    def recv_keypoints(self) -> Optional[T]:
        """Get the latest button feedback data.

        Returns:
            The latest button feedback data if available, None otherwise
        """
        return self._last_data
