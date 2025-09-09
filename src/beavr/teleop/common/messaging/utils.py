import logging

import zmq

logger = logging.getLogger(__name__)

# Exceptions
class SerializationError(Exception):
    """Exception raised for serialization/deserialization issues"""
    pass

# Global ZMQ context (one per process)
_GLOBAL_ZMQ_CONTEXT = zmq.Context()

def get_global_context() -> zmq.Context:
    """Get the global ZMQ context (shared across all sockets)"""
    global _GLOBAL_ZMQ_CONTEXT
    return _GLOBAL_ZMQ_CONTEXT

def set_global_context(context: zmq.Context) -> None:
    """Set a custom global ZMQ context (useful for testing).
    
    Args:
        context: The ZMQ context to use globally
    """
    global _GLOBAL_ZMQ_CONTEXT
    _GLOBAL_ZMQ_CONTEXT = context

def cleanup_zmq_resources() -> None:
    """Clean up all ZMQ resources gracefully.
    
    This function should be called before process termination to ensure
    proper cleanup of all ZMQ resources, including threads and sockets.
    """
    try:
        # Import locally to avoid circular imports at module import time
        from .handshake import HandshakeCoordinator
        from .publisher import ZMQPublisherManager

        # First stop the publisher manager and its monitor thread
        manager = ZMQPublisherManager.get_instance()
        manager.close_all()
        
        # Clean up handshake coordinator
        HandshakeCoordinator.cleanup_all()
        
        # Then terminate the context
        context = get_global_context()
        context.term()
    except Exception as e:
        logger.error(f"Error during ZMQ cleanup: {e}")
        # In case of error, try to force context termination
        try:
            context = get_global_context()
            context.term(linger=0)
        except Exception as e2:
            logger.error(f"Failed to force terminate ZMQ context: {e2}")

def create_push_socket(host: str, port: int) -> zmq.Socket:
    """Create a PUSH socket with error handling."""
    try:
        socket = get_global_context().socket(zmq.PUSH)
        addr = f'tcp://{host}:{port}'
        socket.bind(addr)
        return socket
    except zmq.ZMQError as e:
        logger.error(f"Failed to create PUSH socket: {e}")
        raise

def create_pull_socket(host: str, port: int) -> zmq.Socket:
    """Create a PULL socket with error handling and connection verification."""
    try:
        socket = get_global_context().socket(zmq.PULL)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout for receiving
        socket.setsockopt(zmq.LINGER, 0)      # Don't wait on close
        socket.setsockopt(zmq.RCVHWM, 1)      # Only keep latest message
        
        addr = f'tcp://{host}:{port}'
        try:
            # Try to bind to the address
            socket.bind(addr)
            
            # Test if the socket is actually bound
            bound_addrs = socket.getsockopt_string(zmq.LAST_ENDPOINT)
            if not bound_addrs:
                raise zmq.ZMQError("Socket binding verification failed")
            
            return socket
            
        except zmq.ZMQError as e:
            if e.errno == zmq.EADDRINUSE:
                logger.error(f"Address {addr} already in use. Is another instance running?")
            else:
                logger.error(f"Failed to bind PULL socket to {addr}: {e}")
            raise
    except zmq.ZMQError as e:
        logger.error(f"Failed to create PULL socket: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error creating PULL socket: {e}")
        raise

def create_response_socket(host: str, port: int) -> zmq.Socket:
    """Create a REP socket with error handling."""
    try:
        socket = get_global_context().socket(zmq.REP)
        addr = f'tcp://{host}:{port}'
        socket.bind(addr)
        return socket
    except zmq.ZMQError as e:
        logger.error(f"Failed to create REP socket: {e}")
        raise

def create_request_socket(host: str, port: int) -> zmq.Socket:
    """Create a REQ socket with error handling."""
    try:
        socket = get_global_context().socket(zmq.REQ)
        addr = f'tcp://{host}:{port}'
        socket.connect(addr)
        return socket
    except zmq.ZMQError as e:
        logger.error(f"Failed to create REQ socket: {e}")
        raise