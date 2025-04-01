import zmq
import cv2
import base64
import numpy as np
import pickle
import blosc as bl
import threading
import time

# Global ZMQ context (one per process)
_GLOBAL_ZMQ_CONTEXT = zmq.Context()

def get_global_context():
    """Get the global ZMQ context (shared across all sockets)"""
    global _GLOBAL_ZMQ_CONTEXT
    return _GLOBAL_ZMQ_CONTEXT

# ZMQ Sockets
def create_push_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_pull_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_response_socket(host, port):
    content = zmq.Context()
    socket = content.socket(zmq.REP)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_request_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://{}:{}'.format(host, port))
    return socket

# Pub/Sub classes for Keypoints
class ZMQKeypointPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def pub_keypoints(self, keypoint_array, topic_name):
        """
        Process the keypoints into a byte stream and input them in this function
        """
        buffer = pickle.dumps(keypoint_array, protocol = -1)
        self.socket.send(bytes('{} '.format(topic_name), 'utf-8') + buffer)

    def stop(self):
        print('Closing the publisher socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQKeypointSubscriber:
    def __init__(self, host, port, topic):
        self._host, self._port, self._topic = host, port, topic
        self._init_subscriber()

        # Topic chars to remove
        self.strip_value = bytes("{} ".format(self._topic), 'utf-8')

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self._topic, 'utf-8'))

    def recv_keypoints(self, flags=None):
        if flags is None:
            raw_data = self.socket.recv()
            raw_array = raw_data.lstrip(self.strip_value)
            return pickle.loads(raw_array)
        else: # For possible usage of no blocking zmq subscriber
            try:
                raw_data = self.socket.recv(flags)
                raw_array = raw_data.lstrip(self.strip_value)
                return pickle.loads(raw_array)
            except zmq.Again:
                # print('zmq again error')
                return None
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

# Pub/Sub classes for storing data from Realsense Cameras
class ZMQCameraPublisher(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        print('tcp://{}:{}'.format(self._host, self._port))
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))


    def pub_intrinsics(self, array):
        self.socket.send(b"intrinsics " + pickle.dumps(array, protocol = -1))

    def pub_rgb_image(self, rgb_image, timestamp):
        _, buffer = cv2.imencode('.jpg', rgb_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        data = dict(
            timestamp = timestamp,
            rgb_image = base64.b64encode(buffer)
        )
        self.socket.send(b"rgb_image " + pickle.dumps(data, protocol = -1))

    def pub_depth_image(self, depth_image, timestamp):
        compressed_depth = bl.pack_array(depth_image, cname = 'zstd', clevel = 1, shuffle = bl.NOSHUFFLE)
        data = dict(
            timestamp = timestamp,
            depth_image = compressed_depth
        )
        self.socket.send(b"depth_image " + pickle.dumps(data, protocol = -1))

    def stop(self):
        print('Closing the publisher socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQCameraSubscriber(threading.Thread):
    def __init__(self, host, port, topic_type):
        self._host, self._port, self._topic_type = host, port, topic_type
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        print('tcp://{}:{}'.format(self._host, self._port))
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))

        if self._topic_type == 'Intrinsics':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"intrinsics")
        elif self._topic_type == 'RGB':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")
        elif self._topic_type == 'Depth':
            self.socket.setsockopt(zmq.SUBSCRIBE, b"depth_image")

    def recv_intrinsics(self):
        raw_data = self.socket.recv()
        raw_array = raw_data.lstrip(b"intrinsics ")
        return pickle.loads(raw_array)

    def recv_rgb_image(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
        encoded_data = np.fromstring(base64.b64decode(data['rgb_image']), np.uint8)
        return cv2.imdecode(encoded_data, 1), data['timestamp']
        
    def recv_depth_image(self):
        raw_data = self.socket.recv()
        striped_data = raw_data.lstrip(b"depth_image ")
        data = pickle.loads(striped_data)
        depth_image = bl.unpack_array(data['depth_image'])
        return np.array(depth_image, dtype = np.int16), data['timestamp']
        
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

# Publisher for image visualizers
class ZMQCompressedImageTransmitter(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_push_socket()
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def _init_push_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def send_image(self, rgb_image):
        _, buffer = cv2.imencode('.jpg', rgb_image, [int(cv2.IMWRITE_WEBP_QUALITY), 10])
        self.socket.send(np.array(buffer).tobytes())

    def stop(self):
        print('Closing the publisher in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQCompressedImageReciever(threading.Thread):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_pull_socket()
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.subscribe("")

    def _init_pull_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))

    def recv_image(self):
        raw_data = self.socket.recv()
        encoded_data = np.fromstring(raw_data, np.uint8)
        decoded_frame = cv2.imdecode(encoded_data, 1)
        return decoded_frame
        
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class ZMQButtonFeedbackSubscriber(threading.Thread):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_pull_socket()
        self._init_subscriber()

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.subscribe("")

    def _init_pull_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))


    def recv_keypoints(self):
        raw_data = self.socket.recv()
        return pickle.loads(raw_data)
    
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()
# Improved Pub/Sub classes with shared context
class ZMQPublisherManager:
    """Centralized management of ZMQ publishers"""
    _instance = None
    _publishers = {}  # (host, port) -> publisher
    
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = ZMQPublisherManager()
        return cls._instance
    
    def get_publisher(self, host, port):
        """Get or create a publisher for the given host and port"""
        key = (host, port)
        if key not in self._publishers:
            self._publishers[key] = self._create_publisher(host, port)
        return self._publishers[key]
    
    def _create_publisher(self, host, port):
        """Create a new PUB socket"""
        socket = get_global_context().socket(zmq.PUB)
        socket.bind(f'tcp://{host}:{port}')
        print(f"Created new PUB socket at {host}:{port}")
        return socket
    
    def publish(self, host, port, topic, data):
        """Publish data to a topic using the appropriate publisher"""
        publisher = self.get_publisher(host, port)
        buffer = pickle.dumps(data, protocol=-1)
        publisher.send(bytes(f'{topic} ', 'utf-8') + buffer)
    
    def close_all(self):
        """Close all publishers"""
        for key, publisher in self._publishers.items():
            print(f"Closing publisher socket at {key[0]}:{key[1]}")
            publisher.close()
        self._publishers.clear()

# Legacy-compatible wrapper classes
class EnhancedZMQKeypointPublisher(ZMQKeypointPublisher):
    """Enhanced version that uses the centralized publisher manager"""
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._manager = ZMQPublisherManager.get_instance()
        # Don't call super().__init__ to avoid creating a new socket
        
    def pub_keypoints(self, keypoint_array, topic_name):
        """Process the keypoints into a byte stream and publish them"""
        self._manager.publish(self._host, self._port, topic_name, keypoint_array)
        
    def stop(self):
        # Individual publishers don't need to be stopped
        print('Publisher reference released.')

# Direct ZMQ subscriber replacement for EnhancedZMQKeypointSubscriber
class ZMQKeyPointSubscriber:
    """Direct ZMQ subscriber for keypoints without complex management"""
    def __init__(self, host, port, topic):
        self._host, self._port, self._topic = host, port, topic
        self._init_subscriber()
        
        # Topic chars to remove for compatibility with old code
        self.strip_value = bytes("{} ".format(self._topic), 'utf-8')
        
    def _init_subscriber(self):
        self.context = get_global_context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect(f'tcp://{self._host}:{self._port}')
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self._topic)
        
    def recv_keypoints(self, flags=None):
        """Get keypoints with optional non-blocking behavior"""
        try:
            if flags is None:
                raw_data = self.socket.recv()
            else:
                raw_data = self.socket.recv(flags)
                
            raw_array = raw_data.lstrip(self.strip_value)
            return pickle.loads(raw_array)
        except zmq.Again:
            return None
        
    def stop(self):
        """Close the subscriber socket"""
        print(f'Closing the subscriber socket in {self._host}:{self._port}.')
        self.socket.close()

# For backward compatibility - used by existing code
EnhancedZMQKeypointSubscriber = ZMQKeyPointSubscriber

# Function to clean up all ZMQ resources (call on program exit)
def cleanup_zmq_resources():
    """Clean up all ZMQ resources"""
    ZMQPublisherManager.get_instance().close_all()
    _GLOBAL_ZMQ_CONTEXT.term()
