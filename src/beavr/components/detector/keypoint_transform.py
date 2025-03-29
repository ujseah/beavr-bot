import numpy as np
from copy import deepcopy as copy
from beavr.components import Component
from beavr.constants import *
from beavr.utils.vectorops import *
from beavr.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from beavr.utils.timer import FrequencyTimer
from enum import Enum, auto, IntEnum

class HandMode(IntEnum):
    ABSOLUTE = 1
    RELATIVE = 0

class TransformHandPositionCoords(Component):
    def __init__(self, host, keypoint_port, transformation_port, moving_average_limit = 5):
        self.notify_component_start('keypoint position transform')
        
        # Initializing the subscriber for right hand keypoints
        self.original_keypoint_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'right')
        # Initializing the publisher for transformed right hand keypoints
        self.transformed_keypoint_publisher = ZMQKeypointPublisher(host, transformation_port)
        # Timer
        self.timer = FrequencyTimer(VR_FREQ)
        # Define key landmark indices for stable frame calculation
        self.wrist_idx = 0  # Wrist is typically the first point
        self.index_knuckle_idx = OCULUS_JOINTS['knuckles'][0]  # Index finger knuckle
        self.middle_knuckle_idx = OCULUS_JOINTS['knuckles'][1]  # Middle finger knuckle
        self.pinky_knuckle_idx = OCULUS_JOINTS['knuckles'][-1]  # Pinky finger knuckle
        # Moving average queue
        self.moving_average_limit = moving_average_limit
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []
        

    # Function to get the hand coordinates from the VR
    def _get_hand_coords(self):
        data = self.original_keypoint_subscriber.recv_keypoints()
        if data is None:
            return None, None
        if data[0] == HandMode.ABSOLUTE:
            data_type = 'absolute'
        else:
            data_type = 'relative'
        return data_type, np.asanyarray(data[1:]).reshape(OCULUS_NUM_KEYPOINTS, 3)
    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[0]

    # New method to orthogonalize a set of vectors to ensure they form a valid rotation matrix
    def _orthogonalize_frame(self, x_vec, y_vec, z_vec):
        """Ensure three vectors form an orthogonal frame using Gram-Schmidt process"""
        # Ensure x is normalized
        x_vec = normalize_vector(x_vec)
        
        # Make y orthogonal to x
        y_vec = y_vec - np.dot(y_vec, x_vec) * x_vec
        y_vec = normalize_vector(y_vec)
        
        # Make z orthogonal to both x and y
        z_vec = np.cross(x_vec, y_vec)
        z_vec = normalize_vector(z_vec)
        
        return x_vec, y_vec, z_vec

    # Create a stable coordinate frame for the hand using multiple landmarks
    def _get_stable_coord_frame(self, hand_coords):
        """Create a more stable coordinate frame using multiple hand landmarks"""
        wrist = hand_coords[self.wrist_idx]
        v1 = hand_coords[self.index_knuckle_idx] - wrist
        v2 = hand_coords[self.pinky_knuckle_idx] - wrist
        v3 = hand_coords[self.middle_knuckle_idx] - wrist

        # Calculate frame vectors using multiple references
        palm_normal = normalize_vector(np.cross(v1, v3))  # Z direction
        palm_direction = normalize_vector((v1 + v2 + v3) / 3)  # Y direction
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))  # X direction

        # Orthogonalize explicitly to ensure a valid rotation matrix
        x_vec, y_vec, z_vec = self._orthogonalize_frame(cross_product, palm_direction, palm_normal)

        # Return as a list of vectors for compatibility with existing code
        return [x_vec, y_vec, z_vec]

    # Create a coordinate frame for the arm with improved stability
    def _get_stable_hand_dir_frame(self, hand_coords):
        """Create a more stable frame for hand direction using multiple landmarks"""
        wrist = hand_coords[self.wrist_idx]
        v1 = hand_coords[self.index_knuckle_idx] - wrist
        v2 = hand_coords[self.pinky_knuckle_idx] - wrist
        v3 = hand_coords[self.middle_knuckle_idx] - wrist

        # Calculate frame vectors using multiple references
        palm_normal = normalize_vector(np.cross(v1, v3))  # Unity space - Y
        palm_direction = normalize_vector((v1 + v2 + v3) / 3)  # Unity space - Z
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))  # Unity space - X

        # Orthogonalize explicitly
        x_vec, y_vec, z_vec = self._orthogonalize_frame(cross_product, palm_normal, palm_direction)
        
        return [wrist, x_vec, y_vec, z_vec]

    # Legacy method for backward compatibility
    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord):
        """Legacy method kept for backward compatibility"""
        palm_normal = normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Current Z
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Current Y
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))              # Current X
        
        # Orthogonalize for improved stability
        return self._orthogonalize_frame(cross_product, palm_direction, palm_normal)

    # Legacy method for backward compatibility
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord):
        """Legacy method kept for backward compatibility"""
        palm_normal = normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Unity space - Y
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))              # Unity space - X
        
        # Orthogonalize for improved stability
        x_vec, y_vec, z_vec = self._orthogonalize_frame(cross_product, palm_normal, palm_direction)
        
        return [origin_coord, x_vec, y_vec, z_vec]

    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        
        # Use the new, more stable coordinate frame method
        original_coord_frame = self._get_stable_coord_frame(translated_coords)

        # Finding the rotation matrix and rotating the coordinates
        rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        
        # Use the new, more stable hand direction frame method
        hand_dir_frame = self._get_stable_hand_dir_frame(hand_coords)

        return transformed_hand_coords, hand_dir_frame

    def stream(self):
        while True:
            try:
                self.timer.start_loop()
                data_type, hand_coords = self._get_hand_coords()
                if hand_coords is None:
                    continue

                # Shift the points to required axes
                transformed_hand_coords, translated_hand_coord_frame = self.transform_keypoints(hand_coords)

                # Passing the transformed coords into a moving average
                self.averaged_hand_coords = moving_average(
                    transformed_hand_coords, 
                    self.coord_moving_average_queue, 
                    self.moving_average_limit
                )

                # Apply moving average to frame vectors
                self.averaged_hand_frame = moving_average(
                    translated_hand_coord_frame, 
                    self.frame_moving_average_queue, 
                    self.moving_average_limit
                )
                
                # Ensure frame vectors remain orthogonal regardless of data type
                # Keep origin point as is
                origin = self.averaged_hand_frame[0]
                # Extract the rotation vectors
                x_vec = normalize_vector(self.averaged_hand_frame[1])
                y_vec = normalize_vector(self.averaged_hand_frame[2])
                z_vec = normalize_vector(self.averaged_hand_frame[3])
                
                # Re-orthogonalize the frame
                x_vec, y_vec, z_vec = self._orthogonalize_frame(x_vec, y_vec, z_vec)
                
                # Reconstruct orthogonal frame
                self.averaged_hand_frame = [origin, x_vec, y_vec, z_vec]

                self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_coords, 'transformed_hand_coords')

                # Only publish the frame in absolute mode, but it's always orthogonalized
                if data_type == 'absolute':
                    self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_frame, 'transformed_hand_frame')

                self.timer.end_loop()
            except Exception as e:
                print(f"Error in keypoint transform: {e}")
                break
        
        self.original_keypoint_subscriber.stop()
        self.transformed_keypoint_publisher.stop()

        print('Stopping the keypoint position transform process.')