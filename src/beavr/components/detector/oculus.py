from beavr.constants import VR_FREQ, ARM_LOW_RESOLUTION, ARM_HIGH_RESOLUTION, ARM_TELEOP_STOP, ARM_TELEOP_CONT
from beavr.components import Component
from beavr.utils.timer import FrequencyTimer
from beavr.utils.network import create_pull_socket, ZMQKeypointPublisher

class OculusVRHandDetector(Component):
    def __init__(self, host, oculus_port, unified_pub_port, button_port, teleop_reset_port):
        self.notify_component_start('vr detector') 
        # Input sockets
        self.raw_keypoint_socket = create_pull_socket(host, oculus_port)
        self.button_keypoint_socket = create_pull_socket(host, button_port)
        self.teleop_reset_socket = create_pull_socket(host, teleop_reset_port)
        
        # Store the oculus_port for topic detection
        self.oculus_port = oculus_port
        self.host = host

        # ONE publisher for ALL data - this is the key change
        self.unified_publisher = ZMQKeypointPublisher(
            host=host,
            port=unified_pub_port
        )
        
        self.timer = FrequencyTimer(VR_FREQ)

    # Function to process the data token received from the VR
    def _process_data_token(self, data_token):
        return data_token.decode().strip()

    # Function to Extract the Keypoints from the String Token sent by the VR
    def _extract_data_from_token(self, token):        
        data = self._process_data_token(token)
        information = dict()
        keypoint_vals = [0] if data.startswith('absolute') else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(':')[1].strip().split('|')
        for vector_str in vector_strings:
            vector_vals = vector_str.split(',')
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))
            
        information['keypoints'] = keypoint_vals
        return information

    # Function to Stream the Keypoints
    def stream(self):
        # Determine if we're handling left hand based on the port
        hand_side = 'left' if str(self.oculus_port) == '8110' else 'right'
        print(f"VR detector identified as {hand_side} hand")
        
        while True:
            try:
                self.timer.start_loop()
                # Getting the raw keypoints
                raw_keypoints = self.raw_keypoint_socket.recv()
                # Getting the button feedback
                button_feedback = self.button_keypoint_socket.recv()
                # Getting the Teleop Reset Status
                pause_status = self.teleop_reset_socket.recv()
                
                # Processing the keypoints
                keypoint_dict = self._extract_data_from_token(raw_keypoints)
                
                # Analyzing the resolution based on Button Feedback 
                if button_feedback == b'Low':
                    button_feedback_num = ARM_LOW_RESOLUTION
                else:
                    button_feedback_num = ARM_HIGH_RESOLUTION
                    
                # Analyzing the Teleop Reset Status
                if pause_status == b'Low':
                    pause_status = ARM_TELEOP_STOP 
                else:
                    pause_status = ARM_TELEOP_CONT
                
                # Publish ALL data through ONE publisher with different topics
                # 1. Hand keypoints
                self.unified_publisher.pub_keypoints(
                    keypoint_array=keypoint_dict['keypoints'], 
                    topic_name=hand_side  # Use the detected hand side
                )
                
                # 2. Button feedback
                self.unified_publisher.pub_keypoints(
                    keypoint_array=button_feedback_num, 
                    topic_name='button'
                )
                
                # 3. Pause status
                self.unified_publisher.pub_keypoints(
                    keypoint_array=pause_status, 
                    topic_name='pause'
                )
                
                self.timer.end_loop()
            except Exception as e:
                print(f"Error in oculus stream: {e}")
                break

        self.raw_keypoint_socket.close()
        self.unified_publisher.stop()
        print('Stopping the oculus keypoint extraction process.')