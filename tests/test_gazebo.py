from openteach.components.detector.oculus import OculusVRHandDetector
import time
import threading
import zmq

def test_vr_connection():
    host = "10.29.187.186"  # Your host
    oculus_port = 8087
    keypoint_port = 8088
    
    try:
        print("\nTesting VR connection with:")
        print(f"Host: {host}")
        print(f"Oculus port: {oculus_port}")
        print(f"Keypoint port: {keypoint_port}")
        
        detector = OculusVRHandDetector(
            host=host,
            oculus_port=oculus_port,
            keypoint_pub_port=keypoint_port,
            button_port=8095,
            button_publish_port=8093,
            teleop_reset_port=8100,
            teleop_reset_publish_port=8102
        )
        
        print("\nVR detector initialized")
        print("Starting stream for 10 seconds...")
        
        # Create a subscriber to monitor the published keypoints
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.connect(f"tcp://{host}:{keypoint_port}")
        subscriber.setsockopt(zmq.SUBSCRIBE, b"right")  # Changed to binary
        
        # Set non-blocking mode with 1 second timeout
        subscriber.RCVTIMEO = 1000
        
        # Start streaming in a separate thread
        stream_thread = threading.Thread(target=detector.stream)
        stream_thread.daemon = True
        stream_thread.start()
        
        # Monitor for data for 10 seconds
        start_time = time.time()
        data_received = False
        
        while time.time() - start_time < 10:
            try:
                # Receive raw bytes for topic
                topic = subscriber.recv()
                # Receive the actual data
                data = subscriber.recv()
                
                try:
                    # Try to decode if it's text
                    topic = topic.decode('utf-8')
                    data_str = data.decode('utf-8')
                    print(f"\nReceived data on topic {topic}:")
                    print(f"Data: {data_str}")
                except UnicodeDecodeError:
                    # If it's binary data, show the raw bytes
                    print(f"\nReceived binary data on topic {topic}:")
                    print(f"Raw data (bytes): {data[:100]}...")  # Show first 100 bytes
                
                data_received = True
            except zmq.Again:
                print(".", end="", flush=True)
                continue
            except Exception as e:
                print(f"\nError receiving data: {e}")
                
        print("\n\nTest completed.")
        if not data_received:
            print("⚠️  No data was received from the VR system!")
            print("Please check:")
            print("1. Is the VR headset connected?")
            print("2. Is the VR software running?")
            print("3. Are the ports correct and available?")
            print(f"4. Is the host IP ({host}) reachable from your machine?")
            
    except Exception as e:
        print(f"Error in VR connection: {e}")
    finally:
        subscriber.close()
        context.term()

if __name__ == '__main__':
    test_vr_connection()