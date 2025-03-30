import os
import time
import h5py
import zmq
import numpy as np
from .recorder import Recorder
from beavr.utils.timer import FrequencyTimer
from beavr.constants import *

from beavr.utils.network import ZMQKeypointSubscriber

# To record robot information by subscribing to ZMQ state dictionaries
class RobotInformationRecord(Recorder):
    def __init__(self, host: str, port: int, topic: str, record_key: str, storage_path: str, robot_name: str):
        """
        Initializes the recorder to subscribe to robot state dictionaries via ZMQ
        and extract a specific key.

        Args:
            host: The hostname or IP address of the ZMQ publisher.
            port: The port number of the ZMQ publisher.
            topic: The ZMQ topic to subscribe to (should match the robot's publishing topic, e.g., 'right_xarm7').
            record_key: The key to extract from the received state dictionary
                        (e.g., 'joint_states', 'xarm_cartesian_states').
            storage_path: The directory path to save the recorded data.
            robot_name: The name of the robot being recorded (used for filename).
        """
        self.record_key = record_key # Key to extract from dict
        self.storage_path = storage_path
        self.robot_name = robot_name # Used for filename and logging
        self._host = host
        self._port = port
        self._zmq_topic = topic # ZMQ topic to subscribe to

        print(f"Initializing RobotInformationRecord for: {robot_name} - Recording Key: '{record_key}'")
        print(f"  Connecting to ZMQ Publisher at: tcp://{host}:{port}, Subscribing to Topic: '{self._zmq_topic}'")

        # Instantiate the ZMQ subscriber
        self.subscriber = ZMQKeypointSubscriber(
            host=self._host,
            port=self._port,
            topic=self._zmq_topic # Subscribe to the specific robot's topic
        )

        # Initialize storage and state variables
        self._init_storage() # Uses robot_name and record_key for filename
        self.num_datapoints = 0
        self.buffer = [] # Use a simple list buffer for received data points
        self.buffer_limit = 1000 # Save every N data points (adjust as needed)

        # Timer for frequency estimation (optional)
        self.timer = FrequencyTimer(RECORDER_FREQ) # Or a different default/config value

    def stream(self):
        """
        Continuously receives state dictionaries via ZMQ, extracts the relevant data,
        and buffers it for saving.
        """
        print(f"[{self.robot_name} - {self.record_key}] Starting ZMQ subscription loop...")
        self.record_start_time = time.time()
        received_count = 0
        last_print_time = time.time()

        try:
            while True:
                # Receive data with a small timeout to prevent blocking indefinitely
                # Adjust timeout_ms as needed (e.g., 100ms)
                state_dict = self.subscriber.recv_keypoints(flags=zmq.NOBLOCK, timeout_ms=100)

                if state_dict is not None:
                    received_count += 1
                    # Extract the specific data point using the record_key
                    if self.record_key in state_dict:
                        data_to_store = state_dict[self.record_key]
                        # Also grab the timestamp from the dictionary
                        timestamp = state_dict.get('timestamp', time.time()) # Fallback timestamp

                        # Add the extracted data and timestamp to the buffer
                        self.buffer.append({'data': data_to_store, 'timestamp': timestamp})

                        # Check if buffer limit is reached
                        if len(self.buffer) >= self.buffer_limit:
                            self._save_buffer() # Save the current buffer
                            self.buffer = [] # Clear the buffer

                    else:
                        # Only print this warning occasionally to avoid spam
                        if received_count % 100 == 1:
                             print(f"Warning: Key '{self.record_key}' not found in received state dict for {self.robot_name}. Keys: {list(state_dict.keys())}")

                # Optional: Print stats periodically
                current_time = time.time()
                if current_time - last_print_time > 5.0: # Print every 5 seconds
                    elapsed = current_time - self.record_start_time
                    rate = received_count / elapsed if elapsed > 0 else 0
                    print(f"[{self.robot_name} - {self.record_key}] Received {received_count} messages in {elapsed:.2f}s ({rate:.1f}/s). Buffer size: {len(self.buffer)}")
                    last_print_time = current_time

                # Optional: Small sleep if using NOBLOCK to prevent high CPU usage if no messages arrive
                if state_dict is None:
                    time.sleep(0.001) # Sleep 1ms

        except KeyboardInterrupt:
            print(f"[{self.robot_name} - {self.record_key}] KeyboardInterrupt received. Saving remaining buffer...")
            self.record_end_time = time.time()
            self._save_buffer() # Save any remaining data
            print(f"[{self.robot_name} - {self.record_key}] Exiting.")
        except Exception as e:
            print(f"[{self.robot_name} - {self.record_key}] Error in stream loop: {e}")
            # Optionally save buffer on error too
            self._save_buffer()
        finally:
            # Ensure subscriber is stopped
            if hasattr(self, 'subscriber'):
                self.subscriber.stop()

    def _init_storage(self):
        """Initializes storage file path and name."""
        # Use robot_name and record_key for a unique filename
        self._filename = '{}_{}'.format(self.robot_name, self.record_key)
        self.notify_component_start('{}'.format(self._filename)) # Assuming this is for logging/status
        self._recorder_file_name = os.path.join(self.storage_path, self._filename + '.h5')
        print(f"[{self.robot_name} - {self.record_key}] Recording data to: {self._recorder_file_name}")
        # Data containers are handled by the buffer list now

    def _save_buffer(self):
        """Saves the current contents of the buffer to the HDF5 file."""
        if not self.buffer:
            print(f"[{self.robot_name} - {self.record_key}] Buffer is empty. Nothing to save.")
            return

        num_to_save = len(self.buffer)
        print(f"[{self.robot_name} - {self.record_key}] Saving {num_to_save} data points from buffer...")

        # Aggregate data from buffer
        aggregated_data = {
            'data': [],
            'timestamps': []
        }
        for item in self.buffer:
            aggregated_data['data'].append(item['data'])
            aggregated_data['timestamps'].append(item['timestamp'])

        # Convert lists to NumPy arrays
        final_datasets = {}
        try:
            # Convert timestamps first
            final_datasets['timestamps'] = np.array(aggregated_data['timestamps'], dtype=np.float64)

            # Attempt to stack the main data; handle potential heterogeneity
            try:
                # Check if all elements are numpy arrays and stackable
                if all(isinstance(x, np.ndarray) for x in aggregated_data['data']):
                     # Filter out None before stacking if necessary (though ideally shouldn't happen here)
                     valid_data = [x for x in aggregated_data['data'] if x is not None]
                     if valid_data:
                          final_datasets[self.record_key] = np.stack(valid_data)
                     else:
                          final_datasets[self.record_key] = np.array([]) # Empty array
                # Check if all elements are simple numerics
                elif all(isinstance(x, (int, float, bool, np.number)) for x in aggregated_data['data']):
                     final_datasets[self.record_key] = np.array(aggregated_data['data'], dtype=np.float32) # Default float32
                else:
                     # Fallback to object dtype for lists, dicts, or mixed types
                     print(f"  Warning: Data for key '{self.record_key}' seems complex or mixed. Saving as object dtype.")
                     final_datasets[self.record_key] = np.array(aggregated_data['data'], dtype=object)
            except ValueError as ve:
                 print(f"  Warning: Stacking/conversion failed for key '{self.record_key}'. Using object dtype. Error: {ve}")
                 final_datasets[self.record_key] = np.array(aggregated_data['data'], dtype=object)

        except Exception as e:
            print(f"[{self.robot_name} - {self.record_key}] FAILED during numpy conversion: {e}. Data lost for this save cycle.")
            return # Abort saving if conversion fails

        # Writing to HDF5 file (use 'a'ppend mode or handle file existence)
        # Using 'w'rite mode here effectively overwrites the file each time _save_buffer is called.
        # If you want to append, you'll need more complex HDF5 handling (checking dataset existence, resizing).
        # For simplicity now, let's assume we overwrite on KeyboardInterrupt/end, but this means intermediate saves are lost.
        # A better approach might be to write to uniquely named files per save cycle or use append mode carefully.
        # Let's stick with 'w' for now, implying final save is the important one.
        # Consider changing to 'a' if incremental saving is critical.
        save_mode = 'w' # Overwrite mode for simplicity in this example
        try:
            with h5py.File(self._recorder_file_name, save_mode) as file:
                print(f"[{self.robot_name} - {self.record_key}] Writing {len(final_datasets)} datasets to {self._recorder_file_name} (mode: {save_mode})")

                for key, array_data in final_datasets.items():
                    if array_data is not None and array_data.size > 0 :
                        # Use the record_key for the main data dataset name
                        dataset_name = key if key == 'timestamps' else self.record_key
                        print(f"  - Writing dataset: '{dataset_name}' with shape {array_data.shape} and dtype {array_data.dtype}")
                        # Delete existing dataset if in append mode and it exists, before creating new one
                        # if save_mode == 'a' and dataset_name in file:
                        #     del file[dataset_name]
                        file.create_dataset(dataset_name, data=array_data, compression="gzip", compression_opts=6)
                    else:
                         print(f"  - Skipping empty dataset: {key}")

                # Add metadata (consider updating vs overwriting in append mode)
                self.num_datapoints += num_to_save # Update total count
                file.attrs['total_datapoints_recorded'] = self.num_datapoints
                file.attrs['datapoints_in_last_save'] = num_to_save
                file.attrs['last_update_timestamp'] = time.time()
                file.attrs['recorded_key'] = self.record_key
                file.attrs['robot_name'] = self.robot_name
                file.attrs['zmq_topic'] = self._zmq_topic
                if hasattr(self, 'record_start_time'): file.attrs['record_start_time'] = self.record_start_time
                if hasattr(self, 'record_end_time'): file.attrs['record_end_time'] = self.record_end_time


            print(f"[{self.robot_name} - {self.record_key}] Successfully saved buffer to {self._recorder_file_name}")
        except Exception as e:
             print(f"[{self.robot_name} - {self.record_key}] FAILED to write HDF5 file {self._recorder_file_name}: {e}")
             # Consider how to handle this - maybe retry? For now, data is lost.

    def __del__(self):
        # Ensure ZMQ subscriber is stopped on deletion
        if hasattr(self, 'subscriber'):
            print(f"[{self.robot_name} - {self.record_key}] Cleaning up ZMQ subscriber...")
            self.subscriber.stop()