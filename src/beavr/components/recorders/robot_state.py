import os
import time
import h5py
import hydra
import numpy as np
from .recorder import Recorder
from beavr.utils.timer import FrequencyTimer
from beavr.constants import *
from beavr.utils.registry import GlobalRegistry

# To record robot information
class RobotInformationRecord(Recorder):
    def __init__(self, robot_configs, recorder_function_key, storage_path):
        # Create a robot instance in "record-only" mode
        self.robot = hydra.utils.instantiate(robot_configs, _record_mode=True)
        self.record_type = recorder_function_key
        self.storage_path = storage_path
        
        # Verify the robot has the required recorder function
        if not hasattr(self.robot, 'recorder_functions'):
            raise ValueError(f"Robot {type(self.robot)} must implement recorder_functions property")
        
        if self.record_type not in self.robot.recorder_functions:
            raise ValueError(f"Robot {type(self.robot)} does not support recording type {self.record_type}")
        
        # Get the specific recorder function
        self.keypoint_function = self.robot.recorder_functions[self.record_type]
        
        # Initialize storage
        self._init_storage()

    def stream(self):
        # Checking if the keypoint port is active
        print(f'Checking if the keypoint port is active for {self.robot.name}...')
        retry_count = 0
        max_retries = 30
        
        while retry_count < max_retries:
            data = self.keypoint_function()
            if data is not None:
                break
            
            retry_count += 1
            print(f"Waiting for data... {retry_count}/{max_retries}")
            time.sleep(1)
        
        if retry_count >= max_retries:
            print(f"Warning: Could not get data after {max_retries} attempts. Continuing anyway.")
        else:
            print(f'Starting to record {self.record_type} for {self.robot.name}')

        while True:
            try:
                data = self.keypoint_function()
                if data is not None:
                    self._save_data(data)
            except Exception as e:
                print(f"Error recording data: {e}")
                time.sleep(1/60)  # Default fallback frequency

    def _init_storage(self):
        # Timer
        self.timer = FrequencyTimer(self.robot.data_frequency)

        # Storage path for file
        self._filename = '{}_{}'.format(self.robot.name, self.record_type)
        self.notify_component_start('{}'.format(self._filename))
        self._recorder_file_name = os.path.join(self.storage_path, self._filename + '.h5')

        # Initializing data containers
        self.robot_information = dict()

    def _save_data(self, data):
        self.timer.start_loop()
        try:
            for attribute in data.keys():
                if attribute not in self.robot_information.keys():
                    self.robot_information[attribute] = [data[attribute]]
                    continue
                
                self.robot_information[attribute].append(data[attribute])

            self.num_datapoints += 1
            self.timer.end_loop()
        except KeyboardInterrupt:
            self.record_end_time = time.time()
            return

        # Displaying statistics
        self._display_statistics(self.num_datapoints)
        
        # Saving the metadata
        self._add_metadata(self.num_datapoints)

        # Writing to dataset
        print('Compressing keypoint data...')
        with h5py.File(self._recorder_file_name, "w") as file:
            # Main data
            for key in self.robot_information.keys():
                if key != 'timestamp':
                    self.robot_information[key] = np.array(self.robot_information[key], dtype = np.float32)
                    file.create_dataset(key +'s', data = self.robot_information[key], compression="gzip", compression_opts = 6)
                else:
                    self.robot_information['timestamp'] = np.array(self.robot_information['timestamp'], dtype = np.float64)
                    file.create_dataset('timestamps', data = self.robot_information['timestamp'], compression="gzip", compression_opts = 6)

            # Other metadata
            file.update(self.metadata)
        print('Saved keypoint data in {}.'.format(self._recorder_file_name))