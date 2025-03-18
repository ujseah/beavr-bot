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
        # Instead of creating new robot instance, get existing one from registry
        robot_name = robot_configs['_target_'].split('.')[-1]  # Get class name
        self.robot = GlobalRegistry.get(robot_name.lower())  # Match the name used in registration
        
        if self.robot is None:
            raise ValueError(f"Robot {robot_name} not found in registry. Make sure robot is initialized before recorder.")
            
        self.record_type = recorder_function_key
        self.storage_path = storage_path
        
        # Verify the robot has the required recorder function
        if not hasattr(self.robot, 'recorder_functions'):
            raise ValueError(f"Robot {type(self.robot)} must implement recorder_functions property")
        
        if self.record_type not in self.robot.recorder_functions:
            raise ValueError(f"Robot {type(self.robot)} does not support recording type {self.record_type}")
        
        # Initialize storage
        self._init_storage()

    def stream(self):
        while True:
            try:
                # Get data using the appropriate recorder function
                data = self.robot.recorder_functions[self.record_type]()
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