# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from typing import Protocol

from beavr.common.robot_devices.robots.configs import (
    AlohaRobotConfig,
    BeavrRobotAdapterConfig,
    KochBimanualRobotConfig,
    KochRobotConfig,
    LeKiwiRobotConfig,
    ManipulatorRobotConfig,
    MossRobotConfig,
    RobotConfig,
    So100RobotConfig,
    So101RobotConfig,
    StretchRobotConfig,
)


def get_arm_id(name, arm_type):
    """Returns the string identifier of a robot arm. For instance, for a bimanual manipulator
    like Aloha, it could be left_follower, right_follower, left_leader, or right_leader.
    """
    return f"{name}_{arm_type}"


class Robot(Protocol):
    # TODO(rcadene, aliberts): Add unit test checking the protocol is implemented in the corresponding classes
    robot_type: str
    features: dict

    def connect(self): ...
    def run_calibration(self): ...
    def teleop_step(self, record_data=False): ...
    def capture_observation(self): ...
    def send_action(self, action): ...
    def disconnect(self): ...


def make_robot_config(robot_type: str, **kwargs) -> RobotConfig:
    if robot_type == "aloha":
        return AlohaRobotConfig(**kwargs)
    elif robot_type == "koch":
        return KochRobotConfig(**kwargs)
    elif robot_type == "koch_bimanual":
        return KochBimanualRobotConfig(**kwargs)
    elif robot_type == "moss":
        return MossRobotConfig(**kwargs)
    elif robot_type == "so100":
        return So100RobotConfig(**kwargs)
    elif robot_type == "so101":
        return So101RobotConfig(**kwargs)
    elif robot_type == "stretch":
        return StretchRobotConfig(**kwargs)
    elif robot_type == "lekiwi":
        return LeKiwiRobotConfig(**kwargs)
    elif robot_type == "beavr_adapter":
        return BeavrRobotAdapterConfig(**kwargs)
    else:
        raise ValueError(f"Robot type '{robot_type}' is not available.")


def make_robot_from_config(config: RobotConfig, xarm_controller=None):
    if isinstance(config, ManipulatorRobotConfig):
        from beavr.common.robot_devices.robots.manipulator import ManipulatorRobot
        return ManipulatorRobot(config)
    elif isinstance(config, LeKiwiRobotConfig):
        from beavr.common.robot_devices.robots.mobile_manipulator import MobileManipulator
        return MobileManipulator(config)
    elif isinstance(config, BeavrRobotAdapterConfig):
        from beavr.common.robot_devices.robots.beavr_robot_adapter import BeavrRobotAdapter
        
        # Use provided controller or the one from config
        controller = xarm_controller or config.xarm_controller
        
        return BeavrRobotAdapter(
            robot_state_host=config.robot_state_host,
            robot_state_port=config.robot_state_port,
            robot_state_topic=config.robot_state_topic,
            actual_state_input_key=config.actual_state_input_key,
            actual_state_payload_key=config.actual_state_payload_key,
            command_input_key=config.command_input_key,
            command_payload_key=config.command_payload_key,
            use_cartesian_action=config.use_cartesian_action,
            input_angles_are_degrees=config.input_angles_are_degrees,
            cameras=config.cameras,
            xarm_controller=controller
        )
    else:
        from beavr.common.robot_devices.robots.stretch import StretchRobot
        return StretchRobot(config)


def make_robot(robot_type: str, **kwargs) -> Robot:
    config = make_robot_config(robot_type, **kwargs)
    return make_robot_from_config(config)
