"""Auto-generated strongly-typed config for robot `leap_pybullet`."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any

# Shared component configs driven by `configs.constants`
from beavr.teleop.configs_teleop.shared_components import (
    OculusVRHandDetectorCfg,
    TransformHandPositionCoordsCfg,
    Hand2DVisualizerCfg,
)

# Constants (host address, port numbers, …)
from beavr.teleop.configs_teleop import constants as CONST

from beavr.teleop.components.operators.leap_pybullet import LeapHandOperator
from beavr.teleop.interfaces.leap_robot import LeapHandRobot
from beavr.teleop.configs_teleop import TeleopRobotConfig


@dataclass
class LeapHandOperatorCfg:
    host: str = CONST.HOST_ADDRESS
    transformed_keypoints_port: int | str = CONST.KEYPOINT_TRANSFORM_PORT
    joint_angle_subscribe_port: int | str = CONST.JOINT_PUBLISHER_PORT
    joint_angle_publish_port: int | str = CONST.CARTESIAN_COMMAND_PUBLISHER_PORT  # keeps original order
    reset_publish_port: int | str = CONST.TELEOP_RESET_PUBLISH_PORT
    finger_configs: dict[str, Any] = field(default_factory=lambda: {"freeze_index": False, "freeze_middle": False, "freeze_ring": False, "freeze_thumb": False, "no_index": False, "no_middle": False, "no_ring": False, "no_thumb": False, "three_dim": True})
    logging_config: dict[str, Any] = field(default_factory=lambda: {"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "leap"})

    def build(self):
        return LeapHandOperator(host=self.host, transformed_keypoints_port=self.transformed_keypoints_port, joint_angle_subscribe_port=self.joint_angle_subscribe_port, joint_angle_publish_port=self.joint_angle_publish_port, reset_publish_port=self.reset_publish_port, finger_configs=self.finger_configs, logging_config=self.logging_config)

@dataclass
class LeapHandRobotCfg:
    host: str = CONST.HOST_ADDRESS
    joint_angle_subscribe_port: int | str = CONST.CARTESIAN_COMMAND_PUBLISHER_PORT
    joint_angle_publish_port: int | str = CONST.JOINT_PUBLISHER_PORT
    reset_subscribe_port: int | str = CONST.TELEOP_RESET_PUBLISH_PORT
    simulation_mode: bool = False

    def build(self):
        return LeapHandRobot(host=self.host, joint_angle_subscribe_port=self.joint_angle_subscribe_port, joint_angle_publish_port=self.joint_angle_publish_port, reset_subscribe_port=self.reset_subscribe_port, simulation_mode=self.simulation_mode)

@dataclass
@TeleopRobotConfig.register_subclass("leap_pybullet")
class LeapPybulletConfig:
    robot_name: str = 'leap_hand'
    detector: OculusVRHandDetectorCfg = OculusVRHandDetectorCfg()
    transforms: list = field(default_factory=lambda: [TransformHandPositionCoordsCfg()])
    visualizers: list = field(default_factory=lambda: [Hand2DVisualizerCfg(display_plot=False)])
    operators: list = field(default_factory=lambda: [LeapHandOperatorCfg()])
    robots: list = field(default_factory=lambda: [LeapHandRobotCfg(simulation_mode=False)])
    recorded_data: list = field(default_factory=lambda: [["joint_states", "commanded_joint_states"]])

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'operators': [item.build() for item in self.operators],
            'robots': [item.build() for item in self.robots],
            'recorded_data': self.recorded_data,
        }