"""Auto-generated strongly-typed config for robot `xarm7_left`."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from beavr.teleop.components.detector.left_keypoint_transform import TransformLeftHandPositionCoords
from beavr.teleop.components.detector.oculus import OculusVRHandDetector
from beavr.teleop.components.operators.xarm7_left import XArm7LeftOperator
from beavr.teleop.components.visualizers.visualizer_2d import Hand2DVisualizer
from beavr.teleop.interfaces.xarm7_robot import XArm7Robot
from beavr.teleop.configs_teleop import TeleopRobotConfig


@dataclass
class OculusVRHandDetectorCfg:
    host: str = '10.31.152.148'
    oculus_port: str = '8110'
    unified_pub_port: str = '8088'
    button_port: str = '8095'
    teleop_reset_port: str = '8100'

    def build(self):
        return OculusVRHandDetector(host=self.host, oculus_port=self.oculus_port, unified_pub_port=self.unified_pub_port, button_port=self.button_port, teleop_reset_port=self.teleop_reset_port)

@dataclass
class TransformLeftHandPositionCoordsCfg:
    host: str = '10.31.152.148'
    keypoint_port: str = '8088'
    transformation_port: str = '8093'
    moving_average_limit: int = 1

    def build(self):
        return TransformLeftHandPositionCoords(host=self.host, keypoint_port=self.keypoint_port, transformation_port=self.transformation_port, moving_average_limit=self.moving_average_limit)

@dataclass
class Hand2DVisualizerCfg:
    host: str = '10.31.152.148'
    transformed_keypoint_port: str = '8093'
    oculus_feedback_port: str = '15001'
    display_plot: bool = False

    def build(self):
        return Hand2DVisualizer(host=self.host, transformed_keypoint_port=self.transformed_keypoint_port, oculus_feedback_port=self.oculus_feedback_port, display_plot=self.display_plot)

@dataclass
class XArm7RobotCfg:
    host: str = '10.31.152.148'
    endeff_publish_port: int = 10010
    endeff_subscribe_port: int = 10009
    joint_subscribe_port: int = 10029
    reset_subscribe_port: int = 10009
    robot_ip: str = '192.168.1.237'
    is_right_arm: bool = False

    def build(self):
        return XArm7Robot(host=self.host, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, joint_subscribe_port=self.joint_subscribe_port, reset_subscribe_port=self.reset_subscribe_port, robot_ip=self.robot_ip, is_right_arm=self.is_right_arm)

@dataclass
class XArm7LeftOperatorCfg:
    host: str = '10.31.152.148'
    transformed_keypoints_port: str = '8093'
    stream_configs: dict[str, Any] = field(default_factory=lambda: {"host": "10.31.152.148", "port": "10005"})
    stream_oculus: bool = True
    endeff_publish_port: int = 10009
    endeff_subscribe_port: int = 10010
    moving_average_limit: int = 1
    arm_resolution_port: str = '8088'
    use_filter: bool = False
    teleoperation_reset_port: str = '8088'
    logging_config: dict[str, Any] = field(default_factory=lambda: {"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm"})

    def build(self):
        return XArm7LeftOperator(host=self.host, transformed_keypoints_port=self.transformed_keypoints_port, stream_configs=self.stream_configs, stream_oculus=self.stream_oculus, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, moving_average_limit=self.moving_average_limit, arm_resolution_port=self.arm_resolution_port, use_filter=self.use_filter, teleoperation_reset_port=self.teleoperation_reset_port, logging_config=self.logging_config)

@dataclass
@TeleopRobotConfig.register_subclass("xarm7_left")
class Xarm7LeftConfig:
    robot_name: str = 'xarm7_left'
    detector: OculusVRHandDetectorCfg = OculusVRHandDetectorCfg(host='10.31.152.148', oculus_port='8110', unified_pub_port='8088', button_port='8095', teleop_reset_port='8100')
    transforms: list = field(default_factory=lambda: [TransformLeftHandPositionCoordsCfg(host='10.31.152.148', keypoint_port='8088', transformation_port='8093', moving_average_limit=1)])
    visualizers: list = field(default_factory=lambda: [Hand2DVisualizerCfg(host='10.31.152.148', transformed_keypoint_port='8093', oculus_feedback_port='15001', display_plot=False)])
    robots: list = field(default_factory=lambda: [XArm7RobotCfg(host='10.31.152.148', endeff_publish_port=10010, endeff_subscribe_port=10009, joint_subscribe_port=10029, reset_subscribe_port=10009, robot_ip='192.168.1.237', is_right_arm=False)])
    operators: list = field(default_factory=lambda: [XArm7LeftOperatorCfg(host='10.31.152.148', transformed_keypoints_port='8093', stream_configs={"host": "10.31.152.148", "port": "10005"}, stream_oculus=True, endeff_publish_port=10009, endeff_subscribe_port=10010, moving_average_limit=1, arm_resolution_port='8088', use_filter=False, teleoperation_reset_port='8088', logging_config={"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm"})])

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'robots': [item.build() for item in self.robots],
            'operators': [item.build() for item in self.operators],
        }