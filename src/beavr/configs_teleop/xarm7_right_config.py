"""Auto-generated strongly-typed config for robot `xarm7_right`."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from beavr.components.detector.keypoint_transform import TransformHandPositionCoords
from beavr.components.detector.oculus import OculusVRHandDetector
from beavr.components.operators.xarm7_right import XArm7RightOperator
from beavr.components.visualizers.visualizer_2d import Hand2DVisualizer
from beavr.interfaces.xarm7_robot import XArm7Robot
from beavr.configs_teleop import TeleopRobotConfig
import logging

logger = logging.getLogger(__name__)

@dataclass
class OculusVRHandDetectorCfg:
    host: str = '10.31.152.148'
    oculus_hand_port: int = 8087
    oculus_pub_port: int = 8088
    button_port: int = 8095
    teleop_reset_port: int = 8100

    def build(self):
        # Validate port configuration
        all_ports = [self.oculus_hand_port, self.oculus_pub_port, self.button_port, self.teleop_reset_port]
        if len(set(all_ports)) != len(all_ports):
            logger.error("Duplicate ports found in OculusVRHandDetector configuration!")
            raise ValueError("Duplicate ports in configuration")
            
        return OculusVRHandDetector(
            host=self.host, 
            oculus_hand_port=self.oculus_hand_port, 
            oculus_pub_port=self.oculus_pub_port, 
            button_port=self.button_port, 
            teleop_reset_port=self.teleop_reset_port
        )

@dataclass
class TransformHandPositionCoordsCfg:
    host: str = '10.31.152.148'
    keypoint_sub_port: int = 8088
    keypoint_transform_pub_port: int = 8092
    moving_average_limit: int = 1

    def build(self):
        return TransformHandPositionCoords(host=self.host, keypoint_sub_port=self.keypoint_sub_port, keypoint_transform_pub_port=self.keypoint_transform_pub_port, moving_average_limit=self.moving_average_limit)

@dataclass
class Hand2DVisualizerCfg:
    host: str = '10.31.152.148'
    transformed_keypoint_port: int = 8092
    oculus_feedback_port: int = 15001
    display_plot: bool = False

    def build(self):
        return Hand2DVisualizer(host=self.host, transformed_keypoint_port=self.transformed_keypoint_port, oculus_feedback_port=self.oculus_feedback_port, display_plot=self.display_plot)

@dataclass
class XArm7RobotCfg:
    host: str = '10.31.152.148'
    robot_ip: str = '192.168.1.197'
    is_right_arm: bool = True
    endeff_publish_port: int = 10010
    endeff_subscribe_port: int = 10009
    joint_subscribe_port: int = 10029
    reset_subscribe_port: int = 10009
    state_publish_port: int = 10011
    home_subscribe_port: int = 10007
    recorder_config: dict[str, Any] = field(
        default_factory=lambda: {
            "robot_identifier": "right_xarm7",
            "recorded_data": [
                "joint_states",
                "xarm_cartesian_states",
                "commanded_cartesian_state",
                "joint_angles_rad"
                ]
            }
        )

    def build(self):
        return XArm7Robot(host=self.host, robot_ip=self.robot_ip, is_right_arm=self.is_right_arm, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, joint_subscribe_port=self.joint_subscribe_port, reset_subscribe_port=self.reset_subscribe_port, state_publish_port=self.state_publish_port, home_subscribe_port=self.home_subscribe_port, recorder_config=self.recorder_config)

@dataclass
class XArm7RightOperatorCfg:
    host: str = '10.31.152.148'
    transformed_keypoints_port: int = 8092
    stream_configs: dict[str, Any] = field(default_factory=lambda: {"host": "10.31.152.148", "port": 10005})
    stream_oculus: bool = True
    endeff_publish_port: int = 10009
    endeff_subscribe_port: int = 10010
    moving_average_limit: int = 1
    arm_resolution_port: int = 8088
    use_filter: bool = False
    teleoperation_reset_port: int = 8088
    logging_config: dict[str, Any] = field(default_factory=lambda: {"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm"})

    def build(self):
        return XArm7RightOperator(host=self.host, transformed_keypoints_port=self.transformed_keypoints_port, stream_configs=self.stream_configs, stream_oculus=self.stream_oculus, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, moving_average_limit=self.moving_average_limit, arm_resolution_port=self.arm_resolution_port, use_filter=self.use_filter, teleoperation_reset_port=self.teleoperation_reset_port, logging_config=self.logging_config)

@dataclass
@TeleopRobotConfig.register_subclass("xarm7_right")
class Xarm7RightConfig:
    robot_name: str = 'xarm7_right'
    detector: OculusVRHandDetectorCfg = OculusVRHandDetectorCfg(host='10.31.152.148', oculus_hand_port=8087, oculus_pub_port=8088, button_port=8095, teleop_reset_port=8100)
    transforms: list = field(default_factory=lambda: [TransformHandPositionCoordsCfg(host='10.31.152.148', keypoint_sub_port=8088, keypoint_transform_pub_port=8092, moving_average_limit=1)])
    visualizers: list = field(default_factory=lambda: [Hand2DVisualizerCfg(host='10.31.152.148', transformed_keypoint_port=8092, oculus_feedback_port=15001, display_plot=False)])
    robots: list = field(default_factory=lambda: [XArm7RobotCfg(host='10.31.152.148', robot_ip='192.168.1.197', is_right_arm=True, endeff_publish_port=10010, endeff_subscribe_port=10009, joint_subscribe_port=10029, reset_subscribe_port=10009, state_publish_port=10011, home_subscribe_port=10007, recorder_config={"robot_identifier": "right_xarm7", "recorded_data": ["joint_states", "xarm_cartesian_states", "commanded_cartesian_state", "joint_angles_rad"]})])
    operators: list = field(default_factory=lambda: [XArm7RightOperatorCfg(host='10.31.152.148', transformed_keypoints_port=8092, stream_configs={"host": "10.31.152.148", "port": 10005}, stream_oculus=True, endeff_publish_port=10009, endeff_subscribe_port=10010, moving_average_limit=1, arm_resolution_port=8088, use_filter=False, teleoperation_reset_port=8088, logging_config={"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm"})])

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'robots': [item.build() for item in self.robots],
            'operators': [item.build() for item in self.operators],
        }