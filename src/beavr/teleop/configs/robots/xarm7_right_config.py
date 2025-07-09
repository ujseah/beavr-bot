"""Auto-generated strongly-typed config for robot `xarm7_right`."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from beavr.teleop.components.detector.keypoint_transform import TransformHandPositionCoords
from beavr.teleop.components.detector.oculus import OculusVRHandDetector
from beavr.teleop.components.operators.xarm7_right import XArm7RightOperator
from beavr.teleop.components.visualizers.visualizer_2d import Hand2DVisualizer
from beavr.teleop.interfaces.xarm7_robot import XArm7Robot
from beavr.teleop.configs.robots import TeleopRobotConfig

# Import shared constants from centralized modules
from beavr.teleop.configs.constants import network, ports, robots

import logging

logger = logging.getLogger(__name__)

@dataclass
class OculusVRHandDetectorCfg:
    host: str = network.HOST_ADDRESS
    oculus_hand_port: int = ports.RIGHT_HAND_OCULUS_RECEIVER_PORT
    oculus_pub_port: int = ports.KEYPOINT_STREAM_PORT
    button_port: int = ports.RESOLUTION_BUTTON_PORT
    teleop_reset_port: int = ports.TELEOP_RESET_PORT

    def __post_init__(self):
        """Validate port configuration."""
        all_ports = [self.oculus_hand_port, self.oculus_pub_port, self.button_port, self.teleop_reset_port]
        if len(set(all_ports)) != len(all_ports):
            logger.error("Duplicate ports found in OculusVRHandDetector configuration!")
            raise ValueError("Duplicate ports in configuration")
        
        # Validate port ranges
        for port_name, port_value in [
            ("oculus_hand_port", self.oculus_hand_port),
            ("oculus_pub_port", self.oculus_pub_port),
            ("button_port", self.button_port),
            ("teleop_reset_port", self.teleop_reset_port)
        ]:
            if not (1 <= port_value <= 65535):
                raise ValueError(f"{port_name} out of valid range (1-65535): {port_value}")
            
    def build(self):
        return OculusVRHandDetector(
            host=self.host, 
            oculus_hand_port=self.oculus_hand_port, 
            oculus_pub_port=self.oculus_pub_port, 
            button_port=self.button_port, 
            teleop_reset_port=self.teleop_reset_port
        )

@dataclass
class TransformHandPositionCoordsCfg:
    host: str = network.HOST_ADDRESS
    keypoint_sub_port: int = ports.KEYPOINT_STREAM_PORT
    keypoint_transform_pub_port: int = ports.KEYPOINT_TRANSFORM_PORT
    moving_average_limit: int = 1

    def __post_init__(self):
        """Validate configuration."""
        if not (1 <= self.keypoint_sub_port <= 65535):
            raise ValueError(f"keypoint_sub_port out of range: {self.keypoint_sub_port}")
        if not (1 <= self.keypoint_transform_pub_port <= 65535):
            raise ValueError(f"keypoint_transform_pub_port out of range: {self.keypoint_transform_pub_port}")
        if self.moving_average_limit < 1:
            raise ValueError(f"moving_average_limit must be >= 1: {self.moving_average_limit}")

    def build(self):
        return TransformHandPositionCoords(
            host=self.host, 
            keypoint_sub_port=self.keypoint_sub_port, 
            keypoint_transform_pub_port=self.keypoint_transform_pub_port, 
            moving_average_limit=self.moving_average_limit
        )

@dataclass
class Hand2DVisualizerCfg:
    host: str = network.HOST_ADDRESS
    transformed_keypoint_port: int = ports.KEYPOINT_TRANSFORM_PORT
    oculus_feedback_port: int = ports.OCULUS_GRAPH_PORT
    display_plot: bool = False

    def __post_init__(self):
        """Validate port configuration."""
        for port_name, port_value in [
            ("transformed_keypoint_port", self.transformed_keypoint_port),
            ("oculus_feedback_port", self.oculus_feedback_port)
        ]:
            if not (1 <= port_value <= 65535):
                raise ValueError(f"{port_name} out of valid range: {port_value}")

    def build(self):
        return Hand2DVisualizer(
            host=self.host, 
            transformed_keypoint_port=self.transformed_keypoint_port, 
            oculus_feedback_port=self.oculus_feedback_port, 
            display_plot=self.display_plot
        )

@dataclass
class XArm7RobotCfg:
    host: str = network.HOST_ADDRESS
    robot_ip: str = network.RIGHT_XARM_IP
    is_right_arm: bool = True
    endeff_publish_port: int = ports.XARM_ENDEFF_PUBLISH_PORT
    endeff_subscribe_port: int = ports.XARM_ENDEFF_SUBSCRIBE_PORT
    joint_subscribe_port: int = ports.XARM_JOINT_SUBSCRIBE_PORT
    reset_subscribe_port: int = ports.XARM_RESET_SUBSCRIBE_PORT
    state_publish_port: int = ports.XARM_STATE_PUBLISH_PORT
    home_subscribe_port: int = ports.XARM_HOME_SUBSCRIBE_PORT
    teleoperation_state_port: int = ports.XARM_TELEOPERATION_STATE_PORT
    recorder_config: dict[str, Any] = field(
        default_factory=lambda: {
            "robot_identifier": robots.ROBOT_IDENTIFIER_RIGHT_XARM7,
            "recorded_data": [
                robots.RECORDED_DATA_JOINT_STATES,
                robots.RECORDED_DATA_XARM_CARTESIAN_STATES,
                robots.RECORDED_DATA_COMMANDED_CARTESIAN_STATE,
                robots.RECORDED_DATA_JOINT_ANGLES_RAD
            ]
        }
    )

    def __post_init__(self):
        """Validate configuration."""
        # Validate all ports are in valid range
        all_ports = [
            self.endeff_publish_port,
            self.endeff_subscribe_port, 
            self.joint_subscribe_port,
            self.reset_subscribe_port,
            self.state_publish_port,
            self.home_subscribe_port,
            self.teleoperation_state_port
        ]
        for port in all_ports:
            if not (1 <= port <= 65535):
                raise ValueError(f"Port out of valid range (1-65535): {port}")
        
        # Validate IP address format
        ip_parts = self.robot_ip.split(".")
        if len(ip_parts) != 4:
            raise ValueError(f"Invalid IP address format: {self.robot_ip}")

    def build(self):
        return XArm7Robot(
            host=self.host,
            robot_ip=self.robot_ip,
            is_right_arm=self.is_right_arm,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            joint_subscribe_port=self.joint_subscribe_port,
            reset_subscribe_port=self.reset_subscribe_port,
            state_publish_port=self.state_publish_port,
            home_subscribe_port=self.home_subscribe_port,
            recorder_config=self.recorder_config,
            teleoperation_state_port=self.teleoperation_state_port
        )

@dataclass
class XArm7RightOperatorCfg:
    host: str = network.HOST_ADDRESS
    transformed_keypoints_port: int = ports.KEYPOINT_TRANSFORM_PORT
    stream_configs: dict[str, Any] = field(
        default_factory=lambda: {
            "host": network.HOST_ADDRESS, 
            "port": ports.CONTROL_STREAM_PORT
        }
    )
    stream_oculus: bool = True
    endeff_publish_port: int = ports.XARM_ENDEFF_SUBSCRIBE_PORT
    endeff_subscribe_port: int = ports.XARM_ENDEFF_PUBLISH_PORT
    moving_average_limit: int = 1
    arm_resolution_port: int = ports.KEYPOINT_STREAM_PORT
    use_filter: bool = False
    teleoperation_state_port: int = ports.XARM_TELEOPERATION_STATE_PORT
    logging_config: dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False, 
            "log_dir": "logs", 
            "log_poses": True, 
            "log_prefix": "xarm"
        }
    )

    def __post_init__(self):
        """Validate configuration."""
        # Validate ports
        all_ports = [
            self.transformed_keypoints_port,
            self.endeff_publish_port,
            self.endeff_subscribe_port,
            self.arm_resolution_port,
            self.teleoperation_state_port
        ]
        for port in all_ports:
            if not (1 <= port <= 65535):
                raise ValueError(f"Port out of valid range (1-65535): {port}")
        
        if self.moving_average_limit < 1:
            raise ValueError(f"moving_average_limit must be >= 1: {self.moving_average_limit}")

    def build(self):
        return XArm7RightOperator(
            host=self.host,
            transformed_keypoints_port=self.transformed_keypoints_port,
            stream_configs=self.stream_configs,
            stream_oculus=self.stream_oculus,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            moving_average_limit=self.moving_average_limit,
            arm_resolution_port=self.arm_resolution_port,
            use_filter=self.use_filter,
            teleoperation_state_port=self.teleoperation_state_port,
            logging_config=self.logging_config
        )

@dataclass
@TeleopRobotConfig.register_subclass("xarm7_right")
class Xarm7RightConfig:
    robot_name: str = 'xarm7_right'
    detector: OculusVRHandDetectorCfg = field(default_factory=OculusVRHandDetectorCfg)
    transforms: list = field(default_factory=lambda: [TransformHandPositionCoordsCfg()])
    visualizers: list = field(default_factory=lambda: [Hand2DVisualizerCfg()])
    robots: list = field(default_factory=lambda: [XArm7RobotCfg()])
    operators: list = field(default_factory=lambda: [XArm7RightOperatorCfg()])

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'robots': [item.build() for item in self.robots],
            'operators': [item.build() for item in self.operators],
        }