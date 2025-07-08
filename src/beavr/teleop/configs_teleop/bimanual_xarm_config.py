"""Auto-generated strongly-typed config for robot `bimanual_xarm`."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from beavr.teleop.components.operators.xarm7_left import XArm7LeftOperator
from beavr.teleop.components.operators.xarm7_right import XArm7RightOperator
from beavr.teleop.configs_teleop.shared_components import (
    TransformHandPositionCoordsCfg,
    Hand2DVisualizerCfg,
)
from beavr.teleop.interfaces.xarm7_robot import XArm7Robot
from beavr.teleop.configs_teleop import constants as CONST
from beavr.teleop.components.detector.left_keypoint_transform import TransformLeftHandPositionCoords
from beavr.teleop.components.detector.oculusbimanual import OculusVRTwoHandDetector
from beavr.teleop.configs_teleop import TeleopRobotConfig
from beavr.teleop.components.detector.keypoint_transform import TransformHandPositionCoords
from beavr.teleop.components.visualizers.visualizer_2d import Hand2DVisualizer



@dataclass
class OculusVRTwoHandDetectorCfg:
    host: str = CONST.HOST_ADDRESS
    oculus_right_port: str = CONST.RIGHT_HAND_OCULUS_RECEIVER_PORT
    oculus_left_port: str = CONST.LEFT_HAND_OCULUS_RECEIVER_PORT
    keypoint_pub_port: str = CONST.KEYPOINT_STREAM_PORT
    button_port: str = CONST.RESOLUTION_BUTTON_PORT
    button_publish_port: str = CONST.RESOLUTION_BUTTON_PUBLISH_PORT

    def build(self):
        return OculusVRTwoHandDetector(host=self.host, oculus_right_port=self.oculus_right_port, oculus_left_port=self.oculus_left_port, keypoint_pub_port=self.keypoint_pub_port, button_port=self.button_port, button_publish_port=self.button_publish_port)

@dataclass
class TransformHandPositionCoordsCfg:
    host: str = CONST.HOST_ADDRESS
    keypoint_sub_port: int | str = CONST.KEYPOINT_STREAM_PORT
    keypoint_transform_pub_port: int | str = CONST.KEYPOINT_TRANSFORM_PORT
    moving_average_limit: int = 1

    def build(self):
        return TransformHandPositionCoords(
            host=self.host,
            keypoint_sub_port=self.keypoint_sub_port,
            keypoint_transform_pub_port=self.keypoint_transform_pub_port,
            moving_average_limit=self.moving_average_limit,
        )

@dataclass
class TransformLeftHandPositionCoordsCfg:
    host: str = CONST.HOST_ADDRESS
    keypoint_port: int | str = CONST.KEYPOINT_STREAM_PORT
    transformation_port: int | str = CONST.LEFT_KEYPOINT_TRANSFORM_PORT
    moving_average_limit: int = 1

    def build(self):
        return TransformLeftHandPositionCoords(host=self.host, keypoint_port=self.keypoint_port, transformation_port=self.transformation_port, moving_average_limit=self.moving_average_limit)

@dataclass
class Hand2DVisualizerCfg:
    host: str = CONST.HOST_ADDRESS
    transformed_keypoint_port: str = CONST.KEYPOINT_TRANSFORM_PORT
    oculus_feedback_port: str = CONST.OCULUS_GRAPH_PORT
    display_plot: str = '${visualize_right_2d}'

    def build(self):
        return Hand2DVisualizer(host=self.host, transformed_keypoint_port=self.transformed_keypoint_port, oculus_feedback_port=self.oculus_feedback_port, display_plot=self.display_plot)

@dataclass
class XArm7RobotCfg:
    host: str = CONST.HOST_ADDRESS
    robot_ip: str = '192.168.1.197'
    is_right_arm: bool = True
    endeff_publish_port: int = 10010
    endeff_subscribe_port: int = 10009
    joint_subscribe_port: int = 10029
    reset_subscribe_port: int = 10009
    state_publish_port: int = 10011
    recorder_config: dict[str, Any] = field(default_factory=lambda: {"robot_identifier": "right_xarm7", "recorded_data": ["joint_states", "xarm_cartesian_states", "commanded_cartesian_state"]})

    def build(self):
        return XArm7Robot(host=self.host, robot_ip=self.robot_ip, is_right_arm=self.is_right_arm, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, joint_subscribe_port=self.joint_subscribe_port, reset_subscribe_port=self.reset_subscribe_port, state_publish_port=self.state_publish_port, recorder_config=self.recorder_config)

@dataclass
class XArm7RightOperatorCfg:
    host: str = CONST.HOST_ADDRESS
    transformed_keypoints_port: str = CONST.KEYPOINT_TRANSFORM_PORT
    stream_configs: dict[str, Any] = field(default_factory=lambda: {"host": CONST.HOST_ADDRESS, "port": "10005"})
    stream_oculus: bool = True
    endeff_publish_port: int = 10009
    endeff_subscribe_port: int = 10010
    moving_average_limit: int = 1
    arm_resolution_port: int | str = CONST.KEYPOINT_STREAM_PORT
    use_filter: bool = False
    teleoperation_reset_port: int | str = CONST.KEYPOINT_STREAM_PORT
    logging_config: dict[str, Any] = field(default_factory=lambda: {"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm_right"})

    def build(self):
        return XArm7RightOperator(host=self.host, transformed_keypoints_port=self.transformed_keypoints_port, stream_configs=self.stream_configs, stream_oculus=self.stream_oculus, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, moving_average_limit=self.moving_average_limit, arm_resolution_port=self.arm_resolution_port, use_filter=self.use_filter, teleoperation_reset_port=self.teleoperation_reset_port, logging_config=self.logging_config)

@dataclass
class XArm7LeftOperatorCfg:
    host: str = CONST.HOST_ADDRESS
    transformed_keypoints_port: str = CONST.LEFT_KEYPOINT_TRANSFORM_PORT
    stream_configs: dict[str, Any] = field(default_factory=lambda: {"host": CONST.HOST_ADDRESS, "port": "10005"})
    stream_oculus: bool = True
    endeff_publish_port: int = 10011
    endeff_subscribe_port: int = 10012
    moving_average_limit: int = 1
    arm_resolution_port: int | str = CONST.KEYPOINT_STREAM_PORT
    use_filter: bool = False
    teleoperation_reset_port: int | str = CONST.KEYPOINT_STREAM_PORT
    logging_config: dict[str, Any] = field(default_factory=lambda: {"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm_left"})

    def build(self):
        return XArm7LeftOperator(host=self.host, transformed_keypoints_port=self.transformed_keypoints_port, stream_configs=self.stream_configs, stream_oculus=self.stream_oculus, endeff_publish_port=self.endeff_publish_port, endeff_subscribe_port=self.endeff_subscribe_port, moving_average_limit=self.moving_average_limit, arm_resolution_port=self.arm_resolution_port, use_filter=self.use_filter, teleoperation_reset_port=self.teleoperation_reset_port, logging_config=self.logging_config)

@dataclass
@TeleopRobotConfig.register_subclass("bimanual_xarm")
class BimanualXarmConfig:
    robot_name: str = 'bimanual_xarm'
    detector: OculusVRTwoHandDetectorCfg = OculusVRTwoHandDetectorCfg(host=CONST.HOST_ADDRESS, oculus_right_port=CONST.RIGHT_HAND_OCULUS_RECEIVER_PORT, oculus_left_port=CONST.LEFT_HAND_OCULUS_RECEIVER_PORT, keypoint_pub_port=CONST.KEYPOINT_STREAM_PORT, button_port=CONST.RESOLUTION_BUTTON_PORT, button_publish_port=CONST.RESOLUTION_BUTTON_PUBLISH_PORT)
    transforms: list = field(default_factory=lambda: [TransformHandPositionCoordsCfg(), TransformLeftHandPositionCoordsCfg()])
    visualizers: list = field(default_factory=lambda: [Hand2DVisualizerCfg(display_plot=False)])
    robots: list = field(default_factory=lambda: [XArm7RobotCfg(host=CONST.HOST_ADDRESS, robot_ip='192.168.1.197', is_right_arm=True, endeff_publish_port=10010, endeff_subscribe_port=10009, joint_subscribe_port=10029, reset_subscribe_port=10009, state_publish_port=10011, recorder_config={"robot_identifier": "right_xarm7", "recorded_data": ["joint_states", "xarm_cartesian_states", "commanded_cartesian_state"]}), XArm7RobotCfg(host=CONST.HOST_ADDRESS, endeff_publish_port=10012, endeff_subscribe_port=10011, joint_subscribe_port=10030, reset_subscribe_port=10011, robot_ip='192.168.1.237', is_right_arm=False, recorder_config={"robot_identifier": "left_xarm7", "recorded_data": ["joint_states", "xarm_cartesian_states", "commanded_cartesian_state"]})])
    operators: list = field(default_factory=lambda: [XArm7RightOperatorCfg(host=CONST.HOST_ADDRESS, transformed_keypoints_port=CONST.KEYPOINT_TRANSFORM_PORT, stream_configs={"host": CONST.HOST_ADDRESS, "port": "10005"}, stream_oculus=True, endeff_publish_port=10009, endeff_subscribe_port=10010, moving_average_limit=1, arm_resolution_port=CONST.KEYPOINT_STREAM_PORT, use_filter=False, teleoperation_reset_port=CONST.KEYPOINT_STREAM_PORT, logging_config={"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm_right"}), XArm7LeftOperatorCfg(host=CONST.HOST_ADDRESS, transformed_keypoints_port=CONST.LEFT_KEYPOINT_TRANSFORM_PORT, stream_configs={"host": CONST.HOST_ADDRESS, "port": "10005"}, stream_oculus=True, endeff_publish_port=10011, endeff_subscribe_port=10012, moving_average_limit=1, arm_resolution_port=CONST.KEYPOINT_STREAM_PORT, use_filter=False, teleoperation_reset_port=CONST.KEYPOINT_STREAM_PORT, logging_config={"enabled": False, "log_dir": "logs", "log_poses": True, "log_prefix": "xarm_left"})])

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'robots': [item.build() for item in self.robots],
            'operators': [item.build() for item in self.operators],
        }