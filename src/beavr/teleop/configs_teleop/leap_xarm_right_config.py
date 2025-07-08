from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any
from beavr.teleop.components.operators.leap_pybullet import LeapHandOperator
from beavr.teleop.components.operators.xarm7_right import XArm7RightOperator
from beavr.teleop.interfaces.leap_robot import LeapHandRobot
from beavr.teleop.interfaces.xarm7_robot import XArm7Robot
from beavr.teleop.configs_teleop import TeleopRobotConfig
from beavr.teleop.configs_teleop.shared_components import OculusVRHandDetectorCfg, TransformHandPositionCoordsCfg, Hand2DVisualizerCfg


@dataclass
class XArm7RobotCfg:
    host: str = '10.31.152.148'
    is_right_arm: bool = True
    endeff_publish_port: int = 10010
    endeff_subscribe_port: int = 10009
    joint_subscribe_port: int = 10029
    reset_subscribe_port: int = 10009
    home_subscribe_port: int = 10007
    robot_ip: str = '192.168.1.197'
    simulation_mode: bool = False
    state_publish_port: int = 10011
    teleoperation_state_port: int = 8089
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
        return XArm7Robot(
            host=self.host,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            joint_subscribe_port=self.joint_subscribe_port,
            reset_subscribe_port=self.reset_subscribe_port,
            home_subscribe_port=self.home_subscribe_port,
            robot_ip=self.robot_ip,
            is_right_arm=self.is_right_arm,
            simulation_mode=self.simulation_mode,
            state_publish_port=self.state_publish_port,
            teleoperation_state_port=self.teleoperation_state_port,
            recorder_config=self.recorder_config,
        )

@dataclass
class LeapHandRobotCfg:
    host: str = '10.31.152.148'
    joint_angle_subscribe_port: int = 8120
    joint_angle_publish_port: int = 8119
    reset_subscribe_port: int = 8102
    simulation_mode: bool = False
    state_publish_port: int = 10012
    home_suscribe_port: int = 10007
    recorder_config: dict[str, Any] = field(
        default_factory=lambda: {
            "robot_identifier":
            "leap",
            "recorded_data": [
                "joint_states",
                "commanded_joint_states",
                "joint_angles_rad",
            ],
        }
    )

    def build(self):
        return LeapHandRobot(
            host=self.host,
            joint_angle_subscribe_port=self.joint_angle_subscribe_port,
            joint_angle_publish_port=self.joint_angle_publish_port,
            reset_subscribe_port=self.reset_subscribe_port,
            simulation_mode=self.simulation_mode,
            state_publish_port=self.state_publish_port,
            recorder_config=self.recorder_config,
            home_suscribe_port=self.home_suscribe_port,
        )

@dataclass
class XArm7RightOperatorCfg:
    host: str = '10.31.152.148'
    transformed_keypoints_port: int = 8092
    stream_configs: dict[str, Any] = field(default_factory=lambda: {"host": "10.31.152.148", "port": "10005"})
    stream_oculus: bool = True
    endeff_publish_port: int = 10009
    endeff_subscribe_port: int = 10010
    moving_average_limit: int = 1
    use_filter: bool = False
    teleoperation_state_port: int = 8089
    logging_config: dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False,
            "log_dir": "logs",
            "log_poses": True,
            "log_prefix": "xarm",
        }
    )

    def build(self):
        return XArm7RightOperator(
            host=self.host,
            transformed_keypoints_port=self.transformed_keypoints_port,
            stream_configs=self.stream_configs,
            stream_oculus=self.stream_oculus,
            endeff_publish_port=self.endeff_publish_port,
            endeff_subscribe_port=self.endeff_subscribe_port,
            moving_average_limit=self.moving_average_limit,
            use_filter=self.use_filter,
            teleoperation_state_port=self.teleoperation_state_port,
            logging_config=self.logging_config,
        )

@dataclass
class LeapHandOperatorCfg:
    host: str = '10.31.152.148'
    transformed_keypoints_port: int = 8092
    joint_angle_subscribe_port: int = 8119
    joint_angle_publish_port: int = 8120
    reset_publish_port: int = 8102
    finger_configs: dict[str, Any] = field(
        default_factory=lambda: {
            "freeze_index": False,
            "freeze_middle": False,
            "freeze_ring": False,
            "freeze_thumb": False,
            "no_index": False,
            "no_middle": False,
            "no_ring": False,
            "no_thumb": False,
            "three_dim": True,
        }
    )
    logging_config: dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False,
            "log_dir": "logs",
            "log_poses": True,
            "log_prefix": "leap",
        }
    )

    def build(self):
        return LeapHandOperator(
            host=self.host,
            transformed_keypoints_port=self.transformed_keypoints_port,
            joint_angle_subscribe_port=self.joint_angle_subscribe_port,
            joint_angle_publish_port=self.joint_angle_publish_port,
            reset_publish_port=self.reset_publish_port,
            finger_configs=self.finger_configs,
            logging_config=self.logging_config,
        )

@TeleopRobotConfig.register_subclass("leap_xarm_right")
@dataclass
class LeapXarmRightConfig(TeleopRobotConfig):
    robot_name: str = 'leap_xarm7_right_combo'
    detector: OculusVRHandDetectorCfg = OculusVRHandDetectorCfg(
        host='10.31.152.148',
        oculus_hand_port=8087,
        oculus_pub_port=8088,
        button_port=8095,
        teleop_reset_port=8100
    )
    transforms: list = field(
        default_factory=lambda: [
            TransformHandPositionCoordsCfg(
                host='10.31.152.148',
                keypoint_sub_port=8088,
                keypoint_transform_pub_port=8092,
                moving_average_limit=1
            )
        ]
    )
    visualizers: list = field(
        default_factory=lambda: [
            Hand2DVisualizerCfg(
                host='10.31.152.148',
                transformed_keypoint_port=8092,
                oculus_feedback_port=15001,
                display_plot=False
            )
        ]
    )
    robots: list = field(
        default_factory=lambda: [
            XArm7RobotCfg(
                host='10.31.152.148',
                endeff_publish_port=10010,
                endeff_subscribe_port=10009,
                joint_subscribe_port=10029,
                reset_subscribe_port=10009,
                home_subscribe_port=10007,
                robot_ip='192.168.1.197',
                simulation_mode=False,
                state_publish_port=10011,
                teleoperation_state_port=8089,
                recorder_config={
                    "robot_identifier":
                        "right_xarm7",
                        "recorded_data": [
                            "joint_states",
                            "xarm_cartesian_states",
                            "commanded_cartesian_state",
                            "joint_angles_rad"
                        ]
                }
            ),
            LeapHandRobotCfg(
                host='10.31.152.148',
                joint_angle_subscribe_port=8120,
                joint_angle_publish_port=8119,
                reset_subscribe_port=8102,
                simulation_mode=False,
                state_publish_port=10012,
                home_suscribe_port=10007,
                recorder_config={
                    "robot_identifier":
                        "leap",
                        "recorded_data": [
                            "joint_states",
                            "commanded_joint_states",
                            "joint_angles_rad"
                        ]
                    }
            )
        ]
    )
    operators: list = field(
        default_factory=lambda: [
            XArm7RightOperatorCfg(
                host='10.31.152.148',
                transformed_keypoints_port=8092,
                stream_configs={
                    "host": "10.31.152.148",
                    "port": "10005"
                },
                stream_oculus=True,
                endeff_publish_port=10009,
                endeff_subscribe_port=10010,
                moving_average_limit=1,
                use_filter=False,
                teleoperation_state_port=8089,
                logging_config={
                    "enabled": False,
                    "log_dir": "logs",
                    "log_poses": True,
                    "log_prefix": "xarm"
                }
            ),
            LeapHandOperatorCfg(
                host='10.31.152.148',
                transformed_keypoints_port=8092,
                joint_angle_subscribe_port=8119,
                joint_angle_publish_port=8120,
                reset_publish_port=8102,
                finger_configs={
                    "freeze_index": False,
                    "freeze_middle": False,
                    "freeze_ring": False,
                    "freeze_thumb": False,
                    "no_index": False,
                    "no_middle": False,
                    "no_ring": False,
                    "no_thumb": False,
                    "three_dim": True
                },
                logging_config={
                    "enabled": False,
                    "log_dir": "logs",
                    "log_poses": True,
                    "log_prefix": "leap"
                }
            )
        ]
    )
    recorded_data: list = field(
        default_factory=lambda: [
            ["joint_states", "cartesian_states"]
        ]
    )

    def build(self):
        return {
            'robot_name': self.robot_name,
            'detector': self.detector.build(),
            'transforms': [item.build() for item in self.transforms],
            'visualizers': [item.build() for item in self.visualizers],
            'robots': [item.build() for item in self.robots],
            'operators': [item.build() for item in self.operators],
            'recorded_data': self.recorded_data,
        }