"""Reusable component configuration dataclasses shared across robot configs.

Each class exposes sensible *defaults* that come from ``configs.constants`` so
individual robot config files no longer need to duplicate IP addresses or port
numbers.  Override any field as usual when you instantiate the dataclass.
"""

from dataclasses import dataclass

from beavr.configs_teleop import constants as CONST  # single source of truth for defaults

from beavr.components.detector.oculus import OculusVRHandDetector
from beavr.components.detector.keypoint_transform import TransformHandPositionCoords
from beavr.components.visualizers.visualizer_2d import Hand2DVisualizer

@dataclass
class OculusVRHandDetectorCfg:
    """Configuration for :class:`beavr.components.detector.oculus.OculusVRHandDetector`."""

    host: str = CONST.HOST_ADDRESS
    oculus_port: int | str = CONST.OCULUS_RECEIVER_PORT
    unified_pub_port: int | str = CONST.KEYPOINT_STREAM_PORT  # alias: unified data
    button_port: int | str = CONST.RESOLUTION_BUTTON_PORT
    teleop_reset_port: int | str = CONST.TELEOP_RESET_PORT

    def build(self):
        return OculusVRHandDetector(
            host=self.host,
            oculus_port=self.oculus_port,
            unified_pub_port=self.unified_pub_port,
            button_port=self.button_port,
            teleop_reset_port=self.teleop_reset_port,
        )

@dataclass
class TransformHandPositionCoordsCfg:
    """Right-hand keypoint transform (VR frame → robot frame)."""

    host: str = CONST.HOST_ADDRESS
    keypoint_port: int | str = CONST.KEYPOINT_STREAM_PORT
    transformation_port: int | str = CONST.KEYPOINT_TRANSFORM_PORT
    moving_average_limit: int = 1

    def build(self):
        return TransformHandPositionCoords(
            host=self.host,
            keypoint_port=self.keypoint_port,
            transformation_port=self.transformation_port,
            moving_average_limit=self.moving_average_limit,
        )

@dataclass
class Hand2DVisualizerCfg:
    """2-D hand visualizer (Matplotlib / OpenCV)."""

    host: str = CONST.HOST_ADDRESS
    transformed_keypoint_port: int | str = CONST.KEYPOINT_TRANSFORM_PORT
    oculus_feedback_port: int | str = CONST.OCULUS_GRAPH_PORT
    display_plot: bool = False

    def build(self):
        return Hand2DVisualizer(
            host=self.host,
            transformed_keypoint_port=self.transformed_keypoint_port,
            oculus_feedback_port=self.oculus_feedback_port,
            display_plot=self.display_plot,
        )