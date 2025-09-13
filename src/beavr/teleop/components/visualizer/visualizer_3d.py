import logging

import numpy as np

from beavr.teleop.common.messaging.vr.subscribers import ZMQSubscriber
from beavr.teleop.components import Component
from beavr.teleop.configs.constants import robots

from .plotters.plotter_3d import PlotHand3D, PlotHandDirection

logger = logging.getLogger(__name__)


class Hand3DVisualizer(Component):
    def __init__(self, host, port):
        self.notify_component_start("hand 3D plotter")
        self.subscriber = ZMQSubscriber(host=host, port=port, topic="transformed_hand_coords")

        # Initializing the plotting object
        self.plotter3D = PlotHand3D()

    def _get_keypoints(self):
        raw_keypoints = self.subscriber.recv_keypoints()
        return np.array(raw_keypoints).reshape(robots.OCULUS_NUM_KEYPOINTS, 3)

    def stream(self):
        while True:
            try:
                keypoints = self._get_keypoints()
                self.plotter3D.draw(keypoints[:, 0], keypoints[:, 1], keypoints[:, 2])
            except Exception as e:
                logger.error(f"Error in hand 3D visualizer: {e}")
                break

        self.subscriber.stop()
        print("Stopping the hand 3D visualizer process.")


class OculusRightHandDirVisualizer(Component):
    def __init__(self, host, port, scaling_factor=0.2):
        # Other parameters
        self.scaling_factor = scaling_factor

        # Initializing the Keypoint variable and subscriber
        self.subscriber = ZMQSubscriber(host=host, port=port, topic="transformed_hand_frame")

        # Initializing the plotting object
        self.notify_component_start("hand direction plotter")
        self.dir_plotter = PlotHandDirection()

    def _get_directions(self):
        raw_directions = self.subscriber.recv_keypoints()
        return np.array(raw_directions).reshape(4, 3)

    def stream(self):
        while True:
            try:
                directions = self._get_directions()
                self.dir_plotter.draw(directions[:, 0], directions[:, 1], directions[:, 2])
            except Exception as e:
                logger.error(f"Error in hand direction visualizer: {e}")
                break

        self.subscriber.stop()
        print("Stopping the hand direction visualizer process.")
