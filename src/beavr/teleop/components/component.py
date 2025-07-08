from abc import ABC, abstractmethod

import logging

logger = logging.getLogger(__name__)


class Component(ABC):
    @abstractmethod
    def stream(self):
        raise NotImplementedError()

    def notify_component_start(self, component_name):
        logger.info("***************************************************************")
        logger.info("     Starting {} component".format(component_name))
        logger.info("***************************************************************")