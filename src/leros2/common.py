# Copyright 2025 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import threading
from typing import Any, Generic, TypeVar

from .components.common import BaseComponent, StateComponent, ActionComponent

import rclpy
from rclpy.executors import MultiThreadedExecutor

from lerobot.utils.errors import DeviceAlreadyConnectedError

from .common_config import ROS2CommonConfig


logger = logging.getLogger(__name__)


ComponentT = TypeVar("ComponentT", bound=BaseComponent[Any])

ConfigT = TypeVar("ConfigT", bound=ROS2CommonConfig)


class ROS2Common(Generic[ConfigT, ComponentT]):
    """ROS2 common class for teleoperator and robot."""

    def __init__(self, config: ConfigT, components: list[ComponentT]):
        self.config = config
        self._is_connected = False
        self._node = None
        self._executor = None
        self._spin_thread = None

        self._components: list[ComponentT] = components

    def get_state_from_topics(self, topics: dict[str, Any]) -> dict[str, Any]:
        """Get state from ROS topics.

        This method can be called without needing to connect to the teleoperator. It is useful for
        getting the state from a bag file.

        Args:
            topics (dict[str, Any]): Dictionary of ROS topics.

        Returns:
            dict[str, Any]: Dictionary of states.
        """
        state: dict[str, Any] = {}
        for comp in self._components:
            if not isinstance(comp, StateComponent):
                continue

            msg = topics.get(comp.topic)
            if not msg:
                logger.warning(f"Topic {comp.topic} not found in topics.")
                continue

            state.update(comp.to_value(msg))  # type: ignore

        return state

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self.config.node_name)

        for comp in self._components:
            comp.connect(self._node)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._is_connected = True
        logger.info(f"{self} connected.")

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        for comp in self._components:
            comp.disconnect()

        if self._executor:
            self._executor.shutdown()
        if self._spin_thread:
            self._spin_thread.join()
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        self._is_connected = False
        logger.info(f"{self} disconnected.")

    @property
    def subscription_topics(self) -> list[str]:
        topics = []
        for comp in self._components:
            if isinstance(comp, StateComponent):
                topics.append(comp.topic)
        return topics

    @property
    def publisher_topics(self) -> list[str]:
        topics = []
        for comp in self._components:
            if isinstance(comp, ActionComponent):
                topics.append(comp.topic)
        return topics

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def calibrate(self):
        pass

    def configure(self):
        pass

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def is_configured(self) -> bool:
        return True
