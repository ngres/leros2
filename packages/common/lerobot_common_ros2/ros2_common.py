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

from abc import abstractmethod
import logging
import threading
from typing import Any, Generic, TypeVar

from .components.common import BaseComponent

import rclpy
from rclpy.executors import MultiThreadedExecutor

from lerobot.utils.errors import DeviceAlreadyConnectedError

from .config_ros2_common import ROS2CommonConfig


logger = logging.getLogger(__name__)


T = TypeVar("T", bound=ROS2CommonConfig[Any])

B = TypeVar("B", bound=BaseComponent)


class ROS2Common(Generic[T, B]):

    def __init__(self, config: T):
        self.config: T = config
        self._is_connected = False
        self._node = None
        self._executor = None
        self._spin_thread = None

        self._components: dict[str, B] = {}

        self._init_components()

    @abstractmethod
    def _init_components(self):
        pass

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self.config.node_name)

        for comp in self._components.values():
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

        for comp in self._components.values():
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
    def is_connected(self) -> bool:
        return self._is_connected
