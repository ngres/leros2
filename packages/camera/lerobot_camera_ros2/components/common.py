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

from abc import ABC, abstractmethod
from rclpy.node import Node
from numpy.typing import NDArray
from lerobot.cameras import ColorMode
from lerobot_camera_ros2.config_ros2_camera import ROS2CameraConfig
from rclpy.qos import QoSProfile
import numpy as np
from rclpy.subscription import Subscription
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from typing import TypeVar


T = TypeVar('T')

class BaseComponent(ABC):
    """
    Base interface for LeROS2 robots.
    """

    node: Node | None = None

    def __init__(self, config: ROS2CameraConfig, msg_type: T):
        self.config = config

        self._msg_type = msg_type
        self._data: NDArray[np.uint8] | None = None
        self._subscriber: Subscription | None = None

    def connect(self, node: Node) -> None:
        """Connect to the robot."""
        self.node = node

        self._subscriber = node.create_subscription(
            msg_type=self._msg_type,
            topic=self.config.topic,
            callback=self._callback,
            qos_profile=QoSProfile(depth=1)
        )

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        if self._subscriber is not None:
            self._subscriber.destroy()
            self._subscriber = None

    def _callback(self, msg: T) -> None:
        """Callback to be called when a new message is received.
        
        Should update 
        """
        self._data = self.msg_to_data(msg)

    @abstractmethod
    def msg_to_data(self, msg: T) -> NDArray[np.uint8]:
        """Convert a message to a numpy array."""
        pass

    def read(self, color_mode: ColorMode | None = None) -> NDArray[np.uint8]:
        if self._subscriber is None:
            raise RuntimeError("Camera is not connected")

        if self._data is None:
            raise RuntimeError("No image available")

        return self._data

    def async_read(self, timeout_ms: float = ...) -> NDArray[np.uint8]:
        if self._subscriber is None:
            raise RuntimeError("Camera is not connected")

        if self.node is None:
            raise RuntimeError("Node is not connected")

        # wait for a new image to be available
        context = self.node.context
        wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
        wait_set.clear_entities()

        wait_set.add_subscription(self._subscriber.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = -1

        if timeout_ms != ...:
            timeout_nsec = timeout_ms * 1000000
        
        wait_set.wait(int(timeout_nsec))

        subs_ready = wait_set.get_ready_entities('subscription')
        guards_ready = wait_set.get_ready_entities('guard_condition')

        if guards_ready:
            if sigint_gc.handle.pointer in guards_ready:
                raise KeyboardInterrupt

        if subs_ready:
            if self._subscriber.handle.pointer in subs_ready:
                msg_info = self._subscriber.handle.take_message(self._subscriber.msg_type, self._subscriber.raw)
                if msg_info is not None:
                    return self.msg_to_data(msg_info[0])

        raise RuntimeError("Timeout")

