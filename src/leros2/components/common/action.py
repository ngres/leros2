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

from dataclasses import dataclass
from abc import abstractmethod
from typing import Any, Generic, TypeVar
from rclpy.node import Node

from .base import BaseComponent, BaseComponentConfig


@dataclass
class ActionComponentConfig(BaseComponentConfig):
    pass


ActionConfigT = TypeVar("ActionConfigT", bound=ActionComponentConfig)

MsgT = TypeVar("MsgT")


class ActionComponent(BaseComponent[ActionConfigT], Generic[ActionConfigT, MsgT]):
    """Adapter for converting action features to a ROS 2 message."""

    _msg_type: type[MsgT]

    def __init__(self, config: ActionConfigT, msg_type: type[MsgT]):
        super().__init__(config)

        self._msg_type = msg_type

    @abstractmethod
    def to_message(self, action: dict[str, Any]) -> MsgT:
        """Convert action features to a ROS 2 message.

        Args:
            action: The action features to convert.

        Returns:
            The ROS 2 message.
        """
        raise NotImplementedError

    def connect(self, node: Node) -> None:
        super().connect(node)

        self._publisher = node.create_publisher(
            self._msg_type,
            self.topic,
            10,
        )

    def disconnect(self) -> None:
        super().disconnect()

        self._publisher.destroy()

    def send_action(self, action: dict[str, Any]) -> None:
        """Send an action to the ROS 2 message.

        Args:
            action: The action features to send.
        """
        msg = self.to_message(action)
        self._publisher.publish(msg)
