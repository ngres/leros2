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
from numpy.typing import NDArray
from rclpy.node import Node

from .base import BaseComponent, BaseComponentConfig


@dataclass
class StateComponentConfig(BaseComponentConfig):
    pass


StateConfigT = TypeVar("StateConfigT", bound=StateComponentConfig)

MsgT = TypeVar("MsgT")


class StateComponent(BaseComponent[StateConfigT], Generic[StateConfigT, MsgT]):
    """Adapter for converting a ROS 2 message to state features."""

    _msg_type: type[MsgT]

    def __init__(self, config: StateConfigT, msg_type: type[MsgT]):
        super().__init__(config)

        self._msg_type = msg_type

    @abstractmethod
    def to_value(self, msg: MsgT) -> dict[str, Any] | NDArray[Any]:
        """Convert a ROS 2 message to state features.

        Args:
            msg: The ROS 2 message to convert.

        Returns:
            The state features.
        """
        raise NotImplementedError

    def connect(self, node: Node) -> None:
        super().connect(node)

        self._subscription = node.create_subscription(
            self._msg_type,
            self.topic,
            self._callback,
            10,
        )

    def disconnect(self) -> None:
        super().disconnect()

        self._subscription.destroy()

    def get_state(self) -> dict[str, Any] | NDArray[Any]:
        """Get the state features.

        Returns:
            The state features.
        """
        return self._value

    def _callback(self, msg: MsgT) -> None:
        """Callback function for the subscription.

        Args:
            msg: The ROS 2 message received.
        """
        self._value = self.to_value(msg)
