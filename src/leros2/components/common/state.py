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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .base import BaseComponent, BaseComponentConfig


@dataclass
class StateComponentConfig(BaseComponentConfig):
    pass


StateConfigT = TypeVar("StateConfigT", bound=StateComponentConfig)

MsgT = TypeVar("MsgT")


class StateComponent(BaseComponent[StateConfigT], Generic[StateConfigT, MsgT]):
    """Adapter for converting a ROS 2 message to state features."""

    def __init__(self, config: StateConfigT, msg_type: type[MsgT]):
        super().__init__(config)

        self._msg_type: type[MsgT] = msg_type
        self._value: dict[str, Any] | NDArray[Any] | None = None

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
        """Connect the component, subscribing to the state topic.

        The subscription uses a depth-1 KEEP_LAST queue as the latest-message
        buffer, which ``get_state`` polls directly via ``_take``. The owning
        node must NOT be added to an executor: a spinning executor would consume
        the message first, leaving ``_take`` to always return ``None``. If
        callbacks are needed elsewhere, this component requires a dedicated,
        un-spun ``Node``.

        Args:
            node: The node to connect to.
        """
        super().connect(node)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # must match the publisher
        )
        self._subscription = node.create_subscription(
            self._msg_type,
            self.topic,
            lambda _msg: None,  # never dispatched; messages are polled via _take
            qos,
        )

    def disconnect(self) -> None:
        self._node.destroy_subscription(self._subscription)

        super().disconnect()

    def get_state(self) -> dict[str, Any] | NDArray[Any]:
        """Get the state features.

        Returns:
            The state features.
        """
        msg = self._take()
        if msg is not None:
            self._value = self.to_value(msg)
        if self._value is None:
            raise ValueError(f"No message received on topic {self.topic}")
        return self._value

    def _take(self) -> MsgT | None:
        """Take the latest message from the subscription's native queue.

        A single non-blocking take: the depth-1 KEEP_LAST cache already discards
        older samples, so one take yields the newest message (or ``None`` when
        nothing new has arrived).

        Returns:
            The latest message, or ``None`` if the queue is empty.
        """
        sub = self._subscription
        with sub.handle:
            msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
        if msg_info is None:
            return None
        return msg_info[0]  # (msg, message_info) tuple
