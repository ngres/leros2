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

from abc import abstractmethod, ABC
from typing import Generic, TypeVar
from rclpy.node import Node
from dataclasses import dataclass
from draccus import ChoiceRegistry


@dataclass
class BaseComponentConfig(ChoiceRegistry):
    topic: str


ConfigT = TypeVar("ConfigT", bound=BaseComponentConfig)


class BaseComponent(ABC, Generic[ConfigT]):
    """Adapter for returning the feature representation of a message."""

    _config: ConfigT

    def __init__(self, config: ConfigT):
        self._config = config
        self._node: Node | None = None

    @property
    @abstractmethod
    def features(self) -> dict[str, type | tuple[type, ...]]:
        """Return the feature representation of a message.

        Returns:
            The feature representation of a message.
        """
        raise NotImplementedError

    def connect(self, node: Node) -> None:
        """Connect to the component.

        Args:
            node: The node to connect to.
        """
        self._node = node

    def disconnect(self) -> None:
        """Disconnect from the component."""
        self._node = None

    @property
    def topic(self) -> str:
        return self._config.topic
