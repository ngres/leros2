# Copyright 2026 Nicolas Gres
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

from typing import Any
from dataclasses import dataclass
from geometry_msgs.msg import WrenchStamped

from leros2.components.common import ActionTopicComponent, ActionComponentConfig
from leros2.components.common.base import BaseComponentConfig


@dataclass
@BaseComponentConfig.register_subclass('wrench_action')
class WrenchActionComponentConfig(ActionComponentConfig):
    name: str
    frame_id: str


class WrenchActionComponent(
    ActionTopicComponent[WrenchActionComponentConfig, WrenchStamped]
):
    def __init__(self, config: WrenchActionComponentConfig):
        super().__init__(config, WrenchStamped)

    @property
    def features(self) -> dict[str, type | tuple[type, ...]]:
        return {
            f"{self._config.name}.force.x": float,
            f"{self._config.name}.force.y": float,
            f"{self._config.name}.force.z": float,
            f"{self._config.name}.torque.x": float,
            f"{self._config.name}.torque.y": float,
            f"{self._config.name}.torque.z": float,
        }

    def to_message(self, action: dict[str, Any]) -> WrenchStamped:
        msg = WrenchStamped()
        if self._node:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id
        msg.wrench.force.x = action[f"{self._config.name}.force.x"]
        msg.wrench.force.y = action[f"{self._config.name}.force.y"]
        msg.wrench.force.z = action[f"{self._config.name}.force.z"]
        msg.wrench.torque.x = action[f"{self._config.name}.torque.x"]
        msg.wrench.torque.y = action[f"{self._config.name}.torque.y"]
        msg.wrench.torque.z = action[f"{self._config.name}.torque.z"]
        return msg
