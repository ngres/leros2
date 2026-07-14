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

from geometry_msgs.msg import PoseStamped
from typing import Any
from leros2.components.common import ActionComponentConfig, ActionTopicComponent
from leros2.components.common.base import BaseComponentConfig


@dataclass
@BaseComponentConfig.register_subclass('pose_action')
class PoseActionComponentConfig(ActionComponentConfig):
    # name to identify the pose component
    name: str

    # ros2 frame id
    frame_id: str


class PoseActionComponent(ActionTopicComponent[PoseActionComponentConfig, PoseStamped]):
    def __init__(self, config: PoseActionComponentConfig):
        super().__init__(config, PoseStamped)

    @property
    def features(self) -> dict[str, type | tuple[type, ...]]:
        return {
            f"{self._config.name}.pos.x": float,
            f"{self._config.name}.pos.y": float,
            f"{self._config.name}.pos.z": float,
            f"{self._config.name}.quat.x": float,
            f"{self._config.name}.quat.y": float,
            f"{self._config.name}.quat.z": float,
            f"{self._config.name}.quat.w": float,
        }

    def to_message(self, action: dict[str, Any]) -> PoseStamped:
        msg = PoseStamped()

        if self._node:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id

        msg.pose.position.x = action[f"{self._config.name}.pos.x"]
        msg.pose.position.y = action[f"{self._config.name}.pos.y"]
        msg.pose.position.z = action[f"{self._config.name}.pos.z"]

        msg.pose.orientation.x = action[f"{self._config.name}.quat.x"]
        msg.pose.orientation.y = action[f"{self._config.name}.quat.y"]
        msg.pose.orientation.z = action[f"{self._config.name}.quat.z"]
        msg.pose.orientation.w = action[f"{self._config.name}.quat.w"]

        return msg
