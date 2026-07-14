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

from leros2.components.common import StateComponent
from geometry_msgs.msg import PoseStamped
from typing import Any
from leros2.components.common import StateComponentConfig
from leros2.components.common.base import BaseComponentConfig
from dataclasses import dataclass


@dataclass
@BaseComponentConfig.register_subclass('pose_state')
class PoseStateComponentConfig(StateComponentConfig):
    name: str


class PoseStateComponent(StateComponent[PoseStateComponentConfig, PoseStamped]):
    def __init__(self, config: PoseStateComponentConfig):
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

    def to_value(self, msg: PoseStamped) -> dict[str, Any]:
        return {
            f"{self._config.name}.pos.x": msg.pose.position.x,
            f"{self._config.name}.pos.y": msg.pose.position.y,
            f"{self._config.name}.pos.z": msg.pose.position.z,
            f"{self._config.name}.quat.x": msg.pose.orientation.x,
            f"{self._config.name}.quat.y": msg.pose.orientation.y,
            f"{self._config.name}.quat.z": msg.pose.orientation.z,
            f"{self._config.name}.quat.w": msg.pose.orientation.w,
        }
