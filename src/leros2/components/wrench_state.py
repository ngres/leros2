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

from typing import Any
from leros2.components.common import StateComponentConfig
from leros2.components.common.base import BaseComponentConfig
from dataclasses import dataclass
from geometry_msgs.msg import WrenchStamped


@dataclass
@BaseComponentConfig.register_subclass('wrench_state')
class WrenchStateComponentConfig(StateComponentConfig):
    name: str


class WrenchStateComponent(StateComponent[WrenchStateComponentConfig, WrenchStamped]):
    def __init__(self, config: WrenchStateComponentConfig):
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

    def to_value(self, msg: WrenchStamped) -> dict[str, Any]:
        return {
            f"{self._config.name}.force.x": msg.wrench.force.x,
            f"{self._config.name}.force.y": msg.wrench.force.y,
            f"{self._config.name}.force.z": msg.wrench.force.z,
            f"{self._config.name}.torque.x": msg.wrench.torque.x,
            f"{self._config.name}.torque.y": msg.wrench.torque.y,
            f"{self._config.name}.torque.z": msg.wrench.torque.z,
        }
