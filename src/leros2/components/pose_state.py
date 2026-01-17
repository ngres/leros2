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
import numpy as np
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

    def _quat_to_euler(self, quat: np.ndarray) -> np.ndarray:
        x, y, z, w = quat
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return [roll, pitch, yaw]

    @property
    def features(self) -> dict[str, type]:
        return {
            f"{self._config.name}_x.pos": float,
            f"{self._config.name}_x.pos": float,
            f"{self._config.name}_z.pos": float,
            f"{self._config.name}_roll.pos": float,
            f"{self._config.name}_pitch.pos": float,
            f"{self._config.name}_yaw.pos": float,
        }

    def to_value(self, msg: PoseStamped) -> dict[str, Any]:
        roll, pitch, yaw = self._quat_to_euler([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        return {
            f"{self._config.name}_x.pos": msg.pose.position.x,
            f"{self._config.name}_x.pos": msg.pose.position.y,
            f"{self._config.name}_z.pos": msg.pose.position.z,
            f"{self._config.name}_roll.pos": roll,
            f"{self._config.name}_pitch.pos": pitch,
            f"{self._config.name}_yaw.pos": yaw,
        }
