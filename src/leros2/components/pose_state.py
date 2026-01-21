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
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass


@dataclass
@BaseComponentConfig.register_subclass('pose_state')
class PoseStateComponentConfig(StateComponentConfig):
    name: str


class PoseStateComponent(StateComponent[PoseStateComponentConfig, PoseStamped]):
    def __init__(self, config: PoseStateComponentConfig):
        super().__init__(config, PoseStamped)

    @property
    def features(self) -> dict[str, type]:
        return {
            f"{self._config.name}_x.pos": float,
            f"{self._config.name}_y.pos": float,
            f"{self._config.name}_z.pos": float,
            f"{self._config.name}_x.quat": float,
            f"{self._config.name}_y.quat": float,
            f"{self._config.name}_z.quat": float,
            f"{self._config.name}_w.quat": float,
            f"{self._config.name}_x.rot": float,
            f"{self._config.name}_y.rot": float,
            f"{self._config.name}_z.rot": float,
        }

    def to_value(self, msg: PoseStamped) -> dict[str, Any]:
        wx, wy, wz = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_rotvec()
        return {
            f"{self._config.name}_x.pos": msg.pose.position.x,
            f"{self._config.name}_y.pos": msg.pose.position.y,
            f"{self._config.name}_z.pos": msg.pose.position.z,
            f"{self._config.name}_x.quat": msg.pose.orientation.x,
            f"{self._config.name}_y.quat": msg.pose.orientation.y,
            f"{self._config.name}_z.quat": msg.pose.orientation.z,
            f"{self._config.name}_w.quat": msg.pose.orientation.w,
            f"{self._config.name}_x.rot": wx,
            f"{self._config.name}_y.rot": wy,
            f"{self._config.name}_z.rot": wz,
        }
