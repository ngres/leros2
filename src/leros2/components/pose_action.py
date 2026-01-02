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

from geometry_msgs.msg import Pose
from typing import Any
import numpy as np
from lerobot.utils.rotation import Rotation
from leros2.components.common import ActionComponentConfig, ActionComponent


@dataclass
class PoseActionComponentConfig(ActionComponentConfig):
    name: str


class PoseActionComponent(ActionComponent[PoseActionComponentConfig, Pose]):
    def __init__(self, config: PoseActionComponentConfig):
        super().__init__(config, Pose)

    @property
    def features(self) -> dict[str, type]:
        return {
            f"{self._config.name}.pos": np.ndarray,  # shape (3,)
            f"{self._config.name}.rot": Rotation,  # quaternion
        }

    def to_message(self, action: dict[str, Any]) -> Pose:
        msg = Pose()

        pos: np.ndarray = action[f"{self._config.name}.pos"]
        msg.position.x = pos[0]
        msg.position.y = pos[1]
        msg.position.z = pos[2]

        rot: Rotation = action[f"{self._config.name}.rot"]
        msg.orientation.x = rot.x
        msg.orientation.y = rot.y
        msg.orientation.z = rot.z
        msg.orientation.w = rot.w

        return msg
