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

from leros2.components.common import StateComponent
from geometry_msgs.msg import Pose
from typing import Any
import numpy as np
from lerobot.utils.rotation import Rotation
from leros2.components.common import StateComponentConfig
from dataclasses import dataclass


@dataclass
class PoseStateComponentConfig(StateComponentConfig):
    name: str


class PoseStateComponent(StateComponent[PoseStateComponentConfig, Pose]):
    def __init__(self, config: PoseStateComponentConfig):
        super().__init__(config, Pose)

    @property
    def features(self) -> dict[str, type]:
        return {
            f"{self._config.name}.pos": np.ndarray,  # shape (3,)
            f"{self._config.name}.rot": Rotation,  # quaternion
        }

    def to_value(self, msg: Pose) -> dict[str, Any]:
        return {
            f"{self._config.name}.pos": np.array(
                [
                    msg.position.x,
                    msg.position.y,
                    msg.position.z,
                ]
            ),
            f"{self._config.name}.rot": Rotation.from_quat(
                np.array(
                    [
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    ]
                )
            ),
        }
