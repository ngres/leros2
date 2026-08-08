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
from leros2.components.common.rotation import RotationRepresentation
from dataclasses import dataclass


@dataclass
@BaseComponentConfig.register_subclass('pose_state')
class PoseStateComponentConfig(StateComponentConfig):
    # name to identify the pose component
    name: str

    # representation of the orientation features (see ``RotationRepresentation``)
    rotation: RotationRepresentation = RotationRepresentation.QUATERNION


class PoseStateComponent(StateComponent[PoseStateComponentConfig, PoseStamped]):
    """Adapter for converting a ROS 2 ``PoseStamped`` to a feature value dictionary.

    The position is exposed as ``<name>.pos.{x,y,z}``; the orientation keys
    depend on the configured :class:`RotationRepresentation`.
    """

    def __init__(self, config: PoseStateComponentConfig):
        super().__init__(config, PoseStamped)

        self._rotation = config.rotation.encoding

    @property
    def features(self) -> dict[str, type | tuple[type, ...]]:
        name = self._config.name
        return {
            f"{name}.pos.x": float,
            f"{name}.pos.y": float,
            f"{name}.pos.z": float,
            **self._rotation.features(name),
        }

    def to_value(self, msg: PoseStamped) -> dict[str, Any]:
        name = self._config.name
        orientation = msg.pose.orientation
        return {
            f"{name}.pos.x": msg.pose.position.x,
            f"{name}.pos.y": msg.pose.position.y,
            f"{name}.pos.z": msg.pose.position.z,
            **self._rotation.encode(
                name, (orientation.x, orientation.y, orientation.z, orientation.w)
            ),
        }
