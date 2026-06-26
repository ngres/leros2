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

from std_msgs.msg import Float64MultiArray

from leros2.components.common import StateComponent, StateComponentConfig
from leros2.components.common.base import BaseComponentConfig
from leros2.components.joint_state import JointConfig


@dataclass
@BaseComponentConfig.register_subclass('float_array_state')
class FloatArrayStateComponentConfig(StateComponentConfig):
    # Per-element configuration, ordered to match the received array.
    # Each element supports normalization like a joint (see ``JointConfig``),
    # which is useful e.g. for gripper joint states published as a Float array.
    joints: list[JointConfig]


class FloatArrayStateComponent(
    StateComponent[FloatArrayStateComponentConfig, Float64MultiArray]
):
    """Adapter for converting a ROS 2 ``Float64MultiArray`` to a feature value dictionary.

    Each configured element is normalized via its :class:`JointConfig` from the
    array ``data`` in order.
    """

    def __init__(self, config: FloatArrayStateComponentConfig):
        super().__init__(config, Float64MultiArray)

    @property
    def features(self) -> dict[str, type | tuple[type, ...]]:
        features: dict[str, type | tuple[type, ...]] = {}

        for joint in self._config.joints:
            features[f"{joint.name}.pos"] = float

        return features

    def to_value(self, msg: Float64MultiArray) -> dict[str, Any]:
        value: dict[str, Any] = {}

        for index, joint in enumerate(self._config.joints):
            value[f"{joint.name}.pos"] = joint.normalize(msg.data[index])

        return value
