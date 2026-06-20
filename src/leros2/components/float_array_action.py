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

from leros2.components.common import ActionTopicComponent, ActionComponentConfig
from leros2.components.common.base import BaseComponentConfig
from leros2.components.joint_state import JointConfig


@dataclass(kw_only=True)
@BaseComponentConfig.register_subclass('float_array_action')
class FloatArrayActionComponentConfig(ActionComponentConfig):
    # Per-element configuration, ordered to match the published array.
    # Each element supports normalization like a joint (see ``JointConfig``),
    # which is useful e.g. for gripper joint commands published as a Float array.
    joints: list[JointConfig]


class FloatArrayActionComponent(
    ActionTopicComponent[FloatArrayActionComponentConfig, Float64MultiArray]
):
    """Adapter for publishing action features as a ROS 2 ``Float64MultiArray``.

    Each configured element is unnormalized via its :class:`JointConfig` and the
    resulting values are published in order as the array ``data``.
    """

    def __init__(self, config: FloatArrayActionComponentConfig):
        super().__init__(config, Float64MultiArray)

    @property
    def features(self) -> dict[str, type | tuple[type, ...]]:
        features: dict[str, type | tuple[type, ...]] = {}

        for joint in self._config.joints:
            features[f"{joint.name}.pos"] = float

        return features

    def to_message(self, action: dict[str, Any]) -> Float64MultiArray:
        msg = Float64MultiArray()

        data: list[float] = []
        for joint in self._config.joints:
            joint_value = action[f"{joint.name}.pos"]
            if joint_value is None:
                raise ValueError(f"Joint '{joint.name}' not found in action.")
            data.append(joint.unnormalize(joint_value))

        msg.data = data

        return msg
