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

from typing import Any
from leros2.components.common import ActionComponent, ActionComponentConfig
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from leros2.components.joint_state import JointConfig
from dataclasses import dataclass
from leros2.components.common.base import BaseComponentConfig


@dataclass(kw_only=True)
@BaseComponentConfig.register_subclass('joint_action')
class JointActionComponentConfig(ActionComponentConfig):
    joints: list[JointConfig]


class JointActionComponent(
    ActionComponent[JointActionComponentConfig, JointTrajectory]
):
    """Adapter for converting a ROS 2 joint state message to a feature value dictionary."""

    def __init__(self, config: JointActionComponentConfig):
        super().__init__(config, JointTrajectory)

    @property
    def features(self) -> dict[str, type]:
        features: dict[str, type] = {}

        for joint in self._config.joints:
            features[f"{joint.name}.pos"] = float

        return features

    def to_message(self, action: dict[str, Any]) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = []
        msg.points = [JointTrajectoryPoint()]

        for joint in self._config.joints:
            joint_value = action[f"{joint.name}.pos"]
            if joint_value is None:
                raise ValueError(f"Joint '{joint.name}' not found in action.")
            msg.joint_names.append(joint.name)
            msg.points[0].positions.append(self._unnormalize_joint(joint_value, joint))

        return msg

    def _unnormalize_joint(
        self, joint_value: float, joint_config: JointConfig
    ) -> float:
        """Unnormalize a joint value to radians."""

        return ((joint_value + 1.0) / 2.0) * (
            joint_config.range_max - joint_config.range_min
        ) + joint_config.range_min
