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
from leros2.components.common import StateComponent, StateComponentConfig
from trajectory_msgs.msg import JointTrajectory
from dataclasses import dataclass
import math


@dataclass
class JointConfig:
    """Configuration for a joint."""

    # Name of the joint
    name: str

    # Minimum range of the joint
    range_min: float = -math.pi

    # Maximum range of the joint
    range_max: float = math.pi


@dataclass
class JointStateComponentConfig(StateComponentConfig):
    joints: list[JointConfig]


class JointStateComponent(StateComponent[JointStateComponentConfig, JointTrajectory]):
    """Adapter for converting a ROS 2 joint state message to a feature value dictionary."""

    def __init__(self, config: JointStateComponentConfig):
        super().__init__(config, JointTrajectory)

        self._joints: dict[str, JointConfig] = {}

        for joint in config.joints:
            self._joints[joint.name] = joint

    @property
    def features(self) -> dict[str, type]:
        features: dict[str, type] = {}

        for name in self._joints:
            features[f"{name}.pos"] = float

        return features

    def to_value(self, msg: JointTrajectory) -> dict[str, Any]:
        value: dict[str, Any] = {}

        for index, name in enumerate(msg.joint_names):
            joint_config = self._joints.get(name)
            if joint_config is None:
                continue

            value[f"{name}.pos"] = self._normalize_joint(
                msg.points[0].positions[index],
                joint_config,
            )

        return value

    def _normalize_joint(self, value: float, config: JointConfig) -> float:
        """Normalize a joint value from radians."""

        return (value - config.range_min) / (
            config.range_max - config.range_min
        ) * 2.0 - 1.0
