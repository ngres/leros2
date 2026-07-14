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

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

from leros2.components.common import StateComponentConfig
from leros2.teleoperator_config import ROS2TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ros2")
@dataclass
class ROS2TeleopConfig(ROS2TeleoperatorConfig):
    """Configuration for a generic, component-driven ROS 2 teleoperator.

    A teleoperator only reads, so it is assembled from a single list of
    ``action`` component configs mapping ROS 2 topics to action features.

    Each entry is a ``draccus`` choice type discriminated by a ``type`` key
    (e.g. ``pose_state``, ``joint_state``); see ``leros2.components``.
    """

    # Teleop actions captured by state-producing components (subscriptions).
    action: list[StateComponentConfig] = field(default_factory=list)
