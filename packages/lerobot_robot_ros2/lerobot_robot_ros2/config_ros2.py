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

from lerobot.robots.config import RobotConfig

from leros2.components.common import StateComponentConfig, ActionComponentConfig
from leros2.robot_config import ROS2RobotConfig


@RobotConfig.register_subclass("ros2")
@dataclass
class ROS2Config(ROS2RobotConfig):
    """Configuration for a generic, component-driven ROS 2 robot.

    The robot is assembled from two lists of component configs:

    - ``state`` components map ROS 2 topics to observation features.
    - ``action`` components map action features to ROS 2 topics/action goals.

    Each entry is a ``draccus`` choice type discriminated by a ``type`` key
    (e.g. ``joint_state``, ``pose_action``); see ``leros2.components``.
    """

    # Observation-producing components (subscriptions).
    state: list[StateComponentConfig] = field(default_factory=list)

    # Action-consuming components (publishers / action clients).
    action: list[ActionComponentConfig] = field(default_factory=list)
