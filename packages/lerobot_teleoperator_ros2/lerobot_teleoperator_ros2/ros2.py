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
from leros2.components.factory import make_components
from leros2.teleoperator import ROS2Teleoperator

from .config_ros2 import ROS2TeleopConfig


class ROS2Teleop(ROS2Teleoperator):
    """Generic ROS 2 teleoperator assembled from configured state components."""

    name = "ros2"

    def __init__(self, config: ROS2TeleopConfig):
        components = make_components(config.action)

        for component in components:
            if not isinstance(component, StateComponent):
                raise ValueError(
                    f"Teleoperator components must be state components, got "
                    f"{type(component).__name__}."
                )

        super().__init__(config, components)
