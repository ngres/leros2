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

import logging
from typing import Any

from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator

from .components.common import StateComponent
from .common import ROS2Common
from .teleoperator_config import ROS2TeleoperatorConfig


logger = logging.getLogger(__name__)


class ROS2Teleoperator(
    ROS2Common[ROS2TeleoperatorConfig, StateComponent], Teleoperator
):
    config_class = ROS2TeleoperatorConfig
    name = "ros2_teleoperator"

    def __init__(
        self, config: ROS2TeleoperatorConfig, components: list[StateComponent]
    ):
        Teleoperator.__init__(self, config)
        ROS2Common.__init__(self, config, components)

    def connect(self, calibrate: bool = True) -> None:
        ROS2Common.connect(self)

    def disconnect(self) -> None:
        ROS2Common.disconnect(self)

    @property
    def action_features(self) -> dict[str, type]:
        """Get the action features provided by this teleoperator.
        Returns:
            dict[str, type]: The action features provided by this teleoperator.
        """
        action_features = {}
        for comp in self._components:
            action_features.update(comp.features)
        return action_features

    def get_action(self) -> dict[str, Any]:
        """Get the current teleoperator action.

        Returns:
            dict[str, Any]: The current teleoperator action.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} not connected")

        action: dict[str, Any] = {}

        for comp in self._components:
            action.update(comp.get_state())

        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}
