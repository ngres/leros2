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
from leros2.components.common.base_image import ImageBaseComponent

import logging
import copy
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.robots.robot import Robot

from .components.common import BaseComponent, StateComponent, ActionComponent
from .common import ROS2Common
from .robot_config import ROS2RobotConfig

logger = logging.getLogger(__name__)


class ROS2Robot(ROS2Common[ROS2RobotConfig, StateComponent | ActionComponent], Robot):
    config_class = ROS2RobotConfig
    name = "ros2_robot"

    def __init__(
        self,
        config: ROS2RobotConfig,
        components: list[BaseComponent],
        init_cameras: bool = True,
    ):
        Robot.__init__(self, config)
        ROS2Common.__init__(self, config, components)

        if init_cameras:
            self.cameras = make_cameras_from_configs(config.cameras)
        else:
            self.cameras = {}

    def connect(self, calibrate: bool = True) -> None:
        ROS2Common.connect(self)

        if self.cameras:
            for cam in self.cameras.values():
                cam.connect(self._node)

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        for cam in self.cameras.values():
            cam.disconnect()

        ROS2Common.disconnect(self)

    @property
    def action_features(self) -> dict[str, type]:
        action_features = {}
        for comp in self._components:
            if not isinstance(comp, ActionComponent):
                continue

            action_features.update(comp.features)
        return action_features

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        action = copy.deepcopy(action)

        for comp in self._components:
            if not isinstance(comp, ActionComponent):
                continue

            comp_action = {
                k: action.pop(k) for k in comp.features.keys() if k in action
            }
            if comp_action:
                comp.send_action(comp_action)

        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read components observations
        obs_dict = {}
        for comp in self._components:
            if not isinstance(comp, StateComponent):
                continue

            obs_dict.update(comp.get_state())

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def reset(self):
        pass

    @property
    def observation_features(self) -> dict[str, Any]:
        obs_features = {}
        for comp in self._components:
            if not isinstance(comp, StateComponent):
                continue

            obs_features.update(comp.features)
        for cam_key, cam in self.cameras.items():
            obs_features[cam_key] = (cam.config.height, cam.config.width, 3)
        return obs_features

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def image_feature_count(self) -> int:
        return len(self.cameras) + sum(
            1 if isinstance(component, ImageBaseComponent) else 0
            for component in self._components
        )

    def calibrate(self):
        pass

    def configure(self):
        pass

    @property
    def is_calibrated(self) -> bool:
        return True

    @property
    def is_connected(self) -> bool:
        return self._is_connected
