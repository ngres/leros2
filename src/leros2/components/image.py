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

from leros2.components.common import ImageBaseComponent, ImageBaseComponentConfig
from leros2.components.common.base import BaseComponentConfig
from sensor_msgs.msg import Image
from typing import Any
import numpy as np
from dataclasses import dataclass


@dataclass
@BaseComponentConfig.register_subclass('image')
class ImageComponentConfig(ImageBaseComponentConfig):
    pass


class ImageComponent(ImageBaseComponent[ImageComponentConfig, Image]):
    """Adapter for converting an image to state features."""

    def __init__(self, config: ImageComponentConfig):
        super().__init__(config, Image)

    def to_value(self, msg: Image) -> dict[str, Any]:
        return {
            self._config.name: np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ),
        }
