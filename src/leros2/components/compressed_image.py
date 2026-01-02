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
from leros2.components.common.state import StateComponent

from cv_bridge import CvBridge
from leros2.components.image import ImageComponentConfig
from sensor_msgs.msg import CompressedImage


class CompressedImageComponent(StateComponent[ImageComponentConfig, CompressedImage]):
    def __init__(self, config: ImageComponentConfig):
        super().__init__(config, CompressedImage)

        self.bridge = CvBridge()

    def to_value(self, msg: CompressedImage) -> dict[str, Any]:
        return {
            self._config.name: self.bridge.compressed_imgmsg_to_cv2(msg),
        }
