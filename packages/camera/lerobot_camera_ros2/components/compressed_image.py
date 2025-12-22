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

from lerobot_camera_ros2.components.common import BaseComponent
from numpy.typing import NDArray
from sensor_msgs.msg import CompressedImage
import numpy as np
from lerobot_camera_ros2.config_ros2_camera import ROS2CameraConfig
from cv_bridge import CvBridge


class CompressedImageComponent(BaseComponent):
    def __init__(self, config: ROS2CameraConfig):
        super().__init__(config, CompressedImage)

    def msg_to_data(self, msg: CompressedImage) -> NDArray[np.uint8]:
        bridge = CvBridge()
        return bridge.compressed_imgmsg_to_cv2(msg)
