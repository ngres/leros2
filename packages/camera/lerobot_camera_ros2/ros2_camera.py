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

from lerobot.cameras.camera import Camera
from lerobot_camera_ros2.config_ros2_camera import ROS2CameraConfig
from lerobot_camera_ros2.components.image import ImageComponent
from lerobot_camera_ros2.components.compressed_image import CompressedImageComponent
from lerobot.cameras import ColorMode
from numpy.typing import NDArray
from typing import Any
from lerobot_common_ros2 import ROS2Common

class ROS2Camera(ROS2Common[ROS2CameraConfig, ImageComponent], Camera):
    def __init__(self, config: ROS2CameraConfig):
        ROS2Common.__init__(self, config)
        Camera.__init__(self, config)

    def _init_components(self):
        if self.config.is_compressed:
            self._components["image"] = CompressedImageComponent(self.config)
        else:
            self._components["image"] = ImageComponent(self.config)

    @property
    def is_connected(self) -> bool:
        """Check if the camera is currently connected.

        Returns:
            bool: True if the camera is connected and ready to capture frames,
                  False otherwise.
        """
        return self._is_connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Detects available cameras connected to the system.
        Returns:
            List[Dict[str, Any]]: A list of dictionaries,
            where each dictionary contains information about a detected camera.
        """
        # [TODO] implement camera detection
        return []

    def connect(self, warmup: bool = True) -> None:
        """Establish connection to the camera.

        Args:
            warmup: If True (default), captures a warmup frame before returning. Useful
                   for cameras that require time to adjust capture settings.
                   If False, skips the warmup frame.
        """
        ROS2Common.connect(self)

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        """Capture and return a single frame from the camera.

        Args:
            color_mode: Desired color mode for the output frame. If None,
                        uses the camera's default color mode.

        Returns:
            np.ndarray: Captured frame as a numpy array.
        """
        return self._components["image"].read(color_mode)

    def async_read(self, timeout_ms: float = ...) -> NDArray[Any]:
        """Asynchronously capture and return a single frame from the camera.

        Args:
            timeout_ms: Maximum time to wait for a frame in milliseconds.
                        Defaults to implementation-specific timeout.

        Returns:
            np.ndarray: Captured frame as a numpy array.
        """
        return self._components["image"].async_read(timeout_ms)

    def disconnect(self) -> None:
        """Disconnect from the camera and release resources."""
        ROS2Common.disconnect(self)

    