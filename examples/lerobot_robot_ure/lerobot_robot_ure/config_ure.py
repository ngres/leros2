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
from leros2.robot_config import ROS2RobotConfig
from lerobot.robots.config import RobotConfig

from dataclasses import dataclass


@RobotConfig.register_subclass("ure")
@dataclass
class UReConfig(ROS2RobotConfig):
    """Configuration for the UR12e robot arm."""

    joint_state_topic: str = "/joint_states"

    pose_state_topic = "/current_pose"

    pose_action_topic: str = "/target_pose"

    wrist_image_topic: str = "/wrist_camera/color/image_raw/compressed"

    base_image_topic: str = "/left_camera/color/image_raw/compressed"
