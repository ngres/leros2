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
from leros2.components.pose_state import PoseStateComponent, PoseStateComponentConfig
from leros2.teleoperator import ROS2Teleoperator

from leros2.robot import ROS2Robot
from leros2.components.joint_action import (
    JointActionComponent,
    JointActionComponentConfig,
)
from leros2.components.joint_state import (
    JointStateComponent,
    JointStateComponentConfig,
    JointConfig,
)
import math

from .config_pose import PoseConfig


class Pose(ROS2Teleoperator):
    """Robot class for an URe series robot arm."""

    name = "pose"

    def __init__(self, config: PoseConfig):
        super().__init__(
            config,
            [
                PoseStateComponent(PoseStateComponentConfig(topic=config.pose_topic, name="pose")),
                JointStateComponent(  # Desired gripper state
                    JointStateComponentConfig(
                        topic=config.gripper_topic, joints=[
                            JointConfig(
                                name="gripper",
                                range_min=0.0,
                                range_max=0.7929,
                                norm_min=0.0,
                                ros_name="gripper"
                            ),
                        ]
                    )
                ),
            ],
        )
