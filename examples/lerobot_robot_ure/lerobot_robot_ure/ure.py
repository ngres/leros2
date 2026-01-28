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

from leros2.components.pose_action import PoseActionComponent, PoseActionComponentConfig
from leros2.components.pose_state import PoseStateComponent, PoseStateComponentConfig
from leros2.components.compressed_image import CompressedImageComponent, CompressedImageComponentConfig

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

from .config_ure import UReConfig

URE_JOINTS = [
    JointConfig(
        name="gripper",
        range_min=0.0,
        range_max=0.7929,
        norm_min=0.0,
        ros_name="robotiq_85_left_knuckle_joint"
    ),
    JointConfig(
        name="ur_shoulder_pan_joint",
        range_min=-math.pi,
        range_max=math.pi,
    ),
    JointConfig(
        name="ur_shoulder_lift_joint",
        range_min=-math.pi / 2,
        range_max=math.pi / 2,
    ),
    JointConfig(
        name="ur_elbow_joint",
        range_min=-math.pi,
        range_max=math.pi,
    ),
    JointConfig(
        name="ur_wrist_1_joint",
        range_min=-math.pi,
        range_max=math.pi,
    ),
    JointConfig(
        name="ur_wrist_2_joint",
        range_min=-math.pi,
        range_max=math.pi,
    ),
    JointConfig(
        name="ur_wrist_3_joint",
        range_max=math.pi,
    ),
]


class URe(ROS2Robot):
    """Robot class for an URe series robot arm."""

    name = "ure"

    def __init__(self, config: UReConfig):
        joints = URE_JOINTS.copy()

        super().__init__(
            config,
            [
                PoseStateComponent(
                    PoseStateComponentConfig(
                        topic=config.pose_state_topic,
                        name="pose"
                    )
                ),
                JointStateComponent(  # Read joint states
                    JointStateComponentConfig(
                        topic=config.joint_state_topic, joints=joints
                    )
                ),
                CompressedImageComponent(
                    CompressedImageComponentConfig(
                        topic=config.base_image_topic, 
                        name="base",
                        width=640,
                        height=480
                    )
                ),
                CompressedImageComponent(
                    CompressedImageComponentConfig(
                        topic=config.wrist_image_topic, 
                        name="wrist",
                        width=640,
                        height=480
                    )
                ),
                # actions
                PoseActionComponent(
                    PoseActionComponentConfig(
                        topic=config.pose_action_topic,
                        name="pose"
                    )
                ),
                # - or -
                # JointActionComponent(
                #     JointActionComponentConfig(
                #         topic=config.joint_trajectory_topic, joints=joints
                #     )
                # ),
            ],
        )
