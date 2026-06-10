# Copyright 2026 Nicolas Gres
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
import math

from leros2.robot import ROS2Robot
from leros2.components.joint_state import (
    JointStateComponent,
    JointStateComponentConfig,
    JointConfig,
)
from leros2.components.joint_action import (
    JointActionComponent,
    JointActionComponentConfig,
)
from leros2.components.pose_action import PoseActionComponent, PoseActionComponentConfig
from leros2.components.pose_state import PoseStateComponent, PoseStateComponentConfig
from leros2.components.wrench_state import WrenchStateComponent, WrenchStateComponentConfig
from leros2.components.compressed_image import (
    CompressedImageComponent,
    CompressedImageComponentConfig,
)
from leros2.components.parallel_gripper_action import (
    ParallelGripperActionComponent,
    ParallelGripperActionComponentConfig,
)

from .config_ur import URConfig

# UR arm joints in `/joint_states` order, as (suffix, range_min, range_max).
# The configurable `joint_prefix` is prepended to each suffix.
UR_ARM_JOINTS: list[tuple[str, float, float]] = [
    ("shoulder_pan_joint", -math.pi, math.pi),
    ("shoulder_lift_joint", -math.pi / 2, math.pi / 2),
    ("elbow_joint", -math.pi, math.pi),
    ("wrist_1_joint", -math.pi, math.pi),
    ("wrist_2_joint", -math.pi, math.pi),
    ("wrist_3_joint", -math.pi, math.pi),
]


def _with_namespace(namespace: str, topic: str) -> str:
    """Prepend the ROS 2 namespace to a topic (no-op when namespace is empty)."""
    if not namespace:
        return topic
    return f"/{namespace.strip('/')}/{topic.lstrip('/')}"


def _arm_joints(prefix: str) -> list[JointConfig]:
    return [
        JointConfig(name=f"{prefix}{suffix}", range_min=range_min, range_max=range_max)
        for suffix, range_min, range_max in UR_ARM_JOINTS
    ]


class UR(ROS2Robot):
    """LeRobot robot interface for a Universal Robots e-series arm."""

    name = "ur"

    def __init__(self, config: URConfig):
        def topic(name: str) -> str:
            return _with_namespace(config.namespace, name)

        arm_joints = _arm_joints(config.joint_prefix)

        # --- Observations ------------------------------------------------
        state_joints = list(arm_joints)
        if config.gripper is not None:
            state_joints.insert(
                0,
                JointConfig(
                    name="gripper",
                    range_min=0.0,
                    range_max=config.gripper.opened,
                    norm_min=0.0,
                    ros_name=config.gripper.state_joint,
                ),
            )

        components = [
            PoseStateComponent(
                PoseStateComponentConfig(topic=topic(config.pose_state_topic), name="pose")
            ),
            JointStateComponent(
                JointStateComponentConfig(
                    topic=topic(config.joint_state_topic), joints=state_joints
                )
            ),
            WrenchStateComponent(
                WrenchStateComponentConfig(
                    topic=topic(config.wrench_state_topic), name="wrench"
                )
            ),
        ]

        for camera_name, camera in config.ros_cameras.items():
            components.append(
                CompressedImageComponent(
                    CompressedImageComponentConfig(
                        topic=topic(camera.topic),
                        name=camera_name,
                        width=camera.width,
                        height=camera.height,
                    )
                )
            )

        # --- Actions -----------------------------------------------------
        if config.action_space == "ee":
            components.append(
                PoseActionComponent(
                    PoseActionComponentConfig(
                        topic=topic(config.pose_action_topic),
                        frame_id=config.pose_action_frame,
                        name="pose",
                    )
                )
            )
        elif config.action_space == "joint":
            components.append(
                JointActionComponent(
                    JointActionComponentConfig(
                        topic=topic(config.joint_action_topic), joints=arm_joints
                    )
                )
            )
        else:
            raise ValueError(
                f"Unknown action_space {config.action_space!r}, expected 'ee' or 'joint'."
            )

        if config.gripper is not None:
            components.append(
                ParallelGripperActionComponent(
                    ParallelGripperActionComponentConfig(
                        topic=topic(config.gripper.action_topic),
                        joints=[
                            JointConfig(
                                name="gripper",
                                range_min=0.0,
                                range_max=config.gripper.opened,
                                norm_min=0.0,
                            ),
                        ],
                    )
                )
            )

        # Cameras are provided as image components, so LeRobot's device-camera
        # handling is disabled.
        super().__init__(config, components, init_cameras=False)
