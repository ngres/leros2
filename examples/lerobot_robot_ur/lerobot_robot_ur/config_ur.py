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

from dataclasses import dataclass, field
from typing import Literal

from lerobot.robots.config import RobotConfig

from leros2.robot_config import ROS2RobotConfig


@dataclass
class CameraConfig:
    """A camera exposed as a compressed-image ROS 2 topic."""

    # Topic publishing `sensor_msgs/CompressedImage`.
    topic: str

    # Frame size the images are stored at in the dataset.
    width: int = 640
    height: int = 480


@dataclass
class GripperConfig:
    """A parallel gripper commanded via a ROS 2 action and observed via `/joint_states`."""

    # Action server the gripper command goal is sent to.
    action_topic: str = "/robotiq_gripper_controller/gripper_cmd"

    # Name of the gripper joint inside the `/joint_states` message.
    state_joint: str = "robotiq_85_left_knuckle_joint"

    # Joint position (radians) at which the gripper is fully open
    opened: float = 0.7929


def _default_cameras() -> dict[str, CameraConfig]:
    return {
        "base": CameraConfig(topic="/left_camera/color/image_raw/compressed"),
        "wrist": CameraConfig(topic="/wrist_camera/color/image_raw/compressed"),
    }


@RobotConfig.register_subclass("ur")
@dataclass
class URConfig(ROS2RobotConfig):
    """Configuration for a Universal Robots e-series arm."""

    # ROS 2 namespace prepended to every topic below
    namespace: str = ""

    # Prefix of the arm joint names in `/joint_states`, e.g. "ur_"
    joint_prefix: str = ""

    action_space: Literal["ee", "joint", "wrench"] = "ee"

    # --- State topics ---------------------------------------------------
    joint_state_topic: str = "/joint_states"
    pose_state_topic: str = "/current_pose"
    wrench_state_topic: str = "/force_torque_sensor_broadcaster/wrench"

    # --- Action topics --------------------------------------------------
    action_frame: str = "ur_base_link"
    pose_action_topic: str = "/target_pose_alt"
    wrench_action_topic: str = "/cartesian_compliance_controller/target_wrench"
    joint_action_topic: str = "/scaled_joint_trajectory_controller/joint_trajectory"

    # --- Cameras & gripper ----------------------------------------------
    ros_cameras: dict[str, CameraConfig] = field(default_factory=_default_cameras)
    gripper: GripperConfig | None = field(default_factory=GripperConfig)
