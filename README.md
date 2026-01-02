# ROS 2 LeRobot

Map [ROS 2](https://www.ros.org/) topics and actions to [LeRobot](https://github.com/huggingface/lerobot) robots and teleoperators.

## Robot

A [LeRobot robot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/robot.py) outputs observations such as joint states and publishes actions in the form of joint trajectories.

A ROS 2 LeRobot robot consists of *components* that subscribe to topics (`StateComponent`) and publish ROS 2 messages (`ActionComponent`).

```python
# file: lerobot_robot_ure/ure.py

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

class URe(ROS2Robot):
    """Robot class for an URe series robot arm."""

    name = "ure"

    def __init__(self, config: UReConfig):
        joints = [
            JointConfig(
                name="shoulder_pan_joint",
            ),
            JointConfig(
                name="shoulder_lift_joint",
                range_min=-math.pi / 2,
                range_max=math.pi / 2,
            ),
            JointConfig(
                name="elbow_joint"
            ),
            JointConfig(
                name="wrist_1_joint",
            ),
            JointConfig(
                name="wrist_2_joint",
            ),
            JointConfig(
                name="wrist_3_joint",
            ),
        ]
        
        super().__init__(
            config,
            [
                JointStateComponent(  # Read joint state
                    JointStateComponentConfig(
                        topic=config.joint_state_topic, joints=joints
                    )
                ),
                JointActionComponent(  # Send joint trajectory
                    JointActionComponentConfig(
                        topic=config.joint_trajectory_topic, joints=joints
                    )
                ),
                ImageComponent(
                    ImageComponentConfig(
                        topic=config.wrist_image_topic,
                        width=config.wrist_image_width,
                        height=config.wrist_image_height,
                    )
                ),
            ],
        )
```

For LeRobot to recognize you need place your robot class in a python package prefixed with `lerobot_robot_` and register the associated config file.

```python
# file: lerobot_robot_ure/config_ure.py

from dataclasses import dataclass
from lerobot_robot_ros2 import ROS2RobotConfig
from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("ure")
@dataclass
class UReConfig(ROS2RobotConfig):
    """Configuration for the UR12e robot arm."""

    joint_state_topic: str = "joint_state"
    joint_trajectory_topic: str = "scaled_joint_trajectory_topic/joint_trajectory"
    wrist_image_topic: str = "wrist_camera/image_raw"
    wrist_image_width: int = 640
    wrist_image_height: int = 480
```

Read more about creating custom LeRobot devices [here](https://huggingface.co/docs/lerobot/en/integrate_hardware#using-your-own-lerobot-devices-) or explore the [examples](./examples).

## Teleoperator

A [LeRobot teleoperator](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/teleoperator.py) retrieves actions from a teleportation device such as a leader arm or a VR controller.

A teleoperator can simply extend the `ROS2Teleoperator` class and initilize all state components.

## `rosbag2` Conversion

This package provides a `rosbag2` converter to convert ROS 2 bag files to LeRobot datasets. 