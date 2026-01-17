# ROS 2 LeRobot

Map [ROS 2](https://www.ros.org/) topics and actions to [LeRobot](https://github.com/huggingface/lerobot) robots and teleoperators.

## Quick Start

Clone the repository and sync dependencies.

```shell
git clone https://github.com/ngres/leros2.git
cd leros2
uv sync --all-packages
```

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

This package provides a `rosbag2` converter to convert ROS 2 bag files to LeRobot datasets via the `leros2-convert` command. It behaves similar to the `lerobot-record` command-line tool and accepts all robots and teleoperators that extends the `ROS2Robot` and `ROS2Teleoperator` classes respectively.

ROS 2 messages need to be quantized into dataset frames. This can be done using one of the following methods:

- `--dataset.fps`: Use a fixed FPS rate to capture frames.
- `--clock_topic`: Use a ROS 2 topic to capture frames every time a message is published. (`--dataset.fps` should also be specified to populate the FPS metadata)

> [!IMPORTANT]
> Make sure your camera topic publish frequency and dataset FPS are the same. Ideally the camera topic should be used as the clock topic to allign properioceptive and image observations.

### Multi Episode Example

If multiple episodes are performed during the recording a `task_topic` should be specified. After each string message published with the tasked description the converter will create a new episode. If no `task_topic` is specified, only one episode will be created, starting after all required topics are recieved atleast once.

```shell
leros2-convert \
    --robot.type=ure \
    --dataset.repo_id=<my_username>/<my_dataset_name> \
    --input_bag=/path/to/your/rosbag \
    --task_topic=/task \
    --clock_topic=/camera/image_raw \
    --teleop.type=ure \
    --teleop.id=blue
```