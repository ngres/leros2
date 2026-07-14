![LeROS2](./docs/logo.svg)

Integrate any [ROS 2](https://www.ros.org/) robot or teleoperation device with [LeRobot](https://github.com/huggingface/lerobot). LeROS2 allows you to record and control robots using joint positions, end-effector poses, joint torques, or end-effector wrench actions. The framework aims to be completely composable to allow the combination of one or more robot arms with different cameras and grippers.

🦾 Customizable joint position/torque or end-effector pose/wrench support

🧩 Composable design by defining robot components (mapping between ROS 2 topics/actions and LeRobot features)

📼 Convert ROS 2 bags into LeRobot datasets

## Quick Start

Clone the repository and sync dependencies.

```shell
git clone https://github.com/ngres/leros2.git
cd leros2
```

## Config

A [LeRobot robot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/robot.py) outputs observations such as joint states and publishes actions in the form of joint trajectories. A ROS 2 LeRobot robot consists of _components_ that subscribe to topics (`StateComponent`) and publish ROS 2 messages (`ActionComponent`).

A [LeRobot teleoperator](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/teleoperator.py) subscribes to a teleportation device (e.g. leader arm, VR controller).

```yaml
robot:
  type: ros2
  state:
    - type: pose_state
      name: pose
      topic: /cartesian_controller/current_pose
    - type: joint_state
      topic: /joint_states
      joints:
        # instead of end-effector poses, joint configurations could also be added as states
        - {
            name: gripper,
            ros_name: robotiq_85_left_knuckle_joint,
            range_min: 0.0,
            range_max: 0.8,
            norm_min: 0.0,
          }
    - type: wrench_state
      name: wrench # end-effector force/torque observations
      topic: /force_torque_sensor_broadcaster/wrench

    - type: compressed_image
      name: wrist
      topic: /wrist/color/image_raw/compressed
      width: 512
      height: 512
    - type: compressed_image
      name: base
      topic: /base/color/image_raw/compressed
      width: 640
      height: 480

  action:
    - type: pose_action
      name: pose
      topic: /cartesian_controller/target_pose
      frame_id: base_link
    - type: float_array_action
      topic: /gripper_controller/external_commands
      joints:
        - { name: gripper, range_min: 0.0, range_max: 0.8, norm_min: 0.0 }

teleop:
  type: ros2
  action:
    - type: pose_state
      name: pose
      topic: /cartesian_controller/target_pose
    - type: joint_state
      topic: /gripper_controller/commands
      joints:
        - {
            name: gripper,
            ros_name: robotiq_85_left_knuckle_joint,
            range_min: 0.0,
            range_max: 0.8,
            norm_min: 0.0,
          }
```

## `rosbag2` Conversion

This package provides a `rosbag2` converter to convert ROS 2 bag files to LeRobot datasets via the `leros2-convert` command. It behaves similar to the `lerobot-record` command-line tool and accepts all robots and teleoperators that extends the `ROS2Robot` and `ROS2Teleoperator` classes respectively.

ROS 2 messages need to be quantized into dataset frames. This can be done using one of the following methods:

- `--dataset.fps`: Use a fixed FPS rate to capture frames.
- `--clock_topic`: Use a ROS 2 topic to capture frames every time a message is published. (`--dataset.fps` should also be specified to populate the FPS metadata)

### Multi Episode Example

If multiple episodes are performed during the recording a `task_topic` should be specified. After each string message published with the task description the converter will create a new episode. If no `task_topic` is specified, only one episode will be created.

```shell
leros2-convert \
    --config_path /path/to/your/config.yaml
    --dataset.repo_id <my_username>/<my_dataset_name> \
    --dataset.fps 30 \
    --input_bag /path/to/your/bag.mcap \
    --task_topic /task \
```

Checkout `leros2-convert --help` for more command options.
