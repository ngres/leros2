![LeROS2](./docs/logo.svg)

Integrate any [ROS 2](https://www.ros.org/) robot or teleoperation device with [LeRobot](https://github.com/huggingface/lerobot). LeROS2 allows you to record and control robots using joint positions, end-effector poses, joint torques, or end-effector wrench actions. The framework aims to be completely composable to allow the combination of one or more robot arms with different cameras and grippers.

🦾 Customizable joint position/torque or end-effector pose/wrench support

🧩 Composable design by defining robot components (mapping between ROS 2 topics/actions and LeRobot features)

📼 Convert ROS 2 bags into LeRobot datasets

## Quick Start (Inference)

1. Install the LeRobot support package wrapping LeROS2:

```shell
uv add lerobot-robot-ros2 lerobot-teleoperator-ros2
```

2. Create a config file, mapping your topics and actions to LeRobot features:

```yaml
# file: my-config.yaml

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

3. Deploy the policy:

```shell
lerobot-rollout \
    --config-path=./my-config.yaml
    --strategy.type=base \
    --policy.path=${HF_USER}/my_policy \
    --task="pick up cube" \
```

## Recording

LeROS2 is compatible with the `lerobot-record` command to capture LeRobot datasets directly. However, this requires places the burden of mirroring the teleportation device actions on the LeRobot Python record loop, which can introduce additional latency.

Therefore, it is recommended to connect the teleoperation device natively via ROS 2 (i.e. publish the action topics directly to the desired robot subscribers) and record each episode into a [ROS 2 bag](https://github.com/ros2/rosbag2#recording-data-).

These raw recordings have the additional benefit of containing a higher temporal (i.e. native frequencies) and spacial (i.e. ) resolution.

### `rosbag2` Conversion

```
uv add leros2[dataset]
```

This package provides a `rosbag2` converter to convert ROS 2 bag files to LeRobot datasets via the `leros2-convert` command. It behaves similar to the `lerobot-record` command-line tool and accepts all robots and teleoperators that extends the `ROS2Robot` and `ROS2Teleoperator` classes respectively.

ROS 2 messages need to be quantized into dataset frames. This can be done using one of the following methods:

- `--dataset.fps`: Use a fixed FPS rate to capture frames.
- `--clock_topic`: Use a ROS 2 topic to capture frames every time a message is published. (`--dataset.fps` should also be specified to populate the FPS metadata)

#### Multi Episode Example

If multiple episodes are performed inside a single bag a `task_topic` should be specified. After each string message published with the task description the converter will create a new episode. If no `task_topic` is specified, only one episode will be created.

```shell
leros2-convert \
    --config_path=./my-config.yaml
    --dataset.repo_id=${HF_USER}/my_dataset \
    --input_bag=/path/to/your/bag.mcap \
```

Alternatively, a glob can be specified, to convert multiple bags containing each one episode into a single dataset:

```shell
leros2-convert \
    --config_path=./my-config.yaml
    --dataset.repo_id=${HF_USER}/my_dataset \
    --input_bag=./recordings/episodes/*/*.mcap \
```

Checkout `leros2-convert --help` for more command options.
