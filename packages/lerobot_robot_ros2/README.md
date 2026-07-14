# lerobot_robot_ros2

A generic, component-driven [LeRobot](https://github.com/huggingface/lerobot) `Robot`
backed by ROS 2. Registered as `ros2`.

Rather than hard-coding a specific arm, the robot is assembled from two lists of
component configs:

- `state`: observation-producing components (subscriptions), e.g. `joint_state`,
  `pose_state`, `wrench_state`, `image`, `compressed_image`, `float_array_state`.
- `action`: action-consuming components (publishers / action clients), e.g.
  `joint_action`, `pose_action`, `wrench_action`, `parallel_gripper_action`,
  `float_array_action`.

Each entry is a `draccus` choice type discriminated by a `type` key. Example YAML:

```yaml
robot:
  type: ros2
  node_name: lerobot_robot
  state:
    - type: joint_state
      topic: /joint_states
      joints:
        - name: shoulder_pan_joint
        - name: shoulder_lift_joint
    - type: pose_state
      topic: /current_pose
      name: pose
  action:
    - type: joint_action
      topic: /scaled_joint_trajectory_controller/joint_trajectory
      joints:
        - name: shoulder_pan_joint
        - name: shoulder_lift_joint
```

See `leros2.components` for the full set of available component types and their fields.
