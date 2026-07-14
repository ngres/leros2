# lerobot_teleoperator_ros2

A generic, component-driven [LeRobot](https://github.com/huggingface/lerobot)
`Teleoperator` backed by ROS 2. Registered as `ros2`.

A teleoperator only reads, so it is assembled from a single list of `state`
component configs (e.g. `pose_state`, `joint_state`, `wrench_state`,
`float_array_state`). Each entry provides part of the teleop action.

Each entry is a `draccus` choice type discriminated by a `type` key. Example YAML:

```yaml
teleop:
  type: ros2
  node_name: lerobot_teleoperator
  state:
    - type: pose_state
      topic: /target_pose
      name: pose
    - type: joint_state
      topic: /target_gripper
      joints:
        - name: gripper
          range_min: 0.0
          range_max: 0.7929
          norm_min: 0.0
          ros_name: gripper
```

See `leros2.components` for the full set of available state component types and
their fields.
