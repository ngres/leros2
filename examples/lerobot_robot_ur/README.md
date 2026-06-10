# UR ROS 2 LeRobot

Example LeRobot interface for a Universal Robots e-series arm with an optional
Robotiq gripper and cameras.

## Configuration

The robot is configured through `URConfig` (registered as `ur`). Highlights:

- `namespace` — ROS 2 namespace prepended to every topic (default `""`, e.g.
  `ur5e` maps `/joint_states` to `/ur5e/joint_states`).
- `joint_prefix` — prefix of the arm joint names in `/joint_states`
  (default `ur_`, e.g. `ur_shoulder_pan_joint`).
- `action_space` — `"ee"` for cartesian end-effector pose control (default) or
  `"joint"` for joint-trajectory control.
- `cameras` — mapping of camera name to compressed-image topic; defaults to a
  `base` and a `wrist` camera. Set to `{}` to disable.
- `gripper` — `GripperConfig` for a parallel gripper (default), or `None` to run
  the arm without one.

State and action topics each have their own field and can be overridden
individually.
