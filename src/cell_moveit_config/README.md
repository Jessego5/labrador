# cell_moveit_config

This is the MoveIt 2 configuration for the UR5e and Robotiq 2F-85 cell. It is written by hand
rather than with the Setup Assistant GUI; the one generated part is the SRDF's self-collision
matrix, which comes from MoveIt's `collisions_updater`.
`cell_bringup/launch/cell.launch.py` loads it through `MoveItConfigsBuilder` for both move_group
and the skill server.

| File | Contents |
|---|---|
| `config/cell.srdf` | groups `arm` (base_link to tcp) and `gripper`; named poses `home`, `observe`, `open`, `closed`; self-collision matrix |
| `config/kinematics.yaml` | KDL IK for `arm` |
| `config/joint_limits.yaml` | velocity and acceleration limits for time parameterization |
| `config/moveit_controllers.yaml` | `joint_trajectory_controller` and `gripper_controller` (ParallelGripperCommand) |
| `config/pilz_cartesian_limits.yaml` | limits for the Pilz planner |

## Regenerating the self-collision matrix

The `<disable_collisions>` entries in `cell.srdf` come from sampling 20000 random robot poses,
and each one's reason (Adjacent, Default or Never) says why that pair of links is skipped. Re-run
it after changing the robot geometry, rather than editing the entries by hand:

```bash
xacro $(ros2 pkg prefix cell_description)/share/cell_description/urdf/cell.urdf.xacro > /tmp/cell.urdf
/opt/ros/jazzy/lib/moveit_setup_assistant/collisions_updater \
  --urdf /tmp/cell.urdf --srdf config/cell.srdf --output config/cell.srdf \
  --default --always --trials 20000
```
