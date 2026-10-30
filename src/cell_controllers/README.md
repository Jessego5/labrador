# cell_controllers

This is `cell_controllers/CartesianComplianceController`, a ros2_control Cartesian admittance
controller for position-controlled arms, and the notes on how it was tuned. The UR5e in MuJoCo is
driven by stiff joint position servos, as a real UR is, and the sim rejects effort commands on
position actuators, so rather than replacing the servos with torque control the controller keeps
them and makes the commanded pose compliant. The skills switch it in, in place of
`joint_trajectory_controller`, for compliant placing and for insertion.

```
wrist F/T sensor --> zero (at activation) --> low-pass --> deadband --> clamp --+
                                                                                |
                                                                                v
reference pose --> rate limit --> M (a) + D (v - v_ref) + K (x_c - x_ref) = F_ext
(~/target_pose)                                         |
                                                        v  compliant pose x_c
                       damped least-squares IK step (KDL) --> joint position commands
```

In free space the compliant pose `x_c` follows the reference. In contact it settles where the
virtual spring balances the measured force, so the contact force is K times how far the reference
is pushed past the surface (400 N/m × 7.5 mm = 3 N, for example). Damping acts on the velocity
relative to the reference, so the heavy damping that stable contact needs does not make
free-space motion lag. For safety there is a force and torque clamp, velocity limits on the
compliant frame and on every joint, and a box the TCP has to stay inside.

## Interfaces

| | |
|---|---|
| command | `<joint>/position` for the six arm joints (the same as `joint_trajectory_controller`; the two are switched, never active together) |
| state | `<joint>/position`, `wrist_ft_fts/{force,torque}.{x,y,z}` |
| `~/target_pose` (in) | `geometry_msgs/PoseStamped`, TCP reference in `base_frame` |
| `~/current_pose` (out) | measured TCP pose |
| `~/wrench` (out) | external wrench on the tool at the TCP, base frame, zeroed at activation, filtered |

The parameters are documented in `cell_description/config/ros2_controllers.yaml`.

## Tuning notes

The sensor sign is the first thing to get right. MuJoCo's `force` and `torque` sensors give the
wrench the child body (the gripper) receives from its parent, and `mujoco_ros2_control` negates
both, so the exported reading is what the tool applies at the sensor: the gripper's weight plus
the external wrench. After zeroing, the change is the external wrench. Getting this wrong is
positive feedback, the arm pushing harder the more it is resisted; the wrong sign gave -60 N.

Contact stability decided the mass and damping. Against the servoed arm, about 2·10⁴ N/m at the
TCP, the loop is a stiff spring on the virtual mass. With M = 2 kg and critical damping for K
alone it chattered between 0 and 22 N. M = 5 kg and D = 400 N·s/m (ζ ≈ 0.6 in contact) holds a
steady 7.1 N.

The stiffness is different along each direction. Lateral stiffness (2000 N/m) is high so a pressed
part can be moved without lagging millimetres behind through friction; axial (400 N/m) is soft so
contact forces stay small; rotation (100 N·m/rad) keeps a part resting on a hole rim from tilting
into it.

## Tests

`test/test_admittance.cpp` covers the pose algebra, the clamps, the deadband, a steady-state
deflection of F/K, no overshoot when critically damped, tracking a moving reference without lag,
the velocity and workspace clamps, the KDL FK and Jacobian (against finite differences) and the IK
step.
