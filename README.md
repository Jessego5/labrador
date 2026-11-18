# Labrador

Labrador is a simulated lab-automation robot: a UR5e arm with a Robotiq 2F-85 gripper on a
bench, an overhead camera, and the software that turns a short YAML task file into finished work
without anyone watching. It sorts parts into bins and trays, inserts a peg into a fixture by feel,
and runs four small lab protocols, from watching vials change colour to harvesting culture dishes
once their colonies have grown. When things go wrong on the way, an object that moved, a grasp
that slipped, a part dropped in transit, it notices from what it sees and feels and recovers.

It is built on ROS 2 Jazzy, MoveIt 2, BehaviorTree.CPP and MuJoCo, in C++ for everything that
moves the arm and Python for the benchmarks and the lab chemistry. Every claim below comes from
a seeded benchmark that anyone can run again.

## Build and run

On Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
ros2 launch cell_bringup cell.launch.py task:=sort_and_insert
```

That opens the simulator and RViz, and the robot sorts two red cubes into a bin, four blue
cylinders into a tray and inserts the yellow peg. `scene:=lab` gives the lab bench instead, where
the tasks are `color_assay`, `titration`, `growth` and `readout`, and `headless:=true rviz:=false`
runs it without windows, as the benchmarks do.

## How it fits together

```
  task file (YAML)
        |
        v
  cell_executor ----------> behaviour tree: steps, loops over objects, retries and recoveries
        |                    pause / resume / abort, a JSON log of every run, live in Groot2
        | actions
        v
  cell_skills ------------> MoveToNamedPose, DetectObjects, Pick, Place, Insert
   |        |        |
   |        |        +----> cell_controllers: admittance control on the wrist F/T sensor
   |        +-------------> cell_perception: colour and depth detection on the overhead camera
   +----------------------> MoveIt 2 (cell_moveit_config), targets from cell_workspace
        |
        v
  MuJoCo through ros2_control (cell_description) <---- cell_monitor injects faults
                                                 <---- cell_lab runs the lab chemistry
```

A task names what to do with which objects (`pick class: cube`, `place target: bin_a`), never
where anything is. The workspace model resolves names like `tray_1/C3` to poses, perception finds
the objects, and the skills work out the motion. The same skills serve the sorting task and the
lab protocols.

## Results

Each part was finished only when its benchmark passed, and the earlier benchmarks were rerun
after changes that could affect them.

| Part | What it added | Benchmark | Result |
|---|---|---|---|
| 1, 2 | the simulated cell and its workspace model | URDF and MJCF kinematics agree, workspace config and scene in sync | checked by tests |
| 3 | MoveIt and the Pick and Place skills | 10 pick and place trials | 10/10, about 1.3 mm placement error |
| 4 | perception on the overhead camera | 50 picks from randomized layouts, 400 detections | 50/50 picks, 400/400 detections, 0.72 mm mean error |
| 5 | compliant peg insertion | 30 insertions with the fixture up to 3 mm off | 30/30 |
| 6 | the task executor | `sort_and_insert` on 10 layouts, checked against ground truth | 10/10 |
| 7 | lab protocols | 10 seeds of each protocol, every decision scored | 130/130 decisions right |
| 8 | fault injection and recovery | 100 runs without faults and 100 with 450 injected faults | 96% and 96% of runs passed, 98% of faults recovered |

The reliability report with its plots is in `docs/reliability`, and each package README has the
details of its own benchmark.

## Packages

| Package | What it is |
|---|---|
| `cell_description` | the robot, the bench and the objects as MJCF and URDF, generated from a few YAML files |
| `cell_workspace` | trays, slots, bins and fixtures resolved to poses, as a ROS-free library and a node |
| `cell_moveit_config` | MoveIt 2 configuration for the arm and gripper |
| `cell_perception` | the colour and depth detector behind `/perception/detect` |
| `cell_controllers` | a Cartesian admittance controller for ros2_control |
| `cell_skills` | the five skills as ROS 2 actions |
| `cell_executor` | YAML tasks run as behaviour trees, with recovery |
| `cell_lab` | the lab scene's chemistry: colour changes, dosing, colony growth |
| `cell_monitor` | the fault injector the reliability benchmark runs against |
| `cell_benchmarks` | seeded layouts, the benchmark runners and the report |
| `cell_interfaces` | the actions, services and messages |
| `cell_bringup` | the launch files |

## Design decisions

- **The UR5e over the Franka Panda.** A comparison in MuJoCo before anything else was built
  gave the UR5e more reach over the bench (94% of target points against 86%), more joint-limit
  margin and better manipulability at the cell's targets.
- **Behaviour trees generated from YAML.** A task file is short enough to write by hand and
  check by eye, while the tree it becomes gives pause, abort, retries and live monitoring for
  free. Failure handling is part of the task (`retries: 2`, `on_failure: skip_step`), not code.
- **Insertion by feel, not by vision.** The hole leaves the peg 1 mm of radial clearance, less
  than a fixture can be off where the workspace says it stands, so Insert touches off on the
  fixture top and pecks a 1.5 mm grid under admittance control, which finds a hole up to 6 mm
  from where it should be.
- **Recovery decided by what failed.** A missed grasp looks again, a dropped part is found and
  picked up where it landed, and a place that still holds its part goes home and tries again.
  Errors another try cannot change, an abort or an unknown target, stop at once.
- **Faults injected at the worst moment.** The monitor reads the skills' feedback and nudges an
  object as the gripper descends or drops it in transit, rather than at random times, and the
  executor is never told. The benchmark measures recovery, not luck.

## Licence

Labrador is released under the Apache License 2.0, see `LICENSE`. The UR5e and 2F-85 meshes in
`cell_description/mjcf/assets` come from the MuJoCo Menagerie and keep their own BSD licences,
which sit beside them.
