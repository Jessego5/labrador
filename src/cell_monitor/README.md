# cell_monitor

This is the fault injector the reliability benchmark runs the cell against. A cell that only
ever meets the scene it was tuned on proves little, so between `/monitor/start_run` and
`/monitor/end_run` this node makes the things go wrong that go wrong on a real bench: objects
that are not where they were seen, grasps that do not hold, parts that fall out on the way, a
fixture a few millimetres off and a camera whose colours drift. The executor and the skills are
never told. They have to notice from what they see and feel, and recover (see "Recovery" in
`cell_executor/README.md`).

`cell.launch.py` starts it on the default table, idle until a run starts.
`run_task_trials --faults ...` (in `cell_benchmarks`) arms it before each task and collects the
faults after.

## The faults

Each fault waits for the moment its skill is most exposed, which the node reads from the skills'
action feedback rather than from `/executor/state`, since that only updates twice a second and a
descent lasts about one.

| Fault | When | What happens in the simulator | How the cell notices |
|---|---|---|---|
| `nudge` | a Pick starts its descent | the object is moved 45 to 65 mm along the TCP's x axis, across the fingers' closing direction, so they close on nothing | the gripper closes past 0.75 rad: `GRASP_FAILED` |
| `slip` | a Pick starts its lift | the object leaves the closed fingers and lands 6 to 20 cm away | the post-lift check finds the fingers closed |
| `drop` | 0.3 s into a Place or Insert transport | the held object lands 6 to 20 cm from below the gripper | the arrival check finds the fingers closed |
| `fixture_shift` | the start of the run | the fixture moves 2 to 4 mm in a random direction | Insert's touch-off and peck search, which cover 6 mm |
| `camera_noise` | the whole run | perception's `color_noise` is set to 0.03, a random gain per colour channel and frame | the detector's colour ranges and the executor's matching |

Every moved object lands upright on free table: inside the benchmark layout's region and reach
band, outside every tray, bin and fixture keep-out, and at least the layout's minimum spacing from
every other object, by the same rules `cell_benchmarks/scene.py` lays the objects out with. A
nudge that has nowhere free to go is not injected. `rate` is the chance that each opportunity
gets its fault, and an object gets at most one fault a run, so the benchmark measures recovery
from a fault rather than the odds of several in a row on one cube. Each object meets three
opportunities (its descent, its lift and its transport), so at the benchmark's rate of 0.15 about
four in ten objects get a fault. The draws come from one
generator seeded per run, and every opportunity draws whether its fault is on or not, so turning
one fault off does not move where the others land.

## Results

Over 100 runs of `sort_and_insert` at a rate of 0.15 the monitor injected 450 faults and the
cell recovered 442 of them (98%), with 96 of the 100 runs passing, the same as without faults.
The numbers, the plots and the failures are in "Reliability (part 8)" in
`cell_executor/README.md`, and the full report in `docs/reliability`.

## Interfaces

| Kind | Name | Type | |
|---|---|---|---|
| service | `/monitor/start_run` | `cell_interfaces/StartRun` | arm the faults (`faults`, `rate`, `seed`) and start a rosbag in `bag_dir` if given |
| service | `/monitor/end_run` | `std_srvs/Trigger` | disarm, stop the bag, reset the camera noise; the message is `{"run_id", "faults": [...]}` |
| topic | `/monitor/faults` | `std_msgs/String` | one JSON record per fault: `t`, `kind`, `skill`, `object`, the executor's `step` and `item` at the time, `from` and `to` |

The bag is MCAP, with the executor state, the faults, joint states, TF, the true object poses and
the skills' feedback and status, about 20 MB for a 110 s task. The camera images are left out,
which would make every bag hundreds of megabytes. `ros2 bag play` replays a run into RViz, and
`ros2 bag info` lists what it holds.

## Tests

`test_faults` checks the decisions in `faults.py` against a stub layout: the same seed gives the
same faults, the rate holds and a fault that is off still draws, a nudge goes across the fingers
to the nearest free distance and never onto an object or a keep-out, a dropped object lands on
free table, and the fixture shift stays within 2 to 4 mm. Run them with
`colcon test --packages-select cell_monitor`.
