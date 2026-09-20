# cell_executor

This is the task executor, the part of the cell that runs whole jobs rather than single skills.
A task is a YAML list of steps. The executor turns it into a
[BehaviorTree.CPP v4](https://www.behaviortree.dev/) tree whose leaves call the `cell_skills`
action servers, and ticks that tree until the task succeeds, fails or is aborted. A running task
can be paused, resumed or aborted from outside, every run leaves a log behind, and the running
tree can be watched live in Groot2. Run a task with:

```bash
# run a task at startup
ros2 launch cell_bringup cell.launch.py task:=sort_and_insert

# or send one to a running cell
ros2 action send_goal --feedback /executor/execute_task cell_interfaces/action/ExecuteTask \
  "{task_file: sort_and_insert}"

ros2 service call /executor/pause  std_srvs/srv/Trigger   # finish the running skill, then hold
ros2 service call /executor/resume std_srvs/srv/Trigger
ros2 service call /executor/abort  std_srvs/srv/Trigger   # cancel the running skill and stop
ros2 topic echo /executor/state                            # JSON: step, object, skill, phase
ros2 run cell_executor tree_status.py                      # running branch of the live tree
```

## Task files

This is the reference task, `tasks/sort_and_insert.yaml`:

```yaml
name: sort_and_insert
on_failure: safe_stop          # or: skip_step, abort
steps:
  - detect: {}
  - for_each:
      filter: {color: red}
      do:
        - pick: {object: $item}
        - place: {target: bin_a}
  - for_each:
      filter: {class: cylinder}
      max: 4
      do:
        - pick: {object: $item}
        - place: {target: tray_1/C$index}   # C1..C4
  - pick: {object: peg_0}
  - insert: {target: peg_fixture, max_attempts: 3}
  - move_to: {pose: home}
```

The cylinder loop filters on `class: cylinder` rather than on the shape, because the peg's shape
is a cylinder too.

| Step | Arguments | Skill |
|---|---|---|
| `detect` | `class`, `color` (optional filters) | `DetectObjects` |
| `pick` | `object` (required), `grasp` | `Pick` |
| `place` | `target` (required), `compliance: true/false` | `Place` |
| `insert` | `target` (required), `max_force` (N), `max_attempts` | `Insert` |
| `move_to` | `pose` (required, SRDF named pose) | `MoveToNamedPose` |
| `for_each` | `do` (required, list of steps), `filter`, `max` | none |

A step with one main argument can be written as a scalar, so `- move_to: home`,
`- place: bin_a` and `- detect` are all valid.

Objects come from the most recent `detect`. A step refers to one by its detection id
(`blue_cylinder_2`) or by `<class>_<n>` or `<type>_<n>`, the n-th detection of that class or
type ordered by id, so `peg_0` is the first detected peg.

`for_each` runs its `do` steps once per detected object that matches `filter` (the fields are
`class`, `color` and `type`), at most `max` times, skipping objects that have already been put
away. The objects are chosen when the loop starts. Inside the loop `$item` is the object id and
`$index` its 1-based number, and they work in any argument, as in `tray_1/C$index`. Loops do
not nest.

Bins have drop spots. `workspace.yaml` gives a bin a `spots: [rows, cols]` grid, and
`place: bin_a` puts each object on the next free spot (`bin_a/1`, `bin_a/2`, ...). The spots are
there because two cubes placed at the bin centre landed on each other; with them the objects end
up side by side. Once every spot is used, the executor starts again at spot 1 and logs a
warning.

When a step does not give `grasp`, the grasp strategy comes from the object class through the
`grasp_strategies` parameter. The default is `peg:insert`, the high grasp that keeps the
fingertips above the fixture.

Places into tray slots use the compliance controller (parameter `place_compliance`, default
true). Bins always use position control, and the object is released 3 mm above the floor. A
compliant release on a bin floor jammed: the cube stayed pinned under the fingers, and the
self-adapting 2F-85 pads dragged on it as they opened, so the gripper settled at about 0.36 rad
instead of the free 0.31 and lifting met about 20 N.

The executor checks the whole file before the robot moves: unknown steps or arguments, wrong
types, `$item` used outside a loop, and unknown workspace targets. Every error gives the line
number.

### Failure policy

A step fails when its skill fails after the skill's own retries; Insert, for example, already
searches and retries `max_attempts` times before it gives up.

| `on_failure` | Effect |
|---|---|
| `safe_stop` (default) | Put down whatever the gripper holds in `bin_b`, go home, end as `safe_stopped` (`trees/safe_stop.xml`) |
| `skip_step` | Log the failure and continue. Inside a `for_each`, only that object is skipped. |
| `abort` | Stop where the robot is, end as `failed` |

There are no recovery subtrees yet. A failed step is not re-detected, retried or tried with
another grasp before it is declared failed.

## The generated tree

Each top-level step becomes a `Step` decorator, which reports progress and applies the failure
policy. Under `safe_stop` the task's `Sequence` sits in a `Fallback` whose second branch is the
`SafeStop` subtree:

```xml
<Fallback name="task_or_safe_stop">
  <Sequence name="sort_and_insert">
    <Step index="1" title="detect" on_failure="safe_stop"><DetectObjects .../></Step>
    <Step index="2" title="for_each color=red" on_failure="safe_stop">
      <ForEachObject filter="color=red" max="0" on_failure="safe_stop">
        <Sequence name="per_object">
          <Pick object="$item" grasp=""/>
          <Place target="bin_a" compliance="default"/>
        </Sequence>
      </ForEachObject>
    </Step>
    ...
  </Sequence>
  <Sequence name="safe_stop"><SubTree ID="SafeStop"/><AlwaysFailure/></Sequence>
</Fallback>
```

| Node | Kind | |
|---|---|---|
| `DetectObjects`, `Pick`, `Place`, `Insert`, `MoveToNamedPose` | action | one per skill (`skill_nodes.hpp`) |
| `Step` | decorator | progress, failure policy |
| `ForEachObject` | decorator | loop over detected objects, sets `item` / `index` on the blackboard |
| `IsHolding` | condition | the gripper holds an object |

The executor loads every `.xml` file in `trees/` except files whose names start with '.'.
AppleDouble `._*.xml` files that came along in a copy from macOS broke tree loading.

The skill nodes share one small asynchronous wrapper, `SkillNode<ActionT>`. It works like
BehaviorTree.ROS2's `RosActionNode`, and exists because BehaviorTree.ROS2 is not packaged for
Jazzy. It waits for the action server and then sends the goal. If the skill server is still busy
finishing the previous goal (it rejects goals while busy), it resends the goal for a few seconds.
With the GUIs up the skill server also answers NOT_READY for a while after its actions appear;
the executor first failed the task in 0.1 s on that, so the node now waits and resends for up to
`ready_wait`. It checks for the result on every tick, enforces a timeout per skill (the
`timeouts.*` parameters) and cancels the goal when halted. The tick thread never blocks; the ROS
executor delivers the responses.

## Interfaces

| | Name | Type |
|---|---|---|
| action | `/executor/execute_task` | `cell_interfaces/ExecuteTask`: goal `task_file` (a bare name is looked up in `tasks/`) or `task_yaml` (inline YAML); feedback is the current step, object, skill and phase; the result gives the outcome (`succeeded`, `failed`, `safe_stopped`, `aborted`, `invalid`) and counts |
| service | `/executor/pause`, `/executor/resume`, `/executor/abort` | `std_srvs/Trigger` |
| topic | `/executor/state` | `std_msgs/String`, JSON, 2 Hz |
| ZeroMQ | tcp port 1667 (`groot_port`) | Groot2 live view (`BT::Groot2Publisher`) |

Pause stops ticking the tree. The skill that is already running finishes and nothing new starts
until resume, so the robot never stops in the middle of a grasp. Abort halts the tree, which
cancels the running skill goal, and the skill server then stops the motion safely. Cancelling
the action goal is the same as abort. Abort while paused used to crash the executor: the running
skill had finished during the pause, so cancelling its forgotten goal threw
UnknownGoalHandleError. The cancel is now guarded, and a skill that finished during the pause
has its real result recorded instead.

## Run output

Every run writes `~/cell_ws/runs/task_<name>_<time>/` (the `log_dir` parameter):

| File | Contents |
|---|---|
| `task.yaml` | the task as run |
| `tree.xml` | the generated tree |
| `log.jsonl` | one JSON record per skill call and per step: `t`, `step`, `item`, `skill`, `target`, `result`, `error`, `message`, `duration_s` and skill extras (gripper position, insertion depth, attempts, force) |
| `tree.btlog` | every node transition, for replay in Groot2 |
| `summary.json` | outcome, step counts, skill calls and failures, duration |

## Groot2

The executor publishes the running tree on ports 1667 and 1668. Lima forwards guest ports to
the Mac's localhost, so Groot2 can connect in Monitor mode to `localhost:1667` from wherever it
runs. Where Groot2 is not available, on the ARM64 VM for example, `tree_status.py` speaks the
same protocol from a terminal. `tree.btlog` opens in Groot2's log replay.

## Acceptance (part 6)

`ros2 run cell_benchmarks run_task_trials --seeds 0-9` runs `sort_and_insert` unattended on the
default layout (seed 0) and on randomized layouts, then checks the result against the
simulator's ground truth: both red cubes are in `bin_a`, a cylinder is seated in each of
`tray_1/C1..C4`, the peg is inserted to full depth, and the green cube was not disturbed.

It passed 10 of 10. The slot error was about 1.9 mm, which is cylinders resting against a pocket
wall and within the 2 mm clearance, the peg error about 0.37 mm, and a task took about 107 s.

## Tests

`test_task` covers the parser, the validation messages, XML generation and variable expansion.
`test_tree` runs generated trees with the real `Step`, `ForEachObject`, `IsHolding` and
`SafeStop` nodes and stub skills, and checks the call order, the bin spots and each failure
policy. Both run without ROS or a robot.
