# cell_lab

This is the lab scene's chemistry, the part of the simulation that plays the real world for the
lab protocols. MuJoCo has no chemistry and cannot recolour a body while it runs, so every sample
has a visual twin for each look it can have, all parked on the floor under the table, and
`lab_node` shows a change by teleporting the right twin into the sample's place and parking the
old one in a single `set_free_joint_state` call. The robot never reads this package's state: it
sees a change only through the camera, as a real cell would, and only the benchmark reads the
truth kept here to score what the robot did.

```bash
ros2 launch cell_bringup cell.launch.py scene:=lab        # starts lab_node with the cell
ros2 service call /lab/reset cell_interfaces/srv/LabReset "{protocol: assay, seed: 3}"
ros2 service call /lab/dispense std_srvs/srv/Trigger       # one dose into the dispenser's vial
ros2 topic echo --full-length /lab/state                   # the true state, JSON
```

## The protocols

Both are defined in `cell_description/config/lab/lab.yaml`, with the samples' looks and start
slots; `chemistry.py` holds the rules.

| Protocol | Samples | What happens |
|---|---|---|
| `assay` | four vials in `rack_in` row A | each starts blue and, after a reaction time drawn between 20 and 90 s, turns green (worked) or brown (failed, one in four on average) |
| `titration` | two vials in `rack_in` row B | each dose moves a vial along yellow, orange, red, purple; a step takes 1 to 3 doses, drawn per vial, up to red, the endpoint, and one dose past red shows purple |
| `growth` | three culture dishes on the table | each dish's colony grows through five stages (radius 4 to 18 mm), one stage after a time drawn between 15 and 35 s; the dish should be harvested once the colony covers 600 mm^2, which is stage 3 |
| `readout` | four test tubes in `rack_in` row A | each tube shows one point of an indicator scale, drawn per tube: strong or weak negative, borderline, weak or strong positive; nothing changes during the run |

Every draw comes from one random generator seeded by `/lab/reset`, so a seed always gives the
same run. A dose shows after `mix_time` (1.5 s), as an indicator takes a moment to turn. A
change that falls due while the sample is in the gripper (lifted above `held_height`, 65 mm, or
moving) waits until it is set down, because teleporting a held body would pull it out of the
fingers.

## Interfaces

| | Name | Type |
|---|---|---|
| service | `/lab/reset` | `cell_interfaces/LabReset`: set out one protocol's samples with a seed |
| service | `/lab/dispense` | `std_srvs/Trigger`: one dose into the vial standing in the dispenser |
| topic | `/lab/state` | `std_msgs/String`, JSON: each sample's look, state, doses, the plan drawn for it and its events (set out, changed, dosed, lifted, with simulation times), twice a second and after every change |

## Why the lab table looks the way it does

Two findings shaped `config/lab/workspace.yaml` and `lab.yaml`, and are worth knowing before
changing them:

- The colours were chosen on the rendered image. The light overexposes the vials' top faces, and
  a pink tried first read hue 133 to 150 there, inside purple's 125 to 138, so a finished assay
  shows green instead. The test tubes' scale is kept dark for the same reason: unexposed, it
  renders at exactly the hues it was given (60 to 176 degrees), and its value sets it apart from
  the bright vials of the same hues.
- The racks are spaced for the gripper and the detector, not for density. At 60 mm along the
  fingers' closing direction the open fingers came down on the neighbouring vial, which left the
  picked one hanging 15 to 20 degrees and tipping over wherever it was set down; the columns are
  80 mm apart. At 45 mm the walls between rows were thin enough for the detector to join four
  vials into one blob; the rows are 60 mm apart.

## The camera noise

The lab scene turns on perception's `color_noise` (0.05): every frame gets a random gain on each
colour channel, standing in for a real camera's frame-to-frame white-balance drift. It is what
makes the readout's second look worth having, since six averaged frames have a sixth of the
variance of one.

## Tests

`test_chemistry` steps a clock through each protocol: a seed always draws the same futures, an
assay vial changes once at its reaction time and not while held, the failure rate comes out near
a quarter over many seeds, a titration reaches red at exactly its planned dose and purple one
dose later, a colony grows one stage per drawn interval and is ready at stage 3, a readout tube
keeps the look it was given, and the parking grid matches the MJCF that `build_lab_mjcf.py`
wrote. Run them with
`colcon test --packages-select cell_lab`.
