"""
These are the decisions behind cell_monitor's faults, kept free of ROS so they can be tested
without a simulator: whether an opportunity gets a fault, where a moved or dropped object may
land, and how far the fixture shifts. Every random draw comes from one seeded generator, so a
run with the same seed meets the same faults at the same opportunities.

A fault is only worth injecting if the cell can notice it, and only fair if it leaves the scene
recoverable, so a moved object always lands on free table inside the benchmark layout's region
and reach band and away from the trays, bins, fixture and other objects (the rules of
cell_benchmarks.scene). The faults are the ones the plan lists for the cell:

  nudge          the object is moved 45 to 65 mm while the gripper descends to it, along the
                 direction the fingers do not close in, so they close on nothing
  slip           the object leaves the closed gripper as the lift starts, a weak grasp
  drop           the object falls out of the gripper on the way to its place or the fixture
  fixture_shift  the fixture is moved 2 to 4 mm before the run, so the insert has to search
  camera_noise   every camera frame gets a random colour gain (perception's color_noise)
"""
import math
import random

KINDS = ("nudge", "slip", "drop", "fixture_shift", "camera_noise")
NUDGE_DISTANCES = (0.045, 0.055, 0.065)  # m; the fingertips are 22 mm wide, a cube 40 mm
DROP_RADIUS = (0.06, 0.20)               # m from below the gripper, where a dropped object lands
FIXTURE_SHIFT = (0.002, 0.004)           # m; the insert's peck search covers 6 mm
CAMERA_NOISE = 0.03                      # std dev of the per-channel gain


class FaultPlan:
    """Which faults a run may have, how often, and the seeded generator they all draw from."""

    def __init__(self, faults, rate, seed):
        unknown = set(faults) - set(KINDS)
        if unknown:
            raise ValueError(f"unknown fault(s) {sorted(unknown)} (known: {', '.join(KINDS)})")
        if not 0.0 <= rate <= 1.0:
            raise ValueError("rate must be between 0 and 1")
        self.faults = set(faults)
        self.rate = rate
        self.rng = random.Random(seed)

    def fires(self, kind):
        """True when this opportunity for `kind` gets the fault. Draws even when `kind` is off,
        so turning one fault off does not change where the others land."""
        draw = self.rng.random()
        return kind in self.faults and draw < self.rate

    def has(self, kind):
        return kind in self.faults


def nudge_target(scene, xy, axis, others):
    """Where to move an object at `xy` (x, y) for a nudge: the nearest of NUDGE_DISTANCES along
    +axis or -axis (a unit (x, y) vector, the TCP's x axis, across the fingers' closing
    direction) that scene.is_valid accepts given the `others` [(x, y, yaw)]. None if none is."""
    ax, ay = axis
    for d in NUDGE_DISTANCES:
        for sign in (1.0, -1.0):
            x, y = xy[0] + sign * d * ax, xy[1] + sign * d * ay
            if scene.is_valid(x, y, others):
                return (x, y)
    return None


def drop_target(scene, xy, others, rng, tries=300):
    """Where an object dropped above `xy` lands: a valid spot DROP_RADIUS from below the
    gripper, or anywhere valid in the region if there is none nearby. None if the table is
    full."""
    lo, hi = DROP_RADIUS
    for _ in range(tries):
        r, a = rng.uniform(lo, hi), rng.uniform(-math.pi, math.pi)
        x, y = xy[0] + r * math.cos(a), xy[1] + r * math.sin(a)
        if scene.is_valid(x, y, others):
            return (x, y)
    region = scene.region
    for _ in range(tries):
        x, y = rng.uniform(region.xmin, region.xmax), rng.uniform(region.ymin, region.ymax)
        if scene.is_valid(x, y, others):
            return (x, y)
    return None


def fixture_offset(rng):
    """A (dx, dy) shift of FIXTURE_SHIFT length in a random direction."""
    r, a = rng.uniform(*FIXTURE_SHIFT), rng.uniform(-math.pi, math.pi)
    return (r * math.cos(a), r * math.sin(a))
