"""
These are the tests for cell_monitor's fault decisions, against a stub scene whose only free table
is the square from 0.3 to 0.7 m with a keep-out in the middle. They check that the seeded draws
repeat, that the rate holds over many opportunities and that an off fault still draws, that a nudge
goes across the fingers to the nearest free distance and never onto an object or a keep-out, that a
dropped object lands on free table, and that the fixture shift stays within 2 to 4 mm. Run them
with colcon test --packages-select cell_monitor.
"""
import math
import random
from types import SimpleNamespace

import pytest

from cell_monitor.faults import (FIXTURE_SHIFT, NUDGE_DISTANCES, FaultPlan, drop_target,
                                 fixture_offset, nudge_target)


class StubScene:
    region = SimpleNamespace(xmin=0.3, xmax=0.7, ymin=0.3, ymax=0.7)
    min_spacing = 0.07

    def is_valid(self, x, y, placed):
        r = self.region
        if not (r.xmin <= x <= r.xmax and r.ymin <= y <= r.ymax):
            return False
        if 0.48 <= x <= 0.52 and 0.48 <= y <= 0.52:   # a keep-out
            return False
        return all(math.hypot(x - px, y - py) >= self.min_spacing for px, py, _ in placed)


def test_the_same_seed_gives_the_same_faults():
    a = FaultPlan(["nudge", "drop"], 0.3, seed=7)
    b = FaultPlan(["nudge", "drop"], 0.3, seed=7)
    assert [a.fires("nudge") for _ in range(50)] == [b.fires("nudge") for _ in range(50)]


def test_the_rate_holds_and_an_off_fault_still_draws():
    plan = FaultPlan(["nudge"], 0.25, seed=1)
    fired = sum(plan.fires("nudge") for _ in range(4000))
    assert 900 < fired < 1100
    # slip is off, so it never fires, but it uses up a draw like any other opportunity
    with_off = FaultPlan(["nudge"], 0.25, seed=3)
    without = FaultPlan(["nudge"], 0.25, seed=3)
    seq_a = []
    for _ in range(20):
        assert not with_off.fires("slip")
        seq_a.append(with_off.fires("nudge"))
    seq_b = []
    for _ in range(20):
        without.rng.random()
        seq_b.append(without.fires("nudge"))
    assert seq_a == seq_b


def test_unknown_faults_and_bad_rates_are_refused():
    with pytest.raises(ValueError):
        FaultPlan(["teleport"], 0.1, 1)
    with pytest.raises(ValueError):
        FaultPlan(["nudge"], 1.5, 1)


def test_a_nudge_goes_across_the_fingers_to_the_nearest_free_distance():
    scene = StubScene()
    to = nudge_target(scene, (0.40, 0.40), (1.0, 0.0), [])
    assert to == pytest.approx((0.40 + NUDGE_DISTANCES[0], 0.40))
    # with +x blocked by another object, it goes the other way
    to = nudge_target(scene, (0.40, 0.40), (1.0, 0.0), [(0.45, 0.40, 0.0)])
    assert to[0] < 0.40 and to[1] == pytest.approx(0.40)
    # boxed in on both sides, there is no nudge rather than one onto another object
    assert nudge_target(scene, (0.40, 0.40), (1.0, 0.0),
                        [(0.45, 0.40, 0.0), (0.35, 0.40, 0.0)]) is None
    # nor into the keep-out: from 0.44 along +x every distance lands in it or past it
    to = nudge_target(scene, (0.44, 0.50), (1.0, 0.0), [])
    assert to is None or not 0.48 <= to[0] <= 0.52


def test_a_dropped_object_lands_on_free_table():
    scene = StubScene()
    rng = random.Random(4)
    others = [(0.40, 0.40, 0.0), (0.60, 0.60, 0.0)]
    for _ in range(100):
        x, y = drop_target(scene, (0.50, 0.40), others, rng)
        assert scene.is_valid(x, y, others)


def test_the_fixture_shift_is_2_to_4_mm():
    rng = random.Random(9)
    for _ in range(100):
        d = math.hypot(*fixture_offset(rng))
        assert FIXTURE_SHIFT[0] - 1e-12 <= d <= FIXTURE_SHIFT[1] + 1e-12
