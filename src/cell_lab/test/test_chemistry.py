"""
These are the tests for the lab's chemistry. They step a clock through each protocol and pin
what the benchmarks rely on: a seed always draws the same futures, an assay changes once at its
reaction time and not while it is held, a titration only reaches its endpoint at the dose its
plan says and one more dose overshoots, and the parking grid here is the one build_lab_mjcf.py
wrote into the MJCF. Run them with colcon test --packages-select cell_lab.
"""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from cell_lab.chemistry import Lab, parking_spots, twin_name
from cell_lab.layout import slot_floor

SHARE = get_package_share_directory("cell_description")


def config():
    with open(os.path.join(SHARE, "config", "lab", "lab.yaml")) as f:
        return yaml.safe_load(f)


def never_held(_name):
    return False


# - assay

def test_same_seed_same_futures():
    a, b = Lab(config()), Lab(config())
    a.reset("assay", 7, 0.0)
    b.reset("assay", 7, 0.0)
    assert [s.plan for s in a.samples.values()] == [s.plan for s in b.samples.values()]
    c = Lab(config())
    c.reset("assay", 8, 0.0)
    assert [s.plan for s in a.samples.values()] != [s.plan for s in c.samples.values()]


def test_assay_changes_once_at_its_reaction_time():
    lab = Lab(config())
    active = lab.reset("assay", 3, 10.0)
    assert set(active) == {"assay_1", "assay_2", "assay_3", "assay_4"}
    s = lab.samples["assay_1"]
    assert s.look == "vial_blue" and s.state == "start"
    due = 10.0 + s.plan["reaction_time"]
    assert "assay_1" not in [n for n, _, _ in lab.step(due - 0.01, never_held)]
    swaps = {n: (old, new) for n, old, new in lab.step(due + 0.01, never_held)}
    expected = "vial_green" if s.plan["outcome"] == "done" else "vial_brown"
    assert swaps["assay_1"] == ("vial_blue", expected)
    assert s.state == s.plan["outcome"]
    assert "assay_1" not in [n for n, _, _ in lab.step(1e6, never_held)]


def test_a_held_sample_waits_until_it_is_set_down():
    lab = Lab(config())
    lab.reset("assay", 1, 0.0)
    late = 1e3
    assert lab.step(late, lambda n: True) == []
    assert len(lab.step(late, never_held)) == 4


def test_outcomes_follow_the_failure_rate_over_many_seeds():
    failed = total = 0
    for seed in range(200):
        lab = Lab(config())
        lab.reset("assay", seed, 0.0)
        for s in lab.samples.values():
            if s.protocol == "assay":
                total += 1
                failed += s.plan["outcome"] == "failed"
    assert 0.18 < failed / total < 0.32  # failure_rate is 0.25


# - titration

def test_titration_reaches_the_endpoint_at_the_planned_dose_and_then_overshoots():
    cfg = config()
    lab = Lab(cfg)
    lab.reset("titration", 11, 0.0)
    s = lab.samples["titr_1"]
    mix = cfg["protocols"]["titration"]["mix_time"]
    t = 0.0
    for dose in range(1, s.plan["endpoint_doses"] + 1):
        lab.dispense("titr_1", t)
        t += mix + 0.1
        lab.step(t, never_held)
        if dose < s.plan["endpoint_doses"]:
            assert s.look != "vial_red"
    assert s.look == "vial_red"
    lab.dispense("titr_1", t)
    lab.step(t + mix + 0.1, never_held)
    assert s.look == "vial_purple"


def test_a_dose_shows_only_after_the_mix_time():
    cfg = config()
    lab = Lab(cfg)
    lab.reset("titration", 2, 0.0)
    s = lab.samples["titr_2"]
    first = s.plan["thresholds"][0]
    for _ in range(first):
        lab.dispense("titr_2", 0.0)
    assert lab.step(0.5, never_held) == []
    assert lab.step(cfg["protocols"]["titration"]["mix_time"] + 0.01, never_held)[0][2] == \
        "vial_orange"


def test_dosing_an_assay_sample_is_refused():
    lab = Lab(config())
    lab.reset("assay", 0, 0.0)
    with pytest.raises(ValueError):
        lab.dispense("assay_1", 0.0)


def test_unknown_protocol_is_refused():
    with pytest.raises(KeyError):
        Lab(config()).reset("distillation", 0, 0.0)


# - growth and readout

def test_growth_advances_one_stage_per_drawn_interval():
    lab = Lab(config())
    lab.reset("growth", 4, 0.0)
    s = lab.samples["plate_1"]
    assert s.look == "dish_0"
    t = 0.0
    for k, dt in enumerate(s.plan["stage_times"], start=1):
        assert "plate_1" not in [n for n, _, _ in lab.step(t + dt - 0.01, never_held)]
        t += dt
        assert ("plate_1", f"dish_{k - 1}", f"dish_{k}") in lab.step(t + 0.001, never_held)
    assert s.look == "dish_4"
    assert "plate_1" not in [n for n, _, _ in lab.step(1e6, never_held)]
    # the harvest stage is the first whose colony covers harvest_area (600 mm^2): radius 15 mm
    assert s.plan["harvest_stage"] == 3
    assert s.plan["areas_mm2"][3] >= 600 > s.plan["areas_mm2"][2]


def test_readout_draws_one_fixed_look_per_tube():
    looks = set()
    for seed in range(30):
        lab = Lab(config())
        lab.reset("readout", seed, 0.0)
        for name in ("tube_1", "tube_2", "tube_3", "tube_4"):
            s = lab.samples[name]
            looks.add(s.look)
            assert s.state == s.plan["truth"]
        assert lab.step(1e6, never_held) == []
    assert len(looks) == 5


# - geometry

def test_parking_matches_the_generated_mjcf():
    spots = parking_spots(config())
    root = ET.parse(os.path.join(SHARE, "mjcf", "lab_objects.xml")).getroot()
    bodies = {b.get("name"): tuple(float(v) for v in b.get("pos").split())
              for b in root.iter("body")}
    assert set(bodies) == set(spots)
    for name, pos in bodies.items():
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(pos, spots[name])), name
    assert twin_name("assay_1", "vial_blue") in bodies


def test_slot_floor_follows_the_workspace():
    with open(os.path.join(SHARE, "config", "lab", "workspace.yaml")) as f:
        ws = yaml.safe_load(f)
    x, y, z = slot_floor(ws, "rack_in/B3")
    assert (round(x, 4), round(y, 4), round(z, 4)) == (0.46, -0.24, 0.005)
    assert slot_floor(ws, "dispenser") == slot_floor(ws, "dispenser/A1")
    # plate_rack is turned a quarter turn, so its rows run along table y
    x, y, _ = slot_floor(ws, "plate_rack/B1")
    assert (round(x, 4), round(y, 4)) == (0.27, 0.0)
    with pytest.raises(KeyError):
        slot_floor(ws, "rack_in")
    with pytest.raises(KeyError):
        slot_floor(ws, "waste")
