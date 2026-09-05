"""
These are the tests for the scene randomiser. They load the real objects.yaml and workspace.yaml
from cell_description and check that a seed always gives the same layout, that 200 seeds all
keep every rule (region, reach, keep-out zones and spacing), and that only boxes are turned.
Run them with colcon test --packages-select cell_benchmarks, or with pytest from the package.
"""
import math
from pathlib import Path

import pytest

from cell_benchmarks.scene import SceneConfig

CONFIG = Path(__file__).resolve().parents[2] / "cell_description" / "config"


@pytest.fixture(scope="module")
def scene():
    return SceneConfig(str(CONFIG / "objects.yaml"), str(CONFIG / "workspace.yaml"))


def test_same_seed_same_layout(scene):
    assert scene.sample(7) == scene.sample(7)
    assert scene.sample(7) != scene.sample(8)


@pytest.mark.parametrize("seed", range(1, 201))
def test_layouts_respect_every_rule(scene, seed):
    poses = scene.sample(seed)
    assert set(poses) == set(scene.objects)
    pts = list(poses.values())
    for i, (x, y, yaw) in enumerate(pts):
        assert scene.region.contains(x, y)
        assert scene.reach[0] <= math.hypot(x, y) <= scene.reach[1]
        assert not any(k.contains(x, y) for k in scene.keepouts.values())
        for (x2, y2, _) in pts[i + 1:]:
            assert math.hypot(x - x2, y - y2) >= scene.min_spacing


def test_only_boxes_get_a_random_yaw(scene):
    poses = scene.sample(3)
    for name, (_, _, yaw) in poses.items():
        if scene.objects[name].shape != "box":
            assert yaw == 0.0

