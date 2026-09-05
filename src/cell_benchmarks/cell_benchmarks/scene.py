"""
This lays the cell's objects out at random for a benchmark, from a seed. The rules come from the
randomize: section of cell_description/config/objects.yaml (the region, a reach band around the
robot base and a minimum spacing between objects) and the keep-out zones from the trays, bins
and fixtures in config/workspace.yaml, so the same seed always gives the same layout and a
failing trial can be rerun exactly. It is plain Python with no ROS in it, so the layouts can be
tested without a simulator.
"""
import math
import random
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class ObjectSpec:
    name: str
    type: str
    object_class: str
    shape: str
    size: tuple  # (x, y, z) bounding box, m


@dataclass(frozen=True)
class Rect:
    """An axis-aligned rectangle in the table plane."""

    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def grown(self, margin):
        """This rectangle with every side moved out by margin."""
        return Rect(self.xmin - margin, self.xmax + margin, self.ymin - margin, self.ymax + margin)

    def contains(self, x, y):
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax


def _rotated_aabb(cx, cy, yaw, lo, hi):
    """The bounding Rect of the local box lo..hi, rotated by yaw about (cx, cy)."""
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for lx in (lo[0], hi[0]):
        for ly in (lo[1], hi[1]):
            xs.append(cx + c * lx - s * ly)
            ys.append(cy + s * lx + c * ly)
    return Rect(min(xs), max(xs), min(ys), max(ys))


def workspace_footprints(workspace):
    """
    {name: Rect} footprints of the trays, bins and fixtures in workspace.yaml. They follow the
    same definitions as cell_workspace's C++ footprints, so a keep-out zone here is the area the
    workspace model itself treats as occupied.
    """
    rects = {}
    for name, t in (workspace.get("trays") or {}).items():
        pr, pc = t["pitch"]
        rects[name] = _rotated_aabb(
            t["origin"][0], t["origin"][1], t.get("yaw", 0.0),
            (-pr / 2, -pc / 2), ((t["rows"] - 0.5) * pr, (t["cols"] - 0.5) * pc))
    for name, b in (workspace.get("bins") or {}).items():
        (cx, cy, _), (sx, sy, _) = b["center"], b["size"]
        rects[name] = Rect(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
    for name, f in (workspace.get("fixtures") or {}).items():
        x, y, _, _, _, yaw = f["hole_pose"]
        h = f.get("block_size", 0.08) / 2
        rects[name] = _rotated_aabb(x, y, yaw, (-h, -h), (h, h))
    return rects


class SceneConfig:
    """The objects, their default poses and the layout rules, read from the two YAML files."""

    def __init__(self, objects_yaml: str, workspace_yaml: str):
        with open(objects_yaml) as f:
            objects = yaml.safe_load(f)
        with open(workspace_yaml) as f:
            workspace = yaml.safe_load(f)
        types = objects["types"]
        self.objects = {
            name: ObjectSpec(name, o["type"], types[o["type"]]["class"], types[o["type"]]["shape"],
                             tuple(types[o["type"]]["size"]))
            for name, o in objects["objects"].items()
        }
        self.default_poses = {name: tuple(o["pose"]) for name, o in objects["objects"].items()}
        r = objects["randomize"]
        self.region = Rect(*r["region"]["x"], *r["region"]["y"])
        self.reach = tuple(r["reach"])
        self.min_spacing = r["min_spacing"]
        self.keepouts = {n: rect.grown(r["keepout_margin"])
                         for n, rect in workspace_footprints(workspace).items()}

    def is_valid(self, x, y, placed):
        """
        True when (x, y) is inside the region and the reach band, outside every keep-out zone,
        and at least min_spacing from every object already placed.
        """
        if not self.region.contains(x, y):
            return False
        if not self.reach[0] <= math.hypot(x, y) <= self.reach[1]:
            return False
        if any(k.contains(x, y) for k in self.keepouts.values()):
            return False
        return all(math.hypot(x - px, y - py) >= self.min_spacing for px, py, _ in placed)

    def sample(self, seed: int, max_attempts: int = 10000):
        """
        {object name: (x, y, yaw)} for this seed, always the same for the same seed. Objects
        are placed in name order by rejection sampling; only boxes get a random yaw, since
        turning a cylinder changes nothing. Raises RuntimeError when an object finds no free
        spot in max_attempts tries.
        """
        rng = random.Random(seed)
        poses = {}
        for name in sorted(self.objects):
            for _ in range(max_attempts):
                x = rng.uniform(self.region.xmin, self.region.xmax)
                y = rng.uniform(self.region.ymin, self.region.ymax)
                if self.is_valid(x, y, poses.values()):
                    yaw = rng.uniform(-math.pi, math.pi) if self.objects[name].shape == "box" else 0.0
                    poses[name] = (x, y, yaw)
                    break
            else:
                raise RuntimeError(f"seed {seed}: no free spot for {name}")
        return poses
