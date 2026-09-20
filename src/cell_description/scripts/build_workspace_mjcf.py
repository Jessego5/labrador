#!/usr/bin/env python3
"""
This builds mjcf/workspace.xml, the MuJoCo geometry of the trays, bins and fixtures, from
config/workspace.yaml, the same file cell_workspace reads, so that what the robot is told and
what MuJoCo simulates come from one place. Its geometry follows the C++ model's conventions. A
tray's frame is at slot A1's centre on the top surface, with rows along the tray's +x and
columns along its +y, and the tray body sits on the table, `origin z` tall, with square pockets
`slot_depth` deep. A bin's `center` is the floor's top surface and its `size` the outer x, y and
wall height. A fixture's `hole_pose` is the centre of the hole opening; the hole is square with
a side equal to the diameter, and the block is a free body, so it can be displaced in
simulation. It also writes a site for every workspace frame, directly under <worldbody> in
world coordinates and named like its TF frame (tray_1_C4), and cell_workspace's
test_mjcf_alignment compares those with the C++ model, so the two implementations check each
other. The sites are in group 3, hidden by default; press 3 in the MuJoCo viewer to show them.
Run it with python3 build_workspace_mjcf.py (it needs PyYAML).
"""
import math
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / "config" / "workspace.yaml"
OUT = PKG / "mjcf" / "workspace.xml"
BIN_WALL = 0.005  # m, the same as Bin::wall in cell_workspace
FIXTURE_MASS = 20.0  # kg, heavy enough not to slide under insertion forces


def fmt(*values):
    return " ".join(f"{v:.6f}".rstrip("0").rstrip(".") if v != 0 else "0" for v in values)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def rpy_quat(roll, pitch, yaw):
    """The quaternion for fixed-axis RPY, as in URDF: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    qx = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    qy = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    return quat_mul(quat_mul(yaw_quat(yaw), qy), qx)


def box(name, center, half, material):
    return (f'      <geom name="{name}" type="box" pos="{fmt(*center)}" size="{fmt(*half)}" '
            f'material="{material}"/>')


def site(name, pos, quat):
    return (f'    <site name="{name}" pos="{fmt(*pos)}" quat="{fmt(*quat)}" size="0.004" '
            f'rgba="1 0.5 0 1" group="3"/>')


def slot_name(row, col):
    return f"{chr(ord('A') + row)}{col + 1}"


def tray(name, t):
    ox, oy, oz = t["origin"]
    yaw = t.get("yaw", 0.0)
    rows, cols = t["rows"], t["cols"]
    pr, pc = t["pitch"]
    size, depth = t["slot_size"], t["slot_depth"]

    x0, x1 = -pr / 2, (rows - 0.5) * pr
    y0, y1 = -pc / 2, (cols - 0.5) * pc
    xm, ym, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    floor = oz - depth
    wall_r, wall_c = pr - size, pc - size  # wall thickness between rows / between columns
    zc, hz = floor + depth / 2, depth / 2

    geoms = [box(f"{name}_base", (xm, ym, floor / 2), (hx, hy, floor / 2), "ws_tray")]
    # The inner walls are centred on cell boundaries; the outer walls are half as thick and sit
    # just inside the footprint, so every pocket is exactly slot_size wide.
    for i in range(rows + 1):
        xb = x0 + i * pr
        if i in (0, rows):
            xc, half = xb + (wall_r / 4 if i == 0 else -wall_r / 4), wall_r / 4
        else:
            xc, half = xb, wall_r / 2
        geoms.append(box(f"{name}_wall_row{i}", (xc, ym, zc), (half, hy, hz), "ws_tray"))
    for j in range(cols + 1):
        yb = y0 + j * pc
        if j in (0, cols):
            yc, half = yb + (wall_c / 4 if j == 0 else -wall_c / 4), wall_c / 4
        else:
            yc, half = yb, wall_c / 2
        geoms.append(box(f"{name}_wall_col{j}", (xm, yc, zc), (hx, half, hz), "ws_tray"))

    body = [f'    <body name="{name}" pos="{fmt(ox, oy, 0)}" quat="{fmt(*yaw_quat(yaw))}">',
            *geoms, "    </body>"]

    c, s = math.cos(yaw), math.sin(yaw)
    q = yaw_quat(yaw)
    sites = [site(name, (ox, oy, oz), q)]
    for r in range(rows):
        for k in range(cols):
            lx, ly = r * pr, k * pc
            sites.append(site(f"{name}_{slot_name(r, k)}",
                              (ox + c * lx - s * ly, oy + s * lx + c * ly, oz), q))
    return body, sites


def bin_(name, b):
    cx, cy, cz = b["center"]
    sx, sy, sz = b["size"]
    w = BIN_WALL
    geoms = [
        box(f"{name}_floor", (0, 0, cz / 2), (sx / 2, sy / 2, cz / 2), "ws_bin"),
        box(f"{name}_wall_xn", (-(sx - w) / 2, 0, sz / 2), (w / 2, sy / 2, sz / 2), "ws_bin"),
        box(f"{name}_wall_xp", ((sx - w) / 2, 0, sz / 2), (w / 2, sy / 2, sz / 2), "ws_bin"),
        box(f"{name}_wall_yn", (0, -(sy - w) / 2, sz / 2), (sx / 2, w / 2, sz / 2), "ws_bin"),
        box(f"{name}_wall_yp", (0, (sy - w) / 2, sz / 2), (sx / 2, w / 2, sz / 2), "ws_bin"),
    ]
    body = [f'    <body name="{name}" pos="{fmt(cx, cy, 0)}">', *geoms, "    </body>"]
    return body, [site(name, (cx, cy, cz), (1, 0, 0, 0))]


def fixture(name, f):
    x, y, z, roll, pitch, yaw = f["hole_pose"]
    d, depth = f["hole_diameter"], f["insert_depth"]
    half_block = f.get("block_size", 0.08) / 2
    base_top = z - depth
    zc, hz = z - depth / 2, depth / 2
    side = (half_block + d / 2) / 2   # centre of the walls beside the hole
    wall = (half_block - d / 2) / 2   # half thickness of those walls
    geoms = [
        box(f"{name}_base", (0, 0, base_top / 2), (half_block, half_block, base_top / 2), "ws_fixture"),
        box(f"{name}_wall_xn", (-side, 0, zc), (wall, half_block, hz), "ws_fixture"),
        box(f"{name}_wall_xp", (side, 0, zc), (wall, half_block, hz), "ws_fixture"),
        box(f"{name}_wall_yn", (0, -side, zc), (d / 2, wall, hz), "ws_fixture"),
        box(f"{name}_wall_yp", (0, side, zc), (d / 2, wall, hz), "ws_fixture"),
    ]
    # The block stands on the table, so only the yaw of hole_pose applies to the geometry. It
    # is a free body, heavy enough not to slide under insertion forces, so the benchmarks can
    # displace it from its nominal pose with set_free_joint_state while the robot keeps
    # planning with the nominal pose from workspace.yaml; that is how the insertion search is
    # tested against a fixture that is not where it should be.
    m, s, h = FIXTURE_MASS, 2 * half_block, z
    ixx, izz = m * (s * s + h * h) / 12, m * (2 * s * s) / 12
    body = [f'    <body name="{name}" pos="{fmt(x, y, 0)}" quat="{fmt(*yaw_quat(yaw))}">',
            f'      <freejoint name="{name}"/>',
            f'      <inertial pos="0 0 {fmt(h / 2)}" mass="{fmt(m)}" diaginertia="{fmt(ixx, ixx, izz)}"/>',
            *geoms, "    </body>"]
    return body, [site(name, (x, y, z), rpy_quat(roll, pitch, yaw))]


def main():
    cfg = yaml.safe_load(CONFIG.read_text())
    bodies, sites = [], []
    for section, build in (("trays", tray), ("bins", bin_), ("fixtures", fixture)):
        for name, params in (cfg.get(section) or {}).items():
            b, s = build(name, params)
            bodies += b
            sites += s

    xml = "\n".join([
        "<!-- GENERATED by cell_description/scripts/build_workspace_mjcf.py from "
        "config/workspace.yaml. Do not edit by hand. -->",
        '<mujoco model="workspace">',
        "  <asset>",
        '    <material name="ws_tray" rgba="0.35 0.55 0.75 1"/>',
        '    <material name="ws_bin" rgba="0.3 0.3 0.32 1"/>',
        '    <material name="ws_fixture" rgba="0.6 0.6 0.62 1" specular="0.5"/>',
        "  </asset>",
        "  <worldbody>",
        *bodies,
        "    <!-- Workspace frames (one per TF frame), world coordinates. -->",
        *sites,
        "  </worldbody>",
        "</mujoco>",
        "",
    ])
    OUT.write_text(xml)
    print(f"wrote {OUT}: {len(sites)} frames")


if __name__ == "__main__":
    main()
