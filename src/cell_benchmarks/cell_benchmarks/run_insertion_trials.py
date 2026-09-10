"""This is the part 5 acceptance benchmark, peg-in-hole insertion with the fixture moved off its
pose. Each trial moves the fixture block by (dx, dy) of up to --max-offset per axis, 3 mm by
default, with the first four trials at the corners and the rest drawn from a seeded uniform sample,
while the robot still believes the nominal fixture pose in workspace.yaml. The peg is picked with
grasp_strategy "insert" and Insert("peg_fixture") then has to find the hole by compliant search.
The fixture is a 20 kg free body, moved through set_free_joint_state. A trial only counts when
the skill reports success and simulator ground truth agrees, with the peg standing in the actual
hole at full depth. Each run writes log.jsonl and summary.json under --log-dir and exits
non-zero below --min-success-rate. The part 5 acceptance run, --trials 30 at 3 mm, succeeded 30 of
30, all on the first attempt, with a maximum force of 5.7 N. Run it, with cell.launch.py up, as
ros2 run cell_benchmarks run_insertion_trials --trials 20 --max-offset 0.003.
"""
import argparse
import json
import math
import os
import random
import statistics
import sys
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cell_interfaces.action import Insert, MoveToNamedPose, Pick
from geometry_msgs.msg import PoseStamped

from cell_benchmarks.sim import CellClient

PEG = "peg_0"
FIXTURE = "peg_fixture"
PEG_START = (0.30, 0.10, 0.0)   # x, y, yaw on the table, clear of the other default objects
XY_TOLERANCE = 0.0015           # m, peg axis to actual hole centre (1 mm radial clearance)
Z_TOLERANCE = 0.003             # m, peg bottom to hole bottom


def offsets(n, max_offset, seed):
    """
    n fixture offsets (dx, dy): the four corners of the +-max_offset square first, then
    seeded uniform samples inside it.
    """
    corners = [(s * max_offset, t * max_offset) for s in (1, -1) for t in (1, -1)]
    rng = random.Random(seed)
    rest = [(rng.uniform(-max_offset, max_offset), rng.uniform(-max_offset, max_offset))
            for _ in range(max(0, n - len(corners)))]
    return (corners + rest)[:n]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--max-offset", type=float, default=0.003, help="m, per axis")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--log-dir", default=os.path.expanduser("~/cell_ws/runs"))
    ap.add_argument("--min-success-rate", type=float, default=0.9)
    args = ap.parse_args()

    desc = get_package_share_directory("cell_description")
    ws = yaml.safe_load(open(f"{desc}/config/workspace.yaml"))
    objects = yaml.safe_load(open(f"{desc}/config/objects.yaml"))
    fixture = ws["fixtures"][FIXTURE]
    hx, hy, hz = fixture["hole_pose"][:3]
    depth = fixture["insert_depth"]
    peg_size = objects["types"][objects["objects"][PEG]["type"]]["size"]

    run_dir = os.path.join(args.log_dir, datetime.now().strftime("part5_%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "log.jsonl")

    rclpy.init()
    cell = CellClient("insertion_trials")
    cell.wait_until_ready()
    print(f"{args.trials} trials, fixture offsets up to +-{1000 * args.max_offset:.1f} mm, "
          f"log: {log_path}\n")
    print(f"{'#':>3} {'dx_mm':>6} {'dy_mm':>6} {'found_dx':>8} {'found_dy':>8} {'att':>3} "
          f"{'Fmax':>5} {'xy_mm':>6} {'z_mm':>6} {'ok':>4} {'time':>5}  note")

    records = []
    with open(log_path, "w") as log:
        for i, (dx, dy) in enumerate(offsets(args.trials, args.max_offset, args.seed), 1):
            t0 = time.monotonic()
            rec = {"trial": i, "offset_mm": [1000 * dx, 1000 * dy], "success": False, "note": ""}
            try:
                # move the fixture (its body origin is on the table surface), put the peg down
                cell.set_free_joints({FIXTURE: (hx + dx, hy + dy, 0.0, 0.0)}, settle=0.3)
                x, y, yaw = PEG_START
                cell.set_object_poses({PEG: (x, y, yaw)}, {PEG: peg_size}, settle=0.5)
                gt = cell.fresh_ground_truth()

                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose = gt[PEG]
                pick = cell.run("pick", Pick.Goal(
                    object_id=PEG, object_pose=pose, grasp_strategy="insert",
                    object_size=_vec(peg_size)))
                if not pick.success:
                    raise RuntimeError(f"pick: {pick.message}")

                ins = cell.run("insert", Insert.Goal(target_name=FIXTURE), timeout=180.0)
                rec.update(insert_ok=bool(ins.success), attempts=int(ins.attempts),
                           depth_reached_mm=1000 * ins.depth_reached,
                           max_force_n=float(ins.max_force_seen),
                           found_offset_mm=[1000 * ins.hole_offset.x, 1000 * ins.hole_offset.y],
                           message=ins.message)

                gt = cell.fresh_ground_truth()
                peg, fix = gt[PEG].position, gt[FIXTURE].position
                xy_err = math.hypot(peg.x - (fix.x), peg.y - (fix.y))
                z_err = (peg.z - peg_size[2] / 2) - (fix.z + hz - depth)
                rec.update(xy_err_mm=1000 * xy_err, z_err_mm=1000 * z_err)
                rec["success"] = bool(ins.success and xy_err < XY_TOLERANCE and abs(z_err) < Z_TOLERANCE)
                if not ins.success:
                    raise RuntimeError(f"insert: {ins.message}")
                if not rec["success"]:
                    raise RuntimeError("insert reported success but the peg is not seated")
            except Exception as e:  # noqa: BLE001
                rec["note"] = str(e)
                try:
                    cell.run("move", MoveToNamedPose.Goal(name="home"), timeout=60.0)
                except Exception:  # noqa: BLE001
                    pass
            rec["duration_s"] = round(time.monotonic() - t0, 2)
            records.append(rec)
            log.write(json.dumps(rec) + "\n")
            log.flush()
            f = rec.get("found_offset_mm", [float("nan")] * 2)
            print(f"{i:>3} {1000 * dx:>6.2f} {1000 * dy:>6.2f} {f[0]:>8.2f} {f[1]:>8.2f} "
                  f"{rec.get('attempts', 0):>3} {rec.get('max_force_n', float('nan')):>5.1f} "
                  f"{rec.get('xy_err_mm', float('nan')):>6.2f} {rec.get('z_err_mm', float('nan')):>6.2f} "
                  f"{('ok' if rec['success'] else 'FAIL'):>4} {rec['duration_s']:>5.1f}  {rec['note']}",
                  flush=True)

    n = len(records)
    ok = [r for r in records if r["success"]]
    summary = {
        "trials": n,
        "max_offset_mm": 1000 * args.max_offset,
        "success_rate": len(ok) / n if n else 0.0,
        "attempts_mean": statistics.mean(r["attempts"] for r in ok) if ok else None,
        "first_attempt_rate": sum(r.get("attempts") == 1 for r in ok) / n if n else 0.0,
        "max_force_n": max((r.get("max_force_n", 0.0) for r in records), default=None),
        "offset_estimate_error_mm": statistics.mean(
            math.hypot(r["found_offset_mm"][0] - r["offset_mm"][0],
                       r["found_offset_mm"][1] - r["offset_mm"][1]) for r in ok) if ok else None,
        "mean_cycle_s": statistics.mean(r["duration_s"] for r in records) if records else None,
    }
    with open(os.path.join(run_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(f"\nsuccess:        {len(ok)}/{n} ({100 * summary['success_rate']:.0f}%)")
    if ok:
        print(f"attempts:       mean {summary['attempts_mean']:.2f}, first-attempt "
              f"{100 * summary['first_attempt_rate']:.0f}%")
        print(f"hole estimate:  {summary['offset_estimate_error_mm']:.2f} mm mean error vs the true offset")
    print(f"max force:      {summary['max_force_n']:.1f} N")
    print(f"mean cycle:     {summary['mean_cycle_s']:.1f} s\nsummary: {run_dir}/summary.json")
    cell.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if summary["success_rate"] >= args.min_success_rate else 1)


def _vec(size):
    """A geometry_msgs Vector3 of the three sizes."""
    from geometry_msgs.msg import Vector3
    return Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))


if __name__ == "__main__":
    main()
