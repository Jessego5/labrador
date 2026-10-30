"""This is the part 6 acceptance benchmark, a whole task file run unattended through the executor.
For each seed it sends the robot home, puts the fixture back at its nominal pose and lays the
objects out, seed 0 in the default layout from objects.yaml and every other seed at random as in
the part 4 benchmark, then sends the task to /executor/execute_task and leaves the cell alone until
the task ends. The scene is then checked against simulator ground truth for sort_and_insert:
both red cubes inside bin_a, a blue cylinder seated in each of tray_1 C1 to C4, the peg fully
inserted in peg_fixture, and the green cube where it started, neither knocked over nor moved. A
run only counts when the executor reports success and the scene agrees. Each run writes
log.jsonl and summary.json under --log-dir and exits non-zero below --min-success-rate. The part 6
acceptance run over seeds 0-9 succeeded 10 of 10, with a slot error of about 1.9 mm (cylinders
resting against a pocket wall, within the 2 mm clearance), a peg error of about 0.37 mm and about
107 s per task. Run it, with cell.launch.py up, as
ros2 run cell_benchmarks run_task_trials --seeds 0-9.

With --faults it is the reliability benchmark: before each task cell_monitor is armed with the
listed faults (nudge, slip, drop, fixture_shift, camera_noise, or all) at --rate per opportunity,
seeded with the run's seed, and after it every injected fault is scored. A fault counts as
recovered when the executor logged a successful recovery in the step and object it hit, or when
nothing failed there and the run passed; a fault in a run that did not pass is never counted as
recovered unless the executor recovered from it. --bag also records each run as a rosbag.
"""
import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cell_interfaces.action import ExecuteTask, MoveToNamedPose
from cell_interfaces.srv import StartRun
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from cell_benchmarks.run_perception_trials import parse_seeds
from cell_benchmarks.scene import SceneConfig
from cell_benchmarks.sim import CellClient

SLOT_XY_TOLERANCE = 0.003   # m, cylinder axis to slot centre (34 mm pocket, 30 mm cylinder)
Z_TOLERANCE = 0.004         # m, resting on the pocket, bin or hole bottom
PEG_XY_TOLERANCE = 0.0015   # m, the same as the part 5 insertion benchmark
UNTOUCHED_TOLERANCE = 0.01  # m, how far an object the task must leave alone may move
BIN_WALL = 0.005            # m, the same wall thickness as build_workspace_mjcf.py
FAULT_KINDS = ["nudge", "slip", "drop", "fixture_shift", "camera_noise"]
RUN_LEVEL_FAULTS = ("fixture_shift", "camera_noise")  # not tied to one step or object


def score_faults(faults, events, run_ok):
    """Each fault from cell_monitor with "recovered" set, against the executor's log events."""
    scored = []
    for f in faults:
        if f["kind"] in RUN_LEVEL_FAULTS:
            recovered = run_ok
        else:
            here = [e for e in events if e.get("step") == f.get("step")
                    and (e.get("item") or "") == (f.get("item") or "")]
            failed = any(e.get("event") == "recovery" or
                         (e.get("event") == "skill" and e.get("result") == "FAILURE")
                         for e in here)
            recovered = any(e.get("event") == "recovered" for e in here) or \
                (not failed and run_ok)
        scored.append({**f, "recovered": bool(recovered)})
    return scored


def fault_cell(rec):
    """'recovered/injected' for a run's line, or '-' without fault injection."""
    if "faults" not in rec:
        return "-"
    return f"{sum(f['recovered'] for f in rec['faults'])}/{len(rec['faults'])}"


def fault_list(rec):
    """The run's faults in brief, as 'nudge+ drop- ', + recovered and - not."""
    return "".join(f"{f['kind']}{'+' if f['recovered'] else '-'} " for f in rec.get("faults", []))


def read_jsonl(path):
    try:
        with open(path) as f:
            return [json.loads(line) for line in f if line.strip()]
    except OSError:
        return []


def slot_center(tray, name):
    """
    The (x, y, z) of a slot's pocket floor centre, for a tray from workspace.yaml and a slot
    name such as "C4", by the same slot arithmetic as cell_workspace.
    """
    row = ord(name[0]) - ord("A")
    col = int(name[1:]) - 1
    lx, ly = row * tray["pitch"][0], col * tray["pitch"][1]
    yaw = tray.get("yaw", 0.0)
    ox, oy, oz = tray["origin"]
    return (ox + math.cos(yaw) * lx - math.sin(yaw) * ly,
            oy + math.sin(yaw) * lx + math.cos(yaw) * ly, oz - tray["slot_depth"])


class Checker:
    """The ground-truth checks for sort_and_insert, from workspace.yaml and objects.yaml."""

    def __init__(self, desc):
        self.ws = yaml.safe_load(open(f"{desc}/config/workspace.yaml"))
        objects = yaml.safe_load(open(f"{desc}/config/objects.yaml"))
        self.types = {n: objects["types"][o["type"]] for n, o in objects["objects"].items()}

    def names(self, **match):
        """The sorted names of the objects whose type has every given field, e.g. color="red"."""
        return sorted(n for n, t in self.types.items()
                      if all(t[k] == v for k, v in match.items()))

    def in_bin(self, pose, size, bin_name):
        """True when the object is inside the bin's walls and resting on its floor."""
        b = self.ws["bins"][bin_name]
        (cx, cy, cz), (sx, sy, _) = b["center"], b["size"]
        p = pose.position
        inside = (abs(p.x - cx) < sx / 2 - BIN_WALL and abs(p.y - cy) < sy / 2 - BIN_WALL)
        return inside and abs(p.z - (cz + size[2] / 2)) < Z_TOLERANCE

    def check(self, gt, start):
        """
        (ok, problems, details) for the scene gt, with start the ground truth from before the
        task. Each tray slot takes the nearest cylinder. The class is matched with "class" and
        not "shape" because the peg is a cylinder too.
        """
        details = {}
        problems = []

        for name in self.names(color="red"):
            ok = self.in_bin(gt[name], self.types[name]["size"], "bin_a")
            details[name] = "bin_a" if ok else "missing"
            if not ok:
                problems.append(f"{name} not in bin_a")

        tray = self.ws["trays"]["tray_1"]
        cylinders = self.names(**{"class": "cylinder"})
        slot_err = []
        for k in range(1, 5):
            sx, sy, sz = slot_center(tray, f"C{k}")
            best = None
            for name in cylinders:
                p = gt[name].position
                d = math.hypot(p.x - sx, p.y - sy)
                if best is None or d < best[0]:
                    best = (d, name, p.z - (sz + self.types[name]["size"][2] / 2))
            d, name, dz = best
            ok = d < SLOT_XY_TOLERANCE and abs(dz) < Z_TOLERANCE
            details[f"C{k}"] = {"object": name if ok else None, "xy_err_mm": round(1000 * d, 2)}
            if ok:
                slot_err.append(1000 * d)
            else:
                problems.append(f"tray_1/C{k} empty (nearest {name} {1000 * d:.1f} mm)")

        fx = self.ws["fixtures"]["peg_fixture"]
        hx, hy, hz = fx["hole_pose"][:3]
        if "peg_fixture" in gt:
            # where the fixture stands now, which a fixture_shift fault moves a few mm
            hx, hy = gt["peg_fixture"].position.x, gt["peg_fixture"].position.y
        for name in self.names(**{"class": "peg"}):
            p = gt[name].position
            d = math.hypot(p.x - hx, p.y - hy)
            dz = (p.z - self.types[name]["size"][2] / 2) - (hz - fx["insert_depth"])
            ok = d < PEG_XY_TOLERANCE and abs(dz) < Z_TOLERANCE
            details[name] = {"inserted": ok, "xy_err_mm": round(1000 * d, 2),
                             "z_err_mm": round(1000 * dz, 2)}
            if not ok:
                problems.append(f"{name} not inserted (xy {1000 * d:.1f} mm, z {1000 * dz:.1f} mm)")

        for name in self.names(color="green"):
            p, s = gt[name].position, start[name].position
            moved = math.dist((p.x, p.y, p.z), (s.x, s.y, s.z))
            details[name] = {"moved_mm": round(1000 * moved, 1)}
            if moved > UNTOUCHED_TOLERANCE:
                problems.append(f"{name} disturbed ({1000 * moved:.0f} mm)")

        details["slot_xy_err_mm"] = slot_err
        return not problems, problems, details


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", default="0-9", help="0 = default layout; e.g. 0-9 or 3,7")
    ap.add_argument("--task", default="sort_and_insert")
    ap.add_argument("--timeout", type=float, default=900.0, help="s per run")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/cell_ws/runs"))
    ap.add_argument("--min-success-rate", type=float, default=1.0)
    ap.add_argument("--faults", default="", help="comma list of " + ", ".join(FAULT_KINDS) +
                    ", or all; empty for none")
    ap.add_argument("--rate", type=float, default=0.3, help="chance per fault opportunity")
    ap.add_argument("--bag", action="store_true", help="record each run as a rosbag")
    args = ap.parse_args()
    faults = FAULT_KINDS if args.faults == "all" else [f for f in args.faults.split(",") if f]
    unknown = set(faults) - set(FAULT_KINDS)
    if unknown:
        sys.exit(f"unknown fault(s) {sorted(unknown)}; known: {', '.join(FAULT_KINDS)}")
    monitored = bool(faults) or args.bag
    if args.task != "sort_and_insert":
        sys.exit("only sort_and_insert has ground-truth checks")

    desc = get_package_share_directory("cell_description")
    scene = SceneConfig(f"{desc}/config/objects.yaml", f"{desc}/config/workspace.yaml")
    sizes = {n: o.size for n, o in scene.objects.items()}
    objects = yaml.safe_load(open(f"{desc}/config/objects.yaml"))["objects"]
    defaults = {n: tuple(o["pose"]) for n, o in objects.items()}
    checker = Checker(desc)
    hx, hy, _ = checker.ws["fixtures"]["peg_fixture"]["hole_pose"][:3]
    seeds = parse_seeds(args.seeds)

    prefix = "part8" if monitored else "part6"
    run_dir = os.path.join(args.log_dir, datetime.now().strftime(f"{prefix}_%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "log.jsonl")

    rclpy.init()
    cell = CellClient("task_trials")
    executor = ActionClient(cell, ExecuteTask, "/executor/execute_task")
    cell.wait_until_ready()
    if not executor.wait_for_server(timeout_sec=60.0):
        sys.exit("/executor/execute_task not available (is cell.launch.py running?)")
    start_run = cell.create_client(StartRun, "/monitor/start_run")
    end_run = cell.create_client(Trigger, "/monitor/end_run")
    if monitored and not start_run.wait_for_service(timeout_sec=30.0):
        sys.exit("/monitor/start_run not available (cell_monitor starts with cell.launch.py)")
    if faults:
        print(f"faults: {', '.join(faults)} at rate {args.rate}")
    print(f"{len(seeds)} run(s) of {args.task}, log: {log_path}\n")
    print(f"{'seed':>4} {'outcome':>12} {'steps':>5} {'calls':>5} {'fail':>4} {'slot_mm':>7} "
          f"{'peg_mm':>6} {'ok':>4} {'time':>6} {'faults':>6}  note")

    records = []
    with open(log_path, "w") as log:
        for seed in seeds:
            t0 = time.monotonic()
            rec = {"seed": seed, "success": False, "note": ""}
            try:
                cell.run("move", MoveToNamedPose.Goal(name="home"), timeout=60.0)
                cell.set_free_joints({"peg_fixture": (hx, hy, 0.0, 0.0)}, settle=0.2)
                cell.set_object_poses(defaults if seed == 0 else scene.sample(seed), sizes)
                start = cell.fresh_ground_truth()
                if monitored:
                    armed = cell.wait(start_run.call_async(StartRun.Request(
                        run_id=f"seed{seed}", faults=faults, rate=args.rate, seed=seed,
                        bag_dir=os.path.join(run_dir, "bags", f"seed{seed}") if args.bag
                        else "")), 10.0)
                    if not armed.success:
                        raise RuntimeError(f"monitor: {armed.message}")
                    cell.spin_for(0.5)  # the fixture shift lands before the task looks

                handle = cell.wait(executor.send_goal_async(
                    ExecuteTask.Goal(task_file=args.task)), 10.0)
                if not handle.accepted:
                    raise RuntimeError("executor rejected the task")
                result = cell.wait(handle.get_result_async(), args.timeout).result
                rec.update(outcome=result.outcome, message=result.message,
                           steps=f"{result.steps_succeeded}/{result.steps_total}",
                           skill_calls=result.skill_calls, skill_failures=result.skill_failures,
                           executor_run_dir=result.run_dir, task_duration_s=result.duration_s)

                ok, problems, details = checker.check(cell.fresh_ground_truth(), start)
                rec.update(scene_ok=ok, problems=problems, details=details)
                rec["success"] = bool(result.success and ok)
                executor_summary = {}
                if result.run_dir:
                    try:
                        with open(os.path.join(result.run_dir, "summary.json")) as f:
                            executor_summary = json.load(f)
                    except (OSError, json.JSONDecodeError):
                        pass
                rec["recoveries"] = executor_summary.get("recoveries", 0)
                rec["recovered"] = executor_summary.get("recovered", 0)
                if monitored:
                    ended = cell.wait(end_run.call_async(Trigger.Request()), 20.0)
                    injected = json.loads(ended.message).get("faults", [])
                    events = read_jsonl(os.path.join(result.run_dir, "log.jsonl"))
                    rec["faults"] = score_faults(injected, events, rec["success"])
                if not result.success:
                    rec["note"] = result.message
                elif not ok:
                    rec["note"] = "; ".join(problems)
            except Exception as e:  # noqa: BLE001 - log and continue with the next seed
                rec["note"] = str(e)
                if monitored and end_run.service_is_ready():
                    end_run.call_async(Trigger.Request())  # disarm before the next run
            rec["duration_s"] = round(time.monotonic() - t0, 1)
            records.append(rec)
            log.write(json.dumps(rec) + "\n")
            log.flush()
            d = rec.get("details", {})
            slot = d.get("slot_xy_err_mm") or []
            peg = d.get("peg_0", {}).get("xy_err_mm", float("nan"))
            print(f"{seed:>4} {rec.get('outcome', '-'):>12} {rec.get('steps', '-'):>5} "
                  f"{rec.get('skill_calls', 0):>5} {rec.get('skill_failures', 0):>4} "
                  f"{(statistics.mean(slot) if slot else float('nan')):>7.2f} {peg:>6.2f} "
                  f"{('ok' if rec['success'] else 'FAIL'):>4} {rec['duration_s']:>6.1f} "
                  f"{fault_cell(rec):>6}  {fault_list(rec)}{rec['note']}", flush=True)

    n = len(records)
    ok = sum(r["success"] for r in records)
    slot_err = [e for r in records for e in r.get("details", {}).get("slot_xy_err_mm", [])]
    peg_err = [r["details"]["peg_0"]["xy_err_mm"] for r in records
               if r.get("details", {}).get("peg_0", {}).get("inserted")]
    durations = [r["task_duration_s"] for r in records if r["success"]]
    summary = {
        "task": args.task, "runs": n, "task_success_rate": ok / n if n else 0.0,
        "skill_calls": sum(r.get("skill_calls", 0) for r in records),
        "skill_failures": sum(r.get("skill_failures", 0) for r in records),
        "slot_xy_error_mm": {"mean": statistics.mean(slot_err), "max": max(slot_err)}
        if slot_err else None,
        "peg_xy_error_mm": {"mean": statistics.mean(peg_err), "max": max(peg_err)}
        if peg_err else None,
        "mean_task_duration_s": statistics.mean(durations) if durations else None,
        "recoveries": sum(r.get("recoveries", 0) for r in records),
        "recovered": sum(r.get("recovered", 0) for r in records),
    }
    if monitored:
        scored = [f for r in records for f in r.get("faults", [])]
        by_kind = {}
        for f in scored:
            k = by_kind.setdefault(f["kind"], {"injected": 0, "recovered": 0})
            k["injected"] += 1
            k["recovered"] += f["recovered"]
        summary.update(faults=faults, rate=args.rate, faults_injected=len(scored),
                       faults_recovered=sum(f["recovered"] for f in scored),
                       fault_recovery_rate=sum(f["recovered"] for f in scored) / len(scored)
                       if scored else None, faults_by_kind=by_kind)
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\ntask success:   {ok}/{n} ({100 * summary['task_success_rate']:.0f}%) "
          f"[executor succeeded and ground truth matches]")
    print(f"skill calls:    {summary['skill_calls']} ({summary['skill_failures']} failed)")
    if slot_err:
        print(f"slot error:     mean {summary['slot_xy_error_mm']['mean']:.2f} mm, "
              f"max {summary['slot_xy_error_mm']['max']:.2f} mm")
    if peg_err:
        print(f"peg error:      mean {summary['peg_xy_error_mm']['mean']:.2f} mm")
    if durations:
        print(f"task duration:  mean {summary['mean_task_duration_s']:.1f} s")
    if summary["recoveries"]:
        print(f"recoveries:     {summary['recovered']} of {summary['recoveries']} skill failures "
              f"recovered")
    if monitored and summary["faults_injected"]:
        kinds = ", ".join(f"{k} {v['recovered']}/{v['injected']}"
                          for k, v in sorted(summary["faults_by_kind"].items()))
        print(f"faults:         {summary['faults_recovered']}/{summary['faults_injected']} "
              f"recovered ({100 * summary['fault_recovery_rate']:.0f}%): {kinds}")
    print(f"summary: {run_dir}/summary.json")

    cell.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if summary["task_success_rate"] >= args.min_success_rate else 1)


if __name__ == "__main__":
    main()
