"""This is the lab benchmark, the lab protocols run unattended and scored against what happened.
For each seed it sends the robot home, has cell_lab set the protocol's samples out with that
seed (/lab/reset), runs the protocol's task file through /executor/execute_task and leaves the
cell alone until the task ends. Then it scores every sample against the truth cell_lab kept,
which the robot never saw, and where the simulator says the sample stands.

The assay is right for a vial when it ends in rack_done after turning green or in the waste bin
after turning brown; it also measures how long each vial waited between changing and being
picked up, and counts a vial picked before it changed as a false trigger. The titration is right
for a vial when it stopped at exactly the dose that turned it red and the vial ended in rack_done;
a vial dosed past red is an overshoot. The growth experiment is right for a dish when it was
harvested into plate_rack with its colony at or past the harvest area, never earlier; it also
measures how long each dish waited once ready. The readout is right for a tube when a positive
ended in rack_done row A, a negative in row B and a borderline one in retest; a borderline tube
called either way is a guess, the one mistake the protocol must not make. Each run writes
log.jsonl and summary.json under
--log-dir, and the exit status is non-zero below --min-success-rate. Run it, with
cell.launch.py scene:=lab up, as ros2 run cell_benchmarks run_lab_trials --protocol assay
--seeds 1-5 (or titration, growth, readout, or all four with --protocol all).
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
from cell_interfaces.srv import LabReset
from rclpy.action import ActionClient
from std_msgs.msg import String

from cell_benchmarks.run_perception_trials import parse_seeds
from cell_benchmarks.sim import CellClient

TASKS = {"assay": "color_assay", "titration": "titration", "growth": "growth",
         "readout": "readout"}
SLOT_TOLERANCE = 0.008   # m; a vial counts as in a rack slot within this of its centre
BIN_WALL = 0.005         # m; matches build_workspace_mjcf.py


def slots(tray):
    """[(row letter, x, y)] of every slot centre of a tray from workspace.yaml."""
    ox, oy, _ = tray["origin"]
    yaw = tray.get("yaw", 0.0)
    out = []
    for r in range(tray["rows"]):
        for c in range(tray["cols"]):
            lx, ly = r * tray["pitch"][0], c * tray["pitch"][1]
            out.append((chr(ord("A") + r), ox + math.cos(yaw) * lx - math.sin(yaw) * ly,
                        oy + math.sin(yaw) * lx + math.cos(yaw) * ly))
    return out


def where(pos, ws, tolerance=SLOT_TOLERANCE):
    """Where a sample at pos stands: a tray name, "rack_done/A" style for rack_done's rows, a bin
    name, or "elsewhere"."""
    x, y = pos.x, pos.y
    for name in ("rack_done", "dispenser", "rack_in", "plate_rack"):
        if name not in ws["trays"]:
            continue
        tol = 0.02 if name == "plate_rack" else tolerance  # a dish lies loose in a wide nest
        for row, sx, sy in slots(ws["trays"][name]):
            if math.hypot(x - sx, y - sy) < tol:
                return f"{name}/{row}" if name == "rack_done" else name
    for name, b in ws["bins"].items():
        (cx, cy, _), (sx, sy, _) = b["center"], b["size"]
        if abs(x - cx) < sx / 2 - BIN_WALL and abs(y - cy) < sy / 2 - BIN_WALL:
            return name
    return "elsewhere"


def score_assay(state, gt, ws):
    samples, delays, false_triggers = {}, [], 0
    for name, s in state["samples"].items():
        events = s["events"]
        changed = next((e["t"] for e in events if e["kind"] == "changed"), None)
        lifts = [e["t"] for e in events if e["kind"] == "lifted"]
        early = [t for t in lifts if changed is None or t < changed]
        false_triggers += len(early)
        picked = next((t for t in lifts if changed is not None and t >= changed), None)
        place = where(gt[f"{name}__{s['look']}"].position, ws)
        want = "rack_done" if s["plan"]["outcome"] == "done" else "waste"
        place = "rack_done" if place.startswith("rack_done") else place
        if picked is not None:
            delays.append(picked - changed)
        samples[name] = {"outcome": s["plan"]["outcome"], "ended_in": place,
                         "correct": place == want and not early,
                         "reaction_time_s": s["plan"]["reaction_time"],
                         "pickup_delay_s": round(picked - changed, 2) if picked else None}
    return samples, {"pickup_delays_s": delays, "false_triggers": false_triggers}


def score_titration(state, gt, ws):
    samples, overshoots = {}, 0
    for name, s in state["samples"].items():
        endpoint = s["plan"]["endpoint_doses"]
        place = where(gt[f"{name}__{s['look']}"].position, ws)
        overshoots += s["doses"] > endpoint
        place = "rack_done" if place.startswith("rack_done") else place
        samples[name] = {"doses": s["doses"], "endpoint_doses": endpoint, "look": s["look"],
                         "ended_in": place,
                         "correct": s["doses"] == endpoint and place == "rack_done"
                         and s["look"] == "vial_red"}
    return samples, {"overshoots": overshoots}


def score_growth(state, gt, ws):
    samples, delays, early = {}, [], 0
    for name, s in state["samples"].items():
        plan, events = s["plan"], s["events"]
        ready_look = f"dish_{plan['harvest_stage']}"
        ready_t = next((e["t"] for e in events if e["kind"] == "changed"
                        and e["to"] == ready_look), None)
        lift = next((e for e in events if e["kind"] == "lifted"), None)
        stage_at_lift = int(lift["look"].split("_")[-1]) if lift else None
        too_early = lift is not None and stage_at_lift < plan["harvest_stage"]
        early += too_early
        place = where(gt[f"{name}__{s['look']}"].position, ws)
        if lift and ready_t is not None and not too_early:
            delays.append(lift["t"] - ready_t)
        samples[name] = {"stage_at_harvest": stage_at_lift, "harvest_stage": plan["harvest_stage"],
                         "area_at_harvest_mm2": plan["areas_mm2"][stage_at_lift]
                         if lift else None, "ended_in": place,
                         "correct": place == "plate_rack" and lift is not None and not too_early}
    return samples, {"pickup_delays_s": delays, "early_harvests": early}


def score_readout(state, gt, ws):
    want = {"positive": "rack_done/A", "negative": "rack_done/B", "borderline": "retest"}
    samples, guesses = {}, 0
    for name, s in state["samples"].items():
        truth = s["plan"]["truth"]
        place = where(gt[f"{name}__{s['look']}"].position, ws)
        guess = truth == "borderline" and place in ("rack_done/A", "rack_done/B")
        guesses += guess
        samples[name] = {"look": s["look"], "truth": truth, "ended_in": place,
                         "correct": place == want[truth]}
    return samples, {"borderline_guesses": guesses}


class LabClient(CellClient):
    """CellClient plus the lab's reset service, its state topic and the executor's action."""

    def __init__(self):
        super().__init__("lab_trials")
        self.reset_client = self.create_client(LabReset, "/lab/reset")
        self.task_client = ActionClient(self, ExecuteTask, "/executor/execute_task")
        self.lab_state = None
        self.create_subscription(String, "/lab/state", self._on_state, 10)

    def _on_state(self, msg):
        self.lab_state = json.loads(msg.data)

    def fresh_lab_state(self, timeout=5.0):
        self.lab_state = None
        end = time.monotonic() + timeout
        while self.lab_state is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.lab_state is None:
            raise RuntimeError("no /lab/state (is the cell running with scene:=lab?)")
        return self.lab_state


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--protocol", default="all", choices=[*TASKS, "all"])
    ap.add_argument("--seeds", default="1-5", help="e.g. 1-5 or 3,7")
    ap.add_argument("--timeout", type=float, default=900.0, help="s per run")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/cell_ws/runs"))
    ap.add_argument("--min-success-rate", type=float, default=0.9)
    args = ap.parse_args()

    desc = get_package_share_directory("cell_description")
    with open(f"{desc}/config/lab/workspace.yaml") as f:
        ws = yaml.safe_load(f)
    protocols = list(TASKS) if args.protocol == "all" else [args.protocol]
    seeds = parse_seeds(args.seeds)

    run_dir = os.path.join(args.log_dir, datetime.now().strftime("part7_%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "log.jsonl")

    rclpy.init()
    cell = LabClient()
    cell.wait_until_ready()
    if not cell.reset_client.wait_for_service(timeout_sec=30.0) or \
            not cell.task_client.wait_for_server(timeout_sec=30.0):
        sys.exit("/lab/reset or /executor/execute_task is missing (launch with scene:=lab)")
    print(f"{len(protocols) * len(seeds)} run(s), log: {log_path}\n")
    print(f"{'protocol':>9} {'seed':>4} {'outcome':>10} {'right':>6} {'time':>6}  per sample")

    records = []
    with open(log_path, "w") as log:
        for protocol in protocols:
            for seed in seeds:
                t0 = time.monotonic()
                rec = {"protocol": protocol, "seed": seed, "success": False, "note": ""}
                try:
                    cell.run("move", MoveToNamedPose.Goal(name="home"), timeout=60.0)
                    reset = cell.wait(cell.reset_client.call_async(
                        LabReset.Request(protocol=protocol, seed=seed)), 10.0)
                    if not reset.success:
                        raise RuntimeError(f"reset: {reset.message}")
                    cell.spin_for(1.0)
                    handle = cell.wait(cell.task_client.send_goal_async(
                        ExecuteTask.Goal(task_file=TASKS[protocol])), 10.0)
                    if not handle.accepted:
                        raise RuntimeError("executor rejected the task")
                    result = cell.wait(handle.get_result_async(), args.timeout).result
                    rec.update(outcome=result.outcome, message=result.message,
                               executor_run_dir=result.run_dir,
                               task_duration_s=round(result.duration_s, 1))
                    state = cell.fresh_lab_state()
                    gt = cell.fresh_ground_truth()
                    scorer = {"assay": score_assay, "titration": score_titration,
                              "growth": score_growth, "readout": score_readout}[protocol]
                    samples, extra = scorer(state, gt, ws)
                    rec.update(samples=samples, **extra)
                    right = sum(s["correct"] for s in samples.values())
                    rec["correct"], rec["total"] = right, len(samples)
                    rec["success"] = bool(result.success and right == len(samples))
                    if not result.success:
                        rec["note"] = result.message
                except Exception as e:  # noqa: BLE001 - log the failure and carry on
                    rec["note"] = str(e)
                rec["duration_s"] = round(time.monotonic() - t0, 1)
                records.append(rec)
                log.write(json.dumps(rec) + "\n")
                log.flush()
                parts = []
                for n, s in rec.get("samples", {}).items():
                    doses = f", {s['doses']}/{s['endpoint_doses']}" if "doses" in s else ""
                    parts.append(f"{n}:{'ok' if s['correct'] else 'X'}({s['ended_in']}{doses})")
                per = " ".join(parts)
                print(f"{protocol:>9} {seed:>4} {rec.get('outcome', '-'):>10} "
                      f"{rec.get('correct', 0)}/{rec.get('total', 0):<4} "
                      f"{rec['duration_s']:>6.1f}  "
                      f"{per} {rec['note']}", flush=True)

    summary = {"runs": len(records)}
    for protocol in protocols:
        rs = [r for r in records if r["protocol"] == protocol]
        decisions = sum(r.get("total", 0) for r in rs)
        right = sum(r.get("correct", 0) for r in rs)
        entry = {"runs": len(rs), "run_success_rate": sum(r["success"] for r in rs) / len(rs),
                 "decisions": decisions, "correct": right,
                 "decision_accuracy": right / decisions if decisions else 0.0,
                 "mean_task_duration_s": statistics.mean(
                     r["task_duration_s"] for r in rs if "task_duration_s" in r) if rs else None}
        if protocol in ("assay", "growth"):
            delays = [d for r in rs for d in r.get("pickup_delays_s", [])]
            entry["pickup_delay_s"] = {"mean": statistics.mean(delays), "max": max(delays)} \
                if delays else None
        if protocol == "assay":
            entry["false_triggers"] = sum(r.get("false_triggers", 0) for r in rs)
        elif protocol == "titration":
            entry["overshoots"] = sum(r.get("overshoots", 0) for r in rs)
        elif protocol == "growth":
            entry["early_harvests"] = sum(r.get("early_harvests", 0) for r in rs)
        else:
            entry["borderline_guesses"] = sum(r.get("borderline_guesses", 0) for r in rs)
        summary[protocol] = entry
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print()
    worst = 1.0
    for protocol in protocols:
        e = summary[protocol]
        worst = min(worst, e["decision_accuracy"])
        line = (f"{protocol:>9}: {e['correct']}/{e['decisions']} decisions right "
                f"({100 * e['decision_accuracy']:.0f}%), runs {100 * e['run_success_rate']:.0f}% "
                f"clean, mean task {e['mean_task_duration_s']:.0f} s")
        if protocol in ("assay", "growth") and e["pickup_delay_s"]:
            line += (f", pickup delay mean {e['pickup_delay_s']['mean']:.1f} s "
                     f"(max {e['pickup_delay_s']['max']:.1f})")
        for key in ("false_triggers", "overshoots", "early_harvests", "borderline_guesses"):
            if key in e:
                line += f", {key.replace('_', ' ')} {e[key]}"
        print(line)
    print(f"summary: {run_dir}/summary.json")
    cell.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if worst >= args.min_success_rate else 1)


if __name__ == "__main__":
    main()
