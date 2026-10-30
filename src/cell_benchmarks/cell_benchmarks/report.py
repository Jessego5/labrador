"""
This turns two reliability benchmark runs, one without faults and one with them (run_task_trials
with and without --faults), into the report the README quotes: report.md with the summary table,
and four plots beside it, task success in both conditions, fault recovery by kind, the slot
placement error and the task duration. Run it as
ros2 run cell_benchmarks report --baseline runs/part6_... --faults runs/part8_...
--out docs/reliability

The plots are drawn for a light page with the cell's chart palette: blue for the runs without
faults and orange for the runs with them (both pass the colour-blindness checks against the
surface), thin marks, a hairline grid, and every number that matters written on the chart, so
no reading depends on colour alone.
"""
import argparse
import json
import os
import random
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402 - after choosing the backend

SURFACE, INK, INK2, MUTED, GRID, AXIS = ("#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9",
                                         "#c3c2b7")
BLUE, ORANGE = "#2a78d6", "#eb6834"   # categorical slots 1 and 2
KIND_ORDER = ["nudge", "slip", "drop", "fixture_shift", "camera_noise"]


def load(run_dir):
    with open(os.path.join(run_dir, "summary.json")) as f:
        summary = json.load(f)
    with open(os.path.join(run_dir, "log.jsonl")) as f:
        records = [json.loads(line) for line in f if line.strip()]
    return summary, records


def style(ax, grid_axis="y"):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelcolor=INK2, labelsize=9)
    ax.grid(axis=grid_axis, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def figure(width=6.4, height=3.4):
    fig, ax = plt.subplots(figsize=(width, height), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    return fig, ax


def title(ax, text, sub=None):
    ax.set_title(text, loc="left", color=INK, fontsize=11, fontweight="bold", pad=18 if sub else 8)
    if sub:
        ax.text(0, 1.02, sub, transform=ax.transAxes, color=INK2, fontsize=9, va="bottom")


def plot_success(base, faulty, out):
    fig, ax = figure(5.2, 3.2)
    names = ["without faults", "with faults"]
    rates = [100 * base["task_success_rate"], 100 * faulty["task_success_rate"]]
    counts = [(round(base["task_success_rate"] * base["runs"]), base["runs"]),
              (round(faulty["task_success_rate"] * faulty["runs"]), faulty["runs"])]
    bars = ax.bar(names, rates, color=[BLUE, ORANGE], width=0.5, edgecolor=SURFACE, linewidth=2)
    for b, (ok, n), r in zip(bars, counts, rates):
        ax.text(b.get_x() + b.get_width() / 2, r + 2, f"{r:.0f}%  ({ok}/{n})", ha="center",
                color=INK, fontsize=9)
    ax.set_ylim(0, 110)
    ax.set_ylabel("runs that passed (%)", color=INK2, fontsize=9)
    title(ax, "Task success", "sort_and_insert, executor success and ground truth both required")
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def plot_recovery(faulty, out):
    kinds = [k for k in KIND_ORDER if k in faulty["faults_by_kind"]]
    data = [faulty["faults_by_kind"][k] for k in kinds]
    fig, ax = figure(6.4, 0.5 + 0.55 * len(kinds))
    style(ax, "x")
    rates = [100 * d["recovered"] / d["injected"] for d in data]
    ys = range(len(kinds))[::-1]
    ax.barh(list(ys), rates, color=ORANGE, height=0.55, edgecolor=SURFACE, linewidth=2)
    for y, r, d in zip(ys, rates, data):
        ax.text(r + 1.5, y, f"{r:.0f}%  ({d['recovered']}/{d['injected']})", va="center",
                color=INK, fontsize=9)
    ax.set_yticks(list(ys), [k.replace("_", " ") for k in kinds])
    ax.set_xlim(0, 118)
    ax.set_xlabel("faults recovered (%)", color=INK2, fontsize=9)
    total = faulty["faults_recovered"], faulty["faults_injected"]
    title(ax, "Fault recovery by kind",
          f"{total[0]} of {total[1]} injected faults recovered "
          f"({100 * faulty['fault_recovery_rate']:.0f}%)")
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def slot_errors(records):
    return [e for r in records for e in r.get("details", {}).get("slot_xy_err_mm", [])]


def plot_placement(base_records, faulty_records, out):
    fig, ax = figure()
    a, b = slot_errors(base_records), slot_errors(faulty_records)
    top = max(a + b + [3.0])
    bins = [i * 0.25 for i in range(int(top / 0.25) + 2)]
    for data, color, name in ((a, BLUE, "without faults"), (b, ORANGE, "with faults")):
        if data:
            ax.hist(data, bins=bins, histtype="step", linewidth=2, color=color,
                    label=f"{name}  (mean {statistics.mean(data):.2f} mm, n = {len(data)})")
    ax.axvline(2.0, color=MUTED, linewidth=1, linestyle=(0, (3, 3)))
    ax.text(2.02, ax.get_ylim()[1] * 0.95, "2 mm pocket clearance", color=INK2, fontsize=8,
            va="top")
    ax.set_xlabel("cylinder axis to slot centre (mm)", color=INK2, fontsize=9)
    ax.set_ylabel("cylinders", color=INK2, fontsize=9)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="upper left")
    title(ax, "Placement error", "tray_1 C1 to C4, from simulator ground truth")
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def plot_duration(base_records, faulty_records, out):
    fig, ax = figure(6.4, 3.0)
    style(ax, "x")
    rng = random.Random(0)
    rows = (("with faults", faulty_records, ORANGE, 0), ("without faults", base_records, BLUE, 1))
    for name, records, color, y in rows:
        ok = [r["task_duration_s"] for r in records if r.get("success")]
        if not ok:
            continue
        ys = [y + rng.uniform(-0.18, 0.18) for _ in ok]
        ax.scatter(ok, ys, s=18, color=color, edgecolors=SURFACE, linewidths=1, zorder=3)
        med = statistics.median(ok)
        ax.plot([med, med], [y - 0.3, y + 0.3], color=INK, linewidth=2, zorder=4)
        ax.text(med, y + 0.36, f"median {med:.0f} s", ha="center", color=INK, fontsize=8)
    ax.set_yticks([0, 1], ["with faults", "without faults"])
    ax.set_ylim(-0.6, 1.75)
    ax.set_xlabel("task duration of runs that passed (s)", color=INK2, fontsize=9)
    title(ax, "Cycle time", "each dot one run; recoveries add time")
    fig.tight_layout()
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def table(base, faulty):
    def pct(x):
        return f"{100 * x:.0f}%" if x is not None else "n/a"

    def err(s, key):
        return f"{s[key]['mean']:.2f} mm" if s.get(key) else "n/a"

    rows = [
        ("Runs", base["runs"], faulty["runs"]),
        ("Task success", pct(base["task_success_rate"]), pct(faulty["task_success_rate"])),
        ("Faults injected", "0", faulty.get("faults_injected", 0)),
        ("Faults recovered", "n/a", pct(faulty.get("fault_recovery_rate"))),
        ("Skill failures recovered", f"{base.get('recovered', 0)}/{base.get('recoveries', 0)}",
         f"{faulty.get('recovered', 0)}/{faulty.get('recoveries', 0)}"),
        ("Slot placement error (mean)", err(base, "slot_xy_error_mm"),
         err(faulty, "slot_xy_error_mm")),
        ("Peg error (mean)", err(base, "peg_xy_error_mm"), err(faulty, "peg_xy_error_mm")),
        ("Task duration (mean, passed runs)",
         f"{base['mean_task_duration_s']:.0f} s" if base["mean_task_duration_s"] else "n/a",
         f"{faulty['mean_task_duration_s']:.0f} s" if faulty["mean_task_duration_s"] else "n/a"),
    ]
    lines = ["| | Without faults | With faults |", "|---|---|---|"]
    lines += [f"| {a} | {b} | {c} |" for a, b, c in rows]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--baseline", required=True, help="run directory without faults")
    ap.add_argument("--faults", required=True, help="run directory with faults")
    ap.add_argument("--out", required=True, help="where report.md and the plots go")
    args = ap.parse_args()
    base, base_records = load(args.baseline)
    faulty, faulty_records = load(args.faults)
    os.makedirs(args.out, exist_ok=True)
    plot_success(base, faulty, os.path.join(args.out, "success.png"))
    if faulty.get("faults_by_kind"):
        plot_recovery(faulty, os.path.join(args.out, "recovery.png"))
    plot_placement(base_records, faulty_records, os.path.join(args.out, "placement.png"))
    plot_duration(base_records, faulty_records, os.path.join(args.out, "duration.png"))
    kinds = ", ".join(f"{k} {v['recovered']}/{v['injected']}"
                      for k, v in sorted(faulty.get("faults_by_kind", {}).items()))
    with open(os.path.join(args.out, "report.md"), "w") as f:
        f.write(f"# Reliability\n\n{table(base, faulty)}\n\n")
        f.write(f"Faults: {', '.join(faulty.get('faults', []))} at rate {faulty.get('rate')} per "
                f"opportunity, at most one per object a run. By kind: {kinds}.\n\n")
        for png in ("success", "recovery", "placement", "duration"):
            if os.path.exists(os.path.join(args.out, f"{png}.png")):
                f.write(f"![{png}]({png}.png)\n\n")
    print(f"wrote {args.out}/report.md")


if __name__ == "__main__":
    main()
