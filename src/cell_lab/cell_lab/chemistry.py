"""
This is the lab's chemistry, the true state of every sample and the rules it changes by, kept
apart from ROS so it can be tested by stepping a clock. It reads the protocols in
cell_description/config/lab/lab.yaml. An assay sample reacts once, after a reaction time drawn
for it, and ends done or failed; a titration sample moves one look along its colour scale each
time it has had enough doses, with the number of doses per step drawn for it, so nobody can
know in advance which dose reaches the endpoint; a growth sample's colony grows a stage after
each drawn stage time; and a readout sample shows one drawn point of an indicator scale and
never changes. Every draw comes from one random generator
seeded at reset, which is what makes a benchmark run repeatable. A change that falls due while
the sample is in the gripper waits until it is set down again, because the lab node shows a
change by teleporting a different body into the sample's place and doing that to a held body
would pull it out of the fingers. The lab node turns the changes step() reports into those
swaps and publishes the state for scoring; the robot never reads any of it.
"""
import math
import random
from dataclasses import dataclass, field


@dataclass
class Event:
    """One thing that happened to a sample, at simulation time t."""
    t: float
    kind: str      # set_out, changed, dosed, lifted
    detail: dict = field(default_factory=dict)


@dataclass
class Sample:
    name: str
    protocol: str
    slot: str
    looks: list          # every look this sample can have, in the order of its twins
    look: str            # the look shown now
    state: str           # assay: start, done, failed; titration: stage0, stage1, ...
    due: float = math.inf  # when the next change is due (inf when nothing is pending)
    pending: str = ""      # the look that change shows
    pending_state: str = ""
    doses: int = 0
    plan: dict = field(default_factory=dict)  # what was drawn at reset, for scoring
    events: list = field(default_factory=list)


def looks_of(protocol):
    """A protocol's looks as a list, whether lab.yaml gives them as a mapping or a list."""
    looks = protocol["looks"]
    return list(looks.values()) if isinstance(looks, dict) else list(looks)


class Lab:
    """Every sample in lab.yaml, of which reset() sets out one protocol's."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.protocols = cfg["protocols"]
        self.samples = {}
        for pname, p in self.protocols.items():
            for name, slot in p["samples"].items():
                looks = looks_of(p)
                self.samples[name] = Sample(name, pname, slot, looks, looks[0], "")
        self.active = []
        self.protocol = ""
        self.seed = None

    # - reset

    def reset(self, protocol, seed, t):
        """Set out every sample of `protocol` in its start look, drawing its future from `seed`.
        Returns the active sample names. Raises KeyError for an unknown protocol."""
        if protocol not in self.protocols:
            raise KeyError(f"unknown protocol '{protocol}', expected one of "
                           f"{', '.join(self.protocols)}")
        rng = random.Random(seed)
        p = self.protocols[protocol]
        self.protocol, self.seed = protocol, seed
        self.active = list(p["samples"])
        for name in self.samples:
            s = self.samples[name]
            s.look, s.state, s.due, s.pending, s.pending_state = s.looks[0], "", math.inf, "", ""
            s.doses, s.plan, s.events = 0, {}, []
        for name in self.active:
            s = self.samples[name]
            if protocol == "assay":
                self._plan_assay(s, p, rng, t)
            elif protocol == "titration":
                self._plan_titration(s, p, rng)
            elif protocol == "growth":
                self._plan_growth(s, p, rng, t)
            elif protocol == "readout":
                self._plan_readout(s, p, rng)
            else:
                raise KeyError(f"protocol '{protocol}' has no chemistry")
            s.events.append(Event(t, "set_out", {"slot": s.slot, "look": s.look}))
        return list(self.active)

    def _plan_assay(self, s, p, rng, t):
        looks = p["looks"]
        lo, hi = p["reaction_time"]
        delay = rng.uniform(lo, hi)
        outcome = "failed" if rng.random() < p["failure_rate"] else "done"
        s.state, s.look = "start", looks["start"]
        s.due, s.pending, s.pending_state = t + delay, looks[outcome], outcome
        s.plan = {"reaction_time": round(delay, 2), "outcome": outcome}

    def _plan_titration(self, s, p, rng):
        lo, hi = p["doses_per_stage"]
        looks = looks_of(p)
        # Cumulative doses at which each later look appears: stage k shows after thresholds[k-1].
        # Up to the endpoint each step takes a drawn number of doses; past it every step takes
        # one, so a single dose too many already shows.
        thresholds, total = [], 0
        for k in range(1, len(looks)):
            total += rng.randint(lo, hi) if k <= p["endpoint"] else 1
            thresholds.append(total)
        s.state, s.look = "stage0", looks[0]
        s.plan = {"thresholds": thresholds, "endpoint": p["endpoint"],
                  "endpoint_doses": thresholds[p["endpoint"] - 1]}

    def _plan_growth(self, s, p, rng, t):
        lo, hi = p["stage_time"]
        times = [rng.uniform(lo, hi) for _ in range(len(s.looks) - 1)]
        s.state, s.look = "stage0", s.looks[0]
        s.due, s.pending, s.pending_state = t + times[0], s.looks[1], "stage1"
        areas = [math.pi * self.cfg["types"][look]["colony"]["radius"] ** 2 for look in s.looks]
        ready = next((k for k, a in enumerate(areas) if a >= p["harvest_area"]), None)
        s.plan = {"stage_times": [round(x, 2) for x in times], "areas_mm2":
                  [round(1e6 * a, 1) for a in areas], "harvest_stage": ready,
                  "harvest_area_mm2": round(1e6 * p["harvest_area"], 1)}

    def _plan_readout(self, s, p, rng):
        look = rng.choice(s.looks)
        s.look, s.state = look, p["truth"][look]
        s.plan = {"look": look, "truth": p["truth"][look]}

    # - dosing

    def dispense(self, name, t):
        """One dose into sample `name` at time t. Its colour moves, if this dose is enough for
        the next step, after the protocol's mix time. Returns the dose count."""
        s = self.samples[name]
        if self.protocol != "titration" or name not in self.active:
            raise ValueError(f"{name} is not a titration sample in the running protocol")
        p = self.protocols["titration"]
        s.doses += 1
        stage = sum(1 for th in s.plan["thresholds"] if s.doses >= th)
        looks = looks_of(p)
        s.events.append(Event(t, "dosed", {"doses": s.doses}))
        if looks[stage] != s.look and looks[stage] != s.pending:
            s.due, s.pending, s.pending_state = t + p["mix_time"], looks[stage], f"stage{stage}"
        return s.doses

    # - time

    def step(self, t, held):
        """Apply every change that is due by time t. held(name) says whether a sample is in the
        gripper now, and a held sample's change waits. Returns [(name, old look, new look)]."""
        swaps = []
        for name in self.active:
            s = self.samples[name]
            if s.due > t or held(name):
                continue
            old = s.look
            s.look, s.state = s.pending, s.pending_state
            s.events.append(Event(t, "changed", {"from": old, "to": s.look,
                                                 "due": round(s.due, 3), "state": s.state}))
            s.due, s.pending, s.pending_state = math.inf, "", ""
            swaps.append((name, old, s.look))
            if s.protocol == "growth":
                # the next stage's clock starts when this one shows
                k = s.looks.index(s.look)
                if k + 1 < len(s.looks):
                    s.due = t + s.plan["stage_times"][k]
                    s.pending, s.pending_state = s.looks[k + 1], f"stage{k + 1}"
        return swaps

    def lifted(self, name, t):
        """Record that a sample was picked up at time t (once per change, for reaction times)."""
        s = self.samples[name]
        last_change = max((e.t for e in s.events if e.kind == "changed"), default=None)
        last_lift = max((e.t for e in s.events if e.kind == "lifted"), default=None)
        if last_lift is None or (last_change is not None and last_change > last_lift):
            s.events.append(Event(t, "lifted", {"look": s.look}))

    # - reporting

    def state(self, t):
        """The whole lab as plain data, for /lab/state."""
        return {
            "t": round(t, 3), "protocol": self.protocol, "seed": self.seed,
            "samples": {name: {
                "protocol": s.protocol, "slot": s.slot, "look": s.look, "state": s.state,
                "doses": s.doses, "plan": s.plan, "pending": s.pending,
                "events": [{"t": round(e.t, 3), "kind": e.kind, **e.detail} for e in s.events],
            } for name, s in self.samples.items() if name in self.active},
        }


def twin_name(sample, look):
    """The MuJoCo body (and free joint) that shows `sample` with `look`."""
    return f"{sample}__{look}"


def parking_spots(cfg):
    """{twin name: (x, y, z)} where every twin waits, the same grid build_lab_mjcf.py lays out."""
    parking = cfg["parking"]
    (x0, x1), (y0, y1), pitch = parking["x"], parking["y"], parking["pitch"]
    xs = [x0 + i * pitch for i in range(int(math.floor((x1 - x0) / pitch)) + 1)]
    ys = [y0 + j * pitch for j in range(int(math.floor((y1 - y0) / pitch)) + 1)]
    grid = [(x, y) for x in xs for y in ys]
    names = []
    for p in cfg["protocols"].values():
        for sample in p["samples"]:
            names += [(sample, look) for look in looks_of(p)]
    if len(grid) < len(names):
        raise ValueError(f"parking grid has {len(grid)} spots for {len(names)} twins")
    spots = {}
    for (sample, look), (x, y) in zip(names, grid):
        height = cfg["types"][look]["size"][2]
        spots[twin_name(sample, look)] = (x, y, parking["floor"] + height / 2)
    return spots
