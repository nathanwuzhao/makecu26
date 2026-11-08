"""Named, single-shot skills: the menu the twin page's Skills panel (and later the agent) picks from.

Each skill = one bounded action built on skills.Skills, with
  name / label / group / desc   what it is (desc is shown as a tooltip, and fits an LLM prompt)
  params                        typed, clamped inputs (e.g. twist fraction)
  check(sk) -> str | None       why it can't run right now (None = OK). Keeps the order sane:
                                e.g. twist needs the cap gripped, recenter needs it released.
  run(sk, **params) -> dict     does it, returns a structured result for the event log

Run one with Controller.run_skill(name, params): same Stop / E-stop / lock as the unscrew run.
Unlike the unscrew run, a stopped skill leaves the gripper where it is (no clean-up move).
"""

from dataclasses import asdict, dataclass, field
from typing import Callable

import config
from skills import GripHolder, Skills

BOTTOM, TOP = config.BOTTOM_SQUEEZE, config.TOP_SQUEEZE
SLIDES = config.TWIST_SLIDES
SLIDE_IDX = [s.idx for s in SLIDES]


@dataclass
class Param:
    name: str
    default: float
    lo: float
    hi: float
    desc: str = ""

    def clamp(self, v) -> float:
        try:
            v = float(v)
        except (TypeError, ValueError):
            return self.default
        return max(self.lo, min(self.hi, v))


@dataclass
class Skill:
    name: str
    label: str
    group: str
    desc: str
    run: Callable[..., dict]
    check: Callable[[Skills], str | None] = lambda sk: None
    params: list[Param] = field(default_factory=list)
    servos: list[int] = field(default_factory=list)      # what it moves (shown in the UI)

    def info(self, sk: Skills | None = None) -> dict:
        d = {"name": self.name, "label": self.label, "group": self.group, "desc": self.desc,
             "servos": self.servos, "params": [asdict(p) for p in self.params]}
        if sk is not None:
            d["blocked"] = blocked(self, sk)
        return d


def blocked(skill: Skill, sk: Skills) -> str | None:
    try:
        return skill.check(sk)
    except RuntimeError as e:          # no D lines yet, etc.
        return str(e)


# ---------- checks ----------

def _known(sk: Skills, *idxs: int) -> str | None:
    limp = [i for i in idxs if sk.is_limp(i)]
    if limp:
        return f"servo {', '.join(map(str, limp))} is limp (position unknown): run Home first"
    return None


def _all(*checks):
    def f(sk):
        for c in checks:
            r = c(sk)
            if r:
                return r
        return None
    return f


def _not_holding(sq, what):
    return lambda sk: f"already holding the {what}" if sk.holding.get(sq.idx) else None


def _holding(sq, what):
    return lambda sk: None if sk.holding.get(sq.idx) else f"grip the {what} first"


def _cap_released(sk):
    return ("release the cap first (moving the top fingers while gripping turns the cap)"
            if sk.holding.get(TOP.idx) else None)


def _not_dispense(sk):
    sim = getattr(sk.robot, "sim_state", None)
    if callable(sim) and sim().get("task") == "dispense":
        return "eye-drop bottle: squeeze only (no twist / lift)"
    return None


def _recentered(sk):
    off = [s.idx for s in SLIDES if sk._angle(s.idx) != s.start]
    return f"top fingers not at the start position (servo {off}): Recenter first" if off else None


# ---------- actions ----------

def _home(sk: Skills) -> dict:
    sk.home()
    sk.holding.clear()
    return {"ok": True}


def _grip(sq):
    def f(sk: Skills) -> dict:
        g = sk.grip(sq)
        if not g.contact:
            sk.open(sq)            # closed fully on nothing: don't leave it squeezing
        return {"ok": g.contact, **asdict(g)}
    return f


def _release(sq):
    def f(sk: Skills) -> dict:
        sk.release(sq)
        return {"ok": True, "angle": sk._angle(sq.idx)}
    return f


def _open(sq):
    def f(sk: Skills) -> dict:
        sk.open(sq)
        return {"ok": True}
    return f


def _twist(sk: Skills, fraction: float) -> dict:
    holder = GripHolder(sk, TOP, config.GRIP_TARGET_MA[TOP.idx])
    sk.twist(fraction, holder)
    return {"ok": True, "fraction": fraction, "grip_during_twist": holder.summary()}


MIRROR = None   # main.py --real-fingers: real_fingers.RealFingers, gets the same t / y strokes


def _slide(right_forward: bool):
    """Top fingers only, one-way, same as tools/record.py t / y. 0 = back, 180 = forward.
    Keeps a held cap's grip regulated while sliding (so it turns the cap, either way)."""
    def run(sk: Skills) -> dict:
        target = {4: 180, 6: 0} if right_forward else {4: 0, 6: 180}
        if MIRROR is not None:
            MIRROR.stroke(right_forward)          # real servos 4 / 6 in parallel (background thread)
        holder = GripHolder(sk, TOP, config.GRIP_TARGET_MA[TOP.idx]) if sk.holding.get(TOP.idx) else None
        sk.move(target, config.TWIST_DEG_PER_TICK, hook=holder)
        return {"ok": True, **({"grip_during_twist": holder.summary()} if holder else {})}
    return run


def _recenter(sk: Skills) -> dict:
    sk.recenter()
    return {"ok": True}


def _twist_cycle(sk: Skills, fraction: float) -> dict:
    return sk.twist_cycle(fraction)


def _lift(angle_of):
    def f(sk: Skills) -> dict:
        hook = None
        if sk.holding.get(TOP.idx):    # keep the cap grip on target while the halves move
            hook = GripHolder(sk, TOP, config.GRIP_TARGET_MA[TOP.idx])
        sk.move({config.HALVES.idx: angle_of()}, config.LIFT_DEG_PER_TICK, hook=hook)
        return {"ok": True, "angle": sk._angle(config.HALVES.idx)}
    return f


FRACTION = Param("fraction", config.DEFAULT_TWIST_FRACTION, 0.1, 1.0, "share of the full slide stroke")

SKILLS: list[Skill] = [
    Skill("home", "Home", "setup",
          "Open both squeezes, bottom fingers to their hold position, top fingers to the twist start, "
          "halves to cap height. Safe first move after power-up or E-stop (limp servos jump straight there).",
          _home, servos=[0, 1, 2, 3, 4, 5, 6]),

    Skill("grip_bottle", "Grip bottle", "bottle",
          "Close the bottom squeeze until current says it touches the bottle, then hold at the target current.",
          _grip(BOTTOM), _all(lambda sk: _known(sk, BOTTOM.idx), _not_holding(BOTTOM, "bottle")), servos=[BOTTOM.idx]),
    Skill("release_bottle", "Release bottle", "bottle",
          "Back the bottom squeeze off just enough to let go.",
          _release(BOTTOM), lambda sk: _known(sk, BOTTOM.idx), servos=[BOTTOM.idx]),
    Skill("open_bottom", "Open bottom", "bottle",
          "Bottom squeeze fully open.",
          _open(BOTTOM), servos=[BOTTOM.idx]),

    Skill("grip_cap", "Grip cap", "cap",
          "Close the top squeeze until current says it touches the cap, then hold at the target current.",
          _grip(TOP), _all(lambda sk: _known(sk, TOP.idx), _not_holding(TOP, "cap")), servos=[TOP.idx]),
    Skill("twist", "Twist", "cap",
          "Slide the top fingers in opposite directions while holding the cap grip on target: turns the cap "
          "counter-clockwise (loosening).",
          _twist, _all(_not_dispense, lambda sk: _known(sk, TOP.idx, *SLIDE_IDX), _holding(TOP, "cap")),
          [FRACTION], servos=[*SLIDE_IDX, TOP.idx]),
    Skill("release_cap", "Release cap", "cap",
          "Back the top squeeze off just enough to let go of the cap.",
          _release(TOP), lambda sk: _known(sk, TOP.idx), servos=[TOP.idx]),
    Skill("recenter", "Recenter", "cap",
          "Top fingers back to the twist start position. Needs the cap released.",
          _recenter, _all(lambda sk: _known(sk, *SLIDE_IDX), _cap_released), servos=SLIDE_IDX),
    Skill("twist_cycle", "Twist cycle", "cap",
          "One full loosening stroke: grip cap -> twist -> release -> recenter.",
          _twist_cycle, _all(_not_dispense, lambda sk: _known(sk, TOP.idx, *SLIDE_IDX), _cap_released, _recentered),
          [FRACTION], servos=[TOP.idx, *SLIDE_IDX]),
    Skill("open_top", "Open top", "cap",
          "Top squeeze fully open.",
          _open(TOP), servos=[TOP.idx]),

    # Manual twist strokes (keys t / y on the page). Not offered to Gemini.
    Skill("slide_rf", "R fwd · L back (t)", "manual",
          "Top fingers only: right finger forward, left finger back (the loosening direction).",
          _slide(True), _all(_not_dispense, lambda sk: _known(sk, *SLIDE_IDX)), servos=SLIDE_IDX),
    Skill("slide_rb", "R back · L fwd (y)", "manual",
          "Top fingers only: right finger back, left finger forward (tightens a gripped cap).",
          _slide(False), _all(_not_dispense, lambda sk: _known(sk, *SLIDE_IDX)), servos=SLIDE_IDX),

    Skill("lift_test", "Lift (test)", "lift",
          "Raise the top half a few mm to the test height (keeps the cap grip regulated if holding it).",
          _lift(lambda: config.HALVES.test), _all(_not_dispense, lambda sk: _known(sk, config.HALVES.idx)), servos=[config.HALVES.idx]),
    Skill("lower", "Lower", "lift",
          "Top half back down to cap height.",
          _lift(lambda: config.HALVES.grip), lambda sk: _known(sk, config.HALVES.idx), servos=[config.HALVES.idx]),
    Skill("lift_full", "Lift (full)", "lift",
          "Raise the top half all the way (the reveal, once the cap is off).",
          _lift(lambda: config.HALVES.full), _all(_not_dispense, lambda sk: _known(sk, config.HALVES.idx)), servos=[config.HALVES.idx]),
]

BY_NAME = {s.name: s for s in SKILLS}


def catalog(sk: Skills | None = None) -> list[dict]:
    return [s.info(sk) for s in SKILLS]


# ---------- demo scenarios (simulator only: python src/main.py --sim) ----------
# One bounded macro each, built on the Skills primitives (so Stop / E-stop abort them at the next
# tick). They read the simulated object through sk.robot.sim_state() / sim_info() and load the
# object they need themselves (SimRobot.set_object, after Home has opened both squeezes).

import math

from sim_robot import MM_PER_DEG


def _sim(sk: Skills) -> dict:
    return sk.robot.sim_state()


def _sim_obj(sk: Skills) -> dict:
    info = sk.robot.sim_info()
    return next(o for o in info["objects"] if o["name"] == info["object"])


def _sim_only(sk: Skills) -> str | None:
    return None if callable(getattr(sk.robot, "sim_state", None)) else "simulator only (python src/main.py --sim)"


def _demo_home(sk: Skills, obj: str, fresh: bool):
    """Home (opens both squeezes), then load the object (fresh = reset its state)."""
    sk.home()
    sk.holding.clear()
    if fresh or _sim(sk)["object"] != obj:
        ok, msg = sk.robot.set_object(obj)
        if not ok:
            raise RuntimeError(msg)
        sk.last_contact.clear()          # contact angles of the previous object would overshoot
    sk.calibrate_tactile()


def _grip_or_fail(sk: Skills, sq, what: str):
    g = sk.grip(sq)
    if not g.contact:
        sk.open(sq)
        return {"ok": False, "problem": f"{what}: closed fully without contact", "grip": asdict(g)}
    return None


def _demo_unscrew(sk: Skills, cycles: float) -> dict:
    cur = _sim(sk)
    _demo_home(sk, cur["object"] if cur.get("task") == "unscrew" else "pill_tube", fresh=True)
    fail = _grip_or_fail(sk, BOTTOM, "bottle")
    if fail:
        return fail
    log = []
    for c in range(1, int(cycles) + 1):
        r = sk.twist_cycle(1.0)
        s = _sim(sk)
        log.append({"cycle": c, "ok": r["ok"], "cap_turn_deg": round(s["cap_turn_deg"], 1),
                    "grip_ma": r.get("grip_during_twist"), "tactile_shift": r.get("tactile_centroid_shift")})
        if not r["ok"] or s["cap_off"]:
            break
    s = _sim(sk)
    if s["cap_off"]:                       # the reveal: grip the cap, lift the top half all the way
        fail = _grip_or_fail(sk, TOP, "cap")
        if fail:
            return {**fail, "per_cycle": log}
        sk.move({config.HALVES.idx: config.HALVES.full}, config.LIFT_DEG_PER_TICK,
                hook=GripHolder(sk, TOP, config.GRIP_TARGET_MA[TOP.idx]))
        s = _sim(sk)
    return {"ok": s["cap_off"], "cycles_used": len(log), "cap_turn_deg": round(s["cap_turn_deg"], 1),
            "cap_off": s["cap_off"], "cap_lift_mm": s["cap_lift_mm"], "per_cycle": log}


def _demo_eye_drop(sk: Skills, drops: float) -> dict:
    _demo_home(sk, "eye_dropper", fresh=True)
    sk.move({s.idx: 90 for s in config.TWIST_SLIDES})      # fingers side by side: no twist, just squeeze
    for sq, what in ((BOTTOM, "shoulder"), (TOP, "body")):
        fail = _grip_or_fail(sk, sq, what)
        if fail:
            return fail
    o = _sim_obj(sk)["sim"]
    hold_angle = sk._angle(TOP.idx)
    presses = []
    for _ in range(int(drops)):
        p = _sim(sk)["squeeze_mm"]["top"]
        thr = config.GRIP_TARGET_MA[TOP.idx] / o["k"]["top"] + o["squeeze_mm_per_drop"]
        deg = math.ceil((thr + 0.4 - p) / MM_PER_DEG)          # just past one drop's worth
        sk.move({TOP.idx: min(TOP.closed, hold_angle + deg)}, 2)
        sk._sleep(0.3)
        presses.append({"press_deg": deg, "peak_ma": round(sk.hold_current(TOP, 2, max), 1)})
        sk.move({TOP.idx: hold_angle}, 2)                       # back to the regulated hold
        sk._sleep(0.4)
    s = _sim(sk)
    sk.release(TOP)
    sk.release(BOTTOM)
    return {"ok": s["drops"] >= int(drops), "drops": s["drops"], "presses": presses}


DEMO_SKILLS = [
    Skill("demo_unscrew", "Demo: unscrew cap", "demo",
          "Simulator scenario: load a fresh pill tube / bottle, Home, grip the bottle, twist cycles until the "
          "child-proof cap comes off (or `cycles` runs out), then grip the cap and lift it clear.",
          _demo_unscrew, _sim_only, [Param("cycles", 8, 1, 12, "max twist cycles")], servos=[0, 1, 2, 3, 4, 5, 6]),
    Skill("demo_eye_drop", "Demo: eye drops", "demo",
          "Simulator scenario: load the eye-drop bottle (held inverted), bottom fingers steady it, "
          "top fingers squeeze and relax once per drop (no twisting).",
          _demo_eye_drop, _sim_only, [Param("drops", 2, 1, 5, "drops to dispense")], servos=[0, 2, 5]),
]
SKILLS.extend(DEMO_SKILLS)
BY_NAME.update({s.name: s for s in DEMO_SKILLS})
