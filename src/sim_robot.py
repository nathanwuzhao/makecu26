"""Simulated gripper + object: a DTwinRobot that knows what it is holding.

    python src/main.py --sim [pill_bottle|pill_tube|eye_dropper]

Same interface as DTwinRobot / RobotSerial (state, listeners, set, motion_locked, command,
query_servo_config, close), so skills.py / Controller / the twin run unchanged. Extra:
  set_object(name) -> (ok, msg)   swap the object (only when neither squeeze touches it; resets its state)
  sim_state() -> dict             the object state below (also in every state as st.sim)
  sim_info() -> dict              objects list (web/objects.json merged with fallbacks) + current state

Model (normalized servos: 0 = home/open/back/down, 180 = far/closed/forward/up; 24.3 mm per 0..180):
  squeeze  each side closes 24.3*a/180 mm; open gap 45.2 mm. Per-side penetration
           p = 24.3*a/180 - (45.2 - D)/2  (> 0 = touching). Holding current = free air (3-6 mA)
           + k*p (k mA/mm, ~100: 1.5 mm ~ 150 mA), cap 700, + a 300-500 mA inrush after each move.
  twist    top slides move d mm (loosening direction = config.TWIST_SLIDES start->end) -> a gripped
           part of radius r turns d/r rad, IF top current >= 60% of target (else slip) and the
           bottom grip holds the body (else the whole object spins).
  halves   servo 2 lifts the top layer 0..24.3 mm. Moves while the top squeeze grips: cap lift
           (pill bottle, once off) or plunger push (insulin pen, downward).
  tactile  FlexiTac 16x32 on the inner face of the top RIGHT finger tip (finger tip:3, y=-22.6):
           cols along X (2 mm pitch; col 0 edge at CAD x=40.85 when servo 4 = 0, so the pad moves
           with the finger), rows along Z of the top layer (row 0 = lowest, 2 mm pitch, 0..32 mm).
           Contact = vertical elliptical patch at the object's X; rolls across the pad during a twist,
           shears under load, smears on slip.
"""

import json
import math
import random
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import config
from robot_serial import LIMP, NUM_SERVOS, TACTILE_COLS, TACTILE_COUNT, TACTILE_ROWS, MotionLocked, RobotState

OBJECTS_FILE = Path(__file__).parent.parent / "web" / "objects.json"
MM_PER_DEG = 24.3 / 180
OPEN_GAP_MM = 45.2
WORK_X = 85.0
PAD_PITCH = 2.0
PAD_X0 = 40.85                  # CAD x of the pad's col-0 edge with servo 4 at 0
TOP, BOTTOM, HALVES = config.TOP_SQUEEZE, config.BOTTOM_SQUEEZE, config.HALVES
PAD_SLIDE = 4                   # top finger R slide carries the pad
CURRENT_CAP_MA = 700.0
GRIP_OK_FRACTION = 0.6          # of GRIP_TARGET_MA: below this a twist slips
TWIST_LOAD = 0.75               # holding current sags under twist torque (GripHolder re-tightens)

# Used when web/objects.json is missing a value. k = holding mA per mm of per-side penetration;
# top_z_mm = where the gripped part touches the pad, along Z of the top layer (0..32).
FALLBACK = {
    "pill_bottle": {"label": "Pill bottle", "task": "unscrew", "grip": {"bottom_d_mm": 34, "top_d_mm": 38},
                    "sim": {"turns_to_open": 1.5, "thread_pitch_mm": 3, "turn_efficiency": 0.95,
                            "k": {"bottom": 100, "top": 100}, "top_z_mm": [1, 19]}},
    "eye_dropper": {"label": "Eye-drop bottle", "task": "dispense", "grip": {"bottom_d_mm": 24, "top_d_mm": 24},
                    "sim": {"squeeze_mm_per_drop": 1.5, "k": {"bottom": 110, "top": 70}, "top_z_mm": [0, 32]}},
}


def load_objects() -> dict:
    """objects.json entries merged over FALLBACK (json wins; 'sim' merged key by key)."""
    try:
        objs = json.loads(OBJECTS_FILE.read_text(encoding="utf-8")).get("objects", {})
    except (FileNotFoundError, json.JSONDecodeError):
        objs = {}
    out = {}
    for name in [*FALLBACK, *[n for n in objs if n not in FALLBACK]]:
        fb, js = FALLBACK.get(name, FALLBACK["pill_bottle"]), objs.get(name, {})
        o = {**fb, **js}
        o["grip"] = {**fb["grip"], **js.get("grip", {})}
        o["sim"] = {**fb["sim"], **js.get("sim", {})}
        o["sim"].setdefault("k", {"bottom": 100, "top": 100})
        o["sim"].setdefault("turn_efficiency", 0.95)
        o["sim"].setdefault("top_z_mm", [4, 28])
        out[name] = o
    return out


@dataclass
class SimState(RobotState):
    sim: dict | None = None     # twin_server forwards this as latest.sim


class SimRobot:
    motion_locked = False

    def __init__(self, object_name: str = "pill_bottle"):
        self._angles = [LIMP] * NUM_SERVOS   # limp until the first S, like the firmware
        self._moved_at = [0.0] * NUM_SERVOS
        self._spike = [(0.0, 0.0)] * NUM_SERVOS    # (amplitude mA, duration s) of the last inrush
        self._state = None
        self._lock = threading.RLock()
        self.listeners = []
        self._stop = threading.Event()
        self._rng = random.Random(1)
        r = random.Random(42)                       # fixed per-cell pad character (offset + gain)
        self._pad_off = [r.randint(0, 4) for _ in range(TACTILE_COUNT)]
        self._pad_gain = [0.78 + 0.4 * r.random() for _ in range(TACTILE_COUNT)]
        self.objects = load_objects()
        self._reset_object(object_name if object_name in self.objects else "pill_bottle")
        threading.Thread(target=self._loop, daemon=True).start()

    def start(self) -> "SimRobot":
        time.sleep(0.1)
        return self

    @property
    def state(self):
        with self._lock:
            return self._state

    def set(self, angles: dict[int, float]):
        if self.motion_locked:
            raise MotionLocked("E-stop active: motion locked")
        with self._lock:
            now = time.time()
            for i, a in angles.items():
                a = int(round(min(180, max(0, a))))
                if a != self._angles[i]:
                    size = 4 if self._angles[i] == LIMP else min(4, abs(a - self._angles[i]))
                    # bigger moves = bigger, longer inrush (1 deg nudge ~75 ms, >= 4 deg ~150 ms)
                    self._spike[i] = (self._rng.uniform(300, 500) * min(1.0, 0.4 + 0.15 * size), 0.05 + 0.025 * size)
                    self._moved_at[i] = now
                self._angles[i] = a

    # --- C / R like the firmware (same as DTwinRobot) ---
    _cal = None

    def _ensure_cal(self):
        if self._cal is None:
            self._cal = {i: [500, 2400, 0] for i in range(NUM_SERVOS)}

    def command(self, line: str):
        self._ensure_cal()
        t = line.split()
        if t[0].upper() == "R" and len(t) == 3:
            i, v = int(t[1]), int(t[2])
            if v != self._cal[i][2]:
                self._cal[i][2] = v
                with self._lock:
                    self._angles[i] = LIMP          # firmware: flag change -> limp
        elif t[0].upper() == "C" and len(t) == 4:
            i = int(t[1])
            self._cal[i][0], self._cal[i][1] = int(t[2]), int(t[3])

    def query_servo_config(self, timeout: float = 1.5):
        self._ensure_cal()
        return {i: tuple(v) for i, v in self._cal.items()}

    def close(self):
        self._stop.set()

    # ---------- object ----------

    def set_object(self, name: str) -> tuple[bool, str]:
        """Swap the object (re-reads objects.json). Refused while either squeeze touches the
        current object; always resets the object's state (cap back on, dose 0, drops 0)."""
        self.objects = load_objects()
        if name not in self.objects:
            return False, f"unknown object {name!r} (have {', '.join(self.objects)})"
        with self._lock:
            if self.s["contact"]["bottom"] or self.s["contact"]["top"]:
                return False, "release both squeezes first (the gripper is touching the object)"
            self._reset_object(name)
        return True, f"loaded {self.obj['label']}"

    def sim_state(self) -> dict:
        with self._lock:
            return self._public()

    def _public(self) -> dict:
        """Deep copy of the object state with floats rounded (0.01)."""
        r = lambda v: round(v, 2) if isinstance(v, float) else {k: r(x) for k, x in v.items()} if isinstance(v, dict) else v
        return r(self.s)

    def sim_info(self) -> dict:
        with self._lock:
            return {"object": self.name, "sim": self.sim_state(),
                    "objects": [{"name": n, "label": o["label"], "task": o["task"], "grip": o["grip"], "sim": o["sim"]}
                                for n, o in self.objects.items()],
                    "params": {"mm_per_deg": MM_PER_DEG, "open_gap_mm": OPEN_GAP_MM, "pad_pitch_mm": PAD_PITCH,
                               "pad_x0_mm": PAD_X0, "contact_deg": {k: round(self._contact_deg(k), 1) for k in ("bottom", "top")}}}

    def _contact_deg(self, side: str) -> float:
        return (OPEN_GAP_MM - self.obj["grip"][f"{side}_d_mm"]) / 2 / MM_PER_DEG

    def _reset_object(self, name: str):
        self.name, self.obj = name, self.objects[name]
        self.s = {"object": name, "label": self.obj["label"], "task": self.obj["task"],
                  "contact": {"bottom": False, "top": False}, "squeeze_mm": {"bottom": 0.0, "top": 0.0},
                  "gap_mm": {"bottom": OPEN_GAP_MM, "top": OPEN_GAP_MM},
                  "cap_turn_deg": 0.0, "cap_off": False, "cap_lift_mm": 0.0, "cap_thread_mm": 0.0,
                  "drops": 0, "dose_units": 0, "dial_deg": 0.0, "plunger_mm": 0.0, "knob_out_mm": 0.0,
                  "delivered_units": 0, "slip": False, "spinning": False, "halves_mm": 0.0,
                  "shear_cols": 0.0, "note": f"{self.obj['label']} loaded"}
        self._prev = None            # angles last tick (twist / lift deltas)
        self._anchor = None          # halves_mm when the top grip closed
        self._push_base = 0.0        # plunger_mm when the top grip closed
        self._dose_mm = 0.0
        self._drop_armed = True
        self._slip_until = 0.0
        self._slip_dir = 0.0

    # ---------- physics ----------

    def _step(self, a: list[int], now: float):
        s, o, sim = self.s, self.obj, self.obj["sim"]
        pos = lambda i: 0 if a[i] == LIMP else a[i]          # limp: sags to home (halves sink)
        prev = self._prev or a
        p, hold = {}, {}
        for side, sq in (("bottom", BOTTOM), ("top", TOP)):
            p[side] = pos(sq.idx) * MM_PER_DEG - (OPEN_GAP_MM - o["grip"][f"{side}_d_mm"]) / 2
            hold[side] = sim["k"][side] * max(0.0, p[side])
            s["contact"][side] = p[side] > 0
            s["squeeze_mm"][side] = round(max(0.0, p[side]), 2)
            s["gap_mm"][side] = round(OPEN_GAP_MM - 2 * pos(sq.idx) * MM_PER_DEG, 2)
        grip_ok = {side: hold[side] >= GRIP_OK_FRACTION * config.GRIP_TARGET_MA[sq.idx]
                   for side, sq in (("bottom", BOTTOM), ("top", TOP))}
        s["halves_mm"] = round(pos(HALVES.idx) * MM_PER_DEG, 2)
        task, top = s["task"], s["contact"]["top"]

        # twist: mean slide travel in the loosening direction (start -> end), mm
        d = sum((a[sl.idx] - prev[sl.idx]) * (1 if sl.end > sl.start else -1)
                for sl in config.TWIST_SLIDES if LIMP not in (a[sl.idx], prev[sl.idx])) * MM_PER_DEG / 2
        twisting = abs(d) > 1e-6 and top
        s["spinning"] = False
        if twisting:
            if not grip_ok["top"]:
                self._slip_until, self._slip_dir = now + 0.6, math.copysign(1, d)
                s["note"] = "top fingers slipping: grip force too low to turn it"
            elif task in ("unscrew", "dial_and_inject") and not grip_ok["bottom"]:
                s["spinning"] = True
                s["note"] = "the whole object spins: grip the body with the bottom fingers first"
            else:
                turn = math.degrees(d / (o["grip"]["top_d_mm"] / 2)) * sim["turn_efficiency"]
                if task == "unscrew" and not s["cap_off"]:
                    s["cap_turn_deg"] = max(0.0, s["cap_turn_deg"] + turn)
                    need = sim["turns_to_open"] * 360
                    if s["cap_turn_deg"] >= need:
                        s["cap_off"] = True
                        s["note"] = "cap free: threads disengaged"
                    else:
                        s["note"] = f"cap {'loosening' if turn > 0 else 'tightening'}: {s['cap_turn_deg']:.0f} / {need:.0f} deg"
                elif task == "dial_and_inject":
                    upt, mx = sim["units_per_turn"], sim["max_units"]
                    s["dial_deg"] = min(mx / upt * 360, max(0.0, s["dial_deg"] + turn))
                    units = round(s["dial_deg"] / 360 * upt)
                    if units != s["dose_units"]:
                        s["dose_units"], s["plunger_mm"], self._push_base = units, 0.0, 0.0
                        self._dose_mm = units * sim["plunger_mm_per_unit"]
                        s["note"] = f"dialled {units} units (click)"
                elif task == "dispense":
                    s["note"] = "nothing to twist on an eye-drop bottle"
        s["slip"] = now < self._slip_until
        # elastic shear of the pad before it slips (decays when the fingers stop)
        if twisting and not s["slip"] and not s["spinning"]:
            s["shear_cols"] = max(-1.5, min(1.5, s["shear_cols"] + 0.5 * d / PAD_PITCH))
        else:
            s["shear_cols"] *= 0.8

        # halves while the top grips: cap lift / plunger push
        if top:
            if self._anchor is None:
                self._anchor, self._push_base = s["halves_mm"], s["plunger_mm"]
            dz = s["halves_mm"] - self._anchor
            if task == "unscrew":
                if s["cap_off"]:
                    s["cap_lift_mm"] = round(s["cap_thread_mm"] + max(0.0, dz), 2)
                    if dz > 0.5:
                        s["note"] = f"cap lifted {dz:.1f} mm"
                elif dz > 0.3:
                    s["note"] = "threads still engaged: the cap won't lift (fingers slide up the cap)"
            elif task == "dial_and_inject" and dz < -0.05:
                if self._dose_mm > 0 and s["dose_units"] > 0:
                    s["plunger_mm"] = round(min(self._dose_mm, self._push_base - dz), 2)
                    if s["plunger_mm"] >= self._dose_mm - 0.05:
                        s["delivered_units"] += s["dose_units"]
                        s["note"] = f"dose delivered: {s['dose_units']} units"
                        s["dose_units"], s["dial_deg"] = 0, 0.0
                    else:
                        s["note"] = f"injecting: {s['plunger_mm']:.1f} / {self._dose_mm:.1f} mm"
                elif dz < -0.3 and not s["note"].startswith("dose delivered"):
                    s["note"] = "knob bottomed out (no dose dialled)"
        else:
            self._anchor = None
            if task == "unscrew" and s["cap_off"] and s["cap_lift_mm"] > s["cap_thread_mm"]:
                s["cap_lift_mm"] = s["cap_thread_mm"]
                s["note"] = "cap released: it dropped back onto the bottle"
        if task == "unscrew":   # thread rise while turning (and while off it just sits loose)
            s["cap_thread_mm"] = round(min(s["cap_turn_deg"], sim["turns_to_open"] * 360) / 360
                                       * sim.get("thread_pitch_mm", 0), 2)
            if not (s["cap_off"] and top):
                s["cap_lift_mm"] = s["cap_thread_mm"]
        if task == "dial_and_inject":
            s["knob_out_mm"] = round(max(0.0, self._dose_mm - s["plunger_mm"]), 2) if s["dose_units"] else 0.0

        # eye drops: each squeeze past (regulated hold + squeeze_mm_per_drop) = one drop
        if task == "dispense":
            thr = config.GRIP_TARGET_MA[TOP.idx] / sim["k"]["top"] + sim["squeeze_mm_per_drop"]
            if p["top"] >= thr and self._drop_armed:
                s["drops"] += 1
                self._drop_armed = False
                s["note"] = f"drop {s['drops']} dispensed" + ("" if grip_ok["bottom"] else " (bottle not steadied!)")
            elif p["top"] < thr - 0.5:
                self._drop_armed = True

        # currents
        cur, expected = [0.0, 0.0], [0.0, 0.0]   # expected = the model without inrush spikes / noise (page plot)
        for side, sq in (("bottom", BOTTOM), ("top", TOP)):
            if a[sq.idx] == LIMP:
                cur[sq.current_ch] = 1.5 + self._rng.uniform(0, 1)
                continue
            h = hold[side] * (TWIST_LOAD if side == "top" and twisting else 1.0)
            expected[sq.current_ch] = round(min(CURRENT_CAP_MA, 3 + 3 * a[sq.idx] / 180 + h), 1)
            amp, dur = self._spike[sq.idx]
            spike = amp if now - self._moved_at[sq.idx] < dur else 0.0
            noise = self._rng.gauss(0, 0.8) + 0.02 * h * self._rng.gauss(0, 1)
            cur[sq.current_ch] = round(min(CURRENT_CAP_MA, max(0.0, 3 + 3 * a[sq.idx] / 180 + h + spike + noise)), 1)

        s["expected_ma"] = expected
        self._prev = list(a)
        tactile = self._tactile(max(0.0, p["top"]), pos(PAD_SLIDE), s["slip"], s["shear_cols"])
        return cur, tactile

    def _tactile(self, p: float, slide: int, slip: bool, shear: float) -> bytes:
        rng, sim, d = self._rng, self.obj["sim"], self.obj["grip"]["top_d_mm"]
        f = [self._pad_off[i] + rng.randint(0, 2) for i in range(TACTILE_COUNT)]   # resting 0-6
        if p > 0:
            k = sim["k"]["top"]
            half_w = math.sqrt(2 * (d / 2) * p * 100 / k)          # Hertz-ish line contact, softer = wider
            sig = max(0.9, 0.6 * half_w / PAD_PITCH)
            peak = 10 + 230 * (1 - math.exp(-k * p / 160))          # ~150 mA hold -> ~150 counts
            c0 = (WORK_X - slide * MM_PER_DEG - PAD_X0) / PAD_PITCH - 0.5 + shear   # rolls with the finger
            if slip:
                sig, peak = sig * 1.8, peak * 0.6
            z0, z1 = (z - self.s.get("halves_mm", 0.0) for z in sim["top_z_mm"])   # band moves down as the layer rises
            zm, hz = (z0 + z1) / 2, max(1.0, (z1 - z0) / 2)
            for r in range(TACTILE_ROWS):
                z = TACTILE_ROWS * PAD_PITCH - PAD_PITCH * (r + 0.5)   # row 0 = TOP of the pad (as drawn on the page)
                e = 1 - ((z - zm) / hz) ** 2
                if e <= 0:
                    continue
                env = math.sqrt(e)                                  # elliptical along Z
                cr = c0 + 0.25 * shear * (r - 7.5) / 7.5            # torsional skew under shear
                row = []
                for c in range(TACTILE_COLS):
                    v = peak * env * math.exp(-0.5 * ((c - cr) / sig) ** 2)
                    if slip:                                        # smeared trail behind the motion
                        v += 0.3 * peak * env * math.exp(-0.5 * ((c - cr + 3 * self._slip_dir) / (2 * sig)) ** 2)
                    row.append(v)
                ghost = 0.02 * sum(row) / TACTILE_COLS * 8          # row crosstalk, like the real pad
                for c, v in enumerate(row):
                    i = r * TACTILE_COLS + c
                    f[i] += v * self._pad_gain[i] + ghost
        return bytes(max(0, min(255, int(v))) for v in f)

    def _loop(self):
        while not self._stop.is_set():
            now = time.time()
            with self._lock:
                angles = list(self._angles)
                try:
                    currents, tactile = self._step(angles, now)
                except Exception:          # a physics bug must not kill the D stream
                    import traceback
                    traceback.print_exc()
                    time.sleep(0.04)
                    continue
                sim = self._public()
                st = SimState(t=now, ok=True, angles=angles, current_ma=currents, tactile=tactile, sim=sim)
                self._state = st
            for fn in list(self.listeners):
                fn(st)
            time.sleep(0.04)
