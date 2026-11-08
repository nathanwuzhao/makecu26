"""Closed-loop motion primitives, run at 25 Hz on the PC with current (and later tactile)
feedback. The AI only chooses WHICH skill to run, never raw angles.

Grip force control (see config.py "Grip force from current"):
  grip()      close in steps until the holding current says "touching", then regulate()
  regulate()  nudge the commanded angle 1 deg at a time until holding current ~= target
  twist()     slides the top fingers while a GripHolder keeps the squeeze current on target

All skills block and raise Aborted when `self.abort` is set.
"""

import statistics
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field

import config
from config import Squeeze
from robot_serial import LIMP, TACTILE_COLS


@dataclass
class GripResult:
    contact: bool            # False = closed all the way without touching anything
    angle: int               # final commanded angle (after force regulation)
    contact_angle: int       # where contact was first detected
    hold_ma: float           # holding current at the end (median)
    peak_current_ma: float   # highest settled holding current while closing
    reason: str              # "current", "tactile", "limit"
    regulated: bool = False  # True if the current ended inside target +- band
    tactile: dict = field(default_factory=dict)


class Aborted(Exception):
    pass


class GripHolder:
    """Keeps a squeeze servo's holding current near target while other servos move."""

    def __init__(self, sk: "Skills", sq: Squeeze, target: float):
        self.sk, self.sq, self.target = sk, sq, target
        self.ticks = 0
        self.adjust = 0                # net degrees added (+ = tighter)
        self.readings: list[float] = []

    def __call__(self):
        self.ticks += 1
        if self.ticks % config.REGULATE_EVERY_TICKS:
            return
        m = self.sk.recent_current(self.sq, 3)
        if m != m:
            return
        self.readings.append(m)
        delta = self.sk.force_step(self.sq, m, self.target)
        if delta:
            self.sk.nudge(self.sq, delta)
            self.adjust += delta

    def summary(self) -> dict:
        r = self.readings or [float("nan")]
        return {"target_ma": self.target, "min_ma": round(min(r), 1), "max_ma": round(max(r), 1),
                "mean_ma": round(sum(r) / len(r), 1), "net_adjust_deg": self.adjust}


class Skills:
    def __init__(self, robot):
        self.robot = robot
        self.abort = threading.Event()
        self.tactile_baseline: bytes | None = None
        self._cmd: dict[int, int] = {}   # last commanded angle per servo
        self.last_contact: dict[int, int] = {}  # contact angle from the previous grip, per servo
        self.holding: dict[int, bool] = {}       # squeeze idx -> currently gripping something
        self.recent = deque(maxlen=50)   # last ~2 s of D-line states
        robot.listeners.append(self.recent.append)

    # ---------- low level ----------

    def _wait_state(self):
        t0 = time.time()
        while self.robot.state is None:
            if time.time() - t0 > 3:
                raise RuntimeError("no D lines from the robot (wrong port? firmware not running?)")
            time.sleep(0.05)
        return self.robot.state

    def _angle(self, idx: int) -> int:
        """Last commanded angle, else the firmware's reported target. LIMP (-1) = not driven,
        physical position unknown."""
        if idx in self._cmd:
            return self._cmd[idx]
        return self._wait_state().angles[idx]

    def is_limp(self, idx: int) -> bool:
        return self._angle(idx) == LIMP

    def _set(self, pose: dict[int, int]):
        self.robot.set(pose)
        self._cmd.update(pose)

    def _tick(self):
        if self.abort.is_set():
            raise Aborted()
        time.sleep(config.TICK_S)

    def _sleep(self, s: float):
        if self.abort.wait(s):
            raise Aborted()

    def move(self, targets: dict[int, int], deg_per_tick: float = config.MOVE_DEG_PER_TICK, hook=None):
        """Ramp several servos to targets together (SG90s jump violently otherwise).
        hook() is called once per tick (used to keep a grip regulated during the move).
        A LIMP servo's position is unknown, so it can't be ramped: it gets one direct jump to its
        target first. Make sure the first target for a limp servo is a safe angle."""
        limp = {i: a for i, a in targets.items() if self.is_limp(i)}
        if limp:
            print(f"[skills] servos {sorted(limp)} were limp (position unknown): jumping straight to {limp}")
            self._set(limp)
            self._sleep(config.SETTLE_S)
            targets = {i: a for i, a in targets.items() if i not in limp}
            if not targets:
                return
        start = {i: self._angle(i) for i in targets}
        steps = max(1, int(max(abs(targets[i] - start[i]) for i in targets) / deg_per_tick + 0.999))
        for s in range(1, steps + 1):
            self._set({i: round(start[i] + (targets[i] - start[i]) * s / steps) for i in targets})
            self._tick()
            if hook:
                hook()
        self._sleep(config.SETTLE_S)

    # ---------- current ----------

    def _fresh_states(self, n: int):
        """Wait for the next n D lines (each ~40 ms apart)."""
        out, last_t = [], self._wait_state().t
        deadline = time.time() + 3.0 + n * 0.1   # 3 s slack: a busy laptop (screen share, browser) can pause the sim thread
        while len(out) < n:
            if time.time() > deadline:
                raise RuntimeError("D stream stalled")
            st = self.robot.state
            if st.t != last_t:
                out.append(st)
                last_t = st.t
            time.sleep(0.005)
        return out

    def hold_current(self, sq: Squeeze, n: int = config.HOLD_SAMPLES, agg=min) -> float:
        """Current over the next n D lines. min() for contact detection (rejects one leftover
        move spike); statistics.median for force regulation."""
        vals = [st.current_ma[sq.current_ch] for st in self._fresh_states(n)]
        vals = [v for v in vals if v == v]
        return agg(vals) if vals else float("nan")

    def recent_current(self, sq: Squeeze, n: int) -> float:
        vals = [st.current_ma[sq.current_ch] for st in list(self.recent)[-n:]]
        vals = [v for v in vals if v == v]
        return statistics.median(vals) if vals else float("nan")

    def baseline(self, sq: Squeeze, angle: int) -> float:
        """Free-air holding current at this angle (from jog.py's profile), else 0."""
        table = config.BASELINE_MA.get(sq.idx)
        if not table:
            return 0.0
        nearest = min(table, key=lambda a: abs(a - angle))
        return table[nearest]

    def force_step(self, sq: Squeeze, current: float, target: float) -> int:
        """-1 / 0 / +1 degrees (in the closing direction) to move current toward target."""
        excess = current - self.baseline(sq, self._angle(sq.idx))
        if excess > config.GRIP_MAX_MA:
            return -2
        if excess < target - config.GRIP_BAND_MA:
            return 1
        if excess > target + config.GRIP_BAND_MA:
            return -1
        return 0

    def nudge(self, sq: Squeeze, delta: int):
        """Move delta degrees in the closing direction, never past open/closed limits."""
        if self.is_limp(sq.idx):
            raise RuntimeError(f"servo {sq.idx} is limp; move() it to a known angle before nudging")
        sign = 1 if sq.closed > sq.open else -1
        lo, hi = sorted((sq.open, sq.closed))
        self._set({sq.idx: max(lo, min(hi, self._angle(sq.idx) + sign * delta))})

    # ---------- tactile ----------

    def calibrate_tactile(self):
        """Call with nothing touching the sensor."""
        self.tactile_baseline = self._wait_state().tactile

    def tactile_features(self) -> dict:
        """Pressure summary vs. the no-touch baseline: total, touched cells, centroid (row, col).
        Logged every cycle; centroid shifting during a twist = the cap sliding on the pad."""
        st = self.robot.state
        if st is None or not st.ok or self.tactile_baseline is None:
            return {"ok": False}
        total = cells = r_sum = c_sum = 0
        for k, (v, b) in enumerate(zip(st.tactile, self.tactile_baseline)):
            d = v - b
            if d > config.TACTILE_DELTA:
                cells += 1
                total += d
                r_sum += d * (k // TACTILE_COLS)
                c_sum += d * (k % TACTILE_COLS)
        cen = [round(r_sum / total, 2), round(c_sum / total, 2)] if total else None
        return {"ok": True, "total": total, "cells": cells, "centroid": cen}

    def touched_cells(self) -> int:
        return self.tactile_features().get("cells", 0)

    # ---------- skills ----------

    def open(self, sq: Squeeze):
        self.move({sq.idx: sq.open})
        self.holding[sq.idx] = False

    def release(self, sq: Squeeze):
        """Back off just enough to let go (fingers aren't grippy, so a small gap is plenty).
        Next grip's fast approach then starts right there. Full open if no contact known."""
        if sq.idx not in self.last_contact:
            return self.open(sq)
        sign = 1 if sq.closed > sq.open else -1
        target = self.last_contact[sq.idx] - sign * config.RELEASE_MARGIN_DEG
        lo, hi = sorted((sq.open, sq.closed))
        self.move({sq.idx: max(lo, min(hi, target))})
        self.holding[sq.idx] = False

    def grip(self, sq: Squeeze, use_tactile: bool = config.USE_TACTILE, regulate: bool = True) -> GripResult:
        """Step closed; after each step let the move transient die out, then check the HOLDING
        current. Contact = it stays above baseline. Then regulate to the target current."""
        sign = 1 if sq.closed > sq.open else -1
        # fast approach: ramp to just short of where contact happened last time
        if sq.idx in self.last_contact:
            pre = self.last_contact[sq.idx] - sign * config.APPROACH_MARGIN_DEG
            if (pre - self._angle(sq.idx)) * sign > 0:
                self.move({sq.idx: pre})
        angle = self._angle(sq.idx)
        peak = 0.0
        while angle != sq.closed:
            angle += sign * min(config.SQUEEZE_STEP_DEG, abs(sq.closed - angle))
            self._set({sq.idx: angle})
            self._sleep(config.STEP_SETTLE_S)
            hold = self.hold_current(sq)
            if hold == hold:
                peak = max(peak, hold)
            reason = None
            if hold - self.baseline(sq, angle) > config.HOLD_CONTACT_MA:
                reason = "current"
            elif use_tactile and self.touched_cells() >= config.TACTILE_CELLS:
                reason = "tactile"
            if reason:
                self.last_contact[sq.idx] = angle
                self.holding[sq.idx] = True
                ok, hold = self.regulate(sq) if regulate else (False, hold)
                return GripResult(True, self._angle(sq.idx), angle, round(hold, 1), round(peak, 1),
                                  reason, ok, self.tactile_features())
        return GripResult(False, angle, angle, round(peak, 1), round(peak, 1), "limit", False,
                          self.tactile_features())

    def regulate(self, sq: Squeeze, target: float | None = None) -> tuple[bool, float]:
        """Adjust 1 deg at a time until holding current is within target +- band."""
        target = config.GRIP_TARGET_MA[sq.idx] if target is None else target
        m = float("nan")
        for _ in range(config.REGULATE_MAX_STEPS):
            self._sleep(config.STEP_SETTLE_S)
            m = self.hold_current(sq, 3, statistics.median)
            delta = self.force_step(sq, m, target)
            if delta == 0:
                return True, m
            self.nudge(sq, delta)
        return False, m

    def twist(self, fraction: float = 1.0, holder: GripHolder | None = None):
        """Slide the top fingers from start toward end (turns the cap counter-clockwise)."""
        fraction = min(1.0, max(0.1, fraction))
        self.move({s.idx: round(s.start + (s.end - s.start) * fraction) for s in config.TWIST_SLIDES},
                  config.TWIST_DEG_PER_TICK, hook=holder)

    def recenter(self):
        self.move({s.idx: s.start for s in config.TWIST_SLIDES})

    def home(self):
        """Everything unsqueezed, top fingers recentered, halves at cap height."""
        self.move({config.TOP_SQUEEZE.idx: config.TOP_SQUEEZE.open,
                   config.BOTTOM_SQUEEZE.idx: config.BOTTOM_SQUEEZE.open})
        self.move({**config.BOTTOM_FINGERS_HOME,
                   **{s.idx: s.start for s in config.TWIST_SLIDES},
                   config.HALVES.idx: config.HALVES.grip})

    def lift_to(self, angle: int):
        self.move({config.HALVES.idx: angle}, config.LIFT_DEG_PER_TICK)

    def lift_test_grip(self) -> GripResult:
        """Grip the cap and raise a little. Leaves it gripped + raised so a picture can be taken;
        call lift_test_finish() afterwards."""
        g = self.grip(config.TOP_SQUEEZE)
        if g.contact:
            holder = GripHolder(self, config.TOP_SQUEEZE, config.GRIP_TARGET_MA[config.TOP_SQUEEZE.idx])
            self.move({config.HALVES.idx: config.HALVES.test}, config.LIFT_DEG_PER_TICK, hook=holder)
        return g

    def lift_test_finish(self, cap_off: bool):
        if cap_off:
            self.lift_to(config.HALVES.full)  # the reveal: cap stays in the top fingers
            return
        self.lift_to(config.HALVES.grip)      # still threaded: put it back down and let go
        self.release(config.TOP_SQUEEZE)

    def twist_cycle(self, fraction: float = 1.0) -> dict:
        """grip cap (force-regulated) -> twist (grip held on target) -> release -> recenter."""
        t0 = time.time()
        sq = config.TOP_SQUEEZE
        g = self.grip(sq)
        if not g.contact:
            self.open(sq)
            return {"ok": False, "problem": "top fingers closed fully without touching the cap",
                    "grip": asdict(g), "duration_s": round(time.time() - t0, 2)}
        holder = GripHolder(self, sq, config.GRIP_TARGET_MA[sq.idx])
        self.twist(fraction, holder)
        tact_after = self.tactile_features()
        self.release(sq)
        self.recenter()
        shift = None
        if g.tactile.get("centroid") and tact_after.get("centroid"):
            shift = round(sum((a - b) ** 2 for a, b in zip(g.tactile["centroid"], tact_after["centroid"])) ** 0.5, 2)
        return {
            "ok": True,
            "grip": asdict(g),
            "twist_fraction": round(fraction, 2),
            "grip_during_twist": holder.summary(),   # min dropping far below target = slip
            "tactile_after_twist": tact_after,
            "tactile_centroid_shift": shift,         # cells; large = cap slid on the pad
            "duration_s": round(time.time() - t0, 2),
        }
