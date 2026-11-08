"""Mirror the sim's finger strokes onto the REAL finger slide servos.

    python src/main.py --sim pill_bottle --camera 1 --real-fingers          # opens the ESP32 too
    python src/main.py --sim pill_bottle --real-fingers --real-stroke 120   # shorter real stroke

  t = top    right FORWARD, left BACK   (servos 4 / 6)      y = top    right BACK, left FORWARD
  x = bottom right FORWARD, left BACK   (servos 1 / 3)      c = bottom right BACK, left FORWARD

Only finger slide servos are ever driven, and only during a stroke (same scheme as tools/record.py).
Each stroke re-engages where the last one left that pair (the first one engages at 90), ramps to
the target, then the pair goes limp again. On connect every servo is made limp and motion is locked
between strokes, so nothing else can move. E-stop on the page also limps them.
A servo whose calibration is missing, or whose firmware C/R differ from calibration.json, is never
driven (its half of the stroke is skipped and logged): its direction and end stops are unknown.
"""

import threading
import time

import config
import servo_cal
from robot_serial import LIMP, RobotSerial

PAIRS = {"top": (4, 6), "bottom": (1, 3)}      # (right, left)
KEYS = {("top", True): "t", ("top", False): "y", ("bottom", True): "x", ("bottom", False): "c"}


class RealFingers:
    def __init__(self, port: str | None = None, stroke_deg: float = 150, deg_per_tick: float = 3.0, log=None):
        self.log = log or (lambda kind, **d: print(kind, d))
        stroke_deg = max(10.0, min(170.0, stroke_deg))
        self.lo, self.hi = round(90 - stroke_deg / 2), round(90 + stroke_deg / 2)
        self.step = deg_per_tick
        self.pos: dict[str, dict[int, int]] = {k: {} for k in PAIRS}
        self._busy = threading.Lock()                       # one stroke at a time (one serial line)
        self.robot = RobotSerial(port).start()
        self.robot.motion_locked = True                     # locked except during a stroke
        problems = servo_cal.check(self.robot, config.CAL)
        wanted = {i for p in PAIRS.values() for i in p}
        self.bad = {i: p for p in problems for i in wanted if p.startswith(f"servo {i} ")}
        self.ok = sorted(wanted - set(self.bad))
        cfg = self.robot.query_servo_config()
        self.rev = {i: int(cfg[i][2]) for i in range(7)}
        self._limp_all()

    def summary(self) -> str:
        s = f"drives servos {self.ok} ({self.lo}-{self.hi} deg), limp between strokes"
        if self.bad:
            s += "; NOT driven (fix with src/calibrate.py): " + "; ".join(self.bad.values())
        return s

    def _limp_all(self):
        servo_cal.limp_all(self.robot, self.rev)              # no motion: every servo slack

    def stroke(self, layer: str, right_forward: bool) -> bool:
        """Start one stroke in the background. False if one is already running."""
        if not self._busy.acquire(blocking=False):
            return False
        threading.Thread(target=self._stroke, args=(layer, right_forward), daemon=True).start()
        return True

    def _stroke(self, layer: str, right_forward: bool):
        R, L = PAIRS[layer]
        key = KEYS[(layer, right_forward)]
        target = {R: self.hi, L: self.lo} if right_forward else {R: self.lo, L: self.hi}
        target = {i: a for i, a in target.items() if i in self.ok}   # uncalibrated servos never move
        start = {i: self.pos[layer].get(i, 90) for i in target}
        skipped = [i for i in (R, L) if i in self.bad]
        try:
            if not target:
                self.log("real_fingers", stroke=key, skipped_uncalibrated=skipped)
                return
            self.robot.motion_locked = False
            self.robot.set(start)                           # engage where the fingers were left
            time.sleep(0.25)
            n = max(1, int(max(abs(target[i] - start[i]) for i in start) / self.step + 0.999))
            for s in range(1, n + 1):
                self.robot.set({i: round(start[i] + (target[i] - start[i]) * s / n) for i in start})
                time.sleep(0.04)
            time.sleep(0.15)
            self.pos[layer].update(target)
            self.log("real_fingers", stroke=key, start=start, target=target,
                     **({"skipped_uncalibrated": skipped} if skipped else {}))
        except Exception as e:                              # incl. MotionLocked from an E-stop
            self.log("real_fingers", stroke=key, error=repr(e)[:200])
        finally:
            self.robot.motion_locked = True
            try:
                for i in target:
                    servo_cal._limp_then(self.robot, i, self.rev[i], self.rev[i])
            except Exception:
                pass
            self._busy.release()

    def estop(self):
        self.robot.motion_locked = True
        self._limp_all()

    def close(self):
        try:
            self.estop()
        finally:
            self.robot.close()

    def driven(self) -> list[int]:
        st = self.robot.state
        return [] if st is None else [i for i, a in enumerate(st.angles) if a != LIMP]
