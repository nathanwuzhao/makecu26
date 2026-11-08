"""Mirror the sim's t / y twist strokes onto the REAL top finger servos (4 = right, 6 = left).

    python src/main.py --sim pill_bottle --camera 1 --real-fingers          # opens the ESP32 too
    python src/main.py --sim pill_bottle --real-fingers --real-stroke 120   # shorter real stroke

Only servos 4 and 6 are ever driven, and only during a stroke (same scheme as tools/record.py):
  t = right finger FORWARD, left finger BACK      y = right BACK, left FORWARD
Each stroke re-engages where the last one left the fingers (the first one engages at 90), ramps
to the target, then both go limp again. On connect every servo is made limp and motion is locked
between strokes, so nothing else can move. E-stop on the page also limps these.
Refuses to start if servo 4 or 6 isn't calibrated or the firmware's C/R differ from calibration.json.
"""

import threading
import time

import config
import servo_cal
from robot_serial import LIMP, RobotSerial

RIGHT, LEFT = 4, 6


class RealFingers:
    def __init__(self, port: str | None = None, stroke_deg: float = 150, deg_per_tick: float = 3.0, log=None):
        self.log = log or (lambda kind, **d: print(kind, d))
        stroke_deg = max(10.0, min(170.0, stroke_deg))
        self.lo, self.hi = round(90 - stroke_deg / 2), round(90 + stroke_deg / 2)
        self.step = deg_per_tick
        self.pos: dict[int, int] | None = None
        self._busy = threading.Lock()
        self.robot = RobotSerial(port).start()
        self.robot.motion_locked = True                       # locked except during a stroke
        problems = [p for p in servo_cal.check(self.robot, config.CAL) if p.startswith((f"servo {RIGHT} ", f"servo {LEFT} "))]
        if problems:
            self.robot.close()
            raise RuntimeError("real fingers: " + "; ".join(problems))
        cfg = self.robot.query_servo_config()
        self.rev = {i: int(cfg[i][2]) for i in range(7)}
        self._limp_all()

    def _limp_all(self):
        servo_cal.limp_all(self.robot, self.rev)                # no motion: every servo slack

    def _limp_fingers(self):
        for i in (RIGHT, LEFT):
            servo_cal._limp_then(self.robot, i, self.rev[i], self.rev[i])

    def stroke(self, right_forward: bool) -> bool:
        """Start one stroke in the background. False if one is already running."""
        if not self._busy.acquire(blocking=False):
            return False
        threading.Thread(target=self._stroke, args=(right_forward,), daemon=True).start()
        return True

    def _stroke(self, right_forward: bool):
        target = {RIGHT: self.hi, LEFT: self.lo} if right_forward else {RIGHT: self.lo, LEFT: self.hi}
        start = self.pos or {RIGHT: 90, LEFT: 90}
        try:
            self.robot.motion_locked = False
            self.robot.set(start)                             # engage where the fingers were left
            time.sleep(0.25)
            n = max(1, int(max(abs(target[i] - start[i]) for i in start) / self.step + 0.999))
            for s in range(1, n + 1):
                self.robot.set({i: round(start[i] + (target[i] - start[i]) * s / n) for i in start})
                time.sleep(0.04)
            time.sleep(0.15)
            self.pos = target
            self.log("real_fingers", stroke="t" if right_forward else "y", start=start, target=target)
        except Exception as e:                                # incl. MotionLocked from an E-stop
            self.log("real_fingers", stroke="t" if right_forward else "y", error=repr(e)[:200])
        finally:
            self.robot.motion_locked = True
            try:
                self._limp_fingers()
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
