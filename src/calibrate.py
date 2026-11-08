"""Servo end-of-travel calibration wizard. One servo at a time, you jog the raw PWM pulse and mark
both physical ends; it then writes the firmware's C (pulse limits) + R (reverse) and
calibration.json so that, for EVERY servo, 0 deg = HOME end and 180 deg = FAR end (servo_cal.py).

    python src/calibrate.py            # every servo not calibrated yet (all of them the first time)
    python src/calibrate.py 6 4        # just these servos (re-calibrate)
    python src/calibrate.py --mock     # practice on the simulated gripper

Per servo:
  1. It starts LIMP. Push the finger by hand to roughly the MIDDLE of its travel.
  2. Enter: engages it at the middle pulse (1450 us). It may twitch to get there.
  3. Jog:  a / d  (or <- / ->)  -/+ 1 step (~11 us, ~0.15 mm)      A / D  -/+ 5 steps
           SPACE = go limp NOW (Enter re-engages where it was)
  4. Jog to the HOME end, press h. Jog to the FAR end, press f. Stop the moment it reaches the
     end: if it hums/buzzes it is stalling, back off right away (a stall burns the servo).
  5. Enter: saves (both ends pulled ~3 deg inside), writes C + R, servo goes limp.
  6. Check: Enter engages at 90, then [ / ] ramps slowly to 0 / 180. Both should reach their end
     WITHOUT straining. y = good, next servo.   r = redo this servo.
  s = skip servo, q = quit. Quitting part-way restores this servo's previous calibration.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import servo_cal  # noqa: E402
from config import CAL_FILE  # noqa: E402
from robot_serial import LIMP  # noqa: E402
from servo_cal import SERVOS, raw_angle_to_us  # noqa: E402

CURRENT_CH = {0: 0, 5: 1}     # squeeze servos have a current sensor
# Sustained above STALL_MA for STALL_LINES D lines (~40 ms each) on a squeeze servo = stalling ->
# auto-limp. MG90S moving a loaded squeeze draws ~600-700 mA for a few hundred ms, so 600 mA /
# 3 lines (120 ms) tripped on normal moves. A real stall at ~800 mA for 0.5 s does no harm.
STALL_MA = float(os.getenv("CAL_STALL_MA", "800"))
STALL_LINES = int(os.getenv("CAL_STALL_LINES", "12"))


def load_cal() -> dict:
    try:
        return json.loads(CAL_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_cal(cal: dict):
    CAL_FILE.write_text(json.dumps(cal, indent=1, sort_keys=True))


def live_keys():
    import msvcrt
    arrows = {"K": "LEFT", "M": "RIGHT", "H": "UP", "P": "DOWN"}
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            yield arrows.get(msvcrt.getwch(), "") if ch in ("\x00", "\xe0") else ch
        else:
            yield None


def scripted_keys(script: str):
    for ch in script:
        yield ch
        for _ in range(8):
            yield None
    while True:
        yield "q"


class Quit(Exception):
    pass


class Wizard:
    def __init__(self, robot, keys):
        self.robot = robot
        self.keys = keys
        self.cal = load_cal()
        self.cal.setdefault("servos", {})
        self.last_draw = 0.0

    # ---------- io ----------
    def say(self, msg=""):
        print("\r" + " " * 150 + "\r" + msg)

    def key(self, status=None):
        """Next key press; redraws `status` and runs the stall guard while waiting."""
        for k in self.keys:
            if k is not None:
                return k
            if status:
                self.stall_guard()
                if time.time() - self.last_draw > 0.1:
                    self.last_draw = time.time()
                    sys.stdout.write("\r" + status()[:150].ljust(150))
                    sys.stdout.flush()
            time.sleep(0.02)
        raise Quit()

    # ---------- servo state ----------
    def angle(self) -> int:
        st = self.robot.state
        return LIMP if st is None else st.angles[self.idx]

    def drive(self, a: int):
        self.robot.set({self.idx: max(0, min(180, a))})
        self.cmd = max(0, min(180, a))
        time.sleep(0.05)

    def ramp(self, a: int, deg_per_step: int = 2):
        cur = self.cmd if self.cmd is not None else a
        while cur != a:
            cur += max(-deg_per_step, min(deg_per_step, a - cur))
            self.drive(cur)
            time.sleep(0.03)

    def go_limp(self):
        servo_cal.limp(self.robot, self.idx)

    _stall = 0

    def stall_guard(self):
        ch = CURRENT_CH.get(self.idx)
        st = self.robot.state
        if ch is None or st is None or st.angles[self.idx] == LIMP:
            self._stall = 0
            return
        ma = st.current_ma[ch]
        self._stall = self._stall + 1 if ma == ma and ma > STALL_MA else 0
        if self._stall >= STALL_LINES * 4:   # ~4 checks per D line
            self.go_limp()
            self._stall = 0
            self.say(f"!! STALL: {ma:.0f} mA sustained. Servo {self.idx} made limp. Back off from that end.")

    def current_str(self) -> str:
        ch = CURRENT_CH.get(self.idx)
        st = self.robot.state
        if ch is None or st is None:
            return "no current sensor: watch/listen for straining"
        return f"i{ch} {st.current_ma[ch]:6.1f} mA"

    # ---------- one servo ----------
    def calibrate(self, idx: int) -> bool:
        """True = saved + checked, False = skipped."""
        self.idx, self.cmd = idx, None
        name, home, far, warn = SERVOS[idx]
        self.say()
        self.say(f"=== Servo {idx}: {name} ===")
        self.say(f"    HOME end (will become 0 deg):  {home}")
        self.say(f"    FAR end  (will become 180 deg): {far}")
        if warn:
            self.say(f"    !! {warn}")
        self.say("Making it limp + raw (500-2400 us, not reversed)...")
        servo_cal.to_raw(self.robot, idx)
        self.say("It's LIMP. Push it by hand to roughly the MIDDLE of its travel, then press Enter "
                 "(engages at 1450 us).   s = skip   q = quit")
        while True:
            k = self.key()
            if k in ("\r", "\n"):
                break
            if k == "s":
                servo_cal.restore(self.robot, self.cal, idx)
                return False
            if k == "q":
                raise Quit()
        self.drive(90)

        marks = {"home": None, "far": None}

        def status():
            a = self.angle()
            pos = "LIMP (Enter = re-engage)" if a == LIMP else f"{raw_angle_to_us(a)} us ({a:3d})"
            mh = f"{marks['home']} us" if marks["home"] else "--"
            mf = f"{marks['far']} us" if marks["far"] else "--"
            return f"[{idx} {name}] pulse {pos} | {self.current_str()} | home(h) {mh}  far(f) {mf} | Enter=save"

        self.say("Jog: a/d = 1 step, A/D = 5 steps, SPACE = limp. h = mark HOME end, f = mark FAR end.")
        while True:
            k = self.key(status)
            limp = self.angle() == LIMP
            if k in ("a", "LEFT", "d", "RIGHT", "A", "D"):
                if limp:
                    self.say("limp: press Enter to re-engage first")
                    continue
                step = {"a": -1, "LEFT": -1, "d": 1, "RIGHT": 1, "A": -5, "D": 5}[k]
                self.drive(self.cmd + step)
            elif k == " ":
                self.go_limp()
                self.say("limp")
            elif k in ("h", "f"):
                if limp:
                    self.say("limp: position unknown, re-engage (Enter) before marking")
                    continue
                marks["home" if k == "h" else "far"] = raw_angle_to_us(self.cmd)
                self.say(f"marked {'HOME' if k == 'h' else 'FAR'} end at {raw_angle_to_us(self.cmd)} us")
            elif k in ("\r", "\n"):
                if limp:
                    self.drive(self.cmd if self.cmd is not None else 90)
                    continue
                if not (marks["home"] and marks["far"]):
                    self.say("mark both ends first (h and f)")
                    continue
                try:
                    c = servo_cal.compute(marks["home"], marks["far"])
                except ValueError as e:
                    self.say(f"can't save: {e}")
                    continue
                break
            elif k == "s":
                servo_cal.restore(self.robot, self.cal, idx)
                return False
            elif k == "q":
                raise Quit()

        servo_cal.apply(self.robot, idx, c["min_us"], c["max_us"], c["reversed"])
        self.cal["servos"][str(idx)] = c
        save_cal(self.cal)
        self.say(f"saved: HOME {c['home_us']} us = 0 deg, FAR {c['far_us']} us = 180 deg "
                 f"(C {c['min_us']} {c['max_us']}, R {c['reversed']}). Servo is limp.")
        return self.check(idx)

    def check(self, idx: int) -> bool:
        self.cmd = None
        self.say("Check: Enter = engage at 90 (middle), then [ = ramp to 0 (HOME), ] = ramp to 180 (FAR).")
        self.say("       y = both ends right, next servo.  r = redo.  SPACE = limp.")

        def status():
            a = self.angle()
            return f"[{idx} {SERVOS[idx][0]}] {'LIMP' if a == LIMP else f'{a:3d} deg'} | {self.current_str()}"

        while True:
            k = self.key(status)
            if k in ("\r", "\n"):
                if self.angle() == LIMP:
                    self.drive(90)
            elif k in ("[", "]"):
                if self.angle() == LIMP:
                    self.say("press Enter to engage first")
                    continue
                self.ramp(0 if k == "[" else 180)
            elif k == " ":
                self.go_limp()
            elif k == "y":
                self.go_limp()
                return True
            elif k == "r":
                return self.calibrate(idx)
            elif k == "q":
                raise Quit()

    def run(self, servos: list[int]):
        print(__doc__)
        done = []
        try:
            for i in servos:
                if self.calibrate(i):
                    done.append(i)
        except (Quit, KeyboardInterrupt):
            idx = getattr(self, "idx", None)
            if idx is not None and str(idx) not in [str(d) for d in done]:
                self.say(f"\nquit: restoring servo {idx}'s previous calibration (limp)")
                try:
                    servo_cal.restore(self.robot, self.cal, idx)
                except Exception as e:
                    self.say(f"restore failed ({e}); main.py will refuse until it's recalibrated")
        left = [i for i in range(7) if str(i) not in self.cal["servos"]]
        self.say(f"\ncalibrated this session: {done or 'none'}. Still uncalibrated: {left or 'none'}")
        if not left:
            self.say("All servos done. Next: python src/jog.py to set the working-position slots.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("servos", nargs="*", type=int, help="servo indices (default: all uncalibrated)")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--port", default=None)
    ap.add_argument("--keys", help="scripted key presses (testing)")
    args = ap.parse_args()
    cal = load_cal()
    servos = args.servos or [i for i in range(7) if str(i) not in cal.get("servos", {})] or list(range(7))
    if args.mock:
        from dtwin_robot import DTwinRobot
        robot = DTwinRobot().start()
    else:
        from robot_serial import RobotSerial
        robot = RobotSerial(args.port).start()
    try:
        Wizard(robot, scripted_keys(args.keys) if args.keys else live_keys()).run(servos)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
