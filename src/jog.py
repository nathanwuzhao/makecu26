"""Keyboard jog + working-position tool. Writes the "slots" in calibration.json.

Do `python src/calibrate.py` first: it sets each servo's end-of-travel so that 0 = HOME end
(open / back / down) and 180 = FAR end (closed / forward / up). On the real gripper jog.py only
moves calibrated servos, and z / x then go exactly to those safe ends.

    python src/jog.py            # real gripper (port auto-detected, opened without resetting)
    python src/jog.py --mock     # simulated gripper
    Also serves the digital twin at http://localhost:8000 (use it to check twin axes/signs).

Keys
  0-6        select servo                    a/d or <-/->   move -/+ one step
  Servos start LIMP ("off": not driven, position unknown). Engage one with g (its slot angle)
  or o (squeeze open angle): that first move is a direct jump. a/d and z/x refuse on a limp servo.
  w/s or ^/v step size up/down               z / x          ramp to 0 / 180 (HOME / FAR end)
  Tab / n    next calibration slot           g              go to the slot's saved angle
  Enter      save the current angle into the slot (written to calibration.json right away)
  o          open the selected squeeze servo (0 or 5): angle 0
  p          profile free-air holding current for the selected squeeze servo (NOTHING between fingers)
  t          grip test on the selected squeeze servo (object between fingers): contact + force regulation
  [ / ]      grip target current -/+ 10 mA for the selected squeeze servo
  r          toggle twist direction (if the twist tightens the cap)
  h          help                            q              quit
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import config  # noqa: E402
from config import CAL_FILE, SLOT_DEFS, Squeeze  # noqa: E402
from robot_serial import LIMP  # noqa: E402
from skills import Skills  # noqa: E402

NAMES = {0: "bottom squeeze", 1: "bottom finger R", 2: "halves up/down", 3: "bottom finger L",
         4: "top finger R", 5: "top squeeze", 6: "top finger L"}
SQUEEZE_CH = {0: 0, 5: 1}
STEPS = [1, 2, 5, 10, 30]


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
            if ch in ("\x00", "\xe0"):
                yield arrows.get(msvcrt.getwch(), "")
            else:
                yield ch
        else:
            yield None


def scripted_keys(script: str):
    for ch in script:
        yield ch
        for _ in range(5):
            yield None
    yield "q"


class Jog:
    def __init__(self, robot, mock: bool = False):
        self.robot = robot
        self.mock = mock
        self.sk = Skills(robot)
        self.cal = load_cal()
        self.cal.setdefault("slots", {})
        self.sel = 0
        self.step_i = 2
        self.slot_i = 0
        self.last_draw = 0.0

    # --- helpers ---
    def angle(self, idx=None) -> int:
        return self.sk._angle(self.sel if idx is None else idx)

    def slot_value(self, name: str, default: int) -> int:
        return int(self.cal["slots"].get(name, default))

    def squeeze(self, idx: int) -> Squeeze:
        return config.BOTTOM_SQUEEZE if idx == 0 else config.TOP_SQUEEZE

    def target(self, idx: int) -> float:
        return float(self.cal.get("grip_target_ma", {}).get(str(idx), config.GRIP_TARGET_MA[idx]))

    def say(self, msg: str):
        print("\r" + " " * 140 + "\r" + msg)

    def draw(self, force=False):
        if not force and time.time() - self.last_draw < 0.1:
            return
        self.last_draw = time.time()
        st = self.robot.state
        cur = "i0 --- i1 ---" if st is None else f"i0 {st.current_ma[0]:6.1f} i1 {st.current_ma[1]:6.1f}"
        name, sidx, dflt = SLOT_DEFS[self.slot_i]
        saved = self.cal["slots"].get(name)
        slot = f"slot {name} (servo {sidx}) = {saved if saved is not None else f'unset, default {dflt}'}"
        extra = f" target {self.target(self.sel):.0f}mA" if self.sel in SQUEEZE_CH else ""
        ang = self.angle()
        ang_s = "off" if ang == LIMP else f"{ang:3d}"
        line = (f"[{self.sel} {NAMES[self.sel]}] {ang_s} deg  step {STEPS[self.step_i]:2d} | "
                f"{cur}{extra} | {slot}")
        sys.stdout.write("\r" + line[:160].ljust(160))
        sys.stdout.flush()

    # --- actions ---
    def uncal_refuse(self, idx=None) -> bool:
        idx = self.sel if idx is None else idx
        if not self.mock and str(idx) not in self.cal.get("servos", {}):
            self.say(f"servo {idx} has no end-of-travel calibration: run `python src/calibrate.py {idx}` first")
            return True
        return False

    def limp_refuse(self) -> bool:
        if self.uncal_refuse():
            return True
        if self.sk.is_limp(self.sel):
            self.say(f"servo {self.sel} is limp (position unknown): press g to drive it to its slot angle first")
            return True
        return False

    def jog(self, delta: int):
        if self.limp_refuse():
            return
        a = max(0, min(180, self.angle() + delta))
        self.sk._set({self.sel: a})

    def ramp(self, a: int, engage: bool = False):
        """engage=True allows a limp servo (direct jump to a): only for g / o, never z / x,
        which would slam a limp servo straight into an end of its travel."""
        if self.uncal_refuse() or (not engage and self.limp_refuse()):
            return
        self.sk.move({self.sel: a}, config.MOVE_DEG_PER_TICK)

    def next_slot(self):
        self.slot_i = (self.slot_i + 1) % len(SLOT_DEFS)
        self.sel = SLOT_DEFS[self.slot_i][1]

    def save_slot(self):
        name, sidx, _ = SLOT_DEFS[self.slot_i]
        a = self.angle(sidx)
        if a == LIMP:
            self.say(f"servo {sidx} is limp: nothing to save. Drive it with g, then jog")
            return
        self.cal["slots"][name] = a
        save_cal(self.cal)
        self.say(f"saved {name} = {a}")
        self.next_slot()

    def profile(self):
        if self.sel not in SQUEEZE_CH:
            self.say("select a squeeze servo (0 or 5) first")
            return
        sq = self.squeeze(self.sel)
        self.say(f"profiling servo {sq.idx} free-air holding current {sq.open}->{sq.closed}; nothing between the fingers!")
        self.sk.move({sq.idx: sq.open})
        sign = 1 if sq.closed > sq.open else -1
        table = {}
        for a in range(sq.open, sq.closed + sign, sign * 3):
            self.sk._set({sq.idx: a})
            time.sleep(0.25)
            table[str(a)] = round(self.sk.hold_current(sq, 5, statistics.median), 1)
        self.sk.move({sq.idx: sq.open})
        self.cal.setdefault("baseline_ma", {})[str(sq.idx)] = table
        save_cal(self.cal)
        config.BASELINE_MA[sq.idx] = {int(k): v for k, v in table.items()}
        vals = list(table.values())
        self.say(f"baseline saved: min {min(vals)} median {statistics.median(vals)} max {max(vals)} mA")

    def grip_test(self):
        if self.sel not in SQUEEZE_CH:
            self.say("select a squeeze servo (0 or 5) first")
            return
        sq = self.squeeze(self.sel)
        config.GRIP_TARGET_MA[sq.idx] = self.target(sq.idx)
        self.sk.move({sq.idx: sq.open})
        g = self.sk.grip(sq)
        self.say(f"grip: contact={g.contact} at {g.contact_angle} -> {g.angle} deg, hold {g.hold_ma} mA "
                 f"(target {self.target(sq.idx):.0f}, in band={g.regulated}), peak {g.peak_current_ma}. "
                 f"'o' to open")

    def adjust_target(self, d: float):
        if self.sel not in SQUEEZE_CH:
            self.say("select a squeeze servo (0 or 5) first")
            return
        t = max(20.0, self.target(self.sel) + d)
        self.cal.setdefault("grip_target_ma", {})[str(self.sel)] = t
        save_cal(self.cal)
        self.say(f"grip target for servo {self.sel} = {t:.0f} mA")

    def run(self, keys):
        print(__doc__)
        print("Calibration file:", CAL_FILE)
        for key in keys:
            if key is None:
                self.draw()
                time.sleep(0.02)
                continue
            if key == "q":
                break
            try:
                if key in "0123456":
                    self.sel = int(key)
                elif key in ("a", "LEFT"):
                    self.jog(-STEPS[self.step_i])
                elif key in ("d", "RIGHT"):
                    self.jog(STEPS[self.step_i])
                elif key in ("w", "UP"):
                    self.step_i = min(len(STEPS) - 1, self.step_i + 1)
                elif key in ("s", "DOWN"):
                    self.step_i = max(0, self.step_i - 1)
                elif key == "z":
                    self.ramp(0)
                elif key == "x":
                    self.ramp(180)
                elif key in ("\t", "n"):
                    self.next_slot()
                elif key == "g":
                    name, sidx, dflt = SLOT_DEFS[self.slot_i]
                    self.sel = sidx
                    self.ramp(self.slot_value(name, dflt), engage=True)
                elif key in ("\r", "\n"):
                    self.save_slot()
                elif key == "o" and self.sel in SQUEEZE_CH:
                    self.ramp(self.squeeze(self.sel).open, engage=True)
                elif key == "p":
                    self.profile()
                elif key == "t":
                    self.grip_test()
                elif key == "[":
                    self.adjust_target(-10)
                elif key == "]":
                    self.adjust_target(10)
                elif key == "r":
                    self.cal["twist_reversed"] = 0 if self.cal.get("twist_reversed") else 1
                    save_cal(self.cal)
                    self.say(f"twist_reversed = {self.cal['twist_reversed']}")
                elif key == "h":
                    print("\n" + __doc__)
            except Exception as e:  # keep the tool alive
                self.say(f"error: {e!r}")
            self.draw(force=True)
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--port", default=None)
    ap.add_argument("--keys", help="scripted key presses instead of the keyboard (testing)")
    ap.add_argument("--no-twin", action="store_true", help="don't start the web digital twin (port 8000)")
    args = ap.parse_args()
    if args.mock:
        from dtwin_robot import DTwinRobot
        robot = DTwinRobot().start()
    else:
        from robot_serial import RobotSerial
        robot = RobotSerial(args.port).start()
    if not args.no_twin:
        from twin_server import Twin
        Twin(robot).start()   # watch the twin while jogging: http://localhost:8000
    try:
        Jog(robot, mock=args.mock).run(scripted_keys(args.keys) if args.keys else live_keys())
    finally:
        robot.close()


if __name__ == "__main__":
    main()
