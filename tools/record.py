"""READ-ONLY recorder: logs every D state (angles, currents, tactile) of the gripper to a .jsonl file.

    python tools/record.py --label squeeze_bottle               # real gripper (port auto), until q
    python tools/record.py --label squeeze_bottle --seconds 30 --rec-now  # record 30 s straight away
    python tools/record.py --mock --seconds 3 --label test      # DTwinRobot, no hardware
    python tools/record.py --mock --keys 1234 --label test      # scripted keys (testing)

ALL SERVOS LIMP, ALWAYS. Motion is locked on the PC the moment the port opens (set() raises, so no
S line can ever be sent). Before recording it checks that every servo reports -1 (limp); a servo still
driven by an earlier program is made limp (its R flag re-sent unchanged, which the firmware answers
by going slack - no motion). It refuses to record until all 7 read limp, and re-limps any servo that
reports driven during the recording. Squeeze / twist the object BY HAND against the pad.
Starts PAUSED (nothing written): r = start recording, s = stop (pause), any number of times per file.
Keys drop phase markers into the file:
    1 idle  2 light  3 hard  4 twist/slip  m free marker (asks for text)  q quit
Output: runs/recordings/<YYYYmmdd-HHMMSS>_<label>.jsonl, one JSON object per line:
    {"kind":"header", "t0", "label", "source", "servo_config"}          (first line)
    {"kind":"D", "t", "ok", "angles", "current_ma", "tactile": "<1024 hex chars>"}   (+ "sim" if any)
    {"kind":"marker", "t", "phase", "note"}
Replay it into the web twin: python src/main.py --replay <file>
"""

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

PHASES = {"1": "idle", "2": "light", "3": "hard", "4": "twist_slip"}
LIMP = -1
TOUCH = 30          # a tactile cell above this (0-255) counts as touched in the console line


def live_keys():
    import msvcrt
    while True:
        yield msvcrt.getwch() if msvcrt.kbhit() else None


def scripted_keys(script: str, ticks: int = 8):
    for ch in script:            # one key every `ticks` loop ticks (0.1 s each), then quit
        yield ch
        for _ in range(ticks):
            yield None
    while True:
        yield "q"


def _num(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(x, 2)


class Recorder:
    def __init__(self, path: Path, header: dict):
        self.f = path.open("w", encoding="utf-8")
        self.lock = threading.Lock()
        self.n = 0
        self.times = deque(maxlen=200)   # recent state times, for Hz
        self.last = None
        self.phase = "-"
        self.active = False              # r = start writing D lines, s = stop (pause)
        self._write({"kind": "header", **header})

    def set_active(self, on: bool):
        if on != self.active:
            self.active = on
            self._write({"kind": "marker", "t": round(time.time(), 4), "phase": "rec_start" if on else "rec_stop",
                         "note": ""})

    def _write(self, rec: dict):
        with self.lock:
            self.f.write(json.dumps(rec) + "\n")
            self.f.flush()

    def on_state(self, st):          # called from the robot's reader thread
        self.times.append(st.t)
        self.last = st                   # live console line keeps updating while paused
        if not self.active:
            return
        rec = {"kind": "D", "t": round(st.t, 4), "ok": st.ok, "angles": list(st.angles),
               "current_ma": [_num(c) for c in st.current_ma], "tactile": st.tactile.hex()}
        sim = getattr(st, "sim", None)
        if sim:
            rec["sim"] = sim
        self._write(rec)
        self.n += 1

    def marker(self, phase: str, note: str = ""):
        if phase in PHASES.values():          # notes / servo strokes don't change the current phase
            self.phase = phase
        self._write({"kind": "marker", "t": round(time.time(), 4), "phase": phase, "note": note})

    def hz(self) -> float:
        now = time.time()
        recent = [t for t in self.times if now - t < 1.0]
        return float(len(recent))

    def close(self):
        with self.lock:
            self.f.close()


def ensure_limp(robot, timeout: float = 5.0) -> bool:
    """True once all 7 servos report LIMP (-1). Each driven servo gets one limp command."""
    import servo_cal
    t0, sent = time.time(), set()
    while time.time() - t0 < timeout:
        st = robot.state
        if st is None:
            time.sleep(0.05)
            continue
        driven = [i for i, a in enumerate(st.angles) if a != LIMP]
        if not driven:
            return True
        for i in driven:
            if i not in sent:
                print(f"servo {i} is driven (angle {st.angles[i]}): making it limp")
                servo_cal.limp(robot, i)      # R re-sent with the same value -> firmware goes limp
                sent.add(i)
        time.sleep(0.2)
    return False


class TwistStroker:
    """--twist: keys t / y drive ONLY the top finger slides (servos 4 and 6) through one stroke, then
    make both limp again. Every other servo stays limp the whole time (you squeeze by hand).
      t = right finger FORWARD, left finger BACK   (one-way move)
      y = right finger BACK, left finger FORWARD   (one-way move)
    Each press re-engages where the previous move left the fingers (no jump); the first press engages at 90.
    The stroke stays inside the middle `stroke_deg` of the 0-180 range (servos 4 / 6 were calibrated at
    the jog-window limits, not measured ends). No current sensing on these servos, so each stroke is
    short and they are never left driven."""
    RIGHT, LEFT = 4, 6

    def __init__(self, robot, rec, stroke_deg: float, deg_per_tick: float = 3.0):
        import servo_cal
        self.robot, self.rec, self.servo_cal = robot, rec, servo_cal
        self.lo, self.hi = round(90 - stroke_deg / 2), round(90 + stroke_deg / 2)
        self.step = deg_per_tick
        cfg = robot.query_servo_config()               # read-only C/R query: current reverse flags
        self.rev = {i: int(cfg[i][2]) for i in (self.RIGHT, self.LEFT)}
        self.pos = None                                # last commanded {4: angle, 6: angle}

    def _ramp(self, a: dict, b: dict):
        n = max(1, int(max(abs(b[i] - a[i]) for i in a) / self.step + 0.999))
        for s in range(1, n + 1):
            self.robot.set({i: round(a[i] + (b[i] - a[i]) * s / n) for i in a})
            time.sleep(0.04)

    def _limp(self):
        for i in (self.RIGHT, self.LEFT):
            self.servo_cal._limp_then(self.robot, i, self.rev[i], self.rev[i])

    def stroke(self, name: str, right_forward: bool):
        """One-way move. 0 = back, 180 = forward (calibration), so forward = hi, back = lo."""
        R, L, lo, hi = self.RIGHT, self.LEFT, self.lo, self.hi
        target = {R: hi, L: lo} if right_forward else {R: lo, L: hi}
        start = self.pos or {R: 90, L: 90}
        self.rec.marker("servo_stroke", f"{name} start {start} -> {target}")
        self.robot.motion_locked = False
        try:
            self.robot.set(start)                      # engage where the fingers were left
            time.sleep(0.25)
            self._ramp(start, target)
            time.sleep(0.15)
            self.pos = target
        finally:
            self.robot.motion_locked = True            # nothing else can move
            self._limp()                               # both slides slack again
            self.rec.marker("servo_stroke", f"{name} end")


def status_line(rec: Recorder) -> str:
    st = rec.last
    if st is None:
        return f"waiting for D lines...  ({rec.n} states)"
    i0, i1 = (_num(c) for c in st.current_ma)
    tac = st.tactile
    touched = sum(1 for v in tac if v > TOUCH)
    return (f"i0 {i0!s:>7} mA  i1 {i1!s:>7} mA | tactile max {max(tac):3d} sum {sum(tac):6d} "
            f"touched {touched:3d} | {rec.hz():4.0f} Hz | {'REC   ' if rec.active else 'paused'} {rec.n} saved | phase {rec.phase:<10}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="rec")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until q)")
    ap.add_argument("--rec-now", action="store_true", help="start recording right away (default: paused until r)")
    ap.add_argument("--port", default=None, help="default: auto-detect ESP32 (VID 303A)")
    ap.add_argument("--mock", action="store_true", help="record DTwinRobot (no hardware)")
    ap.add_argument("--keys", help="scripted key presses, e.g. 12345 (testing)")
    ap.add_argument("--out", type=Path, default=ROOT / "runs" / "recordings")
    ap.add_argument("--twist", action="store_true",
                    help="enable keys t / y: top finger slides (servos 4, 6) do one twist stroke, then go limp")
    ap.add_argument("--stroke", type=float, default=90, help="twist stroke in degrees, centred on 90 (max 150)")
    args = ap.parse_args()
    args.stroke = max(10.0, min(150.0, args.stroke))

    if args.mock:
        from dtwin_robot import DTwinRobot
        robot = DTwinRobot().start()
        cfg = {str(i): list(v) for i, v in robot.query_servo_config().items()}
    else:
        from robot_serial import RobotSerial
        robot = RobotSerial(args.port).start()
        try:                                       # calibration from the file, NOT a firmware query
            import config
            cfg = {i: [c["min_us"], c["max_us"], c["reversed"]] for i, c in config.CAL["servos"].items()}
        except Exception:
            cfg = None

    robot.motion_locked = True                     # set() raises from now on: no S line can be sent
    if not ensure_limp(robot):
        robot.close()
        sys.exit("REFUSING to record: not every servo reports limp. Power-cycle the ESP32 and retry.")
    print("all 7 servos limp, motion locked. Squeeze by hand.")

    args.out.mkdir(parents=True, exist_ok=True)
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label)
    path = args.out / f"{time.strftime('%Y%m%d-%H%M%S')}_{safe}.jsonl"
    rec = Recorder(path, {"t0": round(time.time(), 4), "label": args.label,
                          "source": "mock" if args.mock else "serial", "servo_config": cfg})
    robot.listeners.append(rec.on_state)
    print(f"recording -> {path}")
    print("keys: r start recording  s stop (pause)  |  1 idle  2 light  3 hard  4 twist/slip  m marker  |  q quit")
    rec.set_active(args.rec_now)
    print("RECORDING" if rec.active else "paused: press r to start recording")
    stroker = None
    if args.twist:
        stroker = TwistStroker(robot, rec, args.stroke)
        print(f"     t = right finger forward + left back      y = right finger back + left forward")
        print(f"     ONLY servos 4 + 6 move, {stroker.lo}-{stroker.hi} deg, then go limp. Keep fingers clear of the racks.")

    keys = scripted_keys(args.keys) if args.keys else live_keys()
    t0 = time.time()
    stroke_end = 0.0
    try:
        for k in keys:
            if args.seconds and time.time() - t0 >= args.seconds:
                break
            if k in ("q", "Q", "\x1b"):
                break
            if k in ("r", "R") and not rec.active:
                rec.set_active(True)
                print("\n[REC] recording")
            elif k in ("s", "S") and rec.active:
                rec.set_active(False)
                print(f"\n[stop] paused ({rec.n} states saved so far)")
            if k in PHASES:
                rec.marker(PHASES[k])
                print(f"\n[marker] {PHASES[k]}")
            elif stroker and k in ("t", "T", "y", "Y"):
                fwd = k in ("t", "T")
                print(f"\n[servos 4+6] {'right forward, left back' if fwd else 'right back, left forward'}, then limp")
                stroker.stroke("R_fwd_L_back" if fwd else "R_back_L_fwd", right_forward=fwd)
                stroke_end = time.time()
            elif k in ("m", "M"):
                note = "marker" if args.keys else input("\nmarker text: ").strip() or "marker"
                rec.marker("note", note)
                print(f"\n[marker] {note}")
            st = rec.last
            settled = time.time() - stroke_end > 1.0               # D lines lag a stroke's limp by a frame
            if settled and st is not None and any(a != LIMP for a in st.angles):   # something engaged a servo
                print("\n!! a servo reports driven: making all limp again")
                ensure_limp(robot)
            sys.stdout.write("\r" + status_line(rec))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if stroker:
            robot.motion_locked = True
            try:
                stroker._limp()                    # whatever happened: top slides slack before closing
            except Exception:
                pass
        robot.listeners.remove(rec.on_state)
        robot.close()
        rec.close()
    print(f"\nsaved {rec.n} states in {time.time() - t0:.1f} s -> {path}")


if __name__ == "__main__":
    main()
