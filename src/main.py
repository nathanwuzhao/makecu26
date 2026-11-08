"""Unscrew-the-cap demo: connects to the gripper, starts the web twin, and waits for Start.

    python src/main.py                       # real gripper + webcam; open http://localhost:8000, press Start
    python src/main.py --mock                # fake gripper (no hardware)
    python src/main.py --mock --no-camera --autostart --cycles 3   # CLI: run once and exit

The page has Start (optionally with Gemini), Stop (graceful) and E-STOP (Esc key too: motion
locked + every servo limp). The loop itself lives in controller.py.

Gemini is OFF unless ticked on the page (or --gemini with --autostart): free-tier quota.
Each cycle's before/after image + telemetry is saved to runs/<timestamp>/ (src/replay.py).
Ctrl+C = graceful stop, then exit.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).parent))
load_dotenv(Path(__file__).parent.parent / ".env")

import config  # noqa: E402  (after load_dotenv so env overrides apply)

RUNS = Path(__file__).parent.parent / "runs"


class EventLog:
    def __init__(self):
        self.dir = RUNS / time.strftime("%Y%m%d-%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / "events.jsonl"
        self.f = self.path.open("a", encoding="utf-8")
        self.sinks = []   # e.g. Twin.publish: forwards every event to the web UI

    def save_cycle(self, cycle: int, image: bytes | None, summary: dict, name: str | None = None):
        d = self.dir / (name or f"cycle{cycle:02d}")
        d.mkdir(exist_ok=True)
        if image:
            (d / "before_after.jpg").write_bytes(image)
        (d / "summary.json").write_text(json.dumps(summary, indent=1))

    def __call__(self, kind: str, **data):
        rec = {"t": round(time.time(), 3), "kind": kind, **data}
        self.f.write(json.dumps(rec, default=str) + "\n")
        self.f.flush()
        brief = {k: v for k, v in data.items() if k not in ("summary",)}
        print(f"[{kind}] {json.dumps(brief, default=str)[:300]}")
        for sink in self.sinks:
            sink(kind, data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="use FakeRobot instead of serial")
    ap.add_argument("--port", default=os.getenv("SERIAL_PORT") or None, help="default: auto-detect ESP32 (VID 303A)")
    ap.add_argument("--camera", type=int, default=int(os.getenv("CAMERA_INDEX", "0")))
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--gemini", action="store_true", help="with --autostart: use Gemini (costs free-tier quota)")
    ap.add_argument("--autostart", action="store_true", help="run once right away and exit (no Start button)")
    ap.add_argument("--cycles", type=int, default=config.MAX_CYCLES)
    ap.add_argument("--no-twin", action="store_true", help="don't start the web digital twin")
    ap.add_argument("--twin-port", type=int, default=8000)
    args = ap.parse_args()

    if not args.mock:
        problems = config.calibration_problems()
        if problems:
            # Checked before the port is opened, so nothing is sent to the gripper.
            print(f"REFUSING to run the real gripper: calibration incomplete ({config.CAL_FILE}):\n  "
                  + "\n  ".join(problems) +
                  "\nUncalibrated servos can be driven into (or past) their end stops and burn out.\n"
                  "Run `python src/calibrate.py`, then `python src/jog.py` for the slots. Or use --mock.")
            sys.exit(2)

    log = EventLog()

    if args.mock:
        from fake_robot import FakeRobot
        robot = FakeRobot().start()
    else:
        import servo_cal
        from robot_serial import RobotSerial
        robot = RobotSerial(args.port).start()
        # Firmware must hold exactly the calibrated C/R (a reflash/erase or an abandoned
        # calibration changes them). Queries only: nothing moves.
        problems = servo_cal.check(robot, config.CAL)
        if problems:
            robot.close()
            print("REFUSING to run: firmware servo settings don't match calibration.json:\n  "
                  + "\n  ".join(problems) + "\nRe-run `python src/calibrate.py <servo>` for those servos.")
            sys.exit(2)

    cam = None
    if not args.no_camera:
        from camera import Camera
        cam = Camera(args.camera).start()

    if args.mock:
        reversed_flags = {i: v[2] for i, v in robot.query_servo_config().items()}
    else:   # verified equal to the firmware by servo_cal.check above
        reversed_flags = {int(i): c["reversed"] for i, c in config.CAL["servos"].items()}

    from controller import Controller
    ctl = Controller(robot, cam, log, reversed_flags)
    if not args.no_twin:
        from twin_server import Twin
        twin = Twin(robot, cam, port=args.twin_port, controller=ctl).start()
        log.sinks.append(twin.publish)

    try:
        if args.autostart or args.no_twin:
            ok, msg = ctl.start(gemini=args.gemini, cycles=args.cycles)
            if not ok:
                print(f"can't start: {msg}")
            ctl.wait()
        else:
            print(f"Ready. Open http://localhost:{args.twin_port} and press Start. Ctrl+C to quit.")
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        if ctl.state == "running":
            ctl.stop()
            t0 = time.time()
            while ctl.state != "idle" and time.time() - t0 < 8:
                time.sleep(0.1)
    finally:
        if cam:
            cam.close()
        robot.close()
        print(f"log: {log.path}")


if __name__ == "__main__":
    main()
