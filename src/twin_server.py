"""Digital-twin server: serves web/ and streams live robot state + events to the browser.

    python src/twin_server.py --demo     # no hardware: fake gripper sweeps every joint (check axes)
    python src/twin_server.py --mock     # fake gripper, idle
    python src/twin_server.py            # real gripper, read-only view (no other program may hold the port)
    then open http://localhost:8000

main.py and jog.py start this themselves (they own the serial port), so normally you just open
the page while they run.

Endpoints
  /            viewer (web/index.html)
  /state       Server-Sent Events: 'state' (every D line, 25 Hz) and 'log' (agent/cycle events)
  /cam.mjpg    webcam as MJPEG, if a Camera was passed in
  /api/status                      GET: controller state (404 without a controller)
  /api/start /api/stop /api/estop /api/reset   POST: run control (start body: {"gemini", "cycles"})
  /api/calibration                 GET: calibration.json (the twin derives travel per degree from it)
"""

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

WEB = Path(__file__).parent.parent / "web"


def _num(x):
    return None if isinstance(x, float) and math.isnan(x) else x


class Twin:
    def __init__(self, robot, camera=None, host: str = "127.0.0.1", port: int = 8000, controller=None):
        self.camera = camera
        self.controller = controller
        self.host, self.port = host, port
        self.cond = threading.Condition()
        self.state_json: str | None = None
        self.state_seq = 0
        self.events: deque[tuple[int, str]] = deque(maxlen=300)
        self.event_seq = 0
        robot.listeners.append(self._on_state)

    def _on_state(self, st):
        msg = json.dumps({"t": st.t, "ok": st.ok, "angles": st.angles,
                          "current_ma": [_num(c) for c in st.current_ma], "tactile": st.tactile.hex()})
        with self.cond:
            self.state_json = msg
            self.state_seq += 1
            self.cond.notify_all()

    def publish(self, kind: str, data: dict):
        """Forward an event (decision, lift test, cycle...) to the page's log panel."""
        msg = json.dumps({"t": time.time(), "kind": kind, **data}, default=str)
        with self.cond:
            self.event_seq += 1
            self.events.append((self.event_seq, msg))
            self.cond.notify_all()

    def start(self) -> "Twin":
        handler = partial(_Handler, self, directory=str(WEB))
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"twin: http://localhost:{self.port}")
        return self

    def close(self):
        self.httpd.shutdown()


class _Handler(SimpleHTTPRequestHandler):
    def __init__(self, twin: Twin, *args, **kwargs):
        self.twin = twin
        super().__init__(*args, **kwargs)

    def log_message(self, *args):  # quiet
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json(self, code: int, obj):
        body = json.dumps(obj, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        ctl = self.twin.controller
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            body = {}
        if ctl is None:
            return self._json(404, {"ok": False, "msg": "no controller (start main.py, not twin_server.py)"})
        action = self.path.split("?")[0].rstrip("/")
        if action == "/api/estop":                  # first: never behind parsing/validation
            ok, msg = ctl.estop()
        elif action == "/api/stop":
            ok, msg = ctl.stop()
        elif action == "/api/reset":
            ok, msg = ctl.reset()
        elif action == "/api/start":
            try:
                cycles = max(1, min(100, int(body.get("cycles", 20))))
            except (TypeError, ValueError):
                cycles = 20
            ok, msg = ctl.start(gemini=bool(body.get("gemini")), cycles=cycles)
        else:
            return self._json(404, {"ok": False, "msg": "unknown action"})
        self._json(200 if ok else 409, {"ok": ok, "msg": msg, "status": ctl.status()})

    def do_GET(self):
        if self.path.startswith("/api/status"):
            ctl = self.twin.controller
            return self._json(200, ctl.status()) if ctl else self._json(404, {"state": "no controller"})
        if self.path.startswith("/api/calibration"):
            import config
            try:
                return self._json(200, json.loads(config.CAL_FILE.read_text()))
            except (FileNotFoundError, json.JSONDecodeError):
                return self._json(200, {})
        if self.path.startswith("/state"):
            return self._sse()
        if self.path.startswith("/cam.mjpg"):
            return self._mjpeg()
        return super().do_GET()

    def _sse(self):
        tw = self.twin
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()
        with tw.cond:  # replay recent events so a refreshed page still shows the log
            last_ev = 0
            backlog = list(tw.events)
        last_state = -1
        try:
            for seq, msg in backlog:
                self.wfile.write(f"event: log\ndata: {msg}\n\n".encode())
                last_ev = seq
            while True:
                with tw.cond:
                    tw.cond.wait_for(lambda: tw.state_seq != last_state or tw.event_seq != last_ev, timeout=2)
                    new_events = [(s, m) for s, m in tw.events if s > last_ev]
                    state, last_state = tw.state_json, tw.state_seq
                out = b""
                for s, m in new_events:
                    out += f"event: log\ndata: {m}\n\n".encode()
                    last_ev = s
                out += f"event: state\ndata: {state}\n\n".encode() if state else b": ping\n\n"
                self.wfile.write(out)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _mjpeg(self):
        cam = self.twin.camera
        if cam is None:
            self.send_error(404, "no camera")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpg = cam.jpeg(max_width=640, quality=70)
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpg)}\r\n\r\n".encode() + jpg + b"\r\n")
                time.sleep(1 / 15)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass


def _demo_sweep(robot, stop: threading.Event):
    """Move each servo 0 -> 180 -> 0 in turn, so every twin axis/sign can be checked by eye."""
    while not stop.is_set():
        for idx in range(7):
            for a in list(range(0, 181, 4)) + list(range(180, -1, -4)):
                if stop.is_set():
                    return
                robot.set({idx: a})
                time.sleep(0.04)
            time.sleep(0.3)


def main():
    sys.path.insert(0, str(Path(__file__).parent))
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="fake gripper")
    ap.add_argument("--demo", action="store_true", help="fake gripper that sweeps every joint")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other devices on the network")
    ap.add_argument("--serial", default=None, help="ESP32 port (default: auto)")
    ap.add_argument("--camera", type=int, default=None, help="webcam index to stream")
    args = ap.parse_args()

    if args.mock or args.demo:
        from fake_robot import FakeRobot
        robot = FakeRobot().start()
    else:
        from robot_serial import RobotSerial
        robot = RobotSerial(args.serial).start()
    cam = None
    if args.camera is not None:
        from camera import Camera
        cam = Camera(args.camera).start()

    Twin(robot, cam, args.host, args.port).start()
    stop = threading.Event()
    if args.demo:
        threading.Thread(target=_demo_sweep, args=(robot, stop), daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()
        robot.close()


if __name__ == "__main__":
    main()
