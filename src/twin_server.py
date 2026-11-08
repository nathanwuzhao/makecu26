"""Digital-twin server: serves web/ and streams live robot state + events to the browser.

    python src/twin_server.py --demo     # no hardware: simulated gripper sweeps every joint (check axes)
    python src/twin_server.py --mock     # simulated gripper, idle
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
  /api/skills                      GET: skill catalog + why each is blocked right now
  /api/skill                       POST {"name", "params"}: run one skill (skill_registry.py)
  /api/plan                        POST {"steps": [{"name", "params"}]}: skills in order (Controller.run_plan)
  /api/sim                         GET: SimRobot objects + current sim state (404 on real/fake robots)
  /api/sim/object                  POST {"name"}: swap the simulated object (controller must be idle)
  /api/vision                      POST {"image": dataURL|null, "source": "sim"|"camera"}: one Gemini
                                   scene analysis + plan (vision.py); saved to runs/vision/<ts>/
"""

import argparse
import base64
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
VISION_RUNS = Path(__file__).parent.parent / "runs" / "vision"


def _num(x):
    return None if isinstance(x, float) and math.isnan(x) else x


class Twin:
    def __init__(self, robot, camera=None, host: str = "127.0.0.1", port: int = 8000, controller=None):
        self.robot = robot
        self.camera = camera
        self.controller = controller
        self.host, self.port = host, port
        self.cond = threading.Condition()
        self.state_json: str | None = None
        self.state_seq = 0
        self.events: deque[tuple[int, str]] = deque(maxlen=300)
        self.event_seq = 0
        self._frames: dict[int, list] = {}      # frame-request id -> [threading.Event, jpeg bytes | None]
        self._frame_id = 0
        robot.listeners.append(self._on_state)

    def request_frame(self, timeout: float = 4.0) -> bytes | None:
        """Ask an open twin page for its simulated top-cam image (SSE 'frame_request' -> the page
        renders twin.topCamJPEG and POSTs /api/frame). None if no page answers in time."""
        with self.cond:
            self._frame_id += 1
            fid = self._frame_id
            slot = self._frames[fid] = [threading.Event(), None]
        self.publish("frame_request", {"id": fid})
        slot[0].wait(timeout)
        self._frames.pop(fid, None)
        return slot[1]

    def deliver_frame(self, fid: int, jpeg: bytes) -> bool:
        slot = self._frames.get(fid)
        if not slot or slot[0].is_set():
            return False
        slot[1] = jpeg
        slot[0].set()
        return True

    def _on_state(self, st):
        msg = json.dumps({"t": st.t, "ok": st.ok, "angles": st.angles,
                          "current_ma": [_num(c) for c in st.current_ma], "tactile": st.tactile.hex(),
                          "sim": getattr(st, "sim", None)})   # SimRobot: object/cap state for the twin
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
        action = self.path.split("?")[0].rstrip("/")
        if action == "/api/vision":                 # works without a controller too (page snapshot)
            return self._vision(body)
        if action == "/api/sim/object":
            return self._sim_object(body)
        if action == "/api/frame":                  # the page answering a frame_request
            import base64
            img = str(body.get("image") or "")
            try:
                jpeg = base64.b64decode(img.split(",", 1)[1] if img.startswith("data:") else img)
            except Exception:
                return self._json(400, {"ok": False, "msg": "bad image"})
            ok = self.twin.deliver_frame(int(body.get("id") or 0), jpeg)
            return self._json(200 if ok else 410, {"ok": ok})
        if ctl is None:
            return self._json(404, {"ok": False, "msg": "no controller (start main.py, not twin_server.py)"})
        if action == "/api/estop":                  # first: never behind parsing/validation
            ok, msg = ctl.estop()
        elif action == "/api/stop":
            ok, msg = ctl.stop()
        elif action == "/api/reset":
            ok, msg = ctl.reset()
        elif action == "/api/start":
            try:
                cycles = max(1, min(20, int(body.get("cycles", 8))))
            except (TypeError, ValueError):
                cycles = 8
            ok, msg = ctl.start(gemini=bool(body.get("gemini")), cycles=cycles)
        elif action == "/api/skill":
            params = body.get("params") if isinstance(body.get("params"), dict) else {}
            ok, msg = ctl.run_skill(str(body.get("name", "")), params)
        elif action == "/api/plan":
            steps = body.get("steps")
            if not isinstance(steps, list) or not all(isinstance(x, dict) for x in steps):
                return self._json(400, {"ok": False, "msg": "body must be {\"steps\": [{\"name\", \"params\"}]}"})
            ok, msg = ctl.run_plan(steps)
        elif action == "/api/agent":
            try:
                steps = max(1, min(12, int(body.get("max_steps", 8))))
            except (TypeError, ValueError):
                steps = 8
            ok, msg = ctl.run_agent(str(body.get("goal") or "").strip()[:400], steps,
                                    source=str(body.get("source") or "auto"))
        else:
            return self._json(404, {"ok": False, "msg": "unknown action"})
        self._json(200 if ok else 409, {"ok": ok, "msg": msg, "status": ctl.status()})

    def _log(self, kind: str, **data):
        ctl = self.twin.controller
        (ctl.log(kind, **data) if ctl else self.twin.publish(kind, data))

    def _sim_object(self, body):
        robot, ctl = self.twin.robot, self.twin.controller
        if not hasattr(robot, "set_object"):
            return self._json(404, {"ok": False, "msg": "not a simulator (start main.py --sim)"})
        if ctl is not None and ctl.state != "idle":
            return self._json(409, {"ok": False, "msg": f"controller is {ctl.state}: stop it first"})
        ok, msg = robot.set_object(str(body.get("name", "")))
        if ok and ctl is not None:
            ctl.sk.holding.clear()               # new object: nothing is gripped any more
            ctl.sk.last_contact.clear()          # ...and old contact angles would overshoot the fast approach
            self._log("sim_object", name=body.get("name"), msg=msg)
        return self._json(200 if ok else 400, {"ok": ok, "msg": msg,
                                               "sim": robot.sim_info() if hasattr(robot, "sim_info") else None})

    def _vision(self, body):
        """Snapshot (page dataURL, or the webcam) -> vision.analyze_scene -> log + runs/vision/<ts>/."""
        import skill_registry
        import vision
        ctl, source = self.twin.controller, str(body.get("source") or "sim")
        img = body.get("image")
        jpeg = None
        if isinstance(img, str) and img:
            try:
                jpeg = base64.b64decode(img.split(",", 1)[1] if img.startswith("data:") else img, validate=False)
            except (ValueError, IndexError):
                return self._json(400, {"ok": False, "msg": "bad image dataURL", "analysis": None,
                                        "budget": vision.budget_info()})
            if img.startswith("data:") and not img.startswith("data:image/jpeg"):
                jpeg = _to_jpeg(jpeg)            # PNG etc. from the canvas: Gemini call is sent as JPEG
        elif source == "camera":
            cam = self.twin.camera or (ctl.cam if ctl else None)
            jpeg = cam.jpeg() if cam else None
            if jpeg is None:
                return self._json(409, {"ok": False, "msg": "no camera frame (started with --no-camera?)",
                                        "analysis": None, "budget": vision.budget_info()})
        if not jpeg:
            return self._json(400, {"ok": False, "msg": "no image", "analysis": None, "budget": vision.budget_info()})
        catalog = [s for s in (ctl.skills() if ctl else skill_registry.catalog()) if s.get("group") != "manual"]
        ctx = [f"image source: {source}"]
        goal = str(body.get("goal") or "").strip()[:300]
        if goal:
            ctx.append(f"operator's goal: {goal}")
        if hasattr(self.twin.robot, "sim_info"):
            try:
                cur = self.twin.robot.sim_info() or {}
                ctx.append(f"simulated object loaded: {cur.get('object')}")
            except Exception:
                pass
        if ctl is not None:
            ctx.append(f"controller {ctl.state}; holding: " +
                       (", ".join(f"servo {k}" for k, v in ctl.sk.holding.items() if v) or "nothing"))
        analysis, msg = vision.analyze_scene(jpeg, catalog, "; ".join(ctx))
        out = analysis.model_dump() if analysis else None
        d = VISION_RUNS / time.strftime("%Y%m%d-%H%M%S")
        try:
            d.mkdir(parents=True, exist_ok=True)
            (d / "input.jpg").write_bytes(jpeg)
            (d / "analysis.json").write_text(json.dumps({"source": source, "context": ctx, "msg": msg,
                                                         "analysis": out}, indent=1), encoding="utf-8")
        except OSError:
            pass
        self._log("vision", ok=analysis is not None, source=source, msg=msg, analysis=out, saved=str(d))
        refused = msg.startswith("budget") or msg.startswith("429")
        return self._json(200 if analysis else (409 if refused else 502),
                          {"ok": analysis is not None, "msg": msg, "analysis": out, "budget": vision.budget_info()})

    def do_GET(self):
        if self.path.startswith("/api/sim"):
            robot = self.twin.robot
            return (self._json(200, robot.sim_info()) if hasattr(robot, "sim_info")
                    else self._json(404, {"msg": "not a simulator"}))
        if self.path.startswith("/api/status"):
            ctl = self.twin.controller
            return self._json(200, ctl.status()) if ctl else self._json(404, {"state": "no controller"})
        if self.path.startswith("/api/skills"):
            ctl = self.twin.controller
            return self._json(200, ctl.skills()) if ctl else self._json(404, [])
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


def _to_jpeg(data: bytes) -> bytes | None:
    import cv2 as cv
    import numpy as np
    img = cv.imdecode(np.frombuffer(data, np.uint8), cv.IMREAD_COLOR)
    if img is None:
        return None
    ok, buf = cv.imencode(".jpg", img, [cv.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes() if ok else None


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
    ap.add_argument("--mock", action="store_true", help="simulated gripper")
    ap.add_argument("--demo", action="store_true", help="simulated gripper that sweeps every joint")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other devices on the network")
    ap.add_argument("--serial", default=None, help="ESP32 port (default: auto)")
    ap.add_argument("--camera", type=int, default=None, help="webcam index to stream")
    args = ap.parse_args()

    if args.mock or args.demo:
        from dtwin_robot import DTwinRobot
        robot = DTwinRobot().start()
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
