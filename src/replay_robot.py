"""Plays a tools/record.py recording back as if it were the gripper (read-only), for the web twin.

    bot = ReplayRobot("runs/recordings/20261108-140000_squeeze.jsonl", speed=1.0, loop=True).start()

Same interface as DTwinRobot/RobotSerial (state, listeners, start, set, motion_locked, command,
query_servo_config, close). States keep their original spacing (divided by speed) but get fresh
t = now. Markers ride along on every following state as .sim = {"object": "replay", "note": ...}
so the twin can show which phase (idle / light / hard / twist_slip / release) is playing.
"""

import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from robot_serial import NUM_SERVOS, RobotState


@dataclass
class ReplayState(RobotState):
    sim: dict | None = None


class ReplayRobot:
    def __init__(self, path, speed: float = 1.0, loop: bool = True):
        self.path = Path(path)
        self.speed = max(0.01, speed)
        self.loop = loop
        self.header: dict = {}
        self.events: list[dict] = []    # D states and markers, in file order (= time order)
        with self.path.open(encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                if rec.get("kind") == "header":
                    self.header = rec
                elif rec.get("kind") in ("D", "marker"):
                    self.events.append(rec)
        if not any(e["kind"] == "D" for e in self.events):
            raise ValueError(f"{self.path}: no D states in recording")
        self.events.sort(key=lambda e: e["t"])
        self.listeners = []
        self.done = threading.Event()   # set when a non-looping replay reaches the end
        self._state = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> "ReplayRobot":
        self._thread.start()
        t0 = time.time()
        while self.state is None and time.time() - t0 < 2:
            time.sleep(0.02)
        return self

    @property
    def state(self):
        with self._lock:
            return self._state

    # Always locked: the controller's E-stop / reset write this, but a replay can never move.
    @property
    def motion_locked(self):
        return True

    @motion_locked.setter
    def motion_locked(self, _value):
        pass

    def set(self, angles):
        raise RuntimeError("replay: read-only")

    def command(self, line: str):
        pass                            # E-stop's limp_all lands here: nothing to limp

    def query_servo_config(self, timeout: float = 1.5):
        cfg = self.header.get("servo_config") or {}
        try:
            return {i: tuple(cfg.get(str(i)) or cfg[i]) for i in range(NUM_SERVOS)}
        except (KeyError, TypeError):
            return {i: (500, 2400, 0) for i in range(NUM_SERVOS)}

    def close(self):
        self._stop.set()

    def _emit(self, rec: dict, note: str, t_rel: float):
        cur = [math.nan if c is None else float(c) for c in rec["current_ma"]]
        sim = {"object": "replay", "task": "replay", "note": note,
               "label": self.header.get("label", self.path.stem), "t_rel": round(t_rel, 2)}
        if rec.get("sim"):                # recorded from a SimRobot: keep its fields too
            sim = {**rec["sim"], **sim, "object": rec["sim"].get("object", "replay")}
        st = ReplayState(t=time.time(), ok=rec["ok"], angles=list(rec["angles"]), current_ma=cur,
                         tactile=bytes.fromhex(rec["tactile"]), sim=sim)
        with self._lock:
            self._state = st
        for fn in list(self.listeners):
            try:
                fn(st)
            except Exception as e:      # a bad listener must not kill playback
                print(f"listener error: {e}")

    def _loop(self):
        t_first = self.events[0]["t"]
        while not self._stop.is_set():
            note = "start"
            start = time.time()
            for e in self.events:
                wait = start + (e["t"] - t_first) / self.speed - time.time()
                if wait > 0 and self._stop.wait(wait):
                    return
                if e["kind"] == "marker":
                    note = e.get("note") or e.get("phase") or "marker"
                    if e.get("phase") not in (None, "note"):
                        note = e["phase"] + (f": {e['note']}" if e.get("note") else "")
                else:
                    self._emit(e, note, e["t"] - t_first)
            if not self.loop:
                self.done.set()
                return
            self._stop.wait(0.5)        # short pause before looping
