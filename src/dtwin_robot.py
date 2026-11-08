"""Stand-in for RobotSerial so skills/agent can run without hardware.

Squeeze servos "hit" an object at CONTACT_ANGLE: past it, current jumps and tactile cells light up.
"""

import math
import threading
import time

import config
from robot_serial import LIMP, NUM_SERVOS, TACTILE_COUNT, MotionLocked, RobotState

CONTACT_ANGLE = 95


class DTwinRobot:
    def __init__(self):
        self._angles = [LIMP] * NUM_SERVOS   # like the real firmware: limp until the first S
        self._moved_at = [0.0] * NUM_SERVOS   # for the start-of-move current spike
        self._state = None
        self._lock = threading.Lock()
        self.listeners = []
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def start(self) -> "DTwinRobot":
        time.sleep(0.1)
        return self

    @property
    def state(self):
        with self._lock:
            return self._state

    motion_locked = False

    def set(self, angles: dict[int, float]):
        if self.motion_locked:
            raise MotionLocked("E-stop active: motion locked")
        with self._lock:
            for i, a in angles.items():
                a = int(round(min(180, max(0, a))))
                if a != self._angles[i]:
                    self._moved_at[i] = time.time()
                self._angles[i] = a

    # --- C / R like the firmware: flash-backed per-servo pulse limits + reverse flag ---
    _cal = None

    def _ensure_cal(self):
        if self._cal is None:
            self._cal = {i: [500, 2400, 0] for i in range(NUM_SERVOS)}

    def command(self, line: str):
        self._ensure_cal()
        t = line.split()
        if t[0].upper() == "R" and len(t) == 3:
            i, v = int(t[1]), int(t[2])
            if v != self._cal[i][2]:
                self._cal[i][2] = v
                with self._lock:
                    self._angles[i] = LIMP          # firmware: flag change -> limp
        elif t[0].upper() == "C" and len(t) == 4:
            i = int(t[1])
            self._cal[i][0], self._cal[i][1] = int(t[2]), int(t[3])

    def query_servo_config(self, timeout: float = 1.5):
        self._ensure_cal()
        return {i: tuple(v) for i, v in self._cal.items()}

    def close(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                angles = list(self._angles)
            currents = [0.0, 0.0]
            tactile = bytearray([10] * TACTILE_COUNT)
            for sq in (config.BOTTOM_SQUEEZE, config.TOP_SQUEEZE):
                press = max(0, angles[sq.idx] - CONTACT_ANGLE)
                hold = 4 + 25 * press + 2 * math.sin(time.time() * 40)
                spike = 450 if time.time() - self._moved_at[sq.idx] < 0.08 else 0  # like the real inrush
                currents[sq.current_ch] = hold + spike
                if sq is config.TOP_SQUEEZE and press:
                    for k in range(200, 200 + 12):
                        tactile[k] = min(255, 10 + 15 * press)
            st = RobotState(t=time.time(), ok=True, angles=angles, current_ma=currents, tactile=bytes(tactile))
            with self._lock:
                self._state = st
            for fn in list(self.listeners):
                fn(st)
            time.sleep(0.04)
