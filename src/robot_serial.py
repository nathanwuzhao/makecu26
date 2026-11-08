"""Host side of the serial protocol in docs/serial-protocol.md.

A background thread reads D/CAL/ERR lines and keeps the latest state; set() sends S commands.

    bot = RobotSerial("COM3").start()
    bot.set({0: 90, 3: 45})
    print(bot.state)  # latest RobotState, or None before the first D line
"""

import threading
import time
from dataclasses import dataclass, field

import serial

NUM_SERVOS = 7
LIMP = -1                     # angle the firmware reports for a servo it isn't driving (all, after boot)


class MotionLocked(RuntimeError):
    """set() was called while the E-stop lock is on (controller.py)."""
NUM_CURRENT = 2               # INA219s on the two squeeze servos
TACTILE_ROWS, TACTILE_COLS = 16, 32
TACTILE_COUNT = TACTILE_ROWS * TACTILE_COLS


@dataclass
class RobotState:
    t: float                 # host time.time() when the line arrived
    ok: bool                 # tactile link healthy (see docs/serial-protocol.md)
    angles: list[int]        # servo TARGET angles (no position feedback on SG90s); LIMP (-1) = not driven
    current_ma: list[float]  # squeeze-servo currents in mA; nan if that INA219 wasn't found
    tactile: bytes           # 512 values 0-255, row-major: index = row * 32 + col

    def tactile_grid(self) -> list[list[int]]:
        return [list(self.tactile[r * TACTILE_COLS:(r + 1) * TACTILE_COLS]) for r in range(TACTILE_ROWS)]


def parse_d_line(tokens: list[str]) -> RobotState | None:
    """tokens excludes the leading 'D': <ok> <a0>..<a6> <i0> <i1> <hex>. None if malformed."""
    if len(tokens) != 1 + NUM_SERVOS + NUM_CURRENT + 1:
        return None
    try:
        ok = tokens[0] == "1"
        angles = [int(x) for x in tokens[1:1 + NUM_SERVOS]]
        currents = [float(x) for x in tokens[1 + NUM_SERVOS:1 + NUM_SERVOS + NUM_CURRENT]]
        tactile = bytes.fromhex(tokens[-1])
    except ValueError:
        return None
    if len(tactile) != TACTILE_COUNT:
        return None
    return RobotState(t=time.time(), ok=ok, angles=angles, current_ma=currents, tactile=tactile)


ESP32_VID = 0x303A


def find_port() -> str:
    """COM port of the ESP32-C6 (USB VID 303A). COM numbers change after replugging."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == ESP32_VID:
            return p.device
    raise RuntimeError("ESP32-C6 not found (VID 303A). Plugged in directly, not through the hub?")


class RobotSerial:
    def __init__(self, port: str | None = None, baud: int = 115200):
        # DTR/RTS must be off BEFORE open(): a default open resets the ESP32-C6 and every servo
        # snaps to INITIAL_ANGLE (docs/serial-protocol.md -> "Connection").
        self.ser = serial.Serial()
        self.ser.port = port or find_port()
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self._lock = threading.Lock()
        self._state: RobotState | None = None
        self.calibration: dict[int, tuple[int, int]] = {}   # CAL replies: (min_us, max_us)
        self.reversed: dict[int, int] = {}                  # REV replies: 0 / 1
        self.motion_locked = False                          # E-stop: set() refuses while True
        self.errors: list[str] = []          # recent ERR lines, newest last
        self.listeners = []                  # callables(RobotState), called from the reader thread
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)

    def start(self) -> "RobotSerial":
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self.ser.close()

    @property
    def state(self) -> RobotState | None:
        with self._lock:
            return self._state

    def set(self, angles: dict[int, float]) -> None:
        """Send one S command. Angles are clamped to 0-180 and rounded.
        Raises MotionLocked while motion_locked is set (E-stop)."""
        if self.motion_locked:
            raise MotionLocked("E-stop active: motion locked")
        if not angles:
            return
        parts = []
        for idx, a in angles.items():
            if not 0 <= idx < NUM_SERVOS:
                raise ValueError(f"bad servo index {idx}")
            parts.append(f"{idx} {int(round(min(180, max(0, a))))}")
        self._write("S " + " ".join(parts))

    def request_calibration(self) -> None:
        self._write("C")

    def command(self, line: str) -> None:
        """Send a raw C / R command line (not S: use set())."""
        if line[:1].upper() not in ("C", "R"):
            raise ValueError("command() is for C / R lines only")
        self._write(line)

    def query_servo_config(self, timeout: float = 1.5) -> dict[int, tuple[int, int, int]]:
        """Ask the firmware for every servo's (min_us, max_us, reversed). No motion."""
        self.calibration.clear()
        self.reversed.clear()
        self._write("C")
        self._write("R")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if len(self.calibration) == NUM_SERVOS and len(self.reversed) == NUM_SERVOS:
                return {i: (*self.calibration[i], self.reversed[i]) for i in range(NUM_SERVOS)}
            time.sleep(0.02)
        raise RuntimeError(f"no full C/R reply from firmware (got CAL {sorted(self.calibration)}, "
                           f"REV {sorted(self.reversed)}). Old firmware without R?")

    def _write(self, line: str) -> None:
        with self._lock:
            self.ser.write((line + "\n").encode("ascii"))

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                time.sleep(0.5)
                continue
            if not raw:
                continue
            tokens = raw.decode("ascii", errors="replace").strip().split()
            if not tokens:
                continue
            kind, rest = tokens[0], tokens[1:]
            if kind == "D":
                st = parse_d_line(rest)
                if st is None:
                    continue
                with self._lock:
                    self._state = st
                for fn in list(self.listeners):
                    try:
                        fn(st)
                    except Exception as e:  # a bad listener must not kill the reader
                        print(f"listener error: {e}")
            elif kind == "CAL" and len(rest) == 3:
                idx, lo, hi = (int(x) for x in rest)
                self.calibration[idx] = (lo, hi)
            elif kind == "REV" and len(rest) == 2:
                idx, val = (int(x) for x in rest)
                self.reversed[idx] = val
            elif kind == "ERR":
                self.errors = (self.errors + [" ".join(rest)])[-20:]


if __name__ == "__main__":
    import os
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else os.getenv("SERIAL_PORT") or None
    bot = RobotSerial(port).start()
    try:
        while True:
            time.sleep(0.5)
            st = bot.state
            if st:
                print(st.ok, st.angles, st.current_ma, "tactile max", max(st.tactile))
    except KeyboardInterrupt:
        bot.close()
