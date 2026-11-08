"""Hardware mapping and tuning.

Two kinds of calibration, both in calibration.json:
  1. "servos": each servo's end-of-travel pulses (python src/calibrate.py). Also written to the
     firmware (C + R), so afterwards EVERY servo means: 0 deg = HOME end, 180 deg = FAR end, both
     a margin inside the physical stops (servo_cal.py). That's why the angles below are 0 / 180.
  2. "slots": a few working positions inside that range (python src/jog.py).
main.py refuses to drive the real gripper until both are complete and the firmware matches.
Wiring/servo facts: docs/hardware-wiring.md.
"""

import json
import os
from dataclasses import dataclass
from pathlib import Path

CAL_FILE = Path(os.getenv("CAL_FILE", Path(__file__).parent.parent / "calibration.json"))


def _load_cal() -> dict:
    try:
        return json.loads(CAL_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


CAL = _load_cal()
_SLOTS = CAL.get("slots", {})


def _a(name: str, default: int) -> int:
    return int(_SLOTS.get(name, default))


def calibration_problems() -> list[str]:
    """What's missing from calibration.json before the real gripper may run (empty = OK).
    The firmware-side match is checked separately after connecting (servo_cal.check)."""
    servos = CAL.get("servos", {})
    out = [f"servo {i} end-of-travel not calibrated (python src/calibrate.py {i})"
           for i in range(7) if str(i) not in servos]
    out += [f"slot {name} not set (python src/jog.py)" for name, _, _ in SLOT_DEFS if name not in _SLOTS]
    return out


def missing_slots() -> list[str]:  # kept for older callers
    return calibration_problems()


@dataclass(frozen=True)
class Squeeze:
    idx: int         # servo index 0-6
    open: int        # angle with fingers apart
    closed: int      # furthest the grip is ever driven (hard limit, never commanded past)
    current_ch: int  # 0 = i0 (INA219 @0x40, servo 0), 1 = i1 (INA219 @0x45, servo 5)


@dataclass(frozen=True)
class Slide:
    idx: int
    start: int       # angle at the start of a twist stroke (recentered position)
    end: int         # angle at the end of a full twist stroke


@dataclass(frozen=True)
class Lift:
    idx: int
    grip: int        # height where the top fingers line up with the cap
    test: int        # small lift for the "is it off?" test (a few mm)
    full: int        # big lift for the final reveal


# Servo map (docs/hardware-wiring.md):
#   0 bottom squeeze (i0) | 1 bottom finger right | 2 halves up/down | 3 bottom finger left
#   4 top finger right    | 5 top squeeze (i1)    | 6 top finger left
# After servo calibration: squeeze 0 = open, 180 = closed; fingers 0 = back, 180 = forward;
# halves 0 = down, 180 = up. Only positions INSIDE those ranges need slots:
SLOT_DEFS = [   # (name, servo index, fallback angle); jog.py walks through these in order
    ("bottom_left.home", 3, 90),      # bottom fingers' position while holding the bottle
    ("bottom_right.home", 1, 90),
    ("halves.grip", 2, 90),           # top fingers level with the cap
    ("halves.test", 2, 110),          # a few mm up: lift test
    ("halves.full", 2, 170),          # the reveal
]

BOTTOM_SQUEEZE = Squeeze(0, open=0, closed=180, current_ch=0)
TOP_SQUEEZE = Squeeze(5, open=0, closed=180, current_ch=1)
BOTTOM_FINGERS_HOME = {3: _a("bottom_left.home", 90), 1: _a("bottom_right.home", 90)}

# Twist stroke uses the FULL finger travel (~25 mm; the fingers aren't grippy, so every mm counts).
# Recentered (start): left forward, right back. Twist (start -> end): right forward, left back.
# That shear must turn the cap COUNTER-clockwise seen from above. If it tightens instead,
# set twist_reversed (jog.py key r, or TWIST_REVERSED=1): swaps start/end on both.
_left = Slide(6, start=180, end=0)
_right = Slide(4, start=0, end=180)
if os.getenv("TWIST_REVERSED", str(CAL.get("twist_reversed", 0))) == "1":
    _left, _right = Slide(6, _left.end, _left.start), Slide(4, _right.end, _right.start)
TWIST_SLIDES = (_left, _right)
HALVES = Lift(2, _a("halves.grip", 90), _a("halves.test", 110), _a("halves.full", 170))

# Unscrew plan: twist LIFT_TEST_EVERY cycles, then lift-test (Gemini judges if the cap came off).
# degrees of cap per cycle ~= 360 * 25 mm / (pi * cap_diameter_mm) minus slip; e.g. a 28 mm cap
# turns ~100 deg per full stroke if nothing slips. Typical caps need 1-1.5 turns.
LIFT_TEST_EVERY = int(os.getenv("LIFT_TEST_EVERY", "4"))
LIFT_DEG_PER_TICK = 2

# Motion
TICK_S = 0.04                 # 25 Hz, same as the D stream
MOVE_DEG_PER_TICK = 4         # free moves (open, recenter)
TWIST_DEG_PER_TICK = 3
SETTLE_S = 0.15               # wait after a move (90 deg takes ~150 ms)

# ---- Grip force from current ----
# An SG90 is a position servo: torque ~ (commanded - actual angle). Once the fingers touch the
# object, every extra degree commanded is extra squeeze, and the holding current rises with it.
# So: close until contact, then adjust the commanded angle until the HOLDING current sits in
# [target - band, target + band]. Keep regulating during the twist (the squeeze servo isn't
# moving then, so its INA219 reading has no move spikes in it).
# Free-air holding current can depend on angle (springs, friction): jog.py's "p" key records a
# baseline per angle, and contact/force use (holding - baseline).
SQUEEZE_STEP_DEG = 3
APPROACH_MARGIN_DEG = 10      # fast-move to (last contact - this), then step slowly
RELEASE_MARGIN_DEG = 15       # between twists, back off only this far from the contact angle
STEP_SETTLE_S = 0.12          # must outlast the inrush/braking transient
HOLD_SAMPLES = 2              # D lines checked after settling; min() rejects a leftover spike
HOLD_CONTACT_MA = 40.0        # TODO(calibrate): excess over baseline that means "touching"
GRIP_TARGET_MA = {0: float(CAL.get("grip_target_ma", {}).get("0", 150)),   # TODO(calibrate) bottle
                  5: float(CAL.get("grip_target_ma", {}).get("5", 150))}   # TODO(calibrate) cap
GRIP_BAND_MA = 30.0
GRIP_MAX_MA = 450.0           # stall guard: above this, back off right away
REGULATE_MAX_STEPS = 15       # give up adjusting after this many 1-degree steps
REGULATE_EVERY_TICKS = 4      # during the twist: check the squeeze current every N ticks (~160 ms)
BASELINE_MA = {int(k): {int(a): v for a, v in d.items()} for k, d in CAL.get("baseline_ma", {}).items()}

# Tactile: the sensor has a hardware fault (docs/firmware-notes.md). Contact detection with it
# is off until fixed; its features are still logged every cycle.
USE_TACTILE = os.getenv("USE_TACTILE", "0") == "1"
TACTILE_DELTA = 25
TACTILE_CELLS = 6

# ---- Agent / Gemini (free tier: be stingy) ----
# Free-tier quota is per model. Develop on a lite model, save the full one for the demo.
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-3.5-flash-lite")
# Free tier for this key: 250 requests/day, 15/min (per model). We stay well under both.
GEMINI_MIN_INTERVAL_S = float(os.getenv("GEMINI_MIN_INTERVAL_S", "20"))  # <= 3 calls/min (limit 15)
# Gemini's main job is the lift test. Per-cycle monitoring is extra; 0 = off (failed grips still ask).
AGENT_EVERY_N_CYCLES = int(os.getenv("AGENT_EVERY_N_CYCLES", "0"))
GEMINI_RUN_CAP = int(os.getenv("GEMINI_RUN_CAP", "8"))       # max calls per program run
GEMINI_DAILY_CAP = int(os.getenv("GEMINI_DAILY_CAP", "200"))  # per model per day (limit 250, keep 50 spare)
GEMINI_429_COOLDOWN_S = float(os.getenv("GEMINI_429_COOLDOWN_S", "60"))  # if Gemini doesn't say how long
IMAGE_WIDTH = 384             # per image; before/after are sent as ONE side-by-side JPEG

MAX_CYCLES = int(os.getenv("MAX_CYCLES", "20"))
DEFAULT_TWIST_FRACTION = 1.0
