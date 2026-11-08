"""Per-servo end-of-travel calibration, stored in BOTH the firmware (C + R, in flash) and
calibration.json (so the PC can check the firmware still matches before moving anything).

After calibration every servo means the same thing, whichever way it is mounted:
    angle 0   = HOME end  (squeeze open / finger back / halves down)
    angle 180 = FAR end   (squeeze closed / finger forward / halves up)
and both ends sit MARGIN_US inside the physical end of travel. So no angle 0-180 can drive a
servo into its end stop (or, for top finger left, past where it has no end stop).

How it maps onto the firmware (docs/serial-protocol.md "C" and "R"):
    min_us, max_us = the two end pulses (sorted)
    reversed       = 1 if the HOME end is the larger pulse  (then angle a -> pulse of 180 - a)

Raw mode (used only while calibrating one servo): C 500 2400, R 0. Then angle a drives
500 + a * 1900/180 us, i.e. ~10.6 us per degree, so jogging the angle IS jogging the pulse.
"""

import time

NUM_SERVOS = 7
RAW_MIN_US, RAW_MAX_US = 500, 2400          # firmware default C; the calibration "window"
US_PER_DEG = (RAW_MAX_US - RAW_MIN_US) / 180
MARGIN_US = 30                               # ~3 deg / ~0.4 mm back from each physical end

# idx: (name, HOME end, FAR end, warning)
SERVOS = {
    0: ("bottom squeeze", "OPEN (fingers fully apart)", "CLOSED (fingers touching each other)", ""),
    1: ("bottom finger right", "BACK (fully retracted)", "FORWARD (fully extended)", ""),
    2: ("halves up/down", "DOWN (top half lowest)", "UP (top half highest)", ""),
    3: ("bottom finger left", "BACK (fully retracted)", "FORWARD (fully extended)", ""),
    4: ("top finger right", "BACK (fully retracted)", "FORWARD (fully extended)", ""),
    5: ("top squeeze", "OPEN (fingers fully apart)", "CLOSED (fingers touching each other)", ""),
    6: ("top finger left", "BACK (fully retracted)", "FORWARD (fully extended)",
        "NO END STOPS: stop at the end of useful travel, it won't hit anything to tell you"),
}


def raw_angle_to_us(a: float) -> int:
    return round(RAW_MIN_US + a * US_PER_DEG)


def raw_us_to_angle(us: float) -> int:
    return round((us - RAW_MIN_US) / US_PER_DEG)


def compute(home_us: int, far_us: int, margin: int = MARGIN_US) -> dict:
    """Marked physical ends -> firmware settings. Ends are pulled `margin` us toward each other."""
    if abs(far_us - home_us) < 4 * margin:
        raise ValueError(f"ends only {abs(far_us - home_us)} us apart: marked the same end twice?")
    s = 1 if far_us > home_us else -1
    home, far = home_us + s * margin, far_us - s * margin
    return {"home_us": home, "far_us": far, "min_us": min(home, far), "max_us": max(home, far),
            "reversed": int(home > far), "marked_home_us": home_us, "marked_far_us": far_us,
            "margin_us": margin, "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S")}


def _limp_then(robot, idx: int, rev_now: int, rev_want: int):
    """Make the servo limp (by flipping R, which the firmware answers by going limp) and end
    with the wanted reverse flag. Never drives the servo."""
    robot.command(f"R {idx} {1 - rev_now}")       # flag change -> limp
    if rev_want != 1 - rev_now:
        robot.command(f"R {idx} {rev_want}")      # second change, still limp
    time.sleep(0.1)


def limp(robot, idx: int):
    """Stop driving one servo right now (it goes slack). Flag ends unchanged."""
    rev = robot.query_servo_config()[idx][2]
    _limp_then(robot, idx, rev, rev)


def limp_all(robot, reversed_flags: dict[int, int]):
    """E-stop: every servo limp NOW. No queries (no waiting): flips each servo's R and flips it
    back, which the firmware answers by going limp. reversed_flags must be the firmware's
    current flags (main.py verified them against calibration.json at start)."""
    for i in range(NUM_SERVOS):
        r = int(reversed_flags.get(i, 0))
        robot.command(f"R {i} {1 - r}")
        robot.command(f"R {i} {r}")


def apply(robot, idx: int, min_us: int, max_us: int, reversed_: int):
    """Write C + R for one servo. The servo is made limp FIRST, so the new mapping can't make it
    jump (firmware: C re-drives an engaged servo at its old angle under the new mapping)."""
    rev = robot.query_servo_config()[idx][2]
    _limp_then(robot, idx, rev, reversed_)
    robot.command(f"C {idx} {min_us} {max_us}")   # limp -> firmware doesn't drive it
    time.sleep(0.1)


def to_raw(robot, idx: int):
    apply(robot, idx, RAW_MIN_US, RAW_MAX_US, 0)


def check(robot, cal: dict) -> list[str]:
    """Problems that must stop main.py: servo not calibrated, or firmware C/R differ from
    calibration.json (reflashed/erased board, or a calibration abandoned half-way)."""
    servos = cal.get("servos", {})
    fw = robot.query_servo_config()
    out = []
    for i in range(NUM_SERVOS):
        c = servos.get(str(i))
        if c is None:
            out.append(f"servo {i} ({SERVOS[i][0]}) not calibrated")
            continue
        want = (c["min_us"], c["max_us"], c["reversed"])
        if tuple(fw[i]) != want:
            out.append(f"servo {i} ({SERVOS[i][0]}): firmware has C {fw[i][0]} {fw[i][1]} R {fw[i][2]}, "
                       f"calibration.json says C {want[0]} {want[1]} R {want[2]}")
    return out


def restore(robot, cal: dict, idx: int):
    """Put back the saved calibration for one servo (or raw window if never calibrated)."""
    c = cal.get("servos", {}).get(str(idx))
    if c:
        apply(robot, idx, c["min_us"], c["max_us"], c["reversed"])
    else:
        to_raw(robot, idx)
