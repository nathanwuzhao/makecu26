"""Supervisors: pick the next high-level action after a twist cycle. Never send servo angles.

- RuleSupervisor: no API, instant, free. Default for all control testing.
- GeminiSupervisor: vision model, guarded by a call budget so testing can't burn the free tier:
    * at most one call every GEMINI_MIN_INTERVAL_S, and only every AGENT_EVERY_N_CYCLES cycles
    * GEMINI_RUN_CAP calls per program run, GEMINI_DAILY_CAP per model per day (counted locally
      in runs/gemini_usage.json, which survives restarts)
    * a 429 (quota) response turns Gemini off for the rest of the run
    * before + after frames go as ONE small side-by-side JPEG
  Calls run in a background thread; the robot keeps cycling while Gemini thinks.

Both expose submit(cycle, before, after, summary) -> bool and poll() -> Decision | Exception | None.
"""

import json
import re
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import date
from pathlib import Path
from typing import Literal

import cv2 as cv
import numpy as np
from pydantic import BaseModel

import config

USAGE_FILE = Path(__file__).parent.parent / "runs" / "gemini_usage.json"

SYSTEM_PROMPT = """\
You supervise a small robot gripper (TEND, a Cartesian gripper with tactile and vision assistance: 4 fingers on servo-driven sliders) that is
unscrewing a cap from a small bottle. You see one image with two panels, BEFORE (left) and AFTER
(right) the latest twist cycle, plus sensor telemetry. You do NOT control servos; you pick the next
high-level action.

How the robot works:
- The two BOTTOM fingers squeeze the bottle body and hold it still.
- The two TOP fingers do repeated twist cycles: squeeze the cap, slide in opposite directions
  (turning the cap counter-clockwise seen from above = unscrewing), release, slide back to center.
- Grip contact is detected from the squeeze servo's holding current (a few mA when free, tens to
  hundreds of mA when pressing on something). grip.contact=false means the fingers closed on air.
- The cap and bottle neck have marker stripes. Compare the cap stripe between the panels to judge
  how far the cap turned. Fingers moved but stripe did not = slip.

Actions:
- twist_cycle: another grip-twist-release-recenter cycle. twist_fraction 0.1-1.0 scales the stroke
  (smaller when the cap is nearly off, so it is not flung).
- regrip_bottom: the bottle itself rotated or tilted; release and re-grip it.
- stop_success: cap clearly separated from the threads (lifted, tilted off, or spinning freely).
- stop_problem: something is wrong (cap/bottle not visible, bottle fell, repeated missed grips).

Be concise. Base cap_state on visual evidence; use "not_visible" if you can't see the cap.
"""


class Decision(BaseModel):
    observation: str                 # one sentence: what the images show
    cap_rotation_deg: int            # estimated rotation this cycle, + = unscrewing, 0 if unknown
    cap_state: Literal["tight", "loosening", "nearly_off", "off", "not_visible"]
    action: Literal["twist_cycle", "regrip_bottom", "stop_success", "stop_problem"]
    twist_fraction: float
    reasoning: str                   # one or two sentences, shown on the demo UI
    source: str = "gemini"           # filled in locally, not by the model


LIFT_PROMPT = """A robot gripper is unscrewing a cap from a small bottle. The bottom fingers hold the bottle. The top
fingers just gripped the cap and the top half of the gripper moved UP by a few millimetres.
The image has two panels: BEFORE the lift (left) and AFTER the lift (right).

Did the cap come off? Signs it is OFF: a visible gap between cap and bottle neck in AFTER, the cap
higher relative to the neck than in BEFORE, threads visible, or the cap tilted. Signs it is still ON:
cap and neck in the same relative position, bottle lifted together with the cap, no gap.
If you cannot see the cap and neck clearly, say cap_off=false and visible=false.
"""


class LiftVerdict(BaseModel):
    visible: bool          # could the cap/neck junction be seen?
    cap_off: bool
    confidence: float      # 0-1
    observation: str       # one sentence, shown on the demo UI
    source: str = "gemini"


# ---------------- no-API supervisor ----------------

class RuleSupervisor:
    """Deterministic stand-in: keeps twisting, stops after repeated missed grips."""

    def __init__(self):
        self._result: Decision | None = None
        self.misses = 0

    def note(self, line: str):
        pass

    def submit(self, cycle, before_jpeg, after_jpeg, summary) -> bool:
        if summary.get("ok"):
            self.misses = 0
            self._result = None  # nothing to change
        else:
            self.misses += 1
            stop = self.misses >= 3
            self._result = Decision(
                observation=summary.get("problem", "grip failed"),
                cap_rotation_deg=0, cap_state="not_visible",
                action="stop_problem" if stop else "twist_cycle",
                twist_fraction=config.DEFAULT_TWIST_FRACTION,
                reasoning=f"{self.misses} missed grip(s) in a row" + ("; giving up" if stop else "; retrying"),
                source="rules")
        return False

    def poll(self):
        r, self._result = self._result, None
        return r

    def check_lift(self, before_jpeg, after_jpeg) -> LiftVerdict | None:
        return None  # rules can't see; caller treats None as "unknown, keep twisting"


# ---------------- budget ----------------

class Budget:
    def __init__(self, model: str):
        self.model = model
        self.run_calls = 0
        self.disabled_reason: str | None = None
        self.last_call = 0.0
        self.cooldown_until = 0.0
        self.consecutive_429 = 0

    def _load(self) -> dict:
        try:
            return json.loads(USAGE_FILE.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def used_today(self) -> int:
        return self._load().get(str(date.today()), {}).get(self.model, 0)

    def can_call(self) -> tuple[bool, str]:
        if self.disabled_reason:
            return False, self.disabled_reason
        if self.run_calls >= config.GEMINI_RUN_CAP:
            return False, f"run cap {config.GEMINI_RUN_CAP} reached"
        if self.used_today() >= config.GEMINI_DAILY_CAP:
            return False, f"daily cap {config.GEMINI_DAILY_CAP} reached for {self.model}"
        if time.time() < self.cooldown_until:
            return False, f"429 cooldown, {self.cooldown_until - time.time():.0f} s left"
        if time.time() - self.last_call < config.GEMINI_MIN_INTERVAL_S:
            return False, "min interval"
        return True, ""

    def on_429(self, err) -> str:
        """Per-minute limit: cool down (using Gemini's retryDelay if given) and try later.
        Per-day limit, or 3 x 429 in a row: Gemini off for the rest of the run."""
        self.consecutive_429 += 1
        text = str(err)
        if "PerDay" in text or "per day" in text.lower() or self.consecutive_429 >= 3:
            self.disabled_reason = "429: daily quota (or repeated rate limit); Gemini off for this run"
            return self.disabled_reason
        delay = config.GEMINI_429_COOLDOWN_S
        m = re.search(r"retryDelay'?\"?:\s*'?\"?(\d+(?:\.\d+)?)s", text)
        if m:
            delay = float(m.group(1)) + 2
        self.cooldown_until = time.time() + delay
        return f"429 rate limit; cooling down {delay:.0f} s"

    def on_success(self):
        self.consecutive_429 = 0

    def record(self):
        self.run_calls += 1
        self.last_call = time.time()
        data = self._load()
        day = data.setdefault(str(date.today()), {})
        day[self.model] = day.get(self.model, 0) + 1
        USAGE_FILE.parent.mkdir(exist_ok=True)
        USAGE_FILE.write_text(json.dumps(data, indent=1))


def side_by_side(before_jpeg: bytes | None, after_jpeg: bytes | None, width: int = config.IMAGE_WIDTH) -> bytes | None:
    """One labeled BEFORE|AFTER JPEG: half the requests' image overhead of two images."""
    panels = []
    for label, jpg in (("BEFORE", before_jpeg), ("AFTER", after_jpeg)):
        if jpg is None:
            continue
        img = cv.imdecode(np.frombuffer(jpg, np.uint8), cv.IMREAD_COLOR)
        h, w = img.shape[:2]
        img = cv.resize(img, (width, int(h * width / w)), interpolation=cv.INTER_AREA)
        cv.putText(img, label, (8, 24), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        panels.append(img)
    if not panels:
        return None
    hmin = min(p.shape[0] for p in panels)
    ok, buf = cv.imencode(".jpg", np.hstack([p[:hmin] for p in panels]), [cv.IMWRITE_JPEG_QUALITY, 75])
    return buf.tobytes() if ok else None


# ---------------- Gemini ----------------

class GeminiSupervisor:
    def __init__(self, model: str = config.GEMINI_MODEL, log=print):
        from google import genai  # imported here so the rule path never needs the SDK
        self.genai = genai
        self.client = genai.Client()  # reads GEMINI_API_KEY
        self.model = model
        self.budget = Budget(model)
        self.log = log
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pending: Future | None = None
        self.history: list[str] = []
        self.lock = threading.Lock()
        self.fallback = RuleSupervisor()  # still catches missed grips between Gemini calls

    def note(self, line: str):
        with self.lock:
            self.history = (self.history + [line])[-12:]

    def submit(self, cycle, before_jpeg, after_jpeg, summary) -> bool:
        self.fallback.submit(cycle, before_jpeg, after_jpeg, summary)
        if self.pending is not None and not self.pending.done():
            return False
        # ask on every Nth cycle, or right away when something went wrong
        periodic = config.AGENT_EVERY_N_CYCLES > 0 and cycle % config.AGENT_EVERY_N_CYCLES == 0
        if summary.get("ok") and not periodic:
            return False
        ok, why = self.budget.can_call()
        if not ok:
            if why != "min interval":
                self.log("agent_skip", reason=why)
            return False
        image = side_by_side(before_jpeg, after_jpeg)
        if image is None:  # without a frame it just guesses from telemetry: not worth a call
            self.log("agent_skip", reason="no camera image")
            return False
        self.budget.record()  # count BEFORE sending: a failed call still uses quota
        self.pending = self.pool.submit(self._decide, cycle, image, summary)
        return True

    def poll(self):
        fb = self.fallback.poll()
        if self.pending is None or not self.pending.done():
            return fb
        fut, self.pending = self.pending, None
        try:
            return fut.result()
        except Exception as e:
            return e

    def check_lift(self, before_jpeg, after_jpeg) -> LiftVerdict | None:
        """Synchronous (the gripper waits holding the cap up, ~1-3 s). None = no answer
        (budget, no image, 429, error); the caller keeps twisting."""
        image = side_by_side(before_jpeg, after_jpeg)
        if image is None:
            self.log("agent_skip", reason="lift check: no camera image")
            return None
        if self.pending is not None and not self.pending.done():
            self.pending.result()  # don't overlap calls; a monitoring call is at most a few s
        ok, why = self.budget.can_call()
        if not ok and not why.startswith("min interval"):
            self.log("agent_skip", reason=f"lift check: {why}")
            return None
        if not ok:  # min interval: lift checks are rare, so wait instead of skipping
            time.sleep(max(0.0, config.GEMINI_MIN_INTERVAL_S - (time.time() - self.budget.last_call)))
        self.budget.record()
        types = self.genai.types
        try:
            resp = self.client.models.generate_content(
                model=self.model,
                contents=[types.Part.from_bytes(data=image, mime_type="image/jpeg"), "Did the cap come off?"],
                config=types.GenerateContentConfig(
                    system_instruction=LIFT_PROMPT,
                    response_mime_type="application/json",
                    response_schema=LiftVerdict,
                    temperature=0.0,
                    max_output_tokens=200,
                ),
            )
            self.budget.on_success()
        except self.genai.errors.APIError as e:
            msg = self.budget.on_429(e) if e.code == 429 else repr(e)[:200]
            self.log("agent_error", error=f"lift check: {msg}")
            return None
        except Exception as e:
            self.log("agent_error", error=f"lift check: {e!r}"[:300])
            return None
        v = resp.parsed if isinstance(resp.parsed, LiftVerdict) else LiftVerdict.model_validate_json(resp.text)
        v.source = self.model
        return v

    def decide_once(self, cycle: int, image: bytes | None, summary: dict) -> Decision:
        """Synchronous single call (used by replay). Still counts against the budget."""
        ok, why = self.budget.can_call()
        if not ok and why != "min interval":
            raise RuntimeError(f"budget: {why}")
        self.budget.record()
        return self._decide(cycle, image, summary)

    def _decide(self, cycle, image, summary) -> Decision:
        types = self.genai.types
        with self.lock:
            hist = "\n".join(self.history) or "(none yet)"
        parts: list = [f"Cycle {cycle} just finished.\nTelemetry: {json.dumps(summary)}\n"
                       f"Recent history (oldest first):\n{hist}"]
        if image:
            parts.append(types.Part.from_bytes(data=image, mime_type="image/jpeg"))
        else:
            parts.append("(no camera image available)")
        try:
            resp = self.client.models.generate_content(
                model=self.model,
                contents=parts,
                config=types.GenerateContentConfig(
                    system_instruction=SYSTEM_PROMPT,
                    response_mime_type="application/json",
                    response_schema=Decision,
                    temperature=0.2,
                    max_output_tokens=400,
                ),
            )
        except self.genai.errors.APIError as e:
            # new google-genai SDK: 429 arrives as ClientError(code=429), not ResourceExhausted
            if e.code == 429:
                raise RuntimeError(self.budget.on_429(e)) from e
            raise
        self.budget.on_success()
        d = resp.parsed if isinstance(resp.parsed, Decision) else Decision.model_validate_json(resp.text)
        d.twist_fraction = min(1.0, max(0.1, d.twist_fraction))
        d.source = self.model
        return d
