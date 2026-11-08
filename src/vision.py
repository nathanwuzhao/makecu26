"""Vision -> Gemini -> plan: one camera (or simulated top-camera) frame in, a short skill plan out.

    analysis, msg = analyze_scene(jpeg_bytes, skill_registry.catalog(), context="sim: pill_bottle")

Gemini sees the image plus the skill catalog (name, desc, params) and answers with a SceneAnalysis:
what object it is, cap state, grasp notes, and a <= 8-step plan that uses ONLY catalog names.
The plan is validated here (unknown names dropped, params clamped to the catalog ranges), so the
controller (Controller.run_plan) never sees anything it can't run. It never sends servo angles.

Same free-tier guard as agent.py: one shared Budget (daily cap, run cap, min interval, 429
cooldown). analyze_scene blocks for the call (~1-5 s) and is serialised by a lock.
"""

import threading
import time
from typing import Literal

from pydantic import BaseModel, Field

import config
from agent import Budget

OBJECTS = ("pill_bottle", "eye_dropper", "other", "none")
MAX_STEPS = 8

PROMPT = """\
You are the vision planner of a small robot gripper (TEND, a Cartesian gripper with tactile and vision assistance). You see ONE image of the
work area (a real webcam photo or a rendered digital-twin view looking down/forward at the fingers)
and plan what the gripper should do next. You never control servos directly: you pick named skills.

How the gripper works (2 layers of 2 fingers each, every axis is a servo-driven rack, ~24 mm travel):
- BOTTOM fingers squeeze the object's body and hold it still (servo 0 = bottom squeeze).
- TOP fingers sit on a layer that the "halves" servo lifts up/down (~24 mm). They squeeze the cap /
  dial / bottle body (servo 5 = top squeeze) and can slide in opposite directions, which TWISTS a
  gripped cap counter-clockwise seen from above (loosening). Release + recenter + repeat = ratchet.
- Lifting the halves while the top fingers hold a loosened cap pulls it off.
- Grip contact comes from the squeeze servos' current; a tactile pad sits on the top-right finger.

Objects it is meant for (all held vertical between the fingers):
- pill_bottle (pill bottle or pill tube): cap on top -> task "unscrew" (grip body, grip cap, twist cycles, lift).
- eye_dropper: held inverted, nozzle down -> task "dispense" (steady the shoulder, squeeze the body).
Use "other" for any other object and "none" if the work area is empty or you can't tell.

Skills you may use (use ONLY these exact names; params must be within [lo, hi]):
{catalog}

Rules:
- At most {max_steps} steps. Start with "home" unless the context says the gripper is already set up.
- A skill's preconditions matter (e.g. grip before twist, release + recenter between twists).
- "twist_cycle" grips the cap, twists, releases and recenters BY ITSELF: never put "grip_cap" right
  before it (twist_cycle refuses to start while the cap is gripped). Repeat it for more turns.
- Unscrew recipe: home, grip_bottle, twist_cycle (x3), grip_cap, lift_test. A cap needs ~8 twist
  cycles to come fully off, so when the context says an unscrew should run to completion, use
  home, demo_unscrew instead (it grips, cycles until the cap is off, then lifts it).
- Dispense recipe (eye_dropper): home, demo_eye_drop. Never twist or lift an eye dropper.
- If nothing graspable is visible, or it looks unsafe, return an empty plan and say why in "safety".
- Be concise: observations 1-2 sentences, reasoning 1-3 sentences.
"""


class PlanParam(BaseModel):          # Gemini's schema can't do free-form dicts: name/value pairs
    name: str
    value: float


class GeminiStep(BaseModel):
    name: str
    params: list[PlanParam] = Field(default_factory=list)
    why: str = ""


class GeminiAnalysis(BaseModel):
    object: Literal["pill_bottle", "eye_dropper", "other", "none"]
    confidence: float
    observations: str
    cap_state: Literal["on", "loose", "off", "n/a", "unknown"]
    grasp_notes: str
    recommended_task: Literal["unscrew", "dispense", "none"]
    plan: list[GeminiStep]
    reasoning: str
    safety: str


class PlanStep(BaseModel):
    name: str
    params: dict[str, float] = Field(default_factory=dict)
    why: str = ""


class SceneAnalysis(BaseModel):
    """What the page/controller gets: plan validated against the catalog, params as a dict."""
    object: Literal["pill_bottle", "eye_dropper", "other", "none"]
    confidence: float
    observations: str
    cap_state: Literal["on", "loose", "off", "n/a", "unknown"]
    grasp_notes: str
    recommended_task: Literal["unscrew", "dispense", "none"]
    plan: list[PlanStep]
    reasoning: str
    safety: str
    dropped: list[str] = Field(default_factory=list)   # steps removed by validation (shown in the log)
    model: str = ""
    latency_s: float = 0.0


def catalog_text(catalog: list[dict]) -> str:
    lines = []
    for s in catalog:
        ps = ", ".join(f"{p['name']} (default {p['default']}, {p['lo']}..{p['hi']}: {p.get('desc', '')})"
                       for p in s.get("params", []))
        lines.append(f"- {s['name']}: {s['desc']}" + (f" Params: {ps}." if ps else ""))
    return "\n".join(lines)


def validate_plan(steps, catalog: list[dict]) -> tuple[list[PlanStep], list[str]]:
    """Keep known skill names (max MAX_STEPS), clamp params to the catalog ranges, drop unknown params."""
    by_name = {s["name"]: s for s in catalog}
    out, dropped = [], []
    for st in steps:
        name = st.name if hasattr(st, "name") else str(st.get("name", ""))
        raw = st.params if hasattr(st, "params") else st.get("params", {})
        if isinstance(raw, list):
            raw = {p.name if hasattr(p, "name") else p.get("name"): p.value if hasattr(p, "value") else p.get("value")
                   for p in raw}
        skill = by_name.get(name)
        if skill is None:
            dropped.append(f"{name}: unknown skill")
            continue
        if len(out) >= MAX_STEPS:
            dropped.append(f"{name}: over {MAX_STEPS} steps")
            continue
        params = {}
        for p in skill.get("params", []):
            try:
                v = float(raw.get(p["name"], p["default"]))
            except (TypeError, ValueError):
                v = float(p["default"])
            params[p["name"]] = max(p["lo"], min(p["hi"], v))
        out.append(PlanStep(name=name, params=params, why=getattr(st, "why", "") or ""))
    return repair_plan(out, by_name), dropped


# What each skill leaves the cap grip / top slides as (None = unknown afterwards: stop repairing).
_AFTER = {"home": (False, True), "twist_cycle": (False, True), "twist": (True, False),
          "grip_cap": (True, None), "release_cap": (False, None), "open_top": (False, None),
          "recenter": (None, True)}


def repair_plan(plan: list[PlanStep], by_name: dict, cap_gripped: bool | None = None,
                recentered: bool | None = None) -> list[PlanStep]:
    """Insert the release_cap / recenter / grip_cap steps a plan forgot, so run_plan never stops on a
    precondition (Gemini likes 'grip_cap, twist_cycle', but twist_cycle grips by itself)."""
    out = []

    def add(name, why):
        if name in by_name:
            out.append(PlanStep(name=name, params={}, why=f"(inserted) {why}"))
            apply(name)

    def apply(name):
        nonlocal cap_gripped, recentered
        if name.startswith("demo_"):
            cap_gripped = recentered = None
            return
        g, r = _AFTER.get(name, (None, None))   # other skills (bottle, lift) leave both as they were
        if g is not None:
            cap_gripped = g
        if r is not None:
            recentered = r

    for st in plan:
        n = st.name
        if n == "grip_cap" and cap_gripped is True:
            continue                                   # already holding: grip_cap would be refused
        if n in ("twist_cycle", "recenter") and cap_gripped is True:
            add("release_cap", f"{n} needs the cap released")
        if n == "twist_cycle" and recentered is False:
            add("recenter", "twist_cycle starts from the recentered position")
        if n == "twist" and cap_gripped is False:
            add("grip_cap", "twist needs the cap gripped")
        out.append(st)
        apply(n)
    return out


# ---------- the call ----------

_lock = threading.Lock()
_client = None
_budget: Budget | None = None


def budget() -> Budget:
    global _budget
    if _budget is None:
        _budget = Budget(config.GEMINI_MODEL)
    return _budget


def budget_info() -> dict:
    b = budget()
    ok, why = b.can_call()
    wait = max(0.0, config.GEMINI_MIN_INTERVAL_S - (time.time() - b.last_call)) if b.last_call else 0.0
    return {"model": b.model, "used_today": b.used_today(), "daily_cap": config.GEMINI_DAILY_CAP,
            "run_calls": b.run_calls, "run_cap": config.GEMINI_RUN_CAP, "can_call": ok, "why": why,
            "next_in_s": round(max(wait, b.cooldown_until - time.time(), 0.0), 1)}


def analyze_scene(jpeg: bytes | None, catalog: list[dict], context: str = "",
                  dry: bool = False) -> tuple[SceneAnalysis | None, str]:
    """One Gemini call (if the budget allows). Returns (analysis, msg); analysis None on refusal/error.
    dry=True builds everything but doesn't call (msg = the prompt size)."""
    global _client
    if not jpeg:
        return None, "no image: Gemini is never called without one"
    system = PROMPT.format(catalog=catalog_text(catalog), max_steps=MAX_STEPS)
    user = "Analyse this scene and plan." + (f"\nContext: {context}" if context else "")
    if dry:
        return None, f"dry run: {len(jpeg)} B image, prompt {len(system) + len(user)} chars"
    with _lock:
        b = budget()
        ok, why = b.can_call()
        if not ok:
            return None, f"budget: {why}"
        from google import genai  # imported here so the rest of the app never needs the SDK
        types = genai.types
        if _client is None:
            _client = genai.Client()  # reads GEMINI_API_KEY / GOOGLE_API_KEY
        b.record()  # count BEFORE sending: a failed call still uses quota
        t0 = time.time()
        try:
            resp = _client.models.generate_content(
                model=b.model,
                contents=[types.Part.from_bytes(data=jpeg, mime_type="image/jpeg"), user],
                config=types.GenerateContentConfig(
                    system_instruction=system,
                    response_mime_type="application/json",
                    response_schema=GeminiAnalysis,
                    temperature=0.2,
                    max_output_tokens=1200,
                ),
            )
            b.on_success()
        except genai.errors.APIError as e:
            return None, (b.on_429(e) if e.code == 429 else f"Gemini error: {e!r}"[:300])
        except Exception as e:
            return None, f"Gemini error: {e!r}"[:300]
        try:
            g = resp.parsed if isinstance(resp.parsed, GeminiAnalysis) else GeminiAnalysis.model_validate_json(resp.text)
        except Exception as e:
            return None, f"unparseable answer: {e!r}"[:300]
    plan, dropped = validate_plan(g.plan, catalog)
    a = SceneAnalysis(**{**g.model_dump(exclude={"plan"}), "plan": plan, "dropped": dropped,
                         "model": b.model, "latency_s": round(time.time() - t0, 2)})
    a.confidence = max(0.0, min(1.0, a.confidence))
    return a, f"{a.object} ({a.confidence:.2f}), {len(plan)} step plan"
