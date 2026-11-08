"""Gemini as a closed-loop agent: perceive -> pick ONE skill (function call) -> act -> observe -> ...

Every skill in skill_registry.py (except the scripted "demo" macros) is declared to Gemini as a
function, plus `finish`. Each turn Gemini sees:
  - the top-camera image (real webcam, or the twin's simulated top cam requested from the browser)
  - sensor text: squeeze currents (now + last 2 s), what each squeeze is holding, servo angles, and
    the tactile pad as numbers - features + an 8x16 digit grid (cheaper and more exact than an image
    of a heatmap: ~150 tokens vs ~260 for a small picture, and Gemini reads real values)
  - the result of the skill it called last
and must answer with exactly one function call (mode ANY) that includes a short `reason`.
The fast force control stays local at 25 Hz inside the skills; Gemini only sequences and checks.

Budget (free tier 15/min, 250/day): its own interval (AGENT_MIN_INTERVAL_S, default 5 s) and step cap
(AGENT_MAX_STEPS, default 8), plus the shared daily counter in runs/gemini_usage.json.
Every call is logged as a 'gemini_call' event (prompt/answer/latency/tokens/thumbnail) for the trace panel.
"""

import base64
import os
import time

import cv2 as cv
import numpy as np

import config
import skill_registry
from agent import Budget
from robot_serial import TACTILE_COLS, TACTILE_ROWS
from skills import Aborted

MAX_STEPS = int(os.getenv("AGENT_MAX_STEPS", "8"))
MIN_INTERVAL_S = float(os.getenv("AGENT_MIN_INTERVAL_S", "5"))

SYSTEM = """\
You control a small robot gripper (TEND, a Cartesian gripper with tactile and vision assistance) by calling ONE skill function per turn.
Hardware: two BOTTOM fingers squeeze the container body and hold it; two TOP fingers squeeze the cap
/ knob / soft body; the top finger tips slide in opposite directions along their length, which ROTATES
a gripped cap (no wrist joint); the whole top layer ("halves") can lift up/down.
Grip force is controlled locally from servo current (~150 mA = firm grip on target). A FlexiTac
16x32 tactile pad is on the inner face of the TOP RIGHT finger: you get its contact features and an
8x16 grid of pressure digits 0-9 (rows = height, cols = along the finger; 0 = no contact).
Slip shows up as the contact centroid moving or pressure dropping during a twist while the cap stripe
in the image does not turn.
Rules: use only the given functions; read the image + sensors before each call; give a short concrete
`reason` (what you see and why this step). Prefer small safe steps. If a skill was blocked, fix the
precondition it names. Call finish(success, summary) when the goal is done or impossible.
"""


# ---------------- observation ----------------

def tactile_text(states) -> dict:
    """Pad summary relative to its resting level (per-cell min over the recent window)."""
    frames = [np.frombuffer(s.tactile, np.uint8).astype(np.int16) for s in states if s.ok]
    if not frames:
        return {"ok": False}
    rest = np.min(np.stack(frames), axis=0)
    d = np.clip(frames[-1] - rest - 8, 0, None).reshape(TACTILE_ROWS, TACTILE_COLS)   # 8 = noise floor
    total = int(d.sum())
    out = {"ok": True, "touched_cells": int((d > 0).sum()), "total": total, "peak": int(d.max())}
    if total:
        rr, cc = np.nonzero(d)
        w = d[rr, cc]
        out["centroid_row_col"] = [round(float((rr * w).sum() / w.sum()), 1), round(float((cc * w).sum() / w.sum()), 1)]
        out["patch_rows"] = [int(rr.min()), int(rr.max())]
        out["patch_cols"] = [int(cc.min()), int(cc.max())]
    pooled = d.reshape(8, 2, 16, 2).max(axis=(1, 3))                                  # 16x32 -> 8x16 max-pool
    scale = max(int(pooled.max()), 40)
    out["grid_8x16"] = ["".join(str(min(9, int(round(v * 9 / scale)))) for v in row) for row in pooled]
    return out


def observe(sk) -> str:
    states = list(sk.recent)[-50:]
    st = states[-1] if states else sk.robot.state
    lines = []
    if st is not None:
        lines.append(f"servo angles (0-180, -1 = limp): {st.angles}")
        for name, ch in (("bottom squeeze i0", 0), ("top squeeze i1", 1)):
            vals = [s.current_ma[ch] for s in states if s.current_ma[ch] == s.current_ma[ch]]
            if vals:
                lines.append(f"{name}: now {vals[-1]:.0f} mA, last 2 s min {min(vals):.0f} / mean {sum(vals)/len(vals):.0f} / max {max(vals):.0f} mA")
    lines.append(f"holding: bottom={bool(sk.holding.get(config.BOTTOM_SQUEEZE.idx))} top={bool(sk.holding.get(config.TOP_SQUEEZE.idx))}")
    lines.append(f"grip target current: {config.GRIP_TARGET_MA[config.TOP_SQUEEZE.idx]:.0f} mA")
    t = tactile_text(states)
    if t.get("ok"):
        grid = t.pop("grid_8x16")
        lines.append("tactile (top right finger): " + ", ".join(f"{k}={v}" for k, v in t.items()))
        lines.append("tactile grid 8x16 (top row = top of pad):\n  " + "\n  ".join(grid))
    else:
        lines.append("tactile: no data")
    return "\n".join(lines)


def thumb_b64(jpeg: bytes | None, width: int = 240) -> str | None:
    if not jpeg:
        return None
    img = cv.imdecode(np.frombuffer(jpeg, np.uint8), cv.IMREAD_COLOR)
    h, w = img.shape[:2]
    img = cv.resize(img, (width, int(h * width / w)), interpolation=cv.INTER_AREA)
    ok, buf = cv.imencode(".jpg", img, [cv.IMWRITE_JPEG_QUALITY, 70])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode() if ok else None


# ---------------- function declarations ----------------

def declarations(types):
    decls = []
    for s in skill_registry.SKILLS:
        if s.group in ("demo", "manual"):   # scripted macros + manual keys: Gemini composes the primitives
            continue
        props = {"reason": {"type": "STRING", "description": "one sentence: what you observed and why this step"}}
        for p in s.params:
            props[p.name] = {"type": "NUMBER", "description": f"{p.desc} ({p.lo}-{p.hi}, default {p.default})"}
        decls.append(types.FunctionDeclaration(
            name=s.name, description=f"[{s.group}] {s.desc} Moves servos {s.servos}.",
            parameters={"type": "OBJECT", "properties": props, "required": ["reason"]}))
    decls.append(types.FunctionDeclaration(
        name="finish", description="End the task.",
        parameters={"type": "OBJECT", "properties": {
            "success": {"type": "BOOLEAN"},
            "summary": {"type": "STRING", "description": "what happened, citing visual and sensor evidence"}},
            "required": ["success", "summary"]}))
    return decls


# ---------------- loop ----------------

class GeminiAgent:
    def __init__(self, sk, get_frame, log, model: str = config.GEMINI_MODEL):
        """get_frame() -> JPEG bytes or None (webcam, or the twin's sim top cam via the browser)."""
        from google import genai
        from google.genai import types
        self.genai, self.types = genai, types
        self.client = genai.Client()
        self.sk, self.get_frame, self.log, self.model = sk, get_frame, log, model
        self.budget = Budget(model)

    def _wait_budget(self, should_stop):
        while True:
            if should_stop():
                raise Aborted()
            if self.budget.disabled_reason:
                raise RuntimeError(self.budget.disabled_reason)
            if self.budget.used_today() >= config.GEMINI_DAILY_CAP:
                raise RuntimeError(f"daily cap {config.GEMINI_DAILY_CAP} reached")
            wait = max(self.budget.cooldown_until - time.time(), MIN_INTERVAL_S - (time.time() - self.budget.last_call))
            if wait <= 0:
                return
            time.sleep(min(wait, 0.25))

    def _user_turn(self, text: str, jpeg: bytes | None, fn_name: str | None = None, fn_result: dict | None = None):
        t = self.types
        parts = []
        if fn_name:
            parts.append(t.Part.from_function_response(name=fn_name, response=fn_result or {}))
        parts.append(t.Part.from_text(text=text))
        if jpeg:
            parts.append(t.Part.from_bytes(data=jpeg, mime_type="image/jpeg"))
        return t.Content(role="user", parts=parts)

    def run(self, goal: str, should_stop, max_steps: int = MAX_STEPS) -> dict:
        t = self.types
        cfg = t.GenerateContentConfig(
            system_instruction=SYSTEM, temperature=0.2,
            tools=[t.Tool(function_declarations=declarations(t))],
            tool_config=t.ToolConfig(function_calling_config=t.FunctionCallingConfig(mode="ANY")),
            automatic_function_calling=t.AutomaticFunctionCallingConfig(disable=True))
        jpeg = self.get_frame()
        obs = observe(self.sk)
        contents = [self._user_turn(f"GOAL: {goal}\nMax {max_steps} steps.\n\nSENSORS NOW:\n{obs}"
                                    + ("" if jpeg else "\n(no camera image this turn)"), jpeg)]
        for step in range(1, max_steps + 1):
            self._wait_budget(should_stop)
            self.budget.record()                       # count before sending: failures use quota too
            t0 = time.time()
            try:
                resp = self.client.models.generate_content(model=self.model, contents=contents, config=cfg)
                self.budget.on_success()
            except Exception as e:
                if "429" in str(e) or "RESOURCE_EXHAUSTED" in str(e):
                    why = self.budget.on_429(e)
                    self.log("gemini_call", step=step, error=why)
                    if self.budget.disabled_reason:
                        return {"ok": False, "reason": why}
                    continue                           # cooled down by _wait_budget next round
                self.log("gemini_call", step=step, error=repr(e)[:300])
                return {"ok": False, "reason": f"Gemini error: {e!r}"[:300]}
            latency = round(time.time() - t0, 2)
            calls = resp.function_calls or []
            usage = resp.usage_metadata
            base = {"step": step, "model": self.model, "latency_s": latency,
                    "tokens_in": getattr(usage, "prompt_token_count", None),
                    "tokens_out": getattr(usage, "candidates_token_count", None),
                    "observation": obs, "image": thumb_b64(jpeg), "used_today": self.budget.used_today()}
            if not calls:
                self.log("gemini_call", **base, error="no function call", text=(resp.text or "")[:300])
                return {"ok": False, "reason": "Gemini answered without a function call"}
            fc = calls[0]
            args = dict(fc.args or {})
            reason = args.pop("reason", "")
            self.log("gemini_call", **base, function=fc.name, args=args, reason=reason)
            contents.append(resp.candidates[0].content)

            if fc.name == "finish":
                self.log("agent_finish", success=bool(args.get("success")), summary=args.get("summary", ""), steps=step)
                return {"ok": bool(args.get("success")), "summary": args.get("summary", ""), "steps": step}

            skill = skill_registry.BY_NAME.get(fc.name)
            if skill is None:
                result = {"ok": False, "error": f"unknown skill {fc.name}"}
            elif (why := skill_registry.blocked(skill, self.sk)):
                result = {"ok": False, "blocked": why}
            else:
                kw = {p.name: p.clamp(args.get(p.name, p.default)) for p in skill.params}
                ts = time.time()
                try:
                    result = skill.run(self.sk, **kw)
                except Aborted:
                    raise
                except Exception as e:
                    result = {"ok": False, "error": repr(e)[:200]}
                self.log("skill", name=fc.name, label=skill.label, params=kw,
                         duration_s=round(time.time() - ts, 2), result=result, by="gemini")
            jpeg = self.get_frame()
            obs = observe(self.sk)
            _drop_old_images(contents, self.types)     # only the newest frame is worth its ~260+ tokens
            last = "\nLAST CALL: you must call finish now (summarise what was achieved)." if step == max_steps - 1 else ""
            contents.append(self._user_turn(f"SENSORS AFTER {fc.name}:\n{obs}" + ("" if jpeg else "\n(no camera image)") + last,
                                            jpeg, fc.name, {"result": _jsonable(result)}))
        self.log("agent_finish", success=False, summary=f"step cap {max_steps} reached", steps=max_steps)
        return {"ok": False, "reason": f"step cap {max_steps} reached"}


def _drop_old_images(contents, t):
    for i, c in enumerate(contents):
        if c.role == "user" and any(getattr(p, "inline_data", None) for p in c.parts or []):
            parts = [p for p in c.parts if not getattr(p, "inline_data", None)]
            contents[i] = t.Content(role="user", parts=parts + [t.Part.from_text(text="(image from this step omitted)")])


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, float) and x != x:
        return None
    return x if isinstance(x, (str, int, float, bool)) or x is None else str(x)
